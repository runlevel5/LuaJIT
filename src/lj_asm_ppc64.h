/*
** PPC IR assembler (SSA IR -> machine code).
** Copyright (C) 2005-2026 Mike Pall. See Copyright Notice in luajit.h
*/

/* -- Register allocator extensions --------------------------------------- */

/* Get the integer value of a constant operand, handling both KINT and the
** 64-bit KINT64/KPTR/KGC/KNULL constants (e.g. pointer-offset consts in p64
** arithmetic are KINT64, whose ->i field is 0 -- reading ->i would lose the
** value). Returns the full intptr_t value. */
static intptr_t get_kval(ASMState *as, IRRef ref)
{
  IRIns *ir = IR(ref);
  if (ir->o == IR_KINT64)
    return (intptr_t)ir_kint64(ir)->u64;
#if LJ_GC64
  else if (ir->o == IR_KGC)
    return (intptr_t)ir_kgc(ir);
  else if (ir->o == IR_KPTR || ir->o == IR_KKPTR)
    return (intptr_t)ir_kptr(ir);
#endif
  else
    return (intptr_t)ir->i;
}

/* Allocate a register with a hint. */
static Reg ra_hintalloc(ASMState *as, IRRef ref, Reg hint, RegSet allow)
{
  Reg r = IR(ref)->r;
  if (ra_noreg(r)) {
    if (!ra_hashint(r) && !iscrossref(as, ref))
      ra_sethint(IR(ref)->r, hint);  /* Propagate register hint. */
    r = ra_allocref(as, ref, allow);
  }
  ra_noweak(as, r);
  return r;
}

/* Allocate two source registers for three-operand instructions. */
static Reg ra_alloc2(ASMState *as, IRIns *ir, RegSet allow)
{
  IRIns *irl = IR(ir->op1), *irr = IR(ir->op2);
  Reg left = irl->r, right = irr->r;
  if (ra_hasreg(left)) {
    ra_noweak(as, left);
    if (ra_noreg(right))
      right = ra_allocref(as, ir->op2, rset_exclude(allow, left));
    else
      ra_noweak(as, right);
  } else if (ra_hasreg(right)) {
    ra_noweak(as, right);
    left = ra_allocref(as, ir->op1, rset_exclude(allow, right));
  } else if (ra_hashint(right)) {
    right = ra_allocref(as, ir->op2, allow);
    left = ra_alloc1(as, ir->op1, rset_exclude(allow, right));
  } else {
    left = ra_allocref(as, ir->op1, allow);
    right = ra_alloc1(as, ir->op2, rset_exclude(allow, left));
  }
  return left | (right << 8);
}

/* -- Guard handling ------------------------------------------------------ */

/* Setup exit stubs after the end of each trace. */
static void asm_exitstub_setup(ASMState *as, ExitNo nexits)
{
  ExitNo i;
  int ind;
  uintptr_t target = (uintptr_t)(void *)lj_vm_exit_handler;
  MCode *mxp = as->mctop;
  if (mxp - (nexits + 4 + MCLIM_REDZONE) < as->mclim)
    asm_mclimit(as);
  ind = ((target - (uintptr_t)(mxp - nexits - 2) + 0x02000000u) >> 26) ? 2 : 0;
  /* !ind: 1: mflr r0; bl ->vm_exit_handler; li r0, traceno;
  **  ind: 1: lwz r0, K32_VXH(jgl); mtctr r0; mflr r0; bctrl; li r0, traceno;
  **          bl <1; bl <1; ...
  */
  for (i = nexits-1; (int32_t)i >= 0; i--)
    *--mxp = PPCI_BL | (((-3-ind-i) & 0x00ffffffu) << 2);
  as->mcexit = mxp;
  *--mxp = PPCI_LI|PPCF_T(RID_TMP)|as->T->traceno;  /* Read by exit handler. */
  if (ind) {
    *--mxp = PPCI_BCTRL;
    *--mxp = PPCI_MFLR | PPCF_T(RID_TMP);
    *--mxp = PPCI_MTCTR | PPCF_T(RID_TMP);
    *--mxp = PPCI_LWZ | PPCF_T(RID_TMP) | PPCF_A(RID_JGL) |
	     jglofs(as, &as->J->k32[LJ_K32_VM_EXIT_HANDLER]);
  } else {
    mxp--;
    *mxp = PPCI_BL | ((target - (uintptr_t)mxp) & 0x03fffffcu);
    *--mxp = PPCI_MFLR | PPCF_T(RID_TMP);
  }
  as->mctop = mxp;
}

static MCode *asm_exitstub_addr(ASMState *as, ExitNo exitno)
{
  /* Keep this in-sync with exitstub_trace_addr(). */
  return as->mcexit + exitno;
}

/* Emit conditional branch to exit for guard. */
static void asm_guardcc(ASMState *as, PPCCC cc)
{
  MCode *target = asm_exitstub_addr(as, as->snapno);
  MCode *p = as->mcp;
  if (LJ_UNLIKELY(p == as->invmcp)) {
    as->loopinv = 1;
    *p = PPCI_B | (((target-p) & 0x00ffffffu) << 2);
    emit_condbranch(as, PPCI_BC, cc^4, p);
    return;
  }
  emit_condbranch(as, PPCI_BC, cc, target);
}

/* -- Operand fusion ------------------------------------------------------ */

/* Limit linear search to this distance. Avoids O(n^2) behavior. */
#define CONFLICT_SEARCH_LIM	31

/* Check if there's no conflicting instruction between curins and ref. */
static int noconflict(ASMState *as, IRRef ref, IROp conflict)
{
  IRIns *ir = as->ir;
  IRRef i = as->curins;
  if (i > ref + CONFLICT_SEARCH_LIM)
    return 0;  /* Give up, ref is too far away. */
  while (--i > ref)
    if (ir[i].o == conflict)
      return 0;  /* Conflict found. */
  return 1;  /* Ok, no conflict. */
}

/* Fuse the array base of colocated arrays. */
static int32_t asm_fuseabase(ASMState *as, IRRef ref)
{
  IRIns *ir = IR(ref);
  if (ir->o == IR_TNEW && ir->op1 <= LJ_MAX_COLOSIZE &&
      !neverfuse(as) && noconflict(as, ref, IR_NEWREF))
    return (int32_t)sizeof(GCtab);
  return 0;
}

/* Indicates load/store indexed is ok. */
#define AHUREF_LSX	((int32_t)0x80000000)

/* Fuse array/hash/upvalue reference into register+offset operand. */
static Reg asm_fuseahuref(ASMState *as, IRRef ref, int32_t *ofsp, RegSet allow)
{
  IRIns *ir = IR(ref);
  if (ra_noreg(ir->r)) {
    if (ir->o == IR_AREF) {
      if (mayfuse(as, ref)) {
	if (irref_isk(ir->op2)) {
	  IRRef tab = IR(ir->op1)->op1;
	  int32_t ofs = asm_fuseabase(as, tab);
	  IRRef refa = ofs ? tab : ir->op1;
	  ofs += 8*IR(ir->op2)->i;
	  if (checki16(ofs)) {
	    *ofsp = ofs;
	    return ra_alloc1(as, refa, allow);
	  }
	}
	if (*ofsp == AHUREF_LSX) {
	  Reg base = ra_alloc1(as, ir->op1, allow);
	  Reg idx = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, base));
	  return base | (idx << 8);
	}
      }
    } else if (ir->o == IR_HREFK) {
      if (mayfuse(as, ref)) {
	int32_t ofs = (int32_t)(IR(ir->op2)->op2 * sizeof(Node));
	if (checki16(ofs)) {
	  *ofsp = ofs;
	  return ra_alloc1(as, ir->op1, allow);
	}
      }
    } else if (ir->o == IR_UREFC) {
      if (irref_isk(ir->op1)) {
	GCfunc *fn = ir_kfunc(IR(ir->op1));
	/* GC64: the closed-upvalue tv is a FULL 64-bit address. The old i32ptr()
	** truncated it to int32 (sign-extending a pointer with bit 31 set) ->
	** ra_allock got a bogus 0xffffffff.. base -> a wild load that faulted
	** only when GC moved memory into that range. Use 64-bit arithmetic, like
	** emit_lsptr: JGL-relative if within reach, else materialize the 64-bit
	** base with a signed 16-bit displacement folded into the load. */
	uintptr_t p = (uintptr_t)&gcref(fn->l.uvptr[(ir->op2 >> 8)])->uv.tv;
	uintptr_t jgl = (uintptr_t)J2G(as->J);
	if ((uintptr_t)(p-jgl) < 65536) {
	  *ofsp = (int32_t)(p-jgl-32768);
	  return RID_JGL;
	} else {
	  int32_t i = (int16_t)(int32_t)p;
	  *ofsp = i;
	  return ra_allock(as, (intptr_t)(p - (uintptr_t)(intptr_t)i), allow);
	}
      }
    } else if (ir->o == IR_TMPREF) {
      *ofsp = (int32_t)(offsetof(global_State, tmptv)-32768);
      return RID_JGL;
    }
  }
  *ofsp = 0;
  return ra_alloc1(as, ref, allow);
}

/* Convert a D/DS-form load/store opcode to its X-form (indexed) equivalent.
** The (pi>>20)&0x780 bit-trick only works for the 32-bit D-form opcodes; the
** 64-bit DS-form LD/STD have a different primary opcode and must be mapped
** explicitly (otherwise STD would mis-encode as stfiwx). */
static PPCIns asm_loadstorex(PPCIns pi)
{
  if (pi == PPCI_LD) return PPCI_LDX;
  if (pi == PPCI_STD) return PPCI_STDX;
  return PPCI_LWZX | ((pi >> 20) & 0x780);
}

/* Fuse XLOAD/XSTORE reference into load/store operand. */
static void asm_fusexref(ASMState *as, PPCIns pi, Reg rt, IRRef ref,
			 RegSet allow, int32_t ofs)
{
  IRIns *ir = IR(ref);
  Reg base;
  if (ra_noreg(ir->r) && canfuse(as, ir)) {
    if (ir->o == IR_ADD) {
      int32_t ofs2;
      if (irref_isk(ir->op2) && checki32(ofs + get_kval(as, ir->op2)) &&
	  (ofs2 = ofs + (int32_t)get_kval(as, ir->op2), checki16(ofs2))) {
	ofs = ofs2;
	ref = ir->op1;
      } else if (ofs == 0) {
	Reg right, left = ra_alloc2(as, ir, allow);
	right = (left >> 8); left &= 255;
	emit_fab(as, asm_loadstorex(pi), rt, left, right);
	return;
      }
    } else if (ir->o == IR_STRREF) {
      /* STRREF op1 = string (base pointer), op2 = byte index. Only a constant
      ** INDEX (op2) folds into the displacement; a constant STRING (op1) is a
      ** pointer that must be materialized as the base (NOT used as an offset --
      ** the old irref_isk(op1) branch read IR(op1)->i, a garbage offset, and
      ** used the index as the base -> SIGSEGV for string.byte(s, var)). */
      lj_assertA(ofs == 0, "bad usage");
      ofs = (int32_t)sizeof(GCstr);
      if (irref_isk(ir->op2)) {
	ofs += IR(ir->op2)->i;
	ref = ir->op1;
      } else {
	/* base = string (op1, may be a const GCstr), index = op2. The index is a
	** 32-bit int whose register may have non-canonical high bits (e.g. zero-
	** extended from a prior op, while the int value is negative-then-clamped);
	** sign-extend it before the 64-bit base+index add, else a value like
	** len+(-1) computed as 3+0xffffffff = 0x100000002 -> wild OOB load. */
	Reg right, left = ra_alloc1(as, ir->op1, allow);
	Reg tmp = ra_scratch(as, rset_exclude(allow, left));
	right = ra_alloc1(as, ir->op2, rset_exclude(rset_exclude(allow, left), tmp));
	emit_fai(as, pi, rt, tmp, ofs);
	emit_tab(as, PPCI_ADD, tmp, left, tmp);
	emit_as(as, PPCI_EXTSW, tmp, right);  /* tmp = sign_extend32(index). */
	return;
      }
      if (!checki16(ofs)) {
	Reg left = ra_alloc1(as, ref, allow);
	Reg right = ra_allock(as, ofs, rset_exclude(allow, left));
	emit_fab(as, asm_loadstorex(pi), rt, left, right);
	return;
      }
    }
  }
  base = ra_alloc1(as, ref, allow);
  emit_fai(as, pi, rt, base, ofs);
}

/* Fuse XLOAD/XSTORE reference into indexed-only load/store operand. */
static void asm_fusexrefx(ASMState *as, PPCIns pi, Reg rt, IRRef ref,
			  RegSet allow)
{
  IRIns *ira = IR(ref);
  Reg right, left;
  if (canfuse(as, ira) && ira->o == IR_ADD && ra_noreg(ira->r)) {
    left = ra_alloc2(as, ira, allow);
    right = (left >> 8); left &= 255;
  } else {
    right = ra_alloc1(as, ref, allow);
    left = RID_R0;
  }
  emit_tab(as, pi, rt, left, right);
}

#if !LJ_SOFTFP
/* Fuse to multiply-add/sub instruction. */
static int asm_fusemadd(ASMState *as, IRIns *ir, PPCIns pi, PPCIns pir)
{
  IRRef lref = ir->op1, rref = ir->op2;
  IRIns *irm;
  if ((as->flags & JIT_F_OPT_FMA) &&
      lref != rref &&
      ((mayfuse(as, lref) && (irm = IR(lref), irm->o == IR_MUL) &&
	ra_noreg(irm->r)) ||
       (mayfuse(as, rref) && (irm = IR(rref), irm->o == IR_MUL) &&
	(rref = lref, pi = pir, ra_noreg(irm->r))))) {
    Reg dest = ra_dest(as, ir, RSET_FPR);
    Reg add = ra_alloc1(as, rref, RSET_FPR);
    Reg right, left = ra_alloc2(as, irm, rset_exclude(RSET_FPR, add));
    right = (left >> 8); left &= 255;
    emit_facb(as, pi, dest, left, right, add);
    return 1;
  }
  return 0;
}
#endif

/* -- Calls --------------------------------------------------------------- */

/* Generate a call to a C function. */
static void asm_gencall(ASMState *as, const CCallInfo *ci, IRRef *args)
{
  uint32_t n, nargs = CCI_XNARGS(ci);
  /* ELFv2 consecutive-doubleword ABI: the parameter save area at sp+32 has one
  ** 8-byte doubleword per scalar argument, and the GPR/FPR register files +
  ** param area advance in LOCKSTEP. The first 8 integer/pointer args go in
  ** r3..r10 (their param-area doublewords are register shadows we don't write);
  ** any further arg (9th+) is stored into its own doubleword. So `ofs` must
  ** track the per-arg doubleword position for EVERY arg, not only stack args --
  ** an overflow arg then lands at sp+32 + 8*arg_index (e.g. the 9th arg at
  ** sp+32+64 = sp+96), matching vm_ffi_call and the callee's expectations. */
  int32_t ofs = PPC_SPOFS_PARAM;
  Reg gpr = REGARG_FIRSTGPR;
#if !LJ_SOFTFP
  Reg fpr = REGARG_FIRSTFPR;
#endif
  /* Outgoing C-call stack args (the ELFv2 param save area beyond r3..r10 /
  ** f1..f13) are NYI for traces: emitting them correctly requires growing the
  ** trace's own frame for the param area AND rematerializing any constant args
  ** each loop iteration (a hoist into a callee-saved reg such as RID_BASE=r14
  ** corrupts BASE / yields stale args -> SIGSEGV). Bail to the interpreter,
  ** which marshals these correctly via vm_ffi_call (it grows its frame). */
  {
    Reg cg = REGARG_FIRSTGPR;
#if !LJ_SOFTFP
    Reg cf = REGARG_FIRSTFPR;
#endif
    for (n = 0; n < nargs; n++) {
      IRIns *ir = IR(args[n]);
#if !LJ_SOFTFP
      if (args[n] && irt_isfp(ir->t)) {
	if (cf > REGARG_LASTFPR) lj_trace_err(as->J, LJ_TRERR_NYICALL);
	cf++;
	if (cg <= REGARG_LASTGPR) cg++;
      } else
#endif
      {
	if (cg > REGARG_LASTGPR) lj_trace_err(as->J, LJ_TRERR_NYICALL);
	cg++;
      }
    }
  }
  if ((void *)ci->func)
    emit_call(as, (void *)ci->func);
  for (n = 0; n < nargs; n++) {  /* Setup args. */
    IRRef ref = args[n];
    if (ref) {
      IRIns *ir = IR(ref);
#if !LJ_SOFTFP
      if (irt_isfp(ir->t)) {
	if (fpr <= REGARG_LASTFPR) {
	  lj_assertA(rset_test(as->freeset, fpr),
		     "reg %d not free", fpr);  /* Already evicted. */
	  ra_leftov(as, fpr, ref);
	  fpr++;
	  if (gpr <= REGARG_LASTGPR) gpr++;  /* FP arg also consumes a GPR slot. */
	} else {
	  Reg r = ra_alloc1(as, ref, RSET_FPR);
	  emit_fai(as, irt_isnum(ir->t) ? PPCI_STFD : PPCI_STFS, r, RID_SP,
		   ofs + (irt_isnum(ir->t) || LJ_LE ? 0 : 4));
	}
      } else
#endif
      {
	if (gpr <= REGARG_LASTGPR) {
	  lj_assertA(rset_test(as->freeset, gpr),
		     "reg %d not free", gpr);  /* Already evicted. */
	  /* ELFv2: a sub-64-bit integer C-arg must be widened to the full 64-bit
	  ** register (sign-ext for signed/int, zero-ext for u32) -- a 32-bit int
	  ** register may carry non-canonical high bits the callee reads as a huge
	  ** value (e.g. size_t len for lj_str_new -> "string length overflow").
	  ** Widen FROM the source reg INTO the arg gpr (don't sign-ext in place:
	  ** the source may be a PHI/loop-carried value -- ra_leftov can rename it). */
	  /* ELFv2: widen a sub-64-bit integer arg in place after ra_leftov places
	  ** it in gpr (emitted reverse -> executes after). Skip PHI values:
	  ** ra_leftov may rename their reg to gpr, and sign-extending it in place
	  ** would corrupt the loop-carried value (PHI int C-args are rare). */
	  int widen = !irt_isphi(ir->t) &&
	    (irt_isinteger(ir->t) || irt_isu32(ir->t) || irt_isu16(ir->t) ||
	     irt_isu8(ir->t) || irt_isi8(ir->t) || irt_isi16(ir->t));
	  if (widen) {
	    if (irt_isu32(ir->t) || irt_isu16(ir->t) || irt_isu8(ir->t))
	      emit_rotdi(as, PPCI_RLDICL, gpr, gpr, 0, 32);  /* clrldi: zero-ext. */
	    else
	      emit_as(as, PPCI_EXTSW, gpr, gpr);  /* sign-extend 32->64. */
	  }
	  ra_leftov(as, gpr, ref);
	  gpr++;
	} else {
	  Reg r = ra_alloc1(as, ref, RSET_GPR);
	  emit_tai(as, irt_is64(ir->t) ? PPCI_STD : PPCI_STW, r, RID_SP,
		   ofs + (irt_is64(ir->t) || LJ_LE ? 0 : 4));
	}
      }
    } else {
      if (gpr <= REGARG_LASTGPR)
	gpr++;
    }
    ofs += 8;  /* Every scalar arg consumes one param-area doubleword. */
    checkmclim(as);
  }
}

/* Setup result reg/sp for call. Evict scratch regs. */
static void asm_setupresult(ASMState *as, IRIns *ir, const CCallInfo *ci)
{
  RegSet drop = RSET_SCRATCH;
  int hiop = ((ir+1)->o == IR_HIOP && !irt_isnil((ir+1)->t));
#if !LJ_SOFTFP
  if ((ci->flags & CCI_NOFPRCLOBBER))
    drop &= ~RSET_FPR;
#endif
  if (ra_hasreg(ir->r))
    rset_clear(drop, ir->r);  /* Dest reg handled below. */
  if (hiop && ra_hasreg((ir+1)->r))
    rset_clear(drop, (ir+1)->r);  /* Dest reg handled below. */
  ra_evictset(as, drop);  /* Evictions must be performed first. */
  if (ra_used(ir)) {
    lj_assertA(!irt_ispri(ir->t), "PRI dest");
    if (!LJ_SOFTFP && irt_isfp(ir->t)) {
      if ((ci->flags & CCI_CASTU64)) {
	/* GC64/ELFv2: the u64 result is returned in a single 64-bit GPR (RID_RET).
	** Bounce it through a stack slot to reinterpret as a double. */
	int32_t ofs = ir->s ? sps_scale(ir->s) : SPOFS_TMP;
	Reg dest = ir->r;
	if (ra_hasreg(dest)) {
	  ra_free(as, dest);
	  ra_modified(as, dest);
	  emit_fai(as, PPCI_LFD, dest, RID_SP, ofs);
	}
	emit_tai(as, PPCI_STD, RID_RET, RID_SP, ofs);
      } else {
	ra_destreg(as, ir, RID_FPRET);
      }
    } else if (hiop) {
      ra_destpair(as, ir);
    } else {
      ra_destreg(as, ir, RID_RET);
    }
  }
}

static void asm_callx(ASMState *as, IRIns *ir)
{
  IRRef args[CCI_NARGS_MAX*2];
  CCallInfo ci;
  IRRef func;
  IRIns *irf;
  ci.flags = asm_callx_flags(as, ir);
  asm_collectargs(as, ir, &ci, args);
  asm_setupresult(as, ir, &ci);
  func = ir->op2; irf = IR(func);
  if (irf->o == IR_CARG) { func = irf->op1; irf = IR(func); }
  if (irref_isk(func)) {  /* Call to constant address. */
    ci.func = (ASMFunction)(void *)get_kval(as, func);
  } else {  /* Indirect call: load target into r12 (ELFv2 global-entry reg). */
    /* ELFv2: the callee recomputes its own r2/TOC from r12 and does NOT restore
    ** the caller's r2 -- so we must save/restore our TOC (RID_SYS1 = r2) around
    ** the call ourselves, exactly like emit_call() does for constant targets. */
    RegSet allow = RSET_GPR & ~RSET_RANGE(RID_R0, REGARG_LASTGPR+1);
    Reg freg;
    emit_tai(as, PPCI_LD, RID_SYS1, RID_SP, PPC_TOC_SAVE_OFS);  /* restore r2. */
    *--as->mcp = PPCI_BCTRL;
    *--as->mcp = PPCI_MTCTR | PPCF_T(RID_R12);
    emit_tai(as, PPCI_STD, RID_SYS1, RID_SP, PPC_TOC_SAVE_OFS);  /* save r2. */
    /* Move the target into r12 (excluded from arg regs). */
    allow &= ~RID2RSET(RID_R12);
    freg = ra_alloc1(as, func, allow);
    emit_mr(as, RID_R12, freg);
    ci.func = (ASMFunction)(void *)0;
  }
  asm_gencall(as, &ci, args);
}

/* -- Returns ------------------------------------------------------------- */

/* Return to lower frame. Guard that it goes to the right spot. */
static void asm_retf(ASMState *as, IRIns *ir)
{
  Reg base = ra_alloc1(as, REF_BASE, RSET_GPR);
  void *pc = ir_kptr(IR(ir->op2));
  int32_t delta = 1+LJ_FR2+bc_a(*((const BCIns *)pc - 1));
  as->topslot -= (BCReg)delta;
  if ((int32_t)as->topslot < 0) as->topslot = 0;
  irt_setmark(IR(REF_BASE)->t);  /* Children must not coalesce with BASE reg. */
  emit_setgl(as, base, jit_base);
  emit_addptr(as, base, -8*delta);
  asm_guardcc(as, CC_NE);
  /* GC64: the saved frame PC at base-8 is a full 64-bit pointer. A 32-bit load
  ** + compare reads the high word on BE (vs. the low-word i32ptr(pc)) and
  ** spuriously fails the RETF guard -> endless side traces. Use a 64-bit
  ** load + compare against the full pointer (cf. arm64). */
  emit_ab(as, PPCI_CMPD, RID_TMP,
	  ra_allock(as, i64ptr(pc), rset_exclude(RSET_GPR, base)));
  emit_tai(as, PPCI_LD, RID_TMP, base, -8);
}

/* -- Buffer operations --------------------------------------------------- */

#if LJ_HASBUFFER
static void asm_bufhdr_write(ASMState *as, Reg sb)
{
  Reg tmp = ra_scratch(as, rset_exclude(RSET_GPR, sb));
  IRIns irgc;
  irgc.ot = IRT(0, IRT_PGC);  /* GC type. */
  emit_storeofs(as, &irgc, RID_TMP, sb, offsetof(SBuf, L));
  emit_rot(as, PPCI_RLWIMI, RID_TMP, tmp, 0, 31-lj_fls(SBUF_MASK_FLAG), 31);
  emit_getgl(as, RID_TMP, cur_L);
  emit_loadofs(as, &irgc, tmp, sb, offsetof(SBuf, L));
}
#endif

/* -- Type conversions ---------------------------------------------------- */

#if !LJ_SOFTFP
static void asm_tointg(ASMState *as, IRIns *ir, Reg left)
{
  RegSet allow = RSET_FPR;
  Reg tmp = ra_scratch(as, rset_clear(allow, left));
  Reg fbias = ra_scratch(as, rset_clear(allow, tmp));
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg hibias = ra_allock(as, 0x43300000, rset_exclude(RSET_GPR, dest));
  asm_guardcc(as, CC_NE);
  emit_fab(as, PPCI_FCMPU, 0, tmp, left);
  emit_fab(as, PPCI_FSUB, tmp, tmp, fbias);
  emit_fai(as, PPCI_LFD, tmp, RID_SP, SPOFS_TMP);
  emit_tai(as, PPCI_STW, RID_TMP, RID_SP, SPOFS_TMPLO);
  emit_tai(as, PPCI_STW, hibias, RID_SP, SPOFS_TMPHI);
  emit_asi(as, PPCI_XORIS, RID_TMP, dest, 0x8000);
  emit_tai(as, PPCI_LWZ, dest, RID_SP, SPOFS_TMPLO);
  emit_lsptr(as, PPCI_LFS, (fbias & 31),
	     (void *)&as->J->k32[LJ_K32_2P52_2P31], RSET_GPR);
  emit_fai(as, PPCI_STFD, tmp, RID_SP, SPOFS_TMP);
  emit_fb(as, PPCI_FCTIWZ, tmp, left);
}

static void asm_tobit(ASMState *as, IRIns *ir)
{
  RegSet allow = RSET_FPR;
  /* Allocate the FPR operands (incl. the bias KNUM, whose far const load may
  ** need a GPR base) BEFORE the GPR dest. Allocating dest first let the bias
  ** const-load's base GPR collide with dest -> dest clobbered before its use
  ** (a wild load). Mirrors arm64's ordering (dest last). */
  Reg left = ra_alloc1(as, ir->op1, allow);
  Reg right = ra_alloc1(as, ir->op2, rset_clear(allow, left));
  Reg tmp = ra_scratch(as, rset_clear(allow, right));
  Reg dest = ra_dest(as, ir, RSET_GPR);
  emit_tai(as, PPCI_LWZ, dest, RID_SP, SPOFS_TMPLO);
  emit_fai(as, PPCI_STFD, tmp, RID_SP, SPOFS_TMP);
  emit_fab(as, PPCI_FADD, tmp, left, right);
}
#endif

static void asm_conv(ASMState *as, IRIns *ir)
{
  IRType st = (IRType)(ir->op2 & IRCONV_SRCMASK);
  int st64 = (st == IRT_I64 || st == IRT_U64 || st == IRT_P64);
#if !LJ_SOFTFP
  int stfp = (st == IRT_NUM || st == IRT_FLOAT);
#endif
  IRRef lref = ir->op1;
#if LJ_SOFTFP
  /* FP conversions are handled by SPLIT. */
  lj_assertA(!irt_isfp(ir->t) && !(st == IRT_NUM || st == IRT_FLOAT),
	     "IR %04d has FP type",
	     (int)(ir - as->ir) - REF_BIAS);
  /* Can't check for same types: SPLIT uses CONV int.int + BXOR for sfp NEG. */
#else
  lj_assertA(irt_type(ir->t) != st, "inconsistent types for CONV");
  if (irt_isfp(ir->t)) {
    Reg dest = ra_dest(as, ir, RSET_FPR);
    if (stfp) {  /* FP to FP conversion. */
      if (st == IRT_NUM)  /* double -> float conversion. */
	emit_fb(as, PPCI_FRSP, dest, ra_alloc1(as, lref, RSET_FPR));
      else  /* float -> double conversion is a no-op on PPC. */
	ra_leftov(as, dest, lref);  /* Do nothing, but may need to move regs. */
    } else if (st == IRT_I64 || st == IRT_U64) {
      /* int64/uint64 -> FP conversion via FCFID (move to FPR, convert). */
      /* NYI: full unsigned correction for u64 >= 2^63 (rare in practice). */
      Reg left = ra_alloc1(as, lref, RSET_GPR);
      Reg tmp = ra_scratch(as, rset_exclude(RSET_FPR, dest));
      /* Emitted in reverse, so execution order is: STD; LFD; FCFID; [FRSP]. */
      if (irt_isfloat(ir->t)) emit_fb(as, PPCI_FRSP, dest, dest);
      emit_fb(as, PPCI_FCFID, dest, tmp);
      emit_fai(as, PPCI_LFD, tmp, RID_SP, SPOFS_TMP);
      emit_tai(as, PPCI_STD, left, RID_SP, SPOFS_TMP);
    } else {  /* Integer to FP conversion. */
      /* IRT_INT: Flip hibit, bias with 2^52, subtract 2^52+2^31. */
      /* IRT_U32: Bias with 2^52, subtract 2^52. */
      RegSet allow = RSET_GPR;
      Reg left = ra_alloc1(as, lref, allow);
      Reg hibias = ra_allock(as, 0x43300000, rset_clear(allow, left));
      Reg fbias = ra_scratch(as, rset_exclude(RSET_FPR, dest));
      if (irt_isfloat(ir->t)) emit_fb(as, PPCI_FRSP, dest, dest);
      emit_fab(as, PPCI_FSUB, dest, dest, fbias);
      emit_fai(as, PPCI_LFD, dest, RID_SP, SPOFS_TMP);
      emit_lsptr(as, PPCI_LFS, (fbias & 31),
		 &as->J->k32[st == IRT_U32 ? LJ_K32_2P52 : LJ_K32_2P52_2P31],
		 rset_clear(allow, hibias));
      emit_tai(as, PPCI_STW, st == IRT_U32 ? left : RID_TMP,
	       RID_SP, SPOFS_TMPLO);
      emit_tai(as, PPCI_STW, hibias, RID_SP, SPOFS_TMPHI);
      if (st != IRT_U32) emit_asi(as, PPCI_XORIS, RID_TMP, left, 0x8000);
    }
  } else if (stfp) {  /* FP to integer conversion. */
    if (irt_isguard(ir->t)) {
      /* Checked conversions are only supported from number to int. */
      lj_assertA(irt_isint(ir->t) && st == IRT_NUM,
		 "bad type for checked CONV");
      asm_tointg(as, ir, ra_alloc1(as, lref, RSET_FPR));
    } else if (irt_is64(ir->t)) {  /* FP -> int64/uint64 conversion (FCTIDZ). */
      /* Allocate the FPR source (its far KNUM load may need a GPR base) and the
      ** scratch BEFORE the GPR dest, so the const-load base can't collide with
      ** dest and clobber it. Same fix as asm_tobit; cf. arm64. */
      Reg left = ra_alloc1(as, lref, RSET_FPR);
      Reg tmp = ra_scratch(as, rset_exclude(RSET_FPR, left));
      Reg dest = ra_dest(as, ir, RSET_GPR);
      emit_tai(as, PPCI_LD, dest, RID_SP, SPOFS_TMP);
      emit_fai(as, PPCI_STFD, tmp, RID_SP, SPOFS_TMP);
      emit_fb(as, PPCI_FCTIDZ, tmp, left);
    } else {
      Reg left = ra_alloc1(as, lref, RSET_FPR);
      Reg tmp = ra_scratch(as, rset_exclude(RSET_FPR, left));
      Reg dest = ra_dest(as, ir, RSET_GPR);
      lj_assertA(!irt_isu32(ir->t), "bad CONV u32.fp emitted");
      emit_tai(as, PPCI_LWZ, dest, RID_SP, SPOFS_TMPLO);
      emit_fai(as, PPCI_STFD, tmp, RID_SP, SPOFS_TMP);
      emit_fb(as, PPCI_FCTIWZ, tmp, left);
    }
  } else
#endif
  {
    Reg dest = ra_dest(as, ir, RSET_GPR);
    if (st >= IRT_I8 && st <= IRT_U16) {  /* Extend to 32 bit integer. */
      Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
      lj_assertA(irt_isint(ir->t) || irt_isu32(ir->t), "bad type for CONV EXT");
      if ((ir->op2 & IRCONV_SEXT))
	emit_as(as, st == IRT_I8 ? PPCI_EXTSB : PPCI_EXTSH, dest, left);
      else
	emit_rot(as, PPCI_RLWINM, dest, left, 0, st == IRT_U8 ? 24 : 16, 31);
    } else if (irt_is64(ir->t)) {  /* Conversion to 64 bit integer. */
      if (st64 || !(ir->op2 & IRCONV_SEXT)) {
	/* 64/64 bit no-op (cast) or 32 to 64 bit zero extension. */
	Reg left = ra_alloc1(as, lref, RSET_GPR);
	if (st64) {
	  ra_leftov(as, dest, lref);  /* Cast: may need to move regs. */
	} else {  /* 32 to 64 bit zero extension: clear the upper 32 bits. */
	  emit_rotdi(as, PPCI_RLDICL, dest, left, 0, 32);  /* clrldi rd,rs,32 */
	}
      } else {  /* 32 to 64 bit sign extension. */
	Reg left = ra_alloc1(as, lref, RSET_GPR);
	emit_as(as, PPCI_EXTSW, dest, left);
      }
    } else {  /* Conversion to 32 bit integer. */
      if (st64) {
	/* Truncate a 64 bit integer to 32 bits (keep low word). */
	Reg left = ra_alloc1(as, lref, RSET_GPR);
	emit_rotdi(as, PPCI_RLDICL, dest, left, 0, 32);  /* clrldi rd,rs,32 */
      } else {  /* 32/32 bit no-op (cast). */
	ra_leftov(as, dest, lref);  /* Do nothing, but may need to move regs. */
      }
    }
  }
}

static void asm_strto(ASMState *as, IRIns *ir)
{
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_lj_strscan_num];
  IRRef args[2];
  int32_t ofs = SPOFS_TMP;
#if LJ_SOFTFP
  ra_evictset(as, RSET_SCRATCH);
  if (ra_used(ir)) {
    if (ra_hasspill(ir->s) && ra_hasspill((ir+1)->s) &&
	(ir->s & 1) == LJ_BE && (ir->s ^ 1) == (ir+1)->s) {
      int i;
      for (i = 0; i < 2; i++) {
	Reg r = (ir+i)->r;
	if (ra_hasreg(r)) {
	  ra_free(as, r);
	  ra_modified(as, r);
	  emit_spload(as, ir+i, r, sps_scale((ir+i)->s));
	}
      }
      ofs = sps_scale(ir->s & ~1);
    } else {
      Reg rhi = ra_dest(as, ir+1, RSET_GPR);
      Reg rlo = ra_dest(as, ir, rset_exclude(RSET_GPR, rhi));
      emit_tai(as, PPCI_LWZ, rhi, RID_SP, ofs);
      emit_tai(as, PPCI_LWZ, rlo, RID_SP, ofs+4);
    }
  }
#else
  RegSet drop = RSET_SCRATCH;
  if (ra_hasreg(ir->r)) rset_set(drop, ir->r);  /* Spill dest reg (if any). */
  ra_evictset(as, drop);
  if (ir->s) ofs = sps_scale(ir->s);
#endif
  asm_guardcc(as, CC_EQ);
  emit_ai(as, PPCI_CMPWI, RID_RET, 0);  /* Test return status. */
  args[0] = ir->op1;      /* GCstr *str */
  args[1] = ASMREF_TMP1;  /* TValue *n  */
  asm_gencall(as, ci, args);
  /* Store the result to the spill slot or temp slots. */
  emit_tai(as, PPCI_ADDI, ra_releasetmp(as, ASMREF_TMP1), RID_SP, ofs);
}

/* -- Memory references --------------------------------------------------- */

/* Get pointer to TValue. */
static void asm_tvstore64(ASMState *as, Reg base, int32_t ofs, IRRef ref);

static void asm_tvptr(ASMState *as, Reg dest, IRRef ref, MSize mode)
{
  int32_t tmpofs = (int32_t)(offsetof(global_State, tmptv)-32768);
  if ((mode & IRTMPREF_IN1)) {
    IRIns *ir = IR(ref);
    if (irt_isnum(ir->t)) {
      if ((mode & IRTMPREF_OUT1)) {
#if LJ_SOFTFP
	lj_assertA(irref_isk(ref), "unsplit FP op");
	emit_tai(as, PPCI_ADDI, dest, RID_JGL, tmpofs);
	emit_setgl_u32(as,
		   ra_allock(as, (int32_t)ir_knum(ir)->u32.lo, RSET_GPR),
		   tmptv.u32.lo);
	emit_setgl_u32(as,
		   ra_allock(as, (int32_t)ir_knum(ir)->u32.hi, RSET_GPR),
		   tmptv.u32.hi);
#else
	Reg src = ra_alloc1(as, ref, RSET_FPR);
	emit_tai(as, PPCI_ADDI, dest, RID_JGL, tmpofs);
	emit_fai(as, PPCI_STFD, src, RID_JGL, tmpofs);
#endif
      } else if (irref_isk(ref)) {
	/* Use the number constant itself as a TValue. GC64: its address is a full
	** 64-bit pointer -- i32ptr() would truncate it (same bug class as the
	** UREFC/STRREF fixes). */
	ra_allockreg(as, i64ptr(ir_knum(ir)), dest);
      } else {
#if LJ_SOFTFP
	lj_assertA(0, "unsplit FP op");
#else
	/* Otherwise force a spill and use the spill slot. */
	emit_tai(as, PPCI_ADDI, dest, RID_SP, ra_spill(as, ir));
#endif
      }
    } else {
      /* GC64: store a single 64-bit tagged TValue into g->tmptv. */
      emit_tai(as, PPCI_ADDI, dest, RID_JGL, tmpofs);
      asm_tvstore64(as, RID_JGL, tmpofs, ref);
    }
  } else {
    emit_tai(as, PPCI_ADDI, dest, RID_JGL, tmpofs);
  }
}

static void asm_aref(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg idx, base;
  if (irref_isk(ir->op2)) {
    IRRef tab = IR(ir->op1)->op1;
    int32_t ofs = asm_fuseabase(as, tab);
    IRRef refa = ofs ? tab : ir->op1;
    ofs += 8*IR(ir->op2)->i;
    if (checki16(ofs)) {
      base = ra_alloc1(as, refa, RSET_GPR);
      emit_tai(as, PPCI_ADDI, dest, base, ofs);
      return;
    }
  }
  base = ra_alloc1(as, ir->op1, RSET_GPR);
  idx = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, base));
  emit_tab(as, PPCI_ADD, dest, RID_TMP, base);
  emit_slwi(as, RID_TMP, idx, 3);
}

/* Inlined hash lookup. Specialized for key type and for const keys.
** The equivalent C code is:
**   Node *n = hashkey(t, key);
**   do {
**     if (lj_obj_equal(&n->key, key)) return &n->val;
**   } while ((n = nextnode(n)));
**   return niltv(L);
*/
static void asm_href(ASMState *as, IRIns *ir, IROp merge)
{
  RegSet allow = RSET_GPR;
  int destused = ra_used(ir);
  Reg dest = ra_dest(as, ir, allow);
  Reg tab = ra_alloc1(as, ir->op1, rset_clear(allow, dest));
  Reg tmp1 = RID_TMP, tmp2, type = RID_NONE, key = RID_NONE, tkey;
  IRRef refkey = ir->op2;
  IRIns *irkey = IR(refkey);
  int isk = irref_isk(refkey);
  IRType1 kt = irkey->t;
  uint32_t khash;
  MCLabel l_end, l_loop;

  rset_clear(allow, tab);
  tmp2 = ra_scratch(as, allow);
  rset_clear(allow, tmp2);

  /* GC64: allocate/build the full 64-bit tagged key (tkey) outside the loop. */
  if (isk) {
    int64_t kk;
    if (irt_isaddr(kt)) {
      kk = ((int64_t)irt_toitype(kt) << 47) | (int64_t)ir_kgc(irkey);
    } else if (irt_isnum(kt)) {
      kk = (int64_t)ir_knum(irkey)->u64;  /* -0.0 already canonicalized. */
    } else {
      lj_assertA(irt_ispri(kt) && !irt_isnil(kt), "bad HREF key type");
      kk = ~((int64_t)~irt_toitype(kt) << 47);
    }
    tkey = ra_allock(as, (intptr_t)kk, allow);
    rset_clear(allow, tkey);
  } else {
    tkey = ra_scratch(as, allow);
    rset_clear(allow, tkey);
  }

  /* Key not found in chain: jump to exit (if merged) or load niltv. */
  l_end = emit_label(as);
  as->invmcp = NULL;
  if (merge == IR_NE)
    asm_guardcc(as, CC_EQ);
  else if (destused)
    emit_loada(as, dest, niltvg(J2G(as->J)));

  /* Follow hash chain until the end. */
  l_loop = --as->mcp;
  emit_ai(as, PPCI_CMPDI, dest, 0);
  emit_tai(as, PPCI_LD, dest, dest, (int32_t)offsetof(Node, next));

  /* Type and value comparison: one 64-bit compare of the whole Node.key. */
  if (merge == IR_EQ)
    asm_guardcc(as, CC_EQ);
  else
    emit_condbranch(as, PPCI_BC|PPCF_Y, CC_EQ, l_end);
  emit_ab(as, PPCI_CMPD, tmp1, tkey);
  emit_tai(as, PPCI_LD, tmp1, dest, (int32_t)offsetof(Node, key));
  *l_loop = PPCI_BC | PPCF_Y | PPCF_CC(CC_NE) |
	    (((char *)as->mcp-(char *)l_loop) & 0xffffu);

  /* Construct tkey as a canonicalized or tagged key for non-const keys. */
  if (!isk) {
    if (irt_isnum(kt)) {
      key = ra_alloc1(as, refkey, RSET_FPR);
      /* -0.0 canonicalized to +0.0: if the (already loaded) tkey == 0x8000..0
      ** (negative zero) select the literal 0 instead (isel rT,0,rC on cr0.EQ;
      ** rA=0 means the XO add-form literal zero). This must run AFTER the hash
      ** (which reads tkey) but BEFORE the chain compare, mirroring arm64's CSEL.
      ** The STFD+LD that fills tkey is emitted inside the hash block below so it
      ** executes before the hash -- emitting it here would place it after the
      ** hash (emit prepends), so the hash would read a stale tkey. */
      Reg neg0 = ra_allock(as, (intptr_t)U64x(80000000,00000000), allow);
      emit_tab(as, PPCI_ISEL | PPCF_MB(CC_EQ&3), tkey, 0, tkey);
      emit_ab(as, PPCI_CMPD, tkey, neg0);
    } else {
      lj_assertA(irt_isaddr(kt), "bad HREF key type");
      key = ra_alloc1(as, refkey, allow);
      rset_clear(allow, key);
      type = ra_allock(as, (int64_t)irt_toitype(kt) << 47, allow);
      emit_tab(as, PPCI_ADD, tkey, key, type);  /* tkey = (itype<<47) | ptr. */
    }
  }

  /* Load main position relative to tab->node into dest. */
  khash = isk ? ir_khash(as, irkey) : 1;
  if (khash == 0) {
    emit_tai(as, PPCI_LD, dest, tab, (int32_t)offsetof(GCtab, node));
  } else {
    Reg tmphash = tmp1;
    if (isk)
      tmphash = ra_allock(as, khash, allow);
    emit_tab(as, PPCI_ADD, dest, dest, tmp1);
    emit_tai(as, PPCI_MULLI, tmp1, tmp1, sizeof(Node));
    emit_asb(as, PPCI_AND, tmp1, tmp2, tmphash);
    emit_tai(as, PPCI_LD, dest, tab, (int32_t)offsetof(GCtab, node));
    emit_tai(as, PPCI_LWZ, tmp2, tab, (int32_t)offsetof(GCtab, hmask));
    if (isk) {
      /* Nothing to do. */
    } else if (irt_isstr(kt)) {
      emit_tai(as, PPCI_LWZ, tmp1, key, (int32_t)offsetof(GCstr, sid));
    } else {  /* Must match with hash*() in lj_tab.c. */
      emit_tab(as, PPCI_SUBF, tmp1, tmp2, tmp1);
      emit_rotlwi(as, tmp2, tmp2, HASH_ROT3);
      emit_asb(as, PPCI_XOR, tmp1, tmp1, tmp2);
      emit_rotlwi(as, tmp1, tmp1, (HASH_ROT2+HASH_ROT1)&31);
      emit_tab(as, PPCI_SUBF, tmp2, dest, tmp2);
      if (irt_isnum(kt)) {
	/* hashnum(t,o) = hashlohi(o->u32.lo, o->u32.hi<<1): the hashrot "hi" arg
	** (tmp1, the one that gets doubled at ADD and feeds lo^=hi) must be the
	** HIGH word of the double, and the "lo" arg (tmp2) the LOW word. The old
	** code had them swapped (tmp1=lo32, tmp2=hi32) -> doubled the wrong word
	** -> wrong bucket -> num-keyed lookups (e.g. hufcodes[36]) missed. */
	emit_asb(as, PPCI_XOR, tmp2, tmp2, tmp1);
	emit_rotlwi(as, dest, tmp1, HASH_ROT1);
	emit_tab(as, PPCI_ADD, tmp1, tmp1, tmp1);  /* tmp1 = hi32 << 1. */
	emit_rotdi(as, PPCI_RLDICL, tmp2, tkey, 0, 32);   /* lo32 of tkey. */
	emit_rotdi(as, PPCI_RLDICL, tmp1, tkey, 32, 32);  /* hi32 of tkey. */
	/* Fill tkey from the double's bit pattern (bounced via the stack) HERE so
	** it executes before the hash above reads it. The -0.0 canonicalization
	** (in the construct block) then runs after the hash, before the compare. */
	emit_tai(as, PPCI_LD, tkey, RID_SP, SPOFS_TMP);
	emit_fai(as, PPCI_STFD, key, RID_SP, SPOFS_TMP);
      } else {
	/* GC64: hashgcref(t, key->gcr) hashes the FULL tagged key value (gcr holds
	** (itype<<47)|ptr). So lo = ptr&0xffffffff, hi = (itype<<15)|(ptr>>32).
	** Compute from `key` (the masked pointer, available early -- using tkey
	** here would read it stale, as tkey is built after this hash). The first
	** hashrot step (lo^=hi) is folded into the XOR, mirroring the num path. The
	** pre-GC64 lo=key,hi=key+HASH_BIAS formula sent some GC keys to the wrong
	** bucket -> present keys not found in the chain. */
	Reg ithi = ra_allock(as, (int32_t)(irt_toitype(kt) << 15), allow);
	emit_asb(as, PPCI_XOR, tmp2, tmp2, tmp1);     /* tmp2 = lo ^ hi. */
	emit_rotlwi(as, dest, tmp1, HASH_ROT1);       /* dest = rol(hi, R1). */
	emit_tab(as, PPCI_ADD, tmp1, tmp1, ithi);     /* hi = (ptr>>32)|itype<<15. */
	emit_rotdi(as, PPCI_RLDICL, tmp1, key, 32, 32); /* tmp1 = ptr>>32. */
	emit_rotdi(as, PPCI_RLDICL, tmp2, key, 0, 32);  /* tmp2 = lo = ptr&0xffffffff. */
      }
    }
  }
}

static void asm_hrefk(ASMState *as, IRIns *ir)
{
  IRIns *kslot = IR(ir->op2);
  IRIns *irkey = IR(kslot->op1);
  int32_t ofs = (int32_t)(kslot->op2 * sizeof(Node));
  int32_t kofs = ofs + (int32_t)offsetof(Node, key);
  int bigofs = (ofs > 32736);
  Reg dest = (ra_used(ir)||bigofs) ? ra_dest(as, ir, RSET_GPR) : RID_NONE;
  Reg node = ra_alloc1(as, ir->op1, RSET_GPR);
  Reg idx = node;
  RegSet allow = rset_exclude(RSET_GPR, node);
  uint64_t k;
  lj_assertA(ofs % sizeof(Node) == 0, "unaligned HREFK slot");
  if (bigofs) {
    idx = dest;
    rset_clear(allow, dest);
    kofs = (int32_t)offsetof(Node, key);
  } else if (ra_hasreg(dest)) {
    emit_tai(as, PPCI_ADDI, dest, node, ofs);
  }
  asm_guardcc(as, CC_NE);
  /* GC64: build the full 64-bit tagged key and compare the whole Node.key. */
  if (irt_ispri(irkey->t)) {
    k = ~((uint64_t)~irt_toitype(irkey->t) << 47);
  } else if (irt_isnum(irkey->t)) {
    k = ir_knum(irkey)->u64;
  } else {
    k = ((uint64_t)irt_toitype(irkey->t) << 47) | (uint64_t)ir_kgc(irkey);
  }
  emit_ab(as, PPCI_CMPD, RID_TMP, ra_allock(as, (intptr_t)k, allow));
  emit_tai(as, PPCI_LD, RID_TMP, idx, kofs);
  if (bigofs) {
    emit_tai(as, PPCI_ADDIS, dest, dest, (ofs + 32768) >> 16);
    emit_tai(as, PPCI_ADDI, dest, node, ofs);
  }
}

static void asm_uref(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  int guarded = (irt_t(ir->t) & (IRT_GUARD|IRT_TYPE)) == (IRT_GUARD|IRT_PGC);
  /* GC64: upvalue pointers (GCfuncL.uvptr[] entries, GCupval.v) are 64-bit
  ** GCRef/MRef -- load with LD and index the uvptr array by 8, not 4. */
  if (irref_isk(ir->op1) && !guarded) {
    GCfunc *fn = ir_kfunc(IR(ir->op1));
    MRef *v = &gcref(fn->l.uvptr[(ir->op2 >> 8)])->uv.v;
    emit_lsptr(as, PPCI_LD, dest, v, RSET_GPR);
  } else {
    if (guarded) {
      asm_guardcc(as, ir->o == IR_UREFC ? CC_NE : CC_EQ);
      emit_ai(as, PPCI_CMPWI, RID_TMP, 1);
    }
    if (ir->o == IR_UREFC)
      emit_tai(as, PPCI_ADDI, dest, dest, (int32_t)offsetof(GCupval, tv));
    else
      emit_tai(as, PPCI_LD, dest, dest, (int32_t)offsetof(GCupval, v));
    if (guarded)
      emit_tai(as, PPCI_LBZ, RID_TMP, dest, (int32_t)offsetof(GCupval, closed));
    if (irref_isk(ir->op1)) {
      GCfunc *fn = ir_kfunc(IR(ir->op1));
      emit_loadu64(as, dest, gcrefu(fn->l.uvptr[(ir->op2 >> 8)]));
    } else {
      emit_tai(as, PPCI_LD, dest, ra_alloc1(as, ir->op1, RSET_GPR),
	       (int32_t)offsetof(GCfuncL, uvptr) + 8*(int32_t)(ir->op2 >> 8));
    }
  }
}

static void asm_fref(ASMState *as, IRIns *ir)
{
  UNUSED(as); UNUSED(ir);
  lj_assertA(!ra_used(ir), "unfused FREF");
}

static void asm_strref(ASMState *as, IRIns *ir)
{
  /* STRREF: op1 = string (base), op2 = byte index. The result is the address of
  ** the string's data at that index: strdata = (char *)str + sizeof(GCstr) +
  ** index. op1 may be a constant GCstr -- ra_alloc1 rematerializes its pointer;
  ** it is NOT a small-int offset (the old swap-and-use-IR(refk)->i path treated
  ** a constant string operand as an offset -> garbage base -> SIGSEGV when the
  ** index was variable, e.g. string.byte(s, var)). Mirrors lj_asm_arm64.h. */
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg base = ra_alloc1(as, ir->op1, RSET_GPR);
  IRIns *irr = IR(ir->op2);
  int32_t ofs = (int32_t)sizeof(GCstr);
  if (irref_isk(ir->op2) && checki16(ofs + irr->i)) {
    emit_tai(as, PPCI_ADDI, dest, base, ofs + irr->i);
  } else {
    /* Sign-extend the 32-bit int index before the 64-bit base+index add: its
    ** register may carry non-canonical high bits (see asm_fusexref STRREF). */
    Reg idx = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, base));
    Reg tmp = ra_scratch(as, rset_exclude(rset_exclude(RSET_GPR, base), idx));
    emit_tai(as, PPCI_ADDI, dest, dest, ofs);
    emit_tab(as, PPCI_ADD, dest, base, tmp);
    emit_as(as, PPCI_EXTSW, tmp, idx);  /* tmp = sign_extend32(index). */
  }
}

/* -- Loads and stores ---------------------------------------------------- */

static PPCIns asm_fxloadins(ASMState *as, IRIns *ir)
{
  UNUSED(as);
  switch (irt_type(ir->t)) {
  case IRT_I8: return PPCI_LBZ;  /* Needs sign-extension. */
  case IRT_U8: return PPCI_LBZ;
  case IRT_I16: return PPCI_LHA;
  case IRT_U16: return PPCI_LHZ;
  case IRT_NUM: lj_assertA(!LJ_SOFTFP, "unsplit FP op"); return PPCI_LFD;
  case IRT_FLOAT: if (!LJ_SOFTFP) return PPCI_LFS;
  default: return irt_is64(ir->t) ? PPCI_LD : PPCI_LWZ;
  }
}

static PPCIns asm_fxstoreins(ASMState *as, IRIns *ir)
{
  UNUSED(as);
  switch (irt_type(ir->t)) {
  case IRT_I8: case IRT_U8: return PPCI_STB;
  case IRT_I16: case IRT_U16: return PPCI_STH;
  case IRT_NUM: lj_assertA(!LJ_SOFTFP, "unsplit FP op"); return PPCI_STFD;
  case IRT_FLOAT: if (!LJ_SOFTFP) return PPCI_STFS;
  default: return irt_is64(ir->t) ? PPCI_STD : PPCI_STW;
  }
}

static void asm_fload(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  PPCIns pi = asm_fxloadins(as, ir);
  Reg idx;
  int32_t ofs;
  if (ir->op1 == REF_NIL) {  /* FLOAD from GG_State with offset. */
    idx = RID_JGL;
    ofs = (ir->op2 << 2) - 32768 - GG_OFS(g);
  } else {
    idx = ra_alloc1(as, ir->op1, RSET_GPR);
    if (ir->op2 == IRFL_TAB_ARRAY) {
      ofs = asm_fuseabase(as, ir->op1);
      if (ofs) {  /* Turn the t->array load into an add for colocated arrays. */
	emit_tai(as, PPCI_ADDI, dest, idx, ofs);
	return;
      }
    }
    ofs = field_ofs[ir->op2];
  }
  lj_assertA(!irt_isi8(ir->t), "unsupported FLOAD I8");
  emit_tai(as, pi, dest, idx, ofs);
}

static void asm_fstore(ASMState *as, IRIns *ir)
{
  if (ir->r != RID_SINK) {
    Reg src = ra_alloc1(as, ir->op2, RSET_GPR);
    IRIns *irf = IR(ir->op1);
    Reg idx = ra_alloc1(as, irf->op1, rset_exclude(RSET_GPR, src));
    int32_t ofs = field_ofs[irf->op2];
    PPCIns pi = asm_fxstoreins(as, ir);
    emit_tai(as, pi, src, idx, ofs);
  }
}

static void asm_xload(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir,
    (!LJ_SOFTFP && irt_isfp(ir->t)) ? RSET_FPR : RSET_GPR);
  lj_assertA(!(ir->op2 & IRXLOAD_UNALIGNED), "unaligned XLOAD");
  if (irt_isi8(ir->t))
    emit_as(as, PPCI_EXTSB, dest, dest);
  asm_fusexref(as, asm_fxloadins(as, ir), dest, ir->op1, RSET_GPR, 0);
}

static void asm_xstore_(ASMState *as, IRIns *ir, int32_t ofs)
{
  IRIns *irb;
  if (ir->r == RID_SINK)
    return;
  if (ofs == 0 && mayfuse(as, ir->op2) && (irb = IR(ir->op2))->o == IR_BSWAP &&
      ra_noreg(irb->r) && (irt_isint(ir->t) || irt_isu32(ir->t))) {
    /* Fuse BSWAP with XSTORE to stwbrx. */
    Reg src = ra_alloc1(as, irb->op1, RSET_GPR);
    asm_fusexrefx(as, PPCI_STWBRX, src, ir->op1, rset_exclude(RSET_GPR, src));
  } else {
    Reg src = ra_alloc1(as, ir->op2,
      (!LJ_SOFTFP && irt_isfp(ir->t)) ? RSET_FPR : RSET_GPR);
    asm_fusexref(as, asm_fxstoreins(as, ir), src, ir->op1,
		 rset_exclude(RSET_GPR, src), ofs);
  }
}

#define asm_xstore(as, ir)	asm_xstore_(as, ir, 0)

static void asm_ahuvload(ASMState *as, IRIns *ir)
{
  IRType1 t = ir->t;
  Reg dest = RID_NONE, type = RID_TMP, tmp = RID_TMP, idx;
  RegSet allow = RSET_GPR;
  int32_t ofs = AHUREF_LSX;
  if (LJ_SOFTFP && (ir+1)->o == IR_HIOP) {
    t.irt = IRT_NUM;
    if (ra_used(ir+1)) {
      type = ra_dest(as, ir+1, allow);
      rset_clear(allow, type);
    }
    ofs = 0;
  }
  if (ra_used(ir)) {
    lj_assertA((LJ_SOFTFP ? 0 : irt_isnum(ir->t)) ||
	       irt_isint(ir->t) || irt_isaddr(ir->t),
	       "bad load type %d", irt_type(ir->t));
    if (LJ_SOFTFP || !irt_isnum(t)) ofs = 0;
    dest = ra_dest(as, ir, (!LJ_SOFTFP && irt_isnum(t)) ? RSET_FPR : allow);
    rset_clear(allow, dest);
  }
  idx = asm_fuseahuref(as, ir->op1, &ofs, allow);
  if (ir->o == IR_VLOAD) {
    ofs = ofs != AHUREF_LSX ? ofs + 8 * ir->op2 :
	  ir->op2 ? 8 * ir->op2 : AHUREF_LSX;
  }
  /* GC64: load the full 64-bit TValue, extract the itype from the high bits and
  ** load the value half. The LSX-fused case computes idx*8 into tmp first. */
  if (ofs == AHUREF_LSX) {
    tmp = ra_scratch(as, rset_exclude(rset_exclude(RSET_GPR,
						   (idx&255)), (idx>>8)));
  }
  if (irt_isnum(t)) {
    /* Pure double: itype < LJ_TISNUM. Exit (CC_LE) when TISNUMhi <= hi. */
    Reg tisnumhi = ra_allock(as, (int32_t)(LJ_TISNUM << 15),
			     rset_exclude(allow, idx));
    asm_guardcc(as, CC_LE);
    emit_ab(as, PPCI_CMPLW, tisnumhi, type);
    emit_rotdi(as, PPCI_RLDICL, type, type, 32, 32);  /* srdi 32: hi word. */
    if (ra_hasreg(dest)) {
      if (!LJ_SOFTFP && ofs == AHUREF_LSX)
	emit_fab(as, PPCI_LFDX, dest, (idx&255), tmp);
      else
	emit_fai(as, PPCI_LFD, dest, idx, ofs);
    }
  } else if (irt_isint(t)) {
    Reg tisnumhi = ra_allock(as, (int32_t)(LJ_TISNUM << 15),
			     rset_exclude(allow, idx));
    asm_guardcc(as, CC_NE);
    emit_ab(as, PPCI_CMPLW, type, tisnumhi);
    emit_rotdi(as, PPCI_RLDICL, type, type, 32, 32);  /* srdi 32: hi word. */
    if (ra_hasreg(dest)) emit_tai(as, PPCI_LWZ, dest, idx, ofs+(LJ_BE?4:0));
  } else {  /* Address: load full TValue once, derive itype, mask dest.
	    ** Execution order: LD load; sradi RID_TMP,load,47; mask dest;
	    ** cmpdi; guard. Emitted in reverse, so the mask (rldicl) must be
	    ** emitted BEFORE the sradi here, and the sradi reads the unmasked
	    ** value -- so use a distinct itype temp when load==dest. */
    Reg load = ra_hasreg(dest) ? dest : type;
    asm_guardcc(as, CC_NE);
    emit_ai(as, PPCI_CMPDI, RID_TMP, irt_toitype(t));
    if (ra_hasreg(dest))
      emit_rotdi(as, PPCI_RLDICL, dest, dest, 0, 17);  /* GCVMASK (after sradi). */
    emit_sradi(as, RID_TMP, load, 47);
    if (ofs == AHUREF_LSX) {
      emit_fab(as, PPCI_LDX, load, (idx&255), tmp);
      emit_slwi(as, tmp, (idx>>8), 3);
    } else {
      emit_tai(as, PPCI_LD, load, idx, ofs);
    }
    return;
  }
  /* For num/int: load the full TValue into the type register for itype check. */
  if (ofs == AHUREF_LSX) {
    emit_fab(as, PPCI_LDX, type, (idx&255), tmp);
    emit_slwi(as, tmp, (idx>>8), 3);
  } else {
    emit_tai(as, PPCI_LD, type, idx, ofs);
  }
}

static void asm_ahustore(ASMState *as, IRIns *ir)
{
  RegSet allow = RSET_GPR;
  Reg idx, src = RID_NONE, type = RID_NONE;
  int32_t ofs = AHUREF_LSX;
  if (ir->r == RID_SINK)
    return;
  if (!LJ_SOFTFP && irt_isnum(ir->t)) {
    /* A number is stored as the raw 64-bit double. May fuse an indexed ref. */
    src = ra_alloc1(as, ir->op2, RSET_FPR);
    idx = asm_fuseahuref(as, ir->op1, &ofs, allow);
    if (ofs == AHUREF_LSX) {
      emit_fab(as, PPCI_STFDX, src, (idx&255), RID_TMP);
      emit_slwi(as, RID_TMP, (idx>>8), 3);
    } else {
      emit_fai(as, PPCI_STFD, src, idx, ofs);
    }
    return;
  }
  /* GC64: store int/addr/pri as a single 64-bit tagged TValue via STD.
  ** Non-num refs force ofs=0, so asm_fuseahuref never returns the indexed
  ** (AHUREF_LSX) form here -- addressing is always base+ofs.
  */
  ofs = 0;
  if (!irref_isk(ir->op2)) {
    src = ra_alloc1(as, ir->op2, allow);
    rset_clear(allow, src);
    type = ra_allock(as, (int64_t)irt_toitype(ir->t) << 47, allow);
    rset_clear(allow, type);
  }
  idx = asm_fuseahuref(as, ir->op1, &ofs, allow);
  if (irref_isk(ir->op2)) {  /* Constant (incl. primitives): store k.u64. */
    TValue k;
    lj_ir_kvalue(as->J->L, &k, IR(ir->op2));
    emit_tai(as, PPCI_STD, ra_allock(as, (intptr_t)k.u64, rset_exclude(allow, idx)),
	     idx, ofs);
  } else if (irt_isinteger(ir->t)) {  /* (LJ_TISNUM<<47) | (uint32)value. */
    emit_tai(as, PPCI_STD, RID_TMP, idx, ofs);
    emit_tab(as, PPCI_ADD, RID_TMP, RID_TMP, type);
    emit_rotdi(as, PPCI_RLDICL, RID_TMP, src, 0, 32);  /* clrldi: zero-extend. */
  } else {  /* Address: (itype<<47) | gcptr. */
    emit_tai(as, PPCI_STD, RID_TMP, idx, ofs);
    emit_tab(as, PPCI_ADD, RID_TMP, src, type);
  }
}

static void asm_sload(ASMState *as, IRIns *ir)
{
  /* GC64/FR2: each frame slot is a full 8-byte TValue and the FR2 frame has
  ** two header slots, so slot N lives at base + 8*(N-2). The integer/value half
  ** of a TValue is the low 32 bits (LE: ofs+0, BE: ofs+4); the itype is in the
  ** high bits. */
  int32_t ofs = 8*((int32_t)ir->op1-2);
  IRType1 t = ir->t;
  Reg dest = RID_NONE, base;
#if LJ_SOFTFP
  Reg type = RID_NONE;
#endif
  RegSet allow = RSET_GPR;
  int hiop = (LJ_SOFTFP && (ir+1)->o == IR_HIOP);
  if (hiop)
    t.irt = IRT_NUM;
  lj_assertA(!(ir->op2 & IRSLOAD_PARENT),
	     "bad parent SLOAD");  /* Handled by asm_head_side(). */
  lj_assertA(irt_isguard(ir->t) || !(ir->op2 & IRSLOAD_TYPECHECK),
	     "inconsistent SLOAD variant");
  lj_assertA(LJ_DUALNUM ||
	     !irt_isint(t) ||
	     (ir->op2 & (IRSLOAD_CONVERT|IRSLOAD_FRAME|IRSLOAD_KEYINDEX)),
	     "bad SLOAD type");
#if LJ_SOFTFP
  lj_assertA(!(ir->op2 & IRSLOAD_CONVERT),
	     "unsplit SLOAD convert");  /* Handled by LJ_SOFTFP SPLIT. */
  if (hiop && ra_used(ir+1)) {
    type = ra_dest(as, ir+1, allow);
    rset_clear(allow, type);
  }
#else
  if ((ir->op2 & IRSLOAD_CONVERT) && irt_isguard(t) && irt_isint(t)) {
    dest = ra_scratch(as, RSET_FPR);
    asm_tointg(as, ir, dest);
    t.irt = IRT_NUM;  /* Continue with a regular number type check. */
  } else
#endif
  if (ra_used(ir)) {
    lj_assertA(irt_isnum(t) || irt_isint(t) || irt_isaddr(t),
	       "bad SLOAD type %d", irt_type(ir->t));
    dest = ra_dest(as, ir, (!LJ_SOFTFP && irt_isnum(t)) ? RSET_FPR : allow);
    rset_clear(allow, dest);
    base = ra_alloc1(as, REF_BASE, allow);
    rset_clear(allow, base);
    if (!LJ_SOFTFP && (ir->op2 & IRSLOAD_CONVERT)) {
      if (irt_isint(t)) {
	emit_tai(as, PPCI_LWZ, dest, RID_SP, SPOFS_TMPLO);
	dest = ra_scratch(as, RSET_FPR);
	emit_fai(as, PPCI_STFD, dest, RID_SP, SPOFS_TMP);
	emit_fb(as, PPCI_FCTIWZ, dest, dest);
	t.irt = IRT_NUM;  /* Check for original type. */
      } else {
	Reg tmp = ra_scratch(as, allow);
	Reg hibias = ra_allock(as, 0x43300000, rset_clear(allow, tmp));
	Reg fbias = ra_scratch(as, rset_exclude(RSET_FPR, dest));
	emit_fab(as, PPCI_FSUB, dest, dest, fbias);
	emit_fai(as, PPCI_LFD, dest, RID_SP, SPOFS_TMP);
	emit_lsptr(as, PPCI_LFS, (fbias & 31),
		   (void *)&as->J->k32[LJ_K32_2P52_2P31],
		   rset_clear(allow, hibias));
	emit_tai(as, PPCI_STW, tmp, RID_SP, SPOFS_TMPLO);
	emit_tai(as, PPCI_STW, hibias, RID_SP, SPOFS_TMPHI);
	emit_asi(as, PPCI_XORIS, tmp, tmp, 0x8000);
	dest = tmp;
	t.irt = IRT_INT;  /* Check for original type. */
      }
    }
    goto dotypecheck;
  }
  base = ra_alloc1(as, REF_BASE, allow);
  rset_clear(allow, base);
dotypecheck:
  if (irt_isnum(t)) {
    if ((ir->op2 & IRSLOAD_TYPECHECK)) {
      /* Number type check: load the full TValue, extract the high 32 bits
      ** (= itype<<15) and compare against TISNUMhi. A pure double has
      ** itype < LJ_TISNUM (an integer has itype == LJ_TISNUM, which must NOT
      ** pass a num SLOAD). So exit (not a pure double) when TISNUMhi <= hi,
      ** i.e. CC_LE on `cmplw TISNUMhi, hi` (cf arm64 CC_LS). */
      Reg tisnumhi = ra_allock(as, (int32_t)(LJ_TISNUM << 15), allow);
      asm_guardcc(as, CC_LE);
      emit_ab(as, PPCI_CMPLW, tisnumhi, RID_TMP);
      emit_rotdi(as, PPCI_RLDICL, RID_TMP, RID_TMP, 32, 32);  /* srdi 32. */
      emit_tai(as, PPCI_LD, RID_TMP, base, ofs);
    }
    if (ra_hasreg(dest)) emit_fai(as, LJ_SOFTFP ? PPCI_LWZ : PPCI_LFD, dest,
				  base, ofs+(LJ_SOFTFP?(LJ_BE?4:0):0));
  } else if (irt_isint(t)) {
    if ((ir->op2 & IRSLOAD_TYPECHECK)) {
      /* Integer type check: high 32 bits must equal TISNUMhi exactly. */
      Reg tisnumhi = ra_allock(as, (int32_t)(LJ_TISNUM << 15), allow);
      asm_guardcc(as, CC_NE);
      if ((ir->op2 & IRSLOAD_KEYINDEX)) {
	/* keyindex marker: high word == LJ_KEYINDEX. */
	emit_ai(as, PPCI_CMPWI, RID_TMP, (LJ_KEYINDEX & 0xffff));
	emit_asi(as, PPCI_XORIS, RID_TMP, RID_TMP, (LJ_KEYINDEX >> 16));
      } else {
	emit_ab(as, PPCI_CMPLW, RID_TMP, tisnumhi);
      }
      emit_rotdi(as, PPCI_RLDICL, RID_TMP, RID_TMP, 32, 32);  /* srdi 32. */
      emit_tai(as, PPCI_LD, RID_TMP, base, ofs);
    }
    if (ra_hasreg(dest))
      emit_tai(as, PPCI_LWZ, dest, base, ofs+(LJ_BE?4:0));
  } else {
    /* Address type check: extract the itype via sradi 47 and compare. */
    if ((ir->op2 & IRSLOAD_TYPECHECK)) {
      asm_guardcc(as, CC_NE);
      emit_ai(as, PPCI_CMPDI, RID_TMP, irt_toitype(t));
      emit_sradi(as, RID_TMP, RID_TMP, 47);
      emit_tai(as, PPCI_LD, RID_TMP, base, ofs);
    }
    if (ra_hasreg(dest)) {
      /* GC64: load the full 64-bit TValue and mask off the itype. */
      emit_rotdi(as, PPCI_RLDICL, dest, dest, 0, 17);
      emit_tai(as, PPCI_LD, dest, base, ofs);
    }
  }
}

/* -- Allocations --------------------------------------------------------- */

#if LJ_HASFFI
static void asm_cnew(ASMState *as, IRIns *ir)
{
  CTState *cts = ctype_ctsG(J2G(as->J));
  CTypeID id = (CTypeID)IR(ir->op1)->i;
  CTSize sz;
  CTInfo info = lj_ctype_info(cts, id, &sz);
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_lj_mem_newgco];
  IRRef args[4];
  RegSet drop = RSET_SCRATCH;
  lj_assertA(sz != CTSIZE_INVALID || (ir->o == IR_CNEW && ir->op2 != REF_NIL),
	     "bad CNEW/CNEWI operands");

  as->gcsteps++;
  if (ra_hasreg(ir->r))
    rset_clear(drop, ir->r);  /* Dest reg handled below. */
  ra_evictset(as, drop);
  if (ra_used(ir))
    ra_destreg(as, ir, RID_RET);  /* GCcdata * */

  /* Initialize immutable cdata object. */
  if (ir->o == IR_CNEWI) {
    RegSet allow = (RSET_GPR & ~RSET_SCRATCH);
    int32_t ofs = sizeof(GCcdata);
    lj_assertA(sz == 4 || sz == 8, "bad CNEWI size %d", sz);
    /* GC64/LJ_64: the value is a single 32- or 64-bit register (no HIOP). */
    Reg r = ra_alloc1(as, ir->op2, allow);
    emit_tai(as, sz == 8 ? PPCI_STD : PPCI_STW, r, RID_RET, ofs);
  } else if (ir->op2 != REF_NIL) {  /* Create VLA/VLS/aligned cdata. */
    ci = &lj_ir_callinfo[IRCALL_lj_cdata_newv];
    args[0] = ASMREF_L;     /* lua_State *L */
    args[1] = ir->op1;      /* CTypeID id   */
    args[2] = ir->op2;      /* CTSize sz    */
    args[3] = ASMREF_TMP1;  /* CTSize align */
    asm_gencall(as, ci, args);
    emit_loadi(as, ra_releasetmp(as, ASMREF_TMP1), (int32_t)ctype_align(info));
    return;
  }

  /* Initialize gct and ctypeid. lj_mem_newgco() already sets marked. */
  emit_tai(as, PPCI_STB, RID_RET+1, RID_RET, offsetof(GCcdata, gct));
  emit_tai(as, PPCI_STH, RID_TMP, RID_RET, offsetof(GCcdata, ctypeid));
  emit_ti(as, PPCI_LI, RID_RET+1, ~LJ_TCDATA);
  emit_ti(as, PPCI_LI, RID_TMP, id);  /* Lower 16 bit used. Sign-ext ok. */
  args[0] = ASMREF_L;     /* lua_State *L */
  args[1] = ASMREF_TMP1;  /* MSize size   */
  asm_gencall(as, ci, args);
  ra_allockreg(as, (int32_t)(sz+sizeof(GCcdata)),
	       ra_releasetmp(as, ASMREF_TMP1));
}
#endif

/* -- Write barriers ------------------------------------------------------ */

static void asm_tbar(ASMState *as, IRIns *ir)
{
  Reg tab = ra_alloc1(as, ir->op1, RSET_GPR);
  Reg mark = ra_scratch(as, rset_exclude(RSET_GPR, tab));
  Reg link = RID_TMP;
  MCLabel l_end = emit_label(as);
  /* GC64: gclist is a 64-bit GCRef. Storing it with stw leaves the high 32 bits
  ** stale (fatal on BE: the GC later traverses g->gc.grayagain and dereferences
  ** the corrupted gclist -> wild pointer in propagatemark). Use std. */
  emit_tai(as, PPCI_STD, link, tab, (int32_t)offsetof(GCtab, gclist));
  emit_tai(as, PPCI_STB, mark, tab, (int32_t)offsetof(GCtab, marked));
  emit_setgl(as, tab, gc.grayagain);
  lj_assertA(LJ_GC_BLACK == 0x04, "bad LJ_GC_BLACK");
  emit_rot(as, PPCI_RLWINM, mark, mark, 0, 30, 28);  /* Clear black bit. */
  emit_getgl(as, link, gc.grayagain);
  emit_condbranch(as, PPCI_BC|PPCF_Y, CC_EQ, l_end);
  emit_asi(as, PPCI_ANDIDOT, RID_TMP, mark, LJ_GC_BLACK);
  emit_tai(as, PPCI_LBZ, mark, tab, (int32_t)offsetof(GCtab, marked));
}

static void asm_obar(ASMState *as, IRIns *ir)
{
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_lj_gc_barrieruv];
  IRRef args[2];
  MCLabel l_end;
  Reg obj, val, tmp;
  /* No need for other object barriers (yet). */
  lj_assertA(IR(ir->op1)->o == IR_UREFC, "bad OBAR type");
  ra_evictset(as, RSET_SCRATCH);
  l_end = emit_label(as);
  args[0] = ASMREF_TMP1;  /* global_State *g */
  args[1] = ir->op1;      /* TValue *tv      */
  asm_gencall(as, ci, args);
  emit_tai(as, PPCI_ADDI, ra_releasetmp(as, ASMREF_TMP1), RID_JGL, -32768);
  obj = IR(ir->op1)->r;
  tmp = ra_scratch(as, rset_exclude(RSET_GPR, obj));
  emit_condbranch(as, PPCI_BC|PPCF_Y, CC_EQ, l_end);
  emit_asi(as, PPCI_ANDIDOT, tmp, tmp, LJ_GC_BLACK);
  emit_condbranch(as, PPCI_BC, CC_EQ, l_end);
  emit_asi(as, PPCI_ANDIDOT, RID_TMP, RID_TMP, LJ_GC_WHITES);
  val = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, obj));
  emit_tai(as, PPCI_LBZ, tmp, obj,
	   (int32_t)offsetof(GCupval, marked)-(int32_t)offsetof(GCupval, tv));
  emit_tai(as, PPCI_LBZ, RID_TMP, val, (int32_t)offsetof(GChead, marked));
}

/* -- Arithmetic and logic operations ------------------------------------- */

#if !LJ_SOFTFP
static void asm_fparith(ASMState *as, IRIns *ir, PPCIns pi)
{
  Reg dest = ra_dest(as, ir, RSET_FPR);
  Reg right, left = ra_alloc2(as, ir, RSET_FPR);
  right = (left >> 8); left &= 255;
  if (pi == PPCI_FMUL)
    emit_fac(as, pi, dest, left, right);
  else
    emit_fab(as, pi, dest, left, right);
}

static void asm_fpunary(ASMState *as, IRIns *ir, PPCIns pi)
{
  Reg dest = ra_dest(as, ir, RSET_FPR);
  Reg left = ra_hintalloc(as, ir->op1, dest, RSET_FPR);
  emit_fb(as, pi, dest, left);
}

static void asm_fpmath(ASMState *as, IRIns *ir)
{
  if (ir->op2 == IRFPM_SQRT && (as->flags & JIT_F_SQRT))
    asm_fpunary(as, ir, PPCI_FSQRT);
  else
    asm_callid(as, ir, IRCALL_lj_vm_floor + ir->op2);
}
#endif

static void asm_add(ASMState *as, IRIns *ir)
{
#if !LJ_SOFTFP
  if (irt_isnum(ir->t)) {
    if (!asm_fusemadd(as, ir, PPCI_FMADD, PPCI_FMADD))
      asm_fparith(as, ir, PPCI_FADD);
  } else
#endif
  {
    Reg dest = ra_dest(as, ir, RSET_GPR);
    Reg right, left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
    PPCIns pi;
    if (irref_isk(ir->op2) && checki32(get_kval(as, ir->op2))) {
      int32_t k = (int32_t)get_kval(as, ir->op2);
      if (checki16(k)) {
	pi = PPCI_ADDI;
	/* May fail due to spills/restores above, but simplifies the logic. */
	if (as->flagmcp == as->mcp) {
	  as->flagmcp = NULL;
	  as->mcp++;
	  pi = PPCI_ADDICDOT;
	}
	emit_tai(as, pi, dest, left, k);
	return;
      } else if ((k & 0xffff) == 0) {
	emit_tai(as, PPCI_ADDIS, dest, left, (k >> 16));
	return;
      } else if (!as->sectref) {
	emit_tai(as, PPCI_ADDIS, dest, dest, (k + 32768) >> 16);
	emit_tai(as, PPCI_ADDI, dest, left, k);
	return;
      }
    }
    pi = PPCI_ADD;
    /* May fail due to spills/restores above, but simplifies the logic. */
    if (as->flagmcp == as->mcp) {
      as->flagmcp = NULL;
      as->mcp++;
      pi |= PPCF_DOT;
    }
    right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
    emit_tab(as, pi, dest, left, right);
  }
}

static void asm_sub(ASMState *as, IRIns *ir)
{
#if !LJ_SOFTFP
  if (irt_isnum(ir->t)) {
    if (!asm_fusemadd(as, ir, PPCI_FMSUB, PPCI_FNMSUB))
      asm_fparith(as, ir, PPCI_FSUB);
  } else
#endif
  {
    PPCIns pi = PPCI_SUBF;
    Reg dest = ra_dest(as, ir, RSET_GPR);
    Reg left, right;
    if (irref_isk(ir->op1) && checki16(get_kval(as, ir->op1))) {
      int32_t k = (int32_t)get_kval(as, ir->op1);
      right = ra_alloc1(as, ir->op2, RSET_GPR);
      emit_tai(as, PPCI_SUBFIC, dest, right, k);
      return;
    }
    /* May fail due to spills/restores above, but simplifies the logic. */
    if (as->flagmcp == as->mcp) {
      as->flagmcp = NULL;
      as->mcp++;
      pi |= PPCF_DOT;
    }
    left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
    right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
    emit_tab(as, pi, dest, right, left);  /* Subtract right _from_ left. */
  }
}

static void asm_mul(ASMState *as, IRIns *ir)
{
#if !LJ_SOFTFP
  if (irt_isnum(ir->t)) {
    asm_fparith(as, ir, PPCI_FMUL);
  } else
#endif
  {
    PPCIns pi = PPCI_MULLW;
    Reg dest = ra_dest(as, ir, RSET_GPR);
    Reg right, left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
    if (irref_isk(ir->op2) && checki16(get_kval(as, ir->op2))) {
      int32_t k = (int32_t)get_kval(as, ir->op2);
      emit_tai(as, PPCI_MULLI, dest, left, k);
      return;
    }
    /* May fail due to spills/restores above, but simplifies the logic. */
    if (as->flagmcp == as->mcp) {
      as->flagmcp = NULL;
      as->mcp++;
      pi |= PPCF_DOT;
    }
    right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
    emit_tab(as, pi, dest, left, right);
  }
}

#define asm_fpdiv(as, ir)	asm_fparith(as, ir, PPCI_FDIV)

static void asm_neg(ASMState *as, IRIns *ir)
{
#if !LJ_SOFTFP
  if (irt_isnum(ir->t)) {
    asm_fpunary(as, ir, PPCI_FNEG);
  } else
#endif
  {
    Reg dest, left;
    PPCIns pi = PPCI_NEG;
    if (as->flagmcp == as->mcp) {
      as->flagmcp = NULL;
      as->mcp++;
      pi |= PPCF_DOT;
    }
    dest = ra_dest(as, ir, RSET_GPR);
    left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
    emit_tab(as, pi, dest, left, 0);
  }
}

#define asm_abs(as, ir)		asm_fpunary(as, ir, PPCI_FABS)

static void asm_arithov(ASMState *as, IRIns *ir, PPCIns pi)
{
  Reg dest, left, right;
  if (as->flagmcp == as->mcp) {
    as->flagmcp = NULL;
    as->mcp++;
  }
  asm_guardcc(as, CC_SO);
  dest = ra_dest(as, ir, RSET_GPR);
  left = ra_alloc2(as, ir, RSET_GPR);
  right = (left >> 8); left &= 255;
  if (pi == PPCI_MULLWO) {
    /* mullwo. is a 32x32->32 multiply whose OV/SO already reflect 32-bit
    ** overflow, so the plain dot-form suffices. */
    if (pi == PPCI_SUBFO) { Reg tmp = left; left = right; right = tmp; }
    emit_tab(as, pi|PPCF_DOT, dest, left, right);
    return;
  }
  /* GC64: addo./subo. are 64-bit ops -- their OV/SO only catch a 64-bit
  ** overflow, not the 32-bit Lua integer overflow we must guard. Detect 32-bit
  ** overflow like the interpreter's addo32./subo32. macros: shift both operands
  ** left 32 and run the flag-setting op on the shifted values ((a<<32) +/-
  ** (b<<32) overflows 64-bit iff a +/- b overflows 32-bit), with the real
  ** 32-bit result from a separate plain add/subf. Emitted in reverse:
  ** sldi t0,sl,32; sldi RID_TMP,sr,32; <flag>o. RID_TMP,t0,RID_TMP; <guard>;
  ** add/subf dest,sl,sr. */
  {
    int sub = (pi == PPCI_SUBFO);
    Reg sl = left, sr = right;
    Reg t0;
    if (sub) { Reg tmp = sl; sl = sr; sr = tmp; }  /* subf: result = right-left. */
    t0 = ra_scratch(as, rset_exclude(rset_exclude(rset_exclude(RSET_GPR,
		    dest), sl), sr));
    /* Real 32-bit result. */
    emit_tab(as, sub ? PPCI_SUBF : PPCI_ADD, dest, sl, sr);
    /* Flag op on the <<32 operands -> CR0[SO] reflects 32-bit overflow. */
    emit_tab(as, (sub ? PPCI_SUBFO : PPCI_ADDO)|PPCF_DOT, RID_TMP, t0, RID_TMP);
    emit_rotdi(as, PPCI_RLDICR, RID_TMP, sr, 32, 31);  /* sr << 32. */
    emit_rotdi(as, PPCI_RLDICR, t0, sl, 32, 31);       /* sl << 32. */
  }
}

#define asm_addov(as, ir)	asm_arithov(as, ir, PPCI_ADDO)
#define asm_subov(as, ir)	asm_arithov(as, ir, PPCI_SUBFO)
#define asm_mulov(as, ir)	asm_arithov(as, ir, PPCI_MULLWO)

#if LJ_HASFFI
static void asm_add64(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg right, left = ra_alloc1(as, ir->op1, RSET_GPR);
  PPCIns pi = PPCI_ADDE;
  if (irref_isk(ir->op2)) {
    int32_t k = IR(ir->op2)->i;
    if (k == 0)
      pi = PPCI_ADDZE;
    else if (k == -1)
      pi = PPCI_ADDME;
    else
      goto needright;
    right = 0;
  } else {
  needright:
    right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
  }
  emit_tab(as, pi, dest, left, right);
  ir--;
  dest = ra_dest(as, ir, RSET_GPR);
  left = ra_alloc1(as, ir->op1, RSET_GPR);
  if (irref_isk(ir->op2)) {
    int32_t k = IR(ir->op2)->i;
    if (checki16(k)) {
      emit_tai(as, PPCI_ADDIC, dest, left, k);
      return;
    }
  }
  right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
  emit_tab(as, PPCI_ADDC, dest, left, right);
}

static void asm_sub64(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left, right = ra_alloc1(as, ir->op2, RSET_GPR);
  PPCIns pi = PPCI_SUBFE;
  if (irref_isk(ir->op1)) {
    int32_t k = IR(ir->op1)->i;
    if (k == 0)
      pi = PPCI_SUBFZE;
    else if (k == -1)
      pi = PPCI_SUBFME;
    else
      goto needleft;
    left = 0;
  } else {
  needleft:
    left = ra_alloc1(as, ir->op1, rset_exclude(RSET_GPR, right));
  }
  emit_tab(as, pi, dest, right, left);  /* Subtract right _from_ left. */
  ir--;
  dest = ra_dest(as, ir, RSET_GPR);
  right = ra_alloc1(as, ir->op2, RSET_GPR);
  if (irref_isk(ir->op1)) {
    int32_t k = IR(ir->op1)->i;
    if (checki16(k)) {
      emit_tai(as, PPCI_SUBFIC, dest, right, k);
      return;
    }
  }
  left = ra_alloc1(as, ir->op1, rset_exclude(RSET_GPR, right));
  emit_tab(as, PPCI_SUBFC, dest, right, left);
}

static void asm_neg64(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
  emit_tab(as, PPCI_SUBFZE, dest, left, 0);
  ir--;
  dest = ra_dest(as, ir, RSET_GPR);
  left = ra_alloc1(as, ir->op1, RSET_GPR);
  emit_tai(as, PPCI_SUBFIC, dest, left, 0);
}
#endif

static void asm_bnot(ASMState *as, IRIns *ir)
{
  Reg dest, left, right;
  PPCIns pi = PPCI_NOR;
  if (as->flagmcp == as->mcp) {
    as->flagmcp = NULL;
    as->mcp++;
    pi |= PPCF_DOT;
  }
  dest = ra_dest(as, ir, RSET_GPR);
  if (mayfuse(as, ir->op1)) {
    IRIns *irl = IR(ir->op1);
    if (irl->o == IR_BAND)
      pi ^= (PPCI_NOR ^ PPCI_NAND);
    else if (irl->o == IR_BXOR)
      pi ^= (PPCI_NOR ^ PPCI_EQV);
    else if (irl->o != IR_BOR)
      goto nofuse;
    left = ra_hintalloc(as, irl->op1, dest, RSET_GPR);
    right = ra_alloc1(as, irl->op2, rset_exclude(RSET_GPR, left));
  } else {
nofuse:
    left = right = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
  }
  emit_asb(as, pi, dest, left, right);
}

static void asm_bswap(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  IRIns *irx;
  if (irt_is64(ir->t)) {
    /* 64-bit byte-reverse: reverse each 32-bit half, then swap halves.
    ** dest = bswap32(hi) | (bswap32(lo) << 32), where lo = low 32 bits. */
    Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
    Reg tmp = ra_scratch(as, rset_exclude(rset_exclude(RSET_GPR, dest), left));
    Reg tmp2 = ra_scratch(as, rset_exclude(rset_exclude(rset_exclude(
		 RSET_GPR, dest), left), tmp));
    /* Final: dest = rldimi(tmp2_reversed_lo<<32 into dest holding reversed-hi) */
    /* Build reversed low half into tmp (from left's low 32), reversed high half
    ** into dest (from left's high 32), then dest = (tmp<<32) | dest. */
    /* dest = dest | (tmp<<32): use rldimi dest, tmp, 32, 0. */
    emit_rotdi(as, PPCI_RLDIMI, dest, tmp, 32, 0);
    /* Reverse high 32 of `left` into dest (low 32 of dest). */
    emit_rot(as, PPCI_RLWIMI, dest, tmp2, 24, 16, 23);
    emit_rot(as, PPCI_RLWIMI, dest, tmp2, 24, 0, 7);
    emit_rotlwi(as, dest, tmp2, 8);
    emit_rotdi(as, PPCI_RLDICL, tmp2, left, 32, 32);  /* tmp2 = left >> 32. */
    /* Reverse low 32 of `left` into tmp. */
    emit_rot(as, PPCI_RLWIMI, tmp, left, 24, 16, 23);
    emit_rot(as, PPCI_RLWIMI, tmp, left, 24, 0, 7);
    emit_rotlwi(as, tmp, left, 8);
    return;
  }
  if (mayfuse(as, ir->op1) && (irx = IR(ir->op1))->o == IR_XLOAD &&
      ra_noreg(irx->r) && (irt_isint(irx->t) || irt_isu32(irx->t))) {
    /* Fuse BSWAP with XLOAD to lwbrx. */
    asm_fusexrefx(as, PPCI_LWBRX, dest, irx->op1, RSET_GPR);
  } else {
    Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
    Reg tmp = dest;
    if (tmp == left) {
      tmp = RID_TMP;
      emit_mr(as, dest, RID_TMP);
    }
    emit_rot(as, PPCI_RLWIMI, tmp, left, 24, 16, 23);
    emit_rot(as, PPCI_RLWIMI, tmp, left, 24, 0, 7);
    emit_rotlwi(as, tmp, left, 8);
  }
}

/* Fuse BAND with contiguous bitmask and a shift to rlwinm. */
static void asm_fuseandsh(ASMState *as, PPCIns pi, int32_t mask, IRRef ref)
{
  IRIns *ir;
  Reg left;
  if (mayfuse(as, ref) && (ir = IR(ref), ra_noreg(ir->r)) &&
      irref_isk(ir->op2) && ir->o >= IR_BSHL && ir->o <= IR_BROR) {
    int32_t sh = (IR(ir->op2)->i & 31);
    switch (ir->o) {
    case IR_BSHL:
      if ((mask & ((1u<<sh)-1))) goto nofuse;
      break;
    case IR_BSHR:
      if ((mask & ~((~0u)>>sh))) goto nofuse;
      sh = ((32-sh)&31);
      break;
    case IR_BROL:
      break;
    default:
      goto nofuse;
    }
    left = ra_alloc1(as, ir->op1, RSET_GPR);
    *--as->mcp = pi | PPCF_T(left) | PPCF_B(sh);
    return;
  }
nofuse:
  left = ra_alloc1(as, ref, RSET_GPR);
  *--as->mcp = pi | PPCF_T(left);
}

static void asm_band(ASMState *as, IRIns *ir)
{
  Reg dest, left, right;
  IRRef lref = ir->op1;
  PPCIns dot = 0;
  IRRef op2;
  if (as->flagmcp == as->mcp) {
    as->flagmcp = NULL;
    as->mcp++;
    dot = PPCF_DOT;
  }
  dest = ra_dest(as, ir, RSET_GPR);
  /* The RLWINM-mask fusion and ANDIS paths are 32-bit only (they zero the
  ** upper 32 bits), so for 64-bit BAND only andi. with a u16 mask (high bits
  ** known 0) is safe; everything else goes reg-reg via a materialized const. */
  if (irref_isk(ir->op2) &&
      !(irt_is64(ir->t) && !checku16(get_kval(as, ir->op2)))) {
    int32_t k = (int32_t)get_kval(as, ir->op2);
    if (k && !irt_is64(ir->t)) {
      /* First check for a contiguous bitmask as used by rlwinm. */
      uint32_t s1 = lj_ffs((uint32_t)k);
      uint32_t k1 = ((uint32_t)k >> s1);
      if ((k1 & (k1+1)) == 0) {
	asm_fuseandsh(as, PPCI_RLWINM|dot | PPCF_A(dest) |
			  PPCF_MB(31-lj_fls((uint32_t)k)) | PPCF_ME(31-s1),
			  k, lref);
	return;
      }
      if (~(uint32_t)k) {
	uint32_t s2 = lj_ffs(~(uint32_t)k);
	uint32_t k2 = (~(uint32_t)k >> s2);
	if ((k2 & (k2+1)) == 0) {
	  asm_fuseandsh(as, PPCI_RLWINM|dot | PPCF_A(dest) |
			    PPCF_MB(32-s2) | PPCF_ME(30-lj_fls(~(uint32_t)k)),
			    k, lref);
	  return;
	}
      }
    }
    if (checku16(k)) {
      left = ra_alloc1(as, lref, RSET_GPR);
      emit_asi(as, PPCI_ANDIDOT, dest, left, k);
      return;
    } else if ((k & 0xffff) == 0 && !irt_is64(ir->t)) {
      left = ra_alloc1(as, lref, RSET_GPR);
      emit_asi(as, PPCI_ANDISDOT, dest, left, (k >> 16));
      return;
    }
  }
  op2 = ir->op2;
  if (mayfuse(as, op2) && IR(op2)->o == IR_BNOT && ra_noreg(IR(op2)->r)) {
    dot ^= (PPCI_AND ^ PPCI_ANDC);
    op2 = IR(op2)->op1;
  }
  left = ra_hintalloc(as, lref, dest, RSET_GPR);
  right = ra_alloc1(as, op2, rset_exclude(RSET_GPR, left));
  emit_asb(as, PPCI_AND ^ dot, dest, left, right);
}

static void asm_bitop(ASMState *as, IRIns *ir, PPCIns pi, PPCIns pik)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg right, left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
  /* For 64-bit bitops, only the 16-bit zero-extended immediate forms (andi./
  ** ori/xori) are safe; any wider constant (incl. high-word bits) must go
  ** through a materialized register, else the ORIS/32-bit paths corrupt the
  ** upper 32 bits. */
  if (irref_isk(ir->op2) &&
      !(irt_is64(ir->t) && !checku16(get_kval(as, ir->op2)))) {
    int32_t k = (int32_t)get_kval(as, ir->op2);
    Reg tmp = left;
    if ((checku16(k) || (k & 0xffff) == 0) || (tmp = dest, !as->sectref)) {
      if (!checku16(k)) {
	emit_asi(as, pik ^ (PPCI_ORI ^ PPCI_ORIS), dest, tmp, (k >> 16));
	if ((k & 0xffff) == 0) return;
      }
      emit_asi(as, pik, dest, left, k);
      return;
    }
  }
  /* May fail due to spills/restores above, but simplifies the logic. */
  if (as->flagmcp == as->mcp) {
    as->flagmcp = NULL;
    as->mcp++;
    pi |= PPCF_DOT;
  }
  right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
  emit_asb(as, pi, dest, left, right);
}

#define asm_bor(as, ir)		asm_bitop(as, ir, PPCI_OR, PPCI_ORI)
#define asm_bxor(as, ir)	asm_bitop(as, ir, PPCI_XOR, PPCI_XORI)

static void asm_bitshift(ASMState *as, IRIns *ir, PPCIns pi, PPCIns pik)
{
  Reg dest, left;
  Reg dot = 0;
  if (as->flagmcp == as->mcp) {
    as->flagmcp = NULL;
    as->mcp++;
    dot = PPCF_DOT;
  }
  dest = ra_dest(as, ir, RSET_GPR);
  left = ra_alloc1(as, ir->op1, RSET_GPR);
  if (irt_is64(ir->t)) {  /* 64-bit shifts (GC64/FFI i64/u64). */
    /* pik selects the operation: 0=shl, 1=shr, SRAWI=>sar, RLWINM=>rol. */
    if (irref_isk(ir->op2)) {  /* Constant 64-bit shift. */
      int32_t shift = (IR(ir->op2)->i & 63);
      if (pik == 0)  /* sldi = rldicr rd,rs,shift,63-shift */
	emit_rotdi(as, PPCI_RLDICR|dot, dest, left, shift, 63-shift);
      else if (pik == 1)  /* srdi = rldicl rd,rs,64-shift,shift */
	emit_rotdi(as, PPCI_RLDICL|dot, dest, left, (64-shift)&63, shift);
      else if (pik == PPCI_SRAWI)  /* sradi */
	emit_sradi(as, dest, left, shift);
      else  /* rotate left (rldicl rd,rs,shift,0) */
	emit_rotdi(as, PPCI_RLDICL|dot, dest, left, shift, 0);
    } else {
      Reg right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
      PPCIns pi64 = pik == 0 ? PPCI_SLD : pik == 1 ? PPCI_SRD :
		    pik == PPCI_SRAWI ? PPCI_SRAD : PPCI_RLDCL;
      emit_asb(as, pi64|dot, dest, left, right);
    }
    return;
  }
  if (irref_isk(ir->op2)) {  /* Constant shifts. */
    int32_t shift = (IR(ir->op2)->i & 31);
    if (pik == 0)  /* SLWI */
      emit_rot(as, PPCI_RLWINM|dot, dest, left, shift, 0, 31-shift);
    else if (pik == 1)  /* SRWI */
      emit_rot(as, PPCI_RLWINM|dot, dest, left, (32-shift)&31, shift, 31);
    else
      emit_asb(as, pik|dot, dest, left, shift);
  } else {
    Reg right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
    emit_asb(as, pi|dot, dest, left, right);
  }
}

#define asm_bshl(as, ir)	asm_bitshift(as, ir, PPCI_SLW, 0)
#define asm_bshr(as, ir)	asm_bitshift(as, ir, PPCI_SRW, 1)
#define asm_bsar(as, ir)	asm_bitshift(as, ir, PPCI_SRAW, PPCI_SRAWI)
#define asm_brol(as, ir) \
  asm_bitshift(as, ir, PPCI_RLWNM|PPCF_MB(0)|PPCF_ME(31), \
		       PPCI_RLWINM|PPCF_MB(0)|PPCF_ME(31))
#define asm_bror(as, ir)	lj_assertA(0, "unexpected BROR")

#if LJ_SOFTFP
static void asm_sfpmin_max(ASMState *as, IRIns *ir)
{
  CCallInfo ci = lj_ir_callinfo[IRCALL_softfp_cmp];
  IRRef args[4];
  MCLabel l_right, l_end;
  Reg desthi = ra_dest(as, ir, RSET_GPR), destlo = ra_dest(as, ir+1, RSET_GPR);
  Reg righthi, lefthi = ra_alloc2(as, ir, RSET_GPR);
  Reg rightlo, leftlo = ra_alloc2(as, ir+1, RSET_GPR);
  PPCCC cond = (IROp)ir->o == IR_MIN ? CC_EQ : CC_NE;
  righthi = (lefthi >> 8); lefthi &= 255;
  rightlo = (leftlo >> 8); leftlo &= 255;
  args[0^LJ_BE] = ir->op1; args[1^LJ_BE] = (ir+1)->op1;
  args[2^LJ_BE] = ir->op2; args[3^LJ_BE] = (ir+1)->op2;
  l_end = emit_label(as);
  if (desthi != righthi) emit_mr(as, desthi, righthi);
  if (destlo != rightlo) emit_mr(as, destlo, rightlo);
  l_right = emit_label(as);
  if (l_end != l_right) emit_jmp(as, l_end);
  if (desthi != lefthi) emit_mr(as, desthi, lefthi);
  if (destlo != leftlo) emit_mr(as, destlo, leftlo);
  if (l_right == as->mcp+1) {
    cond ^= 4; l_right = l_end; ++as->mcp;
  }
  emit_condbranch(as, PPCI_BC, cond, l_right);
  ra_evictset(as, RSET_SCRATCH);
  emit_cmpi(as, RID_RET, 1);
  asm_gencall(as, &ci, args);
}
#endif

static void asm_min_max(ASMState *as, IRIns *ir, int ismax)
{
  if (!LJ_SOFTFP && irt_isnum(ir->t)) {
    Reg dest = ra_dest(as, ir, RSET_FPR);
    Reg tmp = dest;
    Reg right, left = ra_alloc2(as, ir, RSET_FPR);
    right = (left >> 8); left &= 255;
    if (tmp == left || tmp == right)
      tmp = ra_scratch(as, rset_exclude(rset_exclude(rset_exclude(RSET_FPR,
					dest), left), right));
    emit_facb(as, PPCI_FSEL, dest, tmp, left, right);
    emit_fab(as, PPCI_FSUB, tmp, ismax ? left : right, ismax ? right : left);
  } else {
    Reg dest = ra_dest(as, ir, RSET_GPR);
    Reg tmp1 = RID_TMP, tmp2 = dest;
    Reg right, left = ra_alloc2(as, ir, RSET_GPR);
    right = (left >> 8); left &= 255;
    if (tmp2 == left || tmp2 == right)
      tmp2 = ra_scratch(as, rset_exclude(rset_exclude(rset_exclude(RSET_GPR,
					 dest), left), right));
    emit_tab(as, PPCI_ADD, dest, tmp2, right);
    emit_asb(as, ismax ? PPCI_ANDC : PPCI_AND, tmp2, tmp2, tmp1);
    emit_tab(as, PPCI_SUBFE, tmp1, tmp1, tmp1);
    emit_tab(as, PPCI_SUBFC, tmp2, tmp2, tmp1);
    emit_asi(as, PPCI_XORIS, tmp2, right, 0x8000);
    emit_asi(as, PPCI_XORIS, tmp1, left, 0x8000);
  }
}

#define asm_min(as, ir)		asm_min_max(as, ir, 0)
#define asm_max(as, ir)		asm_min_max(as, ir, 1)

/* -- Comparisons --------------------------------------------------------- */

#define CC_UNSIGNED	0x08	/* Unsigned integer comparison. */
#define CC_TWO		0x80	/* Check two flags for FP comparison. */

/* Map of comparisons to flags. ORDER IR. */
static const uint8_t asm_compmap[IR_ABC+1] = {
  /* op     int cc                 FP cc */
  /* LT  */ CC_GE               + (CC_GE<<4),
  /* GE  */ CC_LT               + (CC_LE<<4) + CC_TWO,
  /* LE  */ CC_GT               + (CC_GE<<4) + CC_TWO,
  /* GT  */ CC_LE               + (CC_LE<<4),
  /* ULT */ CC_GE + CC_UNSIGNED + (CC_GT<<4) + CC_TWO,
  /* UGE */ CC_LT + CC_UNSIGNED + (CC_LT<<4),
  /* ULE */ CC_GT + CC_UNSIGNED + (CC_GT<<4),
  /* UGT */ CC_LE + CC_UNSIGNED + (CC_LT<<4) + CC_TWO,
  /* EQ  */ CC_NE               + (CC_NE<<4),
  /* NE  */ CC_EQ               + (CC_EQ<<4),
  /* ABC */ CC_LE + CC_UNSIGNED + (CC_LT<<4) + CC_TWO  /* Same as UGT. */
};

/* 64-bit integer comparison (GC64/FFI i64/u64). */
static void asm_intcomp64_(ASMState *as, IRRef lref, IRRef rref, Reg cr,
			   PPCCC cc)
{
  Reg right, left = ra_alloc1(as, lref, RSET_GPR);
  if (irref_isk(rref)) {
    intptr_t k = get_kval(as, rref);
    if ((cc & CC_UNSIGNED) == 0) {  /* Signed comparison with constant. */
      if (checki16(k)) {
	emit_tai(as, PPCI_CMPDI, cr, left, (int32_t)k);
	if (k == 0 && lref == as->curins-1)
	  as->flagmcp = as->mcp;
	return;
      } else if ((cc & 3) == (CC_EQ & 3) && checku16(k)) {
	emit_tai(as, PPCI_CMPLDI, cr, left, (int32_t)k);
	return;
      }
    } else {  /* Unsigned comparison with constant. */
      if (checku16(k)) {
	emit_tai(as, PPCI_CMPLDI, cr, left, (int32_t)k);
	return;
      }
    }
  }
  right = ra_alloc1(as, rref, rset_exclude(RSET_GPR, left));
  emit_tab(as, (cc & CC_UNSIGNED) ? PPCI_CMPLD : PPCI_CMPD, cr, left, right);
}

static void asm_intcomp_(ASMState *as, IRRef lref, IRRef rref, Reg cr, PPCCC cc)
{
  Reg right, left = ra_alloc1(as, lref, RSET_GPR);
  if (irref_isk(rref)) {
    int32_t k = IR(rref)->i;
    if ((cc & CC_UNSIGNED) == 0) {  /* Signed comparison with constant. */
      if (checki16(k)) {
	emit_tai(as, PPCI_CMPWI, cr, left, k);
	/* NB: on ppc64 we must NOT eliminate this 32-bit cmpwi via the dot-form
	** of the preceding op: add./addi./etc. record the full 64-bit result, so
	** for a 32-bit int op whose result overflows into bit 31 (e.g. tobit
	** i+0x7fffffff) the recorded sign (bit 63) differs from the int32 sign
	** (bit 31) -> wrong "< 0". The dot-form fusion (flagmcp) is only valid for
	** 64-bit ops, where it is kept in asm_intcomp64_ (cmpdi). */
	return;
      } else if ((cc & 3) == (CC_EQ & 3)) {  /* Use CMPLWI for EQ or NE. */
	if (checku16(k)) {
	  emit_tai(as, PPCI_CMPLWI, cr, left, k);
	  return;
	} else if (!as->sectref && ra_noreg(IR(rref)->r)) {
	  emit_tai(as, PPCI_CMPLWI, cr, RID_TMP, k);
	  emit_asi(as, PPCI_XORIS, RID_TMP, left, (k >> 16));
	  return;
	}
      }
    } else {  /* Unsigned comparison with constant. */
      if (checku16(k)) {
	emit_tai(as, PPCI_CMPLWI, cr, left, k);
	return;
      }
    }
  }
  right = ra_alloc1(as, rref, rset_exclude(RSET_GPR, left));
  emit_tab(as, (cc & CC_UNSIGNED) ? PPCI_CMPLW : PPCI_CMPW, cr, left, right);
}

static void asm_comp(ASMState *as, IRIns *ir)
{
  PPCCC cc = asm_compmap[ir->o];
  if (!LJ_SOFTFP && irt_isnum(ir->t)) {
    Reg right, left = ra_alloc2(as, ir, RSET_FPR);
    right = (left >> 8); left &= 255;
    asm_guardcc(as, (cc >> 4));
    if ((cc & CC_TWO))
      emit_tab(as, PPCI_CROR, ((cc>>4)&3), ((cc>>4)&3), (CC_EQ&3));
    emit_fab(as, PPCI_FCMPU, 0, left, right);
  } else if (irt_isaddr(ir->t)) {
    /* GC64: EQ/NE of 64-bit GCobj references (function-identity guard, the
    ** `tab NE NULL` metatable check, etc.). Compare full 64-bit pointers. Use
    ** ra_alloc1 for the RHS even when it's a constant: it remats KGC/KPTR/KKPTR/
    ** KNULL correctly (a NULL operand is a KPTR/KNULL, NOT a KGC -- ir_kgc would
    ** read a garbage gcr field and the guard would mis-compare). */
    Reg right, left = ra_alloc1(as, ir->op1, RSET_GPR);
    IRRef rref = ir->op2;
    asm_guardcc(as, cc);
    right = ra_alloc1(as, rref, rset_exclude(RSET_GPR, left));
    emit_tab(as, (cc & CC_UNSIGNED) ? PPCI_CMPLD : PPCI_CMPD, 0, left, right);
  } else {
    IRRef lref = ir->op1, rref = ir->op2;
    if (irref_isk(lref) && !irref_isk(rref)) {
      /* Swap constants to the right (only for ABC). */
      IRRef tmp = lref; lref = rref; rref = tmp;
      if ((cc & 2) == 0) cc ^= 1;  /* LT <-> GT, LE <-> GE */
    }
    asm_guardcc(as, cc);
    if (irt_is64(ir->t))  /* GC64/FFI native 64-bit integer comparison. */
      asm_intcomp64_(as, lref, rref, 0, cc);
    else
      asm_intcomp_(as, lref, rref, 0, cc);
  }
}

#define asm_equal(as, ir)	asm_comp(as, ir)

#if LJ_SOFTFP
/* SFP comparisons. */
static void asm_sfpcomp(ASMState *as, IRIns *ir)
{
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_softfp_cmp];
  RegSet drop = RSET_SCRATCH;
  Reg r;
  IRRef args[4];
  args[0^LJ_BE] = ir->op1; args[1^LJ_BE] = (ir+1)->op1;
  args[2^LJ_BE] = ir->op2; args[3^LJ_BE] = (ir+1)->op2;

  for (r = REGARG_FIRSTGPR; r <= REGARG_FIRSTGPR+3; r++) {
    if (!rset_test(as->freeset, r) &&
	regcost_ref(as->cost[r]) == args[r-REGARG_FIRSTGPR])
      rset_clear(drop, r);
  }
  ra_evictset(as, drop);
  asm_setupresult(as, ir, ci);
  switch ((IROp)ir->o) {
  case IR_ULT:
    asm_guardcc(as, CC_EQ);
    emit_ai(as, PPCI_CMPWI, RID_RET, 0);
  case IR_ULE:
    asm_guardcc(as, CC_EQ);
    emit_ai(as, PPCI_CMPWI, RID_RET, 1);
    break;
  case IR_GE: case IR_GT:
    asm_guardcc(as, CC_EQ);
    emit_ai(as, PPCI_CMPWI, RID_RET, 2);
  default:
    asm_guardcc(as, (asm_compmap[ir->o] & 0xf));
    emit_ai(as, PPCI_CMPWI, RID_RET, 0);
    break;
  }
  asm_gencall(as, ci, args);
}
#endif

#if LJ_HASFFI
/* 64 bit integer comparisons. */
static void asm_comp64(ASMState *as, IRIns *ir)
{
  PPCCC cc = asm_compmap[(ir-1)->o];
  if ((cc&3) == (CC_EQ&3)) {
    asm_guardcc(as, cc);
    emit_tab(as, (cc&4) ? PPCI_CRAND : PPCI_CROR,
	     (CC_EQ&3), (CC_EQ&3), 4+(CC_EQ&3));
  } else {
    asm_guardcc(as, CC_EQ);
    emit_tab(as, PPCI_CROR, (CC_EQ&3), (CC_EQ&3), ((cc^~(cc>>2))&1));
    emit_tab(as, (cc&4) ? PPCI_CRAND : PPCI_CRANDC,
	     (CC_EQ&3), (CC_EQ&3), 4+(cc&3));
  }
  /* Loword comparison sets cr1 and is unsigned, except for equality. */
  asm_intcomp_(as, (ir-1)->op1, (ir-1)->op2, 4,
	       cc | ((cc&3) == (CC_EQ&3) ? 0 : CC_UNSIGNED));
  /* Hiword comparison sets cr0. */
  asm_intcomp_(as, ir->op1, ir->op2, 0, cc);
  as->flagmcp = NULL;  /* Doesn't work here. */
}
#endif

/* -- Split register ops -------------------------------------------------- */

/* Hiword op of a split 32/32 bit op. Previous op is be the loword op. */
static void asm_hiop(ASMState *as, IRIns *ir)
{
  /* HIOP is marked as a store because it needs its own DCE logic. */
  int uselo = ra_used(ir-1), usehi = ra_used(ir);  /* Loword/hiword used? */
  if (LJ_UNLIKELY(!(as->flags & JIT_F_OPT_DCE))) uselo = usehi = 1;
#if !LJ_64 && (LJ_HASFFI || LJ_SOFTFP)
  if ((ir-1)->o == IR_CONV) {  /* Conversions to/from 64 bit. */
    as->curins--;  /* Always skip the CONV. */
#if LJ_HASFFI && !LJ_SOFTFP
    if (usehi || uselo)
      asm_conv64(as, ir);
    return;
#endif
  } else if ((ir-1)->o <= IR_NE) {  /* 64 bit integer comparisons. ORDER IR. */
    as->curins--;  /* Always skip the loword comparison. */
#if LJ_SOFTFP
    if (!irt_isint(ir->t)) {
      asm_sfpcomp(as, ir-1);
      return;
    }
#endif
#if LJ_HASFFI
    asm_comp64(as, ir);
#endif
    return;
#if LJ_SOFTFP
  } else if ((ir-1)->o == IR_MIN || (ir-1)->o == IR_MAX) {
      as->curins--;  /* Always skip the loword min/max. */
    if (uselo || usehi)
      asm_sfpmin_max(as, ir-1);
    return;
#endif
  } else if ((ir-1)->o == IR_XSTORE) {
    as->curins--;  /* Handle both stores here. */
    if ((ir-1)->r != RID_SINK) {
      asm_xstore_(as, ir, 0);
      asm_xstore_(as, ir-1, 4);
    }
    return;
  }
#endif
  if (!usehi) return;  /* Skip unused hiword op for all remaining ops. */
  switch ((ir-1)->o) {
#if !LJ_64 && LJ_HASFFI
  case IR_ADD: as->curins--; asm_add64(as, ir); break;
  case IR_SUB: as->curins--; asm_sub64(as, ir); break;
  case IR_NEG: as->curins--; asm_neg64(as, ir); break;
  case IR_CNEWI:
    /* Nothing to do here. Handled by lo op itself. */
    break;
#endif
#if LJ_SOFTFP
  case IR_SLOAD: case IR_ALOAD: case IR_HLOAD: case IR_ULOAD: case IR_VLOAD:
  case IR_STRTO:
    if (!uselo)
      ra_allocref(as, ir->op1, RSET_GPR);  /* Mark lo op as used. */
    break;
  case IR_ASTORE: case IR_HSTORE: case IR_USTORE: case IR_TOSTR: case IR_TMPREF:
    /* Nothing to do here. Handled by lo op itself. */
    break;
#endif
  case IR_CALLN: case IR_CALLL: case IR_CALLS: case IR_CALLXS:
    if (!uselo)
      ra_allocref(as, ir->op1, RID2RSET(RID_RETLO));  /* Mark lo op as used. */
    break;
  default: lj_assertA(0, "bad HIOP for op %d", (ir-1)->o); break;
  }
}

/* -- Profiling ----------------------------------------------------------- */

static void asm_prof(ASMState *as, IRIns *ir)
{
  UNUSED(ir);
  asm_guardcc(as, CC_NE);
  emit_asi(as, PPCI_ANDIDOT, RID_TMP, RID_TMP, HOOK_PROFILE);
  emit_lsglptr(as, PPCI_LBZ, RID_TMP,
	       (int32_t)offsetof(global_State, hookmask));
}

/* -- Stack handling ------------------------------------------------------ */

/* Check Lua stack size for overflow. Use exit handler as fallback. */
static void asm_stack_check(ASMState *as, BCReg topslot,
			    IRIns *irp, RegSet allow, ExitNo exitno)
{
  /* Try to get an unused temp. register, otherwise spill/restore RID_RET*. */
  Reg tmp, pbase = irp ? (ra_hasreg(irp->r) ? irp->r : RID_TMP) : RID_BASE;
  rset_clear(allow, pbase);
  tmp = allow ? rset_pickbot(allow) :
		(pbase == RID_RETHI ? RID_RETLO : RID_RETHI);
  emit_condbranch(as, PPCI_BC, CC_LT, asm_exitstub_addr(as, exitno));
  if (allow == RSET_EMPTY)  /* Restore temp. register. */
    emit_tai(as, PPCI_LWZ, tmp, RID_SP, SPOFS_TMPW);
  else
    ra_modified(as, tmp);
  emit_ai(as, PPCI_CMPLWI, RID_TMP, (int32_t)(8*topslot));
  emit_tab(as, PPCI_SUBF, RID_TMP, pbase, tmp);
  /* GC64: L->maxstack is a 64-bit TValue*; LWZ would truncate it. */
  emit_tai(as, PPCI_LD, tmp, tmp, offsetof(lua_State, maxstack));
  if (pbase == RID_TMP)
    emit_getgl(as, RID_TMP, jit_base);
  emit_getgl(as, tmp, cur_L);
  if (allow == RSET_EMPTY)  /* Spill temp. register. */
    emit_tai(as, PPCI_STW, tmp, RID_SP, SPOFS_TMPW);
}

/* Restore Lua stack from on-trace state. */
/* GC64: build a TValue from a value reference and store it at base+ofs. */
static void asm_tvstore64(ASMState *as, Reg base, int32_t ofs, IRRef ref)
{
  RegSet allow = rset_exclude(RSET_GPR, base);
  IRIns *ir = IR(ref);
  lj_assertA(irt_ispri(ir->t) || irt_isaddr(ir->t) || irt_isinteger(ir->t),
	     "store of IR type %d", irt_type(ir->t));
  if (irref_isk(ref)) {
    TValue k;
    lj_ir_kvalue(as->J->L, &k, ir);
    emit_tai(as, PPCI_STD, ra_allock(as, (intptr_t)k.u64, allow), base, ofs);
  } else {
    Reg src = ra_alloc1(as, ref, allow);
    rset_clear(allow, src);
    if (irt_isinteger(ir->t)) {
      /* TValue = (itype << 47) | (uint32)value. */
      Reg type = ra_allock(as, (int64_t)irt_toitype(ir->t) << 47, allow);
      emit_tai(as, PPCI_STD, RID_TMP, base, ofs);
      emit_tab(as, PPCI_ADD, RID_TMP, RID_TMP, type);
      emit_rotdi(as, PPCI_RLDICL, RID_TMP, src, 0, 32);  /* clrldi: zero-extend. */
    } else {
      /* TValue = (itype << 47) | gcptr (gcptr < 2^47). */
      Reg type = ra_allock(as, (int64_t)irt_toitype(ir->t) << 47, allow);
      emit_tai(as, PPCI_STD, RID_TMP, base, ofs);
      emit_tab(as, PPCI_ADD, RID_TMP, src, type);
    }
  }
}

static void asm_stack_restore(ASMState *as, SnapShot *snap)
{
  SnapEntry *map = &as->T->snapmap[snap->mapofs];
#ifdef LUA_USE_ASSERT
  SnapEntry *flinks = &as->T->snapmap[snap_nextofs(as->T, snap)-1-LJ_FR2];
#endif
  MSize n, nent = snap->nent;
  /* Store the value of all modified slots to the Lua stack. */
  for (n = 0; n < nent; n++) {
    SnapEntry sn = map[n];
    BCReg s = snap_slot(sn);
    int32_t ofs = 8*((int32_t)s-1-LJ_FR2);
    IRRef ref = snap_ref(sn);
    IRIns *ir = IR(ref);
    if ((sn & SNAP_NORESTORE))
      continue;
    if ((sn & SNAP_KEYINDEX)) {
      RegSet allow = rset_exclude(RSET_GPR, RID_BASE);
      Reg r = irref_isk(ref) ? ra_allock(as, ir->i, allow) :
			       ra_alloc1(as, ref, allow);
      rset_clear(allow, r);
      emit_tai(as, PPCI_STW, r, RID_BASE, ofs+(LJ_BE?4:0));
      emit_tai(as, PPCI_STW, ra_allock(as, LJ_KEYINDEX, allow),
	       RID_BASE, ofs+(LJ_BE?0:4));
    } else if (irt_isnum(ir->t)) {
      Reg src = ra_alloc1(as, ref, RSET_FPR);
      emit_fai(as, PPCI_STFD, src, RID_BASE, ofs);
    } else {
      asm_tvstore64(as, RID_BASE, ofs, ref);
    }
    checkmclim(as);
  }
  lj_assertA(map + nent == flinks, "inconsistent frames in snapshot");
}

/* -- GC handling --------------------------------------------------------- */

/* Marker to prevent patching the GC check exit. */
#define PPC_NOPATCH_GC_CHECK	PPCI_ORIS

/* Check GC threshold and do one or more GC steps. */
static void asm_gc_check(ASMState *as)
{
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_lj_gc_step_jit];
  IRRef args[2];
  MCLabel l_end;
  Reg tmp;
  ra_evictset(as, RSET_SCRATCH);
  l_end = emit_label(as);
  /* Exit trace if in GCSatomic or GCSfinalize. Avoids syncing GC objects. */
  asm_guardcc(as, CC_NE);  /* Assumes asm_snap_prep() already done. */
  *--as->mcp = PPC_NOPATCH_GC_CHECK;
  emit_ai(as, PPCI_CMPWI, RID_RET, 0);
  args[0] = ASMREF_TMP1;  /* global_State *g */
  args[1] = ASMREF_TMP2;  /* MSize steps     */
  asm_gencall(as, ci, args);
  emit_tai(as, PPCI_ADDI, ra_releasetmp(as, ASMREF_TMP1), RID_JGL, -32768);
  tmp = ra_releasetmp(as, ASMREF_TMP2);
  emit_loadi(as, tmp, as->gcsteps);
  /* Jump around GC step if GC total < GC threshold. */
  emit_condbranch(as, PPCI_BC|PPCF_Y, CC_LT, l_end);
  emit_ab(as, PPCI_CMPLW, RID_TMP, tmp);
  emit_getgl_u32(as, tmp, gc.threshold);
  emit_getgl_u32(as, RID_TMP, gc.total);
  as->gcsteps = 0;
  checkmclim(as);
}

/* -- Loop handling ------------------------------------------------------- */

/* Fixup the loop branch. */
static void asm_loop_fixup(ASMState *as)
{
  MCode *p = as->mctop;
  MCode *target = as->mcp;
  if (as->loopinv) {  /* Inverted loop branch? */
    /* asm_guardcc already inverted the cond branch and patched the final b. */
    p[-2] = (p[-2] & (0xffff0000u & ~PPCF_Y)) | (((target-p+2) & 0x3fffu) << 2);
  } else {
    p[-1] = PPCI_B|(((target-p+1)&0x00ffffffu)<<2);
  }
}

/* Fixup the tail of the loop. */
static void asm_loop_tail_fixup(ASMState *as)
{
  UNUSED(as);  /* Nothing to do. */
}

/* -- Head of trace ------------------------------------------------------- */

/* Coalesce BASE register for a root trace. */
static void asm_head_root_base(ASMState *as)
{
  IRIns *ir = IR(REF_BASE);
  Reg r = ir->r;
  if (ra_hasreg(r)) {
    ra_free(as, r);
    if (rset_test(as->modset, r) || irt_ismarked(ir->t))
      ir->r = RID_INIT;  /* No inheritance for modified BASE register. */
    if (r != RID_BASE)
      emit_mr(as, r, RID_BASE);
  }
}

/* Coalesce BASE register for a side trace. */
static Reg asm_head_side_base(ASMState *as, IRIns *irp)
{
  IRIns *ir = IR(REF_BASE);
  Reg r = ir->r;
  if (ra_hasreg(r)) {
    ra_free(as, r);
    if (rset_test(as->modset, r) || irt_ismarked(ir->t))
      ir->r = RID_INIT;  /* No inheritance for modified BASE register. */
    if (irp->r == r) {
      return r;  /* Same BASE register already coalesced. */
    } else if (ra_hasreg(irp->r) && rset_test(as->freeset, irp->r)) {
      emit_mr(as, r, irp->r);  /* Move from coalesced parent reg. */
      return irp->r;
    } else {
      emit_getgl(as, r, jit_base);  /* Otherwise reload BASE. */
    }
  }
  return RID_NONE;
}

/* -- Tail of trace ------------------------------------------------------- */

/* Fixup the tail code. */
static void asm_tail_fixup(ASMState *as, TraceNo lnk)
{
  uintptr_t target = lnk ? (uintptr_t)traceref(as->J, lnk)->mcode : (uintptr_t)(void *)lj_vm_exit_interp;
  MCode *mcp = as->mctail;
  int32_t spadj = as->T->spadjust;
  if (spadj) {  /* Emit stack adjustment. */
    /* GC64/ELFv2: grow with stdu (64-bit back-chain); stwu would write only the
    ** low 32 bits and corrupt the back-chain (fatal on BE). spadj is a multiple
    ** of 4 (sps_scale), so its low 2 bits don't clobber the DS-form XO. */
    lj_assertA(checki16(CFRAME_SIZE+spadj), "stack adjustment out of range");
    *mcp++ = PPCI_ADDI | PPCF_T(RID_TMP) | PPCF_A(RID_SP) | (CFRAME_SIZE+spadj);
    *mcp++ = PPCI_STDU | PPCF_T(RID_TMP) | PPCF_A(RID_SP) | spadj;
  }
  /* Emit exit branch. */
  if ((((target - (uintptr_t)mcp) + 0x02000000u) >> 26) == 0) {
    *mcp = PPCI_B | ((target - (uintptr_t)mcp) & 0x03fffffcu); mcp++;
  } else {
    *mcp++ = PPCI_LWZ | PPCF_T(RID_TMP) | PPCF_A(RID_JGL) |
	     jglofs(as, &as->J->k32[LJ_K32_VM_EXIT_INTERP]);
    *mcp++ = PPCI_MTCTR | PPCF_T(RID_TMP);
    *mcp++ = PPCI_BCTR;
  }
  while (as->mctop > mcp) *--as->mctop = PPCI_NOP;
}

/* Prepare tail of code. */
static void asm_tail_prep(ASMState *as, TraceNo lnk)
{
  MCode *p = as->mctop - 1;  /* Leave room for exit branch. */
  if (as->loopref) {
    as->invmcp = as->mcp = p;
  } else {
    if (!lnk) {
      uintptr_t target = (uintptr_t)(void *)lj_vm_exit_interp;
      if ((((target - (uintptr_t)p) + 0x02000000u) >> 26) ||
	  (((target - (uintptr_t)(p-2)) + 0x02000000u) >> 26)) p -= 2;
    }
    p -= 2;  /* Leave room for stack pointer adjustment. */
    as->mcp = p;
    as->invmcp = NULL;
  }
  as->mctail = p;
}

/* -- Trace setup --------------------------------------------------------- */

/* Ensure there are enough stack slots for call arguments. */
static Reg asm_setup_call_slots(ASMState *as, IRIns *ir, const CCallInfo *ci)
{
  IRRef args[CCI_NARGS_MAX*2];
  uint32_t i, nargs = CCI_XNARGS(ci);
  int nslots = 2, ngpr = REGARG_NUMGPR, nfpr = REGARG_NUMFPR;
  asm_collectargs(as, ir, ci, args);
  for (i = 0; i < nargs; i++)
    if (!LJ_SOFTFP && args[i] && irt_isfp(IR(args[i])->t)) {
      if (nfpr > 0) nfpr--; else nslots = (nslots+3) & ~1;
    } else {
      if (ngpr > 0) ngpr--; else nslots++;
    }
  if (nslots > as->evenspill)  /* Leave room for args in stack slots. */
    as->evenspill = nslots;
  return (!LJ_SOFTFP && irt_isfp(ir->t)) ? REGSP_HINT(RID_FPRET) :
					   REGSP_HINT(RID_RET);
}

static void asm_setup_target(ASMState *as)
{
  asm_exitstub_setup(as, as->T->nsnap + (as->parent ? 1 : 0));
}

/* -- Trace patching ------------------------------------------------------ */

/* Patch exit jumps of existing machine code to a new target. */
void lj_asm_patchexit(jit_State *J, GCtrace *T, ExitNo exitno, MCode *target)
{
  MCode *p = T->mcode;
  MCode *pe = (MCode *)((char *)p + T->szmcode);
  MCode *px = exitstub_trace_addr(T, exitno);
  MCode *cstart = NULL;
  MCode *mcarea = lj_mcode_patch(J, p, 0);
  int clearso = 0, patchlong = 1;
  for (; p < pe; p++) {
    /* Look for exitstub branch, try to replace with branch to target. */
    uint32_t ins = *p;
    if ((ins & 0xfc000000u) == 0x40000000u &&
	((ins ^ ((char *)px-(char *)p)) & 0xffffu) == 0) {
      ptrdiff_t delta = (char *)target - (char *)p;
      if (((ins >> 16) & 3) == (CC_SO&3)) {
	clearso = sizeof(MCode);
	delta -= sizeof(MCode);
      }
      /* Many, but not all short-range branches can be patched directly. */
      if (p[-1] == PPC_NOPATCH_GC_CHECK) {
	patchlong = 0;
      } else if (((delta + 0x8000) >> 16) == 0) {
	*p = (ins & 0xffdf0000u) | ((uint32_t)delta & 0xffffu) |
	     ((delta & 0x8000) * (PPCF_Y/0x8000));
	if (!cstart) cstart = p;
      }
    } else if ((ins & 0xfc000000u) == PPCI_B &&
	       ((ins ^ ((char *)px-(char *)p)) & 0x03ffffffu) == 0) {
      ptrdiff_t delta = (char *)target - (char *)p;
      lj_assertJ(((delta + 0x02000000) >> 26) == 0,
		 "branch target out of range");
      *p = PPCI_B | ((uint32_t)delta & 0x03ffffffu);
      if (!cstart) cstart = p;
    }
  }
  /* Always patch long-range branch in exit stub itself. Except, if we can't. */
  if (patchlong) {
    ptrdiff_t delta = (char *)target - (char *)px - clearso;
    lj_assertJ(((delta + 0x02000000) >> 26) == 0,
	       "branch target out of range");
    *px = PPCI_B | ((uint32_t)delta & 0x03ffffffu);
  }
  if (!cstart) cstart = px;
  lj_mcode_sync(cstart, px+1);
  if (clearso) {  /* Extend the current trace. Ugly workaround. */
    MCode *pp = J->cur.mcode;
    J->cur.szmcode += sizeof(MCode);
    *--pp = PPCI_MCRXR;  /* Clear SO flag. */
    J->cur.mcode = pp;
    lj_mcode_sync(pp, pp+1);
  }
  lj_mcode_patch(J, mcarea, 1);
}

