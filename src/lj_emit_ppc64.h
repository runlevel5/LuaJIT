/*
** PPC instruction emitter.
** Copyright (C) 2005-2026 Mike Pall. See Copyright Notice in luajit.h
*/

/* -- Emit basic instructions --------------------------------------------- */

static void emit_tab(ASMState *as, PPCIns pi, Reg rt, Reg ra, Reg rb)
{
  *--as->mcp = pi | PPCF_T(rt) | PPCF_A(ra) | PPCF_B(rb);
}

#define emit_asb(as, pi, ra, rs, rb)	emit_tab(as, (pi), (rs), (ra), (rb))
#define emit_as(as, pi, ra, rs)		emit_tab(as, (pi), (rs), (ra), 0)
#define emit_ab(as, pi, ra, rb)		emit_tab(as, (pi), 0, (ra), (rb))

static void emit_tai(ASMState *as, PPCIns pi, Reg rt, Reg ra, int32_t i)
{
  *--as->mcp = pi | PPCF_T(rt) | PPCF_A(ra) | (i & 0xffff);
}

#define emit_ti(as, pi, rt, i)		emit_tai(as, (pi), (rt), 0, (i))
#define emit_ai(as, pi, ra, i)		emit_tai(as, (pi), 0, (ra), (i))
#define emit_asi(as, pi, ra, rs, i)	emit_tai(as, (pi), (rs), (ra), (i))

#define emit_fab(as, pi, rf, ra, rb) \
  emit_tab(as, (pi), (rf)&31, (ra)&31, (rb)&31)
#define emit_fb(as, pi, rf, rb)		emit_tab(as, (pi), (rf)&31, 0, (rb)&31)
#define emit_fac(as, pi, rf, ra, rc) \
  emit_tab(as, (pi) | PPCF_C((rc) & 31), (rf)&31, (ra)&31, 0)
#define emit_facb(as, pi, rf, ra, rc, rb) \
  emit_tab(as, (pi) | PPCF_C((rc) & 31), (rf)&31, (ra)&31, (rb)&31)
#define emit_fai(as, pi, rf, ra, i)	emit_tai(as, (pi), (rf)&31, (ra), (i))

static void emit_rot(ASMState *as, PPCIns pi, Reg ra, Reg rs,
		     int32_t n, int32_t b, int32_t e)
{
  *--as->mcp = pi | PPCF_T(rs) | PPCF_A(ra) | PPCF_B(n) |
	       PPCF_MB(b) | PPCF_ME(e);
}

static void emit_slwi(ASMState *as, Reg ra, Reg rs, int32_t n)
{
  lj_assertA(n >= 0 && n < 32, "shift out or range");
  emit_rot(as, PPCI_RLWINM, ra, rs, n, 0, 31-n);
}

static void emit_rotlwi(ASMState *as, Reg ra, Reg rs, int32_t n)
{
  lj_assertA(n >= 0 && n < 32, "shift out or range");
  emit_rot(as, PPCI_RLWINM, ra, rs, n, 0, 31);
}

#if LJ_64
/* Emit a 64-bit rotate-and-mask (MD-form: rldicl/rldicr/rldic/rldimi).
** The 6-bit shift and mask fields use the split MD-form encoding.
*/
static void emit_rotdi(ASMState *as, PPCIns pi, Reg ra, Reg rs,
		       int32_t sh, int32_t mbe)
{
  lj_assertA(sh >= 0 && sh < 64, "shift out of range");
  lj_assertA(mbe >= 0 && mbe < 64, "mask bit out of range");
  *--as->mcp = pi | PPCF_T(rs) | PPCF_A(ra) |
	       (((sh & 0x1f) << 11) | ((sh & 0x20) >> 4)) |
	       (((mbe & 0x1f) << 6) | (mbe & 0x20));
}

static void emit_sldi(ASMState *as, Reg ra, Reg rs, int32_t n)
{
  emit_rotdi(as, PPCI_RLDICR, ra, rs, n, 63-n);  /* sldi = rldicr ra,rs,n,63-n */
}

/* sradi rA,rS,sh: arithmetic shift right doubleword immediate (XS-form). */
static void emit_sradi(ASMState *as, Reg ra, Reg rs, int32_t sh)
{
  lj_assertA(sh >= 0 && sh < 64, "shift out of range");
  *--as->mcp = PPCI_SRADI | PPCF_T(rs) | PPCF_A(ra) |
	       (((sh & 0x1f) << 11) | ((sh & 0x20) >> 4));
}
#endif

/* -- Emit loads/stores --------------------------------------------------- */

#define jglofs(as, k) \
  (((uintptr_t)(k) - (uintptr_t)J2G(as->J) - 32768) & 0xffff)

/* Prefer rematerialization of BASE/L from global_State over spills. */
#define emit_canremat(ref)	((ref) <= REF_BASE)

/* Try to find a one step delta relative to another constant. */
static int emit_kdelta1(ASMState *as, Reg rd, int32_t i)
{
  RegSet work = ~as->freeset & RSET_GPR;
  while (work) {
    Reg r = rset_picktop(work);
    IRRef ref = regcost_ref(as->cost[r]);
    lj_assertA(r != rd, "dest reg %d not free", rd);
    if (ref < ASMREF_L) {
      int32_t delta = i - (ra_iskref(ref) ? ra_krefk(as, ref) : IR(ref)->i);
      if (checki16(delta)) {
	emit_tai(as, PPCI_ADDI, rd, r, delta);
	return 1;
      }
    }
    rset_clear(work, r);
  }
  return 0;  /* Failed. */
}

/* Load a 32 bit constant into a GPR. */
static void emit_loadi(ASMState *as, Reg r, int32_t i)
{
  if (checki16(i)) {
    emit_ti(as, PPCI_LI, r, i);
  } else {
    if ((i & 0xffff)) {
      int32_t jgl = i32ptr(J2G(as->J));
      if ((uint32_t)(i-jgl) < 65536) {
	emit_tai(as, PPCI_ADDI, r, RID_JGL, i-jgl-32768);
	return;
      } else if (emit_kdelta1(as, r, i)) {
	return;
      }
      emit_asi(as, PPCI_ORI, r, r, i);
    }
    emit_ti(as, PPCI_LIS, r, (i >> 16));
  }
}

#if LJ_64
/* Load a 64 bit constant into a GPR. */
static void emit_loadu64(ASMState *as, Reg r, uint64_t u64)
{
  if (checki32((int64_t)u64)) {
    emit_loadi(as, r, (int32_t)u64);  /* Sign-extends to 64 bit. */
  } else {
    uint64_t delta = u64 - (uint64_t)(void *)J2G(as->J);
    if (delta < 65536) {  /* Use addi from JGL anchor. */
      emit_tai(as, PPCI_ADDI, r, RID_JGL, (int32_t)(delta-32768));
    } else {  /* General case: (hi32 << 32) | lo32. Emitted in reverse. */
      uint32_t lo = (uint32_t)u64;
      if (lo & 0xffff)
	emit_asi(as, PPCI_ORI, r, r, lo & 0xffff);
      if (lo & 0xffff0000u)
	emit_asi(as, PPCI_ORIS, r, r, (lo >> 16) & 0xffff);
      emit_sldi(as, r, r, 32);
      emit_loadi(as, r, (int32_t)(u64 >> 32));  /* hi32; high bits masked off. */
    }
  }
}
#define emit_loada(as, r, addr)	emit_loadu64(as, (r), (uint64_t)(uintptr_t)(addr))
#else
#define emit_loada(as, r, addr)		emit_loadi(as, (r), i32ptr((addr)))
#endif

static Reg ra_allock(ASMState *as, intptr_t k, RegSet allow);
static void ra_allockreg(ASMState *as, intptr_t k, Reg r);

/* Get/set from constant pointer. */
static void emit_lsptr(ASMState *as, PPCIns pi, Reg r, void *p, RegSet allow)
{
  uintptr_t pp = (uintptr_t)p;
  uintptr_t jgl = (uintptr_t)J2G(as->J);
  int32_t i;
  Reg base;
  if ((uintptr_t)(pp-jgl) < 65536) {  /* Within reach of the JGL anchor. */
    i = (int32_t)(pp-jgl-32768);
    base = RID_JGL;
  } else {
    /* GC64: materialize the full 64-bit base; keep the signed 16-bit displacement
    ** in the load itself (base holds p with the low 16 bits cleared/aligned). */
    i = (int16_t)(int32_t)pp;
    base = ra_allock(as, (intptr_t)(pp - (uintptr_t)(intptr_t)i), allow);
  }
  emit_tai(as, pi, r, base, i);
}

#define emit_loadk64(as, r, ir) \
  emit_lsptr(as, PPCI_LFD, ((r) & 31), (void *)&ir_knum((ir))->u64, RSET_GPR)

/* Get/set global_State fields. */
static void emit_lsglptr(ASMState *as, PPCIns pi, Reg r, int32_t ofs)
{
  emit_tai(as, pi, r, RID_JGL, ofs-32768);
}

/* GC64: get/set global_State fields. Pointer-sized (64-bit) by default, to match
** the shared lj_asm.c contract (it uses these for jit_base/cur_L). Use the _u32
** variants for 32-bit fields (MSize, uint32 tags).
*/
#define emit_getgl(as, r, field) \
  emit_lsglptr(as, PPCI_LD, (r), (int32_t)offsetof(global_State, field))
#define emit_setgl(as, r, field) \
  emit_lsglptr(as, PPCI_STD, (r), (int32_t)offsetof(global_State, field))
#define emit_getgl_u32(as, r, field) \
  emit_lsglptr(as, PPCI_LWZ, (r), (int32_t)offsetof(global_State, field))
#define emit_setgl_u32(as, r, field) \
  emit_lsglptr(as, PPCI_STW, (r), (int32_t)offsetof(global_State, field))

/* Trace number is determined from per-trace exit stubs. */
#define emit_setvmstate(as, i)		UNUSED(i)

/* -- Emit control-flow instructions -------------------------------------- */

/* Label for internal jumps. */
typedef MCode *MCLabel;

/* Return label pointing to current PC. */
#define emit_label(as)		((as)->mcp)

static void emit_condbranch(ASMState *as, PPCIns pi, PPCCC cc, MCode *target)
{
  MCode *p = --as->mcp;
  ptrdiff_t delta = (char *)target - (char *)p;
  lj_assertA(((delta + 0x8000) >> 16) == 0, "branch target out of range");
  pi ^= (delta & 0x8000) * (PPCF_Y/0x8000);
  *p = pi | PPCF_CC(cc) | ((uint32_t)delta & 0xffffu);
}

static void emit_jmp(ASMState *as, MCode *target)
{
  MCode *p = --as->mcp;
  ptrdiff_t delta = (char *)target - (char *)p;
  *p = PPCI_B | (delta & 0x03fffffcu);
}

/* ELFv2 TOC save slot in the current (interpreter) stack frame. The trace runs
** on the interpreter's frame; 24(r1) is the ABI-reserved TOC-save doubleword and
** is not used by trace mcode (FP bounces use SPOFS_TMP=8). */
#define PPC_TOC_SAVE_OFS	24

static void emit_call(ASMState *as, void *target)
{
  /* ELFv2 trace->C calls go indirect via r12 (the callee's global entry
  ** recomputes its own r2/TOC from r12). The target may live in another module
  ** (e.g. libm floor), whose TOC differs from ours, and the ELFv2 callee does
  ** NOT restore the caller's r2 -- so we must save/restore r2 (RID_SYS1 = our
  ** reserved TOC) around the call ourselves. A plain in-range `bl` would reach
  ** the callee's LOCAL entry assuming r2 is already its TOC, which is only true
  ** for same-module calls and cannot be linker-patched in runtime mcode; so use
  ** the indirect global-entry path unconditionally.
  ** Emitted in reverse: r12-load; std r2; mtctr r12; bctrl; ld r2. */
  emit_tai(as, PPCI_LD, RID_SYS1, RID_SP, PPC_TOC_SAVE_OFS);  /* restore r2. */
  *--as->mcp = PPCI_BCTRL;
  *--as->mcp = PPCI_MTCTR | PPCF_T(RID_R12);
  emit_tai(as, PPCI_STD, RID_SYS1, RID_SP, PPC_TOC_SAVE_OFS);  /* save r2. */
  ra_allockreg(as, i64ptr(target), RID_R12);
}

/* -- Emit generic operations --------------------------------------------- */

#define emit_mr(as, dst, src) \
  emit_asb(as, PPCI_MR, (dst), (src), (src))

/* Generic move between two regs. */
static void emit_movrr(ASMState *as, IRIns *ir, Reg dst, Reg src)
{
  UNUSED(ir);
  if (dst < RID_MAX_GPR)
    emit_mr(as, dst, src);
  else
    emit_fb(as, PPCI_FMR, dst, src);
}

/* Generic load of register with base and (small) offset address. */
static void emit_loadofs(ASMState *as, IRIns *ir, Reg r, Reg base, int32_t ofs)
{
  if (r < RID_MAX_GPR)
    emit_tai(as, irt_is64(ir->t) ? PPCI_LD : PPCI_LWZ, r, base, ofs);
  else
    emit_fai(as, irt_isnum(ir->t) ? PPCI_LFD : PPCI_LFS, r, base, ofs);
}

/* Generic store of register with base and (small) offset address. */
static void emit_storeofs(ASMState *as, IRIns *ir, Reg r, Reg base, int32_t ofs)
{
  if (r < RID_MAX_GPR)
    emit_tai(as, irt_is64(ir->t) ? PPCI_STD : PPCI_STW, r, base, ofs);
  else
    emit_fai(as, irt_isnum(ir->t) ? PPCI_STFD : PPCI_STFS, r, base, ofs);
}

/* Emit a compare (for equality) with a constant operand. */
static void emit_cmpi(ASMState *as, Reg r, int32_t k)
{
  if (checki16(k)) {
    emit_ai(as, PPCI_CMPWI, r, k);
  } else if (checku16(k)) {
    emit_ai(as, PPCI_CMPLWI, r, k);
  } else {
    emit_ai(as, PPCI_CMPLWI, RID_TMP, k);
    emit_asi(as, PPCI_XORIS, RID_TMP, r, (k >> 16));
  }
}

/* Add offset to pointer. */
static void emit_addptr(ASMState *as, Reg r, int32_t ofs)
{
  if (ofs) {
    emit_tai(as, PPCI_ADDI, r, r, ofs);
    if (!checki16(ofs))
      emit_tai(as, PPCI_ADDIS, r, r, (ofs + 32768) >> 16);
  }
}

static void emit_spsub(ASMState *as, int32_t ofs)
{
  if (ofs) {
    emit_tai(as, PPCI_STWU, RID_TMP, RID_SP, -ofs);
    emit_tai(as, PPCI_ADDI, RID_TMP, RID_SP,
	     CFRAME_SIZE + (as->parent ? as->parent->spadjust : 0));
  }
}

