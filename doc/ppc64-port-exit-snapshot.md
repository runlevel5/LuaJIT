# PPC64 ELFv2 — trace exit, ExitState & snapshot restore (end-to-end)

Companion to the other `ppc64*` docs. Covers the full path a guard takes when a trace
exits, spanning `lj_asm_ppc.h` (stub gen + guard), `lj_target_ppc.h` (`ExitState`,
`exitstub_trace_addr`), `vm_ppc.dasc` (`vm_exit_handler`/`vm_exit_interp`), and the
generic `lj_snap.c` restore. Template = arm64 (`vm_arm64.dasc:2014-2129`,
`lj_asm_arm64.h:51-83`); legacy POWER mechanics from the existing 32-bit ppc code.

## 0. End-to-end flow

```
trace body: ... cmpd/fcmpu ...
            bc  cc, <exitstub[k]>        ; asm_guardcc — branch to per-trace stub k
                                          (loop-inverted: b <stub>; bc cc^4 back)
exitstub[k]: bl   1b                      ; one word per exit; LR = &exitstub[k]+4
   ...
1:           mflr r0                       ; r0 = return addr inside stub array
             bl   ->vm_exit_handler        ; (or indirect via k64 pool if far)
             li   r0, traceno              ; trace number, read back by handler
->vm_exit_handler:
             save all GPR(std)/FPR(stfd) into ExitState on the C stack
             exitno = (LR_of_li - LR_from_stub) >> 2 - 2     ; which guard fired
             traceno = imm of the `li r0,traceno`            ; which trace
             J->exitno/parent/L set; call lj_trace_exit(J, ExitState*)
             ; lj_trace_exit -> lj_snap_restore reads ex->gpr[]/ex->fpr[]/spill[]
             ; rebuilds the Lua stack for the snapshot, returns MULTRES or -errcode
->vm_exit_interp:
             restore SP from L->cframe, reload BASE/PC/KBASE, re-enter dispatch
```

The clever bit (shared with arm64/legacy ppc): **exit number is derived from the
return address**. Each `bl 1b` in the stub array pushes a distinct LR; the handler
subtracts the two LRs to recover `k`. So the stub array costs **one instruction per
exit**, not a full sequence each.

---

## 1. Per-trace exit stubs (`asm_exitstub_setup`)

The legacy 32-bit code is almost right; the only PPC64 changes are the **indirect
trampoline** (use the 64-bit constant pool + `ld`) and keeping displacements valid.

Layout it builds at `mctop` (downward), `ind` chosen when `vm_exit_handler` is out of
`bl`'s ±32MB range:
```
!ind:  1: mflr r0
          bl   ->vm_exit_handler
          li   r0, traceno
          bl <1 ; bl <1 ; ...            (nexits words; index = exit number)
 ind:  1: ld   r0, K64_VXH(JGL)          ; <-- 64-bit: ld (was lwz + K32) 
          mtctr r0
          mflr  r0
          bctrl
          li    r0, traceno
          bl <1 ; bl <1 ; ...
```
Changes vs `lj_asm_ppc.h` (32-bit):
- Indirect load: `PPCI_LWZ` + `LJ_K32_VM_EXIT_HANDLER` → `PPCI_LD` +
  `LJ_K64_VM_EXIT_HANDLER` (add this k64 enum, mirroring arm64's
  `LJ_K64_VM_EXIT_HANDLER`). `jglofs` still gives the ±32K JGL displacement.
- `li r0, traceno` unchanged (16-bit trace number in the immediate field).
- The `bl <1` chain (24-bit rel) unchanged — stubs are adjacent to the trampoline.
- `as->mcexit = mxp` and `asm_exitstub_addr = mcexit + exitno` unchanged.

`asm_guardcc` — **no change**: `emit_condbranch(PPCI_BC, cc, exitstub_addr)`, with the
loop-inversion path (`b` + `bc cc^4`). The `cc` values come from the compares specced
in `ppc64-port-asm.md` §8.

---

## 2. `exitstub_trace_addr_` (`lj_target_ppc.h`) — LE-safe as-is

```c
static LJ_AINLINE uint32_t *exitstub_trace_addr_(uint32_t *p, uint32_t exitno) {
  while (*p == 0x60000000) p++;       /* skip PPCI_NOP padding */
  if (p[3] == 0x4e800421) p += 2;     /* PPCI_BCTRL → indirect variant */
  return p + 3 + exitno;
}
```
Instruction words are read by the CPU in native endianness, so the literal opcodes
`PPCI_NOP`/`PPCI_BCTRL` match on **both** ppc64le and ppc64 — this function needs no
endianness change. Confirm the `+3`/`+2` offsets still line up with the §1 layout
(they do, since the layout is unchanged in word count).

---

## 3. `ExitState` (`lj_target_ppc.h`) — widen to 64-bit

```c
typedef struct {
  lua_Number fpr[RID_NUM_FPR];   /* 32 doubles, 8B each — unchanged */
  intptr_t   gpr[RID_NUM_GPR];   /* 32 GPRs — now 8B each on PPC64 (was 4B) */
  int32_t    spill[256];         /* spill slots — keep 32-bit elements */
} ExitState;
```
`intptr_t` already widens to 8 bytes under `_LP64`, so the struct is correct once the
arch branch (Phase 0) sets `LJ_64`. **But the spill slots are now 8-byte** (see
`lj_target_ppc.h` §2.1 `sps_scale` ×8) — `spill[]` is indexed in 32-bit units by
`snap_restoredata`, so keep `int32_t spill[]` and index `spill[2*slot]` for 64-bit
loads, matching how the assembler writes them. Verify against `snap_restoredata`
(`lj_snap.c:767+`, reads `ex->spill` / `ex->fpr`).

---

## 4. `vm_exit_handler` (`vm_ppc.dasc`) — the main rewrite

Legacy ppc (`vm_ppc.dasc:2969-3018`) uses **`stmw r2`** (store-multiple *word*) and a
`16+32*8+32*4` frame — both 32-bit. PPC64 has **no store-multiple-doubleword**, so:

```
->vm_exit_handler:
.if JIT
  ; frame = 16 (linkage/chain) + 32*8 (fpr) + 32*8 (gpr)   ; all 8-byte now
  stdu  r1, -(FRAME_EXIT)(r1)        ; or addi+std chain to preserve back-chain
  std   r0, GPRSAVE+0*8(r1)          ; save r0..r31 individually (no stmd)
  std   r2, GPRSAVE+2*8(r1)
  ...                                ; r3..r31
  stfd  f0, FPRSAVE+0*8(r1)          ; save f0..f31
  ...
  addi  DISPATCH, JGL, -GG_DISP2G-32768
  li    CARG2, ~LJ_VMST_EXIT
  stw   CARG2, DISPATCH_GL(vmstate)(DISPATCH)
  addi  CARG2, r1, FRAME_EXIT        ; recompute caller SP
  std   CARG2, GPRSAVE+RID_SP*8(r1)  ; store SP into ExitState slot
  li    TMP, 0; std TMP, GPRSAVE+RID_TMP*8(r1)   ; clear RID_TMP slot
  mflr  CARG3                        ; CARG3 = &li(traceno)+? (return addr)
  ; --- trace number (endian-aware) ---
  lwz   CARG4, 0(CARG3)              ; load the `li r0,traceno` word
.if ENDIAN_BE
  ; (BE) immediate already in low half at byte 2; mask low 16 bits
.endif
  clrldi CARG4, CARG4, 48            ; traceno = imm16 (low 16 bits of li)
  ; --- exit number from LR delta (TMP0 = stub base addr, set in stub) ---
  sub   CARG3, TMP0, CARG3
  srdi  CARG3, CARG3, 2
  subi  CARG3, CARG3, 2
  ld    L, DISPATCH_GL(cur_L)(DISPATCH)
  ld    BASE, DISPATCH_GL(jit_base)(DISPATCH)
  std   L, DISPATCH_J(L)(DISPATCH)
  std   BASE, L->base
  stw   CARG4, DISPATCH_J(parent)(DISPATCH)
  li    TMP, 0; stw TMP, DISPATCH_GL(jit_base)(DISPATCH)
  addi  CARG1, DISPATCH, GG_DISP2J       ; CARG1 = J
  stw   CARG3, DISPATCH_J(exitno)(DISPATCH)
  addi  CARG2, r1, GPRSAVE_OR_FPRSAVE_BASE  ; CARG2 = ExitState*  (= &fpr[0])
  bl    extern lj_trace_exit             ; (jit_State *J, ExitState *ex)
```
Key points / pitfalls:
- **Register save uses 32× `std` + 32× `stfd`** (no `stmw`/`stmd`). Order so the
  ExitState `fpr[]` then `gpr[]` layout is contiguous and `CARG2` points at `&fpr[0]`.
- **`RID_SP`/`RID_TMP` slots** in `gpr[]` must be filled (SP recomputed; TMP zeroed) —
  the legacy code does this; preserve it.
- **Trace-number extraction is endian-sensitive.** The legacy BE code does
  `lhz CARG4, 2(CARG3)` (halfword at byte offset 2 = low 16 of a BE word). On **LE**
  the `li` immediate's low 16 bits sit at byte offset 0, and a 32-bit `lwz`+`clrldi`
  (mask low 16) is endian-neutral — prefer that form. Mirror arm64, which loads the
  full word, byte-reverses on BE, then `ubfx`.
- **ELFv2 prologue/back-chain**: this is a called routine (via `bl`/`bctrl`), so honor
  ELFv2 (save LR to caller frame *before* `stdu`; we need LR via `mflr` to compute the
  exit number, so capture it first). Use the red zone carefully or a real frame.
- **`clrso`**: legacy clears XER[SO] after entry. With the XER-free overflow scheme
  (`ppc64-port-asm.md` §7) guards no longer set SO, but keep `clrso` only if some path
  still uses `addo.`; otherwise it can be dropped. Note: `mcrxr` is illegal on 2.07+ —
  if SO-clear is still wanted, use `mtspr XER` or `mcrxrx` (3.0), not `mcrxr`.

`lj_trace_exit` returns MULTRES (unscaled) or a negated error code in `CARG1/r3`.

---

## 5. `vm_exit_interp` (`vm_ppc.dasc`) — resume the interpreter (GC64/FR2)

Restores execution after `lj_trace_exit` rebuilt the stack. PPC64 changes are the
GC64/FR2 ones, not exit-specific:
- `ld L, SAVE_L`; restore `DISPATCH` from JGL; `std BASE, L->base`.
- Error check: `cmpld CARG1, -LUA_ERRERR` → branch to error path.
- **FR2 frame func load**: `ld LFUNC, FRAME_FUNC(BASE)` then `rldicl LFUNC, LFUNC,
  0, 17` (mask to 47-bit pointer — replaces the 32-bit `lwz`+no-mask). `FRAME_FUNC`
  is `-16` under FR2 (2-slot), per `ppc64-port-interp-abi.md` §2/§3.
- Reload `KBASE` from `proto->k`, set vmstate to INTERP, and run the modified
  `ins_NEXT` that also handles function-header / fast-function / static-dispatch
  (`BC_JLOOP`) cases — straight port of arm64 `vm_arm64.dasc:2066-2129` with PPC
  decode macros (`decode_OPP`, `decode_RA8`, …) and `bctr` dispatch.
- Restore SP from `L->cframe` with the 64-bit mask: `rldicr sp, TMP, 0, 61` (the
  `GPR64` branch the legacy code already has at `:3011`) — keep that, drop the 32-bit
  `rlwinm` branch for this target.

---

## 6. Snapshot restore (`lj_snap.c`) — generic, minimal PPC impact

`lj_snap_restore` → `snap_restoreval`/`snap_restoredata` read the saved registers:
```c
setintV(o, (int32_t)ex->gpr[r-RID_MIN_GPR]);   // int from GPR (low 32)
setnumV(o, ex->fpr[r-RID_MIN_FPR]);            // double from FPR
o->u64 = ex->gpr[r-RID_MIN_GPR];               // 64-bit GC64 tagged value
setgcV(..., (GCobj *)ex->gpr[...], irt_toitype(t));
```
This is architecture-neutral and already GC64-aware (it stores full `u64`). The only
requirements PPC64 must meet:
- `ex->gpr[]` elements are 64-bit (§3) so a tagged value / pointer round-trips intact.
- Spill restore (`snap_restoredata`, `ex->spill`) indices match the assembler's
  8-byte spill layout — verify the index scaling.
- FP register values stored as raw doubles by `stfd` in §4.
No PPC-specific edits to `lj_snap.c` are expected; if any appear, they indicate a
layout mismatch in §3/§4 to fix there, not in the generic code.

---

## 7. Validation

- **Exit-number/trace-number correctness**: a trace with several guards; force each to
  fail and assert the right snapshot is restored. Test on **both** ppc64le and ppc64
  (BE) — the trace-number extraction (§4) is the LE/BE-divergent spot.
- **ExitState integrity**: a trace using many GPRs/FPRs + spills; verify all live
  values restore (compare `-jon` result to `-joff`).
- **Indirect trampoline**: force `vm_exit_handler` out of ±32MB (large mcode) to
  exercise the `ind` path + `LJ_K64_VM_EXIT_HANDLER`.
- **Stack unwinding from exit**: error thrown during exit (`cmpld` error path) must
  unwind cleanly with the ELFv2 frame.
