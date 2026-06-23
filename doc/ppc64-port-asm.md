# PPC64 ELFv2 — JIT assembler deep dive (`lj_asm_ppc.h` / `lj_emit_ppc.h`)

Companion to `ppc64le-port.md` and `ppc64-port-interp-abi.md`. Handler-by-handler
spec for Phase 2–3 of the JIT backend. Synthesizes the **arm64 GC64 algorithm**
(structural template) with **POWER instruction selection** (from the legacy 32-bit
`lj_asm_ppc.h` + `dasm_ppc.lua`). Asm sketches are illustrative, to be finalized.

## GC64 representation (the invariants every handler obeys)

- **Tagged value:** `(itype << 47) | payload`. `itype` is the negative-of-tag for GC
  refs; numbers are raw IEEE doubles; integers are `(LJ_TISNUM << 47) | (uint32)v`.
- **Pointer extract:** `pheld = val & LJ_GCVMASK` where `GCVMASK = (1<<47)-1`.
  → PPC: `rldicl rd, rs, 0, 17` (clear top 17 bits, keep low 47).
- **Tag extract (for addr type check):** arm64 `asr x,#47`.
  → PPC: `sradi rd, rs, 47`, then `cmpdi rd, itype`.
- **Int/num discrimination:** arm64 `cmp TISNUMhi, x, lsr #32`.
  → PPC: `srdi tmp, rs, 32` (= `rldicl tmp,rs,32,32`); `cmplw TISNUMhi, tmp`
  (number if `tmp < TISNUMhi`, int if `==`), guard accordingly.
- **Guard:** PPC keeps the existing `asm_guardcc` → `emit_condbranch(PPCI_BC, cc, exit)`
  on a CR0 bit (`PPCCC` enum + `PPCF_CC`). Loop inversion flips `cc ^= 4`. **No change
  to the guard mechanism** — it already works; only the compare-producing code changes.

---

## 1. Emit layer (`lj_emit_ppc.h`) — 64-bit foundation

The existing emitter is 32-bit. Required changes/additions (keep the JGL ±32K trick,
`emit_kdelta1`, `emit_condbranch`, `emit_jmp` largely as-is):

| Existing (32-bit) | PPC64 change |
|---|---|
| `emit_loadi` (LI/LIS/ORI, 32-bit) | add `emit_loadi64`: build 64-bit const — `lis/ori` low32, `rldicr`+`oris/ori` for high32; tiered (see §12) |
| `emit_loadofs`/`storeofs` (LWZ/STW) | LD/STD for GPR; LFD/LFS unchanged for FPR |
| `emit_cmpi` (CMPWI/CMPLWI) | add 64-bit `cmpdi`/`cmpldi` variant |
| `emit_addptr` (ADDI/ADDIS) | unchanged (works on 64-bit regs) |
| `emit_loadk64` (LFD from k64) | keep for FP consts; **add GPR path** for tagged-value consts via `ra_allock` |
| `emit_slwi`/`emit_rotlwi` (RLWINM) | add `emit_sldi`/`emit_rldicl/r` helpers (64-bit rotate-mask) |
| `emit_call` (BL ±32MB, else MTCTR+BCTRL via 32-bit `ra_allock`) | ELFv2: 64-bit target; far call via `mtctr`+`bctrl`; reload `r2` after indirect inter-module calls |
| `emit_spsub` (STWU) | STDU; 64-bit frame |

New emitters needed: `emit_dnm`-style wrappers for `mulld`, `sld/srd/srad`, `extsw`,
`rldicl/rldicr/rldic`, `isel`, `fcfid(s)`/`fctid(z)`/`fctiw(z)`, `mtvsrd`/`mfvsrd`.

---

## 2. Address fusion (`asm_fusexref`, `asm_fuseahuref`) + load/store ins tables

PPC addressing = **D-form** `disp16(ra)` or **X-form** `ra+rb` (indexed). There is no
scaled-index/extend mode (unlike arm64), so scaling must be materialized.

**Fusion rules:**
- `IR_ADD` with const op2 fitting `int16` → D-form, fold offset into `disp`.
- `IR_ADD` reg+reg → X-form (`LDX`/`STDX`/…); if one operand is `IR_BSHL #k`, the
  shift must be emitted (`sldi`) since there's no addressing-mode shift.
- Offset exceeds `int16` → materialize into a temp reg, use X-form (legacy backend
  already does this; reuse the pattern).
- `IR_STRREF`: fold `sizeof(GCstr)` into disp when it fits, else X-form.

**Load/store opcode tables** (extend legacy `asm_fxloadins`/`asm_fxstoreins`):

| IRT | load (D / X) | store (D / X) | post |
|---|---|---|---|
| I8 | LBZ / LBZX | STB / STBX | `extsb` after load |
| U8 | LBZ / LBZX | STB / STBX | — |
| I16 | LHA / LHAX | STH / STHX | hw sign-extend |
| U16 | LHZ / LHZX | STH / STHX | — |
| INT (32) | LWZ / LWZX | STW / STWX | `extsw` if used as 64-bit |
| U32 | LWZ / LWZX | STW / STWX | zero-extended |
| **PTR/64/GC** | **LD / LDX** | **STD / STDX** | the GC64 change |
| FLOAT | LFS / LFSX | STFS / STFSX | — |
| NUM | LFD / LFDX | STFD / STFDX | — |

---

## 3. GC64 reference handlers

- **`asm_aref`** (array ref): const index → `addi dest, base, abase+8*k` (D-form if
  fits, else two-step). Dynamic → `sldi tmp, idx, 3; add dest, base, tmp` (PPC has no
  `add+lsl`; the legacy backend already does the explicit shift).
- **`asm_href`** (hash lookup): port arm64's loop. Canonicalize key into a tagged
  64-bit value in a reg (`ra_allock` of `(itype<<47)|kgc`, or raw `u64` for num,
  or inverted for pri). Hash → node index → chain loop comparing `node->key.u64`
  (`ld`+`cmpd`) and `node->next`. Number keys: `mfvsrd` the FP key to GPR (not stack),
  then compare. String keys: load `str->sid`/`hash`, mix.
- **`asm_hrefk`** (const key, precomputed slot): `addi`/two-step to `node + k*sizeof(Node)`,
  `ld tmp, node->key`, `cmpd` against the materialized tagged key, `asm_guardcc(CC_NE)`.
- **`asm_uref`**: const+unguarded → `emit_lsptr(LD,...)` of `uvptr[i]`. Guarded →
  `lbz` the `closed` flag + guard; UREFC adds `offsetof(tv)`, UREFO loads `->v`.
- **`asm_fref`**: no-op (always fused into FLOAD/FSTORE) — assert unused.
- **`asm_strref`**: `addi dest, base, sizeof(GCstr)+k` (D-form) or X-form with the
  index; fuse a const `IR_ADD` offset.

---

## 4. Field load/store (`asm_fload`, `asm_fstore`)

- `asm_fload`: `op1==REF_NIL` → load from `global_State` via JGL anchor with computed
  offset (reuse `emit_lsglptr`). Else `ld`/typed load at `field_ofs[op2]`. Special:
  `IRFL_TAB_ARRAY` colocated-array fuses to `addi` (no load).
- `asm_fstore`: skip if `RID_SINK`; typed store at `field_ofs`. Straight port.

---

## 5. Typed loads/stores (`asm_ahuvload`, `asm_ahustore`, `asm_sload`) — GC64 core

This is where GC64 type checks live. Pattern (port of arm64 lines 1077-1245):

**`asm_ahuvload` / `asm_sload` (load + typecheck):**
```
ld   tmp, [base, ofs]            ; load raw tagged 64-bit value (X-form if indexed)
; --- type check (skip if no IRSLOAD_TYPECHECK) ---
if addr type:
  rldicl dest, tmp, 0, 17        ; dest = pointer (mask low 47)
  sradi  RID_TMP, tmp, 47        ; tag = itype
  cmpdi  RID_TMP, -itype         ; CMN-equivalent
  asm_guardcc(CC_NE)
elif int/num:
  srdi   RID_TMP, tmp, 32
  cmplw  TISNUMhi, RID_TMP       ; num: < ; int: ==
  asm_guardcc(num ? CC_LT-style : CC_NE)
  if num:  mtvsrd dest_fpr, tmp  ; move bits to FPR (NO stack bounce)
  if int:  (low 32 already in tmp; extsw/clrldi as needed)
elif nil/pri:
  cmpdi  tmp, <inverted tag>     ; or compare against materialized pri const
  asm_guardcc(CC_NE)
```
Key PPC win: number values move GPR→FPR via **`mtvsrd`** (and FPR→GPR via `mfvsrd`),
replacing the legacy `std`/`lfd` stack bounce (quality bar #1).

**`asm_ahustore` (build TValue + store):**
```
int:   ra_allock type = (TISNUM<<47);  rldicl t2, src, 0, 32 (clrldi, zero-ext 32);
       or  t2, t2, type   (or: add) ;  std t2, [base,ofs]
gcref: ra_allock type = itype;        sldi t2, type, 47;  or t2, t2, srcptr;
       std t2, [base,ofs]
num:   stfd src_fpr, [base,ofs]
pri:   ra_allock const = ~(~itype<<47); std const, [base,ofs]
```

`IRSLOAD_CONVERT` (num→int slot): use `asm_tointg` (§6).
`LJ_BE` int load reads the high word (`ofs^4`) exactly as arm64 — keep parameterized.

---

## 6. Conversions (`asm_conv`, `asm_tobit`, `asm_tointg`) — VSX, no stack bounce

**Replace the legacy magic-constant stack-bounce entirely.** PPC64 always has VSX
(ISA 2.06+/2.07 baseline):

- **int→double** (`IRT_INT`→`NUM`): `extsw tmp, src` (sign-ext 32→64) `; mtvsrd f, tmp ; fcfid f, f`.
  Unsigned: `clrldi tmp,src,32 ; mtvsrd f,tmp ; fcfidu f,f` (fcfidu = ISA 2.06).
- **int→float**: as above with `fcfids`.
- **double→int** (`fctiwz`) / **double→i64** (`fctidz`): `fctiwz f,src ; mfvsrd tmp,f`
  (or stfiwx for the low word — but `mfvsrd`+extract avoids memory).
- **float↔double**: `frsp` (→single); single→double is implicit in FPR.
- **i32↔i64**: `extsw` (sign) / `clrldi rd,rs,32` (zero) / `ra_leftov` (no-op).
- **`asm_tobit`** (num→bit): `fadd ftmp,left,right ; mfvsrd dest,ftmp` then take low 32.
- **`asm_tointg`** (checked num→int): `fctiwz ftmp,left ; mfvsrd dest,ftmp` (low word);
  back-convert `fcfid ftmp2 ; fcmpu ftmp2,left ; asm_guardcc(CC_NE)` — exit if the
  round-trip differs (non-integral).

---

## 7. Integer arithmetic + overflow (`asm_intop`, `asm_add/sub`, `asm_intmul`, `asm_intneg`)

Plain ops: `add`/`subf`(note operand order: `subf d,b,a` = a−b)/`mulld`/`neg`, with
immediate forms `addi`/`addis`/`subfic`/`mulli`. Reuse legacy selection; promote to
64-bit opcodes where `irt_is64`. AND/OR/XOR → §9.

**Overflow (`ADDOV`/`SUBOV`/`MULOV`) — avoid XER/`mcrxr`.** Important POWER64 wrinkle:
`mcrxr` is **illegal on ISA 2.07+**, so the legacy `ADDO/SUBFO + mcrxr` path must NOT
be ported. Instead detect overflow via 64-bit math on 32-bit values (these IR ops are
`IRT_INT`), mirroring arm64's `SMULL`+compare:
```
ADDOV: extsw la,opA ; extsw lb,opB ; add d64,la,lb
       extsw chk,d64 ; cmpd chk,d64 ; asm_guardcc(CC_NE)   ; result = low 32 of d64
SUBOV: symmetric with subf
MULOV: mulld d64,la,lb ; extsw chk,d64 ; cmpd chk,d64 ; asm_guardcc(CC_NE)
```
(Operands are already 32-bit; if known sign-extended in-reg, skip the `extsw`.)
Optionally, on ISA 3.0+, `mcrxrx` is available for a flag-based fast path — gate it,
but the sign-compare form is correct everywhere and is the recommended default.

`asm_intop_s` flag-fusion (`as->flagmcp`) and `addic.`-style dot forms can be kept for
the compare-with-zero case, but prefer the explicit form above for OV guards.

---

## 8. Comparisons (`asm_intcomp`, `asm_fpcomp`, `asm_comp`)

- Map IR op → `PPCCC` via the existing `asm_compmap`.
- Integer: `cmpw`/`cmplw` (reg) or `cmpwi`/`cmplwi` (imm16), 64-bit forms `cmpd`/
  `cmpld`/`cmpdi`/`cmpldi` when `irt_is64`. Signed vs unsigned from `CC_UNSIGNED`.
  Swap operands to put const on the right; adjust `cc` (`^7` for GE/LE family, `^11`
  for unsigned family) — same algebra as arm64.
- Compare-with-zero: PPC has no `CBZ`/`TBZ`; use `cmpwi rX,0` + guard, or fuse a
  preceding `IR_BAND` via `and. ` (dot form sets CR0) → guard on CR0[EQ].
- FP: `fcmpu` sets CR; map ordered/unordered conditions; guard. (No `FCMPZ` — compare
  against an FPR holding 0.)
- `asm_comp` dispatches num→`asm_fpcomp`, else `asm_intcomp`.

---

## 9. Bit ops & shifts (`asm_band`, `asm_bor/bxor`, `asm_bnot`, `asm_bswap`, `asm_bitshift`)

- **Shifts (variable):** `slw/srw/sraw` (32-bit), `sld/srd/srad` (64-bit).
- **Shifts (const):** `rlwinm` (32) / `rldicl`/`rldicr`/`rldic` (64) with computed
  MB/ME (or SH/ME for the doubleword forms). Add `emit_rldic*` helpers.
- **BAND with contiguous-bit mask:** fuse to a single `rlwinm`/`rldic` mask (legacy
  `asm_fuseandsh` pattern); else `andi.`/`andis.` (imm16) or `and` (reg).
- **BOR/BXOR:** `ori`/`oris`/`xori`/`xoris` (imm) or `or`/`xor` (reg).
- **BNOT:** `nor rd,rs,rs`.
- **BSWAP:** fuse `BSWAP+XLOAD` → `lwbrx`/`ldbrx` (byte-reversed load; `ldbrx` is the
  64-bit form). Standalone → `rlwimi` byte-shuffle (32) or ISA 3.0 `brh/brw/brd` byte-
  reverse instructions (fast path, §ISA-tiering).
- **BROL/BROR:** PPC rotates left only (`LJ_TARGET_UNIFYROT=1` already set); ROR =
  rotate-left by `width-n`.

---

## 10. Calls (`asm_gencall`, `asm_setupresult`, `asm_callx`) — ELFv2

- **Arg marshalling:** GPR `r3–r10`, FPR `f1–f13`; overflow to the parameter save area
  (≥ `32(SP)`). FP args do **not** consume a GPR slot under ELFv2 (unlike some ABIs) —
  but a vararg float still also needs GPR shadowing only for `...` prototypes; follow
  the `CCallInfo` flags. Use `ra_leftov` to place each arg.
- **Direct call:** `bl` (±32MB) to local entry; far → `mtctr`+`bctrl`. **Inter-module
  indirect calls must reload `r2`** from the TOC save slot after return (ELFv2).
- **`asm_setupresult`:** FP result in `f1` (`RID_FPRET`); int in `r3` (`RID_RET`);
  64-bit pair handled via `HIOP`/`ra_destpair` (rare on 64-bit). `CCI_CASTU64` FP-from-
  int → `mtvsrd`.
- **`asm_callx`:** const target from `ir_k64`; indirect target in a non-arg GPR
  (e.g. `r11`/`r12`); set `r12`=target for global-entry callees.

---

## 11. TValue materialization (`asm_tvstore64`, `asm_tvptr`)

- **`asm_tvstore64`:** const ref → `lj_ir_kvalue` then `ra_allock(u64)` + `std`.
  Live int → `clrldi t,src,32 ; or t,t,type(=TISNUM<<47) ; std`. Live gcref →
  `sldi t,typereg,47 ; or t,t,srcptr ; std`. (PPC has no add-with-shift; do the
  `sldi`+`or` explicitly — still 2 ops like arm64.)
- **`asm_tvptr`:** point dest at `g->tmptv` via JGL anchor `addi`; store the value
  there first (FP via `stfd`; others via `asm_tvstore64`).

---

## 12. Constant materialization (`emit_loadk64`, tiered) — quality bar #3

For a 64-bit value `u64`:
1. `int16` → `li`. 16-bit-high → `lis`. 32-bit → `lis`+`ori`.
2. Near JGL (`g±32K`) → `addi rd, RID_JGL, delta` (reuse existing trick — covers most
   GC pointers and `global_State` fields cheaply).
3. Delta from an existing in-reg const → `addi` (reuse `emit_kdelta1`, widen to 64-bit).
4. General 64-bit → `lis/ori` (low32) + `rldicr rd,rd,32,31` + `oris/ori` (high32).
5. **ISA 3.1 fast path:** `pli rd, imm34` (1 prefixed insn for ≤34-bit) / `paddi` for
   PC-relative — gate on `LJ_ARCH_VERSION >= 31` (§ISA-tiering in main plan).
6. FP consts: keep `emit_loadk64`→`lfd` from the k64 pool (or `mtvsrd` from a GPR const
   when cheaper).

---

## 13. Exits & snapshots (`asm_exitstub_setup`, `asm_stack_restore`, `asm_guardcc`)

- **`asm_guardcc`:** unchanged mechanism — `emit_condbranch(PPCI_BC, cc, exit)`; loop
  inversion emits the unconditional `b` + opposite-cc backward branch (port arm64's
  `invmcp`/`loopinv` logic onto the existing `emit_condbranch`).
- **`asm_exitstub_setup`:** per-trace exit stubs (the existing `exitstub_trace_addr`
  in `lj_target_ppc.h` already scans NOPs/`bctrl`; keep its shape but widen to 64-bit
  addresses). Stub: save LR, branch to `vm_exit_handler`, encode trace+exit number.
- **`asm_stack_restore`:** for each restorable snapshot slot, store the reconstructed
  TValue to `[BASE, ofs]` — FP via `stfd`, others via `asm_tvstore64`; keyindex slots
  store value + `LJ_KEYINDEX` tag. Direct port.
- **`ExitState`** (already widened in `lj_target_ppc.h` §2.1): `vm_exit_handler` in
  `vm_ppc.dasc` spills all GPR (`std`)/FPR (`stfd`)/spill slots into `ExitState`, then
  calls `lj_trace_exit`; restore path reloads them.

---

## 14. Handler coverage map (Phase 2 vs Phase 3)

**Phase 2 (MVP):** §1, §2, §5 (load+typecheck), §6, §7, §8, §9, §12, §13 + scaffolding
(`asm_head_root_base`, `asm_tail_fixup/prep`, `asm_loop_fixup`, `asm_stack_check`,
`asm_gc_check`, `asm_setup_target`, `asm_mcode_fixup` incl. `icbi`/`dcbf`/`isync`
cache flush).
**Phase 3:** §3 (refs), §4 (fields), §10 (calls/FFI), §11 (tvstore/tvptr), `asm_cnew`/
`cnewi`/`tnew`/`tdup`/`snew`/`xsnew`, `asm_tbar`/`obar`, `asm_bufput`/`bufstr`/`tostr`,
`asm_min_max` (via `isel`/`fsel`), `asm_fpmath`, `asm_hiop`, `asm_prof`, `asm_retf`.

**`isel`/`fsel`** (quality bar #2) replace compare-branch in `asm_min_max`,
`asm_intmin_max`, and any conditional-select fold — emit `cmpd`/`fcmpu` then
`isel rd,ra,rb,crbit` / `fsel`.

---

## 15. Validation hooks specific to the assembler

- `luajit -jdump` each hot trace; grep the asm for the anti-patterns (no `std`/`lfd`
  round-trips for conv, no `mcrxr`, `isel` present for min/max).
- Differential `-joff` vs `-jon` per fold rule.
- Exercise on **both** ppc64le and ppc64 (BE) — the `LJ_BE` int-load offset (`ofs^4`)
  and TValue byte order are the likely BE-only failure points.
