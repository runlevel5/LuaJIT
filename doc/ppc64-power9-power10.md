# PPC64 POWER9 / POWER10 fast-path plan

Optimization phase for the ppc64 GC64 JIT backend. The port is correctness-complete
(FR1-swept, fuzzer-clean) on the POWER8 / ISA 2.07 baseline, both endians. This adds
ISA-3.0 (POWER9) and ISA-3.1 (POWER10) fast paths as a *separate, additive* phase:
every fast path is semantically identical to the POWER8 path and independently
validated, so a fast-path bug can never masquerade as correct.

## Foundation: build-time ISA tiers (already present)

`lj_arch.h` already defines `LJ_ARCH_VERSION`:
- **80** — ISA 2.07 / POWER8 (baseline; always the fallback)
- **90** — ISA 3.0 / POWER9
- **100** — ISA 3.1 / POWER10

Set at build time from `-mcpu`. Fast paths gate on `#if LJ_ARCH_VERSION >= 90/100`,
with the POWER8 sequence retained as the `#else`. This is the LuaJIT-idiomatic model
(cf. the x86 SSE / ARM version tiers): build per target CPU.

*(Future option: runtime `AT_HWCAP2` detection (PPC_FEATURE2_ARCH_3_00 / _ARCH_3_1;
FreeBSD via `elf_aux_info`) + mcode dispatch for a single fat binary. More complex —
deferred.)*

Scope: **JIT first** (`lj_asm_ppc64.h` / `lj_emit_ppc64.h` — runtime codegen, biggest
leverage), interpreter (`vm_ppc64.dasc` via DynASM) second.

## Hardware / testing matrix

| tier | LE | BE |
|------|----|----|
| POWER8 (80) | yes | yes (power9-hosted KVM guest, `-mcpu=power8`) |
| POWER9 (90) | **power9 box** | **the BE KVM guest** (ISA 3.0) |
| POWER10 (100) | **power10 box** | **deferred** — BE-P10 box pending setup |

Strategy per the project: write endian-aware P10 code now, validate on **LE power10**,
defer BE-P10 testing until the box exists. Endian-neutral P10 paths (prefixed
constants, modulo, setb/setbc, 34-bit-disp) are low-risk to defer; **only the
`brd`/`brw`/`brh` byte-reverse path is byte-order-specific → must be BE-P10-validated
when the box is ready** (mark it `BE-P10 PENDING` in code/comments).

## POWER9 (ISA 3.0) fast paths — `#if LJ_ARCH_VERSION >= 90`

| path | replaces | site | notes |
|------|----------|------|-------|
| `modsw`/`moduw`/`modsd`/`modud` | divw/divd + mullw + subf | `asm_arith` IR_MOD (int); dasc `BC_MOD` | **start here.** Keep the floored-modulo correction (Lua `%` ≠ C `%`: add `b` when signs differ). Watch div-by-0 / `INT_MIN % -1`. |
| `setb` (GPR ← CR0) | branch + `li 0/1` | `asm_comp` integer-result patterns | branchless → fewer trace side-branches |
| `mcrxrx` / `addex` | `addo./subo.` + 64-bit-shift overflow trick | `asm_arithov` | ISA-3.0 overflow primitives; simplifies the int-overflow path |
| `cnttzd`/`cnttzw` | (bit-scan where needed) | — | niche |

## POWER10 (ISA 3.1) fast paths — `#if LJ_ARCH_VERSION >= 100`

| path | replaces | site | notes |
|------|----------|------|-------|
| prefixed `pli`/`paddi`/`pld` | `emit_loadu64` lis+ori+oris (2–5 instr) | `lj_emit_ppc64.h` constant materialization | **biggest win** — a constant per GCRef/ptr/KINT64. Needs prefixed-instruction alignment (below). |
| 34-bit-disp `pld`/`pstd`/`plwz` | addis + ld for large offsets | CFRAME spills, >16-spill frame, TOC | 1 instr vs 2 |
| `brd`/`brw`/`brh` (byte-reverse in GPR) | LWBRX/STWBRX load-store dance | `asm_bswap`, BE byte handling | faster `bit.bswap` + cleaner BE. **BE-P10 PENDING** (byte-order-specific) |
| `setbc`/`setbcr` | (better `setb`) | `asm_comp` | branchless compares |
| PC-relative `pla`/`pld` (R=1) | TOC-indirect global/const loads | JGL/global access, ELFv2 TOC dance | fewer TOC deps |
| `pextd`/`pdepd`/`cntlzdm` | — | bit-manip / hashing | niche |

## The hard parts (build these first)

1. **Prefixed-instruction alignment.** A prefixed (8-byte) instruction must not cross
   a 64-byte boundary. The JIT mcode emitter must detect this and pad with `nop`.
   This gates *all* P10 prefixed fast paths → implement the alignment helper before
   any `pli`/`pld`/`pstd` use.
2. **DynASM.** Interpreter fast paths need new templates in `dynasm/dasm_ppc.lua`
   (+ the `.h`), incl. prefixed-instruction encoding/alignment → JIT-first, interp later.
3. **Modulo semantics.** Hardware `mod*` is C-truncating; Lua `%` is floored — keep the
   sign correction.

## Implementation order (value ÷ effort)

1. **P9 integer modulo** — high value, low effort. Proves the ISA-gating pattern
   end-to-end incl. P9 testing on both endians (power9 LE + BE KVM guest).
2. **P10 prefixed-constant alignment helper + `emit_loadu64`** — highest value,
   moderate effort (the alignment is the real work). LE-power10-validated; BE-P10 later.
3. **P10 `brd` bswap** (BE-P10 PENDING) + **P9/P10 `setb`/`setbc` compares** — cheap wins.
4. **P9 `mcrxrx` overflow**, **P10 34-bit-disp loads**, then niche bit-manip / PC-relative.

## Validation (reuse the existing harness)

- **Correctness:** build P9/P10, run the **gen4 fuzzer** (jit==joff, crashes=0) + the
  full regression guard. Crucially, **cross-build diff P8 vs P9 vs P10** — identical
  output proves each fast path is equivalent. P9 on both endians now; P10 LE now,
  BE-P10 when the box is ready (priority: the `brd` path).
- **Performance:** per-path microbenchmarks (modulo loop, constant/FFI-pointer-heavy
  loop, bswap loop) + a macro benchmark, P8 vs P9 vs P10 — confirm each is faster.

## Results

### P9 build setup (validated 2026-06-28)

Per-CPU build via `-mcpu`: `make XCFLAGS="-mcpu=power9" TARGET_CFLAGS="-mcpu=power9"`
(TARGET_CFLAGS feeds the `-E lj_arch.h -dM` probe that sets `VER`/`LJ_ARCH_VERSION`
and the DASM `-D VER=` flag; XCFLAGS feeds the C compile). `-mcpu=power9` defines
`_ARCH_PWR9` -> `LJ_ARCH_VERSION=90` -> the `#if VER >= 90` dasc path. Keep separate
build dirs for P8 and P9 (e.g. `/tmp/ljP8`, `/tmp/ljP9`) for the cross-build diff.
Confirmed: P9 binary has `modsw`=1/`divwo`=2; P8 binary `modsw`=0/`divwo`=1. Both
endians (LE power9 box; BE KVM guest, which also defines `_ARCH_PWR9`).

### POC 1: integer modulo (commit 54414d98)

Lowering found: integer `%` is NOT inlined in the JIT trace — the shared `asm_mod`
(lj_asm.c) emits `asm_callid(IRCALL_lj_vm_modi)` for a variable divisor (ALL arches,
incl. arm64); a CONSTANT divisor is turned into multiply-by-reciprocal in the shared
IR optimizer (never calls the helper). So the modulo lowering lives entirely in
`lj_vm_modi` (vm_ppc64.dasc), shared by the interpreter BC_MOD and the JIT call.
=> Fixing `lj_vm_modi` covers both interp and JIT; no separate lj_asm_ppc64.h change
is correct here (inlining modsw in the trace would diverge from the shared design).
The PPCI_MOD* encodings were still added for a possible future inline use.

P9 path: `modsw` replaces `divwo.+mullw+sub` in the common case. KEY subtlety: the
edge cases (b==0, INT_MIN%-1) must set XER[SO] exactly like the P8 `divwo.`, because
callers detect them via SO to fall back to FP modulo (`5%0` -> nan, not 0). `modsw`
never sets SO, so the edges are detected explicitly and run `divwo.` only on them;
the common path is a single `modsw`.

Cross-build diff (the validation pattern): gen4 fuzzer, serial, race-free harness,
1500 seeds -- **P8: diffs=0/crashes=0; P9: diffs=0/crashes=0 (IDENTICAL)**. LE.
Modulo correct for all signs jit==joff==P8 both endians (-7%3=2, 7%-3=-2, INT_MIN%3=1,
INT_MIN%-1=0, 5%0=nan). Full regression guard on P9 LE: diffs=0 (bar pre-existing
joff debug_gc). BE: P9 modtest/divzero identical to P8, jit==joff.

Benchmark (modulo-heavy, variable divisor): the JIT case is dominated by the
vm_modi C-call overhead, so the ~2-instruction saving (modsw vs divwo+mullw+sub) is
in the noise there (P8 ~0.59s == P9 ~0.59s). The INTERPRETER path (joff, BC_MOD ->
vm_modi directly) shows a small real gain: P8 ~1.65s -> P9 ~1.60s (~3%). HONEST
TAKEAWAY: integer modulo is rarely a hot bottleneck and the win is marginal; the
POC's real value was proving the ISA-gating + cross-build-validation pattern
end-to-end on both endians. Higher-value P9/P10 targets (prefixed constants, setb)
should follow.

### POC 2: setb (branchless compare-to-value) — ASSESSED, NOT APPLICABLE

`setb RT,BFA` sets RT to -1/0/+1 from CR field BFA's LT/GT bits (a 3-way
sign-of-comparison as a value, no branch). Surveyed every value-producing
comparison site in the ppc64 backend (asm_comp/asm_intcomp/asm_intcomp64_/
asm_tointg/asm_strto, asm_min_max, the interp compare-to-value paths):

- **All IR comparisons (IR_LT/LE/GT/GE/EQ/NE, ABC, type checks) are GUARDS** —
  `asm_comp` emits `asm_guardcc` (a conditional branch to a side exit). A guard
  must branch; `setb` (which produces a value) cannot replace it. This is
  fundamental LuaJIT trace design.
- **min/max is already branchless** — FP via `fsel`, integer via the
  SUBFC/SUBFE carry trick. No compare-to-value, no branch.
- **The interpreter's compare-to-value sites already use `isel`** (ISA 2.06,
  P8-available) and they are **2-way** selects (`-1 if LT else 0`, etc.), not the
  **3-way** -1/0/+1 that `setb` produces. `setb` would give +1 for the GT case
  where these want 0 → semantically wrong, so it cannot replace them.
- **No 3-way -1/0/+1 comparison-result materialization exists** in the backend
  (string comparison and sort comparators go through C helpers / Lua calls).

Cross-check: arm64 HAS the equivalent (`cset`/`csinc`) but **also uses none of
them for value-producing compares** — its `asm_comp` is likewise all `asm_guardcc`.
This confirms it's a design property, not a ppc64 gap.

CONCLUSION: **`setb` has no applicable site.** Forcing it would mean either no-op
changes or contorting working `isel`/guard code with wrong-valued results, for zero
benefit — and risk on correctness-complete code. Per the "no speculative changes"
rule, NOT implemented. `PPCI_SETB` (lj_target_ppc64.h) + the `setb` DynASM template
(dasm_ppc.lua) are added as ready infrastructure for any future 3-way
compare-to-value site, but nothing emits them today. No fast path, no benchmark
(nothing changed), cross-build diff trivially identical (no codegen change).
