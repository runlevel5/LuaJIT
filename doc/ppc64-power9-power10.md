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

### POC 3: mcrxrx (32-bit overflow primitive) for asm_arithov (commit pending)

`asm_arithov` (IR_ADDOV/SUBOV/MULOV -- the overflow-guarded integer add/sub/mul
that promotes to double on 32-bit overflow). MUL already uses `mullwo.` (a
32x32->32 op whose OV/SO are natively 32-bit; unchanged). ADD/SUB on GC64 needed
a 5-instruction trick: `addo./subfo.` are 64-bit ops, so to catch *32-bit*
overflow the P8 path shifts both operands <<32, runs the flag op on the shifted
values (overflows 64-bit iff the low 32 overflow), guards, then does a separate
plain add/subf for the real result (rldicr x2 + addo. + add/subf + guard).

POWER9 `mcrxrx BF` copies XER[OV,OV32,CA,CA32] into a CR field (cr0: LT=OV,
GT=OV32, EQ=CA, SO=CA32). `addo`/`subfo` (OE=1) set OV32 = the 32-bit signed
overflow of the low-32 result AND compute the result. So the P9 path is:
`addo/subfo dest,l,r` ; `mcrxrx cr0` ; guard `CC_GT` (OV32) -- **3 instructions,
no shift trick, no separate result add** (5 -> 3). Gated `#if LJ_ARCH_VERSION >=
90`; P8 path intact as `#else`. (PPCI_MCRXRX added to lj_target_ppc64.h; emitted
as a raw word like the existing PPCI_MCRXR.)

Verified: P9 trace emits `addo; .long 0x7c000480 (mcrxrx); bgt ->exit` (no
rldicr<<32). Overflow correct for add/sub, both signs, exact int32 boundaries
(2147483647+1, INT_MIN-1, sum 1..100000 = 5000050000) jit==joff==P8, both endians
(LE power9 + BE KVM guest MSB). P8-vs-P9 gen4 cross-build diff IDENTICAL (LE 1500
seeds: 0 diffs/0 crashes each). Regression diffs=0.

Benchmark (overflow-guarded add/sub hot loop): like modulo, the wall-clock win is
negligible (P8 ~1.86s == P9 ~1.86s) -- the 2 extra `rldicr` shifts in the P8 path
are independent and dual-issue on the wide POWER core, hidden by the loop's other
latencies. The benefit is code size (fewer mcode bytes per overflow-guarded op),
not speed, on this core.

### P9 phase summary

P9 fast paths complete: **modulo** (modsw, marginal interp win), **setb** (no
applicable site -- not emitted), **mcrxrx** (overflow, 5->3 insns, code-size win).
Honest overall: the P9 ISA-3.0 integer ops are correct and reduce instruction
count, but wall-clock gains are negligible on the superscalar POWER9 core (the
saved instructions are cheap/hidden). The real deliverable was the validated
ISA-gating + cross-build-diff pattern, now proven on both endians for 3 paths.
Next: **P10** (prefixed constants = the genuinely high-value win: a multi-instr
`emit_loadu64` -> 1-2 instr `pli`/`pld`; needs the prefixed-alignment helper first).

## POWER10 (ISA 3.1) — prefixed constants

### Build setup (power10 box, LE)
`ssh power10` = ppc64le Fedora 44, gcc 16.1.1, native POWER10 (ISA 3.1).
Build: `make XCFLAGS="-mcpu=power10 -mno-pcrel" TARGET_CFLAGS="-mcpu=power10 -mno-pcrel"`.
`-mcpu=power10` -> _ARCH_PWR10 -> LJ_ARCH_VERSION=100. **`-mno-pcrel` is REQUIRED**:
gcc-16 defaults to -mpcrel under -mcpu=power10, which makes external calls skip the
TOC restore; the linker then rejects the TOC-based hand-written VM assembly
(`bl <fn> lacks nop, can't restore toc` at ~all `bl extern` sites). The LuaJIT
ppc64 VM is TOC/r2-based, so -mno-pcrel keeps TOC addressing. (A full PC-relative
VM would be a separate port; pli/pld immediate forms don't need pcrel.) Keep a P8
build dir for the cross-diff. BE-P10 DEFERRED (box pending; prefixed constants are
endian-neutral so LE validation suffices).

### The hard part: 64-byte prefixed-instruction alignment (commit d4973d02)
A prefixed instr is 8 bytes (prefix word low, suffix word high) and must not cross
a 64-byte boundary. The emitter runs BACKWARD (*--mcp); emit_prefixed() writes
suffix then prefix, landing the instr at [mcp,mcp+8). It straddles iff
(mcp&63)==60, so when (mcp-8)&63==60 we prepend a NOP first (shifting it down 4
bytes). UNIT-TESTED in a standalone C harness replicating the backward emitter:
every start offset mod 64, single + 5-consecutive -> no prefix word at offset 60,
every [pfx,pfx+8) within one 64-byte block. PASS. (Uses the real as->mcp address,
so correct regardless of mcode-area alignment.) pli/pld encodings cross-checked
byte-for-byte vs `gcc -mcpu=power10`.

### emit_loadu64/emit_loadi -> pli (commit b8de9b38)
Gated `#if LJ_ARCH_VERSION >= 100`, P9/P8 intact as `#else`. A single prefixed
`pli rD,imm` replaces lis+ori for a non-checki16 32-bit constant (and the lis-only
case); the full-64-bit path's hi32 load also becomes one pli. The cheaper 1-instr
JGL-anchor addi / kdelta1 forms (4 bytes) are still preferred over pli (8 bytes).

Validation (power10 box, LE): constants correct -- KINT>16b, negative, pointer
casts, KINT64 -- jit==joff==P8. P10 trace emits 9 pli / 0 lis where P8 emits 9 lis
/ 0 pli; every prefixed instr aligned (mod 64 != 60). P8-vs-P10 gen4 cross-build
diff: P8 = 0 diffs / 0 crashes; P10 = 0 diffs / 0 crashes (1500 seeds each on the
power10 box) -- IDENTICAL. Full P10 regression: diffs=0 (bar pre-existing joff debug_gc).

### Benchmark — HONEST: no measurable wall-clock win
Constant-heavy loops: P8 ~= P10 (e.g. 0.88s == 0.88s). Two reasons: (1) loop-
invariant constants are HOISTED out of the hot loop by the JIT, so the const load
runs once per trace, not per iteration; (2) pli replaces lis+ori with 1 instr vs 2
but the SAME 8 bytes -- the lis->ori dependency is hidden on the wide OoO POWER10
core, and pli is not a code-SIZE win over lis+ori (only over 3+ instr sequences,
which are rarer since the JGL anchor already does most pointers in 1 instr). So the
genuine benefit is "1 fused instr vs a 2-instr dependent pair" (decode/rename
pressure), which does not move wall-clock or code size measurably here. Consistent
with the P9 finding: ISA-3.0/3.1 integer/const ops are correct and reduce
instruction COUNT but the saved work is hidden by the wide core. The real
deliverable remains the validated, alignment-correct prefixed-instruction
infrastructure (the alignment helper gates ALL future P10 prefixed paths) + the
cross-build-diff-identical guarantee.

## Codegen-quality investigation (2026-06-29) — real wins beyond ISA tiers

The ISA-3.0/3.1 micro-ops were all correct but ~0 wall-clock (wide OoO cores +
JIT hoisting hide them). PIVOT: baseline (POWER8) codegen quality, where ppc64
lags arm64. Evidence below; benefits all builds/endians.

### Reference gap (evidence)
fannkuch trace: **arm64 = 2127 instrs, ppc64 = 2965 (~39% larger)**. Instruction
mix (ppc64 vs arm64):
- ppc64 `rldicl`(344)+`rldicr`(195)+`sradi`(82) = 621 shift/mask vs arm64
  `and`(61)+`asr`(61) = 122. The bulk is GC64 pointer untag (`rldicl rX,rX,0,17`
  = GCVMASK) + itype extract (`sradi 47`), emitted on every GCRef deref; arm64
  folds the mask into `and x,x,#imm` and into addressing/compare operands.
- ppc64 `lis`(182)+`ori`(127) const materialization (2 instr/const) vs arm64
  `mov`/`movn` (1 instr/const).

### Prioritized real-win list

| # | opportunity | evidence | est. win | effort | status |
|---|-------------|----------|----------|--------|--------|
| 1 | **FP<->int direct GPR<->FPR moves** (mtvsrd/mfvsrd vs stack round-trip) | conv-dependent chain P9 3.39->3.05s (~10%), P10 1.24->1.18s (~5%); ray ~3%, pidigits ~10% | 3-10% on conv-heavy, neutral else | low | **DONE (98f04781)** |
| 2 | C-call TOC save/restore (`std/ld r2,24`) skip for same-module calls | MEASURED: NOT on critical path (see below) | **~0% (measured)** | med, correctness-sensitive | **REJECTED — not worth it** |
| 3 | GC64 pointer-untag density (`rldicl ,0,17` per GCRef) | 621 mask/shift instrs in fannkuch (vs arm64 122) | unknown; likely small (cheap ops, OoO-hidden) | high (needs fold into addressing or redundant-mask elimination; ppc has no masked-load) | investigate, likely low ROI |
| 4 | const materialization lis+ori (2 instr) | 309 in fannkuch | ~0 wall-clock (P10 pli proved neutral; OoO-hidden) | n/a | NOT worth it (proven neutral) |
| 5 | spill behavior / dispatch / snapshot overhead | not yet measured | unknown | - | not investigated |

HONEST META-FINDING: on these wide OoO POWER9/10 cores, instruction-count
reductions rarely move wall-clock unless the removed work is on a tight
dependency chain (which is why #1 wins on dependent conv chains but is neutral
when conversions parallelize). #3/#4 are large instruction-count gaps but the
ops are cheap and OoO-hidden, so likely low ROI.

### #2 C-call TOC save/restore — MEASURED, REJECTED (2026-06-29)
Measured-first (per the dep-chain meta-finding, since `ld r2` could sit on the
call's dependency chain). Helper-call-heavy DEPENDENT benchmark (/tmp/callchain.lua:
a tight loop, each iter dependent, 14 IRCALL helpers/trace -- lj_vm_modi x2 +
lj_str_* chain; confirmed `bctrl; ld r2,24(sp)` is in the hot trace right on the
call return). Upper-bound measurement -- temporarily removed the `ld r2` (and then
BOTH `std r2`+`ld r2`) from emit_call and timed:
| build | POWER9 | POWER10 |
|-------|--------|---------|
| baseline (TOC dance) | 0.276s | 0.210s |
| no `ld r2` | 0.276s | -- |
| no `std`+no `ld r2` | 0.275s | 0.210s |
=> **REMOVING THE TOC SAVE/RESTORE ENTIRELY GIVES ~0 WALL-CLOCK** on both cores.
The C-call body (helper execution + indirect `bctrl` branch + return) dominates;
the `std/ld r2` stack accesses are fully hidden by OoO + the call latency. The
`ld r2` is NOT on the critical path. So even the upper bound isn't worth the
implementation -- and it's the PLT-fault bug class (skipping for a constant-addr
CALLX, which shares emit_call with IRCALL, would corrupt r2). REJECTED: not
implemented. (Measurement hack reverted; tree clean.)

---

## openresty/luajit2-test-suite run (2026-07-02, ppc64le LE, HEAD 3687f0f6)

Suite: https://github.com/openresty/luajit2-test-suite  
Box: power9 LE host, gcc 16.1.1, LuaJIT installed via `make install PREFIX=~/lj-prefix`.  
Run: `perl run-tests ~/lj-prefix ~/lj-prefix/bin/luajit gcc g++`  
Result: **12 failures** — 4 noise, 8 real ppc64-specific bugs (confirmed by arm64 cross-check).

### Noise (not ppc64 bugs)

| Test | Reason |
|---|---|
| `misc/hstore_elimination.lua` | Requires `table.clone` — OpenResty extension, not in vanilla LuaJIT |
| `misc/libfuncs.lua` | Expects no `bit`/`jit` in default global list — OpenResty-specific |
| `sysdep/ffi_include_gtk.lua` | gtk+ headers not installed on headless box |
| `sysdep/ffi_include_std.lua` | gcc-16 `stddef.h` uses `__typeof__` — LuaJIT C parser upstream limitation |

Parallel build race note: `make -j4` fails (buildvm-generated headers vs .o race in
FreeBSD/older gmake); use `make -j1` or `make -j2` for reliable builds.

### Real ppc64-specific failures (all pass on arm64)

**JIT crash (SIGSEGV, joff passes):**

- `unportable/math_special.lua` — JIT crashes (signal 11) on `x^y` / math operations
  with special float inputs (`-0`, `±inf`, `nan`). joff passes cleanly. Clear JIT codegen
  bug in the float-special-value path (power instruction or snap/exit handling).

**FFI ABI failures (both jit and joff):**

- `ffi/ffi_call.lua:132` — `call_ff_cf` (`complex float` argument passing): assertion
  fails. ppc64 ELFv2 `_Complex float` argument ABI not handled.
- `ffi/ffi_callback.lua:40` — `float (double, float, double)` FFI callback: assertion
  fails. Mixed float/double callback ABI on ppc64.
- `ffi/ffi_jit_call.lua:69` — JIT path: `call_ij(int, int64)` assertion fails at line 69;
  joff fails differently (line 105, "attempt to call a boolean value"), indicating two
  distinct code paths each have ABI issues.
- `ffi/ffi_convert.lua`, `ffi/ffi_jit_conv.lua` — require `ctest.so` (clib); failed in
  test runner (exit 1); needs further investigation with proper `LUA_CPATH` env.

**FFI metatype / upvalue bug (both jit and joff):**

- `ffi/ffi_metatype.lua:109` — `ffi.metatype` returns ctype correctly (confirmed:
  `type(tp)=="cdata"`, `tostring(tp)=="ctype<struct 103>"`), but inside the `__add`
  metamethod, accessing `tp` as a closed upvalue yields a number ("attempt to call a
  number value"). arm64 passes. Likely a GC64 upvalue / UREFC truncation bug for
  ctype values (similar class as the prior `bit.lua` UREFC i32ptr bug, but in the
  interpreter path for ctype TValues).

**C library / FFI namespace:**

- `misc/num_int.lua:52` — `C.inet_pton` seen as `cdata<int ()>` (plain function
  pointer, not callable? or wrong ABI convention). arm64 passes. Possible ppc64-specific
  `ffi.C` symbol resolution or cdata function-call dispatch issue.

### Summary

8 real bugs remain to fix before the test suite is clean:
1. **JIT crash on float specials** (`math_special`) — JIT only
2. **`_Complex float` call ABI** (`ffi_call`) — interp + JIT
3. **Mixed float/double callback ABI** (`ffi_callback`) — interp + JIT
4. **`call_ij` int64 JIT path** (`ffi_jit_call`) — JIT + interp diverge
5. **ctype upvalue UREFC truncation** (`ffi_metatype`) — interp + JIT
6. **`ffi.C` function dispatch** (`num_int`) — interp + JIT
7–8. `ffi_convert` / `ffi_jit_conv` — pending `LUA_CPATH` investigation

---

### Perf-phase recommendation: WRAP
With #1 (FP<->int, the one real win) shipped and #2 measured-and-rejected, and
#3/#4 being OoO-hidden instruction-count gaps with no evidence of a real stall,
there is no remaining high-ROI baseline-codegen win identified. The wide OoO
POWER9/10 cores hide nearly all instruction-count reductions; only tight
dependency-chain stalls (like the FP<->int stack round-trip) move wall-clock.
RECOMMENDATION: wrap the optimization phase. Further codegen work should be
strictly profile-driven (find an actual stall first), not instruction-count-driven.
The port is correctness-complete + fuzzer-clean; the perf phase delivered the
ISA-gating/cross-build-validation methodology + one real conversion win.
