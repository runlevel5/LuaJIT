# PPC64 (ELFv2) JIT backend for LuaJIT — implementation plan

Status: planning. Target branch: `v2.1`.

## Goal & scope

Add a 64-bit POWER JIT backend to LuaJIT.

- **ISA:** baseline **2.07 (POWER8)**, with **3.0 (POWER9)** and **3.1 (POWER10)**
  fast paths selected at **build time** via `LJ_ARCH_VERSION` (from `-mcpu`).
  No runtime CPU dispatch (matches every existing LuaJIT arch).
- **ABIs in scope (all ELFv2):**
  1. Linux `ppc64le` (little-endian) — primary.
  2. Big-endian `ppc64` ELFv2 (LE/BE parameterized, not hardcoded).
  3. FreeBSD `ppc64le`.
- **Out of scope:** ELFv1 (legacy big-endian function-descriptor ABI), AIX,
  32-bit ppc, PS3/Xbox360 32-on-64 (these stay on the existing legacy path).

Because two endiannesses are in scope, the backend must be **endian-parameterized**
throughout (`LJ_BE`/`ENDIAN_BE`), not LE-hardcoded. ELFv2 is the unifying constant
(`_CALL_ELF == 2` on both LE and BE), so detection gates on ELFv2, not endianness.

## Why prior ports were judged poor — and the reframing

Prior community ports (PPC64/LuaJIT `ppc64-port`, mwkmwkmwk `ppc64-ffi`) extended
the **32-bit, big-endian, non-GC64** PPC backend to 64-bit. That drags along
assumptions that produce slow, hard-to-maintain code:

- FP↔int conversions bouncing through memory (`stw`/`lfd`) instead of VSX moves.
- Compare-and-branch instead of `isel`/`fsel`.
- Bloated constant materialization; multi-instruction 64-bit masks.

**Reframing:** PPC64 needs 64-bit pointers ⇒ `LJ_GC64=1` ⇒ `LJ_FR2=1` (2-slot
frames). That is the *same architecture* as the **arm64** backend, which is
LuaJIT's modern GC64 + dual-number 64-bit reference. So:

> Build a fresh GC64 backend whose **GC64/FR2 control logic mirrors arm64**, and
> whose **instruction selection uses POWER encodings** from the existing ppc files
> and DynASM. Do *not* grow the 32-bit ppc backend.

## What already exists (don't redo)

- `dynasm/dasm_ppc.lua` is the "PPC/PPC64" module: it already encodes 64-bit ops
  (`ld`/`std`, `rldic*`), full **VSX** (`xv*`/`xs*`) and **AltiVec** (`v*`), and
  has `P64` plumbed. Needs verification + ISA 3.1 **prefixed instructions**.
- `src/Makefile` host detection already recognizes `ppc` + endian + `LJ_ARCH_BITS 64`
  and passes DASM `-D P64` / `-D ENDIAN_LE|BE`.
- `lj_arch.h` already computes the `ppc64le`/`ppc64` name — but then hits
  `#error "No support for PPC64"` (`lj_arch.h:369`).

## Quality bar (acceptance criteria, enforced in review via `-jdump`)

1. No FP↔int via memory bounce — use `mtvsrd`/`mfvsrd` + `fcfid`/`fctidz`.
2. Branchless selects via `isel` / `fsel`.
3. Minimal constant materialization; collapse to `pli` (1 prefixed insn) on 3.1.
4. 64-bit rotates/masks via `rldicl`/`rldicr`/`rldic`.
5. ELFv2 **local entry points** used for intra-module calls (skip `r2`/TOC setup).
6. No micro-coded/slow forms on hot paths.
7. Correct CR/XER (carry/overflow) handling alongside GC64 47-bit pointer checks.

## Decisions (locked)

- Number mode: `LJ_NUMMODE_DUAL` (match arm64), hardware FPU assumed.
- ISA selection: build-time only.
- ABIs: Linux ELFv2 LE, BE ppc64 ELFv2, FreeBSD ppc64le.
- Endianness: parameterized (both LE and BE), ELFv2-only.

## Components / workstreams

### A. Build plumbing & arch detection (small, first)
- `lj_arch.h`: replace the `#error` with an ELFv2 ppc64 branch — set `LJ_64`,
  `LJ_TARGET_GC64=1`, `LJ_FR2`, `NUMMODE_DUAL`, `JUMPRANGE` (B-form ±32MB ⇒ keep 25;
  far calls via `ctr`/trampolines), `LJ_ARCH_VERSION` for 2.07/3.0/3.1
  (`_ARCH_PWR8/9/10`), and gate on `_CALL_ELF == 2`.
- `src/Makefile` + host: add `CCOPT_ppc`/`-mcpu`, ELFv2 path, FreeBSD glue.
- CI: QEMU **ppc64le** + QEMU **ppc64 (BE, ELFv2)** + FreeBSD ppc64le; POWER9/POWER10
  boxes for perf and final sign-off.

### B. Interpreter — `vm_ppc.dasc` (long pole of Phase 1)
Write fresh, modeled on `vm_arm64.dasc`, reusing POWER instruction knowledge from
the existing file. Must implement:
- ELFv2 prologue/epilogue (global vs local entry, `r12`→`r2` TOC, back-chain,
  CR/LR save, 32-byte min frame, 288-byte red zone).
- GC64 + FR2 frame and TValue handling (2-slot frames, 47-bit pointers, tag checks).
- Endian-parameterized bytecode decode and lo/hi word handling.
- FFI call paths (current file errors out: `.error lib_ffi not yet implemented`).
- Fast-function + helper coverage to pass the suite with `-joff`.

### C. JIT backend (bulk; Phase 2–3)
- `lj_target_ppc.h`: 64-bit `ExitState.gpr`, **8-byte spill slots**
  (`sps_scale` ×8; fix `SPS_FIXED`/offsets for ELFv2), ELFv2 reg ranges, exit-stub
  addressing, optional VSX reg class for moves.
- `lj_emit_ppc.h`: 64-bit emitters (`ld/std/ldx/stdx`, 64-bit arith, `cmpd/cmpld`,
  `rldic*`), `emit_loadk64`, large-displacement helpers, ELFv2-aware `emit_call`,
  ISA-tiered constant loader.
- `lj_asm_ppc.h`: IR→machine. Borrow GC64 patterns from `lj_asm_arm64.h`
  (HREF/HREFK, pointer hi-checks, FLOAD/FSTORE, FR2 frame, snapshot restore, exits)
  and POWER instruction selection from existing `lj_asm_ppc.h`. Full IR coverage.

### D. FFI ABI — `lj_ccall.c` / `lj_ccall.h` / `lj_ccallback.c`
Implement **ELFv2** convention (replaces the ELFv1/32-bit path): GPR `r3–r10`,
FPR `f1–f13`, homogeneous float/vector aggregates (≤8), vector args, parameter save
area, struct return, results in `r3`(`/r3:r4`) or `f1`. ELFv2 callback trampolines.
Endian-correct aggregate/byte handling for BE.

### E. DynASM — `dasm_ppc.lua` / `dasm_ppc.h`
Verify `P64`; add missing ISA 3.0/3.1 ops, esp. **prefixed instructions**
(`pli`/`paddi`/`pld`/`pstd`, 8-byte, must not span 64-byte boundaries); ELFv2 helpers.

### F. Unwinding & tooling
- `lj_err.c` external unwinding + `.eh_frame` for ELFv2 (LR = ra reg 65).
- `lj_gdbjit.c` DWARF for ppc64 (both endians).

## ISA tiering (build-time)

| Operation            | 2.07 baseline               | 3.0 (P9)          | 3.1 (P10)            |
|----------------------|-----------------------------|-------------------|----------------------|
| 64-bit const load    | `lis/ori/rldicr/oris/ori`   | `addpcis` (PC-rel)| `pli` (1 insn, 34b)  |
| Large disp load/store| `addis`+`ld`                | same              | `pld`/`pstd` (34b)   |
| int↔fp               | `mtvsrd`+`fcfid`/`fctidz`   | same              | same                 |
| select               | `isel`                      | `isel`            | `isel`               |
| integer modulo       | `divd`+`mulld`+`subf`       | `modsd`/`modud`   | same                 |
| count-trailing-zeros | emulate                     | `cnttzd`          | same                 |

## Phasing & milestones

- **Phase 0 — Plumbing:** arch detection, Makefile, 3× QEMU CI green.
  Exit: `luajit -joff -e "print(1+1)"` runs.
- **Phase 1 — Interpreter:** `vm_ppc.dasc` ELFv2 + GC64 + FR2 + endian-parameterized
  + FFI ccall/ccallback. **Exit: full LuaJIT + PUC suites pass `-joff` on LE and BE.**
- **Phase 2 — JIT MVP:** emit/target/asm for int+fp arith, loads/stores, guards,
  loops, exits/snapshots. Exit: numeric loops trace correctly; differential vs `-joff`.
- **Phase 3 — Full JIT:** complete IR coverage, FFI/cdata in traces (`CALLX`),
  strings/tables, fold/narrow, math fast funcs. **Exit: full suite passes JIT-on.**
- **Phase 4 — ISA fast paths + tuning:** wire 3.0/3.1, perf on POWER9/POWER10,
  `-jdump` audits against the quality bar.
- **Phase 5 — Hardening & upstreaming:** unwinding/gdbjit, edge cases, soak,
  upstream-quality review (coordinate with Mike Pall's CONTRIBUTING expectations).

## Validation
- Differential `-joff` vs `-jon` on every trace; PUC-Lua + LuaJIT test suites.
- `-jdump`/`-jbc` audits enforce the quality bar.
- QEMU (LE + BE) + FreeBSD for correctness gating; real POWER9/POWER10 for perf.

## Top risks
1. GC64/FR2 correctness in the interpreter (Phase 1 — highest leverage).
2. ELFv2 ABI subtleties (TOC/local entry, HFA, callbacks, unwinding) — test against
   real C libs via FFI early.
3. ISA 3.1 prefixed-instruction alignment in DynASM.
4. Two-endianness test matrix doubling validation cost.
5. Upstream acceptance — mitigated by the quality bar and mirroring arm64.

---

# Detailed task breakdown

Each task lists the files it touches and an exit/verify check. Tasks within a phase
are roughly ordered by dependency. `arm64` files are the structural template;
existing `ppc` files + `dasm_ppc.lua` are the POWER-instruction source.

## Phase 0 — Build plumbing & skeleton

- **0.1 Arch branch** — `lj_arch.h`: replace `#error "No support for PPC64"`
  (`:368-371`) with an ELFv2 ppc64 block gated on `_LP64 && _CALL_ELF == 2`. Set
  `LJ_TARGET_PPC`, `LJ_TARGET_GC64=1`, `LJ_FR2`, keep `JUMPRANGE 25`,
  `EHRETREG/EHRAREG`. Leave existing 32-bit/PS3 path intact.
  *Verify:* `cc -E -dM lj_arch.h` shows `LJ_TARGET_PPC 1`, `LJ_TARGET_GC64 1`,
  `LJ_ARCH_BITS 64` on a ppc64le/-be cross compiler.
- **0.2 Feature macros** — `lj_arch.h`: `NUMMODE_DUAL`; `LJ_ARCH_VERSION` tiers from
  `_ARCH_PWR8/9/10`; `LJ_ARCH_SQRT`/`ROUND`; remove `LJ_ARCH_NOFFI` for this path.
- **0.3 Build system** — `src/Makefile`: `CCOPT_ppc=-mcpu=power8` (overridable);
  confirm host detection already emits `-D P64` + `-D ENDIAN_LE|BE` (it does, `:409-415`);
  add FreeBSD ppc64le OS glue. *Verify:* `make` reaches the DynASM + buildvm steps.
- **0.4 Compile skeleton** — minimal `lj_target_ppc.h`/`lj_emit_ppc.h`/`lj_asm_ppc.h`
  edits + a `vm_ppc.dasc` that assembles (all interp paths may be `NYI`/`trap`).
  Goal: the tree *links* a `luajit` binary for ppc64le and ppc64. *Verify:* binary
  builds and prints version; running anything may trap — that's fine.
- **0.5 CI** — QEMU `ppc64le`, QEMU `ppc64` (BE, ELFv2), FreeBSD ppc64le runners;
  smoke job = build + `luajit -v`.

## Phase 1 — Interpreter (`vm_ppc.dasc`, fresh, modeled on `vm_arm64.dasc`)

- **1.1 Frame & ELFv2 entry** — register assignment (match `lj_target_ppc.h`
  `RID_BASE/LPC/DISPATCH/LREG/JGL`), `saveregs`/`restoreregs`, global vs local entry
  point, `r12`→`r2` TOC, back-chain, CR/LR/nonvolatile save, 288-byte red zone,
  32-byte min frame. Template: `vm_arm64.dasc:106-196`.
- **1.2 Core macros** — endian-parameterized `decode_RA/RB/RC/RD`, `ins_NEXT`,
  `ins_call`/`ins_callt`, `checktp`/`checknum`/`checkint`, `init_constants`,
  `hotcheck`/`hotloop`/`hotcall`, `mv_vmstate`, `barrierback`.
  Template: `vm_arm64.dasc:223-353`.
- **1.3 Dispatch + arithmetic/compare BCs** — interpreter dispatch table, all
  numeric `BC_*` (ADD/SUB/MUL/DIV/MOD/POW, comparisons, unary, MOV/KSTR/KNUM…).
- **1.4 Calls/returns/loops/tables** — `BC_CALL*`/`RET*`/`TAILCALL`, `BC_FORL`/`ITERL`/
  `LOOP`, `BC_TGET*`/`TSET*`/`GGET`, upvalue/`UCLO`, `VARG`, `CAT`.
- **1.5 Fast functions** — `fff_*` builtins (math, string, `pairs`/`next`, `type`,
  `tonumber`/`tostring`, `pcall`, etc.).
- **1.6 Helpers/exits** — `vm_*` C-call helpers, `growstack`, error/`vm_unwind`,
  hook/profiler entry, `vm_exit_handler`/`vm_exit_interp` for trace exits.
- **1.7 FFI ccall** — `lj_ccall.c` ELFv2 block + `lj_ccall.h`: define
  `CCALL_HANDLE_REGARG/STRUCTARG/STRUCTRET/COMPLEX*` for GPR r3–r10, FPR f1–f13,
  HFA (≤8), vector args, parameter save area, struct return, `r3:r4`/`f1` results.
  Endian-correct aggregate packing (BE differs). Template: arm64 block `lj_ccall.c:36-100`.
- **1.8 FFI callbacks** — `lj_ccallback.c` ELFv2 section (`:73` `LJ_TARGET_PPC`):
  `CALLBACK_MCODE_HEAD`/`SLOTSZ`/`GROUP`, trampoline asm, arg gather mirroring 1.7.
- **1.9 Glue** — `host/buildvm*` externs, `lj_vm.h`, `Makefile.dep`.
- **Exit:** `vm_ppc.dasc` removes the `lib_ffi not yet implemented` error; **full
  LuaJIT + PUC-Lua suites pass with `-joff` on both ppc64le and ppc64 (BE).**

## Phase 2 — JIT MVP (`lj_target_ppc.h`, `lj_emit_ppc.h`, `lj_asm_ppc.h`)

- **2.1 Target GC64 update** — `lj_target_ppc.h`: widen `ExitState.gpr` to 64-bit,
  **8-byte spill slots** (`sps_scale` ×8, fix `SPS_FIXED`/`SPOFS_*` for ELFv2 frame),
  ELFv2 `REGARG_*`/scratch sets, `exitstub_trace_addr` for 64-bit, KREF range.
- **2.2 Core emitters** — `lj_emit_ppc.h`: `emit_loadk`/`emit_loadk64` (ISA-tiered
  constant loader), `emit_lso`/large-disp, `emit_loadofs`/`emit_storeofs`,
  `emit_call` (ELFv2 local-entry aware), `emit_kdelta`, immediate predicates
  (`emit_isk16`, mask helpers). Template: `lj_emit_arm64.h`.
- **2.3 Assembler scaffolding** — `lj_asm_ppc.h`: `asm_exitstub_setup`, `asm_guardcc`,
  `asm_gencall`/`asm_setupresult`, `asm_head_root_base`, `asm_tail_fixup`/`prep`,
  `asm_loop_fixup`/`tail_fixup`, `asm_stack_check`/`stack_restore`, `asm_gc_check`,
  `asm_setup_target`, `asm_mcode_fixup`, register-alloc hooks. Template:
  `lj_asm_arm64.h:51-2032`.
- **2.4 Integer + FP arithmetic** — `asm_add/sub/mul/neg/intmul/intneg`,
  `asm_band/bor/borbxor/bnot/bswap/bitshift`, `asm_intmin_max`,
  `asm_fparith/fpunary/fpmath/fpmin_max/min_max`. (IR: ADD SUB MUL NEG DIV POW ABS
  LDEXP MIN MAX BAND BOR BXOR BNOT BSWAP BSHL BSHR BSAR BROL BROR FPMATH.)
- **2.5 Comparisons/guards** — `asm_intcomp/fpcomp/comp` + CR/`isel` mapping for
  IR LT GE LE GT ULT UGE ULE UGT EQ NE ABC.
- **2.6 Loads/stores (baseline)** — `asm_fload/fstore/xload/xstore/ahuvload/ahustore/
  sload` + `asm_fusexref` address fusion. (IR: ALOAD HLOAD ULOAD FLOAD XLOAD SLOAD
  VLOAD ASTORE HSTORE USTORE FSTORE XSTORE.)
- **2.7 Const/conv** — `asm_conv/tobit/tointg/strto`; KINT/KINT64/KNUM/KGC/KPTR
  materialization via the tiered loader; FP↔int through VSX (quality bar #1).
- **Exit:** numeric/array loops trace and exit correctly; differential `-joff` vs
  `-jon` on a focused micro-suite, LE + BE.

## Phase 3 — Full JIT coverage

- **3.1 Table/ref ops** — `asm_aref/href/hrefk/uref/fref/strref/newref` with GC64
  47-bit pointer + hi-word handling (copy arm64 hash/key logic). (IR: AREF HREF HREFK
  NEWREF UREFO UREFC FREF TMPREF STRREF LREF ALEN.)
- **3.2 Allocation + barriers** — `asm_cnew/cnewi/tnew/tdup/snew/xsnew/bufhdr`,
  `asm_tbar/obar`. (IR: TNEW TDUP CNEW CNEWI SNEW XSNEW TBAR OBAR XBAR BUFHDR.)
- **3.3 FFI in traces** — `asm_callx/gencall/setupresult` for ELFv2 (`CALLXS`/`CARG`),
  `asm_tvptr/tvstore64`, cdata paths. Exercise against real C libs.
- **3.4 Buffers/strings/misc** — `asm_bufput/bufstr/tostr`, `asm_prof`, `asm_hiop`
  (mostly inert on 64-bit but ALEN/CONV edge cases), `asm_retf`, `asm_bufhdr_write`.
- **3.5 Snapshot/exit fidelity** — verify `asm_stack_restore` + exit stubs restore
  all GPR/FPR/spill correctly under GC64; SLOAD variants (typecheck/convert/inherit).
- **Exit:** **full suite passes JIT-on on ppc64le and ppc64 (BE)**; benchmark outputs
  match the interpreter.

## Phase 4 — ISA fast paths (build-time) + tuning

- **4.1 Constant loader tiers** — `emit_loadk64`: `pli`/`paddi` (3.1, 34-bit, 1 insn),
  `addpcis` PC-relative (3.0), baseline `lis/ori/rldicr/oris/ori` (2.07).
- **4.2 Arithmetic tiers** — `modsd`/`modud` (3.0) vs `divd`+`mulld`+`subf`;
  `cnttzd`/`popcntd`; `darn` if useful.
- **4.3 Memory tiers** — `pld`/`pstd` 34-bit displacement (3.1) in `emit_lso`;
  requires `dasm_ppc.lua` prefixed-instruction support + 64-byte-boundary guard.
- **4.4 Select/FMA audit** — ensure `isel`/`fsel` and `fmadd` used everywhere the
  quality bar requires; no compare-branch or memory-bounce regressions.
- **4.5 Perf** — benchmark on POWER9/POWER10; `-jdump` audit of hot traces against
  the 7-point quality bar; tune register sets / fusion.

## Phase 5 — Hardening & upstreaming

- **5.1 Unwinding** — `lj_err.c` external unwind + `.eh_frame` for ELFv2 (ra = LR,
  reg 65); verify C↔Lua↔C exceptions and `error()` across FFI frames.
- **5.2 gdbjit** — `lj_gdbjit.c` DWARF for ppc64 (both endians).
- **5.3 Edge cases** — far-call trampolines (>±32MB), mcode region allocation range,
  exit-stub count limits, trace stitching/side traces, `mcode_fixup` cache flush
  (`icbi`/`dcbf`/`isync`).
- **5.4 FreeBSD** — OS-specific mmap/exec, unwinding, `clock`/syscall glue.
- **5.5 Conformance & upstream** — full suite + soak on real hardware (all 3 ABIs),
  `-jdump` quality audit, then submit upstream per Mike Pall's CONTRIBUTING norms.

## Suggested parallel tracks (after Phase 1 lands)

- **Track A (backend):** Phases 2→3→4 — one engineer deep on `lj_asm_ppc.h`.
- **Track B (FFI):** 1.7/1.8 → 3.3 → 5.1 — ABI specialist; gates real-world use.
- **Track C (infra):** 0.5 CI, 4.5 perf harness, 5.2 tooling — runs continuously.
