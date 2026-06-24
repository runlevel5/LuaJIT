# PPC64 ELFv2 — Phase 0 (buildable skeleton)

Companion to `ppc64le-port.md`. The concrete first commits: arch detection, build
plumbing, and the minimal stub set that lets the tree **link a `luajit` binary** for
ppc64le / ppc64-ELFv2 (interpreter paths may trap — that's the expected Phase 0 state).

## 0.1 — `lj_arch.h` arch branch  ✅ APPLIED

The `#error "No support for PPC64"` is replaced with a gated ELFv2 branch:

```c
#elif LJ_ARCH_BITS == 64
/* Endianness picks the ABI: little-endian = ELFv2, big-endian = ELFv1. */
#if LJ_ARCH_ENDIAN == LUAJIT_LE
#if defined(_CALL_ELF) && _CALL_ELF == 2
#define LJ_TARGET_GC64		1	/* ppc64le, ELFv2. */
#else
#error "ppc64le requires the ELFv2 ABI"
#undef LJ_TARGET_PPC
#endif
#else
/* Big-endian ppc64 (ELFv1) is not implemented yet -- build ppc64le for now. */
#error "PPC64 big-endian (ELFv1) is not implemented yet -- build ppc64le for now"
#undef LJ_TARGET_PPC
#endif
#endif
```

Why this is safe / sufficient:
- **Gated.** Only fires for 64-bit PPC. x86/arm/arm64/mips and 32-bit PPC and
  PS3 (`LJ_TARGET_CONSOLE`, hit first) are untouched.
- **Drives GC64/FR2 automatically.** `LJ_TARGET_GC64 1` flows into the existing
  plumbing at `lj_arch.h:597` → `LJ_GC64 1` → `LJ_FR2 1`. No other macro edits needed.
- **ABI split:** little-endian ppc64 uses ELFv2 (implemented), big-endian uses ELFv1
  (planned; currently a clean `#error`). When BE lands, the BE branch sets
  `LJ_TARGET_GC64` too and selects the legacy descriptor/TOC paths in the dasc.

Verify (with a cross-compiler):
```sh
powerpc64le-linux-gnu-gcc -E -dM src/lj_arch.h | grep -E 'LJ_TARGET_PPC |LJ_TARGET_GC64 |LJ_GC64 |LJ_FR2 |LJ_ARCH_BITS |LJ_ARCH_NAME'
# expect: LJ_TARGET_PPC 1, LJ_TARGET_GC64 1, LJ_GC64 1, LJ_FR2 1, LJ_ARCH_BITS 64, "ppc64le"
```
Optional follow-ups in this file (not required to build): set
`LJ_ARCH_VERSION` from `_ARCH_PWR8/9/10`, and confirm `LJ_ARCH_NUMMODE` — the existing
`LJ_NUMMODE_DUAL_SINGLE` defaults to dual-number, which is what the interpreter/asm
specs assume (arm64 hard-requires dualnum). Leave as-is unless you want to hard-pin
`LJ_NUMMODE_DUAL`.

## 0.2 — `src/Makefile`  (NO change required for detection)

Already correct for ppc64:
- `Makefile:252` matches any `LJ_TARGET_PPC` → `TARGET_LJARCH = ppc`, and `:253-257`
  sets `-DLJ_ARCH_ENDIAN=…` from `LJ_LE`.
- `Makefile:414-415` adds `-D P64` to `DASM_AFLAGS` when `LJ_ARCH_BITS 64`, and
  `:409-412` adds `-D ENDIAN_LE|BE`. So DynASM is invoked in 64-bit mode automatically.

Optional/recommended:
- `CCOPT_ppc=` (`Makefile:53`) is empty — leave empty to use the compiler's default
  `-mcpu`, or set `-mcpu=power8` to pin the ISA 2.07 baseline for reproducible builds.
  Do **not** hardcode `power9`/`power10` here (build-time ISA gating per the main plan).
- FreeBSD: add OS glue under the `TARGET_SYS` handling if/when targeting it (Phase 5.4).

## 0.3 — CI (smoke)

Three runners, build + `luajit -v`:
- `qemu-ppc64le` (Linux, primary) — `powerpc64le-linux-gnu-` cross toolchain.
- `qemu-ppc64` (Linux, big-endian **ELFv1**) — `powerpc64-linux-gnu-` (BE ppc64 uses
  the traditional ELFv1 descriptor/TOC ABI).
- FreeBSD ppc64le (Phase 5; can be a non-blocking lane initially).

## 0.4 — Minimal stub set to LINK a binary

With 0.1 applied, the build now proceeds past `lj_arch.h` and pulls in the PPC backend
files — which are 32-bit and will fail to compile under GC64. To get a *linking*
binary whose interpreter traps (the Phase 0 target), each arch file needs a 64-bit
skeleton. Smallest viable set:

| File | Phase 0 stub work |
|------|-------------------|
| `lj_target_ppc.h` | already 64-bit-clean (`intptr_t gpr[]`, 32-bit paired spill like arm64/mips64) — likely compiles as-is under GC64; reconcile `SPS_FIXED`/`SPOFS_*` with the dasc frame later. |
| `lj_emit_ppc.h` | provide 64-bit `emit_loadi64`, `emit_loadofs/storeofs` (LD/STD), `emit_call`; can be partial — only what the asm skeleton references. |
| `lj_asm_ppc.h` | every `asm_*` handler may be a `lj_assertA(0,"NYI")`/`lua_assert` stub **except** the scaffolding the trace compiler always calls (`asm_setup_target`, `asm_*_fixup`, `asm_exitstub_setup`, `asm_guardcc`, `asm_mcode_fixup`). Easiest: keep JIT effectively unused at first (traces won't form until `vm_ppc.dasc` hotcounters call in). |
| `vm_ppc.dasc` | the long pole — but for *linking*, it must assemble. Start from a copy whose opcodes are `NYI` (`trap`) except the ELFv2 prologue, `lj_vm_call`/`vm_returnc`, and `ins_NEXT`, enough to run `print()` (see `ppc64-port-interp-abi.md` §8 milestone 1). DynASM must accept the 64-bit `.dasc`. |
| `dasm_ppc.lua` | verify it assembles the `.dasc` in `P64` mode; add `.localentry`/`@ha`/`@l` if the prologue uses them (see interp-abi §7). |

Realistically, **0.4 overlaps Phase 1** — there is no "links but does nothing" state
that's much cheaper than "runs `print()`". Recommended sequencing:
1. Apply 0.1 (done) + 0.3 CI scaffold.
2. Stub `lj_target_ppc.h`/`lj_emit_ppc.h`/`lj_asm_ppc.h` enough to **compile**.
3. Bring up `vm_ppc.dasc` to the `print()` milestone (interp-abi §2-§3, §8).
That is the true Phase 0→1 boundary; everything after is Phase 1 per the main plan.

## Current working-tree state

- `src/lj_arch.h`: **edited** (0.1, applied, uncommitted). All other files unchanged.
- Building for any non-PPC64 target is unaffected.
- Building for ppc64le/ppc64-ELFv2 now advances past arch detection and will fail in
  the 32-bit backend files until the 0.4 skeleton lands — the intended next step.
