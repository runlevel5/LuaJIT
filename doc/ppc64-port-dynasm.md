# PPC64 ELFv2 — DynASM & build-pipeline assessment

Companion to the other `ppc64*` docs. Corrects and narrows the DynASM-related tasks
mentioned earlier (`ppc64-port-asm.md` §7, `ppc64-port-interp-abi.md` §7) after reading
the actual tooling. **Headline: DynASM needs far less work than the plan implied.**

## 1. The two-tool split (the key clarification)

There are two independent code generators in LuaJIT, and DynASM is only one of them:

| | What | Uses DynASM? | Files |
|---|---|---|---|
| **Interpreter** | `vm_ppc.dasc` assembled once at **build time** by the host tool `buildvm` | **Yes** — `dasm_ppc.lua` (generator) + `dasm_ppc.h` (encoder), driven by `host/buildvm.c` (`dasm_init/setup/link/encode`) | `dynasm/dasm_ppc.*`, `src/host/buildvm*.c` |
| **JIT backend** | runtime trace machine code | **No** — emits words directly via `emit_*` into `as->mcp` | `lj_emit_ppc.h`, `lj_asm_ppc.h` |

Confirmed: `lj_asm_ppc.h`/`lj_mcode.c`/`lj_jit.h` do **not** include any `dasm_*`
header. So:
- DynASM changes only affect the **interpreter** build.
- **ISA 3.1 prefixed instructions are a JIT concern, not a DynASM concern** (see §4).

## 2. DynASM PPC module status

`dasm_ppc.lua` is v1.5.0 (2021), titled "PPC/PPC64 module". It already encodes:
- 64-bit ops (`ld`/`std`/`ldx`/`stdx`, `rldic*` via `parse_shiftmask`, `sradi`, `cmpd`/
  `cmpld`, `mulld`, `fcfid`/`fctid(z)`, `mtvsrd`/`mfvsrd`, `isel`) — all in the op
  template tables.
- Full **VSX** (`xv*`/`xs*`) and **AltiVec** (`v*`).
- `P64` mode is plumbed through (`vm_ppc.dasc` already gates on `.if P64`), and the
  Makefile passes `-D P64` automatically for 64-bit (`Makefile:414`).
- Relocations (`dasm_ppc.h` encoder): 26-bit `b`/`bl` (±32MB, mask `0x03fffffc`) and
  16-bit `bc` — sufficient for the interpreter's internal/external branches.

**Verification task (not a rewrite):** assemble the rewritten `vm_ppc.dasc` in `P64` +
`ENDIAN_LE`/`ENDIAN_BE` and confirm every instruction the new interpreter uses is in
the op tables; add any missing baseline-2.07 op (unlikely). The `~`/`=`/`%` template
operators and the `decode_OP8`/`decode_OPP` macros for P64 already exist.

## 3. ELFv2 linkage — handled by the assembler/linker, not DynASM

`buildvm` emits the interpreter as a `.S` file (`BUILD_elfasm`). External calls
(`bl extern foo` in the `.dasc`) become a literal `bl foo` assembler directive
(`buildvm_asm.c:143-153`, primary opcodes 18=`b/bl`, 16=`bc/bcl`). The **system
assembler + linker resolve these**, inserting ELFv2 PLT call stubs and r2/TOC handling
as needed. Consequences:

- **Function descriptors:** ELFv2 (ppc64le) has **none** — the non-PS3 path
  (`TOCPREFIX ""`, `bl sym`) is already correct, no change. **Big-endian ppc64 uses
  ELFv1**, which *does* use `.opd` descriptors + per-call TOC — that is exactly the
  existing PS3-style machinery (`TOCPREFIX "."`, `buildvm_asm.c:138-142`/`176-194`,
  and the `.toc`/descriptor-deref in `vm_ppc.dasc`). So BE/ELFv1 **reuses** it; the
  buildvm `.opd` emission just needs its guard broadened from `LJ_TARGET_PS3` to
  also cover big-endian ELFv1 ppc64.
- **`@ha`/`@l`/`.TOC.` data relocations are NOT needed.** The interpreter reaches
  `global_State` through the `DISPATCH`/`JGL` register (passed in), not TOC-relative
  data. So the missing TOC-reloc support in `dasm_ppc.lua` is irrelevant here.
- `emit_asm_label` emits `.globl`/`.hidden`/`.type … function`/`.size` — all valid for
  ELFv2. No `.opd`, no descriptor symbol.

### Global vs local entry (the one real ELFv2 subtlety)
VM functions called **from C** (e.g. `lj_vm_call`) are entered at the **global entry**;
that entry must compute `r2` from `r12` (`addis/addi r2,r12,.TOC.-func@ha/@l`). This
prologue lives in **`vm_ppc.dasc`** (`ppc64-port-interp-abi.md` §2), not DynASM.

`.localentry sym, .-sym` (which lets callers skip the r2 setup) is a **pure
optimization**. `buildvm`/DynASM can't currently emit it, and **omitting it is
correct**: external callers just always run the 2-instruction global-entry prologue
(the linker's call stub sets `r12`). Internal `bl ->label` branches stay within the
blob with `r2` already valid — no prologue per internal label.

Optional later optimization: teach `emit_asm_label` (or a `.dasc` directive) to emit
`.localentry name, 8` for `lj_vm_*` exports. Low priority; not Phase 1.

## 4. ISA 3.1 prefixed instructions — a JIT/emitter concern

Prefixed instructions (`pli`, `paddi`, `pld`, `pstd`) are 8 bytes (prefix word +
suffix word) and **must not span a 64-byte instruction boundary**. Where this matters:

- **JIT backend (`lj_emit_ppc.h`)**: emits prefixed ops by writing two words directly.
  The 64-byte-boundary guard is implemented **here / in the mcode layout**, not in
  DynASM — e.g. in the prefixed-emit helper, check `(uintptr_t)mcp & 63` and insert a
  `nop` (`ori 0,0,0`) when the prefix would straddle the boundary. This is gated on
  `LJ_ARCH_VERSION >= 31` (build-time).
- **Interpreter (`vm_ppc.dasc`)**: built for the 2.07 baseline, so it **does not use**
  prefixed instructions. Therefore `dasm_ppc.lua` does **not** need prefixed-op support
  for this port. (Only add it if a future change wants prefixed ops in the `.dasc` —
  out of scope.)

This supersedes `ppc64-port-asm.md` §7 / interp-abi §7, which over-attributed prefixed
support to DynASM.

## 5. Unwinding / `.eh_frame` (real gap, but not DynASM)

`buildvm_asm.c` emits only `.gnu_attribute 4,1` (hard-float marker) for PPC; it does
**not** emit CFI for the interpreter frame. For ELFv2, external unwinding (C↔Lua↔C
exceptions, `lj_err.c`) needs correct unwind info across the VM frame. Handle in
`vm_ppc.dasc` (CFI directives / the LuaJIT unwind mechanism) + `lj_err.c` — **Phase
5.1**, tracked in the main plan, not here.

## 6. Net task list for DynASM/build (revised, much smaller)

1. **Verify** `dasm_ppc.lua` assembles the rewritten `vm_ppc.dasc` in `P64` +
   `ENDIAN_LE`/`BE`; add any missing baseline op (expected: none/few).
2. **Confirm** `buildvm_asm.c` ELFv2 path: non-PS3 branch reloc emits `bl sym`
   (already true); no `.opd`; `.type function`/`.size` valid. Likely **no change**.
3. **(JIT, not DynASM)** implement the prefixed-op 64-byte-boundary guard in
   `lj_emit_ppc.h`, gated on `LJ_ARCH_VERSION >= 31`.
4. **(Optional)** `.localentry` emission for `lj_vm_*` exports — perf only.
5. **(Phase 5)** interpreter CFI/`.eh_frame` for ELFv2 unwinding.

Items previously feared (TOC relocs, function descriptors, prefixed-op assembler
support, ELFv2 reloc types in DynASM) are **not required** for a correct port.
