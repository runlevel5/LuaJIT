# PPC64 ELFv2 — unwinding & `.eh_frame` (Phase 5.1)

Companion to the other `ppc64*` docs. Covers DWARF2 stack unwinding for ppc64le /
ppc64-ELFv2: the interpreter `.eh_frame`, the JIT-code unwind template, the C-side
personality + landing pads, and build detection. Grounded in `lj_err.c`,
`vm_ppc.dasc:6090-6214` (hand-written `.eh_frame`), and `lj_arch.h`.

## 0. Why this matters

Errors propagate as a forced unwind (`error()`, `lua_error`, FFI C++ exceptions). The
external (system `libgcc`/`libunwind`) unwinder must be able to walk **through** the
interpreter C frame and JIT mcode frames to reach the right `pcall` landing pad. That
requires correct DWARF unwind info for: (a) the interpreter frame, (b) each trace's
mcode, and (c) `lj_vm_ffi_call`. Get the frame description wrong and `pcall` across a
trace or FFI boundary crashes instead of catching.

## 1. Architecture (three cooperating pieces)

| Piece | Where | Role |
|---|---|---|
| **C personality + driver** | `lj_err.c` (`lj_err_unwind_dwarf`, `err_unwind`, `err_unwind_jit`) | arch-neutral; decides catch/continue, sets IP/GR to a landing pad |
| **Interpreter `.eh_frame`** | hand-written GAS in `vm_ppc.dasc` (emitted into `lj_vm.S`) | describes the interpreter CFRAME so the unwinder can traverse it |
| **JIT-code `.eh_frame`** | `err_frame_jit_template[]` in `lj_err.c`, registered per-mcode via `__register_frame` | describes trace frames; the personality always installs a side-exit context |
| **VM landing pads** | `->vm_unwind_c_eh`, `->vm_unwind_ff_eh`, rethrow (`vm_ppc.dasc:639/654/3100`) | where the unwinder resumes to return from `pcall`/ff-`pcall` |

Config gating (already correct for PPC64): `LJ_TARGET_EHRETREG=3` (r3, error-code
return reg), `LJ_TARGET_EHRAREG=65` (LR, the DWARF return-address column) at
`lj_arch.h:358-359`. With external unwind + JIT, `LJ_UNWIND_JIT=1` (PPC is not excluded
at `lj_arch.h:728`).

## 2. Build detection (`src/Makefile`) — no PPC change expected

`Makefile:339-343` compiles a probe and greps the object for `eh_frame`; if the
toolchain emits unwind tables it defines `LUAJIT_UNWIND_EXTERNAL`. GCC/Clang on ppc64
ELFv2 emit `.eh_frame` by default (`-funwind-tables`), so external unwind is
auto-detected. If a toolchain doesn't, add `-funwind-tables` (as the arm64 line 325
forces `LUAJIT_UNWIND_EXTERNAL`). `LJ_NO_UNWIND` is **not** set for ppc64-ELFv2 (the
exclusions at `lj_arch.h:718` are iOS/PS3/PS4/PS5/Symbian), so unwinding is on.

## 3. Interpreter `.eh_frame` — the main rewrite

The existing emitter (`vm_ppc.dasc:6090-6214`) hand-writes CIE/FDE GAS directives for a
**32-bit ELFv1** frame. It must be rewritten for the **64-bit ELFv2 CFRAME** (layout in
`ppc64-port-interp-abi.md` §2). The mechanical transformation:

### 3.1 CIE (`.Lframe1`/CIE1, `vm_ppc.dasc:6147-6163`)
- **Data alignment factor `-4` → `-8`** (`.sleb128 -4` → `.sleb128 -8`). This is the
  single most important change: every factored offset below is in units of this.
- Code alignment factor stays `1`; RA column stays `65` (LR).
- Augmentation `zPR`, personality `lj_err_unwind_dwarf` via `pcrel|sdata4` (`0x1b`):
  unchanged — `sdata4` PC-relative reaches within ±2GB, fine for 64-bit.
- Initial CFA rule `DW_CFA_def_cfa r1, 0` (`0x0c, uleb 1, uleb 0`): unchanged (CFA = SP
  at entry = caller SP; ELFv2 back-chain is at 0(CFA)).
- `.align 2` → `.align 3` (8-byte align the records on the 64-bit ABI).

### 3.2 FDE for the interpreter (FDE2, `vm_ppc.dasc:6164-6183`)
Recompute every factored offset against the new frame (saves are 8-byte `std`/`stfd`
slots, not 4-byte). Rules to emit, in CFA-relative byte terms divided by 8:
- `DW_CFA_def_cfa_offset CFRAME_SIZE` (`0x0e, uleb CFRAME_SIZE`) — the **new** 64-bit
  `CFRAME_SIZE` in bytes.
- **LR save**: `DW_CFA_offset_extended_sf r65` — ELFv2 saves LR at `16(CFA)`, so the
  factored offset becomes `16 / -8 = -2` (`0x11, uleb 65, sleb -2`) — was `-1` for the
  ELFv1 `4(CFA)` slot. **This is a concrete, easy-to-miss change.**
- **CR save** (the `0x5, uleb 70, uleb 55` rule): only if CR2–CR4 are saved by the
  prologue; recompute the factored offset, and **verify the DWARF register number** for
  the saved CR field against the ppc64 ELFv2 DWARF reg assignment (LR=65, CTR=66,
  GPR 0–31, FPR 32–63; CR fields are in the 68–75 range — confirm before trusting the
  legacy `70`).
- **GPR r14–r31** (`DW_CFA_offset 0x80+i`) and **FPR f14–f31** (`0x80+32+i` →
  regs 46–63): recompute each factored offset = `(CFA_byte_offset_of_slot)/8` from the
  §2 `saveregs` layout. The legacy loop's `37+(31-i)` / `2+2*(31-i)` formulas are
  4-byte-slot arithmetic and must be regenerated for the 8-byte layout.

### 3.3 `lj_vm_ffi_call` FDE (CIE2/FDE3, `vm_ppc.dasc:6184-6213`)
Same treatment: data align `-8`, LR rule offset for ELFv2 (`sleb -2` for `16(CFA)`),
`zR` (no personality — this frame just needs to be walkable), recompute its small
prologue's CFA/LR/saved-reg offsets for the 64-bit frame.

### 3.4 The second emitter block (`.Lframe0`/FDE0, `vm_ppc.dasc:6090-6143`)
There are two hand-written blocks (per build mode / `LJ_HASFFI`). Apply the identical
−8 / 8-byte-slot / `sleb -2` transformation to both. Keep them in sync with `saveregs`.

### 3.5 Endianness
The hand-written GAS directives are endian-neutral (the assembler emits native byte
order), so **BE needs no special handling here** — unlike the JIT template (§4), which
hard-codes length-field byte order. The only LE/BE-sensitive interpreter spot remains
the bytecode decode, not `.eh_frame`.

## 4. JIT-code unwind (`err_frame_jit_template[]`, `lj_err.c:557-590`)

**Already 64-bit- and BE-parameterized** — minimal PPC work:
- The template is keyed on `LJ_64` (CIE/FDE lengths, code/data align `0x78` = SLEB −8)
  and `LJ_BE` (the length-field byte padding), and uses `LJ_TARGET_EHRAREG` (= 65)
  for the RA column. So the bytes come out correct for ppc64le and ppc64-BE.
- The personality `err_unwind_jit` **always installs a side-exit context** (sets IP to
  the trace's exit stub via `lj_trace_unwind`), so the FDE carries **no CFA/save
  opcodes** — meaning we do *not* need to hand-describe trace register saves. This is
  why JIT unwinding is nearly arch-neutral.
- **Verify** the template offsets `ERR_FRAME_JIT_OFS_HANDLER/FDE/CODE_SIZE`
  (`lj_err.c:592-599`) land on the right bytes for LJ_64 (they're computed from
  `LJ_64`, so should be correct — confirm with a built trace + `_Unwind_Find_FDE`
  assert at `lj_err.c:624`).
- `__register_frame`/`__deregister_frame` must exist in the platform's `libgcc`
  (they do on Linux/FreeBSD ppc64). No pointer-auth (`LJ_ABI_PAUTH` is arm64-only).
- The MIPS-style `lj_vm_unwind_stub` GR-setup (`lj_err.c:536-539`) is **not** used by
  PPC — PPC takes the `#else` branch (`_Unwind_SetIP(ctx, stub)` directly). Good.

## 5. VM landing pads (`vm_ppc.dasc`)

`lj_err_unwind_dwarf` redirects the unwinder to one of:
- `lj_vm_unwind_c_eh` (`->vm_unwind_c_eh`, `:639`) — resume returning from `lj_vm_pcall`.
- `lj_vm_unwind_ff_eh` (`->vm_unwind_ff_eh`, `:654`) — resume returning from a fast-func
  `pcall`.
- `lj_vm_unwind_rethrow` (`:3100`, "rethrow from the right C frame").

These must restore the **64-bit ELFv2 frame** correctly (reload nonvolatiles from the
8-byte save slots, restore SP from the back-chain/`L->cframe`, `mtlr`, set r3 =
errcode = `EHRETREG`). They are reached with the unwinder having set `r3` (errcode) and
IP; the pad continues as a normal `restoreregs`+return. Port from arm64
(`lj_vm_unwind_*` in `vm_arm64.dasc`) using the §2 frame.

## 6. Validation

- `pcall` catching `error()` raised in: plain interpreter code; a JIT trace; a C
  function called via FFI (C++ `throw` interop is NYI — same as other arches,
  `lj_err.c:522`).
- Error thrown **across** a trace mcode frame → must hit `err_unwind_jit` and side-exit
  (assert `g->jit_base` path, `lj_err.c:533`).
- Deep nested `pcall`/`xpcall`, `coroutine` resume/yield across frames.
- Build with `LUA_USE_ASSERT` to arm the `_Unwind_Find_FDE` checks
  (`lj_err.c:509/512/624`) — they catch missing/broken unwind tables immediately.
- Run on **both** ppc64le and ppc64-BE (the JIT template's `LJ_BE` path) and on
  FreeBSD (different `libgcc`/unwind backend) in CI/real hardware.

## 7. Net task list

1. Rewrite the interpreter `.eh_frame` emitter (`vm_ppc.dasc`, both blocks): data align
   `-8`, `CFRAME_SIZE`, LR `sleb -2`, recompute all GPR/FPR/CR factored offsets for the
   8-byte ELFv2 layout, verify CR DWARF reg number, `.align 3`. (Main work.)
2. Port the VM landing pads (`vm_unwind_c_eh`/`ff_eh`/rethrow) to the 64-bit frame.
3. Verify the JIT `err_frame_jit_template` offsets/EHRAREG (expected: correct as-is).
4. Confirm `LUAJIT_UNWIND_EXTERNAL` auto-detection; else add `-funwind-tables`.
5. Validate per §6; arm asserts via `LUA_USE_ASSERT`.

When stuck on the exact factored offsets / CR reg number, cross-check the reference
ports (master plan "Reference implementations") — but verify against the ELFv2 DWARF
register assignment, since their frame layout differs from this port's.
