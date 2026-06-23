# PPC64 ELFv2 — interpreter & FFI ABI deep dive

Companion to `ppc64le-port.md`. Implementation-ready spec for the two
highest-risk areas: the **Phase 1 interpreter** (`vm_ppc.dasc`) and the
**ELFv2 FFI ABI** (`lj_ccall.*`, `lj_ccallback.c`). The structural template is the
arm64 backend; instruction selection comes from POWER + `dasm_ppc.lua`.

All asm sketches below are *illustrative encodings to be finalized*, not final code.

---

## 1. Register assignment (ELFv2, GC64)

ELFv2 ABI register roles: `r0` scratch (0 in addressing), `r1`=SP, `r2`=TOC,
`r3–r10` arg/ret (volatile), `r11`/`r12` volatile (`r12`=call target for global
entry), `r13` reserved (TLS), `r14–r31` nonvolatile; `f0` volatile, `f1–f13`
arg/volatile, `f14–f31` nonvolatile; `CR2–CR4`, `LR`, `CTR`.

Interpreter fixed registers (must match `lj_target_ppc.h` `RID_*`):

| Role            | Reg  | `RID_*`         | Notes |
|-----------------|------|-----------------|-------|
| Interp BASE     | r14  | `RID_BASE`      | nonvolatile |
| Interp PC       | r16  | `RID_LPC`       | |
| DISPATCH base   | r17  | `RID_DISPATCH`  | GL reached via `DISPATCH - GG_DISP2G` |
| lua_State `L`   | r18  | `RID_LREG`      | |
| `global_State`  | (via r17) |            | or pin a GLREG if a reg is free |
| On-trace JGL    | r31  | `RID_JGL`       | `global_State + 32768` |
| TMP/scratch     | r0, r11, r12 |          | r12 also = branch target reg |
| TISNUM/TISNIL constants | 2 nonvolatile GPRs | | mirror arm64 `TISNUM`/`TISNIL` |

Keep `r3–r10`/`f1–f13` as the C-call/arg scratch set (matches `RSET_SCRATCH`).
Confirm the chosen pins against `RSET_FIXED` in `lj_target_ppc.h`.

---

## 2. ELFv2 C frame (`vm_ppc.dasc` + `lj_frame.h`)

ELFv2 stack frame (16-byte aligned, grows down); leaf code may use the 288-byte
red zone without a frame:

```
 caller frame
 +------------------------+
 | param save area / TOC  |  [SP+32 ...]   (varargs / large args)
 | TOC save        (+24)  |  reserved
 | LR save         (+16)  |  callee stores caller LR here
 | CR save          (+8)  |  if CR2-4 modified
 | back chain       (+0)  |  <- SP after stdu
 +------------------------+
```

Proposed interpreter CFRAME (analogue of arm64 `CFRAME_SPACE=208`,
`vm_arm64.dasc:108-123`) — finalize sizes, must match `lj_frame.h` `CFRAME_*`:

| Slot                    | Offset from SP | Width |
|-------------------------|----------------|-------|
| back chain              | 0              | 8 |
| CR save                 | 8              | 8 |
| LR save (caller's)      | 16             | 8 |
| SAVE_CFRAME / SAVE_PC / SAVE_L | low fixed slots | 8 each |
| SAVE_NRES/ERRF/MULTRES  | 32-bit fixed slots | 4 each |
| TMPD (FP↔int bounce, conversions) | 8-byte aligned | 8 |
| GPR saves r14–r31       | block          | 18×8 |
| FPR saves f14–f31       | block          | 18×8 |

Prologue (`saveregs`, mirrors `vm_arm64.dasc:160-170`):
```
mflr  r0
std   r0, 16(r1)            ; LR into caller frame
stdu  r1, -CFRAME_SPACE(r1) ; frame + back chain
mfcr  r0; stw r0, 8+CFRAME_SPACE(r1)   ; if CR2-4 used
std   r14, SAVE_GPR+0(r1)   ; ... r15..r31
stfd  f14, SAVE_FPR+0(r1)   ; ... f15..f31
```
Epilogue (`restoreregs`): reload GPR/FPR/CR, `ld r0,16+CFRAME_SPACE(r1)`,
`addi r1,r1,CFRAME_SPACE`, `mtlr r0`, `blr`.

Entry points (every globally-visible VM symbol called from C — e.g. `lj_vm_call`):
emit the ELFv2 dual entry so internal calls can use the **local entry** and skip
TOC setup:
```
func:               ; global entry (r12 = func)
  addis r2, r12, .TOC.-func@ha
  addi  r2, r2, .TOC.-func@l
  .localentry func, .-func   ; local entry starts here; r2 already valid
  ...
```
DynASM must support `.localentry`/`@ha`/`@l` relocations (verify in `dasm_ppc.lua`;
add if missing — see §6).

---

## 3. Macro translation: arm64 → PPC64 (the hot path)

Each row is one `vm_arm64.dasc` macro and its PPC64 realization. Endianness is
parameterized via `ENDIAN_LE`/`OFS_*` exactly as arm64 (`vm_arm64.dasc:209-221`).

| Macro | arm64 | PPC64 sketch |
|-------|-------|--------------|
| `ins_NEXT` | `ldr INSw,[PC],#4; add TMP1,GL,INS,uxtb #3; ldr TMP0,[TMP1,#GG_G2DISP]; br_auth` | `lwz INS,0(PC); addi PC,PC,4; rlwinm TMP1,INS,3,21,28; ldx TMP0,DISPATCH,TMP1; mtctr TMP0; bctr` |
| `decode_RA` (b8..15) | `ubfx dst,ins,#8,#8` | `rlwinm dst,ins,8+... ` (rotate so byte→[24,31], mask) |
| `decode_RD` (b16..31)| `ubfx dst,ins,#16,#16`| `rlwinm dst,ins,16,16,31` |
| `decode_RC8RD` | `ubfiz dst,src,#3,#8` | `rlwinm dst,src,3,21,28` |
| `checktp r,tp` | `asr ITYPE,reg,#47; cmn ITYPE,#-tp; and reg,reg,#GCVMASK; bne` | `sradi ITYPE,reg,47; cmpdi ITYPE,tp; rldicl reg,reg,0,17; bne` |
| `checkint` | `cmp TISNUMhi,reg,lsr #32; bne` | `srdi TMP,reg,32; cmpw TISNUMhi,TMP; bne` |
| `checknum` | `cmp TISNUMhi,reg,lsr #32; bls` | `srdi TMP,reg,32; cmplw TISNUMhi,TMP; ble` |
| pointer mask `&GCVMASK` | `and reg,reg,#GCVMASK` | `rldicl reg,reg,0,17` (keep low 47 bits) |
| `mov_nil`/`cmp_nil` | `mov reg,TISNIL` | `mr reg,TISNIL` / `cmpd reg,TISNIL` |
| `mov_false`/`mov_true` | `movn ...` | materialize the 64-bit tag pattern (tiered loader) |
| `add_TISNUM` | `add dst,src,TISNUM` | `add dst,src,TISNUM` |
| `init_constants` | `movn/movz` | `lis/ori/rldicr` build of `TISNIL`/`TISNUM`/`TISNUMhi` |
| `hotcheck` | `ldrh/subs/strh` on dispatch hotcount | `lhz/subi/sth` at `DISPATCH+GG_DISP2HOT` (PC-derived index) |
| `barrierback` | `ldr/and/str/strb/str` | `ld/rldicl(clr BLACK)/std/stb/std` on `GL->gc.grayagain` |
| `ins_call`/`ins_callt` | store PC, load proto pc, dispatch | same shape with `std PC,FRAME_PC(BASE)`; `ld PC,LFUNC->pc` |
| `NYI` | `brk` | `trap` (tw 31,0,0) |

GC64 invariants to preserve everywhere: tags live in bits [47..63] (sign-extended
type via `sradi …,47`), pointers are the low 47 bits (`rldicl …,0,17`), `TISNUM`
tag added to integers. These mirror arm64 exactly — copy its logic, not the 32-bit
ppc logic.

---

## 4. Phase 1 per-bytecode checklist (`vm_ppc.dasc`)

Implement against the `BC_*` list in `lj_bc.h` (≈100 ops). Group / order:

1. **Comparison** (8): `ISLT ISGE ISLE ISGT ISEQV ISNEV ISEQS ISNES ISEQN ISNEN
   ISEQP ISNEP` — CR + `isel`/branch; dual-number int vs num paths.
2. **Unary test/copy** (6): `ISTC ISFC IST ISF ISTYPE ISNUM` then `MOV NOT UNM LEN`.
3. **Arithmetic** (binop VV/VN/NV + unary): `ADDVV SUBVV MULVV DIVVV MODVV POW CAT`
   and the `*VN`/`*NV` immediate/const forms — dual-number fast int path + FP path
   (FP via hardware FPU; int overflow → fallback).
4. **Constant/upvalue/global** : `KSTR KCDATA KSHORT KNUM KPRI KNIL`,
   `UGET USETV USETS USETN USETP UCLO`, `GGET GSET`.
5. **Table** : `TGETV TGETS TGETB TGETR TSETV TSETS TSETB TSETM TSETR TNEW TDUP`.
6. **Calls/returns** : `CALL CALLM CALLMT CALLT ITERC ITERN VARG ISNEXT`,
   `RET RET0 RET1 RETM` — FR2 2-slot frames, `ins_call`/`ins_callt`.
7. **Loops/branches** : `FORI JFORI FORL IFORL JFORL ITERL IITERL JITERL LOOP
   ILOOP JLOOP JMP` — `hotloop`/`hotcall` hooks into JIT.
8. **Func headers** : `FUNCF IFUNCF JFUNCF FUNCV IFUNCV JFUNCV FUNCC FUNCCW` —
   arg setup, stack check (`->vm_growstack_*`).

Also port the subroutine block (`build_subroutines`, `vm_arm64.dasc:361+`):
return handling (`vm_returnp/returnc/return`), `vm_call*`, `vm_pcall`, `vm_resume`,
`vm_growstack_*`, hook dispatch, `vm_exit_handler`/`vm_exit_interp` (trace exits →
restore `ExitState`), `vm_hotloop`/`vm_hotcall`, `vm_record`, fast-function block
(`fff_*`), and the FFI call helper `lj_vm_ffi_call`.

**Phase 1 exit:** suite passes `-joff` on ppc64le **and** ppc64 (BE).

---

## 5. ELFv2 FFI ABI (`lj_ccall.c` / `lj_ccall.h`)

### 5.1 Difference from the current PPC (ELFv1) block

The current `LJ_TARGET_PPC` block (`lj_ccall.c:373-430`) is ELFv1/SysV:
- `CCALL_HANDLE_STRUCTRET` returns **all** structs by reference.
- `CCALL_HANDLE_STRUCTARG` copies every struct to a cdata and passes **by reference**.

ELFv2 is register-based, like arm64:
- **Homogeneous aggregates** of ≤8 identical FP/vector members pass in consecutive
  FPRs/VRs (HFA/HVA). Other aggregates ≤16 bytes pass in up to 2 GPRs by value;
  >16 bytes go to the parameter save area (still by value, not reference).
- **Struct return**: ≤16 bytes (or HFA ≤8 members) returned in GPRs/FPRs; larger
  via hidden pointer in `r3` (sret), like arm64's `retp`/`x8` (PPC uses r3).

### 5.2 `lj_ccall.h` — new ELFv2 sub-block

Gate a new block on `LJ_TARGET_PPC && _CALL_ELF == 2` (keep the 32-bit/ELFv1 block):
```c
#define CCALL_NARG_GPR   8         /* r3-r10 */
#define CCALL_NARG_FPR   13        /* f1-f13 */
#define CCALL_NRET_GPR   2         /* r3:r4  */
#define CCALL_NRET_FPR   8         /* HFA up to 8 doubles in f1-f8 */
#define CCALL_SPS_FREE   0
#define CCALL_VECTOR_REG 1         /* if/when vector args supported */
typedef intptr_t GPRArg;
typedef union FPRArg { double d; struct { LJ_ENDIAN_LOHI(float f;,float g;) }; } FPRArg;
```
Add to `CCallState`: keep `nfpr`; HFA needs no extra return pointer field (sret
uses `gpr[0]`). Mind `LJ_ENDIAN_LOHI` for float-in-FPR placement on BE.

### 5.3 `lj_ccall.c` — ELFv2 `CCALL_HANDLE_*`

Port from arm64 (`lj_ccall.c:36-100`), adjusting the "FPRs always hold doubles"
rule the existing PPC block already encodes (`CCALL_HANDLE_REGARG`/`CCALL_HANDLE_RET`,
`:413-430` — keep this):
- `CCALL_HANDLE_REGARG`: HFA (≤8 same-FP-kind members) → consecutive FPRs; small
  aggregate → GPR pair; else → stack. Floats widened to double in FPRs.
- `CCALL_HANDLE_STRUCTARG`: pass small structs by value (drop the unconditional
  cdata-copy/by-reference behavior).
- `CCALL_HANDLE_STRUCTRET`/`STRUCTRET2`: classify HFA / ≤16-byte in regs; else sret
  pointer in `r3`.
- `CCALL_HANDLE_COMPLEXARG`/`RET`/`RET2`: complex float/double as 2-member HFA.
- `CCALL_HANDLE_GPR`: keep int64 regpair alignment? — ELFv2 does **not** require
  even-reg alignment for 64-bit args (unlike o32/ELFv1 some cases); verify and
  simplify accordingly.

### 5.4 `lj_vm_ffi_call` in `vm_ppc.dasc`

Marshal `CCallState`: set up the parameter save area + back chain, load `gpr[]`→r3–r10,
`fpr[]`→f1–f13, set `r12`=target (global-entry calling convention; must `mtctr`
+`bctrl` and reload `r2` from the TOC save slot after return), copy results back into
`cc->gpr`/`cc->fpr`. Mirror the arm64 `lj_vm_ffi_call` structure.

---

## 6. ELFv2 callbacks (`lj_ccallback.c`)

Add a `LJ_TARGET_PPC && _CALL_ELF==2` section (existing PPC at `:73`):
- Define `CALLBACK_MCODE_HEAD` (ELFv2 trampoline header incl. TOC/local-entry
  handling), `CALLBACK_MCODE_SLOTSZ`, `CALLBACK_MCODE_GROUP`.
- The per-slot stub loads the slot id and branches to the shared
  `lj_vm_ffi_callback` entry; the C side gathers r3–r10/f1–f13 into the callback
  arg buffer per §5 classification (reverse direction).
- Mirror arm64's slot math (`lj_ccallback.c:65-72`).

---

## 7. DynASM gaps to confirm/add (`dasm_ppc.lua`)

- `.localentry`, `@ha`/`@l`/`@got`/`.TOC.` relocations for ELFv2 dual entry.
- ISA 3.1 **prefixed** ops (`pli`, `paddi`, `pld`, `pstd`): 8-byte encoding, and a
  guarantee the prefix word never spans a 64-byte instruction boundary.
- Verify `P64` selects 64-bit forms; verify `ld/std/rldic*/cmpd/cmpld/sradi/srdi/
  isel/fcfid/fctidz/mtvsrd/mfvsrd` all encode.

---

## 8. Earliest end-to-end validation

To de-risk GC64/FR2 + ELFv2 ABI as early as possible:
1. After §2-§3, get `luajit -joff -e "print('hi', 1+2, 3.5*2)"` correct on LE and BE.
2. After §5, run an FFI smoke test calling `printf`/`pow`/a struct-returning C fn
   against a real `libc`/`libm` — catches ABI classification bugs immediately.
3. Then proceed to the full suite (`-joff`) before any JIT work.
