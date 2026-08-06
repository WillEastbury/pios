# Mesa V3D and Windows build changes

This file preserves the local Mesa changes that existed before the PC rebuild.
They were based on Mesa commit:

```text
410a764236d2918fba636330a978ce6ab53d22b2
```

The local commits were:

```text
3af0ef060f246785f73dc609d10df4a26d8d2a92 v3d: lower nir_op_extract_u8
cd41df6ff400b450d8991fec3af00710375a74c3 windows: fix Clang atomic and string compatibility
```

## Purpose

1. Add V3D compiler lowering for `nir_op_extract_u8`. The byte index is
   multiplied by eight, the source is shifted right, and the result is masked
   to eight bits.
2. Include Mesa's Windows `asprintf`/`vasprintf` compatibility definitions in
   the Broadcom VIR implementation.
3. Supply missing MSVC-style atomic aliases and add-return wrappers when
   building with Clang on Windows.

## Re-create the changes

Apply the patch below from the root of a Mesa checkout:

```powershell
git apply mesa-v3d-windows.patch
```

Alternatively, make the three edits manually from the diff.

```diff
diff --git a/src/broadcom/compiler/nir_to_vir.c b/src/broadcom/compiler/nir_to_vir.c
index e669ff180162760333b84885d143741b89d18f15..18efd723b48cd18f774196e1ae8f6eecd45189c5 100644
--- a/src/broadcom/compiler/nir_to_vir.c
+++ b/src/broadcom/compiler/nir_to_vir.c
@@ -1675,6 +1675,12 @@ ntq_emit_alu(struct v3d_compile *c, nir_alu_instr *instr)
         case nir_op_ixor:
                 result = vir_XOR(c, src[0], src[1]);
                 break;
+        case nir_op_extract_u8: {
+                struct qreg shift = vir_SHL(c, src[1], vir_uniform_ui(c, 3));
+                struct qreg byte = vir_SHR(c, src[0], shift);
+                result = vir_AND(c, byte, vir_uniform_ui(c, 0xff));
+                break;
+        }
         case nir_op_inot:
                 result = vir_NOT(c, src[0]);
                 break;
diff --git a/src/broadcom/compiler/vir.c b/src/broadcom/compiler/vir.c
index 27931f5da2e0afb1f1c7055adad40e02eb5dc261..fc5b48d19d49ea0eb9b5df0e19dfa83e2cea7308 100644
--- a/src/broadcom/compiler/vir.c
+++ b/src/broadcom/compiler/vir.c
@@ -28,6 +28,7 @@
 #include "compiler/nir/nir_builtin_builder.h"
 #include "compiler/nir/nir_format_convert.h"
 #include "util/perf/cpu_trace.h"
+#include "util/u_string.h"
 
 int
 vir_get_nsrc(struct qinst *inst)
diff --git a/src/util/u_atomic.h b/src/util/u_atomic.h
index 9f79113280a4b8545529825d2be1d2d23cdd5b7b..ba85412e0d78b042783875dcb5111ca088c6adb6 100644
--- a/src/util/u_atomic.h
+++ b/src/util/u_atomic.h
@@ -119,6 +119,13 @@
 #include <intrin.h>
 #include <assert.h>
 
+#if defined(__clang__)
+#define _interlockedexchange64 _InterlockedExchange64
+#define _interlockedexchangeadd64 _InterlockedExchangeAdd64
+#define _interlockeddecrement64 _InterlockedDecrement64
+#define _interlockedincrement64 _InterlockedIncrement64
+#endif
+
 __forceinline char _interlockedadd8(char volatile * _Addend, char _Value)
 {
    return _InterlockedExchangeAdd8(_Addend, _Value) + _Value;
@@ -129,6 +136,16 @@ __forceinline short _interlockedadd16(short volatile * _Addend, short _Value)
    return _InterlockedExchangeAdd16(_Addend, _Value) + _Value;
 }
 
+__forceinline long _interlockedadd(long volatile * _Addend, long _Value)
+{
+   return _InterlockedExchangeAdd(_Addend, _Value) + _Value;
+}
+
+__forceinline __int64 _interlockedadd64(__int64 volatile * _Addend, __int64 _Value)
+{
+   return _InterlockedExchangeAdd64(_Addend, _Value) + _Value;
+}
+
 /* MSVC supports decltype keyword, but it's only supported on C++ and doesn't
  * quite work here; and if a C++-only solution is worthwhile, then it would be
  * better to use templates / function overloading, instead of decltype magic.
```

## Validation status

`git diff --check` passed. A targeted Meson/Ninja build was attempted against a
Windows Clang build directory. Meson detected `clang-cl 22.1.8`, but the build
environment could not locate the Visual Studio/LLVM librarian during
reconfiguration (`lib.exe` or `llvm-lib.exe`), so full compilation was not
completed before the rebuild.

The V3D lowering uses existing `vir_SHL`, `vir_SHR`, `vir_AND`, and uniform
helpers already used in the same compiler.
