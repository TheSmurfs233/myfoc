#!/bin/bash
# =============================================================================
#  Host unit tests for the myfoc firmware logic modules.
#
#  Builds the pure-logic part of User/** with a fake HAL against the host GCC
#  shipped with Vitis_HLS (msys2 mingw64), then runs the test binary.
#
#  Usage:  test/host/build.sh          (from anywhere)
# =============================================================================
set -u

GCC_BIN="/d/Xilinx/Vitis_HLS/2022.2/tps/win64/msys64/mingw64/bin"
export PATH="$GCC_BIN:$PATH"

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

if ! command -v gcc >/dev/null 2>&1; then
    echo "ERROR: host gcc not found. Expected at $GCC_BIN" >&2
    exit 1
fi

INC="-I$HERE/fake -I$ROOT/User/Cfg -I$ROOT/User/Bsp -I$ROOT/User/Mid -I$ROOT/User/Alg -I$ROOT/User/App"
SRC="$HERE/test_main.c
     $HERE/fake/hal_stub.c
     $ROOT/User/Mid/mid_ringbuf.c
     $ROOT/User/Mid/mid_log.c
     $ROOT/User/Mid/mid_vofa.c
     $ROOT/User/Mid/mid_current.c
     $ROOT/User/Mid/mid_trig.c
     $ROOT/User/Mid/mid_enc.c
     $ROOT/User/Alg/alg_svpwm.c
     $ROOT/User/Alg/alg_vf.c
     $ROOT/User/Alg/alg_pi.c
     $ROOT/User/Alg/alg_foc.c
     $ROOT/User/Alg/alg_spd.c
     $ROOT/User/Alg/alg_pll.c
     $ROOT/User/Bsp/bsp_gpio.c
     $ROOT/User/Bsp/bsp_pwm.c
     $ROOT/User/Bsp/bsp_adc.c
     $ROOT/User/Bsp/bsp_prof.c
     $ROOT/User/Bsp/bsp_vbus.c
     $ROOT/User/Bsp/bsp_sys.c
     $HERE/fake/bsp_flash_fake.c
     $HERE/fake/bsp_ssi_fake.c
     $HERE/fake/bsp_cordic_fake.c
     $ROOT/User/Bsp/bsp_uart.c
     $ROOT/User/App/app_main.c
     $ROOT/User/App/app_cmd.c"

OUT="$HERE/run_tests.exe"

echo "compiling..."
# shellcheck disable=SC2086
gcc -std=gnu99 -O1 -Wall -Wextra -Wno-unused-parameter $INC $SRC -o "$OUT" || exit 1

echo "running..."
"$OUT"
RC=$?
exit $RC
