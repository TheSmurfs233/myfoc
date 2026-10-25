#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S2 acceptance test (serial side) against the real board.

S2 is mostly a scope job: 20 kHz, complementary pairs, dead time, output-off
state. This script covers everything that can be proven over the UART:

  - stat reports the PWM state / dead time actually programmed in BDTR
  - start/stop gating (OPENLOOP only, FOC rejected until S6)
  - pwm <d> / pwm <a> <b> <c> set + echo + clamp
  - stop restores the 50% zero vector and turns the bridge off

The scope checklist is printed at the end; run it with a probe on PA8/PA9/PA10
and PA7/PB0/PF0 (and PA11 for the CH4 trigger).

Usage:
    python test/accept/s2_accept.py [COM7] [baud]
"""

import sys
import os
import time
import re

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

RESULT = []


def check(name, ok, detail=""):
    RESULT.append((name, bool(ok), detail))
    print(("  PASS  " if ok else "  FAIL  ") + name + (("   " + detail) if detail else ""))
    return ok


class Link(object):
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.buf = ""
        try:
            self.ser.write(b"wave off\r\nstop\r\n")
        except Exception:
            pass
        self.drain(quiet=0.2, max_seconds=1.5)

    def drain(self, quiet=0.2, max_seconds=2.0):
        start = time.time()
        last = start
        while True:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n).decode("ascii", "replace")
                last = time.time()
            else:
                time.sleep(0.003)
            now = time.time()
            if (now - last) >= quiet:
                break
            if (now - start) >= max_seconds:
                break
        out, self.buf = self.buf, ""
        return out

    def send(self, cmd, wait=0.2):
        self.buf = ""
        self.ser.write((cmd + "\r\n").encode("ascii"))
        return self.drain(quiet=wait, max_seconds=wait + 0.8)

    def close(self):
        try:
            self.ser.write(b"stop\r\n")
            self.ser.write(b"mode 0\r\n")
        except Exception:
            pass
        self.ser.close()


def lines(text):
    return [ln.rstrip("\r") for ln in text.split("\n") if ln.strip() != ""]


def source_dtg():
    """PWM_DTG_REG from board_cfg.h so the check is not hard-coded."""
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", ".."))
    path = os.path.join(root, "User", "Cfg", "board_cfg.h")
    if not os.path.exists(path):
        return None
    m = re.search(r"#define\s+PWM_DTG_REG\s+(\d+)", open(path, "r").read())
    return int(m.group(1)) if m else None


def stat(lk):
    return lines(lk.send("stat"))


def duty_line(st):
    for x in st:
        if "pwm      =" in x:
            return x
    return ""


def main():
    print("=" * 62)
    print(" S2 acceptance test   port=%s  baud=%d" % (PORT, BAUD))
    print("=" * 62)

    want_dtg = source_dtg()

    try:
        lk = Link(PORT, BAUD)
    except Exception as exc:                      # noqa: BLE001
        print("cannot open %s: %s" % (PORT, exc))
        return 2

    # ---------- 0. clean baseline ----------
    lk.send("stop")
    lk.send("mode 0")

    # ---------- 1. bridge is off at rest ----------
    st = stat(lk)
    d = duty_line(st)
    check("stat has a pwm line", bool(d), d)
    check("bridge OFF at rest", "pwm      = OFF" in d, d)
    check("start duty is the 50% zero vector", "duty=0.50/0.50/0.50" in d, d)
    if want_dtg is not None:
        check("dead time in BDTR == PWM_DTG_REG (source)",
              ("dtg=%d" % want_dtg) in d, "%s  source=%s" % (d, want_dtg))

    # ---------- 2. duty is refused while stopped ----------
    out = lines(lk.send("pwm 0.5"))
    check("pwm refused while stopped", any("ERR start first" in x for x in out), str(out))
    out = lines(lk.send("pwm"))
    check("pwm with no args only reports", any("pwm = OFF" in x for x in out), str(out))

    # ---------- 3. FOC mode is refused until S6 ----------
    lk.send("mode 2")
    out = lines(lk.send("start"))
    check("start refused in mode=FOC", any("ERR mode=FOC not ready" in x for x in out), str(out))
    check("bridge still OFF after refused start",
          "pwm      = OFF" in duty_line(stat(lk)))

    # ---------- 4. OPENLOOP start ----------
    lk.send("mode 1")
    out = lines(lk.send("start"))
    check("start accepted in OPENLOOP", any("OK start, mode=OPENLOOP" in x for x in out), str(out))
    d = duty_line(stat(lk))
    check("bridge ON after start", "pwm      = ON" in d, d)
    check("duty unchanged by start (50%)", "duty=0.50/0.50/0.50" in d, d)

    # ---------- 5. single duty ----------
    out = lines(lk.send("pwm 0.25"))
    check("pwm 0.25 echoes the applied duty",
          any("OK duty = 0.25/0.25/0.25" in x for x in out), str(out))
    check("stat reflects 0.25", "duty=0.25/0.25/0.25" in duty_line(stat(lk)))

    # ---------- 6. three independent duties ----------
    out = lines(lk.send("pwm 0.1 0.5 0.9"))
    check("pwm 0.1 0.5 0.9 echoes the applied duties",
          any("OK duty = 0.10/0.50/0.90" in x for x in out), str(out))
    check("stat reflects the three duties",
          "duty=0.10/0.50/0.90" in duty_line(stat(lk)))

    # ---------- 7. clamping ----------
    out = lines(lk.send("pwm 2"))
    check("pwm 2 clamps to the high end (applied 0.99)",
          any("OK duty = 0.99/0.99/0.99" in x for x in out), str(out))
    out = lines(lk.send("pwm 0.001"))
    check("pwm 0.001 clamps to the low end (applied 0.01)",
          any("OK duty = 0.01/0.01/0.01" in x for x in out), str(out))

    # ---------- 8. bad arguments ----------
    out = lines(lk.send("pwm -1"))
    check("pwm -1 rejected", any("ERR bad duty" in x for x in out), str(out))
    out = lines(lk.send("pwm abc"))
    check("pwm abc rejected", any("ERR bad duty" in x for x in out), str(out))
    out = lines(lk.send("pwm 0.1 0.2"))
    check("pwm with two args rejected", any("ERR usage=pwm" in x for x in out), str(out))

    # ---------- 9. stop ----------
    out = lines(lk.send("stop"))
    check("stop accepted", any("OK stop, PWM off" in x for x in out), str(out))
    d = duty_line(stat(lk))
    check("bridge OFF after stop", "pwm      = OFF" in d, d)
    check("duty reset to the 50% zero vector", "duty=0.50/0.50/0.50" in d, d)
    out = lines(lk.send("stop"))
    check("double stop is harmless", any("OK stop" in x for x in out), str(out))
    check("state is IDLE at the end",
          any("state    = IDLE" in x for x in stat(lk)))

    lk.send("mode 0")
    lk.close()

    print("=" * 62)
    print(" ---- now verify on the scope (DUTY IS 50%, NO MOTOR) ----")
    print("  1. before start : PA8/PA9/PA10/PA7/PB0/PF0 all LOW")
    print("  2. mode 1+start : PA8 = 20.000 kHz +-0.1%, 50% high")
    print("  3. pwm 0.1 0.5 0.9 : three different duties, mapping A/B/C correct")
    print("  4. PA8 vs PA7   : complementary, dead time ~23.5ns at the MCU pins")
    print("                    (the ~60ns figure only applies at the EG3033 gates)")
    print("  5. PA10 vs PF0  : same (PF0 is the C-phase low side)")
    print("  6. PA11 / CH4   : one narrow pulse per period, at the counter peak")
    print("  7. stop         : all six LOW, no residual pulses over 10 start/stop")
    print("=" * 62)

    total = len(RESULT)
    fails = [r for r in RESULT if not r[1]]
    print(" checks=%d  failed=%d  ->  %s" % (total, len(fails), "PASS" if not fails else "FAIL"))
    print("=" * 62)
    for name, _ok, detail in fails:
        print("  FAILED: %s  %s" % (name, detail))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
