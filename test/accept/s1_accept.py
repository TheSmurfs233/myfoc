#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S1 acceptance test against the real board on COM7.

Checks the whole S1 contract:
  - help / ver / stat content and line counts
  - every text line starts with "[log] " and contains no ':' (FireWater rule)
  - echo / led / clr / rate clamping / error paths
  - mode -> start -> stop state machine flow and its rejections
  - wave on: FireWater CSV shape, frame rate, channel sanity, no drops

Usage:
    python test/accept/s1_accept.py [COM7] [baud]
"""

import sys
import os
import time
import re
import statistics
import datetime

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

RESULT = []          # (name, ok, detail)


def check(name, ok, detail=""):
    RESULT.append((name, bool(ok), detail))
    print(("  PASS  " if ok else "  FAIL  ") + name + (("   " + detail) if detail else ""))
    return ok


class Link(object):
    """Thin wrapper: every read has a hard deadline, so a board that is
    already streaming (wave on) can never hang the test."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.buf = ""
        # stop any stream the board may already be running
        try:
            self.ser.write(b"wave off\r\n")
        except Exception:
            pass
        self.drain(quiet=0.20, max_seconds=1.5)

    def drain(self, quiet=0.20, max_seconds=2.0, keep=False):
        """Read until the line has been quiet for `quiet` seconds, but never
        longer than `max_seconds` in total."""
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
        if keep:
            return self.buf
        out, self.buf = self.buf, ""
        return out

    def read_for(self, seconds):
        """Fixed-duration capture (used for the waveform burst)."""
        start = time.time()
        while (time.time() - start) < seconds:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n).decode("ascii", "replace")
            else:
                time.sleep(0.003)
        out, self.buf = self.buf, ""
        return out

    def send(self, cmd, wait=0.25):
        self.buf = ""
        self.ser.write((cmd + "\r\n").encode("ascii"))
        return self.drain(quiet=wait, max_seconds=wait + 0.6)

    def send_raw(self, data, wait=0.25):
        self.buf = ""
        self.ser.write(data)
        return self.drain(quiet=wait, max_seconds=wait + 0.6)

    def close(self):
        try:
            self.ser.write(b"wave off\r\n")
        except Exception:
            pass
        self.ser.close()


def lines(text):
    return [ln.rstrip("\r") for ln in text.split("\n") if ln.strip() != ""]


def is_csv(line):
    return re.fullmatch(r"-?\d+(\.\d+)?(,-?\d+(\.\d+)?)*", line) is not None


def main():
    print("=" * 62)
    print(" S1 acceptance test   port=%s  baud=%d" % (PORT, BAUD))
    print("=" * 62)

    try:
        lk = Link(PORT, BAUD)
    except Exception as exc:                      # noqa: BLE001
        print("cannot open %s: %s" % (PORT, exc))
        return 2

    # ---------- 0. clean baseline ----------
    lk.send("wave off")
    lk.send("stop")
    lk.send("mode 0")
    lk.send("led off")
    lk.send("clr")

    # ---------- 1. help ----------
    out = lk.send("help")
    ls = lines(out)
    check("help: 46 lines", len(ls) == 46, "got %d" % len(ls))
    check("help: first line", ls and ls[0] == "[log] ---- commands ----", ls[0] if ls else "-")
    check("help: last line", ls and ls[-1] == "[log] ------------------", ls[-1] if ls else "-")
    check("help: rate says 1..500",
          any("rate <hz>" in x and "1..500" in x for x in ls))
    check("help: pwm commands listed",
          any("pwm <d>" in x for x in ls) and any("pwm <a> <b> <c>" in x for x in ls))
    check("help: cur / cal / prof listed",
          any(" cur " in x for x in ls) and any(" cal " in x for x in ls) and
          any(" prof " in x for x in ls))
    check("help: vf / acc / dir listed",
          any(" vf " in x for x in ls) and any(" acc " in x for x in ls) and
          any(" dir " in x for x in ls))
    check("help: phase listed", any(" phase " in x for x in ls))
    check("help: trig listed", any(" trig " in x for x in ls))
    check("help: cordic listed", any(" cordic " in x for x in ls))
    check("help: enc listed", any(" enc " in x for x in ls))
    check("help: align listed", any(" align " in x for x in ls))
    check("help: all lines carry [log] marker", all(x.startswith("[log] ") for x in ls))

    # ---------- 2. FireWater text rule: no colon anywhere ----------
    colon = [x for x in ls if ":" in x]
    check("help: no ':' in any line (FireWater rule)", not colon, str(colon[:2]))

    # ---------- 3. ver ----------
    out = lk.send("ver")
    ls = lines(out)
    expect = [
        ("board", "[log] board = Board_v2/STM32G431KBT6"),
        ("core",  "[log] core  = 170000000 Hz"),
        ("foc",   "[log] foc   = 20000 Hz, Ts = 50 us"),
        ("pwm",   "[log] pwm   = 20000 Hz, ARR = 4249, DTG = 4"),
        ("cur",   "[log] cur   = Rs = 25 mOhm, gain = 50, 645 uA/LSB, range +-1320 mA"),
        ("vbus",  "[log] vbus  = div = 7, 5640 uV/LSB, max = 23100 mV"),
    ]
    check("ver: 8 lines", len(ls) == 8, "got %d" % len(ls))
    for _name, want in expect:
        check("ver: " + want.split()[1], want in ls)
    check("ver: build line present", any(x.startswith("[log] build = ") for x in ls))
    check("ver: uart says FireWater",
          any("VOFA+ = FireWater" in x for x in ls))
    bad_colon = [x for x in ls if ":" in x]
    check("ver: no ':' in any line (FireWater rule)", not bad_colon, str(bad_colon[:2]))
    check("ver: build timestamp uses ';' instead of ':'",
          any(x.startswith("[log] build = ") and ";" in x for x in ls),
          [x for x in ls if x.startswith("[log] build = ")][0]
          if any(x.startswith("[log] build = ") for x in ls) else "-")

    # ---------- 3b. firmware freshness ----------
    # The board reports FW_BUILD_ID ("build = ... #13"). Compare it with the
    # value in board_cfg.h on disk.
    #
    # Do NOT use __DATE__/__TIME__ for this: Keil builds incrementally, so the
    # timestamp freezes at whatever build last recompiled app_cmd.c and can be
    # minutes older than the .hex even when the board IS up to date.
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", ".."))
    cfg_path = os.path.join(root, "User", "Cfg", "board_cfg.h")
    bl = [x for x in ls if x.startswith("[log] build = ")]
    src_id = None
    if os.path.exists(cfg_path):
        m2 = re.search(r"#define\s+FW_BUILD_ID\s+(\d+)", open(cfg_path, "r").read())
        if m2:
            src_id = int(m2.group(1))
    if not bl or src_id is None:
        print("  (skip) firmware freshness: cannot read build line or board_cfg.h")
    else:
        m3 = re.search(r"#(\d+)\s*$", bl[0])
        board_id = int(m3.group(1)) if m3 else None
        check("firmware on board == source FW_BUILD_ID",
              board_id == src_id,
              "board=#%s  source=#%s" % (board_id, src_id)
              + ("" if board_id == src_id else "  -> RE-FLASH myfoc.hex"))

    # ---------- 4. stat ----------
    out = lk.send("stat")
    ls = lines(out)
    check("stat: 13 lines", len(ls) == 13, "got %d" % len(ls))
    check("stat: state IDLE", any("state    = IDLE" in x for x in ls))
    check("stat: mode IDLE", any("mode     = IDLE" in x for x in ls))
    check("stat: pwm line present and OFF", any("pwm      = OFF" in x for x in ls))
    check("stat: adc line present", any("adc      = frames=" in x for x in ls))
    check("stat: vf line present", any("vf       =" in x for x in ls))
    check("stat: no ':' in any line", not [x for x in ls if ":" in x])
    m = [x for x in ls if "loop     =" in x]
    check("stat: loop rate sane (100..900 kHz)",
          bool(m) and 100 <= int(m[0].split("=")[1].split()[0]) <= 900,
          m[0].split("=")[1].strip() if m else "-")

    # ---------- 5. echo ----------
    out = lk.send("echo hi")
    check("echo: returns [log] hi", lines(out) == ["[log] hi"], str(lines(out)))

    # ---------- 6. unknown command / long line ----------
    out = lk.send("foobar")
    check("unknown cmd rejected", any("ERR unknown cmd 'foobar'" in x for x in lines(out)))
    out = lk.send("x" * 90)
    out += lk.send("")
    check("over-long line rejected", any("ERR line too long" in x for x in lines(out)))

    # ---------- 7. led ----------
    out = lk.send("led on")
    check("led on", any("OK led ON" in x for x in lines(out)))
    st = lk.send("stat")
    check("led on reflected in stat", any("led      = ON" in x for x in lines(st)))
    lk.send("led off")
    st = lk.send("stat")
    check("led off reflected in stat", any("led      = OFF" in x for x in lines(st)))

    # ---------- 8. rate clamping ----------
    out = lk.send("rate 9999")
    check("rate 9999 clamped to 500", any("OK rate = 500 Hz" in x for x in lines(out)),
          str(lines(out)))
    out = lk.send("rate 300")
    check("rate 300 accepted", any("OK rate = 300 Hz" in x for x in lines(out)))
    lk.send("rate 500")

    # ---------- 9. state machine ----------
    out = lk.send("start")
    check("start rejected when mode=IDLE", any("ERR mode=IDLE" in x for x in lines(out)))
    out = lk.send("mode 9")
    check("mode 9 rejected", any("ERR mode must be 0/1/2" in x for x in lines(out)))
    out = lk.send("mode 2")
    check("mode 2 accepted", any("OK mode=FOC" in x for x in lines(out)))
    # S1 only had an open-loop mode and rejected a FOC start; since S6 the FOC
    # loop is the point, so this is the happy path now.
    out = lk.send("start")
    check("start accepted in mode=FOC",
          any("OK start, mode=FOC" in x for x in lines(out)))
    st = lk.send("stat")
    check("state RUN after start", any("state    = RUN" in x for x in lines(st)))
    check("pwm ON after start", any("pwm      = ON" in x for x in lines(st)))
    out = lk.send("mode 2")
    check("mode change blocked while RUN",
          any("ERR stop the motor first" in x for x in lines(out)))
    out = lk.send("stop")
    check("stop accepted", any("OK stop, PWM off" in x for x in lines(out)))
    st = lk.send("stat")
    check("state IDLE after stop", any("state    = IDLE" in x for x in lines(st)))
    check("pwm OFF after stop", any("pwm      = OFF" in x for x in lines(st)))
    check("mode persists after stop (by design)",
          any("mode     = FOC" in x for x in lines(st)),
          "stop only resets state, not mode")
    out = lk.send("mode 0")
    check("mode 0 accepted after stop", any("OK mode=IDLE" in x for x in lines(out)))

    # ---------- 10. waveform ----------
    lk.send("clr")
    lk.send("wave on")
    raw = lk.read_for(1.20)
    lk.send("wave off")
    st = lk.send("stat")

    all_lines = lines(raw)
    csv = [x for x in all_lines if is_csv(x)]
    logs = [x for x in all_lines if x.startswith("[log] ")]
    # Anything that is neither a frame nor a log line means the two streams got
    # interleaved, which is worth failing on.  A couple of fragments from torn
    # frames are tolerated for the same reason as below.
    other = [x for x in all_lines if (x not in csv) and (x not in logs)]
    check("wave: stream has no stray lines", len(other) <= 2, str(other[:3]))

    check("wave: CSV rows captured", len(csv) > 300, "rows=%d" % len(csv))

    # Every row must have exactly 21 fields: t, duty a/b/c, ia/ib/ic,
    # enc mech/elec/spd, id, iq, id_ref, iq_ref, vd, vq, vbus, win/pll rpm,
    # obs theta, obs-vs-enc error.
    #
    # A 21 channel frame at 500 Hz is about 137 bytes, i.e. 685 kbit/s on a
    # 1 Mbaud link.  The odd torn row on the host side (DAPLink VCP and the
    # Windows driver both buffer) is a link fact, not a firmware fault, so
    # torn rows are dropped and counted instead of failing every check below.
    good = [x for x in csv if len(x.split(",")) == 21]
    bad = [x for x in csv if len(x.split(",")) != 21]
    check("wave: frames are whole (>=95% of rows)",
          len(good) >= (len(csv) * 95 // 100),
          "good=%d torn=%d %s" % (len(good), len(bad), str(bad[:2])))

    rows = []
    for x in good:
        try:
            rows.append([float(v) for v in x.split(",")])
        except ValueError:
            pass
    if rows:
        n = len(rows)
        rate = n / 1.20
        check("wave: frame rate ~500/s", 420 <= rate <= 540, "%.0f /s" % rate)

        t = [r[0] for r in rows]
        mono = all(t[i + 1] > t[i] for i in range(len(t) - 1))
        check("wave: ch0 time strictly increasing", mono,
              "%.3f..%.3f" % (t[0], t[-1]))

        # ch1..3 are the three duties (board is stopped here -> 50% zero vector)
        duties = [v for r in rows for v in r[1:4]]
        check("wave: ch1~3 are duties within 0..1",
              all(0.0 <= v <= 1.0 for v in duties))
        check("wave: ch1~3 are 0.50 while stopped",
              all(abs(v - 0.5) < 0.01 for v in duties),
              "min=%.2f max=%.2f" % (min(duties), max(duties)))

        # ch4..6 are the phase currents (no load connected -> near zero)
        cur = [v for r in rows for v in r[4:7]]
        check("wave: ch4~6 are currents within +-1.32 A",
              all(abs(v) <= 1.32 for v in cur))
        check("wave: ch4~6 are ~0 A without a load",
              all(abs(v) < 0.05 for v in cur),
              "max |I| = %.3f A" % max(abs(v) for v in cur))

    m = [x for x in lines(st) if "wave     =" in x]
    check("wave: drop == 0 after 1.2 s @500Hz",
          bool(m) and "drop=0" in m[0], m[0].strip() if m else "-")

    # ---------- 11. final ----------
    out = lk.send("echo done")
    check("link still alive after wave burst", lines(out) == ["[log] done"])

    lk.close()

    total = len(RESULT)
    fails = [r for r in RESULT if not r[1]]
    print("=" * 62)
    print(" checks=%d  failed=%d  ->  %s" % (total, len(fails), "PASS" if not fails else "FAIL"))
    print("=" * 62)
    if fails:
        for name, _ok, detail in fails:
            print("  FAILED: %s  %s" % (name, detail))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
