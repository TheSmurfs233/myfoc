#!/usr/bin/env python3
"""S3 acceptance: calibration lifecycle, frame rate, DWT profile, safety guards."""
import sys, time, re, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000
R = []

def check(name, ok, detail=""):
    R.append((name, bool(ok), detail))
    print(("  PASS  " if ok else "  FAIL  ") + name + (("   " + detail) if detail else ""))

s = serial.Serial(PORT, BAUD, timeout=0.05)
buf = ""

def drain(quiet=0.15, maxs=1.5):
    global buf
    t0 = time.time(); last = t0
    while True:
        n = s.in_waiting
        if n:
            buf += s.read(n).decode("ascii", "replace"); last = time.time()
        else:
            time.sleep(0.002)
        now = time.time()
        if now - last >= quiet: break
        if now - t0 >= maxs: break
    out, buf = buf, ""; return out

def send(c, w=0.15):
    global buf
    buf = ""
    s.write((c + "\r\n").encode("ascii"))
    return drain(w, w + 1.0)

def field(text, pat):
    m = re.search(pat, text)
    return m.group(1) if m else None

s.write(b"wave off\r\nstop\r\nmode 0\r\n"); drain(0.4)

# ---- 1. erase -> uncalibrated -------------------------------------------
send("cal erase", 0.25)
out = send("cur")
check("erase: cal=0", "cal=0" in out, out.strip().splitlines()[0] if out else "-")
check("erase: src=none", "src=none" in out)
check("erase: off shows placeholders", "off ----/----/----" in out)
check("erase: gen=0", "gen=0" in out)

# ---- 2. start -> live calibration ---------------------------------------
send("mode 1")
t0 = time.time()
send("start", 0.12)
t_cal = None
for _ in range(60):
    out = send("cur", 0.03)
    if "cal=1" in out:
        t_cal = (time.time() - t0) * 1000.0
        break
    time.sleep(0.004)
check("start: calibration completes", t_cal is not None,
      ("%.0f ms (incl. command round-trips)" % t_cal) if t_cal else "timeout")
out = send("cur")
check("start: src=live", "src=live" in out)
check("start: gen=1", "gen=1" in out)
m = re.search(r"raw (\d+)/(\d+)/(\d+)", out)
vals = [int(x) for x in m.groups()] if m else []
check("start: raw near 2048", bool(vals) and all(abs(v - 2048) <= 20 for v in vals), str(vals))

# ---- 3. frame rate ------------------------------------------------------
def adc_frames():
    out = send("stat")
    v = field(out, r"adc\s+=\s+frames=(\d+)")
    return int(v) if v else -1
def adc_pair():
    out = send("stat")
    f = field(out, r"adc\s+=\s+frames=(\d+)")
    u = field(out, r"uptime\s+=\s+(\d+) ms")
    return (int(f), int(u)) if (f and u) else (-1, -1)

f0, u0 = adc_pair()
time.sleep(2.0)
f1, u1 = adc_pair()
# frames and uptime are sampled by the board in the same stat reply, so this
# ratio is exact regardless of PC-side timing
rate = (f1 - f0) / ((u1 - u0) / 1000.0) if (f0 >= 0 and u1 > u0) else -1
check("frame rate ~20 kHz", 19800 <= rate <= 20200,
      "%.0f /s (board uptime %d->%d ms)" % (rate, u0, u1))

# ---- 4. pwm refused while calibrating -----------------------------------
buf = ""
s.write(b"cal\r\npwm 0.6\r\n")
out = drain(0.25, 1.0)
check("pwm refused while calibrating", "ERR calibrating" in out, out.strip())
time.sleep(0.3)

# ---- 5. cal erase while RUN forces the zero vector ----------------------
send("pwm 0.7", 0.15)
send("cal erase", 0.15)
out = send("stat")
duty = field(out, r"duty=([0-9.]+/[0-9.]+/[0-9.]+)")
check("erase while RUN forces duty 0.50", duty == "0.50/0.50/0.50", str(duty))
time.sleep(0.6)
out = send("cur")
check("recalibration after erase: fresh record gen=1", "gen=1" in out,
      field(out, r"gen=(\d+)") or "-")

# ---- 6. DWT profile -----------------------------------------------------
send("prof reset", 0.15)
time.sleep(2.0)
prof = send("prof", 0.35)
def slot(nm):
    m = re.search(r"%s\s+n=(\d+)\s+min=(\d+)\s+avg=(\d+)\s+max=(\d+)" % nm, prof)
    return tuple(int(x) for x in m.groups()) if m else None
isp, isr, adc, cnv = slot("isr_period"), slot("isr"), slot("adc_read"), slot("conv")
check("profile: isr_period ~50000 ns", isp and abs(isp[2] - 50000) < 1000,
      "avg=%s ns" % (isp[2] if isp else "?"))
check("profile: ISR cost < 3 us", isr and isr[2] < 3000, "avg=%s ns" % (isr[2] if isr else "?"))
check("profile: conversion < 3 us", cnv and cnv[2] < 3000, "avg=%s ns" % (cnv[2] if cnv else "?"))

# ---- cleanup ------------------------------------------------------------
send("stop"); send("mode 0"); send("wave off")
out = send("stat")
check("board left idle with PWM off", "state    = IDLE" in out and "pwm      = OFF" in out)
s.close()

fails = [r for r in R if not r[1]]
print("=" * 62)
print(" s3 checks=%d failed=%d -> %s" % (len(R), len(fails), "PASS" if not fails else "FAIL"))
print("=" * 62)
for n, _o, d in fails:
    print("  FAILED: %s  %s" % (n, d))
sys.exit(0 if not fails else 1)
