#!/usr/bin/env python3
"""Plot a demod trace written by `test_modem --wav capture.wav --trace trace.csv`.

usage: python plot_trace.py trace.csv [start_s] [length_s]      (needs matplotlib)
"""
import csv
import sys

import matplotlib.pyplot as plt

path = sys.argv[1]
start = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
length = float(sys.argv[3]) if len(sys.argv) > 3 else 0.25

cols = {k: [] for k in ("t", "in", "bpf", "mark", "space", "demod", "pll", "sampled")}
with open(path) as f:
    for row in csv.DictReader(f):
        t = float(row.get("t", row.get("n", 0)))
        if t < start:
            continue
        if t > start + length:
            break
        for k in cols:
            cols[k].append(float(row[k]) if k in row else t)

t = cols["t"]
fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 9))
ax[0].plot(t, cols["in"], lw=0.6, label="decimated input")
ax[0].plot(t, cols["bpf"], lw=0.6, label="bandpass")
ax[1].plot(t, cols["mark"], label="mark amplitude")
ax[1].plot(t, cols["space"], label="space amplitude")
ax[2].plot(t, cols["demod"], lw=0.8, label="demod (mark - space, AGC)")
ax[2].axhline(0, color="k", lw=0.5)
ts = [ti for ti, s in zip(t, cols["sampled"]) if s]
ds = [d for d, s in zip(cols["demod"], cols["sampled"]) if s]
ax[2].plot(ts, ds, "rx", label="bit samples")
ax[3].plot(t, [p / 2**31 for p in cols["pll"]], lw=0.8, label="bit clock PLL (wraps at sample)")
for a in ax:
    a.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)
ax[-1].set_xlabel("seconds")
plt.tight_layout()
plt.show()
