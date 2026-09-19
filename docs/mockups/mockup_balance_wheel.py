"""
Mockup: balance-wheel-centric timegrapher display.

Shows the beat information (amplitude, beat error, rate) organised around
a stylised balance wheel rather than as abstract waveform strips.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Arc, FancyBboxPatch
from matplotlib.path import Path
import matplotlib.patheffects as pe

# ── sample values (what the algo would produce) ───────────────────────────────
AMPLITUDE   = 270    # degrees
BEAT_ERROR  = 0.8    # ms  (positive = tic side long)
RATE        = +5     # s/d
BPH         = 21600
UNCERTAINTY = 3      # s/d  (±)

# ── derived geometry ──────────────────────────────────────────────────────────
# The balance arm rests pointing straight up (90°) when at its midpoint.
# Beat error shifts the midpoint: convert ms → degrees using BPH.
seconds_per_beat = 3600 / BPH          # half-period in seconds
be_deg = BEAT_ERROR / (1000 * seconds_per_beat) * (AMPLITUDE / 2)

half_amp   = AMPLITUDE / 2             # degrees each side
tic_deg    = 90 + half_amp + be_deg/2  # top-right extreme
toc_deg    = 90 - half_amp + be_deg/2  # top-left extreme
center_deg = (tic_deg + toc_deg) / 2   # midpoint of swing (should be 90 if BE=0)

# ── figure setup ─────────────────────────────────────────────────────────────
BG   = '#0d0d0d'
FG   = '#e8e8e8'
ARC  = '#d4a017'   # gold for swing arc
TIC  = '#4488ff'
TOC  = '#44ccff'
RED  = '#ff4444'
GRN  = '#44cc66'

fig, ax = plt.subplots(figsize=(9, 9), facecolor=BG)
ax.set_facecolor(BG)
ax.set_aspect('equal')
ax.set_xlim(-1.55, 1.55)
ax.set_ylim(-1.55, 1.55)
ax.axis('off')
fig.subplots_adjust(left=0, right=1, top=1, bottom=0)

def polar(r, deg):
    a = np.radians(deg)
    return r * np.cos(a), r * np.sin(a)

# ── degree scale ring ─────────────────────────────────────────────────────────
SCALE_R = 1.10
for deg in range(0, 360, 5):
    tick_len = 0.06 if deg % 30 == 0 else 0.03 if deg % 10 == 0 else 0.015
    x0, y0 = polar(SCALE_R - tick_len, deg)
    x1, y1 = polar(SCALE_R, deg)
    col = '#555555' if deg % 30 else '#888888'
    ax.plot([x0, x1], [y0, y1], color=col, lw=0.8, solid_capstyle='round')

# degree labels every 30°
for deg in range(0, 360, 30):
    lx, ly = polar(SCALE_R + 0.12, deg)
    label = f'{deg}°'
    ax.text(lx, ly, label, color='#666666', ha='center', va='center',
            fontsize=7, fontfamily='monospace')

# ── thin outer reference circle ───────────────────────────────────────────────
circle_outer = plt.Circle((0, 0), SCALE_R, fill=False,
                           edgecolor='#333333', linewidth=0.8)
ax.add_patch(circle_outer)

# ── amplitude arc (the actual swing of the balance) ───────────────────────────
ARC_R = 0.88
theta = np.linspace(np.radians(toc_deg), np.radians(tic_deg), 400)
ax.plot(ARC_R * np.cos(theta), ARC_R * np.sin(theta),
        color=ARC, lw=6, alpha=0.85, solid_capstyle='round', zorder=4)

# arc end-cap dots
for deg, col in [(tic_deg, TIC), (toc_deg, TOC)]:
    ex, ey = polar(ARC_R, deg)
    ax.plot(ex, ey, 'o', color=col, ms=10, zorder=5)

# ── tic / toc radial lines ────────────────────────────────────────────────────
for deg, col, lbl in [(tic_deg, TIC, 'TIC'), (toc_deg, TOC, 'TOC')]:
    x0, y0 = polar(0.42, deg)
    x1, y1 = polar(0.96, deg)
    ax.plot([x0, x1], [y0, y1], color=col, lw=1.5, alpha=0.7,
            linestyle='--', zorder=3)
    lx, ly = polar(1.02, deg)
    ax.text(lx, ly, lbl, color=col, ha='center', va='center',
            fontsize=8.5, fontweight='bold',
            path_effects=[pe.withStroke(linewidth=2, foreground=BG)])

# ── beat-error indicator: arc between ideal centre and actual centre ───────────
IDEAL_DEG = 90   # where centre would be if BE = 0
if abs(be_deg) > 0.05:
    err_theta = np.linspace(np.radians(min(IDEAL_DEG, center_deg)),
                             np.radians(max(IDEAL_DEG, center_deg)), 80)
    BE_R = 0.68
    ax.plot(BE_R * np.cos(err_theta), BE_R * np.sin(err_theta),
            color=RED, lw=3, alpha=0.9, solid_capstyle='round', zorder=4)
    # small arrowhead at centre_deg end
    arrow_end = polar(BE_R, center_deg)
    arrow_start = polar(BE_R, center_deg - np.sign(be_deg)*2)
    ax.annotate('', xy=arrow_end, xytext=arrow_start,
                arrowprops=dict(arrowstyle='->', color=RED, lw=2))

# ideal-centre dashed line (straight up)
ix, iy = polar(0.95, IDEAL_DEG)
ax.plot([0, ix], [0, iy],
        color='#555555', lw=1, linestyle=':', zorder=2)

# ── balance wheel body ────────────────────────────────────────────────────────
# rim
wheel_rim = plt.Circle((0, 0), 0.40, fill=False,
                        edgecolor='#aaaaaa', linewidth=2.5, zorder=6)
ax.add_patch(wheel_rim)

# spokes (4 of them, one pointing toward swing midpoint)
arm_deg = center_deg  # the arm points toward the current midpoint
for spoke_deg in [arm_deg, arm_deg+90, arm_deg+180, arm_deg+270]:
    sx0, sy0 = polar(0.045, spoke_deg)
    sx1, sy1 = polar(0.385, spoke_deg)
    ax.plot([sx0, sx1], [sy0, sy1], color='#999999', lw=2, zorder=6)

# impulse pin (small filled circle on rim, at midpoint angle)
pin_x, pin_y = polar(0.40, arm_deg)
ax.plot(pin_x, pin_y, 'o', color=ARC, ms=8, zorder=7)

# hub
hub = plt.Circle((0, 0), 0.045, color='#cccccc', zorder=8)
ax.add_patch(hub)
ax.plot(0, 0, 'o', color='#333333', ms=4, zorder=9)

# ── mini waveform sketches at tic and toc positions ───────────────────────────
def draw_mini_waveform(cx, cy, rot_deg, color, ax, scale=0.18):
    """Draw a stylised impulse waveform (rise + exponential decay) centred at cx,cy."""
    t = np.linspace(-1, 2.5, 120)
    y = np.where(t < 0, 0,
        np.where(t < 0.4, t / 0.4,
                 np.exp(-2.5 * (t - 0.4))))
    # normalise to [-0.5, 0.5]
    t_n = (t - 0.75) / 3.5 * scale
    y_n = (y - 0.0) * scale * 0.55

    # rotate into place
    rot = np.radians(rot_deg - 90)   # -90 so "up" waveform aligns radially
    cos_r, sin_r = np.cos(rot), np.sin(rot)
    xr = cx + t_n * cos_r - y_n * sin_r
    yr = cy + t_n * sin_r + y_n * cos_r

    ax.plot(xr, yr, color=color, lw=1.5, alpha=0.9, zorder=5,
            solid_capstyle='round')

    # threshold line
    thr = 0.30 * scale * 0.55
    tx = np.array([-scale*0.5, scale*0.5])
    ty = np.array([thr, thr])
    xrt = cx + tx * cos_r - ty * sin_r
    yrt = cy + tx * sin_r + ty * cos_r
    ax.plot(xrt, yrt, color=color, lw=0.8, linestyle='--', alpha=0.5, zorder=5)

WF_R = 1.28
draw_mini_waveform(*polar(WF_R, tic_deg), tic_deg, TIC, ax)
draw_mini_waveform(*polar(WF_R, toc_deg), toc_deg, TOC, ax)

# waveform labels
for deg, col, txt in [(tic_deg, TIC, 'tic waveform'), (toc_deg, TOC, 'toc waveform')]:
    lx, ly = polar(WF_R + 0.20, deg)
    ax.text(lx, ly, txt, color=col, ha='center', va='center',
            fontsize=7, alpha=0.7,
            path_effects=[pe.withStroke(linewidth=1.5, foreground=BG)])

# ── numeric readouts ──────────────────────────────────────────────────────────
readout_style = dict(transform=ax.transData, ha='center', va='center',
                     fontfamily='monospace',
                     path_effects=[pe.withStroke(linewidth=3, foreground=BG)])

# RATE — bottom centre
rate_sign = '+' if RATE >= 0 else ''
ax.text(0, -0.55, f'{rate_sign}{RATE} s/d',
        color=GRN if abs(RATE) < 15 else RED,
        fontsize=22, fontweight='bold', **readout_style)
ax.text(0, -0.70, f'±{UNCERTAINTY} s/d',
        color='#888888', fontsize=11, **readout_style)

# AMPLITUDE — lower left
ax.text(-0.52, -0.30,
        f'{AMPLITUDE}°', color=ARC, fontsize=20, fontweight='bold',
        **readout_style)
ax.text(-0.52, -0.44, 'amplitude', color='#777777', fontsize=9,
        **readout_style)

# BEAT ERROR — lower right
be_col = GRN if abs(BEAT_ERROR) < 1.0 else RED
ax.text(0.52, -0.30,
        f'{BEAT_ERROR:+.1f} ms',
        color=be_col, fontsize=16, fontweight='bold', **readout_style)
ax.text(0.52, -0.44, 'beat error', color='#777777', fontsize=9,
        **readout_style)

# BPH — centre of wheel
ax.text(0, 0, f'{BPH}', color='#aaaaaa', fontsize=11,
        ha='center', va='center', fontfamily='monospace',
        path_effects=[pe.withStroke(linewidth=2, foreground='#0d0d0d')],
        zorder=10)

# ── legend annotation for beat-error arc ─────────────────────────────────────
if abs(be_deg) > 0.05:
    mid_be_deg = (IDEAL_DEG + center_deg) / 2
    bx, by = polar(0.68 + 0.18, mid_be_deg)
    ax.text(bx, by, f'BE\n{BEAT_ERROR:+.1f} ms', color=RED,
            fontsize=8, ha='center', va='center',
            path_effects=[pe.withStroke(linewidth=2, foreground=BG)])

# ── title ─────────────────────────────────────────────────────────────────────
ax.text(0, -1.45, 'tg — balance wheel view  [mockup]',
        color='#555555', fontsize=9, ha='center', va='center',
        fontfamily='monospace')

plt.savefig('C:/Users/arjun/tg/balance_wheel_mockup.jpg',
            dpi=150, bbox_inches='tight', facecolor=BG,
            pil_kwargs={'quality': 95})
print("saved balance_wheel_mockup.jpg")
