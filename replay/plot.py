#!/usr/bin/env python3
"""Draw every stage of the pipeline for one set of one recording.

The device only hands out the reps it closed, so a session's failures read as numbers: a rom
that is twice the stroke, a span that starts a turn late. This draws what the reps were made of
-- the raw sample, the filter's frame, the cues, the velocity the integrator carried, and the
position each rep was given -- against the spans the detector closed.

It runs `sim`'s `pipeline_dump` to get the numbers, so the pipeline under the drawing is the one
the firmware runs.
"""

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SIM = ROOT / "sim"
CAPTURES = ROOT / "captures"

X_COLOR, Y_COLOR, Z_COLOR = "tab:blue", "tab:green", "tab:red"
BAND, TURN = "0.92", "tab:orange"

# The three turns a rep is anchored on. A rep with no turn between its halves leaves that cell empty.
TURNS = ("turn0_ms", "turn1_ms", "turn2_ms")


def rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def column(data, name, cast=float):
    return [cast(row[name]) for row in data]


def dump(capture, out):
    """The pipeline's numbers for a whole recording, written by the crate that owns the replay."""
    out.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "cargo",
            "run",
            "--quiet",
            "--release",
            "--example",
            "pipeline_dump",
            "--",
            capture,
            "--out",
            str(out),
        ],
        cwd=SIM,
        check=True,
    )


def manifest(capture):
    for entry in json.loads((CAPTURES / "captures.json").read_text())["captures"]:
        if entry["name"] == capture:
            return entry
    return {}


def describe(entry, index, sets, reps):
    """The capture, the set, and what the session notes say it was, where the two can be paired."""
    text = entry.get("exercise") or "recording"
    line = f"{entry.get('name', '')} -- {text}, set {index + 1} of {len(sets)}, {reps} reps detected"
    expected = entry.get("expected", {}).get("sets", [])
    if len(expected) == len(sets) and index < len(expected):
        note = expected[index]
        load = f" at {note['load_lb']} lb" if note.get("load_lb") else " bar only"
        line += f" (noted: {note['reps']} reps{load})"
    return line


def band(ax, spans, turns):
    for start, end in spans:
        ax.axvspan(start, end, color=BAND, lw=0, zorder=0)
    for turn in turns:
        ax.axvline(turn, color=TURN, lw=0.8, ls=(0, (4, 4)), zorder=1)


def line(ax, x, y, label, color, lw=1.0, **kw):
    ax.plot(x, y, color=color, lw=lw, label=label, **kw)


def finish(ax, title, unit, zero=False):
    ax.set_title(title, loc="left", fontsize=10, pad=4)
    ax.set_ylabel(unit, fontsize=8)
    ax.grid(True, color="0.94", lw=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(labelsize=8)
    if zero:
        ax.axhline(0.0, color="0.6", lw=0.8, ls=(0, (5, 4)))
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="upper right", fontsize=7, frameon=False, ncols=4)


def plot_set(data, entry, index, margin, out):
    spans = [(float(r["t_start_ms"]) / 1000, float(r["t_end_ms"]) / 1000) for r in data["reps"]]
    turns = sorted({float(r[key]) / 1000 for r in data["reps"] for key in TURNS if r[key]})
    first, last = spans[0][0] - margin, spans[-1][1] + margin
    keep = [i for i, t in enumerate(data["t"]) if first <= t <= last]
    t = [data["t"][i] for i in keep]

    fig, axes = plt.subplots(
        10,
        1,
        sharex=True,
        figsize=(17, 14),
        gridspec_kw=dict(height_ratios=[1, 1, 1, 0.8, 0.8, 0.35, 1.0, 0.9, 0.55, 1.1]),
        layout="constrained",
    )
    for ax in axes:
        band(ax, spans, turns)

    def series(ax, name, label, color, lw=1.0, **kw):
        line(ax, t, [float(data[name][i]) for i in keep], label, color, lw, **kw)

    for name, label, color in (("ax_raw", "x", X_COLOR), ("ay_raw", "y", Y_COLOR), ("az_raw", "z", Z_COLOR)):
        series(axes[0], name, label, color)
    finish(axes[0], "raw specific force, sensor frame", "m/s²")
    for name, label, color in (("gx_raw", "x", X_COLOR), ("gy_raw", "y", Y_COLOR), ("gz_raw", "z", Z_COLOR)):
        series(axes[1], name, label, color)
    finish(axes[1], "raw angular rate, sensor frame", "dps")
    for name, label, color in (("ax", "x", X_COLOR), ("ay", "y", Y_COLOR), ("az", "z", Z_COLOR)):
        series(axes[2], name, label, color, 2.0 if name == "az" else 1.0)
    finish(axes[2], "fusion: linear acceleration, world frame", "m/s²", zero=True)
    series(axes[3], "tilt_deg", "tilt", "tab:purple", 1.5)
    finish(axes[3], "fusion: tilt of the bar's up axis from gravity", "deg")
    series(axes[4], "rate_mag", "rate", "tab:brown", 1.4)
    finish(axes[4], "cues: bias-corrected rate magnitude", "rad/s")

    sign = [int(data["sign_vert"][i]) for i in keep]
    axes[5].step(t, sign, where="post", color="0.35", lw=1.0)
    axes[5].set_yticks([-1, 0, 1])
    axes[5].set_ylim(-1.6, 1.6)
    finish(axes[5], "cues: sign_vert, ±1 outside a 0.05 m/s² deadband", "sign")

    series(axes[6], "vz", "vz", Z_COLOR, 1.8)
    series(axes[6], "vcross_z", "v_cross z", TURN, 1.1, ls=(0, (4, 3)))
    finish(axes[6], "integration: vertical velocity", "m/s", zero=True)
    series(axes[7], "vx", "vx", X_COLOR)
    series(axes[7], "vy", "vy", Y_COLOR)
    finish(axes[7], "integration: horizontal velocity, where gravity leaks", "m/s", zero=True)

    resting = [int(data["resting"][i]) for i in keep]
    opened = [int(data["rep_open"][i]) for i in keep]
    axes[8].fill_between(t, 0.55, [0.55 + 0.4 * v for v in resting], step="post", color="0.55", lw=0)
    axes[8].fill_between(t, 0.05, [0.05 + 0.4 * v for v in opened], step="post", color="tab:red", lw=0)
    axes[8].set_yticks([0.25, 0.75], ["rep_open", "resting"], fontsize=7)
    axes[8].set_ylim(0, 1)
    finish(axes[8], "flags", "")

    for n, (gid, points) in enumerate(data["traces"].items()):
        color = f"tab:{['blue', 'orange', 'green', 'red', 'purple'][n % 5]}"
        line(axes[9], [p[0] for p in points], [p[3] for p in points], "", color, 1.8)
        axes[9].annotate(
            f"rep {n + 1}: rom {data['rom'][gid]:.2f} m, ends {data['back'][gid]:.2f} m off",
            xy=(points[0][0], points[0][3]),
            xytext=(3, 4),
            textcoords="offset points",
            fontsize=7.5,
            color=color,
        )
    finish(axes[9], "output: the vertical position each rep is given", "m", zero=True)

    axes[9].set_xlabel("device uptime (s)", fontsize=9)
    fig.suptitle(describe(entry, index, data["sets"], len(data["reps"])), x=0.01, ha="left", fontsize=13)
    fig.get_layout_engine().set(rect=(0, 0, 1, 0.955))
    fig.text(0.01, 0.975, f"window {first:.1f}-{last:.1f} s · bands are the detected rep spans · dashed orange are the anchor turns", fontsize=9, color="0.35")
    path = out / f"set-{index + 1}.png"
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def plot_rep(data, entry, index, position, margin, out):
    """One rep of the set, drawn alone: `position` counts from zero within the set."""
    rep_rows = data["reps"]
    gid = int(rep_rows[position]["rep"])
    spans = [(float(r["t_start_ms"]) / 1000, float(r["t_end_ms"]) / 1000) for r in rep_rows]
    turns = sorted({float(r[key]) / 1000 for r in rep_rows for key in TURNS if r[key]})
    start, end = spans[position]
    anchors = [float(rep_rows[position][key]) / 1000 for key in TURNS if rep_rows[position][key]]
    first, last = start - margin, end + margin
    keep = [i for i, t in enumerate(data["t"]) if first <= t <= last]
    t = [data["t"][i] for i in keep]

    fig, axes = plt.subplots(
        4,
        1,
        sharex=True,
        figsize=(15, 8),
        gridspec_kw=dict(height_ratios=[1, 1, 1, 1.1]),
        layout="constrained",
    )
    for ax in axes:
        band(ax, [spans[position]], anchors)

    def series(ax, name, label, color, lw=1.0, **kw):
        line(ax, t, [float(data[name][i]) for i in keep], label, color, lw, **kw)

    for name, label, color in (("ax_raw", "x", X_COLOR), ("ay_raw", "y", Y_COLOR), ("az_raw", "z", Z_COLOR)):
        series(axes[0], name, label, color)
    finish(axes[0], "raw specific force, sensor frame", "m/s²")
    for name, label, color in (("ax", "x", X_COLOR), ("ay", "y", Y_COLOR), ("az", "z", Z_COLOR)):
        series(axes[1], name, label, color, 2.0 if name == "az" else 1.0)
    finish(axes[1], "fusion: linear acceleration, world frame", "m/s²", zero=True)
    series(axes[2], "vz", "vz", Z_COLOR, 1.8)
    series(axes[2], "vcross_z", "v_cross z", TURN, 1.1, ls=(0, (4, 3)))
    finish(axes[2], "integration: vertical velocity", "m/s", zero=True)

    points = data["traces"][gid]
    line(axes[3], [p[0] for p in points], [p[3] for p in points], f"rep {position + 1}", X_COLOR, 2.0)
    finish(axes[3], f"output: vertical position, rom {data['rom'][gid]:.2f} m, ends {data['back'][gid]:.2f} m from where it began", "m", zero=True)

    for ax, anchor, name in zip(axes, anchors, ("entry turn", "turn", "exit turn")):
        ax.annotate(name, xy=(anchor, 1.0), xycoords=("data", "axes fraction"), xytext=(2, -2), textcoords="offset points", fontsize=7, color=TURN, va="top")

    axes[3].set_xlabel("device uptime (s)", fontsize=9)
    fig.suptitle(f"{describe(entry, index, data['sets'], len(data['reps']))} -- rep {position + 1}", x=0.01, ha="left", fontsize=12)
    path = out / f"set-{index + 1}-rep-{position + 1}.png"
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", help="a name in captures/, or one the bucket holds")
    parser.add_argument("--set", dest="set_index", type=int, default=1, help="which set to draw, 1-based (default 1)")
    parser.add_argument("--rep", type=int, default=1, help="which rep of that set to draw alone, 1-based (default 1)")
    parser.add_argument("--margin", type=float, default=3.0, help="seconds drawn either side of the set (default 3)")
    parser.add_argument("--dump", type=Path, help="reuse a pipeline_dump directory instead of running one")
    parser.add_argument("--out", type=Path, default=ROOT / "replay" / "out", help="where the figures go")
    args = parser.parse_args()

    dump_dir = args.dump or ROOT / "replay" / "out" / Path(args.capture).stem
    if args.dump is None:
        dump(args.capture, dump_dir)

    samples = rows(dump_dir / "samples.csv")
    reps = rows(dump_dir / "reps.csv")
    if not reps:
        sys.exit(f"{args.capture}: the detector closed no reps, so there is nothing to draw")
    sets = sorted({int(r["set"]) for r in reps})
    if not 1 <= args.set_index <= len(sets):
        sys.exit(f"{args.capture} closed {len(sets)} set(s), so there is no set {args.set_index}")

    index = args.set_index - 1
    in_set = [i for i, r in enumerate(reps) if int(r["set"]) == index]
    if not 1 <= args.rep <= len(in_set):
        sys.exit(f"set {args.set_index} closed {len(in_set)} rep(s), so there is no rep {args.rep}")

    data = {
        "t": [float(r["t_ms"]) / 1000 for r in samples],
        "reps": [reps[i] for i in in_set],
        "sets": sets,
        "rom": {i: float(reps[i]["rom_m"]) for i in in_set},
        "back": {i: float(reps[i]["back_m"]) for i in in_set},
        "traces": {
            i: [(float(p["t_ms"]) / 1000, float(p["pos_x"]), float(p["pos_y"]), float(p["pos_z"]))
                for p in rows(dump_dir / "traces.csv") if int(p["rep"]) == i]
            for i in in_set
        },
    }
    for name in samples[0]:
        if name != "t_ms":
            data[name] = [float(r[name]) for r in samples]

    entry = manifest(args.capture)
    args.out.mkdir(parents=True, exist_ok=True)
    set_path = plot_set(data, entry, index, args.margin, args.out)
    rep_path = plot_rep(data, entry, index, args.rep - 1, args.margin / 6, args.out)
    print(set_path)
    print(rep_path)


if __name__ == "__main__":
    main()
