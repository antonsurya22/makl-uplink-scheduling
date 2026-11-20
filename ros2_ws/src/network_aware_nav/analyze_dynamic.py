#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path


CONDITION_ORDER = [
    "default/normal",
    "default/severe",
    "conservative/severe",
]


def as_float(value):
    if value is None:
        return None

    value = str(value).strip()

    if value == "":
        return None

    try:
        return float(value)
    except ValueError:
        return None


def as_int(value):
    x = as_float(value)
    return int(x) if x is not None else None


def mean(values):
    values = [x for x in values if x is not None]
    return statistics.mean(values) if values else None


def median(values):
    values = [x for x in values if x is not None]
    return statistics.median(values) if values else None


def stdev(values):
    values = [x for x in values if x is not None]

    if len(values) < 2:
        return None

    return statistics.stdev(values)


def fmt(value, digits=3):
    if value is None:
        return "-"

    return f"{value:.{digits}f}"


def pct(n, total):
    if total == 0:
        return "-"

    return f"{100.0 * n / total:.1f}%"


def wilson_interval(successes, n, z=1.96):
    if n == 0:
        return None, None

    p = successes / n

    denom = 1.0 + z * z / n

    center = (
        p + z * z / (2.0 * n)
    ) / denom

    half = (
        z
        * math.sqrt(
            p * (1.0 - p) / n
            + z * z / (4.0 * n * n)
        )
        / denom
    )

    return center - half, center + half


def exact_sign_test(differences):
    """
    Two-sided exact sign test.
    Zero differences are discarded.
    """

    differences = [
        d for d in differences
        if d is not None and abs(d) > 1e-12
    ]

    n = len(differences)

    if n == 0:
        return None, 0, 0, 0

    positive = sum(d > 0 for d in differences)
    negative = sum(d < 0 for d in differences)

    k = min(positive, negative)

    tail = sum(
        math.comb(n, i)
        for i in range(k + 1)
    ) / (2 ** n)

    p = min(1.0, 2.0 * tail)

    return p, positive, negative, n


def summarize_condition(rows):
    n = len(rows)

    success = sum(
        as_int(r.get("success")) == 1
        for r in rows
    )

    collision = sum(
        as_int(r.get("collision")) == 1
        for r in rows
    )

    proximity = sum(
        as_int(r.get("proximity_violation")) == 1
        for r in rows
    )

    overlap = sum(
        as_int(r.get("geometric_overlap")) == 1
        for r in rows
    )

    contact = sum(
        as_int(r.get("actual_contact")) == 1
        for r in rows
    )

    clearance = [
        as_float(r.get("dynamic_clearance"))
        for r in rows
    ]

    p2p3 = [
        as_float(r.get("p2_p3_time"))
        for r in rows
    ]

    nav_time = [
        as_float(r.get("nav_time"))
        for r in rows
    ]

    episode_time = [
        as_float(r.get("episode_time"))
        for r in rows
    ]

    path_len = [
        as_float(r.get("path_len"))
        for r in rows
    ]

    min_obs = [
        as_float(r.get("min_obs_dist"))
        for r in rows
    ]

    low, high = wilson_interval(success, n)

    return {
        "n": n,
        "success": success,
        "collision": collision,
        "proximity": proximity,
        "overlap": overlap,
        "contact": contact,

        "success_ci_low": low,
        "success_ci_high": high,

        "clearance_mean": mean(clearance),
        "clearance_sd": stdev(clearance),
        "clearance_median": median(clearance),

        "p2p3_mean": mean(p2p3),
        "p2p3_sd": stdev(p2p3),

        "nav_time_mean": mean(nav_time),
        "nav_time_sd": stdev(nav_time),

        "episode_time_mean": mean(episode_time),
        "episode_time_sd": stdev(episode_time),

        "path_mean": mean(path_len),
        "path_sd": stdev(path_len),

        "min_obs_mean": mean(min_obs),
        "min_obs_sd": stdev(min_obs),
    }


def paired_clearance(rows, a, b):
    """
    Computes B - A using matched randomized blocks.
    """

    by_block = defaultdict(dict)

    for r in rows:
        condition = r.get("condition", "").strip()

        try:
            block = int(r.get("block", ""))
        except ValueError:
            continue

        clearance = as_float(
            r.get("dynamic_clearance")
        )

        if clearance is not None:
            by_block[block][condition] = clearance

    pairs = []

    for block in sorted(by_block):
        item = by_block[block]

        if a in item and b in item:
            pairs.append(
                (
                    block,
                    item[a],
                    item[b],
                    item[b] - item[a],
                )
            )

    return pairs


def print_paired(title, pairs, expected_direction):
    print()
    print(title)
    print("-" * len(title))

    if not pairs:
        print("No complete paired blocks.")
        return

    diffs = [p[3] for p in pairs]

    print(f"Complete blocks : {len(pairs)}")
    print(
        f"Mean difference : {fmt(mean(diffs))} m "
        f"({fmt(mean(diffs) * 100.0, 2)} cm)"
    )
    print(
        f"Median diff.    : {fmt(median(diffs))} m"
    )
    print(
        f"SD difference  : {fmt(stdev(diffs))} m"
    )

    p, positive, negative, n = exact_sign_test(diffs)

    print(
        f"Positive/negative: {positive}/{negative}"
    )

    if p is not None:
        print(
            f"Exact sign-test : p={p:.6f}"
        )

    if expected_direction == "negative":
        favourable = sum(d < 0 for d in diffs)
        print(
            f"Expected direction achieved: "
            f"{favourable}/{len(diffs)} blocks"
        )

    elif expected_direction == "positive":
        favourable = sum(d > 0 for d in diffs)
        print(
            f"Expected direction achieved: "
            f"{favourable}/{len(diffs)} blocks"
        )

    print()
    print("Block differences:")

    for block, av, bv, diff in pairs:
        print(
            f"  block {block:02d}: "
            f"{av:+.3f} -> {bv:+.3f} m, "
            f"delta={diff:+.3f} m"
        )


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--csv",
        required=True,
        help="formal randomized experiment CSV"
    )

    args = ap.parse_args()

    path = Path(args.csv)

    if not path.exists():
        raise SystemExit(
            f"ERROR: file not found: {path}"
        )

    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise SystemExit(
            "ERROR: CSV has no data rows"
        )

    print()
    print("=" * 72)
    print("DYNAMIC-SAFETY EXPERIMENT ANALYSIS")
    print("=" * 72)
    print(f"File       : {path}")
    print(f"Total rows : {len(rows)}")
    print()

    grouped = defaultdict(list)

    for row in rows:
        condition = row.get("condition")

        if not condition:
            condition = (
                f"{row.get('policy', '')}/"
                f"{row.get('profile', '')}"
            )

        grouped[condition].append(row)

    # --------------------------------------------------------
    # Condition summaries
    # --------------------------------------------------------

    print("CONDITION SUMMARY")
    print("-" * 72)

    for condition in CONDITION_ORDER:

        condition_rows = grouped.get(condition, [])

        if not condition_rows:
            continue

        s = summarize_condition(condition_rows)

        print()
        print(condition)
        print(f"  N                     : {s['n']}")
        print(
            f"  Success               : "
            f"{s['success']}/{s['n']} "
            f"({pct(s['success'], s['n'])})"
        )

        print(
            f"  Success Wilson 95% CI : "
            f"{fmt(s['success_ci_low'] * 100, 1)}% - "
            f"{fmt(s['success_ci_high'] * 100, 1)}%"
        )

        print(
            f"  Physical collision    : "
            f"{s['collision']}/{s['n']} "
            f"({pct(s['collision'], s['n'])})"
        )

        print(
            f"  Actual contact        : "
            f"{s['contact']}/{s['n']} "
            f"({pct(s['contact'], s['n'])})"
        )

        print(
            f"  Geometric overlap     : "
            f"{s['overlap']}/{s['n']} "
            f"({pct(s['overlap'], s['n'])})"
        )

        print(
            f"  Proximity violation   : "
            f"{s['proximity']}/{s['n']} "
            f"({pct(s['proximity'], s['n'])})"
        )

        print(
            f"  Dynamic clearance     : "
            f"{fmt(s['clearance_mean'])} ± "
            f"{fmt(s['clearance_sd'])} m"
        )

        print(
            f"  Median clearance      : "
            f"{fmt(s['clearance_median'])} m"
        )

        print(
            f"  P2->P3 time           : "
            f"{fmt(s['p2p3_mean'], 2)} ± "
            f"{fmt(s['p2p3_sd'], 2)} s"
        )

        print(
            f"  Nav time (successes)  : "
            f"{fmt(s['nav_time_mean'], 2)} ± "
            f"{fmt(s['nav_time_sd'], 2)} s"
        )

        # Old validation files do not have episode_time.
        if s["episode_time_mean"] is not None:
            print(
                f"  Episode time          : "
                f"{fmt(s['episode_time_mean'], 2)} ± "
                f"{fmt(s['episode_time_sd'], 2)} s"
            )

        print(
            f"  Path length           : "
            f"{fmt(s['path_mean'], 3)} ± "
            f"{fmt(s['path_sd'], 3)} m"
        )

    # --------------------------------------------------------
    # Paired randomized-block analysis
    # --------------------------------------------------------

    network_pairs = paired_clearance(
        rows,
        "default/normal",
        "default/severe",
    )

    print_paired(
        "PAIRED NETWORK EFFECT: "
        "default/severe - default/normal",
        network_pairs,
        expected_direction="negative",
    )

    recovery_pairs = paired_clearance(
        rows,
        "default/severe",
        "conservative/severe",
    )

    print_paired(
        "PAIRED SAFETY RECOVERY: "
        "conservative/severe - default/severe",
        recovery_pairs,
        expected_direction="positive",
    )

    # --------------------------------------------------------
    # Failure reasons
    # --------------------------------------------------------

    print()
    print("FAILURE REASONS")
    print("-" * 72)

    found_failure = False

    for condition in CONDITION_ORDER:
        failures = Counter()

        for row in grouped.get(condition, []):
            if as_int(row.get("success")) == 0:
                note = (
                    row.get("note", "").strip()
                    or "unspecified"
                )

                failures[note] += 1

        if failures:
            found_failure = True

            print(condition)

            for reason, count in failures.items():
                print(
                    f"  {reason}: {count}"
                )

    if not found_failure:
        print("No failures.")

    print()
    print("=" * 72)
    print(
        "Interpretation reminder: collision/actual_contact are "
        "Gazebo physical-contact metrics; geometric_overlap is an "
        "estimated circular-footprint overlap."
    )
    print("=" * 72)


if __name__ == "__main__":
    main()
