#!/usr/bin/env python3
import argparse
import csv
import math
from statistics import mean, pstdev


def percentile(sorted_values, p):
    if not sorted_values:
        return math.nan
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    idx = (p / 100.0) * (len(sorted_values) - 1)
    i = int(math.floor(idx))
    j = int(math.ceil(idx))
    if i == j:
        return float(sorted_values[i])
    frac = idx - i
    return float(sorted_values[i]) * (1.0 - frac) + float(sorted_values[j]) * frac


def load_column(path, column):
    values = []
    with open(path, newline='') as f:
        # Skip commented lines
        lines = [ln for ln in f if not ln.startswith('#')]
        reader = csv.DictReader(lines)
        if column not in reader.fieldnames:
            raise SystemExit(f"Column '{column}' not found in CSV: {reader.fieldnames}")
        for row in reader:
            try:
                v = int(row[column])
                values.append(v)
            except Exception:
                # skip malformed rows
                pass
    return values


def summarize_ns(values_ns):
    if not values_ns:
        return None
    vals = [float(v) for v in values_ns]
    vals_sorted = sorted(vals)
    return {
        'count': len(vals),
        'mean_ns': mean(vals),
        'median_ns': percentile(vals_sorted, 50.0),
        'p95_ns': percentile(vals_sorted, 95.0),
        'p99_ns': percentile(vals_sorted, 99.0),
        'max_ns': vals_sorted[-1],
        'stddev_ns': pstdev(vals) if len(vals) > 1 else 0.0,
    }


def format_summary(s, unit='ms'):
    if s is None:
        return "No data"
    scale = 1e6 if unit == 'ms' else 1.0
    suf = unit
    return (
        f"count={s['count']} "
        f"mean={s['mean_ns']/scale:.6f} {suf} "
        f"median={s['median_ns']/scale:.6f} {suf} "
        f"p95={s['p95_ns']/scale:.6f} {suf} "
        f"p99={s['p99_ns']/scale:.6f} {suf} "
        f"max={s['max_ns']/scale:.6f} {suf} "
        f"stddev={s['stddev_ns']/scale:.6f} {suf}"
    )


def append_summary(path, label, s, unit='ms'):
    scale = 1e6 if unit == 'ms' else 1.0
    with open(path, 'a') as f:
        f.write('\n')
        f.write('# summary_rtt,label,count,mean_{u},median_{u},p95_{u},p99_{u},max_{u},stddev_{u}\n'.format(u=unit))
        f.write('# summary_rtt,{l},{c},{mean:.6f},{med:.6f},{p95:.6f},{p99:.6f},{max:.6f},{stddev:.6f}\n'.format(
            l=label,
            c=s['count'],
            mean=s['mean_ns']/scale,
            med=s['median_ns']/scale,
            p95=s['p95_ns']/scale,
            p99=s['p99_ns']/scale,
            max=s['max_ns']/scale,
            stddev=s['stddev_ns']/scale,
        ))


def main():
    ap = argparse.ArgumentParser(description='Summarize RTT CSV from ros2_latency_rtt (pinger).')
    ap.add_argument('csv', help='Path to RTT CSV (from pinger)')
    ap.add_argument('--unit', choices=['ns', 'ms'], default='ms', help='Output unit (default: ms)')
    ap.add_argument('--append', action='store_true', help='Append a commented summary to the CSV file')
    args = ap.parse_args()

    rtt = load_column(args.csv, 'rtt_ns')
    s_rtt = summarize_ns(rtt)
    print('[RTT] ' + format_summary(s_rtt, unit=args.unit))

    # If present, also summarize one-way estimate
    try:
        one = load_column(args.csv, 'oneway_est_ns')
    except SystemExit:
        one = []
    s_one = summarize_ns(one) if one else None
    if s_one:
        print('[ONE-WAY EST] ' + format_summary(s_one, unit=args.unit))

    if args.append and s_rtt:
        append_summary(args.csv, 'rtt', s_rtt, unit=args.unit)
        if s_one:
            append_summary(args.csv, 'oneway_est', s_one, unit=args.unit)


if __name__ == '__main__':
    main()

