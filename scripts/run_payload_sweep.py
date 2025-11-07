#!/usr/bin/env python3
import argparse
import subprocess
import sys
import os
from pathlib import Path


def parse_sizes(arg: str):
    # Accept comma list: "256,512,1024" or range: "256:4096:256"
    if "," in arg:
        sizes = []
        for tok in arg.split(","):
            tok = tok.strip()
            if not tok:
                continue
            sizes.append(int(tok))
        return sizes
    if ":" in arg:
        parts = arg.split(":")
        if len(parts) != 3:
            raise ValueError("Range format must be start:end:step")
        start, end, step = (int(parts[0]), int(parts[1]), int(parts[2]))
        if step <= 0:
            raise ValueError("step must be > 0")
        if end < start:
            raise ValueError("end must be >= start")
        sizes = list(range(start, end + 1, step))
        return sizes
    # Fallback single value
    v = int(arg)
    return [v]


def run_pinger_once(size: int, out_csv: Path, args):
    cmd = [
        "ros2", "run", "rmw_rtt_bench", "rtt_pinger", "--",
        "--req-topic", args.req_topic,
        "--rep-topic", args.rep_topic,
        "--qos-reliability", args.qos_reliability,
        "--qos-history", args.qos_history,
        "--qos-depth", str(args.qos_depth),
        "--hz", str(args.hz),
        "--payload-size", str(size),
        "--duration", str(args.per_size_duration),
        "--timeout", str(args.per_size_timeout) if args.per_size_timeout > 0 else str(args.per_size_duration + 5),
        "--intra-process", "true" if args.intra_process else "false",
        "--csv", str(out_csv),
    ]
    if args.transport_tag:
        cmd += ["--transport-tag", args.transport_tag]
    if args.notes:
        cmd += ["--notes", args.notes]
    if args.summary:
        cmd += ["--summary", "true"]
        if args.append_summary:
            cmd += ["--append-summary", "true"]
    print(f"[run] payload={size} bytes -> {out_csv}")
    proc = subprocess.run(cmd)
    if proc.returncode != 0:
        raise RuntimeError(f"pinger exited with code {proc.returncode}")


def merge_csvs(csv_paths, merged_path: Path):
    merged_path.parent.mkdir(parents=True, exist_ok=True)
    header_written = False
    with merged_path.open("w", encoding="utf-8") as out:
        for p in csv_paths:
            with open(p, "r", encoding="utf-8") as f:
                for i, line in enumerate(f):
                    if i == 0:
                        if not header_written:
                            out.write(line)
                            header_written = True
                        # skip subsequent headers
                        continue
                    if line.startswith("#"):
                        # skip summary/comment lines
                        continue
                    out.write(line)
    print(f"[merge] wrote {merged_path}")


def main():
    ap = argparse.ArgumentParser(description="Run RTT pinger over multiple payload sizes and merge CSVs")
    ap.add_argument("sizes", help="Comma list or range start:end:step, e.g. '256,512,1024' or '256:4096:256'")
    ap.add_argument("--per-size-duration", type=int, default=10, help="Seconds to run per size (default: 10)")
    ap.add_argument("--per-size-timeout", type=int, default=-1, help="Optional timeout per size (default: duration+5)")
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--req-topic", default="/latency_rtt_req")
    ap.add_argument("--rep-topic", default="/latency_rtt_rep")
    ap.add_argument("--qos-reliability", default="best_effort", choices=["best_effort", "reliable"])
    ap.add_argument("--qos-history", default="keep_last", choices=["keep_last", "keep_all"])
    ap.add_argument("--qos-depth", type=int, default=10)
    ap.add_argument("--intra-process", action="store_true")
    ap.add_argument("--transport-tag", default="")
    ap.add_argument("--notes", default="")
    ap.add_argument("--summary", action="store_true", help="Print summary per run")
    ap.add_argument("--append-summary", action="store_true", help="Append summary lines to per-size CSVs")
    ap.add_argument("--out", default="results/rtt_sweep.csv", help="Merged CSV output path")
    ap.add_argument("--tmp-dir", default="results/rtt_parts", help="Directory for per-size CSVs")
    args = ap.parse_args()

    try:
        sizes = parse_sizes(args.sizes)
    except Exception as e:
        print(f"Invalid sizes: {e}", file=sys.stderr)
        return 2
    if not sizes:
        print("No sizes to run.", file=sys.stderr)
        return 2

    tmp_dir = Path(args.tmp_dir)
    tmp_dir.mkdir(parents=True, exist_ok=True)
    per_csv_paths = []
    for s in sizes:
        csv_path = tmp_dir / f"rtt_payload_{s}.csv"
        run_pinger_once(s, csv_path, args)
        per_csv_paths.append(csv_path)

    merge_csvs(per_csv_paths, Path(args.out))
    print("Done.")


if __name__ == "__main__":
    sys.exit(main())

