#!/usr/bin/env python3
"""Summarize VINS recovery runs and optionally evaluate against EuRoC Leica."""

import argparse
import json
import pathlib
import re
import statistics
import subprocess


def parse_log_stats(run: pathlib.Path) -> dict:
    acc = []
    reasons = {}
    request_lines = 0
    vlog = run / "vins_node.log"
    if vlog.exists():
        for line in vlog.read_text(errors="ignore").splitlines():
            m = re.search(r"accepted (\d+) tracks", line)
            if m:
                acc.append(int(m.group(1)))
            if "recovery request health" in line:
                request_lines += 1
                m = re.search(r"reasons=([^\x1b]*)", line)
                if m:
                    for reason in m.group(1).strip().split():
                        reasons[reason] = reasons.get(reason, 0) + 1

    inf = []
    pub = []
    skip = 0
    llog = run / "lightglue_recovery_candidate_node.log"
    if llog.exists():
        for line in llog.read_text(errors="ignore").splitlines():
            if "skip recovery request" in line:
                skip += 1
            m = re.search(r"published (\d+) ONNX.*inference ([0-9.]+) ms", line)
            if m:
                pub.append(int(m.group(1)))
                inf.append(float(m.group(2)))

    return {
        "accepted_lines": len(acc),
        "accepted_total": sum(acc),
        "request_lines": request_lines,
        "reasons": reasons,
        "logged_publish_lines": len(pub),
        "logged_published_total": sum(pub),
        "skip": skip,
        "inf_median_ms": statistics.median(inf) if inf else 0.0,
        "inf_mean_ms": statistics.mean(inf) if inf else 0.0,
        "inf_max_ms": max(inf) if inf else 0.0,
    }


def evaluate(eval_script: str, bag: str, vio: pathlib.Path) -> dict:
    if not vio.exists():
        return {"vio_exists": False}
    out = subprocess.check_output(["python3", eval_script, bag, str(vio)], text=True, stderr=subprocess.STDOUT)
    result = {"vio_exists": True}
    for line in out.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key] = value
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval-script", required=True)
    parser.add_argument("items", nargs="+", help="scenario,mode,bag,run")
    args = parser.parse_args()

    for item in args.items:
        scenario, mode, bag, run_dir = item.split(",", 3)
        run = pathlib.Path(run_dir)
        row = {"scenario": scenario, "mode": mode, "run": str(run)}
        row.update(evaluate(args.eval_script, bag, run / "vio.csv"))
        row.update(parse_log_stats(run))
        print(json.dumps(row, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
