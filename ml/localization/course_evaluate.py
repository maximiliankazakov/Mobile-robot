"""Reproduce the G1 coursework evaluation using the two C++ executables."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import re
import subprocess

METHODS = ("A", "B", "B-no-history")


def verify(path, digest):
    with path.open("rb") as data:
        actual = hashlib.file_digest(data, "sha256").hexdigest()
    if actual != digest:
        raise ValueError(f"Artifact hash mismatch: {path}")


def safe_id(value):
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", value):
        raise ValueError(f"Invalid identifier: {value}")
    return value


def selected_cases(inputs, scenario, case):
    manifest = json.loads((inputs / "index.json").read_text())
    if manifest["schema"] != 1:
        raise ValueError("Unsupported course dataset schema")
    entries = [entry for entry in manifest["cases"]
               if (scenario is None or entry["scenario"] == scenario)
               and (case is None or entry["id"] == case)]
    ids = [safe_id(entry["id"]) for entry in entries]
    if not ids or len(set(ids)) != len(ids):
        raise ValueError("Missing episodes or duplicate episode identifier")
    return manifest, entries


def infer(entry, args, contract):
    case = safe_id(entry["id"])
    output = args.predictions / case
    output.mkdir(parents=True, exist_ok=False)
    for variant, item in entry["variants"].items():
        source = args.inputs / case / safe_id(variant)
        verify(source / "map.txt", item["hashes"]["map.txt"])
        verify(source / "events.txt", item["hashes"]["events.txt"])
        work = output / variant
        work.mkdir()
        initial = [str(value) for value in item["initial"]]
        for method in METHODS:
            model = str(args.model / method / "model.txt") if method != "A" else "-"
            mode = "classical" if method == "A" else "learned"
            with (work / f"{method}.log").open("w") as log:
                subprocess.run([str(args.executable), str(source / "map.txt"), str(source / "events.txt"),
                                str(work / f"{method}.csv"), mode, model, *initial, str(contract["classical_scale"])],
                               check=True, stdout=log, stderr=subprocess.STDOUT)
            if "model status: ok" not in (work / f"{method}.log").read_text():
                raise ValueError(f"C++ replay did not validate the model: {case}, {variant}, {method}")


def score(entry, args):
    case = safe_id(entry["id"])
    outcomes = {}
    window = entry.get("window_sim_s")
    boundaries = [str(int(Decimal(value) * 10**9)) for value in window] if window else []
    for variant, item in entry["variants"].items():
        variant = safe_id(variant)
        gt = args.truth / case / variant / "ground_truth.csv"
        verify(gt, item["hashes"]["ground_truth.csv"])
        output = args.predictions / case / variant
        outcomes[variant] = {
            method: json.loads(subprocess.check_output([str(args.executable), str(gt),
                               str(output / f"{method}.csv"), *boundaries], text=True))
            for method in METHODS
        }
    return {"id":case, "layout":entry["layout"], "scenario":entry["scenario"], "variants":outcomes}


def aggregate(executable, destination, name, rows, metric_a, metric_b):
    path = destination / f"{name}.csv"
    with path.open("w", newline="") as stream:
        out = csv.writer(stream)
        out.writerow(["layout","A_m","B_m"])
        out.writerows((row["layout"], metric_a(row), metric_b(row)) for row in rows)
    return json.loads(subprocess.check_output([str(executable),"--paired",str(path)],text=True))


def summarize(scores, args):
    result = {}
    for scenario in sorted({row["scenario"] for row in scores}):
        rows = [row for row in scores if row["scenario"] == scenario]
        if len({row["layout"] for row in rows}) < 2:
            continue
        has_gap = "gap" in rows[0]["variants"]
        variant, metric = ("gap", "ate_rmse_window_m") if has_gap else ("observed", "ate_rmse_m")
        values = lambda method: lambda row: row["variants"][variant][method][metric]
        primary = aggregate(args.executable,args.results,scenario,values("A"),values("B"))
        history = aggregate(args.executable,args.results,scenario+"-ablation",values("A"),values("B-no-history"))
        result[scenario] = {"primary":primary, "ablation":history, "window":metric}
        if has_gap:
            for method in ("A","B"):
                intact = lambda row: row["variants"]["intact"][method][metric]
                penalty = aggregate(args.executable,args.results,f"{scenario}-penalty-{method}",
                                    intact,values(method))
                result[scenario][f"{method}_gap_minus_intact"] = penalty
            def penalty(method):
                return lambda row: row["variants"]["gap"][method][metric] - row["variants"]["intact"][method][metric]
            result[scenario]["difference_of_gap_penalties"] = aggregate(
                args.executable,args.results,f"{scenario}-penalty-difference",penalty("A"),penalty("B"))
    (args.results / "summary.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("infer","score"))
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--predictions", type=Path, required=True)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--truth", type=Path)
    parser.add_argument("--results", type=Path)
    parser.add_argument("--scenario")
    parser.add_argument("--case")
    parser.add_argument("--jobs", type=int, choices=(1,2,3,4), default=1)
    args = parser.parse_args()
    index, entries = selected_cases(args.inputs, args.scenario, args.case)
    if args.mode == "infer":
        if args.model is None:
            parser.error("infer requires --model")
        for method in METHODS[1:]:
            verify(args.model / method / "model.txt", index["model_hashes"][method])
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            list(pool.map(lambda entry: infer(entry,args,index), entries))
    else:
        if args.truth is None or args.results is None:
            parser.error("score requires --truth and --results")
        args.results.mkdir(parents=True,exist_ok=True)
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            outcomes = list(pool.map(lambda entry: score(entry,args), entries))
        (args.results / "per-episode.json").write_text(json.dumps(outcomes,indent=2,allow_nan=False)+"\n")
        summarize(outcomes,args)
        print(f"Scored {len(outcomes)} held-out episodes using C++")


if __name__ == "__main__":
    main()
