"""Read the benchmark producer's exports through the TUI and compare the displayed results."""

import csv
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from tempfile import TemporaryDirectory

from AnalyticsTerminal import expect, full_frame, mouse_at, resize_frame, terminal


def check_report(executable, path, case, mode):
    with terminal(executable, "--report", path) as master:
        frame = expect(master, b"o Open report").decode()
        scope = "detection_s" if mode == "replay" else "total_step_s"
        assert ("Replay · detection time" if mode == "replay" else "Simulation · total step time") in frame
        for variant in case["variants"]:
            timing = variant["summary"][scope]
            speedup = variant["summary"]["speedup_vs_brute_force"]
            columns = [variant["algorithm"], str(variant["workers"]), f'{1e6 * timing["median"]:.2f}',
                       f'{1e6 * timing["p95"]:.2f}', "—" if speedup is None else f"{speedup:.2f}×",
                       variant["validation"]]
            assert re.search(r"\s+".join(map(re.escape, columns)), frame), (columns, frame)

        os.write(master, b"\x1b[C")
        expect(master, b"aligned")
        os.write(master, b"\r")
        expect(master, b"Repetition 1/2")
        os.write(master, b"\x1b")
        expect(master, b"Algorithm")


def test_open_from_live(executable, path):
    with terminal(executable) as master:
        expect(master, b"Press n for one tick")
        os.write(master, b"n")
        expect(master, b"8128")
        os.write(master, b"o")
        expect(master, b"Open saved benchmark")
        os.write(master, os.fsencode(path) + b"\r")
        expect(master, b"Saved benchmark")
        os.write(master, b"o")
        expect(master, b"Open saved benchmark")
        os.write(master, os.fsencode(path.parent / "missing") + b"\r")
        expect(master, b"existing report file")
        os.write(master, b"\x1b")
        expect(master, b"Saved benchmark")
        os.write(master, b"\x1b")
        expect(master, b"8128")


def test_report_mouse(executable, path):
    with terminal(executable) as master:
        frame = full_frame(master)
        mouse_at(master, frame, "o Open report")
        expect(master, b"Open saved benchmark")
        frame = resize_frame(master, 81)
        os.write(master, os.fsencode(path))
        mouse_at(master, frame, "Enter Open")
        expect(master, b"Saved benchmark")
        frame = resize_frame(master, 80)
        mouse_at(master, frame, "→")
        expect(master, b"aligned")
        mouse_at(master, frame, "sweep-and-prune")
        frame = resize_frame(master, 81)
        selected = next(line for line in frame if "Variant " in line)
        mouse_at(master, frame, "Algorithm", button=64)
        frame = resize_frame(master, 80)
        assert selected != next(line for line in frame if "Variant " in line)
        mouse_at(master, frame, "Enter Details")
        expect(master, b"Repetition 1/2")
        frame = resize_frame(master, 81)
        mouse_at(master, frame, "→")
        mouse_at(master, frame, "Detection", button=65)
        frame = resize_frame(master, 80)
        assert any("Repetition 2/2   Tick 3" in line for line in frame)
        mouse_at(master, frame, "Esc Back")
        expect(master, b"Algorithm")
        frame = resize_frame(master, 81)
        mouse_at(master, frame, "Esc Live")
        expect(master, b"Salvage Analytics")


def main(executable, producer, threaded):
    with TemporaryDirectory() as directory:
        root = Path(directory)
        for mode in ("simulation", "replay"):
            report, ticks = root / f"{mode} report.json", root / f"{mode} ticks.csv"
            args = [producer, "--mode", mode, "--scenario", "dense,aligned", "--bodies", "16",
                    "--workers", "1,2" if threaded == "ON" else "1", "--warmup", "1", "--steps", "3",
                    "--repetitions", "2", "--json", report, "--csv", ticks]
            if mode == "replay":
                args.append("--no-validate")
            subprocess.run(args, check=True, capture_output=True)
            doc = json.loads(report.read_bytes())
            for path in (report, ticks):
                check_report(executable, path, doc["cases"][0], mode)
        test_open_from_live(executable, ticks)
        test_report_mouse(executable, ticks)

        # Preserve failed variants, including quoted errors in CSV and absent summaries.
        variant = doc["cases"][0]["variants"][0]
        error = 'Could not prepare "dense", seed 1'
        variant.update(status="failed", validation="not-run", error=error, summary=None, runs=[])
        report.write_text(json.dumps(doc))
        with ticks.open(newline="") as source:
            reader = csv.DictReader(source)
            rows = list(reader)
            fields = reader.fieldnames
        row = rows[0]
        row.update(status="failed", validation="not-run", error=error, repetition="", tick="")
        with ticks.open("w", newline="") as out:
            writer = csv.DictWriter(out, fieldnames=fields)
            writer.writeheader()
            writer.writerow(row)
        for path in (report, ticks):
            with terminal(executable, path) as master:
                expect(master, error.encode())
                os.write(master, b"\r")
                expect(master, b"No measured ticks were saved")

        doc["schema_version"] = 2
        report.write_text(json.dumps(doc))
        invalid = subprocess.run([executable, report], capture_output=True)
        assert invalid.returncode and b"Unsupported benchmark schema" in invalid.stderr
        with ticks.open("w", newline="") as out:
            writer = csv.DictWriter(out, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows[1:])
        invalid = subprocess.run([executable, ticks], capture_output=True)
        assert invalid.returncode and b"Incomplete benchmark repetition" in invalid.stderr


if __name__ == "__main__":
    main(*sys.argv[1:])
