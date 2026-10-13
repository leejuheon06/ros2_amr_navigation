#!/usr/bin/env python3
"""Create simple portfolio-ready plots from planner_benchmark.csv."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path
import statistics

import matplotlib.pyplot as plt


def load_rows(path: Path):
    rows = []
    with path.open('r', encoding='utf-8') as handle:
        for row in csv.DictReader(handle):
            if row.get('success') != '1':
                continue
            rows.append(row)
    return rows


def grouped(rows, field):
    values = defaultdict(list)
    for row in rows:
        values[row['planner']].append(float(row[field]))
    return values


def bar_plot(rows, field, ylabel, title, output):
    values = grouped(rows, field)
    planners = ['Dijkstra', 'AStar']
    means = [statistics.mean(values[p]) for p in planners]
    errors = [statistics.stdev(values[p]) if len(values[p]) > 1 else 0.0 for p in planners]

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    ax.bar(planners, means, yerr=errors, capsize=5)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis='y', alpha=0.25)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file')
    parser.add_argument('--output-dir', default=None)
    args = parser.parse_args()

    csv_path = Path(args.csv_file).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else csv_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_rows(csv_path)
    if not rows:
        raise RuntimeError('No successful benchmark rows found')

    bar_plot(
        rows,
        'plugin_planning_ms',
        'Planning time [ms]',
        'Custom Global Planner — Planning Time',
        output_dir / 'planning_time_comparison.png',
    )
    bar_plot(
        rows,
        'expanded_nodes',
        'Expanded nodes',
        'Custom Global Planner — Search Effort',
        output_dir / 'expanded_nodes_comparison.png',
    )
    bar_plot(
        rows,
        'result_path_length_m',
        'Path length [m]',
        'Custom Global Planner — Path Length',
        output_dir / 'path_length_comparison.png',
    )

    print(f'Plots saved to: {output_dir}')


if __name__ == '__main__':
    main()
