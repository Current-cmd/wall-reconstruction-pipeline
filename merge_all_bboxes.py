#!/usr/bin/env python3
"""
Merge wall_bboxes.csv files from multiple batches into one CSV.

Usage:
    python merge_all_bboxes.py "output4/*_bboxes/wall_bboxes.csv" \
        output4/global/bboxes_all_up_to_03.csv
"""

import glob
import sys
import pandas as pd


def main(pattern: str, out_csv: str):
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"[ERROR] No CSV files matching pattern: {pattern}")
        sys.exit(1)

    print(f"[INFO] Merging {len(files)} CSV files:")
    for f in files:
        print("   ", f)

    dfs = []
    next_id = 1
    for f in files:
        df = pd.read_csv(f)
        if df.empty:
            continue
        # give unique wall IDs across all batches
        n = len(df)
        df = df.copy()
        df["wall_id"] = range(next_id, next_id + n)
        next_id += n
        dfs.append(df)

    if not dfs:
        print("[ERROR] No non-empty CSVs to merge.")
        sys.exit(1)

    all_df = pd.concat(dfs, ignore_index=True)
    all_df.to_csv(out_csv, index=False)
    print(f"[OK] Wrote merged CSV with {len(all_df)} walls to: {out_csv}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(
            "Usage:\n"
            "  python merge_all_bboxes.py \"pattern\" out.csv\n\n"
            "Example:\n"
            "  python merge_all_bboxes.py "
            "\"output4/*_bboxes/wall_bboxes.csv\" "
            "output4/global/bboxes_all_up_to_03.csv"
        )
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])
