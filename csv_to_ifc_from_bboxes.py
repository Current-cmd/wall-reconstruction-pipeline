#!/usr/bin/env python3
"""
Wrapper: convert a *CSV* of wall_bboxes directly to IFC.

Usage:
    python csv_to_ifc_from_bboxes.py \
        output4/global/bboxes_all_up_to_03.csv \
        output4/global/walls_up_to_03.ifc
"""

import sys
from pathlib import Path

# import the function from your existing script
from bbox_obj_to_ifc_converter import csv_bboxes_to_ifc  # type: ignore


def main(csv_path: str, out_ifc: str):
    csv_p = Path(csv_path)
    out_p = Path(out_ifc)

    if not csv_p.exists():
        sys.exit(f"[ERROR] CSV not found: {csv_p}")

    csv_bboxes_to_ifc(csv_p, out_p)
    print(f"[OK] IFC written to {out_p}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(
            "Usage:\n"
            "  python csv_to_ifc_from_bboxes.py bboxes.csv output.ifc"
        )
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])
