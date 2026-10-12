#!/usr/bin/env python3
"""Create content-addressed Foxglove roots with independent default layouts."""

import argparse
import hashlib
import json
import os
import re
import shutil
import tempfile
from pathlib import Path


ASSIGNMENT = re.compile(
    r"globalThis\.FOXGLOVE_STUDIO_DEFAULT_LAYOUT = \[.*?\]\[0\];",
    re.DOTALL,
)
MARKER = "/*FOXGLOVE_STUDIO_DEFAULT_LAYOUT_PLACEHOLDER*/"


def prepare(base_root, cache_root, layout_path):
    layout = json.loads(layout_path.read_text(encoding="utf-8"))
    if "configById" not in layout or "layout" not in layout:
        raise ValueError(f"invalid layout: {layout_path}")
    digest = hashlib.sha256()
    digest.update((base_root / "index.html").read_bytes())
    digest.update(layout_path.read_bytes())
    target = cache_root / f"window-{layout_path.stem}-{digest.hexdigest()[:16]}"
    if (target / "index.html").is_file():
        return target

    cache_root.mkdir(parents=True, exist_ok=True)
    if target.exists():
        resolved_cache = cache_root.resolve()
        resolved_target = target.resolve()
        if resolved_cache not in resolved_target.parents:
            raise RuntimeError(f"refusing to remove cache path outside root: {target}")
        shutil.rmtree(target)
    temporary = Path(tempfile.mkdtemp(prefix="foxglove-window-", dir=cache_root))
    try:
        shutil.copytree(base_root, temporary / "root", copy_function=os.link)
        root = temporary / "root"
        index_path = root / "index.html"
        text = index_path.read_text(encoding="utf-8")
        serialized = json.dumps(layout, ensure_ascii=False, separators=(",", ":"))
        if MARKER in text:
            updated = text.replace(MARKER, serialized, 1)
        else:
            updated, count = ASSIGNMENT.subn(
                f"globalThis.FOXGLOVE_STUDIO_DEFAULT_LAYOUT = [{serialized}][0];",
                text,
                count=1,
            )
            if count != 1:
                raise RuntimeError("Foxglove default-layout assignment was not found")
        index_path.unlink()
        index_path.write_text(updated, encoding="utf-8")
        root.replace(target)
    finally:
        shutil.rmtree(temporary, ignore_errors=True)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-root", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--layout", type=Path, action="append", required=True)
    args = parser.parse_args()
    for layout_path in args.layout:
        target = prepare(args.base_root, args.cache_root, layout_path)
        print(f"{layout_path.stem}={target}")


if __name__ == "__main__":
    main()
