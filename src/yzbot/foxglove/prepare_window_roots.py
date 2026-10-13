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
PUBLISH_CHUNK = "4000.91ec4ce57566ceca7123.js"  # bundled offline Studio 1.87.0
JSON_EDITOR = 'placeholder:"Enter message content as JSON",value:e.value,onChange:x=>{n({value:x.target.value})}'
TEXT_TOPICS = '(e.topicName==="/competition/manual_question"||e.topicName==="/competition/mapping_rule_input")'
TEXT_EDITOR = (
    'placeholder:' + TEXT_TOPICS + '?"输入题目或映射规则":"Enter message content as JSON",'
    'value:' + TEXT_TOPICS + '?(()=>{try{return JSON.parse(e.value||"{}").data||""}catch{return ""}})():e.value,'
    'onChange:x=>{n({value:' + TEXT_TOPICS + '?JSON.stringify({data:x.target.value}):x.target.value})}'
)

# Keep Foxglove's native ROS publisher. Only arrange its input and button side by side.
REVIEW_CSS = """
:is([data-testid="panel-mouseenter-container Publish!manualQuestion"],
    [data-testid="panel-mouseenter-container Publish!mappingInput"]) > div > div:last-child {
  flex-direction: row !important; align-items: center !important; gap: 12px; padding: 4px 12px !important;
}
:is([data-testid="panel-mouseenter-container Publish!manualQuestion"],
    [data-testid="panel-mouseenter-container Publish!mappingInput"]) > div > div:last-child > div:first-child {
  flex: 1 1 auto !important; min-width: 0; width: auto !important;
}
:is([data-testid="panel-mouseenter-container Publish!manualQuestion"],
    [data-testid="panel-mouseenter-container Publish!mappingInput"]) > div > div:last-child > div:last-child {
  flex: 0 0 auto !important; padding: 0 !important; display: flex; align-items: center; gap: 8px;
}
/* Without a data source the publisher adds a long "Connect to a data source…" hint
   next to the button. Let the hint truncate; a fixed width squeezed the button into
   two lines (observed at the 1560x1000 judging window). */
:is([data-testid="panel-mouseenter-container Publish!manualQuestion"],
    [data-testid="panel-mouseenter-container Publish!mappingInput"]) > div > div:last-child > div:last-child > div:first-child {
  white-space: nowrap; overflow: hidden; text-overflow: ellipsis; max-width: 240px;
}
:is([data-testid="panel-mouseenter-container Publish!manualQuestion"],
    [data-testid="panel-mouseenter-container Publish!mappingInput"]) > div > div:last-child button {
  white-space: nowrap !important; flex: 0 0 auto !important;
}
:is([data-testid="panel-mouseenter-container Publish!manualQuestion"],
    [data-testid="panel-mouseenter-container Publish!mappingInput"]) textarea:not([aria-hidden="true"]) {
  height: 42px !important; overflow-y: auto !important;
}
"""

# Foxglove's RawMessages panel replaces the mosaic title with its own toolbar (topic
# selector, diff toggle, collapse all), so `foxglovePanelTitle` never reaches the
# screen for it. The judging page therefore hides that toolbar behind a label that is
# generated from `foxglovePanelTitle` itself, and the label must not flex-shrink or
# long titles get clipped (observed as "耗时(s)" rendering as "时(s)").
RAW_HEADER_CSS = (
    '[data-testid^="panel-mouseenter-container RawMessages!"] header::before '
    "{ white-space: nowrap; flex: 0 0 auto; margin-right: auto; padding-left: 6px; }"
)
JUDGING_TAB = 0


def _panel_ids(node, found=None):
    """Panel ids of a mosaic layout tree (a panel id, or a split of two subtrees)."""
    if found is None:
        found = []
    if isinstance(node, str):
        found.append(node)
    elif isinstance(node, dict):
        for side in ("first", "second"):
            if side in node:
                _panel_ids(node[side], found)
    return found


def _review_css(layout):
    """Judging-page CSS. Labels come from foxglovePanelTitle so they cannot drift."""
    tab_panel = layout["configById"].get(layout["layout"])
    tabs = tab_panel.get("tabs") if isinstance(tab_panel, dict) else None
    if not tabs:
        return ""
    judging = set(_panel_ids(tabs[JUDGING_TAB]["layout"]))
    labels, hides = [], []
    for panel_id, config in layout["configById"].items():
        if not panel_id.startswith("RawMessages!") or not isinstance(config, dict):
            continue
        title = config.get("foxglovePanelTitle")
        if not title:
            continue
        if '"' in title or "\\" in title:
            raise ValueError(f"panel title needs CSS escaping: {panel_id}")
        testid = f'[data-testid="panel-mouseenter-container {panel_id}"]'
        labels.append(f'{testid} header::before {{ content: "{title}"; }}')
        if panel_id in judging:
            # Keep the mosaic buttons (last child); drop the topic/diff/collapse row.
            hides.append(f"{testid} header > *:not(:last-child)")
    if not labels:
        return ""
    blocks = [REVIEW_CSS.strip(), RAW_HEADER_CSS]
    if hides:
        blocks.append(",\n".join(hides) + " { display: none; }")
    blocks.extend(labels)
    return '<style id="aic-review-layout">\n' + "\n".join(blocks) + "\n</style>\n"


def prepare(base_root, cache_root, layout_path):
    layout = json.loads(layout_path.read_text(encoding="utf-8"))
    if "configById" not in layout or "layout" not in layout:
        raise ValueError(f"invalid layout: {layout_path}")
    digest = hashlib.sha256()
    digest.update((base_root / "index.html").read_bytes())
    digest.update(layout_path.read_bytes())
    review_style = _review_css(layout) if "Tab!review" in layout["configById"] else ""
    digest.update(review_style.encode("utf-8"))
    if review_style:
        digest.update(TEXT_EDITOR.encode("utf-8"))
        digest.update((base_root / PUBLISH_CHUNK).read_bytes())
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
        if review_style:
            updated = updated.replace("</head>", review_style + "</head>", 1)
            # Preserve the native advertise/publish, capability checks and config save.
            # Only its two std_msgs/String editors expose plain text to the operator.
            chunk_path = root / PUBLISH_CHUNK
            chunk = chunk_path.read_text(encoding="utf-8")
            if chunk.count(JSON_EDITOR) != 1:
                raise RuntimeError("offline Studio Publish editor differs from version 1.87.0")
            chunk_path.unlink()  # break the hard link before changing the cached copy
            chunk_path.write_text(chunk.replace(JSON_EDITOR, TEXT_EDITOR, 1), encoding="utf-8")
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
