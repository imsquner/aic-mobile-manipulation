import { readFileSync, writeFileSync } from "node:fs";

const [indexPath, layoutPath] = process.argv.slice(2);
if (!indexPath || !layoutPath) {
  throw new Error("usage: node inject_default_layout.mjs <index.html> <layout.json>");
}

const marker = "/*FOXGLOVE_STUDIO_DEFAULT_LAYOUT_PLACEHOLDER*/";
const index = readFileSync(indexPath, "utf8");
const markerCount = index.split(marker).length - 1;

const layout = JSON.parse(readFileSync(layoutPath, "utf8"));
if (!layout.configById || !layout.layout) {
  throw new Error("layout must contain configById and layout");
}

const serialized = JSON.stringify(layout);
let updated;
if (markerCount === 1) {
  updated = index.replace(marker, serialized);
} else if (markerCount === 0) {
  const assignment = /globalThis\.FOXGLOVE_STUDIO_DEFAULT_LAYOUT = \[.*?\]\[0\];/s;
  if (!assignment.test(index)) {
    throw new Error("existing default layout assignment not found");
  }
  updated = index.replace(
    assignment,
    `globalThis.FOXGLOVE_STUDIO_DEFAULT_LAYOUT = [${serialized}][0];`,
  );
} else {
  throw new Error(`expected at most one layout marker, found ${markerCount}`);
}
writeFileSync(indexPath, updated, "utf8");
console.log(`injected ${Object.keys(layout.configById).length} panels into ${indexPath}`);
