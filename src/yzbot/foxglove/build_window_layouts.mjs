import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const source = JSON.parse(
  readFileSync(join(here, "competition_final_legacy_v1.json"), "utf8"),
);
const split = (first, second, direction = "row", splitPercentage = 50) => ({
  first,
  second,
  direction,
  splitPercentage,
});
const grid = (rows) => {
  const rowLayouts = rows.map((row) =>
    row.length === 1 ? row[0] : split(row[0], row[1], "row", 50),
  );
  return rowLayouts.reduceRight((tail, row, index) =>
    tail === null
      ? row
      : split(row, tail, "column", 100 / (rowLayouts.length - index)), null);
};
const select = (ids) => Object.fromEntries(ids.map((id) => [id, structuredClone(source.configById[id])]));
const output = (name, configById, layout) => {
  writeFileSync(
    join(here, name),
    `${JSON.stringify({ configById, globalVariables: {}, userNodes: {}, playbackConfig: { speed: 1 }, layout }, null, 2)}\n`,
    "utf8",
  );
};

output(
  "window_map.json",
  select(["3D!competitionMap"]),
  "3D!competitionMap",
);
const detectionConfig = select(["Image!detection"]);
detectionConfig["Image!detection"].foxglovePanelTitle = "OpenCV目标检测";
output("window_detection.json", detectionConfig, "Image!detection");

const statusIds = [
  "Indicator!redTotal",
  "Indicator!blueTotal",
  "Indicator!redDone",
  "Indicator!blueDone",
  "Indicator!color",
  "Indicator!zone",
  "Indicator!nav",
  "Indicator!arm",
  "Indicator!network",
  "Indicator!api",
  "Indicator!state",
];
const config = select([
  "RawMessages!question",
  "RawMessages!planJson",
  "Publish!mappingInput",
  "Publish!start",
  "Publish!resubmit",
  "Publish!cancel",
  "Publish!reset",
  ...statusIds.filter((id) => source.configById[id]),
]);
config["RawMessages!question"].foxglovePanelTitle = "1 题目生成结果";
config["RawMessages!planJson"].foxglovePanelTitle = "3 DS解析有序任务表";
config["Publish!mappingInput"].foxglovePanelTitle = "2 映射输入";
config["Publish!mappingInput"].buttonText = "确认映射";
config["Publish!mappingInput"].value =
  '{"data":"x 代表红色,y 代表蓝色,数量大于等于 3 的去 A 区,其他颜色去 C 区"}';
config["Publish!generate"] = {
  buttonText: "生成题目",
  buttonTooltip: "",
  advancedView: false,
  value: '{"data":true}',
  foxglovePanelTitle: "1 题目生成",
  topicName: "/competition/generate_question",
  datatype: "std_msgs/Bool",
};
config["Indicator!network"] = {
  path: "/competition/network_status.data",
  style: "background",
  fontSize: 20,
  fallbackColor: "#667085",
  fallbackLabel: "等待",
  rules: [
    { operator: "=", rawValue: "ONLINE", color: "#039855", label: "在线" },
    { operator: "=", rawValue: "OFFLINE", color: "#d92d20", label: "离线" },
  ],
  foxglovePanelTitle: "网络状态",
};
// 已运行总耗时：不能再用 Indicator 面板。
// Indicator 只会显示「规则命中的静态文字」（其它面板是把 0..5 之类的小枚举逐条列出来），
// 而耗时是连续增长的数字、无法枚举，`rules: []` 时它永远只显示 fallbackLabel（现场表现为
// 「一直显示 0、点开始也不动」）。改用 RawMessages 面板，直接显示 /competition/elapsed_seconds
// 的实时数值。
config["RawMessages!elapsed"] = {
  ...structuredClone(source.configById["RawMessages!question"]),
  topicPath: "/competition/elapsed_seconds",
  fontSize: 20,
  foxglovePanelTitle: "已运行总耗时(s)",
};
config["Indicator!redTotal"].foxglovePanelTitle = "红色目标总数";
config["Indicator!blueTotal"].foxglovePanelTitle = "蓝色目标总数";
config["Indicator!arm"].rules.push({
  operator: "=", rawValue: "EXECUTING_PICK_SEQUENCE", color: "#6941c6", label: "抓取流水线",
});

const workflow = grid([
  ["Publish!generate", "RawMessages!question"],
  ["Publish!mappingInput"],
  ["RawMessages!planJson"],
  ["Publish!start", "Publish!resubmit"],
  ["Publish!cancel", "Publish!reset"],
]);
const statuses = grid([
  ["Indicator!redTotal", "Indicator!blueTotal"],
  ["Indicator!redDone", "Indicator!blueDone"],
  ["Indicator!color", "Indicator!zone"],
  ["Indicator!nav", "Indicator!arm"],
  ["Indicator!network", "Indicator!api"],
  ["RawMessages!elapsed"],
  ["Indicator!state"],
]);
output("window_dashboard.json", config, split(workflow, statuses, "row", 54));
