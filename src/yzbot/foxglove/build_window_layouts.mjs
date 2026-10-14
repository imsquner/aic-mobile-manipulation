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

// Display groups stay small; backend fine-grained states remain unchanged.
const rule = (rawValue, label, color) => ({ operator: "=", rawValue, label, color });
const blue = "#175cd3", grey = "#667085", green = "#039855", yellow = "#b54708", red = "#d92d20";
config["Indicator!state"].fallbackLabel = "未知";
config["Indicator!state"].fallbackColor = grey;
config["Indicator!state"].rules = [
  rule("BOOTING", "初始化", grey), rule("READY", "就绪", green),
  ...["READING_QUESTION", "NAVIGATING_TO_OBJECT", "PRE_GRASP", "GRASPING", "NAVIGATING_TO_ZONE", "PLACING", "VERIFYING", "ITEM_SUCCEEDED", "ITEM_FAILED"].map(v => rule(v, "运行中", blue)),
  rule("CANCELLING", "取消中", yellow), rule("CANCELLED", "已取消", yellow),
  rule("RESETTING", "复位中", blue),
  rule("TASK_SUCCEEDED", "已结束", green), rule("TASK_FAILED", "已结束", red), rule("TIME_LIMIT", "已结束", yellow),
  ...["API_ERROR", "INITIALIZATION_FAILED", "CANCEL_FAILED", "RESET_FAILED"].map(v => rule(v, "异常", red)),
];
config["Indicator!api"].rules = [
  ...["EMPTY", "CANCELLED", "PLAN_INVALIDATED"].map(v => rule(v, "未准备", grey)),
  rule("READING_QUESTION", "出题中", blue), rule("WAITING_MAPPING", "等映射", yellow),
  ...["CALLING_LLM", "VALIDATING_TASK"].map(v => rule(v, "解析中", blue)),
  rule("PLAN_READY", "可开始", green), rule("PARSE_FAILED", "失败", red),
];
config["Indicator!api"].fallbackLabel = "未知";
config["Indicator!nav"].fallbackLabel = "未知";
config["Indicator!arm"].fallbackLabel = "未知";
config["Indicator!nav"].rules.push(rule("STOPPING", "停止中", yellow), rule("STOPPED", "已停止", yellow));
config["Indicator!arm"].rules.push(
  rule("STOPPING", "停止中", yellow), rule("STOPPED", "已停止", yellow),
  rule("RECOGNITION_POSE", "识别位", green), rule("RETURNING_RECOGNITION_POSE", "回识别位", blue),
  ...Object.entries({LATCH_PREGRASP:"IK校验", PLAN_PICK:"规划", EXECUTE_PREGRASP:"预抓取", EXECUTE_OBSERVATION:"观察位", EXECUTE_DESCEND:"下降", ATTACH_TARGET:"吸附", EXECUTE_LIFT:"抬升", RETURN_ARM:"回识别位", EXECUTING_PICK_SEQUENCE_RETRY:"重试"})
    .map(([value, label]) => rule(value, label, blue)),
);
for (const [id, title, path] of [
  ["step", "当前步骤", "/competition/current_step"],
  ["mapping", "有效映射", "/competition/mapping_rule"],
  ["error", "错误", "/competition/last_error"],
  ["updated", "更新时间", "/competition/state_updated"],
]) {
  config[`RawMessages!${id}`] = { ...structuredClone(config["RawMessages!question"]), topicPath: path, foxglovePanelTitle: title };
}
config["Indicator!attachment"] = {
  ...structuredClone(config["Indicator!state"]), path: "/competition/attachment_state.data", foxglovePanelTitle: "吸附",
  rules: [rule("UNKNOWN","未知",yellow),rule("ATTACHED","已吸附",blue),rule("DETACHED","未吸附",green),rule("DETACHING","释放中",yellow)],
};
config["Indicator!connection"] = {
  ...structuredClone(config["Indicator!state"]), path: "/competition/api_connection_status.data", foxglovePanelTitle: "API连接",
  rules: [rule("CHECKING","检测中",blue),rule("AVAILABLE","可用",green),rule("UNAVAILABLE","不可用",red)],
};
config["Indicator!color"].foxglovePanelTitle = "本轮颜色";
config["Indicator!zone"].foxglovePanelTitle = "本轮区域";
config["Publish!resubmit"].buttonText = "重解析";
config["Publish!generate"].buttonText = "出题";
config["Publish!mappingInput"].buttonText = "映射";
// One judging page follows the published layout; secondary data stays in tabs.
config["Publish!manualQuestion"] = structuredClone(source.configById["Publish!manualQuestion"]);
for (const [id, title, button] of [
  ["manualQuestion", "题目确认", "确认题目"],
  ["mappingInput", "映射确认", "确认映射"],
]) {
  Object.assign(config[`Publish!${id}`], { foxglovePanelTitle: title, buttonText: button,
    advancedView: true, value: '{"data":""}' });
}
config["3D!competitionMap"] = JSON.parse(readFileSync(join(here, "window_map.json"), "utf8")).configById["3D!competitionMap"];
config["3D!competitionMap"].foxglovePanelTitle = "三维 / 导航路径";
config["Image!base"] = structuredClone(source.configById["Image!rgb"]);
Object.assign(config["Image!base"], { foxglovePanelTitle: "底盘摄像头",
  imageMode: { imageTopic: "/camera/image_raw", calibrationTopic: "/camera/camera_info", synchronize: false, rotation: 0 } });
config["Image!wrist"] = structuredClone(source.configById["Image!rgb"]);
config["Image!wrist"].foxglovePanelTitle = "机械臂识别摄像头";
const raw = (id, title, topic, fontSize = 24) => {
  config[`RawMessages!${id}`] = { ...structuredClone(source.configById["RawMessages!question"]),
    topicPath: `${topic}.data`, fontSize, foxglovePanelTitle: title };
};
raw("firstGrab", "先抓取", "/competition/first_grab", 26);
raw("laterGrab", "后抓取", "/competition/later_grab", 26);
config["Indicator!redTotal"].foxglovePanelTitle = "红色数量";
config["Indicator!blueTotal"].foxglovePanelTitle = "蓝色数量";
config["Indicator!redTotal"].fontSize = config["Indicator!blueTotal"].fontSize = 34;
config["Indicator!task"] = structuredClone(source.configById["Indicator!state"]);
config["Indicator!task"].foxglovePanelTitle = "当前任务";
config["Indicator!task"].fallbackLabel = "未知";
config["Indicator!task"].rules.push(...config["Indicator!state"].rules.filter(r =>
  !config["Indicator!task"].rules.some(original => original.rawValue === r.rawValue)));
config["Indicator!attachment"].foxglovePanelTitle = "状态 / 吸附释放";
config["Indicator!attachment"].rules = [
  rule("UNKNOWN", "未知", yellow), rule("ATTACHED", "已吸附", blue),
  rule("DETACHED", "已释放", green), rule("DETACHING", "释放中", yellow),
];
config["Indicator!item"] = { ...structuredClone(source.configById["Indicator!item"]),
  foxglovePanelTitle: "任务进度", fontSize: 30, fallbackLabel: "未知",
  rules: [rule("0", "未开始", grey), ...Array.from({length:10}, (_, i) => rule(String(i+1), `第${i+1}个`, green))] };
for (const id of ["task", "attachment", "state"]) config[`Indicator!${id}`].fontSize = 28;
config["RawMessages!elapsed"].fontSize = 18;
config["RawMessages!elapsed"].topicPath = "/competition/elapsed_seconds.data";
raw("completed", "已完成数量", "/competition/completed_item_count", 18);
raw("question", "已确认题目", "/competition/question", 18);
raw("mapping", "已确认映射", "/competition/mapping_rule", 18);
raw("planText", "抓取顺序 / 任务表", "/competition/task_plan_text", 18);
config["Image!detection"] = structuredClone(source.configById["Image!detection"]);
config["Image!depth"] = structuredClone(source.configById["Image!depth"]);
const columns = (ids) => ids.reduceRight((tail, id, i) => tail === null ? id : split(id, tail, "row", 100/(ids.length-i)), null);
const quantityOrder = split(columns(["Indicator!redTotal", "Indicator!blueTotal"]),
  columns(["RawMessages!firstGrab", "RawMessages!laterGrab"]), "row", 44);
const cameras = split("Image!base", "Image!wrist", "row", 50);
const scene = split(cameras, "3D!competitionMap", "column", 44);
const current = split("Indicator!item", split("Indicator!task", split("Indicator!attachment",
  split("Indicator!state", "Publish!start", "column", 65), "column", 40), "column", 40), "column", 24);
const body = split(scene, current, "row", 72);
raw("resetSummary", "复位确认", "/competition/reset_summary", 18);
const footer = columns(["Publish!cancel", "Publish!reset", "Publish!resubmit", "RawMessages!elapsed", "RawMessages!completed"]);
const main = split("Publish!manualQuestion", split("Publish!mappingInput", split(quantityOrder,
  split(body, split("RawMessages!resetSummary", footer, "column", 35), "column", 89), "column", 15), "column", 11), "column", 10);
const diagnostics = grid([
  ["Indicator!network", "Indicator!connection"],
  ["Indicator!api", "RawMessages!error"],
  ["Indicator!nav", "Indicator!arm"],
  ["Indicator!redDone", "Indicator!blueDone"],
  ["Indicator!color", "Indicator!zone"],
  ["RawMessages!step", "RawMessages!updated"],
  ["RawMessages!planJson"],
]);
const auxiliary = split(grid([["Publish!generate"], ["RawMessages!question"],
  ["RawMessages!mapping"], ["RawMessages!planText"]]),
  split("Image!detection", "Image!depth", "column", 50), "row", 60);
config["Tab!review"] = {activeTabIdx:0, tabs:[
  {title:"评审",layout:main}, {title:"诊断",layout:diagnostics}, {title:"辅助",layout:auxiliary}]};
output("window_dashboard.json", config, "Tab!review");
