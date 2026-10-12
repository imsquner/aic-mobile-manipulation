import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const referencePath = join(here, "..", "..", "..", "可视化布局文件参考", "demo1.json");
const outputPath = join(here, "competition_final_legacy_v1.json");
const reference = JSON.parse(readFileSync(referencePath, "utf8"));

const configById = {};
const add = (id, config) => {
  configById[id] = config;
  return id;
};
const split = (first, second, direction = "row", splitPercentage = 50) => ({
  first,
  second,
  direction,
  splitPercentage,
});
const equalSplit = (items, direction) => {
  if (items.length === 1) {
    return items[0];
  }
  const midpoint = Math.ceil(items.length / 2);
  return split(
    equalSplit(items.slice(0, midpoint), direction),
    equalSplit(items.slice(midpoint), direction),
    direction,
    (midpoint / items.length) * 100,
  );
};
const grid = (ids, columns) => {
  const rows = [];
  for (let i = 0; i < ids.length; i += columns) {
    rows.push(equalSplit(ids.slice(i, i + columns), "row"));
  }
  return equalSplit(rows, "column");
};
const raw = (id, title, topicPath, fontSize = 14) =>
  add(`RawMessages!${id}`, {
    diffEnabled: false,
    diffMethod: "custom",
    diffTopicPath: "",
    showFullMessageForDiff: false,
    topicPath,
    fontSize,
    foxglovePanelTitle: title,
  });
const indicator = (
  id,
  title,
  topic,
  fallbackLabel,
  fallbackColor = "#667085",
  values = [],
) =>
  add(`Indicator!${id}`, {
    path: `${topic}.data`,
    style: "background",
    fontSize: 20,
    fallbackColor,
    fallbackLabel,
    rules: values.map((value) => {
      const rule = typeof value === "object" ? value : { rawValue: value, label: value };
      return {
        operator: "=",
        rawValue: String(rule.rawValue),
        color: rule.color ?? fallbackColor,
        label: String(rule.label ?? rule.rawValue),
      };
    }),
    foxglovePanelTitle: title,
  });
const image = (id, title, imageTopic, calibrationTopic = "") =>
  add(`Image!${id}`, {
    cameraState: {
      distance: 20,
      perspective: true,
      phi: 60,
      target: [0, 0, 0],
      targetOffset: [0, 0, 0],
      targetOrientation: [0, 0, 0, 1],
      thetaOffset: 45,
      fovy: 45,
      near: 0.5,
      far: 5000,
    },
    followMode: "follow-pose",
    scene: {},
    transforms: {},
    topics: {},
    layers: {},
    publish: {
      type: "point",
      poseTopic: "/move_base_simple/goal",
      pointTopic: "/clicked_point",
      poseEstimateTopic: "/initialpose",
      poseEstimateXDeviation: 0.5,
      poseEstimateYDeviation: 0.5,
      poseEstimateThetaDeviation: 0.26179939,
    },
    imageMode: {
      imageTopic,
      ...(calibrationTopic ? { calibrationTopic } : {}),
      synchronize: false,
      rotation: 0,
    },
    foxglovePanelTitle: title,
  });
const publish = (id, title, buttonText, topicName, datatype, value, advancedView = false) =>
  add(`Publish!${id}`, {
    buttonText,
    buttonTooltip: "",
    advancedView,
    value,
    foxglovePanelTitle: title,
    topicName,
    datatype,
  });
const tabs = (id, entries) =>
  add(`Tab!${id}`, {
    activeTabIdx: 0,
    tabs: entries.map(([title, layout]) => ({ title, layout })),
  });

const map = structuredClone(reference.configById["3D!1juc55f"]);
map.foxglovePanelTitle = "导航地图与实时路径";
map.followTf = "map";
map.topics = {
  "/map": { visible: true },
  "/scan": {
    visible: true,
    colorField: "range",
    colorMode: "gradient",
    colorMap: "turbo",
    pointSize: 4,
    decayTime: 0.5,
    pointShape: "square",
  },
  "/plan": { visible: true, lineWidth: 0.2, type: "line", color: "#22c55e" },
  "/local_plan": { visible: true, lineWidth: 0.16, type: "line", color: "#f59e0b" },
  "/amcl_pose": { visible: true },
  "/robot_description": { visible: true, displayMode: "auto", showOutlines: false, showAxis: false },
};
const mapPanel = add("3D!competitionMap", map);

const question = raw("question", "当前任务题目", "/competition/question", 16);
const mapping = raw("mapping", "区域映射规则", "/competition/mapping_rule", 16);
const planText = raw("plan", "抓取与放置顺序", "/competition/task_plan_text", 15);
const events = raw("events", "任务事件", "/competition/event", 13);
const errors = raw("errors", "最近错误", "/competition/last_error", 13);

const statusPanels = [
  indicator("redTotal", "红色总数", "/competition/red_total", "--", "#b42318", [0, 1, 2, 3, 4, 5]),
  indicator("blueTotal", "蓝色总数", "/competition/blue_total", "--", "#175cd3", [0, 1, 2, 3, 4, 5]),
  indicator("redDone", "红色已完成", "/competition/red_completed", "0", "#d92d20", [0, 1, 2, 3, 4, 5]),
  indicator("blueDone", "蓝色已完成", "/competition/blue_completed", "0", "#2970ff", [0, 1, 2, 3, 4, 5]),
  indicator("item", "当前方块", "/competition/current_item_index", "--", "#7f56d9", [1, 2, 3, 4, 5]),
  indicator("done", "总完成数", "/competition/completed_item_count", "0", "#039855", [0, 1, 2, 3, 4, 5]),
  indicator("color", "目标颜色", "/competition/current_color", "等待", "#475467", [
    { rawValue: "red", label: "红色", color: "#d92d20" },
    { rawValue: "blue", label: "蓝色", color: "#2970ff" },
  ]),
  indicator("zone", "目标区域", "/competition/current_zone", "等待", "#475467", ["A", "B", "C"]),
  indicator("state", "总状态", "/competition/current_state", "空闲", "#344054", [
    { rawValue: "BOOTING", label: "启动中" },
    { rawValue: "RESETTING", label: "复位中" },
    { rawValue: "READY", label: "就绪" },
    { rawValue: "READING_QUESTION", label: "识别资源" },
    { rawValue: "NAVIGATING_TO_OBJECT", label: "前往资源点" },
    { rawValue: "PRE_GRASP", label: "预抓取" },
    { rawValue: "GRASPING", label: "抓取" },
    { rawValue: "NAVIGATING_TO_ZONE", label: "前往放置区" },
    { rawValue: "PLACING", label: "放置" },
    { rawValue: "VERIFYING", label: "校验中" },
    { rawValue: "ITEM_SUCCEEDED", label: "单块完成" },
    { rawValue: "ITEM_FAILED", label: "单块失败" },
    { rawValue: "TASK_SUCCEEDED", label: "任务完成", color: "#039855" },
    { rawValue: "TASK_FAILED", label: "任务失败", color: "#d92d20" },
    { rawValue: "API_ERROR", label: "解析失败", color: "#d92d20" },
    { rawValue: "TIME_LIMIT", label: "时间到", color: "#d92d20" },
    { rawValue: "CANCELLED", label: "已取消" },
  ]),
  indicator("nav", "导航状态", "/competition/navigation_state", "空闲", "#026aa2", [
    { rawValue: "IDLE", label: "空闲" },
    { rawValue: "NAVIGATING_TO_OBJECT", label: "前往资源点" },
    { rawValue: "NAVIGATING_TO_ZONE", label: "前往放置区" },
    { rawValue: "CANCELLED", label: "已取消" },
  ]),
  indicator("arm", "机械臂状态", "/competition/manipulation_state", "空闲", "#6941c6", [
    { rawValue: "IDLE", label: "空闲" },
    { rawValue: "PRE_GRASP", label: "预抓取" },
    { rawValue: "MOVEIT_TO_OPENCV_TARGET", label: "视觉微调" },
    { rawValue: "SUCTION_ON", label: "吸附" },
    { rawValue: "PRE_RELEASE", label: "预放置" },
    { rawValue: "SUCTION_OFF", label: "释放" },
    { rawValue: "CANCELLED", label: "已取消" },
  ]),
  indicator("api", "解析状态", "/competition/api_status", "等待", "#b54708", [
    { rawValue: "BOOTING", label: "启动中" },
    { rawValue: "READY", label: "就绪" },
    { rawValue: "READING_QUESTION", label: "读取题目" },
    { rawValue: "WAITING_MAPPING", label: "等待映射" },
    { rawValue: "CALLING_LLM", label: "模型解析" },
    { rawValue: "VALIDATING_TASK", label: "校验任务" },
    { rawValue: "API_ERROR", label: "解析失败", color: "#d92d20" },
    { rawValue: "CANCELLED", label: "已取消" },
    { rawValue: "VISUAL_FIXTURE", label: "验收夹具" },
  ]),
];

const detection = image("detection", "OpenCV目标检测", "/competition/detection_image");
const rgb = image(
  "rgb",
  "RGB画面",
  "/camera_rgbd/camera_rgbd/image_raw",
  "/camera_rgbd/camera_rgbd/camera_info",
);
const depth = image(
  "depth",
  "Depth画面",
  "/camera_rgbd/camera_rgbd/depth/image_raw",
  "/camera_rgbd/camera_rgbd/depth/camera_info",
);
const visionTabs = tabs("vision", [
  ["目标检测", detection],
  ["RGB", rgb],
  ["Depth", depth],
]);

const mappingInput = publish(
  "mappingInput",
  "映射输入",
  "发布映射",
  "/competition/mapping_rule_input",
  "std_msgs/String",
  '{"data":""}',
  true,
);
const manualQuestion = publish(
  "manualQuestion",
  "题目输入",
  "发布题目",
  "/competition/manual_question",
  "std_msgs/String",
  '{"data":""}',
  true,
);
const start = publish("start", "开始任务", "开始", "/competition/start", "std_msgs/Bool", '{"data":true}');
const resubmit = publish(
  "resubmit",
  "重新解析",
  "重新解析",
  "/competition/resubmit",
  "std_msgs/Bool",
  '{"data":true}',
);
const cancel = publish("cancel", "取消任务", "取消", "/competition/cancel", "std_msgs/Bool", '{"data":true}');
const reset = publish("reset", "复位任务", "复位", "/competition/reset", "std_msgs/Bool", '{"data":true}');

const taskInfo = split(question, split(mapping, planText, "column", 45), "column", 32);
const statusGrid = grid(statusPanels, 3);
const controls = grid([mappingInput, manualQuestion, start, resubmit, cancel, reset], 3);
const diagnostics = tabs("diagnostics", [
  ["任务事件", events],
  ["错误信息", errors],
  ["任务JSON", raw("planJson", "任务JSON", "/competition/task_plan_json", 12)],
]);
const left = split(mapPanel, visionTabs, "column", 72);
const rightTop = split(taskInfo, statusGrid, "column", 43);
const rightBottom = split(diagnostics, controls, "column", 58);
const layout = split(left, split(rightTop, rightBottom, "column", 75), "row", 58);

const output = {
  configById,
  globalVariables: {},
  userNodes: {},
  playbackConfig: { speed: 1 },
  layout,
};

writeFileSync(outputPath, `${JSON.stringify(output, null, 2)}\n`, "utf8");
console.log(outputPath);
