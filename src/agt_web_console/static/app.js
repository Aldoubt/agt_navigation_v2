const urlToken = new URLSearchParams(window.location.search).get("token");
if (urlToken) localStorage.setItem("agt_console_token", urlToken);
const state = { overview: {}, runtime: {}, mappingMap: {}, mappingPointcloud: {}, chassis: {}, mappingSession: {}, maps: [], navigationMapVersion: "", experiments: [], bags: [], selectedBagId: "", lastRelocalization: null };
const previewViews = {
  map: { initialized: false, centerOnRobot: false, centerX: 0, centerY: 0, zoom: 1, panX: 0, panY: 0 },
  pointcloud: { initialized: false, centerOnRobot: false, centerX: 0, centerY: 0, zoom: 1, panX: 0, panY: 0, rotationDeg: 0, rotation: 0, projection: "xy" },
};
const previewSize = { width: 900, height: 480 };

function consoleToken() { return localStorage.getItem("agt_console_token") || ""; }

const api = async (path, options = {}) => {
  const headers = { "Content-Type": "application/json", ...(consoleToken() ? { "X-AGT-Token": consoleToken() } : {}), ...(options.headers || {}) };
  const response = await fetch(path, {
    ...options,
    headers,
  });
  if (!response.ok) {
    const body = await response.text();
    let detail = body;
    try { detail = JSON.parse(body).detail || body; } catch (_) { /* keep plain-text response */ }
    throw new Error(detail);
  }
  return response.json();
};

const escapeHtml = (value) => String(value ?? "").replace(/[&<>"']/g, (char) => ({
  "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#039;",
}[char]));

const showToast = (message, error = false) => {
  const toast = document.querySelector("#toast");
  toast.textContent = message;
  toast.className = `toast visible ${error ? "error" : ""}`;
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => { toast.className = "toast"; }, 4500);
};

const modeLabels = {
  IDLE: "空闲", SENSOR_ONLY: "仅传感器", MAPPING: "建图", NAVIGATION: "导航",
  LOCALIZATION_DEBUG: "定位调试", ERROR: "错误",
};
const stateLabels = { UNKNOWN: "未知", OK: "正常", WARN: "警告", ERROR: "错误" };
const localizationStateLabels = {
  0: "未初始化", 1: "搜索中", 2: "验证中", 3: "跟踪中", 4: "降级", 5: "恢复中", 6: "丢失", 7: "错误",
};
const mapStateLabels = { READY: "就绪", ARCHIVED: "已归档", IMPORTED: "已导入", INVALID: "无效" };
const experimentStateLabels = { CREATED: "已创建", RUNNING: "运行中", COMPLETED: "已完成", INVALID: "无效", FINALIZED: "已结束", FAILED: "失败" };

function currentMode() {
  return state.overview.task_readiness?.active_mode || state.overview.mode?.active_mode || "UNKNOWN";
}

function currentRuntime() {
  return Object.keys(state.runtime || {}).length ? state.runtime : (state.overview.runtime || {});
}

function metric(label, value, className = "") {
  return `<div class="metric ${className}"><span>${escapeHtml(label)}</span><strong>${escapeHtml(value)}</strong></div>`;
}

function renderSummary() {
  const health = state.overview.health || {};
  const readiness = state.overview.task_readiness || {};
  const localization = state.overview.localization || {};
  const mode = currentMode();
  document.querySelector("#top-mode").textContent = `模式：${modeLabels[mode] || mode}`;
  document.querySelector("#overview-cards").innerHTML = [
    ["运行模式", modeLabels[mode] || mode],
    ["系统健康", stateLabels[health.overall_state] || "未知"],
    ["任务门禁", readiness.ready ? "允许执行" : "禁止执行"],
    ["定位状态", localizationStateLabels[localization.state] || readiness.localization_state || "未知"],
  ].map(([label, value]) => `<article><h3>${label}</h3><strong>${escapeHtml(value)}</strong></article>`).join("");
}

function renderRuntime() {
  const runtime = currentRuntime();
  const backend = runtime.backend || "ros";
  const selector = document.querySelector("#runtime-backend");
  if (selector && [...selector.options].some((option) => option.value === backend)) selector.value = backend;
  const backendBadge = document.querySelector("#top-backend");
  backendBadge.textContent = `后端：${runtime.offline ? "离线测试" : "ROS 2 真实"}`;
  backendBadge.className = `status ${runtime.offline ? "warning" : "ok"}`;
  document.querySelector("#runtime-description").textContent = runtime.description || "未读取运行后端";
  document.querySelector("#runtime-warning").textContent = runtime.offline ? "当前为离线测试模式：指定 bag 只驱动网页模拟回放和模拟地图预览，不读取 ROS 消息、不写真实 PGM/YAML/PCD，也不会连接传感器、车辆、CAN、安全链或发送任务。" : "当前为 ROS 2 真实模式：按钮会调用真实项目接口，必须先启动系统管理器和对应节点。指定 bag 将在受限运行目录内真实回放。";
  document.querySelector("#runtime-warning").className = `runtime-warning ${runtime.offline ? "visible" : ""}`;
}

function renderOverview() {
  const health = state.overview.health || {};
  const readiness = state.overview.task_readiness || {};
  const localization = state.overview.localization || {};
  document.querySelector("#overview-detail").innerHTML = `
    <div><h3>系统健康</h3><pre>${escapeHtml(JSON.stringify(health, null, 2))}</pre></div>
    <div><h3>任务门禁</h3><pre>${escapeHtml(JSON.stringify(readiness, null, 2))}</pre></div>
    <div><h3>定位摘要</h3><pre>${escapeHtml(JSON.stringify(localization, null, 2))}</pre></div>
    <div><h3>当前地图</h3><pre>${escapeHtml(JSON.stringify({ map_id: readiness.map_id || "", map_version_id: readiness.map_version_id || "" }, null, 2))}</pre></div>`;
  document.querySelector("#overview-updated").textContent = `健康 revision：${health.revision ?? "-"}`;
}

function mappingComponent(health, id) {
  return (health.components || []).find((item) => item.component_id === id) || {};
}

function mappingStatus(health, mode) {
  if (mode !== "MAPPING") return { state: "IDLE", label: "未启动", message: "当前没有运行建图链。" };
  const odometry = mappingComponent(health, "fast_livo_odometry");
  const cloud = mappingComponent(health, "registered_cloud");
  const occupancy = mappingComponent(health, "mapping_occupancy");
  if ([odometry, cloud, occupancy].some((item) => item.state === "ERROR")) {
    const failed = [odometry, cloud, occupancy].find((item) => item.state === "ERROR");
    return { state: "ERROR", label: "建图链异常", message: `${failed.display_name || "建图组件"}：${failed.detail || "健康检查失败"}` };
  }
  if (odometry.state !== "OK") return { state: "WAITING", label: "等待里程计", message: "FAST-LIVO2 还没有发布有效的 /agt/mapping/odometry。" };
  if (cloud.state !== "OK") return { state: "WAITING", label: "等待注册点云", message: "FAST-LIVO2 里程计已到达，正在等待 /agt/mapping/registered_points_lidar。" };
  if (occupancy.state !== "OK") return { state: "WAITING", label: "等待二维地图", message: "点云链已到达，正在等待 /agt/map/mapping_occupancy。" };
  return { state: "MAPPING", label: "建图中", message: "FAST-LIVO2、注册点云和二维栅格地图均在持续更新。" };
}

function renderMappingObservability() {
  const health = state.overview.health || {};
  const status = mappingStatus(health, currentMode());
  const badge = document.querySelector("#mapping-state-badge");
  const className = status.state === "MAPPING" ? "ok" : status.state === "ERROR" ? "error" : status.state === "WAITING" ? "warning" : "unknown";
  badge.textContent = status.label;
  badge.className = `status ${className}`;
  document.querySelector("#mapping-status-message").textContent = status.message;
  const values = [
    ["FAST-LIVO2 里程计", mappingComponent(health, "fast_livo_odometry")],
    ["注册点云", mappingComponent(health, "registered_cloud")],
    ["二维栅格地图", mappingComponent(health, "mapping_occupancy")],
  ];
  document.querySelector("#mapping-metrics").innerHTML = values.map(([label, item]) => {
    const stateText = stateLabels[item.state] || "未知";
    const rate = item.observed_rate_hz == null ? "-" : `${Number(item.observed_rate_hz).toFixed(1)} Hz`;
    const age = item.message_age_sec == null ? "-" : `${Number(item.message_age_sec).toFixed(2)} s`;
    return `<div class="mapping-metric"><strong>${escapeHtml(label)}</strong><span class="status ${item.state === "OK" ? "ok" : item.state === "ERROR" ? "error" : item.state === "WARN" ? "warning" : "unknown"}">${stateText}</span><small>频率 ${rate}；最近消息 ${age}</small></div>`;
  }).join("");
}

function previewPose(data) {
  const pose = data?.robot_pose || {};
  return pose.available && Number.isFinite(Number(pose.x)) && Number.isFinite(Number(pose.y)) ? pose : null;
}

function previewTransform(key, bounds, pose, focus) {
  const view = previewViews[key];
  const center = focus || pose;
  const spanX = Math.max(bounds.maxX - bounds.minX, 0.01);
  const spanY = Math.max(bounds.maxY - bounds.minY, 0.01);
  const fitScale = Math.min((previewSize.width - 36) / spanX, (previewSize.height - 36) / spanY);
  if (!view.initialized) {
    view.centerX = (bounds.minX + bounds.maxX) / 2;
    view.centerY = (bounds.minY + bounds.maxY) / 2;
    view.initialized = true;
  }
  if (view.centerOnRobot) {
    view.centerX = center ? Number(center.x) : (bounds.minX + bounds.maxX) / 2;
    view.centerY = center ? Number(center.y) : (bounds.minY + bounds.maxY) / 2;
    view.panX = 0;
    view.panY = 0;
    view.centerOnRobot = false;
  }
  const scale = fitScale * view.zoom;
  const rotation = Number(view.rotation || 0);
  const cosine = Math.cos(rotation);
  const sine = Math.sin(rotation);
  return {
    scale,
    toScreen(x, y) {
      const deltaX = Number(x) - view.centerX;
      const deltaY = Number(y) - view.centerY;
      const rotatedX = cosine * deltaX - sine * deltaY;
      const rotatedY = sine * deltaX + cosine * deltaY;
      return {
        x: previewSize.width / 2 + rotatedX * scale + view.panX,
        y: previewSize.height / 2 - rotatedY * scale + view.panY,
      };
    },
    screenYaw(yaw) { return -(Number(yaw || 0) + rotation); },
    screenDirection(x, y) {
      return { x: cosine * Number(x) - sine * Number(y), y: sine * Number(x) + cosine * Number(y) };
    },
  };
}

function drawPreviewRobot(context, pose, transform) {
  if (!pose) return;
  const point = transform.toScreen(pose.x, pose.y);
  context.save();
  context.translate(point.x, point.y);
  context.rotate(transform.screenYaw(pose.yaw));
  context.fillStyle = "#f0c84b";
  context.strokeStyle = "#1b2228";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(12, 0);
  context.lineTo(-8, -7);
  context.lineTo(-5, 7);
  context.closePath();
  context.fill();
  context.stroke();
  context.restore();
}

function renderMappingMap() {
  const canvas = document.querySelector("#mapping-map-canvas");
  const empty = document.querySelector("#mapping-map-empty");
  const map = state.mappingMap || {};
  if (!map.available || !map.width || !map.height || !Array.isArray(map.data)) {
    previewViews.map.initialized = false;
    canvas.hidden = true;
    empty.hidden = false;
    empty.textContent = map.message || "尚未收到二维建图地图";
    return;
  }
  const width = previewSize.width;
  const height = previewSize.height;
  const resolution = Number(map.resolution || 0.1);
  const origin = map.origin || { x: 0, y: 0 };
  const bounds = { minX: Number(origin.x), minY: Number(origin.y), maxX: Number(origin.x) + Number(map.width) * resolution, maxY: Number(origin.y) + Number(map.height) * resolution };
  const pose = previewPose(map);
  const transform = previewTransform("map", bounds, pose);
  const context = canvas.getContext("2d");
  canvas.width = width; canvas.height = height;
  context.fillStyle = "#36434b"; context.fillRect(0, 0, width, height);
  const cellWidth = Math.max(1, transform.scale * resolution * 1.02);
  const cellHeight = cellWidth;
  map.data.forEach((value, index) => {
    const sourceX = index % Number(map.width);
    const sourceY = Math.floor(index / Number(map.width));
    const point = transform.toScreen(Number(origin.x) + (sourceX + 0.5) * resolution, Number(origin.y) + (sourceY + 0.5) * resolution);
    if (value < 0) context.fillStyle = "rgba(38, 49, 57, .65)";
    else if (Number(value) >= 65) context.fillStyle = "#d65757";
    else context.fillStyle = `rgb(${Math.max(35, 245 - Math.round(Number(value) * 1.8))}, ${Math.max(35, 245 - Math.round(Number(value) * 1.8))}, ${Math.max(35, 245 - Math.round(Number(value) * 1.8))})`;
    context.fillRect(point.x - cellWidth / 2, point.y - cellHeight / 2, cellWidth, cellHeight);
  });
  drawPreviewRobot(context, pose, transform);
  canvas.hidden = false; empty.hidden = true;
  document.querySelector("#mapping-map-meta").textContent = `${map.width} × ${map.height} 栅格；分辨率 ${resolution.toFixed(3)} m；${pose ? "已显示机器人位置" : "等待机器人位姿"}${map.simulated ? "；离线模拟预览，不可导出" : ""}`;
}

function coordinateStep(span) {
  const rough = Math.max(Number(span) / 8, 0.01);
  const power = 10 ** Math.floor(Math.log10(rough));
  const normalized = rough / power;
  return (normalized <= 1 ? 1 : normalized <= 2 ? 2 : normalized <= 5 ? 5 : 10) * power;
}

function coordinateLabel(value, step) {
  const decimals = Math.max(0, Math.min(3, Math.ceil(-Math.log10(step))));
  return Number(value).toFixed(decimals);
}

function drawPointcloudCoordinates(context, bounds, transform, frameId, horizontalAxis = "X", verticalAxis = "Y") {
  const step = coordinateStep(Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY));
  const startX = Math.floor(bounds.minX / step) * step;
  const startY = Math.floor(bounds.minY / step) * step;
  context.save();
  context.lineWidth = 1;
  context.strokeStyle = "rgba(151, 180, 194, .18)";
  context.fillStyle = "rgba(198, 219, 227, .72)";
  context.font = "11px system-ui, sans-serif";
  context.textBaseline = "top";
  for (let x = startX; x <= bounds.maxX + step * 0.01; x += step) {
    const from = transform.toScreen(x, bounds.minY);
    const to = transform.toScreen(x, bounds.maxY);
    context.beginPath(); context.moveTo(from.x, from.y); context.lineTo(to.x, to.y); context.stroke();
    context.fillText(`${horizontalAxis} ${coordinateLabel(x, step)}`, from.x + 4, from.y + 4);
  }
  for (let y = startY; y <= bounds.maxY + step * 0.01; y += step) {
    const from = transform.toScreen(bounds.minX, y);
    const to = transform.toScreen(bounds.maxX, y);
    context.beginPath(); context.moveTo(from.x, from.y); context.lineTo(to.x, to.y); context.stroke();
    context.fillText(`${verticalAxis} ${coordinateLabel(y, step)}`, from.x + 4, from.y + 4);
  }
  if (bounds.minY <= 0 && bounds.maxY >= 0) {
    const from = transform.toScreen(bounds.minX, 0);
    const to = transform.toScreen(bounds.maxX, 0);
    context.strokeStyle = "rgba(241, 196, 68, .9)";
    context.lineWidth = 1.5;
    context.beginPath(); context.moveTo(from.x, from.y); context.lineTo(to.x, to.y); context.stroke();
  }
  if (bounds.minX <= 0 && bounds.maxX >= 0) {
    const from = transform.toScreen(0, bounds.minY);
    const to = transform.toScreen(0, bounds.maxY);
    context.strokeStyle = "rgba(82, 194, 215, .9)";
    context.lineWidth = 1.5;
    context.beginPath(); context.moveTo(from.x, from.y); context.lineTo(to.x, to.y); context.stroke();
  }
  if (bounds.minX <= 0 && bounds.maxX >= 0 && bounds.minY <= 0 && bounds.maxY >= 0) {
    const origin = transform.toScreen(0, 0);
    context.fillStyle = "#ffffff";
    context.beginPath(); context.arc(origin.x, origin.y, 3, 0, Math.PI * 2); context.fill();
    context.fillText("O", origin.x + 6, origin.y + 4);
  }

  const widgetX = previewSize.width - 128;
  const widgetY = 14;
  context.fillStyle = "rgba(13, 21, 27, .88)";
  context.fillRect(widgetX, widgetY, 114, 62);
  context.fillStyle = "#d9e5eb";
  context.fillText(`坐标系 ${String(frameId || "-")}`, widgetX + 8, widgetY + 7);
  const widgetOrigin = { x: widgetX + 24, y: widgetY + 39 };
  const drawDirection = (direction, color, label) => {
    const length = 20;
    const end = { x: widgetOrigin.x + direction.x * length, y: widgetOrigin.y - direction.y * length };
    context.strokeStyle = color;
    context.fillStyle = color;
    context.lineWidth = 2;
    context.beginPath(); context.moveTo(widgetOrigin.x, widgetOrigin.y); context.lineTo(end.x, end.y); context.stroke();
    context.beginPath(); context.arc(end.x, end.y, 2.5, 0, Math.PI * 2); context.fill();
    context.fillText(label, end.x + 4, end.y - 6);
  };
  drawDirection(transform.screenDirection(1, 0), "#f1c444", horizontalAxis);
  drawDirection(transform.screenDirection(0, 1), "#52c2d7", verticalAxis);
  context.fillStyle = "rgba(198, 219, 227, .72)";
  context.fillText(`网格 ${coordinateLabel(step, step)} m`, widgetX + 48, widgetY + 40);
  context.restore();
}

function renderMappingPointcloud() {
  const canvas = document.querySelector("#mapping-pointcloud-canvas");
  const empty = document.querySelector("#mapping-pointcloud-empty");
  const meta = document.querySelector("#mapping-pointcloud-meta");
  const cloud = state.mappingPointcloud || {};
  const points = Array.isArray(cloud.points) ? cloud.points : [];
  const view = previewViews.pointcloud;
  const projections = {
    xy: { label: "X-Y 俯视", horizontal: "X", vertical: "Y", project: (point) => [Number(point[0]), Number(point[1])], focus: (pose) => pose && { x: Number(pose.x), y: Number(pose.y) } },
    xz: { label: "X-Z 侧视", horizontal: "X", vertical: "Z", project: (point) => [Number(point[0]), Number(point[2])], focus: (pose) => pose && { x: Number(pose.x), y: 0 } },
    yz: { label: "Y-Z 侧视", horizontal: "Y", vertical: "Z", project: (point) => [Number(point[1]), Number(point[2])], focus: (pose) => pose && { x: Number(pose.y), y: 0 } },
  };
  const projection = projections[view.projection] || projections.xy;
  document.querySelectorAll("[data-pointcloud-view]").forEach((button) => {
    const selected = button.dataset.pointcloudView === (view.projection || "xy");
    button.classList.toggle("active", selected);
    button.setAttribute("aria-pressed", selected ? "true" : "false");
  });
  if (!cloud.available || !points.length) {
    previewViews.pointcloud.initialized = false;
    canvas.hidden = true;
    empty.hidden = false;
    const activeMode = String(cloud.active_mode || currentMode()).toUpperCase();
    empty.textContent = activeMode === "MAPPING"
      ? "建图链已启动，等待注册点云；请确认 bag 正在回放原始输入。"
      : cloud.message || "尚未收到注册点云";
    meta.textContent = activeMode === "MAPPING" ? "建图链运行中，等待点云消息" : "等待点云数据";
    return;
  }
  const bounds = { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity };
  for (const point of points) {
    const projected = projection.project(point);
    if (!projected.every(Number.isFinite)) continue;
    bounds.minX = Math.min(bounds.minX, projected[0]);
    bounds.maxX = Math.max(bounds.maxX, projected[0]);
    bounds.minY = Math.min(bounds.minY, projected[1]);
    bounds.maxY = Math.max(bounds.maxY, projected[1]);
  }
  const pose = previewPose(cloud);
  const focus = projection.focus(pose);
  view.rotation = Number(view.rotationDeg || 0) * Math.PI / 180;
  const transform = previewTransform("pointcloud", bounds, pose, focus);
  const context = canvas.getContext("2d");
  canvas.width = previewSize.width; canvas.height = previewSize.height;
  context.fillStyle = "#0d151b"; context.fillRect(0, 0, previewSize.width, previewSize.height);
  drawPointcloudCoordinates(context, bounds, transform, cloud.frame_id, projection.horizontal, projection.vertical);
  for (const point of points) {
    const projected = projection.project(point);
    if (!projected.every(Number.isFinite)) continue;
    const screenPoint = transform.toScreen(projected[0], projected[1]);
    const z = Number(point[2]);
    const shade = Math.max(90, Math.min(235, Math.round(155 + Number(z) * 18)));
    context.fillStyle = `rgb(${Math.round(shade * 0.55)}, ${shade}, ${Math.min(255, shade + 25)})`;
    context.fillRect(screenPoint.x, screenPoint.y, 2, 2);
  }
  if (view.projection === "xy") drawPreviewRobot(context, pose, transform);
  canvas.hidden = false; empty.hidden = true;
  const rotation = Number(view.rotationDeg || 0);
  const gridStep = coordinateStep(Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY));
  meta.textContent = `${points.length} 个体素；${projection.label}；坐标系 ${cloud.frame_id || "-"}；网格 ${coordinateLabel(gridStep, gridStep)} m；视角 ${rotation.toFixed(0)}°；${view.projection === "xy" && pose ? "已显示机器人位置" : view.projection === "xy" ? "等待机器人位姿" : "垂直结构视图"}${cloud.simulated ? "；离线模拟预览，不可导出" : ""}`;
}

function renderChassis() {
  const chassis = state.chassis || {};
  const component = mappingComponent(state.overview.health || {}, "chassis");
  const can = chassis.can || {};
  const connected = chassis.connected === true;
  const interfaceReady = can.present && ["up", "unknown"].includes(String(can.operstate).toLowerCase());
  const stateName = connected ? "正常" : chassis.available || can.present ? "待检查" : "未启动";
  const stateClass = connected ? "ok" : stateName === "待检查" ? "warning" : "unknown";
  const badge = document.querySelector("#chassis-state-badge");
  badge.textContent = stateName; badge.className = `status ${stateClass}`;
  document.querySelector("#chassis-status-message").textContent = connected
    ? "BUNKER 状态桥接正在接收 CAN 反馈，当前网页不发送底盘命令。"
    : can.present
      ? `CAN ${can.interface} 当前为 ${can.operstate}，等待 BUNKER 状态帧。`
      : "未发现 CAN 接口或只读底盘监测尚未启动。";
  const diagnostic = (chassis.diagnostics || [])[0] || {};
  const battery = chassis.battery_voltage == null ? "-" : `${Number(chassis.battery_voltage).toFixed(2)} V`;
  const componentText = component.state ? `${stateLabels[component.state] || component.state}；${component.detail || ""}` : "未纳入当前模式门禁";
  document.querySelector("#chassis-metrics").innerHTML = [
    ["CAN 接口", `${can.interface || "-"}：${can.operstate || "未知"}`],
    ["CAN 设备", can.present ? "已发现" : "未发现"],
    ["BUNKER 连接", connected ? "已连接" : "未连接"],
    ["状态诊断", diagnostic.message || "暂无状态帧"],
    ["电池电压", battery],
    ["健康合同", componentText],
  ].map(([label, value]) => metric(label, value, interfaceReady ? "" : "warning")).join("");
  const interfaceName = document.querySelector("#can-interface")?.value.trim() || "can0";
  const command = [
    "# 终端 1：管理员执行 CAN 初始化（首次安装 can-utils 可取消最后一行注释）",
    "sudo modprobe gs_usb",
    `sudo ip link set ${interfaceName} up type can bitrate 500000`,
    `ip -details link show ${interfaceName}`,
    "# sudo apt-get install -y can-utils",
    "",
    "# 终端 2：普通用户启动只读 BUNKER 状态桥接",
    "source /opt/ros/humble/setup.bash",
    "export AGT_WS=/absolute/path/to/agt_navigation_v2",
    'source "$AGT_WS/install/setup.bash"',
    `ros2 launch agt_chassis bunker.launch.py can_interface:=${interfaceName} operation_mode:=monitor start_safety:=false command_topic:=/agt/chassis/monitor_cmd_vel`,
    "",
    `# 可选终端 3：只读观察 CAN 帧\ncandump ${interfaceName}`,
  ].join("\n");
  const commandElement = document.querySelector("#chassis-admin-command");
  if (commandElement) commandElement.textContent = command;
}

function renderMappingSession() {
  const session = state.mappingSession || {};
  const stateText = {
    IDLE: "未启动",
    MAPPING: "建图中",
    STARTING: "正在启动建图",
    SAVING_GRID: "正在保存二维地图",
    STOPPING_MAPPING: "正在正常停止建图",
    WAITING_ASSETS: "等待 PCD 与 bag 收口",
    CANDIDATE_READY: "候选地图可编辑",
    COMMITTING: "正在校验并登记候选",
    CAPTURE_FAILED: "采集资产不完整",
    COMMIT_FAILED: "候选校验未通过",
    START_FAILED: "建图启动失败",
    REGISTERED: "已登记地图版本",
    DISCARDED: "本次建图已删除",
    SIMULATED: "离线模拟中",
    SIMULATED_RETAINED: "离线模拟地图已保留（不可导出）",
    SIMULATED_DISCARDED: "离线模拟删除",
    ERROR: "保存失败",
  }[session.state] || "未启动";
  const mappingState = document.querySelector("#mapping-action-state");
  const finish = document.querySelector("#finish-mapping");
  const retain = document.querySelector("#retain-mapping");
  const commit = document.querySelector("#commit-mapping");
  const retainedOffline = session.offline && session.state === "SIMULATED_RETAINED";
  const activeMapping = currentMode() === "MAPPING" || session.state === "MAPPING";
  const candidateReady = !session.offline && session.state === "CANDIDATE_READY";
  const discardableFailedSession = !session.offline && !session.version_id && !activeMapping && ["CAPTURE_FAILED", "COMMIT_FAILED", "START_FAILED"].includes(session.state);
  if (retain) {
    retain.textContent = session.offline ? "保留一个模拟地图" : "完成采集并生成候选";
    retain.hidden = retainedOffline || discardableFailedSession || candidateReady || !activeMapping;
  }
  if (commit) commit.hidden = !candidateReady;
  const discard = document.querySelector("#discard-mapping");
  if (discard) discard.textContent = discardableFailedSession ? "删除残留文件" : "删除本次建图";
  if (mappingState) {
    const assets = session.assets || {};
    const fileText = session.available && !session.offline
      ? `PGM/YAML ${session.pgm_ready ? "已写入" : "等待"}；PCD ${session.pcd_ready ? "已写入" : "等待"}`
      : session.offline && session.offline_map_slot?.occupied ? "模拟地图槽位 1/1" : "";
    mappingState.textContent = `${stateText}${fileText ? `；${fileText}` : ""}${session.version_id ? `；版本 ${session.version_id}` : ""}`;
  }
  if (finish) {
    finish.hidden = currentMode() !== "MAPPING" && !retainedOffline && !discardableFailedSession && !candidateReady;
    finish.textContent = retainedOffline ? "删除当前模拟地图" : candidateReady ? "处理可编辑候选地图" : discardableFailedSession ? "删除未登记建图文件" : "完成建图采集";
  }
  const dialogTitle = document.querySelector("#mapping-finish-title");
  if (dialogTitle) dialogTitle.textContent = retainedOffline ? "删除当前模拟地图槽位" : candidateReady ? "候选地图处理" : discardableFailedSession ? "清理未登记建图结果" : "完成建图采集";
  const detail = document.querySelector("#mapping-finish-detail");
  const nameLabel = document.querySelector("#mapping-finish-map-name-label");
  const nameInput = document.querySelector("#mapping-finish-map-name");
  const confirmationLabel = document.querySelector("#mapping-finish-confirm-label");
  if (nameLabel) nameLabel.hidden = true;
  if (nameInput) nameInput.hidden = true;
  if (confirmationLabel) confirmationLabel.hidden = !activeMapping;
  if (detail && session.available) {
    const assets = session.assets || {};
    detail.textContent = session.offline
      ? "离线模式最多保留一个模拟地图槽位。这里的预览不读取 bag 消息，也不会写入真实 PGM、YAML、PCD 或地图版本；要生成可用于语义撰写、实车导航和重定位的资产，请切换 ROS 2 后端，用历史 bag 输入完成真实建图。"
      : `${activeMapping ? "确认采集完成后，Action 会先保存二维栅格，再正常停止建图以收口 PCD 与 bag。" : candidateReady ? "候选地图允许编辑；登记会重新校验 YAML、PGM 和 PCD 绑定，并生成新的不可变版本。" : ""} 地图 ID：${session.map_id || session.map_name || "-"}；PGM/YAML：${session.pgm_ready ? "已完成" : "待保存"}；PCD：${session.pcd_ready ? "已完成" : "待 FAST-LIVO2 正常退出写入"}；候选：${session.candidate_map_yaml || session.root || "-"}`;
  }
}

function renderNavigationMapSelection() {
  const select = document.querySelector("#navigation-map-version");
  const summary = document.querySelector("#navigation-map-summary");
  if (!select) return;
  const ready = (state.maps || []).filter((item) => item.state === "READY" && !item.deleted && item.assets && item.assets.map);
  let selected = state.navigationMapVersion || select.value;
  if (!ready.some((item) => item.version_id === selected)) {
    selected = (ready.find((item) => Number(item.active) === 1) || ready[0] || {}).version_id || "";
  }
  state.navigationMapVersion = selected;
  select.innerHTML = `<option value="">${ready.length ? "请选择 READY 地图版本" : "暂无 READY 地图版本"}</option>${ready.map((item) => `<option value="${escapeHtml(item.version_id)}">${escapeHtml(item.map_id)} / ${escapeHtml(item.version_id)}${Number(item.active) === 1 ? "（当前激活）" : ""}</option>`).join("")}`;
  select.value = selected;
  const row = ready.find((item) => item.version_id === selected);
  if (summary) summary.value = row ? `${row.map_id}；${Number(row.active) === 1 ? "已激活" : "未激活，请先激活"}` : "未选择";
  const navState = document.querySelector("#navigation-action-state");
  if (navState) navState.textContent = row ? (Number(row.active) === 1 ? `已选择 ${row.version_id}` : "该版本未激活，请到地图管理中激活") : "请先选择地图版本";
}

function renderControlActions() {
  const mode = currentMode();
  const health = state.overview.health || {};
  const sensor = mappingComponent(health, "mid360_pointcloud");
  const processes = state.overview.mode?.processes || [];
  const sensorProcess = processes.some((item) => item.profile === "sensor_only" && item.returncode == null);
  const mappingFromBag = mode === "MAPPING" && document.querySelector("#mapping-input-source")?.value === "bag";
  const sensorActive = sensorProcess || sensor.present === true || sensor.state === "OK" || mode === "NAVIGATION" || (mode === "MAPPING" && !mappingFromBag);
  const sensorButton = document.querySelector("#start-profile");
  if (sensorButton && document.querySelector("#mode-profile")?.value === "sensor_only") {
    sensorButton.disabled = sensorActive || mappingFromBag;
    sensorButton.textContent = mappingFromBag ? "当前建图使用 rosbag" : sensorActive ? "传感器已启动" : "启动传感器";
  }
  const mappingButton = document.querySelector("#start-mapping-profile");
  const navigationButton = document.querySelector("#start-navigation-profile");
  if (mappingButton) {
    mappingButton.disabled = mode === "MAPPING" || mode === "NAVIGATION";
    mappingButton.textContent = mode === "MAPPING" ? "建图进行中" : mode === "NAVIGATION" ? "请先停止导航" : "启动建图链";
  }
  if (navigationButton) {
    const selected = (state.maps || []).find((item) => item.version_id === state.navigationMapVersion);
    navigationButton.disabled = mode === "MAPPING" || mode === "NAVIGATION" || !selected || Number(selected.active) !== 1;
    navigationButton.textContent = mode === "NAVIGATION" ? "导航进行中" : mode === "MAPPING" ? "请先完成建图" : "启动导航链";
  }
  const stopButton = document.querySelector("#stop-mode");
  if (stopButton) stopButton.textContent = mode === "MAPPING" ? "完成建图" : "停止受管理模块";
}

function profileButton(profile) {
  if (profile === "mapping") return document.querySelector("#start-mapping-profile");
  if (profile === "navigation") return document.querySelector("#start-navigation-profile");
  return document.querySelector("#start-profile");
}

function renderWorkflow() {
  const health = state.overview.health || {};
  const readiness = state.overview.task_readiness || {};
  const localization = state.overview.localization || {};
  const mode = currentMode();
  const component = (id) => (health.components || []).find((item) => item.component_id === id) || {};
  const sensor = component("mid360_pointcloud");
  const imu = component("imu");
  const sensorReady = sensor.state === "OK" && imu.state === "OK";
  const mappingFromBag = mode === "MAPPING" && document.querySelector("#mapping-input-source")?.value === "bag";
  const inputHint = document.querySelector("#mapping-input-hint");
  if (inputHint) inputHint.textContent = mappingFromBag || document.querySelector("#mapping-input-source")?.value === "bag"
    ? "历史 bag 模式只启动 FAST-LIVO2 和地图处理链，真实 MID360 不启动；请先启动建图链，再回放 bag。"
    : "实时模式启动 MID360；FAST-LIVO2 建图算法可先启动并等待传感器话题。";
  const localizationReady = localization.state === 3 && localization.pose_valid && localization.localization_accepted && !localization.status_stale;
  const steps = [
    { number: 1, title: "系统管理器", detail: currentRuntime().offline ? "离线模拟后端已连接，无需启动真实 system_manager" : state.overview.health ? "已收到结构化健康状态" : "请先在终端启动 system_manager", status: currentRuntime().offline ? "离线已连接" : state.overview.health ? "已连接" : "等待中", action: null },
    { number: 2, title: "传感器/输入", detail: mappingFromBag ? "当前建图使用历史 rosbag，真实 MID360 未启动" : `MID360：${stateLabels[sensor.state] || "未知"}；IMU：${stateLabels[imu.state] || "未知"}`, status: mappingFromBag ? "bag 输入" : sensorReady ? "正常" : "待检查", action: "sensor_only", actionLabel: sensorReady || mode === "NAVIGATION" || mappingFromBag ? (mappingFromBag ? "使用历史 bag" : "传感器已启动") : "启动传感器", disabled: sensorReady || mode === "NAVIGATION" || mappingFromBag },
    { number: 3, title: "建图或导航链", detail: mode === "MAPPING" ? "当前处于建图模式，请在建图链面板结束并处理结果" : mode === "NAVIGATION" ? "当前处于导航模式，请在导航链面板停止" : "建图链和导航链使用下方独立按钮", status: mode === "MAPPING" || mode === "NAVIGATION" ? "运行中" : "未启动", action: mode === "MAPPING" || mode === "NAVIGATION" ? null : "mapping", actionLabel: "启动建图" },
    { number: 4, title: "地图与定位", detail: `${readiness.map_version_id || "未选择地图"}；${localizationStateLabels[localization.state] || "定位未知"}`, status: readiness.map_version_id && localizationReady ? "可用" : "待准备", action: "localization_rviz", actionLabel: "打开定位 RViz" },
    { number: 5, title: "任务执行", detail: readiness.ready ? "Nav2、TF、安全和定位门禁均满足" : (readiness.blocker_messages || ["等待任务门禁"]).slice(0, 2).join("；"), status: readiness.ready ? "允许" : "禁止", action: null },
    { number: 6, title: "重定位", detail: localization.message || "使用当前地图和注册点云执行有界 Action", status: localization.has_converged ? "已收敛" : "可手动触发", action: "relocalize", actionLabel: "执行一次" },
  ];
  document.querySelector("#workflow-steps").innerHTML = steps.map((step) => `
    <article class="workflow-step ${step.status === "允许" || step.status === "正常" || step.status === "已连接" || step.status === "离线已连接" ? "complete" : ""}">
      <div class="step-number">${step.number}</div><div class="step-content"><h3>${step.title}</h3><p>${escapeHtml(step.detail)}</p><span class="step-status">${escapeHtml(step.status)}</span></div>
      ${step.action ? `<button class="step-action secondary" data-workflow-action="${step.action}" ${step.disabled ? "disabled" : ""}>${step.actionLabel}</button>` : ""}
    </article>`).join("");
}

function renderLocalization() {
  const localization = state.overview.localization || {};
  const values = [
    ["状态", localizationStateLabels[localization.state] || "未知"], ["位姿有效", localization.pose_valid ? "是" : "否"],
    ["质量已接受", localization.localization_accepted ? "是" : "否"], ["后端已收敛", localization.has_converged ? "是" : "否"],
    ["结果有歧义", localization.ambiguous_result ? "是" : "否"], ["状态过期", localization.status_stale ? "是" : "否"],
    ["地图 ID", localization.map_id || "-"], ["地图 hash", localization.map_hash || "-"],
    ["fitness", localization.fitness_score ?? "-"], ["重叠率", localization.overlap_ratio ?? "-"],
    ["内点率", localization.inlier_ratio ?? "-"], ["歧义分数", localization.ambiguity_score ?? "-"],
    ["平移创新", localization.translation_innovation ?? "-"], ["角度创新", localization.yaw_innovation ?? "-"],
    ["耗时 ms", localization.runtime_ms ?? "-"], ["候选进度", `${localization.tested_candidates ?? 0}/${localization.total_candidates ?? 0}`],
    ["候选来源", localization.candidate_source || "-"], ["候选 ID", localization.candidate_id || "-"],
    ["错误信息", localization.message || "-"],
  ];
  document.querySelector("#localization-metrics").innerHTML = values.map(([label, value]) => metric(label, value)).join("");
}

function renderReadiness() {
  const readiness = state.overview.task_readiness || {};
  document.querySelector("#task-readiness").className = `readiness ${readiness.ready ? "ready" : ""}`;
  document.querySelector("#task-readiness").innerHTML = `
    <strong>${readiness.ready ? "允许执行" : "禁止执行"}</strong>
    <p>当前模式：${escapeHtml(modeLabels[readiness.active_mode] || readiness.active_mode || "未知")}</p>
    <div class="blockers">${(readiness.blocker_messages || []).map((item, index) => `<div><b>${escapeHtml(readiness.blocker_codes?.[index] || "BLOCKED")}</b>${escapeHtml(item)}</div>`).join("") || "暂无阻断原因"}</div>`;
}

function renderMaps() {
  document.querySelector("#map-list").innerHTML = state.maps.length ? state.maps.map((map) => `
    <article><span><strong>${escapeHtml(map.map_id)}</strong> <span class="muted">${escapeHtml(map.version_id)}</span><br><small>${escapeHtml(mapStateLabels[map.state] || map.state)} ${map.active ? "· 当前" : ""} ${map.pinned ? "· 固定" : ""}</small></span>
      <span class="button-row"><button data-select-navigation="${escapeHtml(map.version_id)}" ${map.state !== "READY" ? "disabled" : ""}>用于导航</button><button data-validate="${escapeHtml(map.version_id)}">校验</button><button data-activate="${escapeHtml(map.version_id)}">激活</button><button data-map-action="${map.pinned ? "unpin" : "pin"}" data-version-id="${escapeHtml(map.version_id)}">${map.pinned ? "取消固定" : "固定"}</button><button data-map-action="archive" data-version-id="${escapeHtml(map.version_id)}">归档</button></span></article>`).join("") : "<p class='muted'>暂无已注册地图版本。</p>";
}

function renderExperiments() {
  const offline = currentRuntime().offline;
  document.querySelector("#experiment-list").innerHTML = state.experiments.length ? state.experiments.map((experiment) => `
    <article><span><strong>${escapeHtml(experiment.title || experiment.experiment_id)}</strong><br><small>${escapeHtml(experimentStateLabels[experiment.state] || experiment.state)} · ${escapeHtml(experiment.experiment_id)}</small></span>
      <span>${escapeHtml(experiment.result_status || "")} ${experiment.state === "CREATED" ? `<button data-exp-action="start" data-exp-id="${escapeHtml(experiment.experiment_id)}">开始实验</button>` : experiment.state === "RUNNING" ? (offline ? "<span class='muted'>离线模式不录包</span>" : `<button data-exp-action="start_bag" data-exp-id="${escapeHtml(experiment.experiment_id)}">开始录包</button><button data-exp-action="stop_bag" data-exp-id="${escapeHtml(experiment.experiment_id)}">停止录包</button>`) + `<button data-exp-action="finalize" data-exp-id="${escapeHtml(experiment.experiment_id)}">结束实验</button>` : ""}</span></article>`).join("") : "<p class='muted'>暂无实验会话。</p>";
}

function renderBags() {
  const bags = state.bags || {};
  const playback = bags.playback || {};
  const offline = currentRuntime().offline;
  const availableBags = bags.bags || [];
  const preferredMappingBag = availableBags.find((bag) => bag.mapping_input_ready && /^mapping_/.test(bag.bag_id));
  if (!availableBags.some((bag) => bag.bag_id === state.selectedBagId)) {
    state.selectedBagId = preferredMappingBag?.bag_id || availableBags.find((bag) => bag.mapping_input_ready)?.bag_id || availableBags[0]?.bag_id || "";
  }
  const selector = document.querySelector("#bag-selection");
  if (selector) {
    selector.innerHTML = `<option value="">${availableBags.length ? "请选择 bag" : "暂无完整 bag"}</option>${availableBags.map((bag) => `<option value="${escapeHtml(bag.bag_id)}">${escapeHtml(bag.bag_id)}（${Number(bag.message_count || 0).toLocaleString()} 条）</option>`).join("")}`;
    selector.value = state.selectedBagId;
  }
  const playButton = document.querySelector("#play-selected-bag");
  if (playButton) {
    playButton.disabled = !state.selectedBagId || playback.playing;
    playButton.textContent = offline ? "离线模拟回放" : currentMode() === "MAPPING" ? "回放建图输入" : "回放指定 bag";
  }
  const hint = document.querySelector("#bag-replay-hint");
  if (hint) hint.textContent = offline
    ? "离线模拟回放会生成带有‘模拟’标记的二维/点云预览，但不读取 bag 中的 ROS topic，也不能导出真实地图；需要真实节点测试请切换 ROS 2 后端。"
    : currentMode() === "MAPPING"
      ? "当前为建图模式：只回放 /clock、/tf_static、MID360 CustomMsg 和 IMU，自动排除 bag 中的 FAST-LIVO2 里程计、注册点云、二维地图和旧 TF。"
      : "ROS 2 后端会在 runtime/rosbag 受限目录内执行 ros2 bag play --clock。被测节点应使用 use_sim_time；导航模式禁止回放。";
  document.querySelector("#bag-playback-state").textContent = playback.playing
    ? `${offline ? "正在模拟回放" : "正在真实回放"}：${playback.bag_id || "未知"}（${playback.playback_profile || "all"}；PID ${playback.pid || "-"}）`
    : offline ? "当前没有进行中的离线模拟回放。" : "当前没有进行中的 rosbag 回放。";
  document.querySelector("#bag-list").innerHTML = (bags.bags || []).length
    ? bags.bags.map((bag) => `<article><span><strong>${escapeHtml(bag.bag_id)}</strong><br><small>${Number(bag.message_count || 0).toLocaleString()} 条消息；${escapeHtml(bag.storage_identifier || "未知存储")}</small></span><button data-bag-play="${escapeHtml(bag.bag_id)}" ${playback.playing ? "disabled" : ""}>${offline ? "模拟回放" : "回放"}</button></article>`).join("")
    : "<p class='muted'>运行目录中暂无可回放 bag。</p>";
}

async function playBag(bagId) {
  const result = await api("/api/v1/bags/play", { method: "POST", body: JSON.stringify({ bag_id: bagId, rate: Number(field("bag-rate") || 1.0), playback_profile: currentMode() === "MAPPING" ? "mapping_inputs" : currentMode() === "LOCALIZATION_DEBUG" ? "localization_inputs" : "" }) });
  await refresh();
  showToast(result.simulated ? "离线 bag 流程模拟已启动，建图面板将显示模拟预览，不会发布 ROS topic" : "rosbag 回放已启动，请确保被测节点使用 use_sim_time");
}

function render() {
  renderRuntime(); renderSummary(); renderOverview(); renderWorkflow(); renderMappingObservability(); renderMappingMap(); renderMappingPointcloud(); renderChassis(); renderMappingSession(); renderLocalization(); renderReadiness(); renderMaps(); renderNavigationMapSelection(); renderControlActions(); renderExperiments(); renderBags();
  document.querySelector("#process-state").textContent = JSON.stringify(state.overview.mode || {}, null, 2);
}

function field(id) { return document.querySelector(`#${id}`).value.trim(); }

function resetPreview(key) {
  const view = previewViews[key];
  view.initialized = false;
  view.centerOnRobot = true;
  view.zoom = 1;
  view.panX = 0;
  view.panY = 0;
  if (key === "pointcloud") {
    view.rotationDeg = 0;
    view.rotation = 0;
    const control = document.querySelector("#mapping-pointcloud-rotation");
    const output = document.querySelector("#mapping-pointcloud-rotation-value");
    if (control) control.value = "0";
    if (output) output.textContent = "0°";
  }
  if (key === "map") renderMappingMap();
  else renderMappingPointcloud();
}

function bindPreviewCanvas(id, key) {
  const canvas = document.querySelector(id);
  if (!canvas) return;
  let dragging = false;
  let lastX = 0;
  let lastY = 0;
  canvas.addEventListener("pointerdown", (event) => {
    dragging = true;
    lastX = event.clientX;
    lastY = event.clientY;
    canvas.classList.add("is-dragging");
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!dragging) return;
    const view = previewViews[key];
    view.panX += event.clientX - lastX;
    view.panY += event.clientY - lastY;
    lastX = event.clientX;
    lastY = event.clientY;
    if (key === "map") renderMappingMap();
    else renderMappingPointcloud();
  });
  const stopDragging = (event) => {
    dragging = false;
    canvas.classList.remove("is-dragging");
    if (event.pointerId != null && canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  };
  canvas.addEventListener("pointerup", stopDragging);
  canvas.addEventListener("pointercancel", stopDragging);
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const view = previewViews[key];
    view.zoom = Math.max(0.5, Math.min(8, view.zoom * (event.deltaY < 0 ? 1.15 : 0.87)));
    if (key === "map") renderMappingMap();
    else renderMappingPointcloud();
  }, { passive: false });
}

function profileArguments(profile, mappingSession = {}) {
  const args = {};
  const runtime = field("runtime-dir");
  if (profile === "navigation" && runtime) args.runtime_dir = runtime;
  if ((profile === "mapping" || profile === "navigation") && field("sensor-config")) args.user_config_path = field("sensor-config");
  if (profile === "sensor_only") {
    if (field("sensor-config")) args.user_config_path = field("sensor-config");
    args.use_sim_time = field("use-sim-time") || "false";
  }
  if (profile === "mapping") {
    const inputSource = field("mapping-input-source") || "live";
    args.start_sensor = inputSource === "bag" ? "false" : "true";
    args.start_chassis = field("start-chassis") || "false"; args.start_chassis_monitor = field("start-chassis-monitor") || "false"; args.chassis_backend = field("chassis-backend") || "bunker_can"; args.can_interface = field("can-interface") || "can0"; args.start_rviz = "true"; args.start_mapping_gui = "false"; args.use_sim_time = inputSource === "bag" ? "true" : (field("use-sim-time") || "false");
  }
  if (profile === "navigation") {
    const selected = (state.maps || []).find((item) => item.version_id === state.navigationMapVersion);
    if (!selected || selected.state !== "READY" || Number(selected.active) !== 1 || !selected.assets?.map || !selected.assets?.global_map_pcd || !selected.assets?.global_map_processing_record) {
      throw new Error("启动导航前必须选择已激活且资产完整的 READY 地图版本");
    }
    args.map_version_id = selected.version_id;
    args.map_id = selected.map_id;
    args.map = selected.assets.map;
    args.global_map_pcd = selected.assets.global_map_pcd;
    args.global_map_processing_record = selected.assets.global_map_processing_record;
    [["candidate-yaml", "configured_candidates_yaml"], ["last-pose", "last_valid_pose_path"]].forEach(([input, key]) => { if (field(input)) args[key] = field(input); });
    args.start_sensor = "true"; args.start_chassis = field("start-chassis") || "false"; args.start_chassis_monitor = "false"; args.chassis_backend = field("chassis-backend") || "bunker_can"; args.can_interface = field("can-interface") || "can0"; args.start_gui = field("start-gui") || "true"; args.auto_relocalize_on_start = field("auto-relocalize") || "false"; args.use_sim_time = field("use-sim-time") || "false";
    const bagProfile = field("record-bag-profile") || "none"; args.record_bag = bagProfile !== "none" ? "true" : "false"; args.bag_profile = bagProfile === "none" ? "full_experiment" : bagProfile;
  }
  if (profile === "qt_mapping") { args.map_frame_id = "odom"; args.use_sim_time = field("use-sim-time") || "false"; }
  if (profile === "qt_navigation") { args.map_frame_id = "map"; args.use_sim_time = field("use-sim-time") || "false"; }
  return args;
}

async function startProfile(profile) {
  const button = profileButton(profile);
  if (!button) throw new Error(`未找到 ${profile} 启动按钮`);
  if (profile === "sensor_only") {
    const mode = currentMode();
    const health = state.overview.health || {};
    const sensor = mappingComponent(health, "mid360_pointcloud");
    const sensorProcess = (state.overview.mode?.processes || []).some((item) => item.profile === "sensor_only" && item.returncode == null);
    if (sensorProcess || sensor.present === true || sensor.state === "OK" || ["MAPPING", "NAVIGATION"].includes(mode)) {
      showToast("传感器已经启动，不能重复启动", true);
      renderControlActions();
      return;
    }
  }
  button.disabled = true;
  document.querySelector("#control-message").textContent = "正在发送模块启动请求…";
  const messages = { sensor_only: "正在启动并检查 MID360…", mapping: field("mapping-input-source") === "bag" ? "正在启动建图算法，等待历史 rosbag 输入…" : "正在启动建图链，传感器将保持运行…", navigation: "正在启动导航链，传感器将保持运行…" };
  showToast(messages[profile] || "正在启动模块，请稍候…");
  try {
    let mappingSession = state.mappingSession || {};
    let result;
    if (profile === "mapping") {
      const arguments = profileArguments(profile, {});
      mappingSession = await api("/api/v1/mapping/session/prepare", {
        method: "POST",
        body: JSON.stringify({ map_name: field("map-name") || "mid360_map", arguments }),
      });
      state.mappingSession = mappingSession;
      renderMappingSession();
      result = mappingSession.offline
        ? await api("/api/v1/system/mode", { method: "POST", body: JSON.stringify({ profile, arguments: profileArguments(profile, mappingSession) }) })
        : mappingSession;
    } else {
      result = await api("/api/v1/system/mode", { method: "POST", body: JSON.stringify({ profile, arguments: profileArguments(profile, mappingSession) }) });
    }
    document.querySelector("#control-message").textContent = result.message || "模块启动请求已发送";
    showToast(result.message || "模块启动请求已发送");
    await refresh();
  } catch (error) {
    document.querySelector("#control-message").textContent = `启动失败：${error.message}`;
    throw error;
  } finally {
    button.disabled = false;
    renderControlActions();
  }
}

async function openMappingFinishDialog() {
  try {
    state.mappingSession = await api("/api/v1/mapping/session");
    renderMappingSession();
    const nameInput = document.querySelector("#mapping-finish-map-name");
    if (nameInput) nameInput.value = state.mappingSession.map_name || field("map-name") || "mid360_map";
    const confirmation = document.querySelector("#mapping-finish-confirm");
    if (confirmation) confirmation.checked = false;
    const dialog = document.querySelector("#mapping-finish-dialog");
    if (dialog?.showModal) dialog.showModal();
    else {
      const confirmation = document.querySelector("#mapping-finish-confirm");
      if (confirmation) confirmation.checked = true;
      finishMappingAction("retain").catch((error) => showToast(error.message, true));
    }
  } catch (error) {
    showToast(`无法读取建图状态：${error.message}`, true);
  }
}

async function finishMappingAction(action) {
  if (action === "retain") {
    const confirmation = document.querySelector("#mapping-finish-confirm");
    if (confirmation && !confirmation.checked) {
      showToast("请先确认本次采集已经完成，再结束建图并保存", true);
      return;
    }
  }
  const buttons = [document.querySelector("#retain-mapping"), document.querySelector("#commit-mapping"), document.querySelector("#discard-mapping")].filter(Boolean);
  buttons.forEach((button) => { button.disabled = true; });
  let polling = true;
  const poller = (async () => {
    while (polling) {
      try {
        state.mappingSession = await api("/api/v1/mapping/session");
        renderMappingSession();
      } catch (_) {
        // Keep the save request authoritative if a transient status poll fails.
      }
      if (polling) await new Promise((resolve) => window.setTimeout(resolve, 1000));
    }
  })();
  try {
    const mapName = action === "retain" ? (state.mappingSession?.map_id || state.mappingSession?.map_name || "") : "";
    const result = await api("/api/v1/mapping/finish", { method: "POST", body: JSON.stringify({ action, map_name: mapName }) });
    const dialog = document.querySelector("#mapping-finish-dialog");
    if (dialog?.open) dialog.close();
    showToast(result.message || (action === "retain" ? "候选地图已生成" : action === "commit" ? "候选地图已登记" : "建图已回收"), false);
    await refresh();
  } catch (error) {
    showToast(`建图处理失败：${error.message}`, true);
    throw error;
  } finally {
    polling = false;
    await poller;
    buttons.forEach((button) => { button.disabled = false; });
  }
}

async function runRelocalization() {
  const button = document.querySelector("#run-relocalization");
  button.disabled = true;
  document.querySelector("#localization-message").textContent = "正在执行有界重定位请求…";
  try {
    const result = await api("/api/v1/localization/relocalize", {
      method: "POST",
      body: JSON.stringify({
        mode: field("relocalize-action-mode") || "AUTO_SEARCH",
        max_candidates: Number(field("relocalize-max-candidates") || 128),
        timeout_s: Number(field("relocalize-timeout") || 30),
      }),
    });
    state.lastRelocalization = result;
    document.querySelector("#relocalization-result").textContent = JSON.stringify(result, null, 2);
    document.querySelector("#localization-message").textContent = result.success ? (result.offline ? "离线模拟重定位成功" : "重定位成功") : (result.failure_reason || "重定位失败");
    showToast(document.querySelector("#localization-message").textContent, !result.success);
    await refresh();
  } catch (error) {
    document.querySelector("#localization-message").textContent = `执行失败：${error.message}`;
    throw error;
  } finally {
    button.disabled = false;
  }
}

async function refresh() {
  state.overview = await api("/api/v1/overview");
  state.runtime = state.overview.runtime || await api("/api/v1/runtime");
  state.mappingMap = await api("/api/v1/mapping/map");
  state.mappingPointcloud = await api("/api/v1/mapping/pointcloud");
  state.chassis = await api("/api/v1/chassis/status");
  state.mappingSession = await api("/api/v1/mapping/session");
  state.maps = await api("/api/v1/maps");
  state.experiments = await api("/api/v1/experiments");
  state.bags = await api("/api/v1/bags");
  render();
  const connection = document.querySelector("#connection"); connection.textContent = "在线"; connection.className = "status ok";
}

document.querySelectorAll(".tabs button").forEach((button) => button.addEventListener("click", () => {
  document.querySelectorAll(".tabs button, .tab-panel").forEach((item) => item.classList.remove("active"));
  button.classList.add("active"); document.querySelector(`#${button.dataset.tab}`).classList.add("active");
}));

document.querySelector("#refresh").addEventListener("click", () => refresh().catch((error) => showToast(error.message, true)));
document.querySelector("#access-token").value = consoleToken();
document.querySelector("#save-token").addEventListener("click", () => { localStorage.setItem("agt_console_token", document.querySelector("#access-token").value.trim()); window.location.reload(); });
document.querySelector("#switch-backend").addEventListener("click", async () => {
  const button = document.querySelector("#switch-backend");
  button.disabled = true;
  try {
    const result = await api("/api/v1/runtime/backend", { method: "POST", body: JSON.stringify({ backend: field("runtime-backend") }) });
    state.runtime = result;
    await refresh();
    showToast(result.offline ? "已切换到离线测试模式" : "已切换到 ROS 2 真实模式");
  } catch (error) {
    showToast(`后端切换失败：${error.message}`, true);
  } finally {
    button.disabled = false;
  }
});
document.querySelector("#start-profile").addEventListener("click", () => startProfile(field("mode-profile")).catch((error) => showToast(error.message, true)));
document.querySelector("#mode-profile").addEventListener("change", renderControlActions);
document.querySelector("#can-interface").addEventListener("input", renderChassis);
document.querySelector("#mapping-input-source").addEventListener("change", () => { renderWorkflow(); renderControlActions(); });
document.querySelector("#mapping-map-center").addEventListener("click", () => resetPreview("map"));
document.querySelector("#mapping-pointcloud-center").addEventListener("click", () => resetPreview("pointcloud"));
document.querySelectorAll("[data-pointcloud-view]").forEach((button) => button.addEventListener("click", () => {
  previewViews.pointcloud.projection = button.dataset.pointcloudView || "xy";
  previewViews.pointcloud.initialized = false;
  previewViews.pointcloud.centerOnRobot = true;
  renderMappingPointcloud();
}));
document.querySelector("#mapping-pointcloud-rotation").addEventListener("input", (event) => {
  const view = previewViews.pointcloud;
  view.rotationDeg = Number(event.target.value || 0);
  view.rotation = view.rotationDeg * Math.PI / 180;
  document.querySelector("#mapping-pointcloud-rotation-value").textContent = `${view.rotationDeg.toFixed(0)}°`;
  renderMappingPointcloud();
});
bindPreviewCanvas("#mapping-map-canvas", "map");
bindPreviewCanvas("#mapping-pointcloud-canvas", "pointcloud");
document.querySelector("#copy-chassis-command").addEventListener("click", async () => { try { await navigator.clipboard.writeText(document.querySelector("#chassis-admin-command").textContent); showToast("管理员 CAN 命令已复制"); } catch (error) { showToast(`复制失败，请手动选择命令：${error.message}`, true); } });
document.querySelector("#start-mapping-profile").addEventListener("click", () => startProfile("mapping").catch((error) => showToast(error.message, true)));
document.querySelector("#start-navigation-profile").addEventListener("click", () => startProfile("navigation").catch((error) => showToast(error.message, true)));
document.querySelector("#finish-mapping").addEventListener("click", () => openMappingFinishDialog().catch(() => {}));
document.querySelector("#retain-mapping").addEventListener("click", () => finishMappingAction("retain").catch(() => {}));
document.querySelector("#commit-mapping").addEventListener("click", () => finishMappingAction("commit").catch(() => {}));
document.querySelector("#discard-mapping").addEventListener("click", () => finishMappingAction("delete").catch(() => {}));
document.querySelector("#navigation-map-version").addEventListener("change", (event) => { state.navigationMapVersion = event.target.value; renderNavigationMapSelection(); renderControlActions(); });
document.querySelector("#stop-mode").addEventListener("click", async () => {
  if (currentMode() === "MAPPING") { openMappingFinishDialog().catch(() => {}); return; }
  try { await api("/api/v1/system/stop", { method: "POST", body: "{}" }); showToast("受管理模块已停止"); await refresh(); } catch (error) { showToast(error.message, true); }
});
document.querySelector("#set-localization-mode").addEventListener("click", async () => { try { const result = await api("/api/v1/localization/mode", { method: "POST", body: JSON.stringify({ mode: field("localization-mode") }) }); document.querySelector("#localization-message").textContent = result.message || "重定位模式已保存"; await refresh(); } catch (error) { showToast(error.message, true); } });
document.querySelector("#run-relocalization").addEventListener("click", () => runRelocalization().catch((error) => showToast(error.message, true)));
document.querySelector("#reload-maps").addEventListener("click", async () => { state.maps = await api("/api/v1/maps"); renderMaps(); });
document.querySelector("#new-map-import").addEventListener("click", async () => { const map_id = window.prompt("地图 ID", "greenhouse_01"); const map_yaml = window.prompt("地图 YAML 路径"); const pcd = window.prompt("定位 PCD 路径"); const processing_record = window.prompt("processing record 路径"); if (map_id && map_yaml && pcd && processing_record) { try { await api("/api/v1/maps/import", { method: "POST", body: JSON.stringify({ map_id, map_yaml, pcd, processing_record }) }); await refresh(); } catch (error) { showToast(error.message, true); } } });
document.querySelector("#new-experiment").addEventListener("click", async () => { const title = window.prompt("实验名称", "导航实验"); if (title) { try { await api("/api/v1/experiments", { method: "POST", body: JSON.stringify({ title }) }); await refresh(); } catch (error) { showToast(error.message, true); } } });
document.querySelector("#reload-logs").addEventListener("click", async () => { try { const logs = await api(`/api/v1/logs?component=${field("log-component")}`); document.querySelector("#log-list").innerHTML = logs.map((item) => `<article><span>${escapeHtml(item.path)}</span><span>${item.size} 字节</span></article>`).join("") || "<p class='muted'>暂无受管理日志。</p>"; } catch (error) { showToast(error.message, true); } });
document.querySelector("#workflow-steps").addEventListener("click", (event) => { const action = event.target.dataset.workflowAction; if (!action) return; if (action === "relocalize") runRelocalization().catch((error) => showToast(error.message, true)); else if (action === "localization_rviz") startProfile(action).catch((error) => showToast(error.message, true)); else { document.querySelector("#mode-profile").value = action; startProfile(action).catch((error) => showToast(error.message, true)); } });
document.querySelector("#map-list").addEventListener("click", async (event) => { const selectedNavigation = event.target.dataset.selectNavigation; const validate = event.target.dataset.validate; const activate = event.target.dataset.activate; const action = event.target.dataset.mapAction; const version = event.target.dataset.versionId; if (selectedNavigation) { state.navigationMapVersion = selectedNavigation; renderNavigationMapSelection(); renderControlActions(); showToast(`已选择导航地图版本 ${selectedNavigation}`); return; } try { if (validate) window.alert(JSON.stringify(await api(`/api/v1/maps/${validate}/validate`), null, 2)); if (activate) { await api(`/api/v1/maps/${activate}/activate`, { method: "POST" }); await refresh(); } if (action) { await api(`/api/v1/maps/${version}/${action}`, { method: "POST" }); await refresh(); } } catch (error) { showToast(error.message, true); } });
document.querySelector("#experiment-list").addEventListener("click", async (event) => { const action = event.target.dataset.expAction; const id = event.target.dataset.expId; if (!action || !id) return; try { await api(`/api/v1/experiments/${id}/${action}`, { method: "POST", body: JSON.stringify(action === "start_bag" ? { profile: "minimal" } : {}) }); await refresh(); } catch (error) { showToast(error.message, true); } });
document.querySelector("#bag-selection").addEventListener("change", (event) => { state.selectedBagId = event.target.value; renderBags(); });
document.querySelector("#play-selected-bag").addEventListener("click", () => playBag(state.selectedBagId).catch((error) => showToast(error.message, true)));
document.querySelector("#bag-list").addEventListener("click", async (event) => { const bagId = event.target.dataset.bagPlay; if (!bagId) return; try { await playBag(bagId); } catch (error) { showToast(error.message, true); } });
document.querySelector("#stop-bag-playback").addEventListener("click", async () => { try { await api("/api/v1/bags/stop", { method: "POST", body: "{}" }); await refresh(); showToast("rosbag 回放已停止"); } catch (error) { showToast(error.message, true); } });

refresh().catch((error) => { const connection = document.querySelector("#connection"); connection.textContent = "离线"; connection.className = "status error"; showToast(`无法连接 Web 服务：${error.message}`, true); });
window.setInterval(() => Promise.all([api("/api/v1/mapping/map"), api("/api/v1/mapping/pointcloud"), api("/api/v1/chassis/status"), api("/api/v1/mapping/session")]).then(([map, pointcloud, chassis, session]) => { state.mappingMap = map; state.mappingPointcloud = pointcloud; state.chassis = chassis; state.mappingSession = session; renderMappingMap(); renderMappingPointcloud(); renderChassis(); renderMappingSession(); renderControlActions(); }).catch(() => {}), 2000);
const websocketUrl = new URL(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/ws`);
if (consoleToken()) websocketUrl.searchParams.set("token", consoleToken());
const socket = new WebSocket(websocketUrl.toString());
socket.onopen = () => { const connection = document.querySelector("#connection"); connection.textContent = "实时在线"; connection.className = "status ok"; };
socket.onmessage = (event) => { try { const update = JSON.parse(event.data); if (update.health) { state.overview = { ...state.overview, ...update }; render(); } else refresh().catch(() => {}); } catch (error) { console.error(error); } };
socket.onclose = () => { const connection = document.querySelector("#connection"); connection.textContent = "在线（无实时通道）"; connection.className = "status unknown"; };
