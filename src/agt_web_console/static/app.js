const state = { overview: {}, runtime: {}, maps: [], experiments: [], lastRelocalization: null };

const api = async (path, options = {}) => {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
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
  document.querySelector("#runtime-warning").textContent = runtime.offline ? "当前为离线测试模式：可以验证网页流程和模拟重定位，但不会连接传感器、车辆、CAN、安全链或发送任务。" : "当前为 ROS 2 真实模式：按钮会调用真实项目接口，必须先启动系统管理器和对应节点。";
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

function renderWorkflow() {
  const health = state.overview.health || {};
  const readiness = state.overview.task_readiness || {};
  const localization = state.overview.localization || {};
  const mode = currentMode();
  const component = (id) => (health.components || []).find((item) => item.component_id === id) || {};
  const sensor = component("mid360_pointcloud");
  const imu = component("imu");
  const sensorReady = sensor.state === "OK" && imu.state === "OK";
  const localizationReady = localization.state === 3 && localization.pose_valid && localization.localization_accepted && !localization.status_stale;
  const steps = [
    { number: 1, title: "系统管理器", detail: currentRuntime().offline ? "离线模拟后端已连接，无需启动真实 system_manager" : state.overview.health ? "已收到结构化健康状态" : "请先在终端启动 system_manager", status: currentRuntime().offline ? "离线已连接" : state.overview.health ? "已连接" : "等待中", action: null },
    { number: 2, title: "传感器", detail: `MID360：${stateLabels[sensor.state] || "未知"}；IMU：${stateLabels[imu.state] || "未知"}`, status: sensorReady ? "正常" : "待检查", action: "sensor_only", actionLabel: "启动传感器" },
    { number: 3, title: "建图或导航链", detail: mode === "MAPPING" ? "当前处于建图模式" : mode === "NAVIGATION" ? "当前处于导航模式" : "选择一个主运行模式", status: mode === "MAPPING" || mode === "NAVIGATION" ? "运行中" : "未启动", action: mode === "MAPPING" ? "navigation" : "mapping", actionLabel: mode === "MAPPING" ? "切换导航" : "启动建图" },
    { number: 4, title: "地图与定位", detail: `${readiness.map_version_id || "未选择地图"}；${localizationStateLabels[localization.state] || "定位未知"}`, status: readiness.map_version_id && localizationReady ? "可用" : "待准备", action: "localization_rviz", actionLabel: "打开定位 RViz" },
    { number: 5, title: "任务执行", detail: readiness.ready ? "Nav2、TF、安全和定位门禁均满足" : (readiness.blocker_messages || ["等待任务门禁"]).slice(0, 2).join("；"), status: readiness.ready ? "允许" : "禁止", action: "navigation", actionLabel: "启动导航" },
    { number: 6, title: "重定位", detail: localization.message || "使用当前地图和注册点云执行有界 Action", status: localization.has_converged ? "已收敛" : "可手动触发", action: "relocalize", actionLabel: "执行一次" },
  ];
  document.querySelector("#workflow-steps").innerHTML = steps.map((step) => `
    <article class="workflow-step ${step.status === "允许" || step.status === "正常" || step.status === "已连接" || step.status === "离线已连接" ? "complete" : ""}">
      <div class="step-number">${step.number}</div><div class="step-content"><h3>${step.title}</h3><p>${escapeHtml(step.detail)}</p><span class="step-status">${escapeHtml(step.status)}</span></div>
      ${step.action ? `<button class="step-action secondary" data-workflow-action="${step.action}">${step.actionLabel}</button>` : ""}
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
      <span class="button-row"><button data-validate="${escapeHtml(map.version_id)}">校验</button><button data-activate="${escapeHtml(map.version_id)}">激活</button><button data-map-action="${map.pinned ? "unpin" : "pin"}" data-version-id="${escapeHtml(map.version_id)}">${map.pinned ? "取消固定" : "固定"}</button><button data-map-action="archive" data-version-id="${escapeHtml(map.version_id)}">归档</button></span></article>`).join("") : "<p class='muted'>暂无已注册地图版本。</p>";
}

function renderExperiments() {
  const offline = currentRuntime().offline;
  document.querySelector("#experiment-list").innerHTML = state.experiments.length ? state.experiments.map((experiment) => `
    <article><span><strong>${escapeHtml(experiment.title || experiment.experiment_id)}</strong><br><small>${escapeHtml(experimentStateLabels[experiment.state] || experiment.state)} · ${escapeHtml(experiment.experiment_id)}</small></span>
      <span>${escapeHtml(experiment.result_status || "")} ${experiment.state === "CREATED" ? `<button data-exp-action="start" data-exp-id="${escapeHtml(experiment.experiment_id)}">开始实验</button>` : experiment.state === "RUNNING" ? (offline ? "<span class='muted'>离线模式不录包</span>" : `<button data-exp-action="start_bag" data-exp-id="${escapeHtml(experiment.experiment_id)}">开始录包</button><button data-exp-action="stop_bag" data-exp-id="${escapeHtml(experiment.experiment_id)}">停止录包</button>`) + `<button data-exp-action="finalize" data-exp-id="${escapeHtml(experiment.experiment_id)}">结束实验</button>` : ""}</span></article>`).join("") : "<p class='muted'>暂无实验会话。</p>";
}

function render() {
  renderRuntime(); renderSummary(); renderOverview(); renderWorkflow(); renderLocalization(); renderReadiness(); renderMaps(); renderExperiments();
  document.querySelector("#process-state").textContent = JSON.stringify(state.overview.mode || {}, null, 2);
}

function field(id) { return document.querySelector(`#${id}`).value.trim(); }

function profileArguments(profile) {
  const args = {};
  const runtime = field("runtime-dir");
  if ((profile === "mapping" || profile === "navigation") && runtime) args.runtime_dir = runtime;
  if (profile === "sensor_only") args.use_sim_time = field("use-sim-time") || "false";
  if (profile === "mapping") {
    if (field("map-name")) args.map_name = field("map-name");
    args.start_sensor = "true"; args.start_chassis = "true"; args.start_rviz = "true"; args.start_mapping_gui = "false"; args.record_bag = "false"; args.use_sim_time = field("use-sim-time") || "false";
  }
  if (profile === "navigation") {
    [["map-yaml", "map"], ["map-pcd", "global_map_pcd"], ["map-record", "global_map_processing_record"], ["map-id", "map_id"], ["candidate-yaml", "configured_candidates_yaml"], ["last-pose", "last_valid_pose_path"]].forEach(([input, key]) => { if (field(input)) args[key] = field(input); });
    args.start_sensor = "true"; args.start_chassis = "true"; args.start_gui = field("start-gui") || "true"; args.auto_relocalize_on_start = field("auto-relocalize") || "false"; args.use_sim_time = field("use-sim-time") || "false";
  }
  if (profile === "qt_mapping") { args.map_frame_id = "odom"; args.use_sim_time = field("use-sim-time") || "false"; }
  if (profile === "qt_navigation") { args.map_frame_id = "map"; args.use_sim_time = field("use-sim-time") || "false"; }
  return args;
}

const mainProfiles = { sensor_only: "SENSOR_ONLY", mapping: "MAPPING", navigation: "NAVIGATION" };

async function startProfile(profile) {
  const button = document.querySelector("#start-profile");
  button.disabled = true;
  document.querySelector("#control-message").textContent = "正在发送模块启动请求…";
  try {
    const active = currentMode();
    const target = mainProfiles[profile];
    if (target && active !== "IDLE" && active !== target) await api("/api/v1/system/stop", { method: "POST", body: "{}" });
    const result = await api("/api/v1/system/mode", { method: "POST", body: JSON.stringify({ profile, arguments: profileArguments(profile) }) });
    document.querySelector("#control-message").textContent = result.message || "模块启动请求已发送";
    showToast(result.message || "模块启动请求已发送");
    await refresh();
  } catch (error) {
    document.querySelector("#control-message").textContent = `启动失败：${error.message}`;
    throw error;
  } finally {
    button.disabled = false;
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
  state.maps = await api("/api/v1/maps");
  state.experiments = await api("/api/v1/experiments");
  render();
  const connection = document.querySelector("#connection"); connection.textContent = "在线"; connection.className = "status ok";
}

document.querySelectorAll(".tabs button").forEach((button) => button.addEventListener("click", () => {
  document.querySelectorAll(".tabs button, .tab-panel").forEach((item) => item.classList.remove("active"));
  button.classList.add("active"); document.querySelector(`#${button.dataset.tab}`).classList.add("active");
}));

document.querySelector("#refresh").addEventListener("click", () => refresh().catch((error) => showToast(error.message, true)));
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
document.querySelector("#stop-mode").addEventListener("click", async () => { try { await api("/api/v1/system/stop", { method: "POST", body: "{}" }); showToast("受管理模块已停止"); await refresh(); } catch (error) { showToast(error.message, true); } });
document.querySelector("#set-localization-mode").addEventListener("click", async () => { try { const result = await api("/api/v1/localization/mode", { method: "POST", body: JSON.stringify({ mode: field("localization-mode") }) }); document.querySelector("#localization-message").textContent = result.message || "重定位模式已保存"; await refresh(); } catch (error) { showToast(error.message, true); } });
document.querySelector("#run-relocalization").addEventListener("click", () => runRelocalization().catch((error) => showToast(error.message, true)));
document.querySelector("#reload-maps").addEventListener("click", async () => { state.maps = await api("/api/v1/maps"); renderMaps(); });
document.querySelector("#new-map-import").addEventListener("click", async () => { const map_id = window.prompt("地图 ID", "greenhouse_01"); const map_yaml = window.prompt("地图 YAML 路径"); const pcd = window.prompt("定位 PCD 路径"); const processing_record = window.prompt("processing record 路径"); if (map_id && map_yaml && pcd && processing_record) { try { await api("/api/v1/maps/import", { method: "POST", body: JSON.stringify({ map_id, map_yaml, pcd, processing_record }) }); await refresh(); } catch (error) { showToast(error.message, true); } } });
document.querySelector("#new-experiment").addEventListener("click", async () => { const title = window.prompt("实验名称", "导航实验"); if (title) { try { await api("/api/v1/experiments", { method: "POST", body: JSON.stringify({ title }) }); await refresh(); } catch (error) { showToast(error.message, true); } } });
document.querySelector("#reload-logs").addEventListener("click", async () => { try { const logs = await api(`/api/v1/logs?component=${field("log-component")}`); document.querySelector("#log-list").innerHTML = logs.map((item) => `<article><span>${escapeHtml(item.path)}</span><span>${item.size} 字节</span></article>`).join("") || "<p class='muted'>暂无受管理日志。</p>"; } catch (error) { showToast(error.message, true); } });
document.querySelector("#workflow-steps").addEventListener("click", (event) => { const action = event.target.dataset.workflowAction; if (!action) return; if (action === "relocalize") runRelocalization().catch((error) => showToast(error.message, true)); else if (action === "localization_rviz") startProfile(action).catch((error) => showToast(error.message, true)); else { document.querySelector("#mode-profile").value = action; startProfile(action).catch((error) => showToast(error.message, true)); } });
document.querySelector("#map-list").addEventListener("click", async (event) => { const validate = event.target.dataset.validate; const activate = event.target.dataset.activate; const action = event.target.dataset.mapAction; const version = event.target.dataset.versionId; try { if (validate) window.alert(JSON.stringify(await api(`/api/v1/maps/${validate}/validate`), null, 2)); if (activate) { await api(`/api/v1/maps/${activate}/activate`, { method: "POST" }); await refresh(); } if (action) { await api(`/api/v1/maps/${version}/${action}`, { method: "POST" }); await refresh(); } } catch (error) { showToast(error.message, true); } });
document.querySelector("#experiment-list").addEventListener("click", async (event) => { const action = event.target.dataset.expAction; const id = event.target.dataset.expId; if (!action || !id) return; try { await api(`/api/v1/experiments/${id}/${action}`, { method: "POST", body: JSON.stringify(action === "start_bag" ? { profile: "minimal" } : {}) }); await refresh(); } catch (error) { showToast(error.message, true); } });

refresh().catch((error) => { const connection = document.querySelector("#connection"); connection.textContent = "离线"; connection.className = "status error"; showToast(`无法连接 Web 服务：${error.message}`, true); });
const socket = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/ws`);
socket.onopen = () => { const connection = document.querySelector("#connection"); connection.textContent = "实时在线"; connection.className = "status ok"; };
socket.onmessage = (event) => { try { const update = JSON.parse(event.data); if (update.health) { state.overview = { ...state.overview, ...update }; render(); } else refresh().catch(() => {}); } catch (error) { console.error(error); } };
socket.onclose = () => { const connection = document.querySelector("#connection"); connection.textContent = "在线（无实时通道）"; connection.className = "status unknown"; };
