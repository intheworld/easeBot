"use strict";
const byId = id => document.getElementById(id);
const jointNames = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper", "gripper_enabled"];
for (const name of jointNames) {
  const row = document.createElement("tr");
  for (const value of [name === "gripper" ? "夹爪总开口 m" : name === "gripper_enabled" ? "夹爪电机使能" : name, "—", "—"]) {
    const cell = document.createElement("td"); cell.textContent = value; row.append(cell);
  }
  byId("joints").append(row);
}
const fmt = (value, places = 3) => typeof value === "number" && Number.isFinite(value) ? value.toFixed(places) : "—";
const set = (id, value) => { byId(id).textContent = value; };
const events = {operator_enable:"操作者使能", operator_disable:"操作者失能", operator_stop:"操作者停止", controller_acquired:"控制客户端取得租约", controller_released:"控制客户端释放租约", command_expired:"动作超时，已停止", lease_expired:"租约过期，已停止", mock_lift_homed:"模拟升降回零", driver_fault:"驱动故障"};
let lastRobotSample = null, lastRobotAdvance = 0;
function clearRobot() {
  for (const id of ["mode","state-age","controller","lift","homed","base","uptime","lift-phase","lift-limits"]) set(id, "—");
  byId("buses").replaceChildren();
  set("enable-state", "控制服务不可用"); set("state-valid", "状态不可用");
  for (const row of byId("joints").rows) { row.cells[1].textContent = "—"; row.cells[2].textContent = "—"; }
}
function updateRobot(robot) {
  set("robot-id", robot.robot_id);
  set("backend-label", robot.backend === "ros2_control_mock" ? "ROS 2 · Mock hardware" : robot.backend);
  const real = robot.backend === "ros2_control_damiao";
  set("backend-detail", real ? (robot.hardware_connected ? "四条真机总线反馈在线；控制使用独立客户端。" : "真机反馈尚未齐全，详见各总线状态。") : "原生 ros2_control 模拟反馈；相机来源独立标识。");
  set("mode", robot.mode); set("enable-state", real ? (robot.controllers_ready ? "运动控制器就绪 · 电机使能分开确认" : robot.read_only ? "只读监控 · 不使能电机" : "运动控制器未全部就绪") : robot.enabled ? "已使能 · 独立客户端控制" : "未使能");
  if (lastRobotSample !== robot.sample_id) {lastRobotSample = robot.sample_id; lastRobotAdvance = performance.now();}
  const valid = robot.valid && performance.now() - lastRobotAdvance < 1000;
  set("state-age", `${fmt(robot.state_age_ms, 0)} ms`); set("state-valid", valid ? "有效反馈" : "反馈陈旧或故障");
  byId("state-valid").classList.toggle("bad", !valid);
  set("controller", robot.controller_active ? "ROS 2 控制器" : "空闲"); set("sequence", `样本 ${robot.sample_id}`);
  const state = robot.state || {};
  jointNames.forEach((name, i) => {
    const row = byId("joints").rows[i];
    if (name === "gripper_enabled") {
      ["right", "left"].forEach((side, index) => {
        const enabled = real ? robot.grippers?.[side]?.enabled : null;
        row.cells[index + 1].textContent = enabled === true ? "已使能" : enabled === false ? "失能" : "未知";
      });
      return;
    }
    row.cells[1].textContent = fmt(name === "gripper" && real ? robot.grippers?.right?.gap_m : state[name === "gripper" ? "a_right_end_joint_L_finger" : `a_right_joint${name.split("_")[1]}`]); row.cells[2].textContent = fmt(name === "gripper" && real ? robot.grippers?.left?.gap_m : state[name === "gripper" ? "b_left_end_joint_L_finger" : `b_left_joint${name.split("_")[1]}`]);
  });
  set("lift", `${fmt(state["lift_joint"], 3)} m`); set("homed", real ? (robot.homed === true ? "已回零" : robot.homed === false ? "未回零" : "未知 / 状态陈旧") : "未知 · Mock 无真实回零");
  const liftStatus = robot.lift?.valid ? robot.lift.state : null;
  set("lift-phase", liftStatus ? (["未回零", "回零中", "已回零", "故障"][liftStatus.phase] || "未知") : "未知 / 状态陈旧");
  set("lift-limits", liftStatus ? `上: ${liftStatus.upper === 1 ? "触发" : "未触发"} / 下: ${liftStatus.lower === 1 ? "触发" : "未触发"}` : "未知 / 状态陈旧");
  byId("hardware").hidden = !real;
  byId("buses").replaceChildren();
  for (const [name, bus] of Object.entries(robot.buses || {})) {
    const row = document.createElement("tr");
    for (const value of [name, bus.valid ? "在线" : "缺失 / 陈旧 / 故障", `${fmt(bus.state_age_ms, 0)} ms`, Object.entries(bus.controllers).map(([n,s]) => `${n}: ${s}`).join(" · ")]) {
      const cell = document.createElement("td"); cell.textContent = value; row.append(cell);
    }
    row.classList.toggle("bad", !bus.valid); byId("buses").append(row);
  }
  set("base", robot.base ? robot.base.map(v => fmt(v, 2)).join(" / ") : "无有效里程计");
  set("uptime", `${fmt(robot.uptime_s, 0)} s`); set("fault", robot.fault || "无已报告故障"); byId("fault").classList.toggle("bad", !!robot.fault);
  const container = byId("event-list"); container.replaceChildren();
  for (const item of [...robot.events].reverse().slice(0, 30)) {
    const row = document.createElement("div"); row.className = "event-row";
    const timestamp = document.createElement("time"); timestamp.textContent = new Date(item.time * 1000).toLocaleTimeString();
    const message = document.createElement("span"); message.textContent = events[item.event] || item.event;
    row.append(timestamp, message); container.append(row);
  }
  if (!container.children.length) container.textContent = "暂无运行事件";
}
const cameraCards = [...document.querySelectorAll("[data-camera]")];
function unavailable(card, message) {
  card.querySelector("img").style.display = "none";
  const overlay = card.querySelector(".camera-unavailable"); overlay.style.display = "flex"; overlay.textContent = message;
  const status = card.querySelector(".camera-status"); status.classList.remove("live"); status.textContent = "不可用";
}
async function refreshImage(card, valid) {
  if (!valid) return unavailable(card, "无有效画面 / 服务离线");
  const response = await fetch(`/api/v1/cameras/${card.dataset.camera}`, {signal: AbortSignal.timeout(1600), cache:"no-store"});
  if (!response.ok) throw new Error("frame unavailable");
  const url = URL.createObjectURL(await response.blob()); const img = card.querySelector("img");
  await new Promise((resolve, reject) => {
    img.onload = () => {img.onload = img.onerror = null; resolve();};
    img.onerror = () => {img.onload = img.onerror = null; reject(new Error("frame decode failed"));};
    img.src = url;
  }).finally(() => URL.revokeObjectURL(url));
  img.style.display = "block"; card.querySelector(".camera-unavailable").style.display = "none";
}
async function poll() {
  try {
    const response = await fetch("/api/v1/status", {signal:AbortSignal.timeout(2000), cache:"no-store"});
    if (!response.ok) throw new Error("monitor unavailable");
    const data = await response.json(); const connected = data.robot.available;
    set("connection", connected ? "监控已连接" : "控制服务离线"); byId("connection").classList.toggle("offline", !connected);
    set("clock", new Date().toLocaleString()); set("version", data.system.version);
    const mem = data.system.memory;
    set("memory", mem.MemTotal ? `${((mem.MemTotal-mem.MemAvailable)/2**30).toFixed(1)} GiB` : "—");
    set("disk", `运行目录所在磁盘剩余 ${(data.system.runtime_disk.free/2**30).toFixed(1)} GiB`);
    if (connected) updateRobot(data.robot.data); else {clearRobot();set("backend-label","控制服务离线");set("backend-detail",data.robot.error || "等待 ROS 2 joint_states");}
    const cameras = data.sensors.available ? data.sensors.data.cameras : {};
    await Promise.allSettled(cameraCards.map(async card => {
      const state = cameras[card.dataset.camera];
      card.querySelector(".camera-detail").textContent = state ? `${state.backend.toUpperCase()} · ${fmt(state.age_ms,0)} ms` : "服务离线";
      const status = card.querySelector(".camera-status"); status.textContent = state?.valid ? "● 在线" : "不可用"; status.classList.toggle("live", !!state?.valid);
      try {await refreshImage(card, state?.valid);} catch {unavailable(card,"画面不可用");}
    }));
  } catch {
    set("connection", "监控连接中断"); byId("connection").classList.add("offline"); clearRobot();
    cameraCards.forEach(card => unavailable(card,"监控连接中断"));
  } finally { setTimeout(poll, 500); }
}
poll();
