#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ModbusRTU.h>
#include <SPIFFS.h>

// ============================================================
// WIFI CONFIGURATION
// ============================================================
static const char *WIFI_SSID = "Flood";
static const char *WIFI_PASS = "degreemajor821";

// ============================================================
// GATEWAY CONFIGURATION
// ============================================================
static constexpr uint8_t NODE_COUNT = 5;
static constexpr uint8_t FIRST_NODE_ADDRESS = 1;
static constexpr uint32_t POLL_INTERVAL_MS = 500;
static constexpr uint32_t WS_BROADCAST_INTERVAL_MS = 1000;
static constexpr uint32_t NODE_STALE_TIMEOUT_MS = 5000;

// ============================================================
// PIN DEFINITIONS
// ============================================================
static constexpr int PIN_RS485_RX = 16;
static constexpr int PIN_RS485_TX = 17;
static constexpr int PIN_RS485_EN = 18;

// ============================================================
// MODBUS REGISTER OFFSETS
// ============================================================
enum InputRegisterOffset : uint16_t
{
    IR_NODE_ID = 0,
    IR_STATUS,
    IR_X_RMS_MG,
    IR_Y_RMS_MG,
    IR_Z_RMS_MG,
    IR_OVERALL_RMS_MG,
    IR_BDU_X10,
    IR_PIEZO_PEAK,
    IR_PIEZO_LEVEL,
    IR_SPIKE_COUNT_LO,
    IR_ERROR_FLAGS,
    IR_UPTIME_SECONDS_LO,
    IR_COUNT
};

enum HoldingRegisterOffset : uint16_t
{
    HR_WARNING_BDU_X10 = 0,
    HR_FAULT_BDU_X10,
    HR_PIEZO_WARN_THRESHOLD,
    HR_PIEZO_FAULT_THRESHOLD,
    HR_LED_ENABLE,
    HR_RESERVED_1,
    HR_COUNT
};

// ============================================================
// GLOBAL OBJECTS
// ============================================================
HardwareSerial rs485Serial(2);
ModbusRTU mb;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ============================================================
// DATA STRUCTURES
// ============================================================
struct NodeTelemetry
{
    uint16_t node_id;
    uint16_t status;
    uint16_t x_rms_mg;
    uint16_t y_rms_mg;
    uint16_t z_rms_mg;
    uint16_t overall_rms_mg;
    uint16_t bdu_x10;
    uint16_t piezo_peak;
    uint16_t piezo_level;
    uint16_t spike_count_lo;
    uint16_t error_flags;
    uint16_t uptime_seconds_lo;

    bool online;
    uint32_t last_poll_ms;
    uint32_t age_ms;
    uint8_t slave_address;
    String display_name;

    uint16_t warning_bdu_x10;
    uint16_t fault_bdu_x10;
    uint16_t piezo_warn_threshold;
    uint16_t piezo_fault_threshold;
    uint16_t led_enable;
};

struct PendingWriteJob
{
    bool active;
    uint8_t slave_address;
    uint16_t values[HR_COUNT];
};

NodeTelemetry nodes[NODE_COUNT];
uint16_t nodeReadBuffer[IR_COUNT] = {0};
PendingWriteJob pendingWrite = {false, 0, {0}};

// Polling / write state
uint32_t lastPollMs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastWsBroadcastMs = 0;
bool modbusBusy = false;
uint8_t currentPollIndex = 0;
uint8_t activePollIndex = 0;

// ============================================================
// HTML PAGE
// ============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Motor Vibration Monitor</title>

  <style>
    :root {
      --green: #0aa064;
      --green-dark: #06784b;
      --red: #ef4444;
      --red-dark: #c62828;
      --bg1: #071611;
      --bg2: #0b241c;
      --panel: #0f2c22;
      --border: #1e5b45;
      --text: #eaf7f1;
      --muted: #aac8bb;
      --warning: #f59e0b;
      --offline: #64748b;
      --graph-bg: #0a1d17;
      --graph-grid: rgba(255,255,255,0.08);
    }

    body {
      margin: 0;
      padding: 24px;
      font-family: Arial, sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at top right, rgba(239,68,68,0.16), transparent 28%),
        radial-gradient(circle at bottom left, rgba(10,160,100,0.18), transparent 36%),
        linear-gradient(180deg, var(--bg1), var(--bg2));
    }

    .topbar {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 18px;
      margin-bottom: 20px;
      border-bottom: 4px solid var(--red);
      box-shadow: 0 6px 18px rgba(239,68,68,0.08);
    }

    .brand {
      display: flex;
      align-items: center;
      gap: 16px;
    }

    .brand img {
      height: 60px;
      background: white;
      padding: 6px;
      border-radius: 8px;
      box-shadow: 0 0 14px rgba(239,68,68,0.15);
    }

    .brand h1 {
      margin: 0;
      font-size: 1.8rem;
    }

    .meta {
      color: var(--muted);
      font-size: 0.9rem;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
      gap: 18px;
    }

    .card {
      background: var(--panel);
      border-radius: 14px;
      padding: 16px;
      border-left: 6px solid var(--border);
      border-top: 1px solid rgba(239,68,68,0.12);
      box-shadow: 0 8px 20px rgba(0,0,0,0.35);
      transition: transform 0.18s ease, box-shadow 0.18s ease;
    }

    .card:hover {
      transform: translateY(-3px);
    }

    .card.normal {
      border-left-color: var(--green);
    }

    .card.warning {
      border-left-color: var(--warning);
      box-shadow: 0 8px 20px rgba(245,158,11,0.12);
    }

    .card.fault {
      border-left-color: var(--red);
      border-top-color: rgba(239,68,68,0.35);
      box-shadow: 0 0 18px rgba(239,68,68,0.28);
    }

    .card.offline {
      border-left-color: var(--offline);
    }

    .title {
      display: flex;
      justify-content: space-between;
      margin-bottom: 10px;
      align-items: center;
      gap: 12px;
    }

    .title-text {
      display: flex;
      flex-direction: column;
      gap: 3px;
    }

    .title-main {
      font-size: 1.1rem;
      font-weight: bold;
    }

    .title-sub {
      color: var(--muted);
      font-size: 0.82rem;
    }

    .badge {
      padding: 4px 10px;
      border-radius: 20px;
      font-size: 0.75rem;
      color: white;
      font-weight: bold;
      white-space: nowrap;
    }

    .badge.normal { background: var(--green); }
    .badge.warning { background: var(--warning); color: #1a1200; }
    .badge.fault { background: var(--red); }
    .badge.offline { background: var(--offline); }

    .section {
      margin-top: 14px;
      padding-top: 12px;
      border-top: 1px solid rgba(255,255,255,0.08);
    }

    .section-title {
      margin: 0 0 10px 0;
      color: var(--muted);
      font-size: 0.9rem;
      font-weight: bold;
      letter-spacing: 0.3px;
    }

    .row {
      display: flex;
      justify-content: space-between;
      margin: 6px 0;
      gap: 12px;
    }

    .label {
      color: var(--muted);
    }

    .value {
      font-weight: bold;
    }

    .graph-wrap {
      margin-top: 10px;
      padding: 10px;
      background: var(--graph-bg);
      border: 1px solid rgba(255,255,255,0.06);
      border-radius: 10px;
    }

    .graph-title {
      font-size: 0.82rem;
      color: var(--muted);
      margin-bottom: 6px;
    }

    canvas {
      width: 100%;
      height: 120px;
      display: block;
      background: transparent;
    }

    .config-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }

    .small {
      font-size: 0.82rem;
      color: var(--muted);
      margin-bottom: 4px;
    }

    input, select {
      width: 100%;
      padding: 6px;
      background: #081f18;
      color: white;
      border: 1px solid var(--border);
      border-radius: 6px;
    }

    input:focus, select:focus {
      outline: none;
      border-color: var(--red);
      box-shadow: 0 0 0 2px rgba(239,68,68,0.14);
    }

    .full {
      grid-column: span 2;
    }

    button {
      width: 100%;
      margin-top: 8px;
      padding: 8px;
      border: none;
      border-radius: 8px;
      background: linear-gradient(90deg, var(--green-dark), var(--green));
      color: white;
      font-weight: bold;
      cursor: pointer;
    }

    button:hover {
      filter: brightness(1.05);
    }

    .fault button {
      background: linear-gradient(90deg, var(--red-dark), var(--red));
    }
  </style>
</head>

<body>

<div class="topbar">
  <div class="brand">
    <img src="/logo.png" alt="Grande Cheese Company Logo">
    <h1>Motor Vibration Monitor</h1>
  </div>
  <div class="meta" id="gatewayMeta">Connecting...</div>
</div>

<div class="grid" id="nodeGrid"></div>

<script>
const nodeGrid = document.getElementById("nodeGrid");
const gatewayMeta = document.getElementById("gatewayMeta");

const trendHistory = {};
const piezoHistory = {};
const MAX_HISTORY = 40;
const editingState = {};

function isEditing(addr) {
  return editingState[addr] === true;
}

function attachEditListeners(addr) {
  const w = document.getElementById(`w${addr}`);
  const f = document.getElementById(`f${addr}`);

  [w, f].forEach(input => {
    if (!input) return;

    input.addEventListener("focus", () => {
      editingState[addr] = true;
    });

    input.addEventListener("blur", () => {
      setTimeout(() => {
        editingState[addr] = false;
      }, 300);
    });
  });
}

function statusClass(node) {
  if (!node.online) return "offline";
  if (node.status === 0) return "normal";
  if (node.status === 1) return "warning";
  if (node.status === 2) return "fault";
  return "offline";
}

function statusText(node) {
  if (!node.online) return "OFFLINE";
  if (node.status === 0) return "NORMAL";
  if (node.status === 1) return "WARNING";
  if (node.status === 2) return "FAULT";
  return "UNKNOWN";
}

async function send(addr) {
  const w = document.getElementById(`w${addr}`).value;
  const f = document.getElementById(`f${addr}`).value;

  await fetch("/set-config", {
    method: "POST",
    headers: {"Content-Type":"application/x-www-form-urlencoded"},
    body: `slave_address=${addr}&warning_bdu=${w}&fault_bdu=${f}&piezo_warning=1000&piezo_fault=2000&led_enable=1`
  });

  alert("Updated");
}

function renderGatewayMeta(payload) {
  gatewayMeta.innerHTML =
    `IP: ${payload.gateway_ip} | Online: ${payload.online_nodes}/${payload.node_count}`;
}

function updateTrendHistory(payload) {
  payload.nodes.forEach(node => {
    const addr = node.slave_address;
    const bdu = node.bdu_x10 / 10.0;
    const piezo = node.piezo_level;

    if (!trendHistory[addr]) trendHistory[addr] = [];
    if (!piezoHistory[addr]) piezoHistory[addr] = [];

    trendHistory[addr].push(bdu);
    piezoHistory[addr].push(piezo);

    if (trendHistory[addr].length > MAX_HISTORY) trendHistory[addr].shift();
    if (piezoHistory[addr].length > MAX_HISTORY) piezoHistory[addr].shift();
  });
}

function drawTrend(canvasId, data, nodeClass, warn, fault) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;

  const parentWidth = canvas.parentElement.clientWidth - 20;
  canvas.width = parentWidth > 50 ? parentWidth : 300;
  canvas.height = 120;

  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;

  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = "rgba(255,255,255,0.08)";
  ctx.lineWidth = 1;
  for (let i = 1; i < 4; i++) {
    const y = (h / 4) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  if (!data || data.length < 2) return;

  const maxVal = Math.max(...data, fault || 1);
  const minVal = Math.min(...data, 0);
  const range = Math.max(maxVal - minVal, 0.5);

  function scaleY(v) {
    return h - ((v - minVal) / range) * (h - 10) - 5;
  }

  if (warn !== undefined) {
    ctx.strokeStyle = "#f59e0b";
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, scaleY(warn));
    ctx.lineTo(w, scaleY(warn));
    ctx.stroke();
  }

  if (fault !== undefined) {
    ctx.strokeStyle = "#ef4444";
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(0, scaleY(fault));
    ctx.lineTo(w, scaleY(fault));
    ctx.stroke();
  }

  ctx.setLineDash([]);

  let strokeColor = "#0aa064";
  if (nodeClass === "warning") strokeColor = "#f59e0b";
  if (nodeClass === "fault") strokeColor = "#ef4444";
  if (nodeClass === "offline") strokeColor = "#64748b";

  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = 2.5;
  ctx.beginPath();

  data.forEach((v, i) => {
    const x = (i / (data.length - 1)) * (w - 1);
    const y = scaleY(v);

    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });

  ctx.stroke();

  const lastVal = data[data.length - 1];
  const lastX = w - 1;
  const lastY = scaleY(lastVal);

  ctx.fillStyle = strokeColor;
  ctx.beginPath();
  ctx.arc(lastX, lastY, 4, 0, Math.PI * 2);
  ctx.fill();
}

function renderNodes(payload) {
  const currentlyEditing = Object.values(editingState).some(v => v === true);
  if (currentlyEditing) {
    return;
  }

  nodeGrid.innerHTML = "";

  payload.nodes.forEach(node => {
    const cls = statusClass(node);
    const status = statusText(node);
    const canvasId = `trend_${node.slave_address}`;

    const card = document.createElement("div");
    card.className = `card ${cls}`;

    card.innerHTML = `
      <div class="title">
        <div class="title-text">
          <div class="title-main">${node.display_name}</div>
          <div class="title-sub">Slave ${node.slave_address}</div>
        </div>
        <span class="badge ${cls}">${status}</span>
      </div>

      <div class="row"><span class="label">BDU</span><span class="value">${(node.bdu_x10/10).toFixed(1)}</span></div>
      <div class="row"><span class="label">RMS</span><span class="value">${(node.overall_rms_mg/1000).toFixed(3)} g</span></div>
      <div class="row"><span class="label">Age</span><span class="value">${(node.age_ms/1000).toFixed(1)} s</span></div>
      <div class="row"><span class="label">Piezo</span><span class="value">${node.piezo_level}</span></div>

      <div class="section">
        <div class="section-title">BDU Trend</div>
        <div class="graph-wrap">
          <div class="graph-title">Recent live trend</div>
          <canvas id="trend_${node.slave_address}"></canvas>
        </div>
      </div>

      <div class="section">
        <div class="section-title">Piezo Trend</div>
        <div class="graph-wrap">
          <div class="graph-title">Recent live trend</div>
          <canvas id="piezo_${node.slave_address}"></canvas>
        </div>
      </div>

      <div class="section">
        <div class="section-title">Configuration</div>
        <div class="config-grid">
          <div>
            <div class="small">Warn BDU</div>
            <input id="w${node.slave_address}" value="${(node.warning_bdu_x10/10).toFixed(1)}">
          </div>
          <div>
            <div class="small">Fault BDU</div>
            <input id="f${node.slave_address}" value="${(node.fault_bdu_x10/10).toFixed(1)}">
          </div>
          <div class="full">
            <button onclick="send(${node.slave_address})">Apply</button>
          </div>
        </div>
      </div>
    `;

    nodeGrid.appendChild(card);
    attachEditListeners(node.slave_address);

    drawTrend(
      `trend_${node.slave_address}`,
      trendHistory[node.slave_address] || [],
      cls,
      node.warning_bdu_x10 / 10,
      node.fault_bdu_x10 / 10
    );

    drawTrend(
      `piezo_${node.slave_address}`,
      piezoHistory[node.slave_address] || [],
      cls
    );
  });
}

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onmessage = (e) => {
    const data = JSON.parse(e.data);
    renderGatewayMeta(data);
    updateTrendHistory(data);
    renderNodes(data);
  };

  ws.onclose = () => {
    setTimeout(connect, 2000);
  };
}

connect();
</script>

</body>
</html>
)rawliteral";

// ============================================================
// HELPER FUNCTIONS
// ============================================================
const char *statusToString(uint16_t status)
{
    switch (status)
    {
    case 0:
        return "NORMAL";
    case 1:
        return "WARNING";
    case 2:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

String getGatewayIpString()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return WiFi.localIP().toString();
    }
    return String("Not Connected");
}

uint8_t countOnlineNodes()
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        if (nodes[i].online)
        {
            count++;
        }
    }
    return count;
}

void updateNodeAgesAndStaleStatus()
{
    uint32_t now = millis();

    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        if (nodes[i].last_poll_ms == 0)
        {
            nodes[i].age_ms = 0;
            nodes[i].online = false;
            continue;
        }

        nodes[i].age_ms = now - nodes[i].last_poll_ms;

        if (nodes[i].age_ms > NODE_STALE_TIMEOUT_MS)
        {
            nodes[i].online = false;
        }
    }
}

void initializeNodeTable()
{
    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        nodes[i].node_id = 0;
        nodes[i].status = 0;
        nodes[i].x_rms_mg = 0;
        nodes[i].y_rms_mg = 0;
        nodes[i].z_rms_mg = 0;
        nodes[i].overall_rms_mg = 0;
        nodes[i].bdu_x10 = 0;
        nodes[i].piezo_peak = 0;
        nodes[i].piezo_level = 0;
        nodes[i].spike_count_lo = 0;
        nodes[i].error_flags = 0;
        nodes[i].uptime_seconds_lo = 0;
        nodes[i].online = false;
        nodes[i].last_poll_ms = 0;
        nodes[i].age_ms = 0;
        nodes[i].slave_address = FIRST_NODE_ADDRESS + i;
        nodes[i].display_name = "Node " + String(i + 1);

        nodes[i].warning_bdu_x10 = 150;
        nodes[i].fault_bdu_x10 = 300;
        nodes[i].piezo_warn_threshold = 1200;
        nodes[i].piezo_fault_threshold = 2200;
        nodes[i].led_enable = 1;
    }
}

void copyReadBufferToNode(NodeTelemetry &node, const uint16_t *buffer)
{
    node.node_id = buffer[IR_NODE_ID];
    node.status = buffer[IR_STATUS];
    node.x_rms_mg = buffer[IR_X_RMS_MG];
    node.y_rms_mg = buffer[IR_Y_RMS_MG];
    node.z_rms_mg = buffer[IR_Z_RMS_MG];
    node.overall_rms_mg = buffer[IR_OVERALL_RMS_MG];
    node.bdu_x10 = buffer[IR_BDU_X10];
    node.piezo_peak = buffer[IR_PIEZO_PEAK];
    node.piezo_level = buffer[IR_PIEZO_LEVEL];
    node.spike_count_lo = buffer[IR_SPIKE_COUNT_LO];
    node.error_flags = buffer[IR_ERROR_FLAGS];
    node.uptime_seconds_lo = buffer[IR_UPTIME_SECONDS_LO];
    node.online = true;
    node.last_poll_ms = millis();
    node.age_ms = 0;
}

bool cbRead(Modbus::ResultCode event, uint16_t transactionId, void *data)
{
    (void)transactionId;
    (void)data;

    modbusBusy = false;

    if (activePollIndex >= NODE_COUNT)
    {
        return true;
    }

    if (event == Modbus::EX_SUCCESS)
    {
        copyReadBufferToNode(nodes[activePollIndex], nodeReadBuffer);
        Serial.printf("[Gateway] Poll success for %s (slave %u)\n",
                      nodes[activePollIndex].display_name.c_str(),
                      nodes[activePollIndex].slave_address);
    }
    else
    {
        nodes[activePollIndex].online = false;
        Serial.printf("[Gateway] Poll failed for %s (slave %u), result code: %d\n",
                      nodes[activePollIndex].display_name.c_str(),
                      nodes[activePollIndex].slave_address,
                      event);
    }

    return true;
}

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void *data)
{
    (void)transactionId;
    (void)data;

    modbusBusy = false;

    if (event == Modbus::EX_SUCCESS)
    {
        Serial.printf("[Gateway] Config write success for slave %u\n", pendingWrite.slave_address);
    }
    else
    {
        Serial.printf("[Gateway] Config write failed for slave %u, result code: %d\n",
                      pendingWrite.slave_address, event);
    }

    pendingWrite.active = false;
    return true;
}

void startNextPoll()
{
    if (modbusBusy || pendingWrite.active)
    {
        return;
    }

    activePollIndex = currentPollIndex;
    uint8_t slaveAddress = nodes[activePollIndex].slave_address;

    modbusBusy = true;

    bool started = mb.readIreg(
        slaveAddress,
        0,
        nodeReadBuffer,
        IR_COUNT,
        cbRead);

    if (!started)
    {
        modbusBusy = false;
        nodes[activePollIndex].online = false;
        Serial.printf("[Gateway] Failed to start poll for slave %u\n", slaveAddress);
    }

    currentPollIndex++;
    if (currentPollIndex >= NODE_COUNT)
    {
        currentPollIndex = 0;
    }
}

void processPendingWrite()
{
    if (!pendingWrite.active || modbusBusy)
    {
        return;
    }

    modbusBusy = true;

    bool started = mb.writeHreg(
        pendingWrite.slave_address,
        0,
        pendingWrite.values,
        HR_COUNT,
        cbWrite);

    if (!started)
    {
        modbusBusy = false;
        Serial.printf("[Gateway] Failed to start config write for slave %u\n", pendingWrite.slave_address);
        pendingWrite.active = false;
    }
}

void printCompactSummaryLine()
{
    Serial.print("[Gateway Summary] ");
    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        Serial.printf("%s:", nodes[i].display_name.c_str());
        if (!nodes[i].online)
        {
            Serial.print("OFFLINE ");
            continue;
        }

        Serial.printf("%s,BDU=%.1f,Age=%.1fs ",
                      statusToString(nodes[i].status),
                      nodes[i].bdu_x10 / 10.0f,
                      nodes[i].age_ms / 1000.0f);
    }
    Serial.println();
}

String buildNodesJson()
{
    String json = "{";
    json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
    json += "\"gateway_ip\":\"" + getGatewayIpString() + "\",";
    json += "\"node_count\":" + String(NODE_COUNT) + ",";
    json += "\"online_nodes\":" + String(countOnlineNodes()) + ",";
    json += "\"nodes\":[";
    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        if (i > 0)
            json += ",";

        json += "{";
        json += "\"display_name\":\"" + nodes[i].display_name + "\",";
        json += "\"slave_address\":" + String(nodes[i].slave_address) + ",";
        json += "\"node_id\":" + String(nodes[i].node_id) + ",";
        json += "\"status\":" + String(nodes[i].status) + ",";
        json += "\"x_rms_mg\":" + String(nodes[i].x_rms_mg) + ",";
        json += "\"y_rms_mg\":" + String(nodes[i].y_rms_mg) + ",";
        json += "\"z_rms_mg\":" + String(nodes[i].z_rms_mg) + ",";
        json += "\"overall_rms_mg\":" + String(nodes[i].overall_rms_mg) + ",";
        json += "\"bdu_x10\":" + String(nodes[i].bdu_x10) + ",";
        json += "\"piezo_peak\":" + String(nodes[i].piezo_peak) + ",";
        json += "\"piezo_level\":" + String(nodes[i].piezo_level) + ",";
        json += "\"spike_count_lo\":" + String(nodes[i].spike_count_lo) + ",";
        json += "\"error_flags\":" + String(nodes[i].error_flags) + ",";
        json += "\"uptime_seconds_lo\":" + String(nodes[i].uptime_seconds_lo) + ",";
        json += "\"warning_bdu_x10\":" + String(nodes[i].warning_bdu_x10) + ",";
        json += "\"fault_bdu_x10\":" + String(nodes[i].fault_bdu_x10) + ",";
        json += "\"piezo_warn_threshold\":" + String(nodes[i].piezo_warn_threshold) + ",";
        json += "\"piezo_fault_threshold\":" + String(nodes[i].piezo_fault_threshold) + ",";
        json += "\"led_enable\":" + String(nodes[i].led_enable) + ",";
        json += "\"age_ms\":" + String(nodes[i].age_ms) + ",";
        json += "\"online\":" + String(nodes[i].online ? "true" : "false");
        json += "}";
    }
    json += "]";
    json += "}";
    return json;
}

void broadcastNodeData()
{
    ws.textAll(buildNodesJson());
}

void connectWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("Connected. Gateway IP: ");
    Serial.println(WiFi.localIP());
}

void onWsEvent(AsyncWebSocket *server,
               AsyncWebSocketClient *client,
               AwsEventType type,
               void *arg,
               uint8_t *data,
               size_t len)
{
    (void)server;
    (void)arg;
    (void)data;
    (void)len;

    if (type == WS_EVT_CONNECT)
    {
        Serial.printf("[Gateway] WebSocket client connected: %u\n", client->id());
        client->text(buildNodesJson());
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        Serial.printf("[Gateway] WebSocket client disconnected: %u\n", client->id());
    }
}

void setupWebServer()
{
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/html", INDEX_HTML); });

    server.on("/logo.png", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/logo.png", "image/png"); });

    server.on("/set-config", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        if (!request->hasParam("slave_address", true) ||
            !request->hasParam("warning_bdu", true) ||
            !request->hasParam("fault_bdu", true) ||
            !request->hasParam("piezo_warning", true) ||
            !request->hasParam("piezo_fault", true) ||
            !request->hasParam("led_enable", true)) {
            request->send(400, "text/plain", "Missing parameters");
            return;
        }

        uint8_t slaveAddress = static_cast<uint8_t>(request->getParam("slave_address", true)->value().toInt());
        float warningBdu = request->getParam("warning_bdu", true)->value().toFloat();
        float faultBdu = request->getParam("fault_bdu", true)->value().toFloat();
        uint16_t piezoWarn = static_cast<uint16_t>(request->getParam("piezo_warning", true)->value().toInt());
        uint16_t piezoFault = static_cast<uint16_t>(request->getParam("piezo_fault", true)->value().toInt());
        uint16_t ledEnable = static_cast<uint16_t>(request->getParam("led_enable", true)->value().toInt());

        if (slaveAddress < FIRST_NODE_ADDRESS || slaveAddress >= FIRST_NODE_ADDRESS + NODE_COUNT) {
            request->send(400, "text/plain", "Invalid slave address");
            return;
        }

        if (faultBdu < warningBdu) {
            faultBdu = warningBdu;
        }

        if (piezoFault < piezoWarn) {
            piezoFault = piezoWarn;
        }

        if (pendingWrite.active || modbusBusy) {
            request->send(409, "text/plain", "Gateway busy, try again");
            return;
        }

        pendingWrite.active = true;
        pendingWrite.slave_address = slaveAddress;
        pendingWrite.values[HR_WARNING_BDU_X10] = static_cast<uint16_t>(warningBdu * 10.0f);
        pendingWrite.values[HR_FAULT_BDU_X10] = static_cast<uint16_t>(faultBdu * 10.0f);
        pendingWrite.values[HR_PIEZO_WARN_THRESHOLD] = piezoWarn;
        pendingWrite.values[HR_PIEZO_FAULT_THRESHOLD] = piezoFault;
        pendingWrite.values[HR_LED_ENABLE] = ledEnable ? 1 : 0;
        pendingWrite.values[HR_RESERVED_1] = 0;

        uint8_t nodeIndex = slaveAddress - FIRST_NODE_ADDRESS;
        nodes[nodeIndex].warning_bdu_x10 = pendingWrite.values[HR_WARNING_BDU_X10];
        nodes[nodeIndex].fault_bdu_x10 = pendingWrite.values[HR_FAULT_BDU_X10];
        nodes[nodeIndex].piezo_warn_threshold = pendingWrite.values[HR_PIEZO_WARN_THRESHOLD];
        nodes[nodeIndex].piezo_fault_threshold = pendingWrite.values[HR_PIEZO_FAULT_THRESHOLD];
        nodes[nodeIndex].led_enable = pendingWrite.values[HR_LED_ENABLE];

        request->send(200, "text/plain", "Configuration queued for write"); });

    server.begin();
    Serial.println("Web server started.");
}

// ============================================================
// ARDUINO SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("Motor Vibration Monitor - Gateway Boot");
    Serial.println("Commit 10: dashboard polish and stale-node handling");

    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    Serial.println("SPIFFS mounted successfully.");

    if (SPIFFS.exists("/logo.png"))
    {
        Serial.println("logo.png found in SPIFFS");
    }
    else
    {
        Serial.println("logo.png NOT found in SPIFFS");
    }
    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    Serial.println("SPIFFS file list:");
    while (file)
    {
        Serial.print(" - ");
        Serial.println(file.name());
        file = root.openNextFile();
    }
    pinMode(PIN_RS485_EN, OUTPUT);
    digitalWrite(PIN_RS485_EN, LOW);

    rs485Serial.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    mb.begin(&rs485Serial, PIN_RS485_EN);
    mb.master();

    initializeNodeTable();

    connectWiFi();
    setupWebServer();
}

// ============================================================
// ARDUINO LOOP
// ============================================================
void loop()
{
    uint32_t now = millis();

    mb.task();
    ws.cleanupClients();

    updateNodeAgesAndStaleStatus();
    processPendingWrite();

    if (now - lastPollMs >= POLL_INTERVAL_MS)
    {
        lastPollMs = now;
        startNextPoll();
    }

    if (now - lastWsBroadcastMs >= WS_BROADCAST_INTERVAL_MS)
    {
        lastWsBroadcastMs = now;
        broadcastNodeData();
    }

    if (now - lastPrintMs >= 3000)
    {
        lastPrintMs = now;
        printCompactSummaryLine();
    }
}