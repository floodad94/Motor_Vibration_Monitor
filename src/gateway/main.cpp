#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ModbusRTU.h>

// ============================================================
// WIFI CONFIGURATION
// ============================================================
static const char *WIFI_SSID = "YOUR_WIFI_SSID";
static const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ============================================================
// GATEWAY CONFIGURATION
// ============================================================
static constexpr uint8_t NODE_COUNT = 5;
static constexpr uint8_t FIRST_NODE_ADDRESS = 1;
static constexpr uint32_t POLL_INTERVAL_MS = 500;
static constexpr uint32_t WS_BROADCAST_INTERVAL_MS = 1000;

// ============================================================
// PIN DEFINITIONS
// ============================================================
static constexpr int PIN_RS485_RX = 16;
static constexpr int PIN_RS485_TX = 17;
static constexpr int PIN_RS485_EN = 18;

// ============================================================
// MODBUS REGISTER OFFSETS
// Must match sensor node firmware.
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
    uint8_t slave_address;
};

NodeTelemetry nodes[NODE_COUNT];
uint16_t nodeReadBuffer[IR_COUNT] = {0};

// Polling state
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
    body {
      font-family: Arial, sans-serif;
      background: #0f172a;
      color: #e5e7eb;
      margin: 0;
      padding: 20px;
    }

    h1 {
      margin-top: 0;
      font-size: 1.8rem;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 16px;
      margin-top: 20px;
    }

    .card {
      background: #1e293b;
      border-radius: 14px;
      padding: 16px;
      box-shadow: 0 4px 18px rgba(0,0,0,0.25);
      border-left: 8px solid #475569;
    }

    .card.normal {
      border-left-color: #22c55e;
    }

    .card.warning {
      border-left-color: #f59e0b;
    }

    .card.fault {
      border-left-color: #ef4444;
    }

    .card.offline {
      border-left-color: #64748b;
      opacity: 0.8;
    }

    .title {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 10px;
      font-size: 1.1rem;
      font-weight: bold;
    }

    .badge {
      border-radius: 999px;
      padding: 4px 10px;
      font-size: 0.8rem;
      font-weight: bold;
      color: #111827;
      background: #cbd5e1;
    }

    .badge.normal { background: #22c55e; }
    .badge.warning { background: #f59e0b; }
    .badge.fault { background: #ef4444; }
    .badge.offline { background: #94a3b8; }

    .row {
      display: flex;
      justify-content: space-between;
      margin: 6px 0;
      font-size: 0.95rem;
    }

    .label {
      color: #cbd5e1;
    }

    .value {
      font-weight: bold;
      color: #f8fafc;
    }

    .topbar {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
    }

    .meta {
      color: #cbd5e1;
      font-size: 0.95rem;
    }
  </style>
</head>
<body>
  <div class="topbar">
    <h1>Motor Vibration Monitor</h1>
    <div class="meta">
      Gateway WebSocket Dashboard
    </div>
  </div>

  <div class="grid" id="nodeGrid"></div>

  <script>
    const nodeGrid = document.getElementById("nodeGrid");

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

    function renderNodes(payload) {
      nodeGrid.innerHTML = "";

      payload.nodes.forEach((node, index) => {
        const cls = statusClass(node);
        const status = statusText(node);

        const card = document.createElement("div");
        card.className = `card ${cls}`;

        card.innerHTML = `
          <div class="title">
            <span>Node ${index + 1}</span>
            <span class="badge ${cls}">${status}</span>
          </div>
          <div class="row"><span class="label">Slave Address</span><span class="value">${node.slave_address}</span></div>
          <div class="row"><span class="label">Node ID</span><span class="value">${node.node_id}</span></div>
          <div class="row"><span class="label">Online</span><span class="value">${node.online ? "YES" : "NO"}</span></div>
          <div class="row"><span class="label">Overall RMS</span><span class="value">${(node.overall_rms_mg / 1000).toFixed(3)} g</span></div>
          <div class="row"><span class="label">BDU</span><span class="value">${(node.bdu_x10 / 10).toFixed(1)}</span></div>
          <div class="row"><span class="label">X RMS</span><span class="value">${(node.x_rms_mg / 1000).toFixed(3)} g</span></div>
          <div class="row"><span class="label">Y RMS</span><span class="value">${(node.y_rms_mg / 1000).toFixed(3)} g</span></div>
          <div class="row"><span class="label">Z RMS</span><span class="value">${(node.z_rms_mg / 1000).toFixed(3)} g</span></div>
          <div class="row"><span class="label">Piezo Peak</span><span class="value">${node.piezo_peak}</span></div>
          <div class="row"><span class="label">Piezo Level</span><span class="value">${node.piezo_level}</span></div>
          <div class="row"><span class="label">Spike Count</span><span class="value">${node.spike_count_lo}</span></div>
          <div class="row"><span class="label">Error Flags</span><span class="value">0x${Number(node.error_flags).toString(16).padStart(4,"0")}</span></div>
          <div class="row"><span class="label">Uptime</span><span class="value">${node.uptime_seconds_lo} s</span></div>
        `;

        nodeGrid.appendChild(card);
      });
    }

    function connectWs() {
      const ws = new WebSocket(`ws://${location.host}/ws`);

      ws.onmessage = (event) => {
        const payload = JSON.parse(event.data);
        renderNodes(payload);
      };

      ws.onclose = () => {
        setTimeout(connectWs, 2000);
      };
    }

    connectWs();
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
        nodes[i].slave_address = FIRST_NODE_ADDRESS + i;
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
        Serial.printf("[Gateway] Poll success for slave %u\n", nodes[activePollIndex].slave_address);
    }
    else
    {
        nodes[activePollIndex].online = false;
        Serial.printf("[Gateway] Poll failed for slave %u, result code: %d\n",
                      nodes[activePollIndex].slave_address, event);
    }

    return true;
}

void startNextPoll()
{
    if (modbusBusy)
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

void printCompactSummaryLine()
{
    Serial.print("[Gateway Summary] ");
    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        Serial.printf("N%u:", i + 1);
        if (!nodes[i].online)
        {
            Serial.print("OFFLINE ");
            continue;
        }

        Serial.printf("%s,BDU=%.1f ",
                      statusToString(nodes[i].status),
                      nodes[i].bdu_x10 / 10.0f);
    }
    Serial.println();
}

String buildNodesJson()
{
    String json = "{";
    json += "\"nodes\":[";
    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        if (i > 0)
            json += ",";

        json += "{";
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
        json += "\"online\":" + String(nodes[i].online ? "true" : "false");
        json += "}";
    }
    json += "]";
    json += "}";
    return json;
}

void broadcastNodeData()
{
    String payload = buildNodesJson();
    ws.textAll(payload);
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
    Serial.println("Commit 8: WebSocket dashboard for five nodes");

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