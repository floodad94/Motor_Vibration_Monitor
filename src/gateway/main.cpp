#include <Arduino.h>
#include <ModbusRTU.h>

// ============================================================
// GATEWAY CONFIGURATION
// ============================================================
static constexpr uint8_t NODE_COUNT = 5;
static constexpr uint8_t FIRST_NODE_ADDRESS = 1;
static constexpr uint32_t POLL_INTERVAL_MS = 500;

// ============================================================
// PIN DEFINITIONS
// ============================================================
static constexpr int PIN_RS485_RX = 16; // MAX485 RO
static constexpr int PIN_RS485_TX = 17; // MAX485 DI
static constexpr int PIN_RS485_EN = 18; // MAX485 DE and RE tied together

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
bool modbusBusy = false;
uint8_t currentPollIndex = 0;
uint8_t activePollIndex = 0;

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
        slaveAddress,   // slave ID
        0,              // starting register offset
        nodeReadBuffer, // destination buffer
        IR_COUNT,       // number of input registers
        cbRead          // callback
    );

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

void printOneNode(const NodeTelemetry &node, uint8_t index)
{
    Serial.println("--------------------------------------------------");
    Serial.printf("Node Slot: %u\n", index + 1);
    Serial.printf("Slave Address: %u\n", node.slave_address);

    if (!node.online)
    {
        Serial.println("Online: NO");
        return;
    }

    Serial.println("Online: YES");
    Serial.printf("Node ID: %u\n", node.node_id);
    Serial.printf("Status: %s (%u)\n", statusToString(node.status), node.status);

    Serial.printf("X RMS: %.3f g\n", node.x_rms_mg / 1000.0f);
    Serial.printf("Y RMS: %.3f g\n", node.y_rms_mg / 1000.0f);
    Serial.printf("Z RMS: %.3f g\n", node.z_rms_mg / 1000.0f);
    Serial.printf("Overall RMS: %.3f g\n", node.overall_rms_mg / 1000.0f);
    Serial.printf("BDU: %.1f\n", node.bdu_x10 / 10.0f);

    Serial.printf("Piezo Peak: %u\n", node.piezo_peak);
    Serial.printf("Piezo Level: %u\n", node.piezo_level);
    Serial.printf("Spike Count: %u\n", node.spike_count_lo);

    Serial.printf("Error Flags: 0x%04X\n", node.error_flags);
    Serial.printf("Uptime (low word): %u s\n", node.uptime_seconds_lo);
    Serial.printf("Last Poll ms: %lu\n", static_cast<unsigned long>(node.last_poll_ms));
}

void printAllNodesSummary()
{
    Serial.println("==================================================");
    Serial.println("Gateway Five-Node Summary");

    for (uint8_t i = 0; i < NODE_COUNT; i++)
    {
        printOneNode(nodes[i], i);
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

// ============================================================
// ARDUINO SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("Motor Vibration Monitor - Gateway Boot");
    Serial.println("Commit 7: five-node Modbus polling");

    pinMode(PIN_RS485_EN, OUTPUT);
    digitalWrite(PIN_RS485_EN, LOW);

    rs485Serial.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    mb.begin(&rs485Serial, PIN_RS485_EN);
    mb.master();

    initializeNodeTable();
}

// ============================================================
// ARDUINO LOOP
// ============================================================
void loop()
{
    uint32_t now = millis();

    mb.task();

    if (now - lastPollMs >= POLL_INTERVAL_MS)
    {
        lastPollMs = now;
        startNextPoll();
    }

    if (now - lastPrintMs >= 3000)
    {
        lastPrintMs = now;
        printCompactSummaryLine();
        printAllNodesSummary();
    }
}