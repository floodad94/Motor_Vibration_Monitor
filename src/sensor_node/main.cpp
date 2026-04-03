#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>
#include <ModbusRTU.h>

// ============================================================
// NODE CONFIGURATION
// ============================================================
static constexpr uint8_t NODE_ID = 1;
static constexpr uint8_t MODBUS_ADDRESS = 1;

// ============================================================
// PIN DEFINITIONS
// ============================================================
static constexpr int PIN_I2C_SDA = 21;
static constexpr int PIN_I2C_SCL = 22;

static constexpr int PIN_PIEZO_ADC = 34;

static constexpr int PIN_WS2812 = 4;
static constexpr int WS2812_COUNT = 1;

static constexpr int PIN_RS485_RX = 16;
static constexpr int PIN_RS485_TX = 17;
static constexpr int PIN_RS485_EN = 18;

// ============================================================
// SAMPLING CONFIGURATION
// ============================================================
static constexpr uint32_t ACCEL_SAMPLE_INTERVAL_MS = 10; // 100 Hz
static constexpr size_t RMS_WINDOW_SIZE = 100;           // 1 second window at 100 Hz

static constexpr uint32_t PIEZO_SAMPLE_INTERVAL_MS = 10; // 100 Hz
static constexpr float PIEZO_BASELINE_ALPHA = 0.01f;
static constexpr uint16_t PIEZO_PEAK_DECAY_STEP = 10;

// ============================================================
// MODBUS REGISTER OFFSETS
// These are 0-based offsets inside the Modbus library.
// We document them as 30001/40001 in docs, but internally we
// map them as 0,1,2... within each register space.
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
// STATUS ENUM
// ============================================================
enum NodeStatus : uint16_t
{
    STATUS_NORMAL = 0,
    STATUS_WARNING = 1,
    STATUS_FAULT = 2
};

// ============================================================
// DATA STRUCTURES
// ============================================================
struct VibrationData
{
    float x_g;
    float y_g;
    float z_g;

    float x_rms_g;
    float y_rms_g;
    float z_rms_g;
    float overall_rms_g;

    float bdu;
};

struct PiezoData
{
    uint16_t raw_adc;
    uint16_t peak_adc;
    uint16_t level_adc;
    uint32_t spike_count;

    float baseline_adc;
    bool spike_armed;
    uint16_t warning_events;
    uint16_t fault_events;
};

struct ThresholdConfig
{
    float warning_bdu;
    float fault_bdu;

    uint16_t piezo_warning;
    uint16_t piezo_fault;

    bool led_enabled;
};

struct NodeRuntime
{
    NodeStatus status;
    uint16_t error_flags;
    uint32_t uptime_seconds;
    bool adxl_ok;
};

struct SampleWindow
{
    float x[RMS_WINDOW_SIZE];
    float y[RMS_WINDOW_SIZE];
    float z[RMS_WINDOW_SIZE];
    size_t index;
    bool filled;
};

// ============================================================
// GLOBAL OBJECTS
// ============================================================
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_NeoPixel pixel(WS2812_COUNT, PIN_WS2812, NEO_GRB + NEO_KHZ800);
HardwareSerial rs485Serial(2);
ModbusRTU mb;

// ============================================================
// GLOBAL STATE
// ============================================================
VibrationData vibration = {};
PiezoData piezo = {
    0,
    0,
    0,
    0,
    0.0f,
    true,
    0,
    0};
ThresholdConfig config = {
    15.0f, // warning_bdu
    30.0f, // fault_bdu
    1200,  // piezo_warning
    2200,  // piezo_fault
    true   // led_enabled
};
NodeRuntime runtimeState = {
    STATUS_NORMAL,
    0,
    0,
    false};
SampleWindow sampleWindow = {{0}, {0}, {0}, 0, false};

// Modbus register storage
uint16_t inputRegs[IR_COUNT] = {0};
uint16_t holdingRegs[HR_COUNT] = {0};

// Timing
uint32_t lastStatusPrintMs = 0;
uint32_t lastPiezoSampleMs = 0;
uint32_t lastUptimeMs = 0;
uint32_t lastAccelSampleMs = 0;

// ============================================================
// HELPER FUNCTIONS
// ============================================================
void setRs485ReceiveMode()
{
    digitalWrite(PIN_RS485_EN, LOW);
}

void setRs485TransmitMode()
{
    digitalWrite(PIN_RS485_EN, HIGH);
}

void setPixelColor(uint8_t r, uint8_t g, uint8_t b)
{
    if (!config.led_enabled)
    {
        pixel.clear();
        pixel.show();
        return;
    }

    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

void updateStatusLed()
{
    switch (runtimeState.status)
    {
    case STATUS_NORMAL:
        setPixelColor(0, 60, 0);
        break;
    case STATUS_WARNING:
        setPixelColor(100, 45, 0);
        break;
    case STATUS_FAULT:
        setPixelColor(100, 0, 0);
        break;
    default:
        setPixelColor(0, 0, 60);
        break;
    }
}

bool initADXL345()
{
    if (!accel.begin())
    {
        runtimeState.adxl_ok = false;
        runtimeState.error_flags |= (1 << 0);
        return false;
    }

    accel.setRange(ADXL345_RANGE_16_G);
    runtimeState.adxl_ok = true;
    runtimeState.error_flags &= ~(1 << 0);
    return true;
}

void readAccelerometerInstant(float &xg, float &yg, float &zg)
{
    if (!runtimeState.adxl_ok)
    {
        xg = 0.0f;
        yg = 0.0f;
        zg = 0.0f;
        return;
    }

    sensors_event_t event;
    accel.getEvent(&event);

    xg = event.acceleration.x / 9.80665f;
    yg = event.acceleration.y / 9.80665f;
    zg = event.acceleration.z / 9.80665f;
}

void addAccelSample(float xg, float yg, float zg)
{
    sampleWindow.x[sampleWindow.index] = xg;
    sampleWindow.y[sampleWindow.index] = yg;
    sampleWindow.z[sampleWindow.index] = zg;

    sampleWindow.index++;
    if (sampleWindow.index >= RMS_WINDOW_SIZE)
    {
        sampleWindow.index = 0;
        sampleWindow.filled = true;
    }

    vibration.x_g = xg;
    vibration.y_g = yg;
    vibration.z_g = zg;
}

size_t getWindowCount()
{
    return sampleWindow.filled ? RMS_WINDOW_SIZE : sampleWindow.index;
}

float computeMean(const float *buffer, size_t count)
{
    if (count == 0)
    {
        return 0.0f;
    }

    float sum = 0.0f;
    for (size_t i = 0; i < count; i++)
    {
        sum += buffer[i];
    }
    return sum / static_cast<float>(count);
}

float computeDynamicRms(const float *buffer, size_t count, float mean)
{
    if (count == 0)
    {
        return 0.0f;
    }

    float sumSq = 0.0f;
    for (size_t i = 0; i < count; i++)
    {
        float dynamicValue = buffer[i] - mean;
        sumSq += dynamicValue * dynamicValue;
    }

    return sqrtf(sumSq / static_cast<float>(count));
}

void updateVibrationMetrics()
{
    size_t count = getWindowCount();
    if (count == 0)
    {
        vibration.x_rms_g = 0.0f;
        vibration.y_rms_g = 0.0f;
        vibration.z_rms_g = 0.0f;
        vibration.overall_rms_g = 0.0f;
        vibration.bdu = 0.0f;
        return;
    }

    float meanX = computeMean(sampleWindow.x, count);
    float meanY = computeMean(sampleWindow.y, count);
    float meanZ = computeMean(sampleWindow.z, count);

    vibration.x_rms_g = computeDynamicRms(sampleWindow.x, count, meanX);
    vibration.y_rms_g = computeDynamicRms(sampleWindow.y, count, meanY);
    vibration.z_rms_g = computeDynamicRms(sampleWindow.z, count, meanZ);

    vibration.overall_rms_g = sqrtf(
        (vibration.x_rms_g * vibration.x_rms_g) +
        (vibration.y_rms_g * vibration.y_rms_g) +
        (vibration.z_rms_g * vibration.z_rms_g));

    vibration.bdu = vibration.overall_rms_g * 100.0f;
}

void samplePiezo()
{
    uint16_t raw = analogRead(PIN_PIEZO_ADC);
    piezo.raw_adc = raw;

    if (piezo.baseline_adc == 0.0f)
    {
        piezo.baseline_adc = static_cast<float>(raw);
    }
    else
    {
        piezo.baseline_adc =
            (1.0f - PIEZO_BASELINE_ALPHA) * piezo.baseline_adc +
            PIEZO_BASELINE_ALPHA * static_cast<float>(raw);
    }

    uint16_t level = static_cast<uint16_t>(fabsf(static_cast<float>(raw) - piezo.baseline_adc));
    piezo.level_adc = level;

    if (level > piezo.peak_adc)
    {
        piezo.peak_adc = level;
    }
    else if (piezo.peak_adc > PIEZO_PEAK_DECAY_STEP)
    {
        piezo.peak_adc -= PIEZO_PEAK_DECAY_STEP;
    }
    else
    {
        piezo.peak_adc = 0;
    }

    if (piezo.spike_armed && level >= config.piezo_warning)
    {
        piezo.spike_count++;
        piezo.warning_events++;
        piezo.spike_armed = false;
    }

    if (level >= config.piezo_fault)
    {
        piezo.fault_events++;
    }

    if (level < (config.piezo_warning / 2))
    {
        piezo.spike_armed = true;
    }
}

void updateNodeStatus()
{
    bool vibrationFault = (vibration.bdu >= config.fault_bdu);
    bool vibrationWarning = (vibration.bdu >= config.warning_bdu);

    bool piezoFault = (piezo.level_adc >= config.piezo_fault);
    bool piezoWarning = (piezo.level_adc >= config.piezo_warning);

    if (!runtimeState.adxl_ok)
    {
        runtimeState.status = STATUS_FAULT;
    }
    else if (vibrationFault || piezoFault)
    {
        runtimeState.status = STATUS_FAULT;
    }
    else if (vibrationWarning || piezoWarning)
    {
        runtimeState.status = STATUS_WARNING;
    }
    else
    {
        runtimeState.status = STATUS_NORMAL;
    }
}

void updateInputRegisters()
{
    inputRegs[IR_NODE_ID] = NODE_ID;
    inputRegs[IR_STATUS] = static_cast<uint16_t>(runtimeState.status);

    inputRegs[IR_X_RMS_MG] = static_cast<uint16_t>(vibration.x_rms_g * 1000.0f);
    inputRegs[IR_Y_RMS_MG] = static_cast<uint16_t>(vibration.y_rms_g * 1000.0f);
    inputRegs[IR_Z_RMS_MG] = static_cast<uint16_t>(vibration.z_rms_g * 1000.0f);
    inputRegs[IR_OVERALL_RMS_MG] = static_cast<uint16_t>(vibration.overall_rms_g * 1000.0f);
    inputRegs[IR_BDU_X10] = static_cast<uint16_t>(vibration.bdu * 10.0f);

    inputRegs[IR_PIEZO_PEAK] = piezo.peak_adc;
    inputRegs[IR_PIEZO_LEVEL] = piezo.level_adc;
    inputRegs[IR_SPIKE_COUNT_LO] = static_cast<uint16_t>(piezo.spike_count & 0xFFFF);
    inputRegs[IR_ERROR_FLAGS] = runtimeState.error_flags;
    inputRegs[IR_UPTIME_SECONDS_LO] = static_cast<uint16_t>(runtimeState.uptime_seconds & 0xFFFF);
}

void updateHoldingRegistersFromConfig()
{
    holdingRegs[HR_WARNING_BDU_X10] = static_cast<uint16_t>(config.warning_bdu * 10.0f);
    holdingRegs[HR_FAULT_BDU_X10] = static_cast<uint16_t>(config.fault_bdu * 10.0f);
    holdingRegs[HR_PIEZO_WARN_THRESHOLD] = config.piezo_warning;
    holdingRegs[HR_PIEZO_FAULT_THRESHOLD] = config.piezo_fault;
    holdingRegs[HR_LED_ENABLE] = config.led_enabled ? 1 : 0;
    holdingRegs[HR_RESERVED_1] = 0;
}

void loadConfigFromHoldingRegisters()
{
    config.warning_bdu = static_cast<float>(holdingRegs[HR_WARNING_BDU_X10]) / 10.0f;
    config.fault_bdu = static_cast<float>(holdingRegs[HR_FAULT_BDU_X10]) / 10.0f;
    config.piezo_warning = holdingRegs[HR_PIEZO_WARN_THRESHOLD];
    config.piezo_fault = holdingRegs[HR_PIEZO_FAULT_THRESHOLD];
    config.led_enabled = (holdingRegs[HR_LED_ENABLE] != 0);

    if (config.fault_bdu < config.warning_bdu)
    {
        config.fault_bdu = config.warning_bdu;
        holdingRegs[HR_FAULT_BDU_X10] = holdingRegs[HR_WARNING_BDU_X10];
    }

    if (config.piezo_fault < config.piezo_warning)
    {
        config.piezo_fault = config.piezo_warning;
        holdingRegs[HR_PIEZO_FAULT_THRESHOLD] = holdingRegs[HR_PIEZO_WARN_THRESHOLD];
    }
}

void setupModbusRegisters()
{
    updateInputRegisters();
    updateHoldingRegistersFromConfig();

    for (uint16_t i = 0; i < IR_COUNT; i++)
    {
        mb.addIreg(i, inputRegs[i]);
    }

    for (uint16_t i = 0; i < HR_COUNT; i++)
    {
        mb.addHreg(i, holdingRegs[i]);
    }
}

void syncModbusRegisters()
{
    updateInputRegisters();

    for (uint16_t i = 0; i < IR_COUNT; i++)
    {
        mb.Ireg(i, inputRegs[i]);
    }

    for (uint16_t i = 0; i < HR_COUNT; i++)
    {
        holdingRegs[i] = mb.Hreg(i);
    }

    loadConfigFromHoldingRegisters();
}

void printNodeStatus()
{
    Serial.println("--------------------------------------------------");
    Serial.printf("Node ID: %u\n", NODE_ID);
    Serial.printf("Modbus Address: %u\n", MODBUS_ADDRESS);
    Serial.printf("ADXL345 OK: %s\n", runtimeState.adxl_ok ? "YES" : "NO");
    Serial.printf("Accel Instant [g] -> X: %.4f, Y: %.4f, Z: %.4f\n",
                  vibration.x_g, vibration.y_g, vibration.z_g);
    Serial.printf("Accel RMS [g] -> X: %.4f, Y: %.4f, Z: %.4f, Overall: %.4f\n",
                  vibration.x_rms_g, vibration.y_rms_g, vibration.z_rms_g, vibration.overall_rms_g);
    Serial.printf("BDU: %.2f\n", vibration.bdu);
    Serial.printf("Piezo -> Raw: %u, Baseline: %.1f, Level: %u, Peak: %u, Spike Count: %lu\n",
                  piezo.raw_adc,
                  piezo.baseline_adc,
                  piezo.level_adc,
                  piezo.peak_adc,
                  static_cast<unsigned long>(piezo.spike_count));
    Serial.printf("Thresholds -> Warn BDU: %.1f, Fault BDU: %.1f, Piezo Warn: %u, Piezo Fault: %u\n",
                  config.warning_bdu,
                  config.fault_bdu,
                  config.piezo_warning,
                  config.piezo_fault);
    Serial.printf("Status: %u, Error Flags: 0x%04X, Uptime: %lu s\n",
                  static_cast<unsigned int>(runtimeState.status),
                  runtimeState.error_flags,
                  static_cast<unsigned long>(runtimeState.uptime_seconds));
}

// ============================================================
// ARDUINO SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("Motor Vibration Monitor - Sensor Node Boot");
    Serial.println("Commit 5: status logic, LED, and Modbus RTU slave");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    pinMode(PIN_PIEZO_ADC, INPUT);

    pixel.begin();
    pixel.clear();
    pixel.show();

    pinMode(PIN_RS485_EN, OUTPUT);
    setRs485ReceiveMode();

    rs485Serial.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    bool adxlReady = initADXL345();
    if (adxlReady)
    {
        Serial.println("ADXL345 initialized successfully.");
    }
    else
    {
        Serial.println("ERROR: ADXL345 not detected. Check wiring.");
    }

    // Modbus RTU slave
    mb.begin(&rs485Serial, PIN_RS485_EN);
    mb.slave(MODBUS_ADDRESS);

    setupModbusRegisters();
    updateNodeStatus();
    updateStatusLed();
    syncModbusRegisters();
}

// ============================================================
// ARDUINO LOOP
// ============================================================
void loop()
{
    uint32_t now = millis();

    if (now - lastUptimeMs >= 1000)
    {
        lastUptimeMs = now;
        runtimeState.uptime_seconds++;
    }

    if (now - lastPiezoSampleMs >= PIEZO_SAMPLE_INTERVAL_MS)
    {
        lastPiezoSampleMs = now;
        samplePiezo();
    }

    if (now - lastAccelSampleMs >= ACCEL_SAMPLE_INTERVAL_MS)
    {
        lastAccelSampleMs = now;

        float xg = 0.0f;
        float yg = 0.0f;
        float zg = 0.0f;

        readAccelerometerInstant(xg, yg, zg);
        addAccelSample(xg, yg, zg);
        updateVibrationMetrics();
    }

    updateNodeStatus();
    updateStatusLed();
    syncModbusRegisters();

    mb.task();

    if (now - lastStatusPrintMs >= 1000)
    {
        lastStatusPrintMs = now;
        printNodeStatus();
    }
}