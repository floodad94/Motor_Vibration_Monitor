#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>

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

static constexpr int PIN_RS485_RX = 16; // MAX485 RO
static constexpr int PIN_RS485_TX = 17; // MAX485 DI
static constexpr int PIN_RS485_EN = 18; // MAX485 DE and RE tied together

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

struct ModbusRegisters
{
    // Input registers (telemetry style)
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

    // Holding register style config mirror
    uint16_t warning_bdu_x10;
    uint16_t fault_bdu_x10;
    uint16_t piezo_warn_threshold;
    uint16_t piezo_fault_threshold;
    uint16_t led_enable;
};

// ============================================================
// GLOBAL OBJECTS
// ============================================================
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_NeoPixel pixel(WS2812_COUNT, PIN_WS2812, NEO_GRB + NEO_KHZ800);

// Use HardwareSerial port 2 for RS-485
HardwareSerial rs485Serial(2);

// ============================================================
// GLOBAL STATE
// ============================================================
VibrationData vibration = {};
PiezoData piezo = {};
ThresholdConfig config = {
    15.0f, // warning_bdu
    30.0f, // fault_bdu
    1200,  // piezo_warning
    2200,  // piezo_fault
    true   // led_enabled
};
NodeRuntime runtimeState = {
    STATUS_NORMAL, // status
    0,             // error_flags
    0,             // uptime_seconds
    false          // adxl_ok
};
ModbusRegisters regs = {};

// Timing
uint32_t lastStatusPrintMs = 0;
uint32_t lastPiezoSampleMs = 0;
uint32_t lastUptimeMs = 0;

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
        setPixelColor(0, 50, 0);
        break;
    case STATUS_WARNING:
        setPixelColor(80, 35, 0);
        break;
    case STATUS_FAULT:
        setPixelColor(80, 0, 0);
        break;
    default:
        setPixelColor(0, 0, 40);
        break;
    }
}

void initRegisters()
{
    regs.node_id = NODE_ID;
    regs.status = static_cast<uint16_t>(runtimeState.status);
    regs.x_rms_mg = 0;
    regs.y_rms_mg = 0;
    regs.z_rms_mg = 0;
    regs.overall_rms_mg = 0;
    regs.bdu_x10 = 0;
    regs.piezo_peak = 0;
    regs.piezo_level = 0;
    regs.spike_count_lo = 0;
    regs.error_flags = 0;
    regs.uptime_seconds_lo = 0;

    regs.warning_bdu_x10 = static_cast<uint16_t>(config.warning_bdu * 10.0f);
    regs.fault_bdu_x10 = static_cast<uint16_t>(config.fault_bdu * 10.0f);
    regs.piezo_warn_threshold = config.piezo_warning;
    regs.piezo_fault_threshold = config.piezo_fault;
    regs.led_enable = config.led_enabled ? 1 : 0;
}

void syncRegistersFromState()
{
    regs.node_id = NODE_ID;
    regs.status = static_cast<uint16_t>(runtimeState.status);

    regs.x_rms_mg = static_cast<uint16_t>(vibration.x_rms_g * 1000.0f);
    regs.y_rms_mg = static_cast<uint16_t>(vibration.y_rms_g * 1000.0f);
    regs.z_rms_mg = static_cast<uint16_t>(vibration.z_rms_g * 1000.0f);
    regs.overall_rms_mg = static_cast<uint16_t>(vibration.overall_rms_g * 1000.0f);
    regs.bdu_x10 = static_cast<uint16_t>(vibration.bdu * 10.0f);

    regs.piezo_peak = piezo.peak_adc;
    regs.piezo_level = piezo.level_adc;
    regs.spike_count_lo = static_cast<uint16_t>(piezo.spike_count & 0xFFFF);

    regs.error_flags = runtimeState.error_flags;
    regs.uptime_seconds_lo = static_cast<uint16_t>(runtimeState.uptime_seconds & 0xFFFF);

    regs.warning_bdu_x10 = static_cast<uint16_t>(config.warning_bdu * 10.0f);
    regs.fault_bdu_x10 = static_cast<uint16_t>(config.fault_bdu * 10.0f);
    regs.piezo_warn_threshold = config.piezo_warning;
    regs.piezo_fault_threshold = config.piezo_fault;
    regs.led_enable = config.led_enabled ? 1 : 0;
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

void readAccelerometerSnapshot()
{
    if (!runtimeState.adxl_ok)
    {
        vibration.x_g = 0.0f;
        vibration.y_g = 0.0f;
        vibration.z_g = 0.0f;
        return;
    }

    sensors_event_t event;
    accel.getEvent(&event);

    // Convert from m/s^2 to g
    vibration.x_g = event.acceleration.x / 9.80665f;
    vibration.y_g = event.acceleration.y / 9.80665f;
    vibration.z_g = event.acceleration.z / 9.80665f;
}

void samplePiezo()
{
    uint16_t raw = analogRead(PIN_PIEZO_ADC);
    piezo.raw_adc = raw;

    if (raw > piezo.peak_adc)
    {
        piezo.peak_adc = raw;
    }

    piezo.level_adc = raw;
}

void printNodeStatus()
{
    Serial.println("--------------------------------------------------");
    Serial.printf("Node ID: %u\n", NODE_ID);
    Serial.printf("Modbus Address: %u\n", MODBUS_ADDRESS);
    Serial.printf("ADXL345 OK: %s\n", runtimeState.adxl_ok ? "YES" : "NO");
    Serial.printf("Accel Snapshot [g] -> X: %.4f, Y: %.4f, Z: %.4f\n",
                  vibration.x_g, vibration.y_g, vibration.z_g);
    Serial.printf("Piezo -> Raw: %u, Peak: %u, Level: %u, Count: %lu\n",
                  piezo.raw_adc,
                  piezo.peak_adc,
                  piezo.level_adc,
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
    Serial.println("Commit 2: firmware skeleton");

    // I2C
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // Piezo ADC input
    pinMode(PIN_PIEZO_ADC, INPUT);

    // WS2812
    pixel.begin();
    pixel.clear();
    pixel.show();

    // RS-485 control
    pinMode(PIN_RS485_EN, OUTPUT);
    setRs485ReceiveMode();

    // RS-485 UART2
    rs485Serial.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

    // Initialize register mirror
    initRegisters();

    // Bring sensor online
    bool adxlReady = initADXL345();
    if (adxlReady)
    {
        Serial.println("ADXL345 initialized successfully.");
        runtimeState.status = STATUS_NORMAL;
        setPixelColor(0, 40, 0);
    }
    else
    {
        Serial.println("ERROR: ADXL345 not detected. Check wiring.");
        runtimeState.status = STATUS_FAULT;
        setPixelColor(40, 0, 0);
    }

    updateStatusLed();
    syncRegistersFromState();
}

// ============================================================
// ARDUINO LOOP
// ============================================================
void loop()
{
    uint32_t now = millis();

    // Update uptime every second
    if (now - lastUptimeMs >= 1000)
    {
        lastUptimeMs = now;
        runtimeState.uptime_seconds++;
    }

    // Piezo sample every 50 ms for basic bring-up
    if (now - lastPiezoSampleMs >= 50)
    {
        lastPiezoSampleMs = now;
        samplePiezo();
    }

    // Read accel snapshot before RMS math commit
    readAccelerometerSnapshot();

    // Sync Modbus-facing register mirror
    syncRegistersFromState();

    // Print every second
    if (now - lastStatusPrintMs >= 1000)
    {
        lastStatusPrintMs = now;
        printNodeStatus();
    }
}