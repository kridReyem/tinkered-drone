#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <math.h>
#include "config.h"
#include "drivers/motor_driver.h"
#include "drivers/flight_controller.h"
#include "drivers/kalman_filter.h"
// ENTFERNT: #include "drivers/nrf24_driver.h"
// Receiver konfiguriert NRF24 direkt — kein NRF24Driver nötig

// ============================================================
// NRF24
// ============================================================
RF24 radio(NRF24_CE, NRF24_CSN);

// ============================================================
// LSM6DS3 raw I2C
// ============================================================
#define LSM6_ADDR        0x6A
#define LSM6_WHO_AM_I    0x0F
#define LSM6_CTRL1_XL    0x10
#define LSM6_CTRL2_G     0x11
#define LSM6_CTRL3_C     0x12
#define LSM6_OUTX_L_G    0x22
#define LSM6_OUTX_L_XL   0x28

static bool lsm6_ok = false;

static constexpr float GYRO_SENS  = 0.07f;
static constexpr float ACCEL_SENS = 0.000488f;

// ============================================================
// BMP280 raw I2C
// ============================================================
#define BMP280_ADDR    0x76

static bool     bmp_ok   = false;
static uint16_t bmpT1;
static int16_t  bmpT2, bmpT3;
static uint16_t bmpP1;
static int16_t  bmpP2, bmpP3, bmpP4, bmpP5, bmpP6, bmpP7, bmpP8, bmpP9;
static int32_t  bmp_t_fine;
static float    bmp_home_alt = 0.0f;

// ============================================================
// IMU state
// ============================================================
static float imu_ax = 0, imu_ay = 0, imu_az = 1.0f;
static float imu_gx = 0, imu_gy = 0, imu_gz = 0;

KalmanAngle kf_pitch;
KalmanAngle kf_roll;
static bool kalman_ready = false;

// ============================================================
// Command state
// ============================================================
static float cmd_throttle = 0.f;
static float cmd_pitch    = 0.f;
static float cmd_roll     = 0.f;
static float cmd_yaw      = 0.f;
static bool  cmd_arm_btn  = false;
static bool  cmd_left_btn = false;

// ============================================================
// Drivers
// ============================================================
MotorDriver      motors;
FlightController fc(motors);

// ============================================================
// I2C helpers
// ============================================================
static void i2c_write8(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}
static uint8_t i2c_read8(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 1);
    return Wire.available() ? Wire.read() : 0;
}
static int16_t i2c_read16le(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 2);
    if (Wire.available() < 2) return 0;
    uint8_t lo = Wire.read(), hi = Wire.read();
    return (int16_t)((hi << 8) | lo);
}
static uint16_t i2c_readU16le(uint8_t addr, uint8_t reg) {
    return (uint16_t)i2c_read16le(addr, reg);
}
static uint32_t i2c_read24be(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 3);
    uint32_t v = (uint32_t)Wire.read() << 16;
    v |= (uint32_t)Wire.read() << 8;
    v |=  Wire.read();
    return v;
}

// ============================================================
// LSM6DS3 init + read
// ============================================================
static bool lsm6_init() {
    uint8_t id = i2c_read8(LSM6_ADDR, LSM6_WHO_AM_I);
    if (id != 0x69) {
        Serial.print(F("[LSM6] WHO_AM_I=0x")); Serial.print(id, HEX);
        Serial.println(F(" expected 0x69"));
        return false;
    }
    i2c_write8(LSM6_ADDR, LSM6_CTRL2_G,  0x8C);
    i2c_write8(LSM6_ADDR, LSM6_CTRL1_XL, 0x8C);
    i2c_write8(LSM6_ADDR, LSM6_CTRL3_C,  0x44);
    delay(10);
    return true;
}

static void lsm6_read() {
    imu_gx = i2c_read16le(LSM6_ADDR, LSM6_OUTX_L_G    ) * GYRO_SENS;
    imu_gy = i2c_read16le(LSM6_ADDR, LSM6_OUTX_L_G + 2) * GYRO_SENS;
    imu_gz = i2c_read16le(LSM6_ADDR, LSM6_OUTX_L_G + 4) * GYRO_SENS;
    imu_ax = i2c_read16le(LSM6_ADDR, LSM6_OUTX_L_XL    ) * ACCEL_SENS;
    imu_ay = i2c_read16le(LSM6_ADDR, LSM6_OUTX_L_XL + 2) * ACCEL_SENS;
    imu_az = i2c_read16le(LSM6_ADDR, LSM6_OUTX_L_XL + 4) * ACCEL_SENS;
}

// ============================================================
// BMP280 init + altitude
// ============================================================
static bool bmp280_init() {
    if (i2c_read8(BMP280_ADDR, 0xD0) != 0x58) return false;

    bmpT1 = i2c_readU16le(BMP280_ADDR, 0x88);
    bmpT2 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x8A);
    bmpT3 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x8C);
    bmpP1 = i2c_readU16le(BMP280_ADDR, 0x8E);
    bmpP2 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x90);
    bmpP3 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x92);
    bmpP4 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x94);
    bmpP5 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x96);
    bmpP6 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x98);
    bmpP7 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x9A);
    bmpP8 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x9C);
    bmpP9 = (int16_t)i2c_readU16le(BMP280_ADDR, 0x9E);

    i2c_write8(BMP280_ADDR, 0xF4, 0x2F);
    i2c_write8(BMP280_ADDR, 0xF5, 0x10);
    delay(50);
    return true;
}

static float bmp280_altitude() {
    uint32_t raw_p = i2c_read24be(BMP280_ADDR, 0xF7) >> 4;
    uint32_t raw_t = i2c_read24be(BMP280_ADDR, 0xFA) >> 4;

    int32_t v1 = (((int32_t)raw_t >> 3) - ((int32_t)bmpT1 << 1));
    v1 = (v1 * (int32_t)bmpT2) >> 11;
    int32_t v2 = (((int32_t)raw_t >> 4) - (int32_t)bmpT1);
    v2 = (((v2 * v2) >> 12) * (int32_t)bmpT3) >> 14;
    bmp_t_fine = v1 + v2;

    int64_t p1 = (int64_t)bmp_t_fine - 128000;
    int64_t p2 = p1 * p1 * (int64_t)bmpP6;
    p2 = p2 + ((p1 * (int64_t)bmpP5) << 17);
    p2 = p2 + (((int64_t)bmpP4) << 35);
    p1 = (p1 * p1 * (int64_t)bmpP3 >> 8) + (p1 * (int64_t)bmpP2 << 12);
    p1 = ((((int64_t)1 << 47) + p1) * (int64_t)bmpP1) >> 33;
    if (p1 == 0) return 0.0f;
    int64_t p = 1048576LL - (int64_t)raw_p;
    p = (((p << 31) - p2) * 3125LL) / p1;
    p1 = ((int64_t)bmpP9 * (p >> 13) * (p >> 13)) >> 25;
    p2 = ((int64_t)bmpP8 * p) >> 19;
    p = ((p + p1 + p2) >> 8) + (((int64_t)bmpP7) << 4);

    float press_hpa = (float)p / (256.0f * 100.0f);
    return 44330.0f * (1.0f - powf(press_hpa / SEA_LEVEL_PRESSURE_HPPA, 0.190295f));
}

// ============================================================
// NRF24 init (RX Ping-Pong)
// ============================================================
static bool nrf24_init() {
    SPI.setMOSI(PA7); SPI.setMISO(PA6); SPI.setSCLK(PA5);
    SPI.begin();
    if (!radio.begin(&SPI) || !radio.isChipConnected()) return false;

    radio.setChannel(NRF24_CHANNEL);
    radio.setPALevel(NRF24_PA_LEVEL);
    radio.setDataRate(NRF24_DATA_RATE);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAddressWidth(5);
    radio.setAutoAck(false);
    radio.setRetries(0, 0);

    // === HIER DIE ANPASSUNG ===
    radio.enableDynamicPayloads();   // Dynamische Paketgröße aktivieren

    radio.openReadingPipe(1, CONTROLLER_ADDRESS);
    radio.openWritingPipe(SENSOR_ADDRESS);

    radio.flush_tx();
    radio.flush_rx();
    radio.startListening();
    return true;
}

// ============================================================
// Build SensorData packet
// ============================================================
static SensorData buildAck(float alt_m) {
    SensorData s = {};

    if (lsm6_ok) {
        s.accel_x = (int16_t)(imu_ax * 1000.0f);
        s.accel_y = (int16_t)(imu_ay * 1000.0f);
        s.accel_z = (int16_t)(imu_az * 1000.0f);
        s.gyro_x  = (int16_t)(imu_gx * 10.0f);
        s.gyro_y  = (int16_t)(imu_gy * 10.0f);
        s.gyro_z  = (int16_t)(imu_gz * 10.0f);
        s.status |= 0x01;
    }

    if (bmp_ok) {
        s.altitude = (int16_t)(alt_m - bmp_home_alt);
        s.status  |= 0x02;
    }

    s.battery_mv = (uint16_t)fc.getBatteryMv();
    if (s.battery_mv > 0) s.status |= 0x04;
    s.status |= (uint8_t)(fc.getBatteryLevel() << 3);

    uint8_t cs = 0;
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&s);
    for (size_t i = 0; i < offsetof(SensorData, checksum); i++) cs ^= ptr[i];
    s.checksum = cs;
    return s;
}

// ============================================================
// Stick conversion
// ============================================================
static inline float stickToThrottle(uint16_t raw) {
    return (float)raw * (1000.0f / 4095.0f);
}
static inline float stickToAxis(uint16_t raw) {
    float centred = (float)raw - 2048.0f;
    if (centred > -ADC_DEADBAND && centred < ADC_DEADBAND) return 0.0f;
    return centred * (500.0f / 2048.0f);
}

// ============================================================
// Process incoming packet
// ============================================================
static void processPacket(const ControllerData& c) {
    cmd_throttle = stickToThrottle(c.left_y);
    cmd_yaw      = stickToAxis(c.left_x);
    cmd_pitch    = stickToAxis(c.right_y);
    cmd_roll     = stickToAxis(c.right_x);
    cmd_arm_btn  = (c.right_click != 0);
    cmd_left_btn = (c.left_click  != 0);
    fc.packetReceived();
}

// ============================================================
// setup
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println(F("===================================="));
    Serial.println(F(" STM32F411  Drone Flight Controller"));
    Serial.println(F("===================================="));

    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, 880, 200); delay(300);
    noTone(BUZZER_PIN);

    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    Serial.print(F("[LSM6DS3] "));
    lsm6_ok = lsm6_init();
    Serial.println(lsm6_ok ? F("OK") : F("FAILED"));

    Serial.print(F("[BMP280]  "));
    bmp_ok = bmp280_init();
    if (bmp_ok) {
        delay(100);
        bmp_home_alt = bmp280_altitude();
        Serial.print(F("OK  home="));
        Serial.print(bmp_home_alt, 1);
        Serial.println(F(" m"));
    } else {
        Serial.println(F("FAILED"));
    }

    Serial.print(F("[MOTORS]  "));
    motors.init(MOTOR_PWM_FREQ_HZ);
    Serial.print(F("OK  PWM "));
    Serial.print(MOTOR_PWM_FREQ_HZ);
    Serial.println(F(" Hz  FL=PA0 FR=PA1 RL=PA2 RR=PA3"));

    fc.init();

    Serial.print(F("[NRF24]   "));
    if (!nrf24_init()) {
        Serial.println(F("FAILED - halting"));
        while (true) delay(1000);
    }
    Serial.println(F("OK  Ping-Pong ready"));

    if (lsm6_ok) {
        lsm6_read();
        float pitch0 = atan2f(-imu_ax, sqrtf(imu_ay*imu_ay + imu_az*imu_az)) * (180.0f / M_PI);
        float roll0  = atan2f( imu_ay, imu_az) * (180.0f / M_PI);
        kf_pitch.init(pitch0);
        kf_roll .init(roll0);
        kalman_ready = true;
        Serial.print(F("[KALMAN]  seeded  P=")); Serial.print(pitch0, 1);
        Serial.print(F("  R=")); Serial.println(roll0, 1);
    }

    Serial.println(F("===================================="));
    Serial.println(F("DISARMED - hold RIGHT_SW 2s to arm"));
    Serial.println(F("====================================\n"));
}

// ============================================================
// loop
// ============================================================
void loop() {
    static uint32_t last_flight_us = 0;
    static uint32_t last_bat_ms    = 0;
    static uint32_t last_log_ms    = 0;
    static float    last_alt       = 0.0f;

    uint32_t now_us = micros();
    uint32_t now_ms = millis();

    // -------------------------------------------------------
    // NRF24 Ping-Pong receive
    // -------------------------------------------------------
    if (radio.available()) {
        ControllerData ctrl;
        radio.read(&ctrl, sizeof(ControllerData));

        uint8_t cs = 0;
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&ctrl);
        for (size_t i = 0; i < offsetof(ControllerData, checksum); i++) cs ^= ptr[i];

        if (cs == ctrl.checksum) {
            // 1. Hören stoppen
            radio.stopListening();
            
            // Neu: Dem Chip Zeit geben, die PLL stabil auf TX umzustellen
            delayMicroseconds(150); 
            
            // 2. Schreib-Adresse auffrischen
            radio.openWritingPipe(SENSOR_ADDRESS); 
            
            // 3. Antworten
            SensorData ack = buildAck(last_alt);
            
            // Neu: Wir erzwingen, dass write() blockiert, bis das Paket RAUS ist
            // (Wichtig bei abgeschaltetem Auto-Ack!)
            radio.write(&ack, sizeof(SensorData));
            
            // Neu: Dem Sender Zeit geben, die Daten vollständig in die Luft zu blasen
            delayMicroseconds(200); 

            // 4. Lese-Pipe frisch initialisieren und wieder hören
            radio.openReadingPipe(1, CONTROLLER_ADDRESS);
            radio.startListening();

            processPacket(ctrl);
        } else {
            Serial.println(F("[NRF24] checksum error"));
        }
    }

    // -------------------------------------------------------
    // Battery 1 Hz
    // -------------------------------------------------------
    if (now_ms - last_bat_ms >= 1000) {
        last_bat_ms = now_ms;
        fc.updateBattery();
    }

    // -------------------------------------------------------
    // 250 Hz flight loop
    // -------------------------------------------------------
    if (now_us - last_flight_us < FLIGHT_LOOP_US) return;

    float dt = (float)(now_us - last_flight_us) * 1e-6f;
    last_flight_us = now_us;
    if (dt > 0.02f) dt = 0.02f;

    if (lsm6_ok) lsm6_read();
    if (bmp_ok)  last_alt = bmp280_altitude();

    float pitch_est = 0.0f, roll_est = 0.0f;
    if (lsm6_ok && kalman_ready) {
        float accel_pitch_deg = atan2f(-imu_ax,
                                       sqrtf(imu_ay*imu_ay + imu_az*imu_az))
                                * (180.0f / M_PI);
        float accel_roll_deg  = atan2f(imu_ay, imu_az) * (180.0f / M_PI);
        pitch_est = kf_pitch.update(imu_gy, accel_pitch_deg, dt);
        roll_est  = kf_roll .update(imu_gx, accel_roll_deg,  dt);
    }

    fc.updateArmButton(cmd_arm_btn);

    float yaw_rate = lsm6_ok ? imu_gz : 0.0f;
    fc.update(cmd_throttle, cmd_pitch, cmd_roll, cmd_yaw,
              pitch_est, roll_est, yaw_rate, dt);

    if (cmd_left_btn && fc.isArmed()) {
        tone(BUZZER_PIN, 1047, 80); delay(90);
        tone(BUZZER_PIN, 1319, 160); delay(180);
        noTone(BUZZER_PIN);
    }

    // -------------------------------------------------------
    // UART telemetry 5 Hz
    // -------------------------------------------------------
    if (now_ms - last_log_ms >= 200) {
        last_log_ms = now_ms;

        static const char* const STATE_STR[] = {
            "DISARMED","ARMING","ARMED","FAILSAFE","LANDING","DISARMING"
        };
        static const char* const BAT_STR[] = {"CRIT","LOW","MED","FULL"};

        Serial.print(F("[FC] "));
        Serial.print(STATE_STR[(int)fc.getState()]);
        Serial.print(F("  THR:")); Serial.print((int)cmd_throttle);
        Serial.print(F("  P:"));  Serial.print(pitch_est, 1);
        Serial.print(F("  R:"));  Serial.print(roll_est,  1);
        Serial.print(F("  Yaw:")); Serial.print(imu_gz,   1);
        Serial.print(F("  ALT:")); Serial.print(last_alt - bmp_home_alt, 1);
        Serial.print(F("m  BAT:")); Serial.print(fc.getBatteryMv() / 1000.0f, 2);
        Serial.print(F("V("));
        Serial.print(BAT_STR[(int)fc.getBatteryLevel()]);
        Serial.println(')');
    }
}