#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include "config_drohne.h"
#include "drivers/lsm6ds3_driver.h"
#include "drivers/bmp280_driver.h"
#include "drivers/motor_driver.h"

// --- Hardware-Instanzen ---
LSM6DS3Driver imu(LSM6DS3_ADDRESS);
BMP280Driver  baro(MY_BMP280_ADDRESS);
RF24          radio(NRF24_CE, NRF24_CSN);
MotorDriver   motors;

// --- Globale Variablen ---
static float    bmp_home_alt         = 0.0f;
static float    drone_alt_home       = 0.0f;
static bool     height_calibrated    = false;
static ControllerData live_ctrl_data = {2047, 2047, 2047, 2047, 0, 0, 100, 0};
static bool     funk_verbindung_aktiv = false;
static uint32_t last_packet_time_ms  = 0;
static bool     hardware_fehler      = false;
static bool     drohne_scharf        = false;

static float pressure          = 0.0f;
static float temp_bmp          = 0.0f;
static float absolute_altitude = 0.0f;
static float smoothed_bat_mv   = 3700.0f;

// ---------------------------------------------------------------------------
uint8_t calcSensorChecksum(const SensorData& s) {
    uint8_t cs = 0;
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&s);
    for (size_t i = 0; i < offsetof(SensorData, checksum); i++) cs ^= ptr[i];
    return cs;
}

bool init_funk_rx() {
    SPI.setMOSI(NRF24_SPI_MOSI);
    SPI.setMISO(NRF24_SPI_MISO);
    SPI.setSCLK(NRF24_SPI_SCK);
    SPI.begin();
    if (!radio.begin(&SPI) || !radio.isChipConnected()) return false;
    radio.setChannel(NRF24_CHANNEL);
    radio.setPALevel(NRF24_PA_LEVEL);
    radio.setDataRate(NRF24_DATA_RATE);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAddressWidth(5);
    radio.setAutoAck(true);
    radio.setRetries(1, 15);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.openReadingPipe(1, CONTROLLER_ADDRESS);
    SensorData first_payload = {};
    radio.writeAckPayload(1, &first_payload, sizeof(SensorData));
    radio.startListening();
    return true;
}

// ---------------------------------------------------------------------------
void setup() {
    Serial1.setTx(UART_TX_PIN);
    Serial1.setRx(UART_RX_PIN);
    DEBUG_UART.begin(115200);
    delay(500);

    analogReadResolution(12);
    pinMode(BAT_ADC_PIN,   INPUT_ANALOG);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN,   OUTPUT);

    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN,   HIGH);
    delay(200);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN,   LOW);

    DEBUG_UART.println(F("\n===================================="));
    DEBUG_UART.println(F("[DROHNE] Stufe 2 — Schubtest"));
    DEBUG_UART.println(F("===================================="));

    // --- Motoren initialisieren (ZUERST — sofort auf 0) ---
    motors.init(MOTOR_PWM_FREQ_HZ);
    motors.stopAll();
    DEBUG_UART.println(F("[MOTOR]  OK — alle Motoren gestoppt"));

    // --- I2C ---
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    // --- IMU ---
    if (!imu.begin(&Wire)) {
        DEBUG_UART.println(F("[LSM6DS3] CRITICAL ERROR: Init failed!"));
        hardware_fehler = true;
    } else {
        DEBUG_UART.println(F("[LSM6DS3] OK"));
    }

    // --- Barometer ---
    if (!baro.init()) {
        DEBUG_UART.println(F("[BMP280]  CRITICAL ERROR: Init failed!"));
        hardware_fehler = true;
    } else {
        DEBUG_UART.println(F("[BMP280]  OK"));
        delay(300);
        baro.readData(pressure, temp_bmp, absolute_altitude);
        bmp_home_alt = absolute_altitude;
        DEBUG_UART.print(F("[BMP280]  Bodenhöhe: "));
        DEBUG_UART.print(bmp_home_alt, 2);
        DEBUG_UART.println(F(" m"));
    }

    // --- NRF24 ---
    DEBUG_UART.print(F("[NRF24]   Initialisiere... "));
    if (!init_funk_rx()) {
        DEBUG_UART.println(F("FAILED!"));
        hardware_fehler = true;
    } else {
        DEBUG_UART.println(F("OK!"));
    }

    DEBUG_UART.println(F("===================================="));
    DEBUG_UART.println(F("STUFE 2: Throttle-Stick steuert alle"));
    DEBUG_UART.println(F("         4 Motoren gleichzeitig."));
    DEBUG_UART.println(F("         Scharfschalten: Linker Klick"));
    DEBUG_UART.println(F("         Drohne FESTHALTEN!"));
    DEBUG_UART.println(F("====================================\n"));
}

// ---------------------------------------------------------------------------
void loop() {
    static uint32_t last_print_ms = 0;
    static uint32_t last_blink_ms = 0;
    static uint32_t last_baro_ms  = 0;
    static bool     blink_state   = false;
    uint32_t now_ms = millis();

    // =========================================================================
    // 1. IMU
    // =========================================================================
    IMUData imu_data = {};
    bool imu_ok = false;
    if (imu.isReady()) imu_ok = imu.readData(imu_data);

    // =========================================================================
    // 2. BAROMETER (10Hz)
    // =========================================================================
    if (now_ms - last_baro_ms >= 100) {
        last_baro_ms = now_ms;
        if (baro.isInitialized()) {
            baro.readData(pressure, temp_bmp, absolute_altitude);
        }
    }

    // =========================================================================
    // 3. AKKUMESSUNG
    // =========================================================================
    int   raw_adc        = analogRead(BAT_ADC_PIN);
    float pin_voltage_mv = ((float)raw_adc * BAT_ADC_VREF_MV) / 4095.0f;
    float current_bat_mv = pin_voltage_mv * BAT_DIVIDER_FACTOR;
    smoothed_bat_mv      = (smoothed_bat_mv * 0.98f) + (current_bat_mv * 0.02f);

    // =========================================================================
    // 4. FUNK-EMPFANG
    // =========================================================================
    if (radio.available()) {
        ControllerData ctrl_packet;
        radio.read(&ctrl_packet, sizeof(ControllerData));

        uint8_t cs = 0;
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&ctrl_packet);
        for (size_t i = 0; i < offsetof(ControllerData, checksum); i++) cs ^= ptr[i];

        if (cs == ctrl_packet.checksum) {
            live_ctrl_data        = ctrl_packet;
            last_packet_time_ms   = now_ms;
            funk_verbindung_aktiv = true;

            // Höhenkalibrierung
            if (!height_calibrated) {
                drone_alt_home    = absolute_altitude;
                height_calibrated = true;
            }

            // Arming-Signal aus Bit 7 von battery_level
            bool arm_signal = (ctrl_packet.battery_level & 0x80) != 0;
            if (arm_signal && !drohne_scharf) {
                drohne_scharf = true;
                DEBUG_UART.println(F("[ARM] Drohne SCHARF"));
            } else if (!arm_signal && drohne_scharf) {
                drohne_scharf = false;
                motors.stopAll();
                DEBUG_UART.println(F("[ARM] Drohne ENTSCHÄRFT"));
            }

            // Telemetrie zusammenbauen
            float rel_alt = absolute_altitude - drone_alt_home;
            if (rel_alt < 0.0f) rel_alt = 0.0f;

            SensorData tx_telemetry = {};
            tx_telemetry.accel_x    = (int16_t)(imu_data.accel_x * 1000.0f);
            tx_telemetry.accel_y    = (int16_t)(imu_data.accel_y * 1000.0f);
            tx_telemetry.accel_z    = (int16_t)(imu_data.accel_z * 1000.0f);
            tx_telemetry.gyro_x     = (int16_t)(imu_data.gyro_x  * 10.0f);
            tx_telemetry.gyro_y     = (int16_t)(imu_data.gyro_y  * 10.0f);
            tx_telemetry.gyro_z     = (int16_t)(imu_data.gyro_z  * 10.0f);
            tx_telemetry.altitude   = (int16_t)(rel_alt * 100.0f);
            tx_telemetry.battery_mv = (uint16_t)smoothed_bat_mv;
            tx_telemetry.status     = (imu_ok ? 0x01 : 0x00) |
                                      (baro.isInitialized() ? 0x02 : 0x00);
            tx_telemetry.checksum   = calcSensorChecksum(tx_telemetry);

            radio.flush_tx();
            radio.writeAckPayload(1, &tx_telemetry, sizeof(SensorData));
        } else {
            radio.flush_rx();
        }
    }

    // =========================================================================
    // 5. FAILSAFE
    // =========================================================================
    if (now_ms - last_packet_time_ms > FAILSAFE_TIMEOUT_MS) {
        if (funk_verbindung_aktiv || drohne_scharf) {
            DEBUG_UART.println(F("[FAILSAFE] Verbindung verloren — Motoren gestoppt!"));
            height_calibrated = false;
        }
        funk_verbindung_aktiv = false;
        drohne_scharf         = false;
        motors.stopAll();
    }

    // =========================================================================
    // 6. MOTORSTEUERUNG STUFE 2 — Throttle direkt, kein PID
    // =========================================================================
    if (drohne_scharf && funk_verbindung_aktiv) {
        // Throttle aus richtigem Stick-Feld lesen (right_y = physisch linker Stick Y)
        float throttle = ((4095.0f - live_ctrl_data.right_y) / 4095.0f) * MOTOR_SPEED_MAX;

        // Sicherheit: Unter 5% Throttle → Motoren aus
        if (throttle < (MOTOR_SPEED_MAX * 0.05f)) {
            motors.stopAll();
        } else {
            uint16_t spd = (uint16_t)throttle;
            motors.write(spd, spd, spd, spd);
            DEBUG_UART.print(F("Motor PWM: ")); DEBUG_UART.println(spd);
        }
    } else {
        motors.stopAll();
    }

    // =========================================================================
    // 7. LED DASHBOARD
    // =========================================================================
    if (now_ms - last_blink_ms >= 250) {
        last_blink_ms = now_ms;
        blink_state   = !blink_state;
    }

    digitalWrite(LED_GREEN_PIN, drohne_scharf ? HIGH : (blink_state ? HIGH : LOW));

    if (hardware_fehler) {
        digitalWrite(LED_RED_PIN, blink_state ? HIGH : LOW);
    } else if (!funk_verbindung_aktiv) {
        digitalWrite(LED_RED_PIN, HIGH);
    } else {
        digitalWrite(LED_RED_PIN, LOW);
    }

    // =========================================================================
    // 8. UART-AUSGABE (5Hz)
    // =========================================================================
    if (now_ms - last_print_ms < 200) return;
    last_print_ms = now_ms;

    float throttle_pct = (live_ctrl_data.left_y / 4095.0f) * 100.0f;
    float rel_alt_print = absolute_altitude - drone_alt_home;
    if (rel_alt_print < 0.0f) rel_alt_print = 0.0f;

    DEBUG_UART.println(F("\n===== [DROHNE — STUFE 2 SCHUBTEST] ====="));
    DEBUG_UART.print(F("Status:    "));
    DEBUG_UART.println(drohne_scharf ? F("SCHARF") : F("ENTSCHÄRFT"));
    DEBUG_UART.print(F("Funk:      "));
    DEBUG_UART.println(funk_verbindung_aktiv ? F("AKTIV") : F("NO SIGNAL"));
    DEBUG_UART.print(F("Throttle:  "));
    DEBUG_UART.print(throttle_pct, 1);
    DEBUG_UART.println(F("%"));
    DEBUG_UART.print(F("Accel[g]:  X=")); DEBUG_UART.print(imu_data.accel_x, 2);
    DEBUG_UART.print(F(" Y="));           DEBUG_UART.print(imu_data.accel_y, 2);
    DEBUG_UART.print(F(" Z="));           DEBUG_UART.println(imu_data.accel_z, 2);
    DEBUG_UART.print(F("Gyro[°/s]: X=")); DEBUG_UART.print(imu_data.gyro_x, 2);
    DEBUG_UART.print(F(" Y="));           DEBUG_UART.print(imu_data.gyro_y, 2);
    DEBUG_UART.print(F(" Z="));           DEBUG_UART.println(imu_data.gyro_z, 2);
    DEBUG_UART.print(F("Rel.Höhe:  "));   DEBUG_UART.print(rel_alt_print, 2);
    DEBUG_UART.println(F(" m"));
    DEBUG_UART.print(F("Akku:      "));   DEBUG_UART.print(smoothed_bat_mv / 1000.0f, 2);
    DEBUG_UART.println(F(" V"));
    DEBUG_UART.println(F("========================================="));
}