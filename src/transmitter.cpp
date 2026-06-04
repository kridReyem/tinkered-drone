#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include "config.h"
#include "drivers/ssd1306_driver.h"
#include "drivers/piezo_buzzer.h"

PiezoBuzzer buzzer(BUZZER_PIN);  // PA15

SSD1306Driver oled(&Wire);

RF24 radio(NRF24_CE, NRF24_CSN);

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------
uint8_t calcControllerChecksum(const ControllerData& c) {
    uint8_t cs = 0;
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&c);
    for (size_t i = 0; i < offsetof(ControllerData, checksum); i++) cs ^= ptr[i];
    return cs;
}

bool init_funk_tx() {
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
    radio.setRetries(1, 5);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.openWritingPipe(CONTROLLER_ADDRESS);
    return true;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    DEBUG_UART.begin(115200);
    delay(1000);
    DEBUG_UART.println(F("\n===================================="));
    DEBUG_UART.println(F("[CONTROLLER] Starte System..."));
    DEBUG_UART.println(F("===================================="));

    analogReadResolution(12);

    pinMode(LEFT_X_PIN,   INPUT_ANALOG);
    pinMode(LEFT_Y_PIN,   INPUT_ANALOG);
    pinMode(RIGHT_X_PIN,  INPUT_ANALOG);
    pinMode(RIGHT_Y_PIN,  INPUT_ANALOG);
    pinMode(LEFT_SW_PIN,  INPUT_PULLUP);
    pinMode(RIGHT_SW_PIN, INPUT_PULLUP);

    // --- I2C & OLED ---
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();

    DEBUG_UART.println(F("[I2C] Scanne Bus (PB6/PB7)..."));
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            DEBUG_UART.print(F("  Gefunden: 0x"));
            DEBUG_UART.println(addr, HEX);
        }
    }

    oled.init(OLED_I2C_ADDR);
    oled.printAt(0, 0, "Controller OK");
    oled.sendBuffer();

    buzzer.init();
    buzzer.playStartupMelody();

    // --- NRF24 ---
    DEBUG_UART.print(F("[NRF24] Initialisiere TX-Modus... "));
    if (!init_funk_tx()) {
        DEBUG_UART.println(F("FAILED! Halting..."));
        while (true) delay(1000);
    }
    DEBUG_UART.println(F("OK!"));
    DEBUG_UART.println(F("====================================\n"));
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
    static uint32_t last_send_ms  = 0;
    static uint32_t last_print_ms = 0;
    uint32_t now_ms = millis();

    ControllerData tx_packet = {};

    analogRead(LEFT_X_PIN);  delayMicroseconds(5);
    tx_packet.left_x  = analogRead(LEFT_X_PIN);
    analogRead(LEFT_Y_PIN);  delayMicroseconds(5);
    tx_packet.left_y  = analogRead(LEFT_Y_PIN);
    analogRead(RIGHT_X_PIN); delayMicroseconds(5);
    tx_packet.right_x = analogRead(RIGHT_X_PIN);
    analogRead(RIGHT_Y_PIN); delayMicroseconds(5);
    tx_packet.right_y = analogRead(RIGHT_Y_PIN);

    tx_packet.left_click    = (digitalRead(LEFT_SW_PIN)  == LOW) ? 1 : 0;
    tx_packet.right_click   = (digitalRead(RIGHT_SW_PIN) == LOW) ? 1 : 0;
    tx_packet.battery_level = 100;
    tx_packet.checksum      = calcControllerChecksum(tx_packet);

    static float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, alt = 0;
    static bool funk_verbunden = false;

    if (now_ms - last_send_ms >= SEND_INTERVAL_MS) {
        last_send_ms   = now_ms;
        funk_verbunden = radio.write(&tx_packet, sizeof(ControllerData));

        if (funk_verbunden && radio.isAckPayloadAvailable()) {
            SensorData rx_telemetry;
            radio.read(&rx_telemetry, sizeof(SensorData));

            uint8_t cs = 0;
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&rx_telemetry);
            for (size_t i = 0; i < offsetof(SensorData, checksum); i++) cs ^= ptr[i];

            if (cs == rx_telemetry.checksum) {
                ax  = rx_telemetry.accel_x  / 1000.0f;
                ay  = rx_telemetry.accel_y  / 1000.0f;
                az  = rx_telemetry.accel_z  / 1000.0f;
                gx  = rx_telemetry.gyro_x   / 10.0f;
                gy  = rx_telemetry.gyro_y   / 10.0f;
                gz  = rx_telemetry.gyro_z   / 10.0f;
                alt = rx_telemetry.altitude / 100.0f;
            }
        }
    }

    if (now_ms - last_print_ms >= 200) {
        last_print_ms = now_ms;

        DEBUG_UART.println(F("================ [LOKALE CONTROLLER-WERTE] ================"));
        DEBUG_UART.print(F("LINKER  Stick: X=")); DEBUG_UART.print(tx_packet.left_x);
        DEBUG_UART.print(F(" | Y="));             DEBUG_UART.print(tx_packet.left_y);
        DEBUG_UART.print(F(" | Klick="));         DEBUG_UART.println(tx_packet.left_click);
        DEBUG_UART.print(F("RECHTER Stick: X=")); DEBUG_UART.print(tx_packet.right_x);
        DEBUG_UART.print(F(" | Y="));             DEBUG_UART.print(tx_packet.right_y);
        DEBUG_UART.print(F(" | Klick="));         DEBUG_UART.println(tx_packet.right_click);

        DEBUG_UART.println(F("---------------- [TELEMETRIE VON DROHNE] ----------------"));
        if (funk_verbunden) {
            DEBUG_UART.print(F("Accel [g]:  X=")); DEBUG_UART.print(ax, 2);
            DEBUG_UART.print(F(" Y="));            DEBUG_UART.print(ay, 2);
            DEBUG_UART.print(F(" Z="));            DEBUG_UART.println(az, 2);
            DEBUG_UART.print(F("Gyro [°/s]: X=")); DEBUG_UART.print(gx, 1);
            DEBUG_UART.print(F(" Y="));            DEBUG_UART.print(gy, 1);
            DEBUG_UART.print(F(" Z="));            DEBUG_UART.println(gz, 1);
            DEBUG_UART.print(F("Höhe [m]:   "));   DEBUG_UART.println(alt, 2);
        } else {
            DEBUG_UART.println(F("[FUNK] Keine Verbindung..."));
        }
        DEBUG_UART.println(F("=========================================================\n"));
    }
}