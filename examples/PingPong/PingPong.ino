// DX-LR01 ping-pong: two boards bounce a 4-byte big-endian counter.
// Set INITIATOR to true on exactly one of the boards.
//
// Wiring (3.3V logic): module TX -> board RX, module RX -> board TX, GND, VCC.
// ESP32: Serial2 on RX=16, TX=17. Other boards: use a spare hardware serial (e.g. Serial1).
#include <dxlr01.h>

static const bool INITIATOR = false;

#if defined(ESP32)
DXLR01 radio(Serial2, 16, 17);
#else
DXLR01 radio(Serial1);
#endif

static uint8_t rx[4];
static size_t have = 0;

static bool emitCounter(uint32_t v) {
    const uint8_t b[4] = { uint8_t(v >> 24), uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v) };
    Serial.print("Sent ");
    Serial.println(v);
    return radio.emit(b, sizeof(b)) == sizeof(b);
}

void setup() {
    Serial.begin(115200);

    // --> init() detects the baudrate and reads the module settings (takes a few seconds).
    if (!radio.init(DXLR_PROFILE_FASTEST)) {
        Serial.println("init failed");
        while (true) { delay(1000); }
    }

    if (!radio.setEndpoint(DXLR01_EP(0x0204, 0x01, DXLR_MODE_TR)) || !radio.restart()) {
        Serial.println("setup failed");
        while (true) { delay(1000); }
    }

    if (INITIATOR) {
        delay(10000); // --> let the peer finish its init().
        emitCounter(0);
    }
}

void loop() {
    const size_t r = radio.recv(rx + have, sizeof(rx) - have, 100);
    have += r;

    if (have == sizeof(rx)) {
        have = 0;
        const uint32_t v = (uint32_t(rx[0]) << 24) | (uint32_t(rx[1]) << 16)
                         | (uint32_t(rx[2]) << 8) | uint32_t(rx[3]);

        Serial.print("Received ");
        Serial.println(v);
        emitCounter(v + 1);
    }
}
