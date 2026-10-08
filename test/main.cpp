#ifndef ARDUINO
#include "dxlr01.h"
#include <cstdio>
#include <cstring>
#include <unistd.h>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <port> <initial-emit: y|n>\n", argv[0]);
        return -1;
    }

    // --
    const char* port = argv[1];
    bool initialEmit = (argv[2][0] == 'y' || argv[2][0] == 'Y');
    DXLR01 dxlr (port);

    if (!dxlr.init()) {
        return -1;
    }

    // --> set the endpoint for the DXLR01 module.
    if (!dxlr.setEndpoint(DXLR01_EP(
        0x0204, 0x01, DXLR_MODE_TR
    ))) {
        fprintf(stderr, "Failed to set endpoint.\n");
        return -1;
    }

    if (!dxlr.setPower(16)) {
        fprintf(stderr, "Failed to set power.\n");
        return -1;
    }

    if (!dxlr.restart()) {
        fprintf(stderr, "Failed to restart DXLR01 module.\n");
        return -1;
    }

    // --> A and B bounce a 4-byte big-endian counter, each side adding 1.
    auto emitCounter = [&](uint32_t v) {
        const uint8_t b[4] = {
            uint8_t(v >> 24), uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)
        };

        printf("Sent     %u\n", v);
        fflush(stdout);
        return dxlr.emit(b, sizeof(b)) == sizeof(b);
    };

    // --> the peer may still be initializing and would miss the first counter.
    // --> init() may take several seconds when the module state is unknown.
    if (initialEmit) {
        sleep(10);
    }

    if (initialEmit && !emitCounter(0)) {
        fprintf(stderr, "Failed to emit.\n");
        return -1;
    }

    // --> a counter may arrive split over several reads, so accumulate 4 bytes.
    uint8_t rx[4];
    size_t have = 0;

    while (true) {
        const size_t r = dxlr.recv(rx + have, sizeof(rx) - have);
        if (r == 0) {
            continue;
        }

        have += r;
        if (have < sizeof(rx)) {
            continue;
        }

        have = 0;
        const uint32_t v = (uint32_t(rx[0]) << 24) | (uint32_t(rx[1]) << 16)
                         | (uint32_t(rx[2]) << 8) | uint32_t(rx[3]);

        printf("Received %u\n", v);
        if (!emitCounter(v + 1)) {
            fprintf(stderr, "Failed to emit.\n");
            return -1;
        }
    }

    return 0;
}

#endif // !ARDUINO
