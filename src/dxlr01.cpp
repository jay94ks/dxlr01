#include "dxlr01.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/**
 * Air settings of each profile, indexed by DXLR01_Profile. See README.md.
 */
struct DXLR_ProfileParams {
    uint8_t level, sf, cr;
};

constexpr DXLR_ProfileParams DXLR_PROFILES[] = {
    { 0, 0, 0 },    // AS_IS (unused)
    { 7, 5, 2 },    // FASTEST
    { 7, 5, 1 },    // HIGH_THROUGHPUT
    { 7, 7, 2 },    // BALANCED
    { 7, 9, 2 },    // LONG_RANGE
    { 7, 12, 4 },   // LONGEST_RANGE
};

constexpr size_t DXLR_BAUDS_COUNT = 8;

/**
 * Supported baud rates for the DXLR01 module.
 */
const uint32_t DXLR_BAUDS[DXLR_BAUDS_COUNT] = {
    115200, 57600, 38400, 19200,
    9600, 4800, 2400, 1200,
};

const uint32_t DXLR_BAUD_NUM[DXLR_BAUDS_COUNT] = {
    8, 7, 6, 5, 4, 3, 2, 1
};

/**
 * Probing timeouts. Initial connection speed does not matter, so favor reliability.
 */
constexpr int32_t DXLR_PROBE_TIMEOUT = 1500;
constexpr int DXLR_PROBE_RETRIES = 2;
constexpr useconds_t DXLR_SETTLE_US = 200 * 1000;

/**
 * Minimum pause before a command. Measured: a command sent right after
 * `AT+CHANNEL` / `AT+MAC` is answered with `ERROR=102` while the module is
 * still storing the setting; 20ms is enough, so use 50ms for margin.
 */
constexpr useconds_t DXLR_CMD_GAP_US = 50 * 1000;

/**
 * The module reboots when it leaves command mode or is reset, and announces it
 * with `Power on`. Anything sent before that banner is lost.
 */
constexpr int32_t DXLR_BOOT_TIMEOUT = 2000;
constexpr const char DXLR_REP_POWER_ON[] = "Power on\r\n";

/**
 * Maximum number of bytes the module forwards in a single packet (measured).
 * The header of the fixed point / broadcast modes counts against it.
 */
// Fixed point and broadcast modes lose data above 250 bytes including the header,
// and transparent mode gets unreliable near 264, so 250 is used everywhere.
constexpr size_t DXLR_MAX_PACKET = 250;
#ifndef DXLR_PACKET_GAP_MS
#define DXLR_PACKET_GAP_MS 210
#endif

/**
 * AT command for querying the module status.
 */
constexpr const char DXLR_CMD_AT[] = "AT\r\n";

/**
 * AT command for switching the module to a specific baudrate.
 * Command prefix; buildCmd() appends the value: the desired baudrate.
 */
constexpr const char DXLR_CMD_AT_BAUD[] = "AT+BAUD";

/**
 * AT command for resetting the module.
 */
constexpr const char DXLR_CMD_RESET[] = "AT+RESET\r\n";

/**
 * AT command for switching the module to command mode.
 */
constexpr const char DXLR_CMD_AT_CMODE[] = "+++\r\n";

/**
 * AT command for setting the transmission power of the module.
 * Command prefix; buildCmd() appends the value: the desired power level.
 */
constexpr const char DXLR_CMD_AT_POWER[] = "AT+POWE";

/**
 * Default air data rate level applied by init().
 */

/**
 * AT command for setting the air data rate level of the module.
 * Command prefix; buildCmd() appends the value: the desired level (0 ~ 7).
 */
constexpr const char DXLR_CMD_AT_HELP[] = "AT+HELP\r\n";
constexpr const char DXLR_CMD_AT_LEVEL[] = "AT+LEVEL";
constexpr const char DXLR_CMD_AT_SF[] = "AT+SF";
constexpr const char DXLR_CMD_AT_CR[] = "AT+CR";

/**
 * AT command for setting the communication channel of the module.
 * Command prefix; buildCmd() appends the value: the desired channel number.
 */
constexpr const char DXLR_CMD_AT_CHANNEL[] = "AT+CHANNEL";

/**
 * AT command for setting the MAC of the module.
 * Command prefix; buildCmd() appends the value: the desired MAC.
 */
constexpr const char DXLR_CMD_AT_MAC[] = "AT+MAC";

/**
 * AT command for setting the working mode of the module.
 * Command prefix; buildCmd() appends the value: the desired mode.
 */
constexpr const char DXLR_CMD_AT_MODE[] = "AT+MODE";

/**
 * `OK\r\n` response from the module.
 */
constexpr const char DXLR_REP_OK[] = "OK\r\n";

/**
 * Responses of DXLR_CMD_AT_CMODE.
 */
constexpr const char DXLR_REP_EXIT_AT[] = "Exit AT\r\n";
constexpr const char DXLR_REP_ENTRY_AT[] = "Entry AT\r\n";

#ifdef ARDUINO
#define DXLR_PORT_NAME ""
#else
#define DXLR_PORT_NAME _port
#endif

/**
 * Sends a command to the DXLR01 module.
 */
bool DXLR01::writeCmd(const char* cmd) {
    if (!_com.isOpen()) {
        return false;
    }

    _buf.clear();
    const size_t s = strlen(cmd);

    dxlr01::delayUs(DXLR_CMD_GAP_US);

    // --> remaining bytes to write.
    size_t rem = s;
    while (rem > 0) {
        const dxlr01::dxlr_ssize_t r = _com.write(cmd, rem);
        if (r <= 0) {
            break;
        }

        rem -= r;
        cmd += r;
    }

    return rem == 0;
}

/**
 * Reads data from the DXLR01 module into the internal buffer.
 * @param timeout The timeout for reading in milliseconds.
 * @return The number of bytes read into the buffer.
 */
size_t DXLR01::readToBuffer(int32_t timeout) {
    if (!_com.isOpen()) {
        return 0;
    }

    uint8_t buf[256];
    const dxlr01::dxlr_ssize_t r = _com.read(buf, sizeof(buf), timeout);
    if (r <= 0) {
        return 0;
    }

    return _buf.append(buf, r) ? static_cast<size_t>(r) : 0;
}

/**
 * Waits until one of the two responses is received.
 * On success, the buffer is consumed up to the end of the match.
 */
int DXLR01::waitEither(int32_t timeout, const char* a, const char* b) {
    const char* const list[2] = { a, b };
    const uint32_t begin = dxlr01::millis();

    while (true) {
        // --> pick the earliest match in the buffer.
        size_t best = static_cast<size_t>(-1), bestEnd = 0;
        int which = -1;

        for (int k = 0; k < 2; ++k) {
            if (list[k] == nullptr) {
                continue;
            }

            const size_t len = strlen(list[k]);
            for (size_t i = 0; i + len <= _buf.size() && i < best; ++i) {
                if (memcmp(&_buf[i], list[k], len) == 0) {
                    best = i;
                    bestEnd = i + len;
                    which = k;
                    break;
                }
            }
        }

        if (which >= 0) {
            _buf.eraseFront(bestEnd);
            return which;
        }

        const uint32_t elapsed = dxlr01::millis() - begin;
        if (timeout >= 0 && elapsed >= static_cast<uint32_t>(timeout)) {
            return -1;
        }

        const int32_t left = timeout < 0 ? -1 : static_cast<int32_t>(timeout - elapsed);
        readToBuffer(left);
    }
}

/**
 * Waits for the expected response (default `OK\r\n`) from the module.
 */
bool DXLR01::waitOk(int32_t timeout, const char* expected) {
    return waitEither(timeout, expected ? expected : DXLR_REP_OK, nullptr) == 0;
}

/**
 * Switches the module to the specified mode (command or data).
 * @param mode The mode to switch to (MODE_DAT or MODE_CMD).
 * @return true if the mode switch was successful, false otherwise.
 */
bool DXLR01::switchEmitMode(uint8_t mode) {
    if (_emitMode == mode) {
        return true;
    }

    // --> `+++` is a toggle, so the response tells the real resulting mode.
    // --> if our assumed state was wrong, the first toggle lands on the opposite
    // --> mode and a second toggle reaches the requested one.
    for (int i = 0; i < 2; ++i) {
        _buf.clear();

        if (!writeCmd(DXLR_CMD_AT_CMODE)) {
            return false;
        }

        const int r = waitEither(DXLR_PROBE_TIMEOUT, DXLR_REP_ENTRY_AT, DXLR_REP_EXIT_AT);
        if (r < 0) {
            return false; // --> no response: wrong baudrate or no module.
        }

        _emitMode = (r == 0) ? EMIT_CMD : EMIT_DAT;

        // --> leaving command mode reboots the module: data sent before the
        // --> banner is lost, and the banner itself must not leak into payload.
        if (_emitMode == EMIT_DAT) {
            waitOk(DXLR_BOOT_TIMEOUT, DXLR_REP_POWER_ON);
        }

        if (_emitMode == mode) {
            return true;
        }
    }

    return false;
}

/**
 * Tries to put the module into command mode at the currently open baudrate.
 */
bool DXLR01::probe() {
    _emitMode = EMIT_DAT;

    // --> `AT` would be sent over the air as payload in data mode, so only the
    // --> `+++` toggle is used: its `Entry AT` / `Exit AT` reply tells the real mode.
    // --> the first connection is not time-critical, so be patient and retry.
    for (int i = 0; i < DXLR_PROBE_RETRIES; ++i) {
        _emitMode = EMIT_DAT;

        if (switchEmitMode(EMIT_CMD)) {
            return true;
        }
    }

    _emitMode = EMIT_DAT;
    return false;
}

/**
 * Builds "<prefix><value>\r\n", or "<prefix><value>,<second>\r\n" when `second` is given.
 * The values are printed in `base` padded to `digits` digits.
 */
static void buildCmd(char* buf, size_t cap, const char* prefix, unsigned value,
                     unsigned base = 10, unsigned digits = 1, int second = -1) {
    size_t n = dxlr01::textAppend(buf, cap, 0, prefix);
    n = dxlr01::textAppendUInt(buf, cap, n, value, base, digits);

    if (second >= 0) {
        n = dxlr01::textAppend(buf, cap, n, ",");
        n = dxlr01::textAppendUInt(buf, cap, n, static_cast<unsigned>(second), base, digits);
    }

    dxlr01::textAppend(buf, cap, n, "\r\n");
}

/**
 * Converts a baudrate to its corresponding numeric code.
 * @param baudrate The baudrate to convert.
 * @return The numeric code corresponding to the baudrate, or 0 if unknown.
 */
uint32_t DXLR01_BaudrateToNum(uint32_t baudrate) {
    for (size_t i = 0; i < DXLR_BAUDS_COUNT; ++i) {
        if (DXLR_BAUDS[i] == baudrate) {
            return DXLR_BAUD_NUM[i];
        }
    }

    return 0; // --> unknown baudrate.
}

/**
 * Switches the module to the specified baudrate.
 * @param baudrate The baudrate to switch to.
 * @return true if the baudrate switch was successful, false otherwise.
 */
bool DXLR01::switchBaudrate(uint32_t baudrate) {
    if (_baudrate == baudrate) {
        return true;
    }

    uint32_t n = DXLR01_BaudrateToNum(baudrate);
    if (n == 0) {
        return false; // --> unknown baudrate.
    }

    // --> try to switch the baudrate by sending the appropriate command.
    char cmd[32] = {0, };

    buildCmd(cmd, sizeof(cmd), DXLR_CMD_AT_BAUD, n);
    if (!writeCmd(cmd) || !waitOk(1000)) {
        return false;
    }

    writeCmd(DXLR_CMD_RESET);
    _com.flush();

    _baudrate = baudrate;
    _com.close();

    // --> try to reopen the serial port with the new baudrate.
    if (!_com.open(DXLR_PORT_NAME, baudrate)) {
        return false;
    }

    // --> the module is ready once it announces itself at the new baudrate.
    _buf.clear();

    if (!waitOk(DXLR_BOOT_TIMEOUT, DXLR_REP_POWER_ON)) {
        return false;
    }

    _buf.clear();
    return true;
}

/**
 * Internal initialization routine for the DXLR01 module.
 * @return true if the module was successfully initialized, false otherwise.
 */
bool DXLR01::onInit() {
    // --> switch to the fastest baudrate allowed.
    uint32_t target = DXLR_BAUDS[DXLR_BAUDS_COUNT - 1];
    for (uint32_t baud : DXLR_BAUDS) {
        if (baud <= _maxBaud) {
            target = baud;
            break;
        }
    }

    if (_baudrate != target && !switchBaudrate(target)) {
        return false;
    }

    // --> keep the settings stored in the module instead of overwriting them.
    if (!loadSettings()) {
        return false;
    }

    // --> leave command mode, so the module is ready for data.
    return switchEmitMode(EMIT_DAT);
}

/**
 * Values reported by `AT+HELP`, filled in line by line.
 */
struct DXLR_Report {
    enum { MODE, LEVEL, CHAN, MAC, SF, CR, POWER, COUNT };

    unsigned mode, level, chan, mac0, mac1, sf, cr, power;
    unsigned seen = 0; // --> bit per field already parsed (the first one wins).

    bool complete() const { return seen == (1u << COUNT) - 1; }

    /**
     * Parses one line, if it carries a field not yet seen.
     */
    void feed(const char* line) {
        const char* p;

        if ((p = strstr(line, "MODE:")) != nullptr) {
            take(MODE, p + 5, 10, mode);
        } else if ((p = strstr(line, "LEVEL:")) != nullptr) {
            take(LEVEL, p + 6, 10, level);
        } else if ((p = strstr(line, "Frequency:")) != nullptr) {
            // --> "<n>MHz ... >> <chan hex>"
            if ((p = strstr(p, ">>")) != nullptr) {
                take(CHAN, p + 2, 16, chan);
            }
        } else if ((p = strstr(line, "MAC:")) != nullptr) {
            p += 4;
            unsigned a, b;
            if (!(seen & (1u << MAC)) && dxlr01::textScanUInt(p, 16, a) && *p++ == ',' &&
                dxlr01::textScanUInt(p, 16, b)) {
                mac0 = a;
                mac1 = b;
                seen |= 1u << MAC;
            }
        } else if ((p = strstr(line, "Spreading Factor:")) != nullptr) {
            take(SF, p + 17, 10, sf);
        } else if ((p = strstr(line, "Coding rate:")) != nullptr) {
            take(CR, p + 12, 10, cr);
        } else if ((p = strstr(line, "Power:")) != nullptr) {
            take(POWER, p + 6, 10, power);
        }
    }

private:
    void take(int field, const char* p, unsigned base, unsigned& out) {
        unsigned v;
        if (!(seen & (1u << field)) && dxlr01::textScanUInt(p, base, v)) {
            out = v;
            seen |= 1u << field;
        }
    }
};

/**
 * Reads the current settings of the module with `AT+HELP` and caches them.
 * The report is parsed line by line while it arrives, so only one line is
 * ever copied out of the receive buffer.
 */
bool DXLR01::loadSettings() {
    if (!switchEmitMode(EMIT_CMD) || !writeCmd(DXLR_CMD_AT_HELP)) {
        return false;
    }

    DXLR_Report rep;
    const uint32_t begin = dxlr01::millis();

    while (!rep.complete() && dxlr01::millis() - begin < DXLR_BOOT_TIMEOUT) {
        // --> consume every complete line received so far.
        while (true) {
            const uint8_t* nl = static_cast<const uint8_t*>(memchr(_buf.data(), '\n', _buf.size()));
            if (nl == nullptr) {
                break;
            }

            const size_t len = static_cast<size_t>(nl - _buf.data()) + 1;
            char line[128];
            const size_t n = len < sizeof(line) ? len : sizeof(line) - 1;

            memcpy(line, _buf.data(), n);
            line[n] = '\0';
            _buf.eraseFront(len);
            rep.feed(line);
        }

        if (!rep.complete()) {
            readToBuffer(100);
        }
    }

    _buf.clear();

    if (!rep.complete()) {
        return false;
    }

    const unsigned mode = rep.mode, level = rep.level, sf = rep.sf, cr = rep.cr;
    const unsigned power = rep.power, chan = rep.chan, mac0 = rep.mac0, mac1 = rep.mac1;

    if (mode > DXLR_MODE_BR) {
        return false;
    }

    _level = level;
    _sf = sf;
    _cr = cr;
    _power = power;
    _endpoint = DXLR01_EP((mac0 << 8) | mac1, chan, mode);
    return true;
}

/**
 * Initializes the DXLR01 module.
 * @param profile The communication profile to apply during initialization.
 * @return true if initialization was successful, false otherwise.
 */
bool DXLR01::init(DXLR01_Profile profile) {
    if (_com.isOpen()) {
        return false; // --> already open.
    }

    // --> the configured baudrate goes first, then every supported one.
    uint32_t cands[1 + DXLR_BAUDS_COUNT];
    size_t n = 0;

    if (_baudrate <= _maxBaud) {
        cands[n++] = _baudrate;
    }

    for (uint32_t baud : DXLR_BAUDS) {
        if (baud != _baudrate && baud <= _maxBaud) {
            cands[n++] = baud;
        }
    }

    for (size_t i = 0; i < n; ++i) {
        _com.close();

        if (!_com.open(DXLR_PORT_NAME, cands[i])) {
            continue;
        }

        // --> let the line settle (e.g. USB adapters) before probing.
        dxlr01::delayUs(DXLR_SETTLE_US);

        if (probe()) {
            _baudrate = cands[i];

            if (!onInit()) {
                _com.close();
                return false;
            }

            if (!setProfile(profile)) {
                _com.close();
                return false;
            }

            return true;
        }
    }

    _com.close();
    return false;
}

/**
 * Deinitializes the DXLR01 module, closing the serial port.
 * @return true if the module was successfully deinitialized, false otherwise.
 */
bool DXLR01::deinit() {
    if (!_com.isOpen()) {
        return false;
    }

    _com.close();
    return true;
}

/**
 * Sets the communication profile of the DXLR01 module.
 * @param profile The communication profile to apply.
 * @return true if the profile was successfully set, false otherwise.
 */
bool DXLR01::setProfile(DXLR01_Profile profile) {
    if (!_com.isOpen()) {
        return false;
    }

    if (profile == DXLR_PROFILE_AS_IS) {
        return true;
    }

    if (static_cast<size_t>(profile) >= sizeof(DXLR_PROFILES) / sizeof(DXLR_PROFILES[0])) {
        return false;
    }

    const DXLR_ProfileParams& p = DXLR_PROFILES[profile];
    const bool changed = _level != p.level || _sf != p.sf || _cr != p.cr;

    if (_level != p.level && !setLevel(p.level)) {
        return false;
    }

    if (_sf != p.sf && !setSpreadFactor(p.sf)) {
        return false;
    }

    if (_cr != p.cr && !setCodingRate(p.cr)) {
        return false;
    }

    // --> settings are applied by a restart, which also returns to data mode.
    return changed ? restart() : switchEmitMode(EMIT_DAT);
}

/**
 * Sets the transmission power of the DXLR01 module.
 * @param db The power level in dB (0dB ~ 22dB).
 * @return true if the power was successfully set, false otherwise.
 */
bool DXLR01::setPower(uint8_t db) {
    if (!_com.isOpen()) {
        return false;
    }

    if (db > 22) {
        return false;
    }

    if (!switchEmitMode(EMIT_CMD)) {
        return false;
    }

    char cmd[32] = {0, };
    buildCmd(cmd, sizeof(cmd), DXLR_CMD_AT_POWER, db);

    if (!writeCmd(cmd) || !waitOk(1000)) {
        return false;
    }

    _power = db;
    return true;
}

/**
 * Sets the air data rate level of the DXLR01 module.
 * @param level The level (0 ~ 7).
 * @return true if the level was successfully set, false otherwise.
 */
bool DXLR01::setLevel(uint8_t level) {
    if (!_com.isOpen()) {
        return false;
    }

    if (level > 7) {
        return false;
    }

    if (!switchEmitMode(EMIT_CMD)) {
        return false;
    }

    char cmd[32] = {0, };
    buildCmd(cmd, sizeof(cmd), DXLR_CMD_AT_LEVEL, level);

    if (!writeCmd(cmd) || !waitOk(1000)) {
        return false;
    }

    _level = level;
    return true;
}

/**
 * Sets the spreading factor (5 ~ 12).
 */
bool DXLR01::setSpreadFactor(uint8_t sf) {
    if (!_com.isOpen() || sf < 5 || sf > 12 || !switchEmitMode(EMIT_CMD)) {
        return false;
    }

    char cmd[32] = {0, };
    buildCmd(cmd, sizeof(cmd), DXLR_CMD_AT_SF, sf);

    if (!writeCmd(cmd) || !waitOk(1000)) {
        return false;
    }

    _sf = sf;
    return true;
}

/**
 * Sets the coding rate (1 ~ 4).
 */
bool DXLR01::setCodingRate(uint8_t cr) {
    if (!_com.isOpen() || cr < 1 || cr > 4 || !switchEmitMode(EMIT_CMD)) {
        return false;
    }

    char cmd[32] = {0, };
    buildCmd(cmd, sizeof(cmd), DXLR_CMD_AT_CR, cr);

    if (!writeCmd(cmd) || !waitOk(1000)) {
        return false;
    }

    _cr = cr;
    return true;
}

/**
 * Restarts the DXLR01 module.
 * @return true if the module was successfully restarted, false otherwise.
 */
bool DXLR01::restart() {
    if (!_com.isOpen()) {
        return false;
    }

    // --> `AT+RESET` is a command, not payload: make sure the module is in command mode.
    if (!switchEmitMode(EMIT_CMD) || !writeCmd(DXLR_CMD_RESET)) {
        _com.close();
        return false;
    }

    _com.flush();

    // --> wait for the "Power on" banner and drop it, so it is not mistaken for payload.
    _buf.clear();
    waitOk(DXLR_BOOT_TIMEOUT, DXLR_REP_POWER_ON);

    _buf.clear();
    _emitMode = EMIT_DAT;
    return true;
}

/**
 * Sets the endpoint of the DXLR01 module.
 * @param endpoint The endpoint to set.
 * @return true if the endpoint was successfully set, false otherwise.
 */
bool DXLR01::setEndpoint(uint32_t endpoint) {
    if (!_com.isOpen()) {
        return false;
    }

    if (endpoint == DXLR01_EP_INVAL) {
        return false;
    }

    if (endpoint == _endpoint) {
        return true;
    }

    const uint16_t mac = DXLR01_MAC(endpoint);
    const uint8_t chan = DXLR01_CHAN(endpoint);
    const uint8_t mode = DXLR01_MODE(endpoint);

    if (chan > 0x1e) {
        return false; // --> channel number exceeds the maximum allowed value (0x1e).
    }

    if (mode > DXLR_MODE_BR) {
        return false; // --> mode exceeds the maximum allowed value (DXLR_MODE_BR).
    }

    if (!switchEmitMode(EMIT_CMD)) {
        return false;
    }

    char cmdMac[32] = {0, };
    char cmdChan[32] = {0, };
    char cmdMode[32] = {0, };

    // --> prepare the AT commands for setting the MAC, channel, and mode.
    buildCmd(cmdMac, sizeof(cmdMac), DXLR_CMD_AT_MAC, (mac >> 8) & 0xff, 16, 2, mac & 0xff);
    buildCmd(cmdChan, sizeof(cmdChan), DXLR_CMD_AT_CHANNEL, chan, 16, 2);
    buildCmd(cmdMode, sizeof(cmdMode), DXLR_CMD_AT_MODE, mode);

    if (!writeCmd(cmdMac) || !waitOk(1000)) {
        return false;
    }

    if (!writeCmd(cmdChan) || !waitOk(1000)) {
        return false;
    }

    if (!writeCmd(cmdMode) || !waitOk(1000)) {
        return false;
    }

    // --> update the internal endpoint state after successfully setting the endpoint.
    _endpoint = endpoint;
    return restart();
}

/**
 * Sends data to a specific endpoint through the DXLR01 module.
 * @param data Pointer to the data to send.
 * @param len Length of the data to send.
 * @param ep The endpoint to send data to.
 * @return The number of bytes successfully sent.
 */
size_t DXLR01::emit(const uint8_t* data, size_t len, uint32_t ep) {
    if (!_com.isOpen()) {
        return 0;
    }

    // --> extract the mode, channel, and MAC address from the target endpoint.
    const uint8_t epMode = DXLR01_MODE(ep);
    const uint8_t epChan = DXLR01_CHAN(ep);
    const uint16_t epMac = DXLR01_MAC(ep);

    // --> extract the mode, channel, and MAC address from the current endpoint.
    const uint8_t toMode = DXLR01_MODE(_endpoint);
    uint8_t toChan = DXLR01_CHAN(_endpoint);
    uint16_t toMac = DXLR01_MAC(_endpoint);

    // --> prepare the header buffer for the outgoing data.
    uint8_t bufHdr[4] = {0, };
    uint8_t lenHdr = 0;

    // --> check if the mode of the current endpoint differs from the target endpoint.
    if (toMode != epMode && ep != DXLR01_EP_DEFAULT) {
        return 0; // --> mode mismatch, cannot emit data.
    }

    // --> prepare the header buffer based on the current mode.
    switch (toMode) {
        case DXLR_MODE_TR:  // --> to same endpoint.
            if (ep != DXLR01_EP_DEFAULT) {
                if (epMac != toMac) {
                    return 0; // --> MAC address mismatch, cannot emit data.
                }

                if (epChan != toChan) {
                    return 0; // --> channel mismatch, cannot emit data.
                }
            }
            break;

        case DXLR_MODE_BR:  // --> to specific channel. (dont care `MAC`)
            if (ep != DXLR01_EP_DEFAULT) {
                toChan = epChan;
            }

            bufHdr[lenHdr++] = toChan;
            break;

        case DXLR_MODE_FP:  // --> to specific channel + mac.
            if (ep != DXLR01_EP_DEFAULT) {
                toChan = epChan;
                toMac = epMac;
            }

            bufHdr[lenHdr++] = (toMac >> 8) & 0xFF;
            bufHdr[lenHdr++] = toMac & 0xFF;
            bufHdr[lenHdr++] = toChan;
            break;

        default:
            return 0;
    }

    if (!switchEmitMode(EMIT_DAT)) {
        return 0; // --> failed to switch mode.
    }

    // --> one packet carries at most DXLR_MAX_PACKET bytes including the header,
    // --> so split longer data and repeat the header for every packet.
    // --> header and payload go out in a single write, or the module may split them.
    const size_t room = DXLR_MAX_PACKET - lenHdr;
    uint8_t pkt[DXLR_MAX_PACKET];
    size_t sent = 0;

    memcpy(pkt, bufHdr, lenHdr);
    while (sent < len) {
        const size_t n = (len - sent) < room ? (len - sent) : room;
        memcpy(pkt + lenHdr, data + sent, n);

        // --> pace consecutive packets, or the module's buffer overflows (measured at SF5).
        if (_sf == 5 && _hasLastPacket) {
            const uint32_t since = dxlr01::millis() - _lastPacketMs;
            if (since < DXLR_PACKET_GAP_MS) {
                dxlr01::delayUs((DXLR_PACKET_GAP_MS - since) * 1000u);
            }
        }

        const bool ok = _com.write(pkt, lenHdr + n) == static_cast<dxlr01::dxlr_ssize_t>(lenHdr + n);
        _lastPacketMs = dxlr01::millis();
        _hasLastPacket = true;

        if (!ok) {
            break;
        }

        sent += n;
    }

    return sent;
}

/**
 * Queues data for a batched transmission. Small messages are merged into
 * one packet, which is much faster than sending each of them on its own.
 * The queue is sent when it is full, when the endpoint changes, or on flush().
 * @return The number of bytes accepted (len, or 0 on failure).
 */
size_t DXLR01::push(const uint8_t* data, size_t len, uint32_t ep) {
    if (!_com.isOpen()) {
        return 0;
    }

    // --> a queue only holds data for a single endpoint.
    if (!_queue.empty() && _queueEp != ep && !flush()) {
        return 0;
    }

    _queueEp = ep;

    size_t done = 0;
    while (done < len) {
        const size_t room = DXLR_QUEUE_SIZE - _queue.size();
        const size_t n = (len - done) < room ? (len - done) : room;

        if (!_queue.append(data + done, n)) {
            return done;
        }
        done += n;

        if (_queue.size() >= DXLR_QUEUE_SIZE && !flush()) {
            return done - n;
        }
    }

    return done;
}

/**
 * Sends all queued data.
 * @return true if everything was sent (or nothing was queued).
 */
bool DXLR01::flush() {
    if (_queue.empty()) {
        return true;
    }

    const size_t len = _queue.size();
    const size_t sent = emit(_queue.data(), len, _queueEp);

    if (sent == len) {
        _queue.clear();
        return true;
    }

    // --> keep what was not sent.
    _queue.eraseFront(sent);
    return false;
}

/**
 * Receives data from the DXLR01 module into the provided buffer.
 * @param data Pointer to the buffer to store received data.
 * @param len Maximum length of data to receive.
 * @param timeout The timeout for receiving in milliseconds.
 * @return The number of bytes successfully received.
 */
size_t DXLR01::recv(uint8_t* data, size_t len, int32_t timeout) {
    if (!_com.isOpen()) {
        return 0;
    }

    if (!switchEmitMode(EMIT_DAT)) {
        return 0; // --> failed to switch mode.
    }

    const dxlr01::dxlr_ssize_t r = _com.read(data, len, timeout);
    return r > 0 ? static_cast<size_t>(r) : 0;
}