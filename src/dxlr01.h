#pragma once
#include "dxlr01_platform.h"

/**
 * Represents an endpoint in the DXLR01 network,
 * typically a combination of channel and MAC address.
 */
using DXLR01_Endpoint = uint32_t;

/**
 * Communication modes of the DXLR01 module.
 */
enum DXLR01_Mode {
    /**
     * Transparent mode.
     */
    DXLR_MODE_TR = 0,

    /**
     * Fixed point mode.
     */
    DXLR_MODE_FP,

    /**
     * Broadcast mode.
     */
    DXLR_MODE_BR,
};

/**
 * Communication profiles of the DXLR01 module. All of them were measured on
 * real modules with level 7 (see README.md). Both ends of a link
 * must use the same profile.
 */
enum DXLR01_Profile {
    /** Keeps the settings stored in the module. */
    DXLR_PROFILE_AS_IS = 0,

    /** SF5/CR2: about 112 ms one way for small packets, about 870 B/s with 240 B packets. */
    DXLR_PROFILE_FASTEST,

    /** SF5/CR1: highest air rate (15625 bps), about 955 B/s with 240 B packets and the same latency as CR2. Less error correction. */
    DXLR_PROFILE_HIGH_THROUGHPUT,

    /** SF7/CR2: about 450 B/s with 240 B packets, 135 ms latency. Longer range than SF5. */
    DXLR_PROFILE_BALANCED,

    /** SF9/CR2: about 170 B/s, 210 ms latency. */
    DXLR_PROFILE_LONG_RANGE,

    /** SF12/CR4: about 24 B/s, about 1 s latency. Slowest, most robust. */
    DXLR_PROFILE_LONGEST_RANGE,
};

/**
 * Payload size of one batched packet. A packet carries at most 250 bytes
 * including the header, and the largest header (fixed point mode) takes 3.
 */
constexpr size_t DXLR_QUEUE_SIZE = 247;

/**
 * Constructs a DXLR01_Endpoint from the given MAC address, channel, and mode.
 * @param mac The MAC address.
 * @param chan The channel number.
 * @param mode The working mode.
 * @return The constructed DXLR01_Endpoint.
 */
#define DXLR01_EP(mac, chan, mode) ((((mode) & 0x03) << 24) | (((mac) & 0xFFFF) << 8) | ((chan) & 0xFF))

/**
 * Macros to extract the MAC address and channel from a DXLR01_Endpoint.
 */
#define DXLR01_MAC(ep) (((ep) >> 8) & 0xFFFF)
#define DXLR01_CHAN(ep) ((ep) & 0xFF)

/**
 * Extracts the working mode from a DXLR01_Endpoint.
 */
#define DXLR01_MODE(ep) (((ep) >> 24) & 0x03)

/* Known endpoint values. */
enum {
    DXLR01_EP_INVAL = 0xFFFFFFFF,
    DXLR01_EP_DEFAULT = DXLR01_EP_INVAL,
};

/**
 * DX-LR01 module driver.
 */
class DXLR01 {
private:
    enum {
        EMIT_DAT = 0,
        EMIT_CMD,
    };

public:

private:
    dxlr01::SerialPort _com;
#ifndef ARDUINO
    char _port[128];
#endif

    uint32_t _baudrate;
    uint32_t _maxBaud = 0xFFFFFFFF;
    uint32_t _endpoint;
    uint8_t _level = 0;
    uint8_t _sf = 0;
    uint8_t _cr = 0;
    uint8_t _power = 0;
    uint32_t _lastPacketMs = 0;
    bool _hasLastPacket = false;
    uint8_t _emitMode;
    dxlr01::DXLR01_Bytes _queue;
    uint32_t _queueEp = DXLR01_EP_DEFAULT;

    dxlr01::DXLR01_Bytes _buf;

public:
    /**
     * Caps the baudrate used by init() (default: no limit).
     * Call before init().
     */
    void setBaudLimit(uint32_t maxBaudrate) { _maxBaud = maxBaudrate; }

#ifdef ARDUINO
    /**
     * Constructor for the DXLR01 module.
     * @param serial The HardwareSerial to which the module is connected.
     *               The driver calls begin()/end() on it, so do not begin() it yourself.
     */
    explicit DXLR01(HardwareSerial& serial) : _com(serial) {
        init_();
    }

#if defined(DXLR01_PIN_SELECT)
    /**
     * Constructor for 32-bit boards (ESP32, RP2040) where the UART pins are selectable.
     */
    DXLR01(HardwareSerial& serial, int8_t rxPin, int8_t txPin) : _com(serial, rxPin, txPin) {
        init_();
    }
#endif

#ifdef DXLR01_HAS_SOFTWARE_SERIAL
    /**
     * Constructor for a module connected through SoftwareSerial.
     * Software serial may be unreliable at high baudrates; init() still uses the
     * fastest one the module supports. Use setBaudLimit() to cap it if needed.
     * On ESP32/ESP8266 create the SoftwareSerial with its pins beforehand.
     */
    explicit DXLR01(SoftwareSerial& serial) : _com(serial) {
        init_();
    }
#endif
#else
    /**
     * Constructor for the DXLR01 module.
     * @param port The serial port to which the module is connected.
     */
    explicit DXLR01(const char* port) {
        strncpy(_port, port, sizeof(_port) - 1);
        _port[sizeof(_port) - 1] = '\0';
        init_();
    }
#endif

    /**
     * Destructor for the DXLR01 module.
     */
    ~DXLR01() {
        _com.close();
    }

private:
    void init_() {
        _baudrate = 9600;
        _endpoint = DXLR01_EP_INVAL;
        _emitMode = EMIT_DAT;
    }

    /**
     * Sends a command to the DXLR01 module.
     * @param cmd The command string to send.
     * @return true if the command was successfully sent, false otherwise.
     */
    bool writeCmd(const char* cmd);

    /**
     * Reads data from the DXLR01 module into the internal buffer.
     * @param timeout The timeout for reading in milliseconds.
     * @return The number of bytes read into the buffer.
     */
    size_t readToBuffer(int32_t timeout = -1);

    /**
     * Waits for an `OK` response from the DXLR01 module.
     * @param timeout The timeout for waiting in milliseconds.
     * @return true if `OK` was received within the timeout, false otherwise.
     */
    bool waitOk(int32_t timeout = 1000, const char* expected = nullptr);

    /**
     * Waits until one of the two responses is received.
     * @return 0 if `a` was received, 1 if `b` was received, -1 on timeout.
     */
    int waitEither(int32_t timeout, const char* a, const char* b);

    /**
     * Switches the module to the specified mode.
     * @param mode The mode to switch to (MODE_DAT or MODE_CMD).
     * @return true if the mode switch was successful, false otherwise.
     */
    bool switchEmitMode(uint8_t mode);

    /**
     * Tries to put the module into command mode at the currently open baudrate.
     * @return true if the module answered and is now in command mode.
     */
    bool probe();

    /**
     * Switches the module to the specified baudrate.
     * @param baudrate The baudrate to switch to.
     * @return true if the baudrate switch was successful, false otherwise.
     */
    bool switchBaudrate(uint32_t baudrate);

    /**
     * Internal initialization routine for the DXLR01 module.
     * @return true if the module was successfully initialized, false otherwise.
     */
    bool onInit();

    /**
     * Reads the module's current settings (`AT+HELP`) into the cached members.
     */
    bool loadSettings();

public:
    /**
     * Initializes the DXLR01 module.
     * The configured baudrate is tried first, then every rate in DXLR_BAUDS.
     * On success `_baudrate` holds the detected rate, and the settings stored in
     * the module (level, SF, CR, power, endpoint) are read and kept as they are.
     * @param profile The communication profile to apply during initialization.
     * @return true if initialization was successful, false otherwise.
     */
    bool init(DXLR01_Profile profile = DXLR_PROFILE_AS_IS);

    /**
     * Deinitializes the DXLR01 module, closing the serial port.
     * @return true if the module was successfully deinitialized, false otherwise.
     */
    bool deinit();

    /**
     * Sets the communication profile (level, SF, CR) of the DXLR01 module.
     * Only the settings that differ are written, and the module restarts only
     * when something changed.
     * @param profile The communication profile to apply.
     * @return true if the profile was successfully set, false otherwise.
     */
    bool setProfile(DXLR01_Profile profile);

    /**
     * Sets the transmission power of the DXLR01 module.
     * @param db The power level in dB (0dB ~ 22dB).
     * @return true if the power was successfully set, false otherwise.
     */
    bool setPower(uint8_t db);

    /**
     * Sets the air data rate level of the DXLR01 module.
     * Both ends of a link must use the same level.
     * @param level The level (0 ~ 7). Higher is faster but shorter range.
     * @return true if the level was successfully set, false otherwise.
     * @note Applied after restart().
     */
    bool setLevel(uint8_t level);

    /**
     * Sets the spreading factor (5 ~ 12). Lower is faster but shorter range.
     * Both ends of a link must use the same value.
     * @note Applied after restart().
     */
    bool setSpreadFactor(uint8_t sf);

    /**
     * Sets the coding rate (1 ~ 4). Lower is faster but less robust.
     * Both ends of a link must use the same value.
     * @note Applied after restart().
     */
    bool setCodingRate(uint8_t cr);

    /**
     * Restarts the DXLR01 module and apply all pending settings.
     * @return true if the module was successfully restarted, false otherwise.
     * @note This will close and reopen the serial port.
     * When this fails, the serial port will be closed and the module may need to be reinitialized.
     */
    bool restart();

    /** Cached settings, read by init() and updated by the setters. */
    inline uint8_t getLevel() const { return _level; }
    inline uint8_t getSpreadFactor() const { return _sf; }
    inline uint8_t getCodingRate() const { return _cr; }
    inline uint8_t getPower() const { return _power; }
    inline uint32_t getBaudrate() const { return _baudrate; }

    /**
     * Sets the current endpoint of the DXLR01 module.
     * @param endpoint The endpoint to set.
     * @return true if the endpoint was successfully set, false otherwise.
     */
    bool setEndpoint(uint32_t endpoint);

    /**
     * Gets the current endpoint of the DXLR01 module.
     * @return The current endpoint.
     */
    inline uint32_t getEndpoint() const {
        return _endpoint;
    }

    /**
     * Sends data to a specific endpoint through the DXLR01 module.
     * @param data Pointer to the data to send.
     * @param len Length of the data to send.
     * @param ep The endpoint to send data to.
     * @return The number of bytes successfully sent.
     */
    size_t emit(const uint8_t* data, size_t len, uint32_t ep = DXLR01_EP_DEFAULT);

    /**
     * Queues data and sends it merged with other queued data in as few packets
     * as possible. A packet holds up to DXLR_QUEUE_SIZE bytes; call flush()
     * to send what is left. All queued data must go to the same endpoint.
     * @return The number of bytes queued.
     */
    size_t push(const uint8_t* data, size_t len, uint32_t ep = DXLR01_EP_DEFAULT);

    /**
     * Sends all data queued by push().
     * @return true if all data was sent.
     */
    bool flush();

    /**
     * Returns the number of bytes waiting in the push() queue.
     */
    inline size_t pending() const {
        return _queue.size();
    }

    /**
     * Receives data from the DXLR01 module into the provided buffer.
     * @param data Pointer to the buffer to store received data.
     * @param len Maximum length of data to receive.
     * @param timeout The timeout for receiving in milliseconds.
     * @return The number of bytes successfully received.
     */
    size_t recv(uint8_t* data, size_t len, int32_t timeout = -1);

};
