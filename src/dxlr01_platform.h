#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/**
 * Platform layer of the driver: serial port, time, delays and a byte buffer.
 * On Arduino the serial port is a wrapper of HardwareSerial; on a host (Linux)
 * it is the termios based SerialPort. Implemented in dxlr01_platform.cpp.
 */
#ifdef ARDUINO
#include <Arduino.h>
#include <HardwareSerial.h>

/* Pin selectable hardware serial: ESP32 (begin with pins), RP2040 (setRX/setTX). */
#if defined(ESP32) || (defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED))
#define DXLR01_PIN_SELECT 1
#endif

/* SoftwareSerial: AVR, ESP8266/ESP32 (EspSoftwareSerial), and others that ship it. */
#if defined(__has_include)
#if __has_include(<SoftwareSerial.h>)
#include <SoftwareSerial.h>
#define DXLR01_HAS_SOFTWARE_SERIAL 1
#endif
#elif defined(ARDUINO_ARCH_AVR) || defined(ESP8266) || defined(ESP32)
#include <SoftwareSerial.h>
#define DXLR01_HAS_SOFTWARE_SERIAL 1
#endif
#else
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#endif

namespace dxlr01 {

#ifdef ARDUINO
typedef long dxlr_ssize_t;

/**
 * Serial port on top of an Arduino HardwareSerial or SoftwareSerial, with the
 * same interface as the termios based SerialPort used on a host.
 */
class SerialPort {
public:
#if defined(DXLR01_PIN_SELECT)
    explicit SerialPort(HardwareSerial& serial, int8_t rx = -1, int8_t tx = -1)
        : _serial(&serial), _hw(&serial), _rx(rx), _tx(tx) {}
#else
    explicit SerialPort(HardwareSerial& serial) : _serial(&serial), _hw(&serial) {}
#endif

#ifdef DXLR01_HAS_SOFTWARE_SERIAL
    explicit SerialPort(SoftwareSerial& serial) : _serial(&serial), _sw(&serial) {}
#endif

    bool open(const char* device, unsigned baudrate);
    void close();
    bool isOpen() const { return _open; }

    /**
     * Reads up to len bytes. Waits until at least one byte is available,
     * for timeoutMs milliseconds (forever if negative).
     * @return The number of bytes read, 0 on timeout.
     */
    dxlr_ssize_t read(void* buf, size_t len, int timeoutMs = -1);
    dxlr_ssize_t write(const void* buf, size_t len);

    bool flush();
    bool discardInput();

private:
    Stream* _serial;
    HardwareSerial* _hw = nullptr;
#ifdef DXLR01_HAS_SOFTWARE_SERIAL
    SoftwareSerial* _sw = nullptr;
#endif
    bool _open = false;
#if defined(DXLR01_PIN_SELECT)
    int8_t _rx = -1;
    int8_t _tx = -1;
#endif
};

#else
typedef ssize_t dxlr_ssize_t;

/**
 * Serial port on top of termios (host).
 */
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /**
     * Opens the device as 8N1 raw mode.
     * @return true on success, false otherwise (errno is set).
     */
    bool open(const char* device, unsigned baudrate = 115200);
    void close();
    bool isOpen() const { return _fd >= 0; }

    /**
     * Reads up to len bytes. Waits for timeoutMs milliseconds
     * (forever if negative, no wait if zero).
     * @return The number of bytes read, 0 on timeout, -1 on error.
     */
    dxlr_ssize_t read(void* buf, size_t len, int timeoutMs = -1);

    /**
     * Writes all len bytes.
     * @return The number of bytes written, or -1 on error.
     */
    dxlr_ssize_t write(const void* buf, size_t len);

    bool flush();
    bool discardInput();

private:
    int _fd = -1;
};
#endif

#ifdef ARDUINO
inline uint32_t millis() { return millis(); }

inline void delayUs(uint32_t us) {
    if (us >= 1000) {
        delay(us / 1000);
    } else {
        delayMicroseconds(us);
    }
}
#else
/**
 * Returns a monotonic time in milliseconds (wraps around after ~49 days,
 * so only use it for differences computed in uint32_t).
 */
inline uint32_t millis() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/**
 * Blocks for the given microseconds.
 */
inline void delayUs(uint32_t us) { usleep(us); }
#endif

/**
 * Minimal text helpers (replacements of printf/scanf, which are limited or
 * heavy on small boards).
 */

/**
 * Appends a string to a buffer of `cap` bytes holding `n` characters.
 * Always keeps the buffer NUL terminated.
 * @return The new length.
 */
inline size_t textAppend(char* buf, size_t cap, size_t n, const char* s) {
    while (*s != '\0' && n + 1 < cap) {
        buf[n++] = *s++;
    }

    buf[n] = '\0';
    return n;
}

/**
 * Appends an unsigned number in decimal, or in lowercase hex padded to
 * `minDigits` digits.
 * @return The new length.
 */
inline size_t textAppendUInt(char* buf, size_t cap, size_t n, unsigned v, unsigned base = 10, unsigned minDigits = 1) {
    char tmp[12];
    unsigned i = 0;

    do {
        const unsigned d = v % base;
        tmp[i++] = static_cast<char>(d < 10 ? '0' + d : 'a' + (d - 10));
        v /= base;
    } while (v != 0 && i < sizeof(tmp));

    while (i < minDigits && i < sizeof(tmp)) {
        tmp[i++] = '0';
    }

    while (i > 0 && n + 1 < cap) {
        buf[n++] = tmp[--i];
    }

    buf[n] = '\0';
    return n;
}

/**
 * Parses an unsigned number at `p` (leading spaces skipped) and advances `p`.
 * @return false if there is no digit.
 */
inline bool textScanUInt(const char*& p, unsigned base, unsigned& out) {
    while (*p == ' ' || *p == '\t') {
        ++p;
    }

    bool any = false;
    unsigned v = 0;

    for (;; ++p) {
        const char c = *p;
        unsigned d;

        if (c >= '0' && c <= '9') {
            d = static_cast<unsigned>(c - '0');
        } else if (c >= 'a' && c <= 'f') {
            d = static_cast<unsigned>(c - 'a') + 10;
        } else if (c >= 'A' && c <= 'F') {
            d = static_cast<unsigned>(c - 'A') + 10;
        } else {
            break;
        }

        if (d >= base) {
            break;
        }

        v = v * base + d;
        any = true;
    }

    out = v;
    return any;
}

/**
 * Minimal growable byte buffer (no STL, so it also works on small boards).
 */
class DXLR01_Bytes {
public:
    DXLR01_Bytes() = default;
    ~DXLR01_Bytes() { free(_p); }

    DXLR01_Bytes(const DXLR01_Bytes&) = delete;
    DXLR01_Bytes& operator=(const DXLR01_Bytes&) = delete;

    size_t size() const { return _n; }
    bool empty() const { return _n == 0; }
    uint8_t* data() { return _p; }
    const uint8_t* data() const { return _p; }
    uint8_t& operator[](size_t i) { return _p[i]; }
    void clear() { _n = 0; }

    /**
     * Appends bytes at the end.
     * @return false if out of memory (nothing is appended).
     */
    bool append(const uint8_t* src, size_t len);

    /**
     * Removes the first `len` bytes.
     */
    void eraseFront(size_t len);

private:
    uint8_t* _p = nullptr;
    size_t _n = 0;
    size_t _cap = 0;
};

} // namespace dxlr01
