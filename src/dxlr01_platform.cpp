#include "dxlr01_platform.h"

#ifndef ARDUINO
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#endif

namespace dxlr01 {

#ifdef ARDUINO

bool SerialPort::open(const char*, unsigned baudrate) {
    close();

    if (_hw != nullptr) {
#if defined(ESP32)
        _hw->begin(baudrate, SERIAL_8N1, _rx, _tx);
#elif defined(DXLR01_PIN_SELECT)
        if (_rx >= 0) _hw->setRX(_rx);
        if (_tx >= 0) _hw->setTX(_tx);
        _hw->begin(baudrate);
#else
        _hw->begin(baudrate);
#endif
    }
#ifdef DXLR01_HAS_SOFTWARE_SERIAL
    else if (_sw != nullptr) {
        _sw->begin(baudrate);
    }
#endif
    else {
        return false;
    }

    _open = true;
    return true;
}

void SerialPort::close() {
    if (!_open) {
        return;
    }

    if (_hw != nullptr) {
        _hw->end();
    }
#ifdef DXLR01_HAS_SOFTWARE_SERIAL
    else if (_sw != nullptr) {
        _sw->end();
    }
#endif

    _open = false;
}

dxlr_ssize_t SerialPort::read(void* buf, size_t len, int timeoutMs) {
    const uint32_t begin = millis();

    while (_serial->available() <= 0) {
        if (timeoutMs >= 0 && millis() - begin >= static_cast<uint32_t>(timeoutMs)) {
            return 0;
        }

        yield();
    }

    uint8_t* p = static_cast<uint8_t*>(buf);
    size_t n = 0;

    while (n < len && _serial->available() > 0) {
        const int c = _serial->read();
        if (c < 0) {
            break;
        }

        p[n++] = static_cast<uint8_t>(c);
    }

    return static_cast<dxlr_ssize_t>(n);
}

dxlr_ssize_t SerialPort::write(const void* buf, size_t len) {
    return static_cast<dxlr_ssize_t>(_serial->write(static_cast<const uint8_t*>(buf), len));
}

bool SerialPort::flush() {
    _serial->flush();
    return true;
}

bool SerialPort::discardInput() {
    while (_serial->available() > 0) {
        _serial->read();
    }

    return true;
}

#else


static bool toSpeed(unsigned baud, speed_t& out) {
    switch (baud) {
#define DXLR_B(n) case n: out = B##n; return true;
        DXLR_B(1200) DXLR_B(2400) DXLR_B(4800) DXLR_B(9600) DXLR_B(19200)
        DXLR_B(38400) DXLR_B(57600) DXLR_B(115200) DXLR_B(230400)
        DXLR_B(460800) DXLR_B(921600)
#undef DXLR_B
    default: return false;
    }
}

bool SerialPort::open(const char* device, unsigned baudrate) {
    close();

    speed_t speed;
    if (!toSpeed(baudrate, speed)) {
        errno = EINVAL;
        return false;
    }

    int fd = ::open(device, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) {
        return false;
    }

    termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        int e = errno; ::close(fd); errno = e; return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
    tio.c_cflag |= CS8;

    // timeouts are handled by poll(), so read returns available data at once
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0 ||
        tcsetattr(fd, TCSANOW, &tio) != 0) {
        int e = errno; ::close(fd); errno = e; return false;
    }

    tcflush(fd, TCIOFLUSH);
    _fd = fd;
    return true;
}

void SerialPort::close() {
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

dxlr_ssize_t SerialPort::read(void* buf, size_t len, int timeoutMs) {
    if (_fd < 0) { errno = EBADF; return -1; }

    pollfd pfd = {_fd, POLLIN, 0};
    int r;
    do {
        r = poll(&pfd, 1, timeoutMs);
    } while (r < 0 && errno == EINTR);

    if (r < 0) return -1;
    if (r == 0) return 0;
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) && !(pfd.revents & POLLIN)) {
        errno = EIO;
        return -1;
    }

    ssize_t n;
    do {
        n = ::read(_fd, buf, len);
    } while (n < 0 && errno == EINTR);

    return n;
}

dxlr_ssize_t SerialPort::write(const void* buf, size_t len) {
    if (_fd < 0) { errno = EBADF; return -1; }

    const uint8_t* p = static_cast<const uint8_t*>(buf);
    size_t done = 0;
    while (done < len) {
        ssize_t n = ::write(_fd, p + done, len - done);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN) {
                pollfd pfd = {_fd, POLLOUT, 0};
                poll(&pfd, 1, -1);
                continue;
            }

            return -1;
        }

        done += static_cast<size_t>(n);
    }

    return static_cast<dxlr_ssize_t>(done);
}

bool SerialPort::flush() { return _fd >= 0 && tcdrain(_fd) == 0; }

bool SerialPort::discardInput() { return _fd >= 0 && tcflush(_fd, TCIFLUSH) == 0; }

#endif

bool DXLR01_Bytes::append(const uint8_t* src, size_t len) {
    if (_n + len > _cap) {
        size_t cap = _cap ? _cap : 64;
        while (cap < _n + len) {
            cap *= 2;
        }

        uint8_t* p = static_cast<uint8_t*>(realloc(_p, cap));
        if (p == nullptr) {
            return false;
        }

        _p = p;
        _cap = cap;
    }

    memcpy(_p + _n, src, len);
    _n += len;
    return true;
}

void DXLR01_Bytes::eraseFront(size_t len) {
    if (len >= _n) {
        _n = 0;
        return;
    }

    memmove(_p, _p + len, _n - len);
    _n -= len;
}

} // namespace dxlr01
