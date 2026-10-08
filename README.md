# dxlr01

A C++ driver for the DX-LR01 LoRa UART module. The same source runs on Linux (termios) and Arduino (HardwareSerial).

[한국어](README.ko.md)

## License

This project is licensed under the [MIT License](LICENSE).

## Layout

| Path | Description |
|---|---|
| `src/dxlr01.h`, `src/dxlr01.cpp` | The driver (`DXLR01` class) |
| `src/dxlr01_platform.h/.cpp` | Platform layer (`dxlr01::SerialPort`, time/delay, byte buffer) |
| `test/main.cpp` | Host ping-pong test (`make test`) |
| `examples/PingPong` | Arduino example |
| `library.properties`, `keywords.txt` | Arduino library metadata |

## Build (host)

```sh
make                 # builds ./dxlr01 (every .cpp under src and test)
make test            # ping-pong: A=/dev/ttyUSB0 (initial sender), B=/dev/ttyUSB1 (echo). Stop with Ctrl+C
make run PORT=/dev/ttyUSB0 EMIT=y
```

Override the ports with `PORT_A` / `PORT_B`. Requires C++17.

## Usage

```cpp
DXLR01 radio("/dev/ttyUSB0");          // Arduino: DXLR01 radio(Serial1);
                                        // ESP32:   DXLR01 radio(Serial2, rxPin, txPin);

radio.init(DXLR_PROFILE_FASTEST);       // baudrate scan + read settings (takes a few seconds)
radio.setEndpoint(DXLR01_EP(0x0204, 0x01, DXLR_MODE_TR));
radio.restart();                        // apply the changed settings

radio.emit(data, len);                  // send immediately (up to 247 B per packet)
radio.push(data, len); radio.flush();   // batch small messages into one packet
size_t n = radio.recv(buf, sizeof(buf), 100);   // timeout in ms, returns bytes read
```

Main API: `init`, `deinit`, `setProfile`, `setPower`, `setLevel`, `setSpreadFactor`, `setCodingRate`, `setEndpoint`, `restart`, `emit`, `push`, `flush`, `recv`.

### Profiles

Both modules of a link must use the same profile (LEVEL/SF/CR). Values below were measured (LEVEL 7).

| Profile | SF/CR | Throughput (240 B packets) | One-way latency |
|---|---|---|---|
| `FASTEST` | 5/2 | about 870 B/s | 112 ms |
| `HIGH_THROUGHPUT` | 5/1 | about 955 B/s | 112 ms (weaker error correction) |
| `BALANCED` | 7/2 | about 450 B/s | 135 ms |
| `LONG_RANGE` | 9/2 | about 170 B/s | 210 ms |
| `LONGEST_RANGE` | 12/4 | about 24 B/s | about 1 s |
| `AS_IS` | - | keeps the settings stored in the module | - |

## Arduino

Put this folder in `libraries/dxlr01` and `#include <dxlr01.h>`. See `examples/PingPong`.

- HardwareSerial and SoftwareSerial (`DXLR01 radio(softSerial);`). The driver calls `begin()/end()`, so do not call `begin()` yourself.
- SoftwareSerial has no baudrate limit: `init()` uses the fastest baudrate like on hardware serial. If that is unreliable on your board, cap it with `radio.setBaudLimit(9600)` before `init()`.
- 32-bit boards: ESP32 (`DXLR01 radio(Serial2, rxPin, txPin);`), RP2040 with the earlephilhower core (same constructor, uses `setRX/setTX`), and other ARM cores with a plain `HardwareSerial` (`DXLR01 radio(Serial1);`). SoftwareSerial is used when the core ships `SoftwareSerial.h` (AVR, ESP8266, ESP32 with EspSoftwareSerial); on ESP32/ESP8266 create it with its pins first.
- AVR is best effort.

## Behavior and cautions

- **Fixed latency**: the module collects UART input for about 100 ms from the first byte and sends it as one packet. Even tiny data takes about 110 ms one way and 220 ms or more round trip, and host code cannot reduce it. Batching small messages with `push` is the only remedy.
- **Packet limit**: 250 B including the header (247 B payload, `DXLR_QUEUE_SIZE`). Larger packets are lost.
- **Packet gap (SF5)**: sending back to back overflows the module buffer. The default gap of 210 ms (change with `-DDXLR_PACKET_GAP_MS=...`) gives about 1150 B/s with almost no loss (an occasional ~0.3% loss remains). At 150 ms or less the loss is clear. Sending large frames with no gap loses 90% and can hang the module (power cycle needed). SF6 and above are unmeasured, so no pacing is applied.
- **No retransmission**: recovering lost packets is up to the code using the driver.
- **Junk in data mode**: everything written in data mode goes over the air (including `AT` and `+++`). A peer still in data mode receives it, so the peer must finish `init()` before the other side starts sending.
- **Mode switching**: `+++\r\n` toggles AT mode (`Entry AT` / `Exit AT`). Leaving AT mode reboots the module, which prints `Power on`.
- **Forbidden**: never send `AT+DEFAULT` (factory reset).
- Error codes: 101 length, 102 format, 103 abnormal data, 104 command error.

## Measurement summary (SF5, two modules)

- Stop-and-wait (send one packet, wait for it, send the next): 16 B 132, 64 B 421, 128 B 650, 240 B 871 B/s (CR2).
- Ping-pong RTT: 4/4 B 223 ms, 240/240 B 548 ms (876 B/s both ways), 240/4 B 388 ms (619 B/s forward). CR1 at 240/240 B: 500 ms (961 B/s).
- Gap pipeline (240 B, 100 packets): 150 ms gap 1587 B/s (no loss, but loss starts at 165 ms), 180 ms 1345 B/s, 210 ms 1153 B/s.
- With slow settings (SF9 and above) wait 20 s or more for reception, or the data looks lost.
