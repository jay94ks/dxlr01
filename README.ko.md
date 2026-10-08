# dxlr01

[English](README.md)

DX-LR01 LoRa UART 모듈용 C++ 드라이버. 리눅스(termios)와 Arduino(HardwareSerial)에서 같은 소스로 동작한다.

## 구성

| 경로 | 설명 |
|---|---|
| `src/dxlr01.h`, `src/dxlr01.cpp` | 드라이버 (`DXLR01` 클래스) |
| `src/dxlr01_platform.h/.cpp` | 플랫폼 계층 (`dxlr01::SerialPort`, 시간/지연, 바이트 버퍼) |
| `test/main.cpp` | 호스트용 핑퐁 테스트 (`make test`) |
| `examples/PingPong` | Arduino 예제 |
| `library.properties`, `keywords.txt` | Arduino 라이브러리 메타데이터 |

## 빌드 (호스트)

```sh
make                 # ./dxlr01 생성 (src, test 아래 모든 .cpp)
make test            # A=/dev/ttyUSB0(최초 송신), B=/dev/ttyUSB1(에코) 핑퐁. Ctrl+C로 종료
make run PORT=/dev/ttyUSB0 EMIT=y
```

`PORT_A`, `PORT_B`로 포트를 바꿀 수 있다. C++17 필요.

## 사용법

```cpp
DXLR01 radio("/dev/ttyUSB0");          // Arduino: DXLR01 radio(Serial1);
                                        // ESP32:   DXLR01 radio(Serial2, rxPin, txPin);

radio.init(DXLR_PROFILE_FASTEST);       // 보레이트 탐색 + 설정 읽기 (수 초 소요)
radio.setEndpoint(DXLR01_EP(0x0204, 0x01, DXLR_MODE_TR));
radio.restart();                        // 변경한 설정 적용

radio.emit(data, len);                  // 즉시 전송 (한 패킷 최대 247B)
radio.push(data, len); radio.flush();   // 작은 메시지를 모아 한 패킷으로 전송
size_t n = radio.recv(buf, sizeof(buf), 100);   // 타임아웃(ms), 읽은 바이트 수 반환
```

주요 API: `init`, `deinit`, `setProfile`, `setPower`, `setLevel`, `setSpreadFactor`, `setCodingRate`, `setEndpoint`, `restart`, `emit`, `push`, `flush`, `recv`.

### 프로파일

양쪽 모듈은 같은 프로파일(LEVEL/SF/CR)이어야 한다. 아래 값은 실측(LEVEL 7).

| 프로파일 | SF/CR | 속도(240B 패킷) | 편도 지연 |
|---|---|---|---|
| `FASTEST` | 5/2 | 약 870 B/s | 112 ms |
| `HIGH_THROUGHPUT` | 5/1 | 약 955 B/s | 112 ms (오류 정정 약함) |
| `BALANCED` | 7/2 | 약 450 B/s | 135 ms |
| `LONG_RANGE` | 9/2 | 약 170 B/s | 210 ms |
| `LONGEST_RANGE` | 12/4 | 약 24 B/s | 약 1 s |
| `AS_IS` | - | 모듈에 저장된 설정 유지 | - |

## Arduino

이 폴더를 `libraries/dxlr01`로 두고 `#include <dxlr01.h>`. 예제는 `examples/PingPong`.

- HardwareSerial과 SoftwareSerial 지원 (`DXLR01 radio(softSerial);`). 드라이버가 `begin()/end()`를 호출하므로 직접 `begin()` 하지 않는다.
- SoftwareSerial도 보레이트 제한이 없다. `init()`이 하드웨어 시리얼과 똑같이 가장 빠른 보레이트를 쓴다. 보드에서 불안정하면 `init()` 전에 `radio.setBaudLimit(9600)`으로 상한을 둔다.
- 32비트 보드: ESP32(`DXLR01 radio(Serial2, rxPin, txPin);`), earlephilhower 코어의 RP2040(같은 생성자, `setRX/setTX` 사용), 그 외 ARM 코어는 일반 `HardwareSerial`(`DXLR01 radio(Serial1);`). 코어에 `SoftwareSerial.h`가 있으면(AVR, ESP8266, EspSoftwareSerial이 있는 ESP32) SoftwareSerial도 쓸 수 있다. ESP32/ESP8266은 핀을 지정해 먼저 만든다.
- AVR은 RAM 때문에 최선 노력 수준이다.

## 동작 특성 / 주의

- **고정 지연**: 모듈이 첫 바이트부터 약 100 ms 동안 UART 입력을 모아 한 패킷으로 보낸다. 소량 데이터도 편도 약 110 ms, 왕복 약 220 ms 이상이며 호스트 코드로 줄일 수 없다. 작은 메시지는 `push`로 묶는 게 유일한 개선책이다.
- **패킷 한계**: 헤더 포함 250 B (payload 247 B, `DXLR_QUEUE_SIZE`). 넘으면 유실된다.
- **패킷 간격(SF5)**: 연속 송신 시 모듈 버퍼가 넘친다. 기본 간격 210 ms(`-DDXLR_PACKET_GAP_MS=...`로 변경)에서 약 1150 B/s, 유실 거의 없음(0.3% 수준 간헐 유실은 있음). 150 ms 이하는 손실이 뚜렷하다. 간격 없이 큰 프레임을 쏘면 유실 90%이고 모듈이 멈출 수 있다(전원 재투입 필요). SF6 이상은 미측정이라 pacing이 없다.
- **재전송 없음**: 유실 복구는 드라이버를 쓰는 쪽에서 구현해야 한다.
- **데이터 모드 junk**: 데이터 모드에서 쓴 것은 전부 공중으로 나간다(`AT`, `+++` 포함). 상대가 데이터 모드면 그대로 수신하므로, 한쪽 `init()`이 끝난 뒤 상대가 송신을 시작해야 한다.
- **모드 전환**: `+++\r\n`으로 AT 모드를 토글(`Entry AT`/`Exit AT`). AT 모드를 나오면 모듈이 재부팅하며 `Power on`을 출력한다.
- **금지**: `AT+DEFAULT`는 공장초기화이므로 절대 보내지 않는다.
- 에러 코드: 101 길이, 102 형식, 103 데이터 이상, 104 명령 오류.

## 실측 요약 (SF5, 모듈 2개)

- 정지-대기(패킷 하나 보내고 수신 후 다음): 16B 132, 64B 421, 128B 650, 240B 871 B/s (CR2).
- 핑퐁 RTT: 4/4B 223 ms, 240/240B 548 ms(양방향 합 876 B/s), 240/4B 388 ms(정방향 619 B/s). CR1은 240/240B 500 ms(961 B/s).
- 갭 파이프라인(240B, 100패킷): 갭 150 ms 1587 B/s(무손실이나 165 ms부터 손실), 180 ms 1345 B/s, 210 ms 1153 B/s.
- 느린 설정(SF9 이상)은 수신 대기를 20 s 이상 줘야 유실로 오인하지 않는다.
