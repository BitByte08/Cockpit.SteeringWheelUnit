# Cockpit.SteeringWheelUnit

STM32F103C8Tx 기반 FFB(Force Feedback) 스티어링 휠 펌웨어.  
CarSimulatorUnit(Unity)과 SLCAN 프로토콜로 통신하고, L298N + DC 모터로 FFB 토크를 출력한다.

---

## 하드웨어 구성

| 구성 요소 | 사양 |
|---|---|
| MCU | STM32F103C8Tx (Blue Pill, LQFP48) |
| 모터 드라이버 | L298N (12V, 2A/채널) |
| 모터 | 12V DC 모터 (감속기 포함 권장) |
| 엔코더 | 인크리멘탈 쿼드러처 엔코더, 기본값 **334 PPR** |
| 외부 크리스탈 | **8MHz HSE 필수** (PLL ×9 = 72MHz) |
| CAN 트랜시버 | SN65HVD230 또는 MCP2551 |
| PC 연결 | USB-시리얼 어댑터 (3.3V TTL, CH340 / CP2102 등) |

---

## 핀 배치 (Pinout)

### STM32F103C8Tx (LQFP48)

| 핀 번호 | 핀 이름 | 기능 | 연결 대상 | 비고 |
|:---:|:---|:---|:---|:---|
| 10 | `PA0` | — | — | 미사용 |
| 11 | `PA1` | — | — | 미사용 |
| 12 | `PA2` | EXTI2 (Falling) | 좌회전 스위치 (`TURN_LEFT`) | 납땜 와이어링 |
| 13 | `PA3` | EXTI3 (Falling) | 우회전 스위치 (`TURN_RIGHT`) | 납땜 와이어링 |
| 14 | `PA4` | — | — | 미사용 |
| 15 | `PA5` | — | — | 미사용 |
| 16 | `PA6` | TIM3_CH1 PWM (1 kHz) | L298N **ENA** | `MOTOR_PWM` |
| 17 | `PA7` | — | — | 미사용 |
| 18 | `PB0` | GPIO OUT | L298N **IN1** | `MOTOR_IN1` |
| 19 | `PB1` | GPIO OUT | L298N **IN2** | `MOTOR_IN2` |
| 20 | `PB2` | — | — | 미사용 (BOOT1, GND 고정 권장) |
| 21 | `PB10` | — | — | 미사용 |
| 22 | `PB11` | — | — | 미사용 |
| 25 | `PB12` | — | — | 미사용 |
| 26 | `PB13` | — | — | 미사용 |
| 27 | `PB14` | — | — | 미사용 |
| 28 | `PB15` | — | — | 미사용 |
| 29 | `PA8` | — | — | 미사용 |
| 30 | `PA9` | USART1_TX | USB-시리얼 **RX** | `SLCAN_TX`, 2 Mbaud |
| 31 | `PA10` | USART1_RX | USB-시리얼 **TX** | `SLCAN_RX`, 2 Mbaud |
| 32 | `PA11` | — | — | 미사용 (CAN 기본 핀, 리맵으로 미사용) |
| 33 | `PA12` | — | — | 미사용 (CAN 기본 핀, 리맵으로 미사용) |
| 34 | `PA13` | SWDIO | ST-Link / SWD | 디버그/플래시 |
| 37 | `PA14` | SWCLK | ST-Link / SWD | 디버그/플래시 |
| 38 | `PA15` | — | — | 미사용 |
| 39 | `PB3` | — | — | 미사용 |
| 40 | `PB4` | — | — | 미사용 |
| 41 | `PB5` | — | — | 미사용 |
| 42 | `PB6` | TIM4_CH1 | 엔코더 **A상** | `ENC_A`, 풀업 |
| 43 | `PB7` | TIM4_CH2 | 엔코더 **B상** | `ENC_B`, 풀업 |
| 45 | `PB8` | CAN1_RX (remap) | CAN 트랜시버 **RXD** | `CAN_RX` |
| 46 | `PB9` | CAN1_TX (remap) | CAN 트랜시버 **TXD** | `CAN_TX` |
| 47 | `VSS_3` | GND | 공통 GND | — |
| 48 | `VDD_3` | 3.3V | 전원 입력 | — |
| 7 | `NRST` | 리셋 | 리셋 스위치 또는 디버거 | — |
| 5 | `OSC_IN` | HSE 8 MHz | 외부 크리스탈 | 필수 |
| 6 | `OSC_OUT` | HSE 8 MHz | 외부 크리스탈 | 필수 |
| 8 | `VSSA` | 아날로그 GND | 공통 GND | — |
| 9 | `VDDA` | 아날로그 3.3V | 3.3V | — |

### 전원 및 공통

| 핀 / 노드 | 설명 |
|---|---|
| `3V3` | 엔코더 VCC, CAN 트랜시버 VCC, L298N 5V 로직 (옵션) |
| `5V` | USB 또는 외부 입력. L298N 5V 핀 공급 가능 |
| `GND` | 전체 공통 그라운드 (L298N, 엔코더, CAN 트랜시버, USB-시리얼) |
| `12V` | **외부 전원** → L298N VCC (모터 전원) |

> **주의:**
> - CAN1은 기본 PA11/PA12 대신 **PB8/PB9로 리맵**됨 (`AFIO_MAPR_CAN_REMAP2`).
> - SWD는 활성화 상태 (JTAG만 비활성화). 플래시는 SWD 또는 UART1 부트로더 사용.
> - `PA2`, `PA3`은 `.ioc`에는 없으나 `main.c`에서 EXTI로 직접 초기화됨 (좌/우 회전 신호 스위치).

---

## 기어비 및 엔코더 계산

### 설정값 (Core/Src/main.c)

```c
#define ENCODER_PPR   334   // 엔코더 분해능 (모터 축, 펄스/회전)
#define GEAR_RATIO      4   // 모터 회전 : 스티어링 휠 회전 = 4 : 1
```

### 유효 카운트 계산

```
쿼드러처 4× 멀티플라이: 334 × 4 = 1,336 카운트/모터 회전
기어비 적용:            1,336 × 4 = 5,344 카운트/휠 회전
±450° 스티어링 범위:    5,344 × (450 / 360) = ±6,680 카운트
```

| 파라미터 | 값 |
|---|---|
| `COUNTS_PER_REV` | 5,344 카운트/휠 회전 |
| `MAX_ENC_COUNT` | ±6,680 카운트 |
| int16 최대값 | ±32,767 (범위 초과 없음) |
| 각도 분해능 | 360° / 5,344 ≈ **0.067°/카운트** |

### 다른 엔코더 사용 시 조정

| 엔코더 PPR | 기어비 | COUNTS_PER_REV | 각도 분해능 |
|---|---|---|---|
| 100 | 4:1 | 1,600 | 0.225°/count |
| 334 | 4:1 | 5,344 | 0.067°/count (기본값) |
| 600 | 4:1 | 9,600 | 0.038°/count |
| 334 | 6:1 | 8,016 | 0.045°/count |

> 기어비가 클수록 토크 증가, 최고 속도 감소.  
> 4:1 기어비에서 L298N 2A 출력으로 충분한 FFB 감각 확보 가능.

---

## 클록 설정

```
HSE 8MHz → PLL ×9 → SYSCLK 72MHz
AHB  = 72MHz
APB1 = 36MHz  (CAN1, TIM3, TIM4, USART1 클록 소스)
APB2 = 72MHz

TIM 클록 = APB1 × 2 = 72MHz (APB1 프리스케일러 ≠ 1 규칙)
```

---

## 통신 프로토콜

### SLCAN (USART1, 2,000,000 baud)

Unity CarSimulatorUnit의 `CANBusManager.cs`와 직접 호환.

| CAN ID | 방향 | 포맷 | 설명 |
|---|---|---|---|
| `0x100` | STM32 → PC | int16 LE | 스티어링 각도. `값 ÷ 100 = 도(°)` |
| `0x101` | STM32 → PC | uint16 LE | 스위치 상태. Bit0=좌회전, Bit1=우회전 |
| `0x105` | PC → STM32 | int16 LE | FFB 토크 명령. `-32767 ~ +32767` |

#### SLCAN 프레임 예시

```
PC  →  STM32:  t105200FF   (0x105, DLC=2, 토크=0x00FF=255)
STM32 → PC:    t100209D0   (0x100, DLC=2, 각도 raw=0x09D0=2512 → 25.12°)
```

#### SLCAN 명령어

| 명령 | 설명 |
|---|---|
| `O\r` | 채널 열기 (FFB 활성화) |
| `C\r` | 채널 닫기 (모터 브레이크) |
| `S6\r` | 속도 설정 (무시됨, 하드웨어 고정) |
| `V\r` | 버전 조회 → `V0101\r` |

### CAN 버스 (CAN1, 500 Kbps)

EntertainmentClusterUnit(Raspberry Pi)과 동일한 물리 CAN 버스 공유 가능.  
스티어링 각도(0x100)는 SLCAN과 동시에 CAN 버스로도 송출된다.

**비트 타이밍:** `36MHz / 9 / (1+6+1) = 500,000 bps`

---

## L298N 배선

```
STM32 PA6  ──────► L298N ENA  (PWM 1kHz, 속도 제어)
STM32 PB0  ──────► L298N IN1  (방향 A)
STM32 PB1  ──────► L298N IN2  (방향 B)
STM32 GND  ──────► L298N GND  (공통)

L298N OUT1 ──────► DC 모터 +
L298N OUT2 ──────► DC 모터 -

전원
  12V ────────────► L298N VCC (모터 전원)
  5V  ────────────► L298N 5V  (로직 전원, STM32 5V 핀 사용 가능)
```

> L298N ENA가 LOW이거나 PWM 듀티가 0이면 모터는 프리-스핀 또는 브레이크 상태.  
> IN1=LOW, IN2=LOW → 브레이크 (본 펌웨어 idle 상태).

---

## 엔코더 배선

```
엔코더 A상 ──────► STM32 PB6 (TIM4_CH1)
엔코더 B상 ──────► STM32 PB7 (TIM4_CH2)
엔코더 VCC ──────► 3.3V
엔코더 GND ──────► GND
```

> 입력 필터 0x0F 적용 (디바운스).  
> 모터 축이 아닌 스티어링 휠 축에 엔코더를 장착하는 경우 `GEAR_RATIO=1` 로 변경.

---

## CAN 트랜시버 배선 (SN65HVD230 예시)

```
STM32 PB9 (CAN_TX) ──► SN65HVD230 TXD
STM32 PB8 (CAN_RX) ◄── SN65HVD230 RXD
3.3V               ──► SN65HVD230 VCC
GND                ──► SN65HVD230 GND
SN65HVD230 CANH  ◄──► CAN 버스 H
SN65HVD230 CANL  ◄──► CAN 버스 L
```

---

## Unity 통합 (CarSimulatorUnit)

기존 OpenFFBoard를 본 STM32 펌웨어로 대체:

1. `CANBusManager.cs`의 시리얼 포트를 STM32 USB-시리얼 포트로 변경
   ```csharp
   // 예: "COM3" (Windows) 또는 "/dev/ttyUSB0" (Linux)
   private string portName = "COM3";
   private int baudRate = 2000000;  // 변경 불필요
   ```
2. 시뮬레이션 모드 비활성화:
   ```csharp
   public bool simulationMode = false;
   ```
3. EPS.cs 파라미터 (참고):
   - `maxFeedbackTorque`: 1.0 (최대 정규화 토크, ±32767에 매핑)
   - `returnTorque`: 0.3 (센터링 스프링 강도)
   - `highSpeedDamping`: 0.4 (고속 댐핑)
   - `lateralGSensitivity`: 0.25 (노면 부하 감도)

---

## 펌웨어 빌드 및 플래시

### 빌드

```bash
cmake -B build --preset Debug
cmake --build build
```

### 플래시 (UART1 부트로더, SWD 비활성 시)

1. BOOT0 핀을 3.3V에 연결 (HIGH)
2. USB-시리얼 어댑터 PA9/PA10 연결
3. MCU 리셋
4. 플래시:
   ```bash
   stm32flash -w build/Cockpit.SteeringWheelUnit.bin -v -g 0x0 /dev/ttyUSB0
   ```
5. BOOT0를 GND로 복귀 후 리셋

### 플래시 (ST-Link SWD)

SWJ가 NOJTAG 모드이므로 SWD는 사용 가능:
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c "program build/Cockpit.SteeringWheelUnit.elf verify reset exit"
```

---

## 소프트웨어 구조

```
main.c
├── SystemClock_Config()     HSE 8MHz → 72MHz PLL
├── MX_GPIO_Init()           PB0/PB1 direction pins
├── MX_CAN_Init()            CAN1 500Kbps
├── MX_TIM3_Init()           PWM 1kHz (PA6 → L298N ENA)
├── MX_TIM4_Init()           Encoder TI12 (PB6, PB7)
├── MX_USART1_UART_Init()    2Mbaud SLCAN
├── Motor_SetTorque()        FFB → PWM + direction + end-stop guard
├── Encoder_Update()         16-bit overflow-safe delta accumulation
├── SLCAN_ProcessByte()      Line accumulator
├── SLCAN_ParseLine()        O/C/S/t command parser
└── SLCAN_SendAngle()        Tx CAN 0x100 via SLCAN + CAN bus

stm32f1xx_hal_msp.c          GPIO clock/mode config per peripheral
stm32f1xx_it.c               CAN1_RX0_IRQHandler, USART1_IRQHandler
```

---

## 외부 회로 연결도 (요약)

```
┌──────────────────────────────────────────────────────────────────────┐
│                         STM32F103C8Tx (Blue Pill)                    │
│                                                                      │
│   PA2 ────────┐  (TURN_LEFT)   PA6 ─────────► L298N ENA (PWM)        │
│   PA3 ────────┘  (TURN_RIGHT)  PB0 ─────────► L298N IN1              │
│                                PB1 ─────────► L298N IN2              │
│   PA9 ─────────────► USB-Serial RX                                 │
│   PA10 ◄──────────── USB-Serial TX                                 │
│                                                                      │
│   PB6 ◄──────────── 엔코더 A상 (TIM4_CH1)                           │
│   PB7 ◄──────────── 엔코더 B상 (TIM4_CH2)                           │
│                                                                      │
│   PB8 ◄──────────── CAN RX (SN65HVD230 RXD)                        │
│   PB9 ─────────────► CAN TX (SN65HVD230 TXD)                       │
│                                                                      │
│   3V3 ─────────────► 엔코더 VCC, CAN 트랜시버 VCC                   │
│   5V  ─────────────► L298N 5V (로직 전원)                           │
│   GND ─────────────► 전체 공통 GND                                  │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘

        ┌─────────────┐
   12V ─┤             │◄── VCC
   GND ─┤    L298N    │◄── 5V
  ENA ──┤   (모터    ├──► OUT1 ────► DC 모터 +
  IN1 ──┤    드라이버) ├──► OUT2 ────► DC 모터 -
  IN2 ──┤             │
        └─────────────┘

        ┌─────────────┐
   VCC ─┤  SN65HVD230 │◄── 3.3V
   GND ─┤  (CAN 트랜  │◄── GND
   TXD ─┤   시버)     ├──► CANH ────► CAN 버스 H
   RXD ─┤             ├──► CANL ────► CAN 버스 L
        └─────────────┘
```
