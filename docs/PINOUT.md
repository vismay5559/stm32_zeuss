# Zeus pinout — every wire into the STM32

The soldering reference. One table per sensor, plus the supply rails and the
traps that are not obvious from a silkscreen.

Board: **NUCLEO-H7S3L8** (MB1737), STM32H7S3L8H6. Header positions are from
UM3276, the board's user manual.

**This file is derived from `nexus_first.ioc` and the drivers.** If it ever
disagrees with them, they are right and this is a bug. The authoritative
sources are, per peripheral: `Appli/App/enc_as5047p.h`, `Appli/App/imu_bno085.c`,
`Appli/App/act_odrive.c`.

A note on headers: **CN7, CN8, CN9 and CN10 are the Zio connectors** — female,
fitted from the factory. **CN11 and CN12 are the ST morpho headers**, which
UM3276 §8.2 says are *not soldered by default*. Where a signal is reachable on
both, the Zio position is given first.

---

## Supply

| Rail | Header | Notes |
|---|---|---|
| **3V3** | CN8 pin 7 | From the U16 regulator, 1.3 A shared with the board itself. Clean. |
| **5V** | CN8 pin 9 | The raw eFuse rail. Ceiling is whatever ST-LINK negotiated — watch **LD9**: green is fine, orange means the USB port cannot supply what the board asked for. |
| **GND** | CN8 pins 11, 13 | |

3V3 is also on CN11 pin 16, 5V on CN11 pin 18, GND on CN11 pins 8/19/20/22.

---

## Spring encoders — 4 × AS5047P on SPI1

One shared bus. Every sensor sees the same SCK, MOSI and MISO; each has its own
chip select, and **exactly one is low at a time**.

| STM32 | Function | Zio header | ST morpho | Goes to |
|---|---|---|---|---|
| **PB3** | `SPI1_SCK` | CN7 pin 15 (**D23**) | CN12 pin 31 | CLK of all four |
| **PD7** | `SPI1_MOSI` | CN9 pin 2 (**D51**) | CN11 pin 45 | MOSI of all four |
| **PB4** | `SPI1_MISO` | CN7 pin 19 (**D25**) | CN12 pin 27 | MISO of all four |
| **PF1** | `enc_cs_l_hip` | CN9 pin 19 (**D69**) | — | CSn, **left hip** |
| **PD15** | `enc_cs_l_knee` | CN7 pin 18 (**D9**) | CN12 pin 48 | CSn, **left knee** |
| **PD14** | `enc_cs_r_hip` | CN7 pin 16 (**D10**) | CN12 pin 46 | CSn, **right hip** |
| **PF5** | `enc_cs_r_knee` | CN7 pin 20 (**D8**) | CN12 pin 36 | CSn, **right knee** |

6.25 MHz, SPI mode 1. `spring_angle[]` order is fixed by which pin a sensor's
CS is on: **0 left hip, 1 left knee, 2 right hip, 3 right knee**. If a spring is
reported as another one, a CS wire is on the wrong pin — `s_cs[]` in
`enc_as5047p.c` is the one place to change.

### On the AS5047P-TS_EK_AB adapter board

| Adapter pin | What it is |
|---|---|
| P1-1 `5V` | supply **input** |
| **P1-2 `3V3`** | **3.3 V LDO OUTPUT — leave unconnected in 5 V mode** |
| P1-4 `CSn`, P1-5 `CLK`, P1-6 `MOSI`, P1-7 `MISO` | SPI |
| P1-8 `GND` | ground |

**P1-2 is an output, not a rail.** The silkscreen says "3V3" and looks like a
supply input; it is the internal regulator's output, decoupled on-board by C2.
Wiring 5 V to it back-feeds the regulator and lifts the digital I/O to 5 V,
which PB4 is not guaranteed to survive.

Supply selection, from the adapter board manual: **5 V operation is R1
populated, R2 removed — the factory default.** 3.3 V operation is the reverse.
JP1 selects the same thing.

The digital I/O runs from that internal 3.3 V rail, so MISO swings 0–3.3 V in
either mode. Before connecting MISO to PB4 on a newly built harness, measure:
P1-2 to GND should read **~3.3 V**, and MISO idle should never exceed it.

### Magnet mounting — check this before blaming the firmware

From the adapter board manual §2.1, and the usual cause of an encoder that reads
nonsense while reporting no error at all:

- diametric magnet, **6 × 2.5 mm** (the AS5000-MD6H-2 in the kit)
- centred on the package to within **0.5 mm**
- airgap **0.5 mm to 3 mm**
- **the holder must not be ferromagnetic** — brass, aluminium, copper or
  stainless steel

A steel holder or a steel screw nearby warps the field, and the chip has no way
to know. Spin the magnet by hand: the angle should sweep smoothly through a full
turn, not jump or stick.

### Long leads

The encoders are a metre of wire from the board, running a 6.25 MHz bus. Put a
100 nF capacitor at each sensor's supply pin, at the connector on the leg, and
twist or screen SCK/MISO/MOSI. The driver counts parity failures and error flags
per sensor — watch `enc_valid` and the error counters before trusting the
springs.

---

## IMU — BNO085, SHTP over UART

| STM32 | Function | Zio header | ST morpho | Breakout pin |
|---|---|---|---|---|
| **PD5** | `USART2_TX` | CN9 pin 6 (**D53**) | — | **SCL** — the sensor's RX |
| **PD6** | `USART2_RX` | CN9 pin 4 (**D52**) | — | **SDA** — the sensor's TX |
| 3V3 | — | CN8 pin 7 | — | Vin |
| GND | — | CN8 pin 11 | — | GND |

3 Mbaud, PCLK1 at 150 MHz, 0% baud error. Crossed as always: our TX to its RX.

**The breakout labels are misleading on purpose.** In UART mode the pin marked
`SCL` is the sensor's receive and `SDA` is its transmit — they are not I²C here.

**Mode straps: `PS1` → 3V3, `PS0` left alone.** That selects UART-SHTP. No reset
pin is needed in this mode. The straps are sampled at the **sensor's** reset, so
after changing them you must fully power-cycle — `-rst` and the black button
restart the STM32 but do not drop the BNO085's 3.3 V.

### Not PA9/PA10

The IMU used to be on USART1 at PA9/PA10. It was moved because **PA9 is not a
free pin**: UM3276 Table 9 has it carrying `I2C_SDA` to the TCPP03-M20 USB-C
controller through solder bridge **SB35, closed by default**. **PA8** is the
matching `I2C_SCL` through **SB31** — so it is also a poor choice for anything,
including the IMU reset line that `imu_bno085.c` once suggested for it.

### First thing to check after soldering

Run `NEXUS_MODE_IMU` and look at the first bytes received:

| bytes | meaning |
|---|---|
| `7E 01 …` | correct — SHTP framing, baud right |
| `AA AA …` | the sensor is in **UART-RVC** mode — fix PS1/PS0 |
| noise | **baud mismatch** — nothing downstream will ever parse |

---

## CAN — 2 × FDCAN to the ODrive S1 drives

| STM32 | Function | Zio header | ST morpho | Bus |
|---|---|---|---|---|
| **PD1** | `FDCAN1_TX` | CN9 pin 27 (**D66**) | CN11 pin 55 | **bus 0 — left leg** |
| **PB8** | `FDCAN1_RX` | CN7 pin 2 (**D15**) | CN12 pin 3 | bus 0 — left leg |
| **PB6** | `FDCAN2_TX` | — | **CN12 pin 17** | **bus 1 — right leg** |
| **PB5** | `FDCAN2_RX` | CN7 pin 14 (**D11**) | CN12 pin 29 | bus 1 — right leg |

1 Mbit nominal / 2 Mbit data, sample point 87.5% to match the ODrive S1.

**PB6 is only on the ST morpho header**, which is not fitted from the factory —
it is the one signal here that needs CN12 populated.

Joint index is **`bus * 4 + (node - 1)`**: bus 0 is FDCAN1 and the left leg,
bus 1 is FDCAN2 and the right. Nodes 1–4 per bus.

### Isolated transceivers (ISO1042B)

The ISO1042B has **two supply domains**, and they must stay separate or the
isolation does nothing:

| Transceiver pin | Supply |
|---|---|
| `VCC1`, `GND1` | the STM32's **3V3** and GND — logic side, so TXD/RXD come out at 3.3 V |
| `VCC2`, `GND2` | an **isolated 5 V**, referenced to the **CAN bus** ground |

Powering `VCC2` from the Nucleo's 5 V and tying `GND2` to the Nucleo's ground
shorts across the barrier. The part still works, as a very expensive
non-isolated transceiver — and the ground shifts from motor current, which are
exactly what the isolation is there to block, go straight into the MCU.

The firmware's transmitter delay compensation is tuned for the ISO1042's 152 ns
loop delay. Fitting a non-isolated part instead means retuning the SSP offset.

Each end of the bus needs **120 Ω** termination, and only the two ends.

---

## Foot switches

| STM32 | Label | ST morpho | Goes to |
|---|---|---|---|
| **PE2** | `L_TOE` | CN11 pin 46 | the **left** foot's switch |
| **PE4** | `R_TOE` | CN11 pin 48 | the **right** foot's switch |
| PE3 | `L_HEEL` | CN11 pin 47 | **unconnected** |
| PE5 | `R_HEEL` | CN11 pin 50 | **unconnected** |

One switch per foot now, at the centre of the sole. The pin names are CubeMX's
and date from when each foot had a toe and a heel switch; renaming them needs
the `.ioc` opened in the GUI, so the mapping is written out in `contact_init()`
instead.

Switches are wired to **GND** with the STM32's internal pull-ups, so a closed
switch reads 0.

---

## Everything else, so it is not accidentally reused

| STM32 | Used by |
|---|---|
| PA13, PA14 | SWD — the debugger. Do not touch. |
| PD8, PD9 | USART3, the ST-LINK virtual COM port (the console) |
| PM5, PM6 | USB high speed to the Pi — the data link |
| PN0–PN11 | XSPI2, the external flash the Appli executes from |
| PH0, PH1 | the 24 MHz crystal |
| PD10, PD13, PB7 | LD1 green, LD2 yellow, LD3 red |
| PC13 | the blue USER button |
| PA8, PA9 | `I2C_SCL`/`I2C_SDA` to the TCPP03-M20 via SB31/SB35 — **not free** |

---

## Grounding, once motor power is live

The Pi and the board on separate supplies are bonded through the USB cable.
Keep both on one power strip. Once the drives are energised, the ground offsets
stop being millivolts: put a **USB isolator** between the Pi and the board.

Note that the link currently enumerates at **high speed (480 Mbit)** and most
cheap isolators are full speed only — check the part before buying.
