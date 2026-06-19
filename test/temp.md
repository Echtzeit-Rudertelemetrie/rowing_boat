# Telemetry Hub — Task Checklist (`ble_sender_mcu` branch)

## Architecture & Data Flow

```
[Oarlock 1]──ESP-NOW──┐
                       ├──[Aggregator ESP32]──UART──[BLE Hub (XIAO nRF52840)]──BLE──[Phone ×4]
[Oarlock 2]──ESP-NOW──┘                              ├── GPS (u-blox, I2C 0x42)
                                                      └── IMU (MPU6050, I2C)
```

**Packets & rates:**

| Source | Struct | Rate | Size |
|---|---|---|---|
| Each oarlock → aggregator | `RowingPacket` (board_id, seq, force[32], angle[32]) | 10 p/s | 133 bytes |
| Aggregator → BLE hub (UART) | `RowingPacket` × 2 boards | **20 p/s** | 133 bytes each |
| GPS (onboard) | `GpsData` (lat, lon, speed, course, sats, valid) | 10 Hz | ~24 bytes |
| IMU (onboard) | `ImuData` (acc x/y/z, timestamp) — accel only | 60 Hz | 16 bytes |
| BLE hub → phones | `TelemetryPacket` (GPS+IMU) | **3.5 p/s** | TBD |
| BLE hub → phones | `RowingPacket` forwarded from UART | **20 p/s** | 133 bytes |
| **Total BLE** | | **~23.5 p/s** | well within limit |

**BLE capacity check (4 phones, 20 ms connection interval):**
23.5 p/s × 4 clients = ~94 radio transmissions/sec ≈ 18% radio duty cycle. No bottleneck.

**Scope:** This worktree owns the BLE hub firmware only — GPS sensor, IMU sensor, UART receive, BLE broadcast, battery management. The ESP-NOW receiver / aggregator side is owned by someone else.

**Library naming convention:** MCU-specific drivers live under a directory named after the MCU role, e.g. `lib/ble_sender/gps.{h,cpp}`, `lib/ble_sender/imu.{h,cpp}`. Shared/cross-MCU code keeps its functional name (`lib/ESPNOW`, `lib/AD7124`, ...).

---

## 1. Refactor: GPS into library  ✅ DONE (in `lib/ble_sender/`)

- [x] Define `GpsData` struct in `lib/ble_sender/gps.h` (int32 lat/lon as millionths, float speed/course, uint8 sats, bool valid — 18 bytes, packed)
- [x] Implement `setupGPS()` — `Wire.begin()`
- [x] Implement `updateGPS()` — DDC poll loop (request 32 bytes from 0x42, skip 0xFF, feed TinyGPSPlus)
- [x] Implement `getGpsData()` — returns populated `GpsData` from TinyGPSPlus object
- [x] Remove GPS globals and `readGPS()` from `main_BLE_sender.cpp`, replace with library calls
- [x] Moved `lib/GPS/` → `lib/ble_sender/` (MCU naming convention); removed obsolete empty `lib/seeed/` stubs
- [x] Build verified: `pio run -e prod_BLE_sender` → SUCCESS (RAM 3.3%, Flash 7.1%)
- [ ] Optional: re-add an NMEA-arrival diagnostic (charsProcessed / failedChecksum) if needed during bring-up — dropped in refactor since it's not part of `GpsData`

## 2. Refactor: IMU into library  ✅ DONE (in `lib/ble_sender/`)

Chip corrected: the IMU is the XIAO's **onboard LSM6DS3TR-C**, not an MPU6050.
Verified against the BSP variant: it lives on the internal I2C bus **Wire1**
(SDA=17/SCL=16), address **0x6A**, and must be powered via **PIN_LSM6DS3TR_C_POWER**
(pin 15). The external GPS uses the primary Wire (D4=SDA/D5=SCL).

- [x] Create `lib/ble_sender/imu.h` + `lib/ble_sender/imu.cpp`
- [x] Define `ImuData` struct (acc_x/y/z, timestamp_ms — **acceleration only**, 16 bytes, packed). Gyro dropped per requirement.
- [x] Implement `setupIMU()` — power-enable pin HIGH, `Wire1.begin()`, `begin_I2C(0x6A, &Wire1)`, accel range + ODR config, returns bool (ok/fail)
- [x] Implement `sampleIMU(ImuData&)` — rate-limited to ~60 Hz internally, returns true when new sample ready
- [x] Remove IMU globals and inline logic from `main_BLE_sender.cpp`, replace with library calls
- [x] Add `adafruit/Adafruit LSM6DS @ ^4.0` to `lib_deps` (kept Adafruit MPU6050 — still used by `lib/Gyroscope`)
- [x] Build verified: `pio run -e prod_BLE_sender` → SUCCESS (RAM 3.3%, Flash 7.2%)
- [ ] Hardware check pending: confirm the IMU answers on Wire1 / `setupIMU()` returns true on the actual board (build passes, but not yet run on hardware)

## 3. Define BLE telemetry packet  ✅ DONE (in `lib/ble_sender/telemetry_packet.h`)

- [x] Create `lib/ble_sender/telemetry_packet.h`
- [x] Define `TelemetryPacket` struct (`__attribute__((packed))`):
  - `uint32_t seq` — rolling counter per notification
  - `GpsData gps` (18 B)
  - `ImuData imu` (16 B)
- [x] Verify `sizeof(TelemetryPacket)` — **38 bytes**, enforced by `static_assert` (<= 247 B MTU, and == 38 to catch padding). Build confirms both pass.
- [x] Included in `main_BLE_sender.cpp` so the asserts are checked by the build
- [ ] `RowingPacket` (133 bytes, from `lib/ESPNOW/espnow.h`) is forwarded as-is on its own characteristic — no wrapping needed, fits in one notification (handled in section 5)

## 4. UART input from aggregator  ✅ DONE (in `lib/ble_sender/uart_rx.{h,cpp}`)

> **Prerequisite owned by aggregator team:** `espnow_init_receiver()` must be implemented on the aggregator side before UART forwarding works.

Implemented frame protocol (**PROPOSED — needs aggregator-team sign-off**):
`[0xAA][0x55][ RowingPacket : 133 B ][ CRC16-CCITT/FALSE : 2 B LE ]` = 137 B/frame.
UART: `Serial1`, **RX = D7 (pin 7)**, TX = D6, 115200 8N1. ESP32 TX → XIAO D7, common GND, no level shifter.

- [~] Agree on UART wire protocol with aggregator team — protocol defined & implemented, but the aggregator team must confirm/match it (sync word, CRC variant, baud)
- [x] Configure UART on XIAO nRF52840 — `Serial1` on D7/D6 (verified against BSP variant)
- [x] Implement receive loop: sync-word state machine, CRC-16 validation, `memcpy` into `RowingPacket`, latest-per-`board_id` buffer (`uartRxGetLatest`)
- [x] `RowingPacket` mirrored locally (espnow.h is ESP32-only) with `static_assert(sizeof == 133)` to guard against drift
- [x] main calls `setupUartRx()`/`updateUartRx()` and prints a 1 Hz report (per-board seq + good/CRC-error counters)
- [x] Build verified: `pio run -e prod_BLE_sender` → SUCCESS
- [ ] Hardware test pending: needs the aggregator (or a UART source on D7). Also blocked on-device by the I2C hang unless tested via a no-I2C path — see note below
- [ ] Consider sharing `RowingPacket` in a platform-neutral header both MCUs include (avoids the mirrored definition) — coordination item with the aggregator team

## 5. BLE broadcast  ✅ DONE (in `lib/ble_sender/ble_telemetry.{h,cpp}`)

**Portable backend** (because the board will move to XIAO ESP32-S3 later): `main`
calls a platform-neutral API (`setupBLE`/`bleNotifyTelemetry`/`bleNotifyRowing`/
`bleConnectionCount`); the `.cpp` picks the backend by platform macro:
- nRF52 (now) → **Adafruit Bluefruit** (bundled with the core — no lib_dep)
- ESP32-S3 (future) → **NimBLE** (skeleton in place; that env must add
  `h2zero/NimBLE-Arduino` + `-D CONFIG_BT_NIMBLE_MAX_CONNECTIONS=4`)

> Checklist correction: NimBLE is ESP32-only — wrong for the current nRF52, so
> Bluefruit is used now. No lib_deps change needed on `prod_BLE_sender`.

- [x] BLE setup: 1 service + 2 notify characteristics (TelemetryPacket 38 B,
      RowingPacket 133 B), up to 4 connections, advertises as "RowingBoat"
- [x] MTU raised (`configPrphBandwidth(BANDWIDTH_MAX)` → 247) so the 133 B
      RowingPacket fits in one notification
- [x] Notify A (~3.5 Hz): assemble `TelemetryPacket` (GPS + latest IMU) → notify
- [x] Notify B (~20 Hz): forward latest `RowingPacket` per board → notify
- [x] `PACKET_DEMO` also drives BLE with synthetic data → testable with a phone now
- [x] Build verified: `pio run -e prod_BLE_sender` → SUCCESS (RAM 5.9%, Flash 14.5%)
- [ ] Test with nRF Connect: connect, enable notifications on both characteristics,
      confirm 38 B / 133 B payloads and rates (flash the `PACKET_DEMO` build)
- [ ] UUIDs are placeholder (`a1b2c3d4-000x-...`) — pick final ones if desired

## 6. Battery charging management

- [ ] Identify charging IC / GPIO on XIAO nRF52840 schematic
- [ ] Implement charge state read
- [ ] Add charge status to `TelemetryPacket` or as a separate BLE characteristic

## 7. Cleanup

- [x] Moved GPS lib to `lib/ble_sender/`, removed obsolete `lib/seeed/`
- [ ] Consider renaming `lib/Gyroscope/` → MCU-named dir (e.g. `lib/oarlock/imu.{h,cpp}`) — it's really an IMU, not just a gyro. Deferred until the oarlock IMU chip is chosen.
- [ ] Verify `prod_BLE_sender` build passes after all refactors
