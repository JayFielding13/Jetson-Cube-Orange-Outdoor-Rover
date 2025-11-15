# ESP32 DevKit Pinout Diagram - Ultrasonic Sensor Network

**Board Type:** ESP32 DevKit (38-pin layout, 19 pins per side)
**Viewing Orientation:** Micro-USB port pointing BACKWARDS (towards you)
**Created:** November 7, 2025

---

## ESP32 Development Board - Physical Pinout with Pin Numbers

```
                        FRONT OF ROVER
                              ↑

    Left Side                                           Right Side
    ─────────                                           ──────────
    Pin #                                               Pin #

    L1   GND  ●                                              ● 3V3      R1
    L2   23   ●                                              ● EN       R2
    L3   22   ●                                              ● VP       R3
    L4   TX   ●                                              ● VN       R4
    L5   RX   ●                                              ● 34       R5
    L6   21   ●                                              ● 35       R6
    L7   GND  ●                                              ● 32       R7
    L8   19   ●                                              ● 33       R8
    L9   18   ●                                              ● 25 ──────── REAR TRIG              R9
    L10  5    ● ──────── CORNER RIGHT TRIG                 ● 26 ──────── REAR ECHO (LV2)        R10
    L11  17   ● ──────── SIDE RIGHT ECHO (LV5)            ● 27 ──────── CORNER RIGHT ECHO (LV3) R11
    L12  16   ● ──────── SIDE RIGHT TRIG                  ● 14 ──────── SIDE LEFT TRIG         R12
    L13  4    ● ──────── SIDE LEFT ECHO (via LV4)          ● 12 ──────── CORNER LEFT ECHO (LV6) R13
    L14  0    ●                                              ● GND      R14
    L15  2    ● ──────── FRONT ECHO (via LV1)              ● 13 ──────── CORNER LEFT TRIG       R15
    L16  15   ● ──────── FRONT TRIG                         ● 9  ──────── ⚠️ DON'T USE (Flash)   R16
    L17  8    ●                                              ● 10 ──────── ⚠️ DON'T USE (Flash)   R17
    L18  7    ●                                              ● 11       R18
    L19  6    ●                                              ● VIN      R19

         ╔═════════════════════════════════╗
         ║       MICRO-USB PORT            ║
         ║     (POINTING TOWARDS YOU)      ║
         ╚═════════════════════════════════╝

                        BACK OF ROVER
                              ↓
```

---

## Quick Pin Number Reference (for counting while wiring)

### Sensor Connections by Physical Pin Number

| Sensor | GPIO | Side | Pin Number | Count from Top |
|--------|------|------|------------|----------------|
| **Front TRIG** | GPIO 15 | LEFT | L16 | 16th pin down |
| **Front ECHO** | GPIO 2 | LEFT | L15 | 15th pin down (via level shifter) |
| **Corner Left TRIG** | GPIO 13 | RIGHT | R15 | 15th pin down |
| **Corner Left ECHO** | GPIO 12 | RIGHT | R13 | 13th pin down (via level shifter) |
| **Corner Right TRIG** | GPIO 5 | LEFT | L10 | 10th pin down |
| **Corner Right ECHO** | GPIO 27 | RIGHT | R11 | 11th pin down (via level shifter) |
| **Side Left TRIG** | GPIO 14 | RIGHT | R12 | 12th pin down |
| **Side Left ECHO** | GPIO 4 | LEFT | L13 | 13th pin down (via level shifter) |
| **Side Right TRIG** | GPIO 16 | LEFT | L12 | 12th pin down |
| **Side Right ECHO** | GPIO 17 | LEFT | L11 | 11th pin down (via level shifter) |
| **Rear TRIG** | GPIO 25 | RIGHT | R9 | 9th pin down |
| **Rear ECHO** | GPIO 26 | RIGHT | R10 | 10th pin down (via level shifter) |

### Power Pin Numbers

| Function | Side | Pin Number | Count from Top |
|----------|------|------------|----------------|
| **GND** | LEFT | L1 | 1st pin (top) |
| **GND** | LEFT | L7 | 7th pin |
| **GND** | RIGHT | R14 | 14th pin |
| **3.3V** | RIGHT | R1 | 1st pin (top) |
| **VIN** | RIGHT | R19 | 19th pin (bottom) |

---

## Simplified Sensor Connection View

**Looking at ESP32 from above, Micro-USB port pointing towards you:**

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 Top View                            │
│            (Micro-USB port at bottom, towards you)           │
└─────────────────────────────────────────────────────────────┘

LEFT SIDE PINS                              RIGHT SIDE PINS
═════════════                               ═══════════════

                FRONT SENSOR
              ┌─────────────────┐
                                │           GPIO 15 ─── Front TRIG
                                │           GPIO 2  ─── Front ECHO (LS)
              └─────────────────┘

            CORNER SENSORS (45°)
              ┌─────────────────┐
   GPIO 13 ───┤ Corner Left TRIG│
   GPIO 12 ───┤ Corner Left ECHO│           GPIO 5  ─── Corner Right TRIG
              │      (LS)        │           GPIO 27 ─── Corner Right ECHO (LS)
              └─────────────────┘

            SIDE SENSORS (90°)
              ┌─────────────────┐
   GPIO 4  ───┤ Side Left ECHO  │           GPIO 14 ─── Side Left TRIG
   GPIO 16 ───┤ Side Right TRIG │
   GPIO 17 ───┤ Side Right ECHO │
              │      (LS)        │
              └─────────────────┘

                REAR SENSOR
              ┌─────────────────┐
                                │           GPIO 25 ─── Rear TRIG
                                │           GPIO 26 ─── Rear ECHO (LS)
              └─────────────────┘

    VIN ─────── 5V Power for sensors (via power rail)
    GND ─────── Common ground (via ground rail)

(LS) = Through Level Shifter
```

---

## Pin Assignment Summary Table

| ESP32 Pin | Sensor | Signal | Connection Path |
|-----------|--------|--------|-----------------|
| **GPIO 15** | Front | TRIG | Direct → Front Sensor |
| **GPIO 2** | Front | ECHO | Level Shifter LV1 → HV1 → Front Sensor |
| **GPIO 13** | Corner Left | TRIG | Direct → Corner Left Sensor |
| **GPIO 12** | Corner Left | ECHO | Level Shifter LV6 → HV6 → Corner Left Sensor |
| **GPIO 5** | Corner Right | TRIG | Direct → Corner Right Sensor |
| **GPIO 27** | Corner Right | ECHO | Level Shifter LV3 → HV3 → Corner Right Sensor |
| **GPIO 14** | Side Left | TRIG | Direct → Side Left Sensor |
| **GPIO 4** | Side Left | ECHO | Level Shifter LV4 → HV4 → Side Left Sensor |
| **GPIO 16** | Side Right | TRIG | Direct → Side Right Sensor |
| **GPIO 17** | Side Right | ECHO | Level Shifter LV5 → HV5 → Side Right Sensor |
| **GPIO 25** | Rear | TRIG | Direct → Rear Sensor |
| **GPIO 26** | Rear | ECHO | Level Shifter LV2 → HV2 → Rear Sensor |
| **VIN** | All sensors | Power | Power rail → All sensor VCC pins (5V) |
| **GND** | All sensors | Ground | Ground rail → All sensor GND pins |

---

## Color-Coded Wiring Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    WIRING COLOR CODE                              │
├──────────────────────────────────────────────────────────────────┤
│  TRIG Wires:  YELLOW (direct to sensor)                          │
│  ECHO Wires:  BLUE (through level shifter)                       │
│  VCC Wires:   RED (to 5V power rail)                             │
│  GND Wires:   BLACK (to ground rail)                             │
└──────────────────────────────────────────────────────────────────┘

ESP32                   Level Shifter            AJ-SR04M Sensors
════════                ══════════════            ════════════════

                                                  ┌──────────────┐
GPIO 15─YELLOW──────────────────────────────────►│ Front TRIG   │
GPIO 2 ─BLUE───► LV1 ──► HV1 ─BLUE─────────────►│ Front ECHO   │
                                           5V ───►│ VCC          │
                                          GND ───►│ GND          │
                                                  └──────────────┘

                                                  ┌────────────────┐
GPIO 13─YELLOW──────────────────────────────────►│ Corner L TRIG  │
GPIO 12─BLUE───► LV6 ──► HV6 ─BLUE─────────────►│ Corner L ECHO  │
                                           5V ───►│ VCC            │
                                          GND ───►│ GND            │
                                                  └────────────────┘

                                                  ┌────────────────┐
GPIO 5 ─YELLOW──────────────────────────────────►│ Corner R TRIG  │
GPIO 27─BLUE───► LV3 ──► HV3 ─BLUE─────────────►│ Corner R ECHO  │
                                           5V ───►│ VCC            │
                                          GND ───►│ GND            │
                                                  └────────────────┘

                                                  ┌────────────────┐
GPIO 14─YELLOW──────────────────────────────────►│ Side L TRIG    │
GPIO 4 ─BLUE───► LV4 ──► HV4 ─BLUE─────────────►│ Side L ECHO    │
                                           5V ───►│ VCC            │
                                          GND ───►│ GND            │
                                                  └────────────────┘

                                                  ┌────────────────┐
GPIO 16─YELLOW──────────────────────────────────►│ Side R TRIG    │
GPIO 17─BLUE───► LV5 ──► HV5 ─BLUE─────────────►│ Side R ECHO    │
                                           5V ───►│ VCC            │
                                          GND ───►│ GND            │
                                                  └────────────────┘

                                                  ┌──────────────┐
GPIO 25─YELLOW──────────────────────────────────►│ Rear TRIG    │
GPIO 26─BLUE───► LV2 ──► HV2 ─BLUE─────────────►│ Rear ECHO    │
                                           5V ───►│ VCC          │
                                          GND ───►│ GND          │
                                                  └──────────────┘

VIN─RED────────────────────────────────────► 5V Power Rail
GND─BLACK──────────────────────────────────► Ground Rail
```

---

## Physical Installation Guide

### Step 1: Position ESP32 on Rover

```
                    FRONT OF ROVER
                         ↑
    ┌────────────────────────────────────┐
    │                                    │
    │ [Corner L]   [Front]  [Corner R]  │
    │     ↖         ↑         ↗         │
    │                                    │
    │                                    │
    │ [Side L] ← ┌──────────┐ → [Side R]│
    │            │  ESP32   │            │
    │            │   USB    │            │
    │            │    ↓     │            │
    │            └──────────┘            │
    │                                    │
    │              [Rear] ↓             │
    │                                    │
    └────────────────────────────────────┘
                         ↓
                    REAR OF ROVER
```

**Mount ESP32:**
- Center of rover chassis
- Micro-USB port facing REAR (for easy access)
- Sensors mounted around perimeter
- Level shifter mounted near ESP32

---

## Wire Length Planning

### Recommended Wire Lengths (from ESP32 to sensors)

| Sensor | Distance from ESP32 | Wire Length Needed |
|--------|--------------------|--------------------|
| Front | ~15cm forward | 20cm |
| Corner Left | ~20cm diagonal | 25cm |
| Corner Right | ~20cm diagonal | 25cm |
| Side Left | ~10cm left | 15cm |
| Side Right | ~10cm right | 15cm |
| Rear | ~15cm backward | 20cm |

**Add extra length for:**
- Routing around obstacles
- Strain relief
- Connection to level shifter

---

## Step-by-Step Connection Checklist

### ✅ Power Rails (Do First!)

- [ ] Create 5V power rail on breadboard
- [ ] Create GND (ground) rail on breadboard
- [ ] Connect ESP32 **VIN** pin (R19) to 5V rail
- [ ] Connect ESP32 **GND** pins (L1, L7, R14) to GND rail
- [ ] Connect ESP32 **3.3V** pin (R1) to 3.3V rail (for level shifter LV side)

### ✅ Level Shifter Setup

- [ ] Mount level shifter on breadboard
- [ ] Connect level shifter **HV** to 5V rail
- [ ] Connect level shifter **LV** to 3.3V rail
- [ ] Connect both level shifter **GND** pins to GND rail
- [ ] Test voltages: HV=5V, LV=3.3V

### ✅ Front Sensor

- [ ] Connect Front **VCC** (red) → 5V rail
- [ ] Connect Front **GND** (black) → GND rail
- [ ] Connect Front **TRIG** (yellow) → ESP32 **GPIO 15** (L16 - LEFT side, 16th pin)
- [ ] Connect Front **ECHO** (blue) → Level Shifter **HV1**
- [ ] Connect Level Shifter **LV1** → ESP32 **GPIO 2** (L15 - LEFT side, 15th pin)

### ✅ Corner Left Sensor

- [ ] Connect Corner Left **VCC** (red) → 5V rail
- [ ] Connect Corner Left **GND** (black) → GND rail
- [ ] Connect Corner Left **TRIG** (yellow) → ESP32 **GPIO 13** (R15 - RIGHT side, 15th pin)
- [ ] Connect Corner Left **ECHO** (blue) → Level Shifter **HV6**
- [ ] Connect Level Shifter **LV6** → ESP32 **GPIO 12** (R13 - RIGHT side, 13th pin)

### ✅ Corner Right Sensor

- [ ] Connect Corner Right **VCC** (red) → 5V rail
- [ ] Connect Corner Right **GND** (black) → GND rail
- [ ] Connect Corner Right **TRIG** (yellow) → ESP32 **GPIO 5** (L10 - LEFT side, 10th pin)
- [ ] Connect Corner Right **ECHO** (blue) → Level Shifter **HV3**
- [ ] Connect Level Shifter **LV3** → ESP32 **GPIO 27** (R11 - RIGHT side, 11th pin)

### ✅ Side Left Sensor

- [ ] Connect Side Left **VCC** (red) → 5V rail
- [ ] Connect Side Left **GND** (black) → GND rail
- [ ] Connect Side Left **TRIG** (yellow) → ESP32 **GPIO 14** (R12 - RIGHT side, 12th pin)
- [ ] Connect Side Left **ECHO** (blue) → Level Shifter **HV4**
- [ ] Connect Level Shifter **LV4** → ESP32 **GPIO 4** (L13 - LEFT side, 13th pin)

### ✅ Side Right Sensor

- [ ] Connect Side Right **VCC** (red) → 5V rail
- [ ] Connect Side Right **GND** (black) → GND rail
- [ ] Connect Side Right **TRIG** (yellow) → ESP32 **GPIO 16** (L12 - LEFT side, 12th pin)
- [ ] Connect Side Right **ECHO** (blue) → Level Shifter **HV5**
- [ ] Connect Level Shifter **LV5** → ESP32 **GPIO 17** (L11 - LEFT side, 11th pin)

### ✅ Rear Sensor

- [ ] Connect Rear **VCC** (red) → 5V rail
- [ ] Connect Rear **GND** (black) → GND rail
- [ ] Connect Rear **TRIG** (yellow) → ESP32 **GPIO 25** (R9 - RIGHT side, 9th pin)
- [ ] Connect Rear **ECHO** (blue) → Level Shifter **HV2**
- [ ] Connect Level Shifter **LV2** → ESP32 **GPIO 26** (R10 - RIGHT side, 10th pin)

---

## Quick Reference: Pin Locations on ESP32

**Micro-USB Port Pointing TOWARDS YOU (Rear of rover)**

### LEFT SIDE (Top to Bottom):
```
L1  = GND
L2  = 23
L3  = 22
L4  = TX
L5  = RX
L6  = 21
L7  = GND
L8  = 19
L9  = 18
L10 = 5     ← Corner Right TRIG
L11 = 17    ← Side Right ECHO (via level shifter)
L12 = 16    ← Side Right TRIG
L13 = 4     ← Side Left ECHO (via level shifter)
L14 = 0
L15 = 2     ← FC ECHO (via level shifter)
L16 = 15    ← FC TRIG
L17 = 8
L18 = 7
L19 = 6
```

### RIGHT SIDE (Top to Bottom):
```
R1  = 3V3
R2  = EN
R3  = VP
R4  = VN
R5  = 34
R6  = 35
R7  = 32
R8  = 33
R9  = 25    ← Rear TRIG
R10 = 26    ← Rear ECHO (via level shifter)
R11 = 27    ← Corner Right ECHO (via level shifter)
R12 = 14    ← Side Left TRIG
R13 = 12    ← Corner Left ECHO (via level shifter)
R14 = GND
R15 = 13    ← Corner Left TRIG
R16 = 9     ← ⚠️ DON'T USE (Flash pin)
R17 = 10    ← ⚠️ DON'T USE (Flash pin)
R18 = 11
R19 = VIN   ← 5V power input
```

---

## Step-by-Step Pin Counting Guide

### When Plugging In Wires (Micro-USB pointing towards you)

**LEFT SIDE (Count from top):**
```
L1  = GND       ← Start here (top of left side)
L2  = 23
L3  = 22
L4  = TX
L5  = RX
L6  = 21
L7  = GND
L8  = 19
L9  = 18
L10 = 5         ← Corner Right TRIG (10th pin)
L11 = 17        ← Side Right ECHO via level shifter (11th pin)
L12 = 16        ← Side Right TRIG (12th pin)
L13 = 4         ← Side Left ECHO via level shifter (13th pin)
L14 = 0
L15 = 2         ← Front ECHO via level shifter (15th pin)
L16 = 15        ← Front TRIG (16th pin)
L17 = 8
L18 = 7
L19 = 6         ← Bottom pin
```

**RIGHT SIDE (Count from top):**
```
R1  = 3V3       ← Start here (top of right side)
R2  = EN
R3  = VP
R4  = VN
R5  = 34
R6  = 35
R7  = 32
R8  = 33
R9  = 25        ← Rear TRIG (9th pin)
R10 = 26        ← Rear ECHO via level shifter (10th pin)
R11 = 27        ← Corner Right ECHO via level shifter (11th pin)
R12 = 14        ← Side Left TRIG (12th pin)
R13 = 12        ← Corner Left ECHO via level shifter (13th pin)
R14 = GND
R15 = 13        ← Corner Left TRIG (15th pin)
R16 = 9         ← ⚠️ DON'T USE (Flash pin)
R17 = 10        ← ⚠️ DON'T USE (Flash pin)
R18 = 11
R19 = VIN       ← 5V power input (bottom pin)
```

### Pro Tip for Counting
1. **Find the top pin** - It's marked as GND (LEFT) or 3V3 (RIGHT)
2. **Count down** from top using the pin numbers (L1, L2, L3... or R1, R2, R3...)
3. **Double-check** by looking at the printed GPIO number on the board
4. **Mark connected pins** with tape as you go to avoid confusion

---

## Testing After Wiring

### Continuity Test (Before Powering On)

1. **Power Rails:**
   - [ ] Verify 5V rail not shorted to GND
   - [ ] Verify 3.3V rail not shorted to GND

2. **TRIG Pins (Direct Connections):**
   - [ ] GPIO 15 continuity to Front TRIG
   - [ ] GPIO 13 continuity to Corner Left TRIG
   - [ ] GPIO 5 continuity to Corner Right TRIG
   - [ ] GPIO 14 continuity to Side Left TRIG
   - [ ] GPIO 16 continuity to Side Right TRIG
   - [ ] GPIO 25 continuity to Rear TRIG

3. **ECHO Pins (Through Level Shifter):**
   - [ ] Verify no direct connection from sensor ECHO to ESP32
   - [ ] Check path: Sensor ECHO → HV → LV → ESP32

### Power-On Test

1. **Voltage Check:**
   - [ ] 5V rail: 4.75V - 5.25V
   - [ ] 3.3V rail: 3.15V - 3.45V
   - [ ] Level shifter HV side: ~5V
   - [ ] Level shifter LV side: ~3.3V

2. **Sensor Power:**
   - [ ] Each sensor VCC pin: ~5V
   - [ ] Each sensor GND pin: 0V

---

## Troubleshooting Visual Guide

### Problem: Sensor not reading

**Check this connection path:**
```
Sensor                Level Shifter           ESP32
┌─────┐              ┌──────────┐          ┌───────┐
│ TRIG│◄─YELLOW──────────────────────────►│GPIO X │
│     │              │          │          │       │
│ ECHO│──BLUE──────►│HV → LV  │──BLUE───►│GPIO Y │
│     │              │          │          │       │
│ VCC │◄─RED─────────────5V Rail           │       │
│ GND │◄─BLACK───────────GND Rail          │       │
└─────┘              └──────────┘          └───────┘

✓ All 4 connections must be solid
✓ Level shifter powered correctly
✓ No loose wires
```

---

**Created:** November 7, 2025
**Author:** Jay
**Project:** Jetson Cube Orange Outdoor Rover
**Document:** ESP32 Visual Pinout for 6-Sensor Ultrasonic Array

**Print this page and keep it with your rover for easy reference during wiring!**
