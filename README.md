# 🚀 Rover ESP32-S3 (4-Pin Direct Drive System)

Clean, ultra-fast firmware and web dashboard for the **Rover** (URRT Robo Cup).
Configured for **4-Pin PWM Control** with dual **BTS7960 (IBT-2)** high-power motor drivers.

> **Hardware Note**: The motor driver enable pins (`R_EN` + `L_EN`) are hardwired directly to the **5V** supply in hardware. No GPIO pin is used for enable (GPIO 15 is completely freed and unused). All other peripheral ports have been stripped out.

---

## 📌 Pin Configuration (Only These 4 Pins Used)

| Module | Pin | ESP32-S3 GPIO | Function |
| :--- | :--- | :--- | :--- |
| **U4 (Left Driver)** | **RPWM** | **GPIO 4** | Left Motors Forward (20 kHz PWM) |
| **U4 (Left Driver)** | **LPWM** | **GPIO 5** | Left Motors Reverse (20 kHz PWM) |
| **U5 (Right Driver)**| **RPWM** | **GPIO 6** | Right Motors Forward (20 kHz PWM) |
| **U5 (Right Driver)**| **LPWM** | **GPIO 7** | Right Motors Reverse (20 kHz PWM) |
| **Both Drivers** | **R_EN + L_EN** | **Tied to 5V** | Hardwired directly to 5V power |

*All other pins are freed and unused.*

---

## 🛡️ Anti-Burn PWM Turning Safety System

Because `EN` is hardwired to 5V (the H-bridge is permanently armed), the firmware's software protections are vital:

1. **Strict PWM Ceiling (`PWM_CAP`)**:
   Regardless of steering or throttle input, neither motor's duty cycle can ever exceed the configured limit (e.g. 75%).
2. **Turn Vector Normalization**:
   When throttle and steering are combined ($L = T + S, R = T - S$), the inputs are normalized so turning never saturates or draws destructive stall current.
3. **Spin-Turn Derating (`SPIN_TURN_DERATE = 0.80`)**:
   When executing a pivot turn on the spot ($T \approx 0$), high tire scrub friction against the floor can cause 25GA DC gear motors to stall. The firmware automatically derates in-place pivot power by 20%, keeping the windings cool.
4. **Slew-Rate Acceleration / Deceleration Limiting**:
   Limits rate of change to 15 PWM units per 10ms cycle. Prevents violent back-EMF voltage doubling (which can spike above 22V across the armature) and protects commutators.
5. **Zero-Crossing Shoot-Through Deadtime**:
   Before reversing direction, the active PWM pin is pulled to 0, held at 0 for 15ms deadtime, and only then is the opposite direction pin ramped up.
6. **Watchdog Failsafe**:
   If connection to the dashboard drops for $>350\text{ ms}$, both motors immediately ramp down to 0 PWM.

---

## 📡 Wi-Fi & Dashboard Access

- **Target Wi-Fi**:
  - SSID: `PC`
  - Password: `MK2001hadid`
  - Dashboard URL: **`http://rover.local`** or **`http://<rover-ip>`**
- **Field Fallback SoftAP**:
  - If `PC` is not detected within 12 seconds:
  - SSID: `EcoEco-Rover-AP`
  - Password: `rover123456`
  - URL: **`http://192.168.4.1`**

---

## 🛠️ Upload Instructions (Arduino IDE)

1. Open **Arduino IDE**.
2. Click **File** $\rightarrow$ **Open** and choose:
   `Rover_ESP32_Drive/Rover_ESP32_Drive.ino`
3. Under **Tools** $\rightarrow$ **Board**, choose **ESP32S3 Dev Module**.
4. Set:
   - **USB CDC On Boot**: *Enabled*
   - **Flash Size**: *16MB (128Mb)*
   - **Upload Speed**: *921600*
5. Connect your ESP32-S3 via USB-C, choose its **COM Port**, and click **Upload** (➡️).
6. Open **Serial Monitor** at **115200 baud** to see the IP address, then open `http://rover.local` in your browser.
