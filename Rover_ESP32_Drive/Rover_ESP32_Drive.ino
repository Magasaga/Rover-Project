/*
 ===============================================================================
   ECO-ECO ROVER — 100% WIRELESS ESP32-S3 DRIVE & TERMINAL DASHBOARD
   All-In-One Self-Contained Sketch (Dual-Channel: WebSocket + HTTP Drive)
   
   Hardware: ESP32-S3 (DevKitC-1)
   Motor Drivers: 2x BTS7960 (IBT-2)
     - Left Motor Forward  : GPIO 4 (RPWM)
     - Left Motor Reverse  : GPIO 5 (LPWM)
     - Right Motor Forward : GPIO 6 (RPWM)
     - Right Motor Reverse : GPIO 7 (LPWM)
     - Driver Enable (EN)  : Hardwired directly to 5V power supply
 ===============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

// =============================================================================
// 1. CONFIGURATION
// =============================================================================
#define WIFI_SSID             "PC"
#define WIFI_PASSWORD         "MK2001hadid"
#define WIFI_CONNECT_TIMEOUT  12000      // 12s timeout before fallback to AP

#define AP_SSID               "EcoEco-Rover-AP"
#define AP_PASSWORD           "rover123456"
#define AP_IP                 192, 168, 4, 1
#define MDNS_HOSTNAME         "rover"    // Access at http://rover.local

#define HTTP_PORT             80
#define WS_PORT               81         // Dedicated WebSocket port

// 4 MOTOR PWM PINS ONLY (EN is hardwired to 5V in circuit)
#define PIN_LEFT_RPWM         4          // GPIO4: Left Driver Forward PWM
#define PIN_LEFT_LPWM         5          // GPIO5: Left Driver Reverse PWM
#define PIN_RIGHT_RPWM        6          // GPIO6: Right Driver Forward PWM
#define PIN_RIGHT_LPWM        7          // GPIO7: Right Driver Reverse PWM

// 4 ULTRASONIC SENSORS (HC-SR04 x4) — Safe Logic Level via 1k/1.8k Dividers
#define PIN_SONAR1_TRIG       10         // Front Centre Trig (GPIO 10)
#define PIN_SONAR1_ECHO       11         // Front Centre Echo (GPIO 11 via R11 1k/1.8k)
#define PIN_SONAR2_TRIG       12         // Front Left Trig (GPIO 12)
#define PIN_SONAR2_ECHO       13         // Front Left Echo (GPIO 13 via R13 1k/1.8k)
#define PIN_SONAR3_TRIG       14         // Front Right Trig (GPIO 14)
#define PIN_SONAR3_ECHO       16         // Front Right Echo (GPIO 16 via R15 1k/1.8k)
#define PIN_SONAR4_TRIG       17         // Rear Trig (GPIO 17)
#define PIN_SONAR4_ECHO       18         // Rear Echo (GPIO 18 via R17 1k/1.8k)

#define SONAR_COUNT           4
#define SONAR_MAX_DIST_CM     250        // Max measurement range (cm)
#define SONAR_TIMEOUT_US      14500      // 14.5ms max pulseIn wait (~250cm range)
#define SONAR_PING_INTERVAL_MS 35        // Non-blocking round-robin ping (sweep all 4 in 140ms)
#define OBSTACLE_ALERT_DIST_CM 20        // Close obstacle warning threshold (cm)

#define MOTOR_LEFT_INVERT     false
#define MOTOR_RIGHT_INVERT    false

// ANTI-BURN TURNING SAFETY PARAMETERS
#define PWM_FREQUENCY_HZ      20000      // 20 kHz ultrasonic PWM (smooth & silent)
#define PWM_RESOLUTION_BITS   8          // 8-bit resolution (0 to 255)
#define PWM_MAX_RAW           255
#define MOTOR_SLEW_STEP       15         // Max PWM change per 10ms cycle (prevents back-EMF spikes)
#define MOTOR_DEADBAND_RAW    10         // Below this duty, output 0 to prevent motor coil buzz
#define SPIN_TURN_DERATE      0.80f      // 20% power derating during pivot turns to prevent stall heat
#define DIRECTION_DEADTIME_MS 15         // Deadtime before reversing H-bridge direction
#define WATCHDOG_TIMEOUT_MS   450        // Coasts to stop if connection drops >450ms

// =============================================================================
// 2. EMBEDDED DASHBOARD HTML (Stored in ESP32-S3 Internal Flash Memory)
// =============================================================================
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Rover Terminal Dashboard</title>
    <link rel="preconnect" href="https://fonts.googleapis.com" />
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin />
    <link
      href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&display=swap"
      rel="stylesheet"
    />
    <style>
      :root {
        --bg-canvas: #f2f4f3;
        --titlebar-bg: #e5eae7;
        --panel-bg: #ffffff;
        --panel-head-bg: #f6f8f7;
        --panel-border: #c5d1ca;
        --panel-divider: #dce4df;
        --slot-bg: #f4f7f5;
        --slot-border: #c7d4cc;
        --screen-bg: #fafcfb;
        --cam-bg: #e9efec;
        --ink: #0c1810;
        --muted: #455c4e;
        --dim: #6c8274;
        --accent: #15803d;
        --accent-strong: #14532d;
        --accent-bright: #16a34a;
        --accent-fg: #ffffff;
        --ink-active-bg: #0c1810;
        --ink-active-fg: #ffffff;
        --dot-idle: #b8c7be;
        --dot-err: #dc2626;
        --trigger-lt-from: #0c1810;
        --trigger-lt-to: #475569;
        --trigger-rt-from: #14532d;
        --trigger-rt-to: #22c55e;
      }

      html.dark {
        --bg-canvas: #0c0c0c;
        --titlebar-bg: #161616;
        --panel-bg: #111312;
        --panel-head-bg: #161a18;
        --panel-border: #26332b;
        --panel-divider: #1f2b24;
        --slot-bg: #161b18;
        --slot-border: #2a3a31;
        --screen-bg: #090b0a;
        --cam-bg: #070908;
        --ink: #e8f0eb;
        --muted: #8aa395;
        --dim: #587062;
        --accent: #22c55e;
        --accent-strong: #16a34a;
        --accent-bright: #4ade80;
        --accent-fg: #04130a;
        --ink-active-bg: #e8f0eb;
        --ink-active-fg: #090b0a;
        --dot-idle: #33453a;
        --dot-err: #ef4444;
        --trigger-lt-from: #475569;
        --trigger-lt-to: #e2e8f0;
        --trigger-rt-from: #14532d;
        --trigger-rt-to: #4ade80;
      }

      *, *::before, *::after { box-sizing: border-box; }
      html, body {
        width: 100%; height: 100%; margin: 0; padding: 0; overflow: hidden;
        background: var(--bg-canvas); color: var(--ink);
        font-family: 'JetBrains Mono', monospace;
        user-select: none; -webkit-user-select: none;
      }
      .app-shell { width: 100vw; height: 100dvh; display: flex; flex-direction: column; overflow: hidden; }
      .titlebar {
        height: 36px; flex-shrink: 0; display: flex; align-items: center; justify-content: space-between;
        gap: 12px; padding: 0 12px; background: var(--titlebar-bg); border-bottom: 1px solid var(--panel-border);
        font-size: 11px;
      }
      .titlebar-left, .titlebar-right { display: flex; align-items: center; gap: 8px; }
      .term-btn {
        background: var(--slot-bg); border: 1px solid var(--slot-border); color: var(--muted);
        font-family: inherit; font-size: 10px; font-weight: 600; padding: 3px 8px; cursor: pointer;
        display: inline-flex; align-items: center; gap: 6px;
      }
      .term-btn:hover { color: var(--ink); border-color: var(--accent); }
      .term-btn.active { background: var(--accent); border-color: var(--accent-strong); color: var(--accent-fg); }
      .theme-btn { background: var(--panel-bg); color: var(--ink); padding: 3px 10px; }
      .ws-pill {
        display: inline-flex; align-items: center; gap: 6px; padding: 2px 8px;
        background: var(--slot-bg); border: 1px solid var(--slot-border); font-size: 10px;
        font-weight: 600; color: var(--muted); cursor: pointer;
      }
      .workspace { flex: 1; min-height: 0; padding: 8px; display: flex; flex-direction: column; gap: 8px; overflow: hidden; }
      .panel { background: var(--panel-bg); border: 1px solid var(--panel-border); display: flex; flex-direction: column; min-height: 0; }
      .panel-head {
        display: flex; align-items: center; justify-content: space-between; gap: 8px;
        padding: 6px 10px; background: var(--panel-head-bg); border-bottom: 1px solid var(--panel-divider);
        font-size: 11px; font-weight: 700; color: var(--ink);
      }
      .panel-title { display: flex; align-items: center; gap: 8px; }
      .status-dot { width: 8px; height: 8px; border-radius: 999px; background: var(--dot-idle); flex-shrink: 0; }
      .status-dot.on { background: var(--accent-bright); box-shadow: 0 0 6px var(--accent-bright); }
      .status-dot.err { background: var(--dot-err); }
      .btn-row { display: flex; align-items: center; gap: 4px; flex-wrap: wrap; }
      .top-row { flex: 1; min-height: 0; display: grid; grid-template-columns: minmax(0, 1.45fr) minmax(340px, 1fr); gap: 8px; }
      .camera-viewport { position: relative; flex: 1; min-height: 0; width: 100%; overflow: hidden; background: var(--cam-bg); }
      .camera-viewport video, .camera-viewport img {
        position: absolute; inset: 0; width: 100%; height: 100%; object-fit: contain; background: #070908; display: none;
      }
      .camera-viewport.has-video video, .camera-viewport.has-image img { display: block; }
      .camera-crosshair { position: absolute; inset: 0; pointer-events: none; display: none; }
      .camera-viewport.show-crosshair .camera-crosshair { display: block; }
      .camera-crosshair::before, .camera-crosshair::after { content: ''; position: absolute; background: var(--accent); opacity: 0.25; }
      .camera-crosshair::before { left: 50%; top: 15%; bottom: 15%; width: 1px; }
      .camera-crosshair::after { top: 50%; left: 20%; right: 20%; height: 1px; }
      .cam-corner-tl, .cam-corner-br { position: absolute; width: 12px; height: 12px; pointer-events: none; }
      .cam-corner-tl { top: 8px; left: 8px; border-top: 2px solid var(--accent); border-left: 2px solid var(--accent); }
      .cam-corner-br { bottom: 8px; right: 8px; border-bottom: 2px solid var(--ink); border-right: 2px solid var(--ink); opacity: 0.6; }
      .url-drawer {
        position: absolute; inset: 0; z-index: 10; background: var(--panel-bg); opacity: 0.98;
        display: none; flex-direction: column; justify-content: center; gap: 10px; padding: 20px;
      }
      .url-drawer.open { display: flex; }
      .url-input { width: 100%; background: var(--screen-bg); border: 1px solid var(--accent); padding: 7px 10px; color: var(--ink); font-family: inherit; font-size: 12px; outline: none; }
      .reg-strip { display: grid; grid-template-columns: repeat(4, 1fr); border-bottom: 1px solid var(--panel-divider); background: var(--panel-bg); }
      .reg-cell { padding: 6px 10px; display: flex; flex-direction: column; gap: 2px; border-right: 1px solid var(--panel-divider); }
      .reg-cell:last-child { border-right: none; }
      .reg-lbl { font-size: 9px; font-weight: 600; color: var(--muted); }
      .reg-val { font-size: 14px; font-weight: 700; line-height: 1.2; color: var(--ink); }
      .reg-val.accent { color: var(--accent); }
      .reg-sub { font-size: 9px; color: var(--dim); }
      .terminal-screen {
        flex: 1; min-height: 0; overflow-y: auto; background: var(--screen-bg);
        padding: 10px 12px; font-size: 11px; line-height: 1.45; color: var(--ink); display: flex; flex-direction: column; gap: 2px;
      }
      .term-log-line { font-family: inherit; white-space: pre-wrap; word-break: break-all; }
      .term-log-line.warn { color: #d97706; }
      .term-log-line.err { color: var(--dot-err); }
      .term-log-line.sys { color: var(--accent); }
      .term-prompt { display: flex; align-items: center; gap: 5px; margin-top: 4px; }
      @keyframes term-blink { 0%, 49% { opacity: 1; } 50%, 100% { opacity: 0; } }
      .term-cursor { display: inline-block; width: 7px; height: 13px; background: var(--accent); animation: term-blink 1.05s steps(1, end) infinite; }
      .right-stack {
        display: flex;
        flex-direction: column;
        gap: 8px;
        min-height: 0;
        height: 100%;
      }
      .terminal-panel {
        flex: 1 1 0;
        min-height: 140px;
        display: flex;
        flex-direction: column;
      }
      .sonar-panel {
        flex-shrink: 0;
        display: flex;
        flex-direction: column;
        background: var(--panel-bg);
      }
      .sonar-grid {
        display: grid;
        grid-template-columns: repeat(4, 1fr);
        gap: 6px;
        padding: 8px 10px;
        background: var(--panel-bg);
      }
      @media (max-width: 700px) {
        .sonar-grid { grid-template-columns: repeat(2, 1fr); }
      }
      .sonar-card {
        background: var(--slot-bg);
        border: 1px solid var(--slot-border);
        padding: 6px 8px;
        display: flex;
        flex-direction: column;
        gap: 4px;
        border-radius: 2px;
        transition: border-color 0.2s, background-color 0.2s;
      }
      .sonar-card.alert {
        border-color: #ef4444;
        background: rgba(239, 68, 68, 0.08);
      }
      .sonar-card.caution {
        border-color: #f59e0b;
        background: rgba(245, 158, 11, 0.06);
      }
      .sonar-card-head {
        display: flex;
        justify-content: space-between;
        align-items: baseline;
        border-bottom: 1px solid var(--panel-divider);
        padding-bottom: 2px;
      }
      .sonar-name {
        font-size: 11px;
        font-weight: 700;
        color: var(--ink);
        letter-spacing: -0.3px;
      }
      .sonar-pos {
        font-size: 8px;
        font-weight: 600;
        color: var(--dim);
      }
      .sonar-card-body {
        display: flex;
        flex-direction: column;
        gap: 3px;
      }
      .sonar-val-wrap {
        display: flex;
        align-items: baseline;
        gap: 3px;
      }
      .sonar-val {
        font-size: 17px;
        font-weight: 700;
        line-height: 1.1;
        color: var(--accent);
        font-variant-numeric: tabular-nums;
      }
      .sonar-card.alert .sonar-val { color: #ef4444; }
      .sonar-card.caution .sonar-val { color: #d97706; }
      .sonar-unit {
        font-size: 9px;
        font-weight: 600;
        color: var(--muted);
      }
      .sonar-bar-track {
        width: 100%;
        height: 4px;
        background: var(--screen-bg);
        border: 1px solid var(--slot-border);
        border-radius: 1px;
        overflow: hidden;
      }
      .sonar-bar {
        height: 100%;
        width: 0%;
        transition: width 0.15s ease, background-color 0.2s;
      }
      .sonar-bar.safe { background: var(--accent-bright); }
      .sonar-bar.caution { background: #f59e0b; }
      .sonar-bar.alert { background: #ef4444; }
      .sonar-card-foot {
        display: flex;
        align-items: center;
        justify-content: space-between;
      }
      .sonar-state {
        font-size: 8px;
        font-weight: 700;
        color: var(--muted);
        text-transform: uppercase;
        letter-spacing: 0.5px;
      }
      .sonar-status-tag {
        font-size: 9px;
        font-weight: 700;
        padding: 2px 6px;
        border-radius: 2px;
        text-transform: uppercase;
        letter-spacing: 0.5px;
      }
      .sonar-status-tag.clear {
        color: var(--accent-bright);
        background: rgba(22, 163, 74, 0.12);
        border: 1px solid rgba(22, 163, 74, 0.3);
      }
      .sonar-status-tag.caution {
        color: #d97706;
        background: rgba(245, 158, 11, 0.12);
        border: 1px solid rgba(245, 158, 11, 0.3);
      }
      .sonar-status-tag.alert {
        color: #ef4444;
        background: rgba(239, 68, 68, 0.15);
        border: 1px solid rgba(239, 68, 68, 0.4);
        animation: pulse-alert 1s infinite;
      }
      .bottom-row { display: flex; align-items: stretch; justify-content: space-between; gap: 8px; flex-shrink: 0; }
      .wasd-panel { flex-shrink: 0; }
      .wasd-grid {
        flex: 1; display: grid; grid-template-columns: repeat(3, clamp(42px, 3.4vw, 48px));
        grid-template-rows: repeat(2, clamp(42px, 3.4vw, 48px)); gap: 6px; padding: 10px; place-content: center;
      }
      .keycap {
        background: var(--slot-bg); border: 1px solid var(--slot-border); color: var(--ink); font-family: inherit;
        cursor: pointer; display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 2px;
      }
      .keycap:hover { border-color: var(--accent); }
      .keycap-letter { font-size: 15px; font-weight: 700; line-height: 1; }
      .keycap-sub { font-size: 9px; line-height: 1; color: var(--muted); }
      .pwm-panel { min-width: 235px; flex-shrink: 0; }
      .pwm-body { flex: 1; padding: 10px; display: flex; flex-direction: column; justify-content: space-between; gap: 8px; }
      .pwm-top { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
      .pwm-big { display: flex; align-items: baseline; gap: 4px; }
      .pwm-big-num { font-size: 22px; font-weight: 700; line-height: 1; }
      .pwm-steppers { display: flex; align-items: center; gap: 4px; }
      .pwm-step-btn {
        width: 24px; height: 24px; background: var(--slot-bg); border: 1px solid var(--slot-border);
        color: var(--ink); font-family: inherit; font-size: 12px; font-weight: 700; cursor: pointer;
        display: flex; align-items: center; justify-content: center;
      }
      .pwm-step-btn:hover { border-color: var(--accent); }
      .pwm-num-input {
        width: 48px; height: 24px; background: var(--screen-bg); border: 1px solid var(--slot-border);
        color: var(--ink); font-family: inherit; font-size: 11px; font-weight: 700; text-align: center; outline: none;
      }
      .pwm-slider-wrap { position: relative; display: flex; align-items: center; }
      .pwm-slider-track { position: absolute; left: 0; right: 0; height: 8px; background: var(--slot-bg); overflow: hidden; pointer-events: none; }
      .pwm-slider-fill { height: 100%; background: var(--accent); }
      input[type='range'].pwm-slider {
        -webkit-appearance: none; appearance: none; position: relative; z-index: 2;
        width: 100%; height: 20px; margin: 0; background: transparent; cursor: pointer;
      }
      input[type='range'].pwm-slider::-webkit-slider-thumb {
        -webkit-appearance: none; appearance: none; width: 12px; height: 18px; margin-top: -6px; background: var(--accent); border: 1px solid var(--ink);
      }
      .pwm-presets { display: grid; grid-template-columns: repeat(5, 1fr); gap: 4px; }
      .pwm-preset-btn { padding: 2px 0; background: var(--slot-bg); border: 1px solid var(--slot-border); color: var(--muted); font-family: inherit; font-size: 10px; font-weight: 600; cursor: pointer; }
      .pwm-preset-btn:hover { border-color: var(--accent); color: var(--ink); }
      .pwm-outputs { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; font-size: 9px; font-weight: 600; color: var(--muted); }
      .pwm-out-row { display: flex; justify-content: space-between; margin-bottom: 2px; }
      .pwm-out-track { height: 6px; border: 1px solid var(--slot-border); background: var(--slot-bg); overflow: hidden; }
      .pwm-out-bar { height: 100%; background: var(--accent); transform-origin: left center; transform: scaleX(0); }
      .gamepad-panel { flex: 1; min-width: 0; }
      .gamepad-deck { flex: 1; display: flex; align-items: center; justify-content: space-between; gap: clamp(8px, 1.2vw, 16px); padding: 10px 12px; }
      .deck-divider { width: 1px; align-self: stretch; background: var(--panel-divider); flex-shrink: 0; }
      .triggers-col { display: flex; align-items: flex-end; gap: 10px; flex-shrink: 0; }
      .trigger-unit { display: flex; flex-direction: column; align-items: center; gap: 4px; cursor: pointer; }
      .trigger-label { font-size: 11px; font-weight: 700; }
      .trigger-meter { width: 24px; height: clamp(62px, 8vh, 80px); background: var(--slot-bg); border: 1px solid var(--slot-border); position: relative; overflow: hidden; }
      .trigger-fill { position: absolute; inset: 0; transform-origin: bottom center; transform: scaleY(0); }
      .trigger-fill.lt { background: linear-gradient(0deg, var(--trigger-lt-from), var(--trigger-lt-to)); }
      .trigger-fill.rt { background: linear-gradient(0deg, var(--trigger-rt-from), var(--trigger-rt-to)); }
      .trigger-val { font-size: 11px; font-weight: 600; }
      .trigger-tag { font-size: 9px; color: var(--muted); }
      .stick-module { display: flex; flex-direction: column; gap: 6px; flex-shrink: 0; }
      .stick-top { display: flex; align-items: center; gap: clamp(8px, 0.9vw, 12px); }
      .stick-axes-col { display: flex; flex-direction: column; gap: 4px; min-width: 78px; }
      .stick-title { font-size: 11px; font-weight: 700; color: var(--accent); display: flex; justify-content: space-between; gap: 6px; }
      .axis-row { display: flex; align-items: center; gap: 6px; }
      .axis-bar-shell { width: 5px; height: 28px; background: var(--slot-bg); border: 1px solid var(--slot-border); position: relative; overflow: hidden; flex-shrink: 0; }
      .axis-bar-fill { position: absolute; left: 0; right: 0; top: 50%; height: 50%; background: var(--accent); transform-origin: top center; transform: scaleY(0); }
      .axis-readout { display: flex; flex-direction: column; line-height: 1.15; }
      .axis-name { font-size: 9px; font-weight: 600; color: var(--muted); }
      .axis-num { font-size: 12px; font-weight: 600; }
      .stick-reticle {
        width: clamp(72px, 8.2vh, 88px); height: clamp(72px, 8.2vh, 88px);
        border-radius: 999px; border: 1px solid var(--slot-border); background: var(--screen-bg);
        position: relative; cursor: crosshair; touch-action: none; flex-shrink: 0;
      }
      .reticle-axis-x, .reticle-axis-y { position: absolute; background: var(--accent); opacity: 0.25; pointer-events: none; }
      .reticle-axis-x { left: 0; right: 0; top: 50%; height: 1px; }
      .reticle-axis-y { top: 0; bottom: 0; left: 50%; width: 1px; }
      .stick-dot {
        position: absolute; left: 50%; top: 50%; width: 10px; height: 10px; margin-left: -5px; margin-top: -5px;
        border-radius: 999px; background: var(--accent-bright); border: 1px solid var(--ink); pointer-events: none;
      }
      .track-foot { display: flex; align-items: center; gap: 8px; font-size: 10px; color: var(--muted); }
      .mini-track { flex: 1; height: 6px; background: var(--slot-bg); border: 1px solid var(--slot-border); position: relative; overflow: hidden; }
      .mini-center-line { position: absolute; left: 50%; top: 0; bottom: 0; width: 1px; background: var(--ink); opacity: 0.35; z-index: 2; }
      .mini-indicator { position: absolute; top: 0; bottom: 0; left: 50%; width: 10px; margin-left: -5px; background: var(--accent); }
      .dpad-module { display: flex; flex-direction: column; gap: 6px; flex-shrink: 0; }
      .dpad-top { display: flex; align-items: center; gap: 10px; }
      .dpad-grid { display: grid; grid-template-columns: repeat(3, 28px); grid-template-rows: repeat(3, 28px); gap: 3px; }
      .dpad-btn {
        background: var(--slot-bg); border: 1px solid var(--slot-border); color: var(--ink);
        font-family: inherit; font-size: 11px; font-weight: 700; cursor: pointer; display: flex; align-items: center; justify-content: center;
      }
      .dpad-btn:hover { border-color: var(--accent); }
      .dpad-up { grid-column: 2; grid-row: 1; }
      .dpad-left { grid-column: 1; grid-row: 2; }
      .dpad-center { grid-column: 2; grid-row: 2; font-size: 9px; color: var(--muted); }
      .dpad-right { grid-column: 3; grid-row: 2; }
      .dpad-down { grid-column: 2; grid-row: 3; }
      .servo-info { display: flex; flex-direction: column; gap: 2px; min-width: 86px; }
      .abxy-pad { display: grid; grid-template-columns: repeat(3, 30px); grid-template-rows: repeat(3, 30px); gap: 4px; place-items: center; flex-shrink: 0; }
      .face-btn {
        width: 30px; height: 30px; border-radius: 999px; background: var(--slot-bg); border: 1px solid var(--slot-border);
        font-family: inherit; font-size: 12px; font-weight: 700; color: var(--ink); cursor: pointer; display: flex; align-items: center; justify-content: center;
      }
      .face-btn:hover { border-color: var(--accent); }
      .face-btn[data-btn="Y"] { grid-column: 2; grid-row: 1; }
      .face-btn[data-btn="X"] { grid-column: 1; grid-row: 2; color: var(--accent); }
      .face-btn[data-btn="B"] { grid-column: 3; grid-row: 2; }
      .face-btn[data-btn="A"] { grid-column: 2; grid-row: 3; color: var(--accent); }
      .active-accent { background: var(--accent) !important; border-color: var(--accent-strong) !important; color: var(--accent-fg) !important; }
      .active-accent .keycap-sub { color: var(--accent-fg); opacity: 0.85; }
      .active-ink { background: var(--ink-active-bg) !important; border-color: var(--ink-active-bg) !important; color: var(--ink-active-fg) !important; }
      .active-ink .keycap-sub { color: var(--ink-active-fg); opacity: 0.8; }
      @media (max-width: 1200px) { .bottom-row { flex-wrap: wrap; } }
      @media (max-width: 960px) { .top-row { grid-template-columns: 1fr; grid-template-rows: 1.3fr 1fr; } }
    </style>
  </head>
  <body>
    <div class="app-shell">
      <header class="titlebar">
        <div class="titlebar-left">
          <span style="color: var(--accent);">&gt;_</span>
          <span id="titlebarHost">rover@esp32-s3:~/dashboard</span>
        </div>
        <div class="titlebar-right">
          <div id="wsStatusPill" class="ws-pill" title="Click to change Rover Host / IP">
            <span id="wsDot" class="status-dot"></span>
            <span id="wsStatusText">WS: CONNECTING...</span>
            <span id="wsPingText" style="color: var(--dim); display: none;">0ms</span>
          </div>
          <button type="button" id="btnThemeToggle" class="term-btn theme-btn">Dark Mode</button>
        </div>
      </header>

      <main class="workspace">
        <div class="top-row">
          <section class="panel" aria-label="ESP32-CAM Feed">
            <div class="panel-head">
              <div class="panel-title">
                <span id="camDot" class="status-dot"></span>
                <span>[esp32-cam]</span>
              </div>
              <div class="btn-row">
                <button type="button" id="btnCamUrl" class="term-btn">Camera URL</button>
                <button type="button" id="btnCamDisconnect" class="term-btn" style="display: none;">Disconnect</button>
                <button type="button" id="btnCamGrid" class="term-btn active">Reticle</button>
              </div>
            </div>

            <div id="camViewport" class="camera-viewport show-crosshair">
              <video id="camVideo" autoplay playsinline muted></video>
              <img id="camImg" alt="ESP32-CAM Stream" />
              <div class="camera-crosshair"></div>
              <div class="cam-corner-tl"></div>
              <div class="cam-corner-br"></div>

              <form id="urlDrawer" class="url-drawer">
                <label for="streamUrlInput" style="font-size: 11px; font-weight: 700; color: var(--accent);">
                  esp32-cam@stream:~$ set-url
                </label>
                <input id="streamUrlInput" class="url-input" type="url" value="http://192.168.4.1:81/stream" />
                <div class="btn-row" style="justify-content: flex-end;">
                  <button type="button" id="btnUrlCancel" class="term-btn">Cancel</button>
                  <button type="submit" class="term-btn active">Connect</button>
                </div>
              </form>
            </div>
          </section>

          <div class="right-stack">
            <section class="panel terminal-panel" aria-label="Rover Terminal">
              <div class="panel-head">
                <div class="panel-title">
                  <span id="padDot" class="status-dot"></span>
                  <span id="padStatusLabel">[esp32-s3 · terminal]</span>
                </div>
                <div class="btn-row">
                  <button type="button" id="btnClearLogs" class="term-btn">Clear Logs</button>
                  <button type="button" id="btnResetInputs" class="term-btn">Reset</button>
                </div>
              </div>

              <div class="reg-strip">
                <div class="reg-cell">
                  <span class="reg-lbl">LEFT_PWM</span>
                  <span id="termLeftPwm" class="reg-val accent">+000%</span>
                  <span id="termLeftRaw" class="reg-sub">RAW +000</span>
                </div>
                <div class="reg-cell">
                  <span class="reg-lbl">RIGHT_PWM</span>
                  <span id="termRightPwm" class="reg-val accent">+000%</span>
                  <span id="termRightRaw" class="reg-sub">RAW +000</span>
                </div>
                <div class="reg-cell">
                  <span class="reg-lbl">PWM_CAP</span>
                  <span id="termPwmCap" class="reg-val">075%</span>
                  <span id="termPwmRaw" class="reg-sub">191/255</span>
                </div>
                <div class="reg-cell">
                  <span class="reg-lbl">CAM_SERVO</span>
                  <span id="termServoDeg" class="reg-val">090°</span>
                  <span class="reg-sub">000–180°</span>
                </div>
              </div>

              <div id="termScreen" class="terminal-screen">
                <div class="term-log-line sys">[SYSTEM] ESP32-S3 Rover Dashboard Active. Connecting WebSocket on port 81...</div>
                <div class="term-prompt">
                  <span style="font-weight: 700; color: var(--accent);">rover@esp32-s3</span>
                  <span style="color: var(--dim);">:</span>
                  <span style="font-weight: 600; color: var(--muted);">~</span>
                  <span style="color: var(--dim);">$</span>
                  <span class="term-cursor" aria-hidden="true"></span>
                </div>
              </div>
            </section>

            <section class="panel sonar-panel" aria-label="Ultrasonic Sonars">
              <div class="panel-head">
                <div class="panel-title">
                  <span id="sonarDot" class="status-dot on"></span>
                  <span>[ultrasonic · 4x hc-sr04]</span>
                </div>
                <div class="btn-row">
                  <span id="sonarAlertBadge" class="sonar-status-tag clear">ALL CLEAR</span>
                </div>
              </div>

              <div class="sonar-grid">
                <!-- Sonar 1: Front Centre (GPIO 10 Trig / GPIO 11 Echo) -->
                <div class="sonar-card" id="cardSonar1">
                  <div class="sonar-card-head">
                    <span class="sonar-name">Sonar_1</span>
                    <span class="sonar-pos">FRONT-CTR</span>
                  </div>
                  <div class="sonar-card-body">
                    <div class="sonar-val-wrap">
                      <span id="valSonar1" class="sonar-val">--</span>
                      <span class="sonar-unit">CM</span>
                    </div>
                    <div class="sonar-bar-track">
                      <div id="barSonar1" class="sonar-bar safe"></div>
                    </div>
                    <div class="sonar-card-foot">
                      <span id="stateSonar1" class="sonar-state">CLEAR</span>
                    </div>
                  </div>
                </div>

                <!-- Sonar 2: Front Left (GPIO 12 Trig / GPIO 13 Echo) -->
                <div class="sonar-card" id="cardSonar2">
                  <div class="sonar-card-head">
                    <span class="sonar-name">Sonar_2</span>
                    <span class="sonar-pos">FRONT-LEFT</span>
                  </div>
                  <div class="sonar-card-body">
                    <div class="sonar-val-wrap">
                      <span id="valSonar2" class="sonar-val">--</span>
                      <span class="sonar-unit">CM</span>
                    </div>
                    <div class="sonar-bar-track">
                      <div id="barSonar2" class="sonar-bar safe"></div>
                    </div>
                    <div class="sonar-card-foot">
                      <span id="stateSonar2" class="sonar-state">CLEAR</span>
                    </div>
                  </div>
                </div>

                <!-- Sonar 3: Front Right (GPIO 14 Trig / GPIO 16 Echo) -->
                <div class="sonar-card" id="cardSonar3">
                  <div class="sonar-card-head">
                    <span class="sonar-name">Sonar_3</span>
                    <span class="sonar-pos">FRONT-RIGHT</span>
                  </div>
                  <div class="sonar-card-body">
                    <div class="sonar-val-wrap">
                      <span id="valSonar3" class="sonar-val">--</span>
                      <span class="sonar-unit">CM</span>
                    </div>
                    <div class="sonar-bar-track">
                      <div id="barSonar3" class="sonar-bar safe"></div>
                    </div>
                    <div class="sonar-card-foot">
                      <span id="stateSonar3" class="sonar-state">CLEAR</span>
                    </div>
                  </div>
                </div>

                <!-- Sonar 4: Rear (GPIO 17 Trig / GPIO 18 Echo) -->
                <div class="sonar-card" id="cardSonar4">
                  <div class="sonar-card-head">
                    <span class="sonar-name">Sonar_4</span>
                    <span class="sonar-pos">REAR</span>
                  </div>
                  <div class="sonar-card-body">
                    <div class="sonar-val-wrap">
                      <span id="valSonar4" class="sonar-val">--</span>
                      <span class="sonar-unit">CM</span>
                    </div>
                    <div class="sonar-bar-track">
                      <div id="barSonar4" class="sonar-bar safe"></div>
                    </div>
                    <div class="sonar-card-foot">
                      <span id="stateSonar4" class="sonar-state">CLEAR</span>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </div>
        </div>

        <div class="bottom-row">
          <section class="panel wasd-panel" aria-label="WASD Keyboard Drive">
            <div class="panel-head">
              <span>[wasd-drive]</span>
              <span id="wasdStateText" style="font-size: 10px; color: var(--muted);">IDLE</span>
            </div>
            <div class="wasd-grid">
              <div></div>
              <button type="button" class="keycap" data-key="KeyW" aria-label="Forward W">
                <span class="keycap-letter">W</span>
                <span class="keycap-sub">FWD</span>
              </button>
              <div></div>
              <button type="button" class="keycap" data-key="KeyA" aria-label="Left A">
                <span class="keycap-letter">A</span>
                <span class="keycap-sub">LEFT</span>
              </button>
              <button type="button" class="keycap" data-key="KeyS" aria-label="Back S">
                <span class="keycap-letter">S</span>
                <span class="keycap-sub">BACK</span>
              </button>
              <button type="button" class="keycap" data-key="KeyD" aria-label="Right D">
                <span class="keycap-letter">D</span>
                <span class="keycap-sub">RIGHT</span>
              </button>
            </div>
          </section>

          <section class="panel pwm-panel" aria-label="PWM Control 0 to 100">
            <div class="panel-head">
              <span>[pwm · 0-100]</span>
              <span id="pwmRawHeader" style="font-size: 10px; color: var(--muted);">191/255</span>
            </div>
            <div class="pwm-body">
              <div class="pwm-top">
                <div class="pwm-big">
                  <span id="pwmBigVal" class="pwm-big-num">75</span>
                  <span style="font-size: 11px; font-weight: 600; color: var(--accent);">%</span>
                </div>
                <div class="pwm-steppers">
                  <button type="button" id="btnPwmDec" class="pwm-step-btn">-</button>
                  <input id="pwmNumInput" class="pwm-num-input" type="number" min="0" max="100" step="1" value="75" />
                  <button type="button" id="btnPwmInc" class="pwm-step-btn">+</button>
                </div>
              </div>

              <div style="display: flex; flex-direction: column; gap: 4px;">
                <div class="pwm-slider-wrap">
                  <div class="pwm-slider-track">
                    <div id="pwmSliderFill" class="pwm-slider-fill" style="width: 75%;"></div>
                  </div>
                  <input id="pwmSlider" class="pwm-slider" type="range" min="0" max="100" step="1" value="75" />
                </div>

                <div class="pwm-presets">
                  <button type="button" class="pwm-preset-btn" data-pwm="0">0</button>
                  <button type="button" class="pwm-preset-btn" data-pwm="25">25</button>
                  <button type="button" class="pwm-preset-btn" data-pwm="50">50</button>
                  <button type="button" class="pwm-preset-btn active-accent" data-pwm="75">75</button>
                  <button type="button" class="pwm-preset-btn" data-pwm="100">100</button>
                </div>
              </div>

              <div class="pwm-outputs">
                <div>
                  <div class="pwm-out-row"><span>L_OUT</span><span id="pwmLeftVal">0%</span></div>
                  <div class="pwm-out-track"><div id="pwmLeftBar" class="pwm-out-bar"></div></div>
                </div>
                <div>
                  <div class="pwm-out-row"><span>R_OUT</span><span id="pwmRightVal">0%</span></div>
                  <div class="pwm-out-track"><div id="pwmRightBar" class="pwm-out-bar"></div></div>
                </div>
              </div>
            </div>
          </section>

          <section class="panel gamepad-panel" aria-label="Gamepad Drive">
            <div class="panel-head">
              <span>[gamepad-drive]</span>
            </div>

            <div class="gamepad-deck">
              <div class="triggers-col">
                <div class="trigger-unit" id="btnLT" role="button" tabindex="0">
                  <span class="trigger-label">LT</span>
                  <div class="trigger-meter"><div id="ltFill" class="trigger-fill lt"></div></div>
                  <span id="ltVal" class="trigger-val">0.00</span>
                  <span class="trigger-tag">REV</span>
                </div>
                <div class="trigger-unit" id="btnRT" role="button" tabindex="0">
                  <span class="trigger-label" style="color: var(--accent);">RT</span>
                  <div class="trigger-meter"><div id="rtFill" class="trigger-fill rt"></div></div>
                  <span id="rtVal" class="trigger-val">0.00</span>
                  <span class="trigger-tag">THROT</span>
                </div>
              </div>

              <div class="deck-divider"></div>

              <div class="stick-module">
                <div class="stick-top">
                  <div class="stick-axes-col">
                    <div class="stick-title"><span>L_STICK</span><span style="font-size: 9px; color: var(--muted);">STEER</span></div>
                    <div class="axis-row">
                      <div class="axis-bar-shell"><div id="axis0Bar" class="axis-bar-fill"></div></div>
                      <div class="axis-readout"><span class="axis-name">AXIS_0</span><span id="axis0Num" class="axis-num">0.00000</span></div>
                    </div>
                    <div class="axis-row">
                      <div class="axis-bar-shell"><div id="axis1Bar" class="axis-bar-fill"></div></div>
                      <div class="axis-readout"><span class="axis-name">AXIS_1</span><span id="axis1Num" class="axis-num">0.00000</span></div>
                    </div>
                  </div>
                  <div id="lStickReticle" class="stick-reticle" title="Drag with mouse or finger to steer">
                    <div class="reticle-axis-x"></div>
                    <div class="reticle-axis-y"></div>
                    <div id="lStickDot" class="stick-dot"></div>
                  </div>
                </div>
                <div class="track-foot">
                  <span>STEER</span>
                  <div class="mini-track">
                    <div class="mini-center-line"></div>
                    <div id="lSteerMini" class="mini-indicator"></div>
                  </div>
                  <span id="lSteerPct" style="min-width: 38px; text-align: right; color: var(--accent); font-weight: 600;">0%</span>
                </div>
              </div>

              <div class="deck-divider"></div>

              <div class="dpad-module">
                <div class="dpad-top">
                  <div class="dpad-grid">
                    <button type="button" class="dpad-btn dpad-up" data-dpad="UP" aria-label="Center Servo">▲</button>
                    <button type="button" class="dpad-btn dpad-left" data-dpad="LEFT" aria-label="Pan Left">◀</button>
                    <button type="button" class="dpad-btn dpad-center" id="btnCenterServo" title="Center Camera Servo to 90°">90°</button>
                    <button type="button" class="dpad-btn dpad-right" data-dpad="RIGHT" aria-label="Pan Right">▶</button>
                    <button type="button" class="dpad-btn dpad-down" data-dpad="DOWN" aria-label="D-Pad Down">▼</button>
                  </div>
                  <div class="servo-info">
                    <span style="font-size: 11px; font-weight: 700; color: var(--accent);">D_PAD</span>
                    <span style="font-size: 10px; color: var(--muted);">CAM_SERVO</span>
                    <span id="servoAngleLabel" style="font-size: 14px; font-weight: 700;">090°</span>
                    <span style="font-size: 9px; color: var(--dim);">PAN: ◀ ▶</span>
                  </div>
                </div>
                <div class="track-foot">
                  <span>0°</span>
                  <div class="mini-track">
                    <div class="mini-center-line"></div>
                    <div id="servoBarDot" class="mini-indicator" style="left: 50%;"></div>
                  </div>
                  <span>180°</span>
                </div>
              </div>

              <div class="deck-divider"></div>

              <div class="abxy-pad">
                <button type="button" class="face-btn" data-btn="Y">Y</button>
                <button type="button" class="face-btn" data-btn="X">X</button>
                <button type="button" class="face-btn" data-btn="B">B</button>
                <button type="button" class="face-btn" data-btn="A">A</button>
              </div>
            </div>
          </section>
        </div>
      </main>
    </div>

    <script>
      (() => {
        const DEADZONE = 0.04;
        const SERVO_SPEED = 110;
        const PWM_BUMPER_SPEED = 55;
        const WS_RATE_MS = 40; // 25Hz command transmit

        const clamp = (v, min, max) => (v < min ? min : v > max ? max : v);
        const dz = (v) => (Math.abs(v) < DEADZONE ? 0 : v);
        const fmtSigned = (n) =>
          n >= 0
            ? `+${String(Math.round(n)).padStart(3, '0')}`
            : `-${String(Math.abs(Math.round(n))).padStart(3, '0')}`;

        const keyState = { KeyW: false, KeyA: false, KeyS: false, KeyD: false };
        const virtualKey = { KeyW: false, KeyA: false, KeyS: false, KeyD: false };
        const virtualBtn = { A: false, B: false, X: false, Y: false, LT: false, RT: false };
        const virtualDpad = { UP: false, DOWN: false, LEFT: false, RIGHT: false };
        const virtualL = { x: 0, y: 0, dragging: false };

        let camServoAngle = 90;
        let pwmLimit = 75;
        let pwmFloat = 75;
        let prevLB = false;
        let prevRB = false;

        let cmdLeftPct = 0;
        let cmdRightPct = 0;
        let telemLeftPct = 0;
        let telemRightPct = 0;
        let telemLeftRaw = 0;
        let telemRightRaw = 0;
        let telemEstop = false;

        const el = {
          titlebarHost: document.getElementById('titlebarHost'),
          wsStatusPill: document.getElementById('wsStatusPill'),
          wsDot: document.getElementById('wsDot'),
          wsStatusText: document.getElementById('wsStatusText'),
          wsPingText: document.getElementById('wsPingText'),
          btnThemeToggle: document.getElementById('btnThemeToggle'),
          camViewport: document.getElementById('camViewport'),
          camVideo: document.getElementById('camVideo'),
          camImg: document.getElementById('camImg'),
          camDot: document.getElementById('camDot'),
          btnCamUrl: document.getElementById('btnCamUrl'),
          btnCamDisconnect: document.getElementById('btnCamDisconnect'),
          btnCamGrid: document.getElementById('btnCamGrid'),
          urlDrawer: document.getElementById('urlDrawer'),
          streamUrlInput: document.getElementById('streamUrlInput'),
          btnUrlCancel: document.getElementById('btnUrlCancel'),

          padDot: document.getElementById('padDot'),
          padStatusLabel: document.getElementById('padStatusLabel'),
          btnClearLogs: document.getElementById('btnClearLogs'),
          btnResetInputs: document.getElementById('btnResetInputs'),
          btnCenterServo: document.getElementById('btnCenterServo'),
          termScreen: document.getElementById('termScreen'),

          termLeftPwm: document.getElementById('termLeftPwm'),
          termLeftRaw: document.getElementById('termLeftRaw'),
          termRightPwm: document.getElementById('termRightPwm'),
          termRightRaw: document.getElementById('termRightRaw'),
          termPwmCap: document.getElementById('termPwmCap'),
          termPwmRaw: document.getElementById('termPwmRaw'),
          termServoDeg: document.getElementById('termServoDeg'),
          termWifiLink: document.getElementById('termWifiLink'),
          termWifiRssi: document.getElementById('termWifiRssi'),
          wasdStateText: document.getElementById('wasdStateText'),

          pwmRawHeader: document.getElementById('pwmRawHeader'),
          pwmBigVal: document.getElementById('pwmBigVal'),
          pwmNumInput: document.getElementById('pwmNumInput'),
          pwmSlider: document.getElementById('pwmSlider'),
          pwmSliderFill: document.getElementById('pwmSliderFill'),
          btnPwmDec: document.getElementById('btnPwmDec'),
          btnPwmInc: document.getElementById('btnPwmInc'),
          pwmLeftVal: document.getElementById('pwmLeftVal'),
          pwmRightVal: document.getElementById('pwmRightVal'),
          pwmLeftBar: document.getElementById('pwmLeftBar'),
          pwmRightBar: document.getElementById('pwmRightBar'),

          ltFill: document.getElementById('ltFill'),
          ltVal: document.getElementById('ltVal'),
          rtFill: document.getElementById('rtFill'),
          rtVal: document.getElementById('rtVal'),
          btnLT: document.getElementById('btnLT'),
          btnRT: document.getElementById('btnRT'),

          axis0Bar: document.getElementById('axis0Bar'),
          axis0Num: document.getElementById('axis0Num'),
          axis1Bar: document.getElementById('axis1Bar'),
          axis1Num: document.getElementById('axis1Num'),
          lStickReticle: document.getElementById('lStickReticle'),
          lStickDot: document.getElementById('lStickDot'),
          lSteerMini: document.getElementById('lSteerMini'),
          lSteerPct: document.getElementById('lSteerPct'),

          servoAngleLabel: document.getElementById('servoAngleLabel'),
          servoBarDot: document.getElementById('servoBarDot'),

          cardSonar1: document.getElementById('cardSonar1'),
          cardSonar2: document.getElementById('cardSonar2'),
          cardSonar3: document.getElementById('cardSonar3'),
          cardSonar4: document.getElementById('cardSonar4'),
          valSonar1: document.getElementById('valSonar1'),
          valSonar2: document.getElementById('valSonar2'),
          valSonar3: document.getElementById('valSonar3'),
          valSonar4: document.getElementById('valSonar4'),
          barSonar1: document.getElementById('barSonar1'),
          barSonar2: document.getElementById('barSonar2'),
          barSonar3: document.getElementById('barSonar3'),
          barSonar4: document.getElementById('barSonar4'),
          stateSonar1: document.getElementById('stateSonar1'),
          stateSonar2: document.getElementById('stateSonar2'),
          stateSonar3: document.getElementById('stateSonar3'),
          stateSonar4: document.getElementById('stateSonar4'),
          sonarAlertBadge: document.getElementById('sonarAlertBadge'),
        };

        const appendTermLog = (msg, level = 'info') => {
          const prompt = el.termScreen.querySelector('.term-prompt');
          const line = document.createElement('div');
          line.className = `term-log-line ${level}`;
          const time = new Date().toTimeString().split(' ')[0];
          line.textContent = `[${time}] ${msg}`;
          if (prompt) el.termScreen.insertBefore(line, prompt);
          else el.termScreen.appendChild(line);
          el.termScreen.scrollTop = el.termScreen.scrollHeight;
        };

        el.btnClearLogs.addEventListener('click', () => {
          el.termScreen.querySelectorAll('.term-log-line').forEach(l => l.remove());
          appendTermLog('Terminal cleared.', 'sys');
        });

        const applyTheme = (isDark) => {
          document.documentElement.classList.toggle('dark', isDark);
          el.btnThemeToggle.textContent = isDark ? 'Light Mode' : 'Dark Mode';
          try { localStorage.setItem('rover-theme', isDark ? 'dark' : 'light'); } catch {}
        };
        let darkMode = false;
        try { darkMode = localStorage.getItem('rover-theme') === 'dark'; } catch {}
        applyTheme(darkMode);
        el.btnThemeToggle.addEventListener('click', () => { darkMode = !darkMode; applyTheme(darkMode); });

        // Auto-detect host from URL if served directly by ESP32-S3!
        let wsHost = window.location.hostname;
        if (!wsHost || wsHost === 'localhost' || wsHost === '127.0.0.1') {
          wsHost = localStorage.getItem('rover-ws-host') || 'rover.local';
        }
        el.titlebarHost.textContent = `rover@esp32-s3 [${wsHost}]`;

        let ws = null;
        let wsConnected = false;
        let pingStartTime = 0;
        let httpOnline = false;

        const updateWsStatus = (connected, text) => {
          wsConnected = connected;
          el.wsDot.classList.toggle('on', connected);
          el.wsDot.classList.toggle('err', !connected);
          el.wsStatusText.textContent = text || (connected ? `ONLINE` : `OFFLINE`);
          el.wsPingText.style.display = connected ? 'inline' : 'none';
          if (el.termWifiLink) {
            el.termWifiLink.textContent = connected ? 'ONLINE' : 'DISCONNECTED';
            el.termWifiLink.style.color = connected ? 'var(--accent)' : 'var(--dot-err)';
          }
        };

        const updateSonarUI = (s1, s2, s3, s4) => {
          const readings = [s1, s2, s3, s4];
          let anyAlert = false;
          let anyCaution = false;

          for (let i = 0; i < 4; i++) {
            const d = Number(readings[i]);
            const valEl = el[`valSonar${i + 1}`];
            const barEl = el[`barSonar${i + 1}`];
            const cardEl = el[`cardSonar${i + 1}`];
            const stateEl = el[`stateSonar${i + 1}`];
            if (!valEl || !barEl || !cardEl || !stateEl) continue;

            if (isNaN(d) || d < 0 || d > 250) {
              valEl.textContent = (d > 250) ? '>250' : '--';
              barEl.style.width = '0%';
              barEl.className = 'sonar-bar safe';
              cardEl.className = 'sonar-card';
              stateEl.textContent = 'CLEAR';
              stateEl.style.color = 'var(--muted)';
            } else {
              valEl.textContent = d;
              // Proximity: closer distance = higher percentage bar (0cm = 100%, 250cm = 0%)
              const proxPct = Math.max(0, Math.min(100, Math.round(((250 - d) / 250) * 100)));
              barEl.style.width = `${proxPct}%`;

              if (d <= 20) {
                anyAlert = true;
                cardEl.className = 'sonar-card alert';
                barEl.className = 'sonar-bar alert';
                stateEl.textContent = 'ALERT <20CM';
                stateEl.style.color = '#ef4444';
              } else if (d <= 50) {
                anyCaution = true;
                cardEl.className = 'sonar-card caution';
                barEl.className = 'sonar-bar caution';
                stateEl.textContent = 'CAUTION';
                stateEl.style.color = '#d97706';
              } else {
                cardEl.className = 'sonar-card';
                barEl.className = 'sonar-bar safe';
                stateEl.textContent = 'CLEAR';
                stateEl.style.color = 'var(--accent-bright)';
              }
            }
          }

          if (el.sonarAlertBadge) {
            if (anyAlert) {
              el.sonarAlertBadge.textContent = 'OBSTACLE ALERT';
              el.sonarAlertBadge.className = 'sonar-status-tag alert';
            } else if (anyCaution) {
              el.sonarAlertBadge.textContent = 'CAUTION NEAR';
              el.sonarAlertBadge.className = 'sonar-status-tag caution';
            } else {
              el.sonarAlertBadge.textContent = 'ALL CLEAR';
              el.sonarAlertBadge.className = 'sonar-status-tag clear';
            }
          }
        };

        const connectWebSocket = () => {
          if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
          const wsUrl = `ws://${wsHost}:81/ws`;
          if (!httpOnline) updateWsStatus(false, `CONNECTING...`);
          try {
            ws = new WebSocket(wsUrl);
            ws.onopen = () => {
              updateWsStatus(true, `WS: CONNECTED`);
              appendTermLog(`Control WebSocket connected on port 81!`, 'sys');
              ws.send('T');
            };
            ws.onmessage = (evt) => {
              const msg = evt.data;
              if (typeof msg !== 'string') return;
              if (msg.startsWith('T,')) {
                const parts = msg.split(',');
                if (parts.length >= 6) {
                  telemLeftPct = parseInt(parts[1], 10) || 0;
                  telemRightPct = parseInt(parts[2], 10) || 0;
                  telemLeftRaw = parseInt(parts[3], 10) || 0;
                  telemRightRaw = parseInt(parts[4], 10) || 0;
                  telemEstop = parts[6] === '1';
                  if (parts.length >= 11) {
                    const s1 = parseInt(parts[7], 10);
                    const s2 = parseInt(parts[8], 10);
                    const s3 = parseInt(parts[9], 10);
                    const s4 = parseInt(parts[10], 10);
                    updateSonarUI(s1, s2, s3, s4);
                  }
                  if (pingStartTime > 0) {
                    const rtt = Math.round(performance.now() - pingStartTime);
                    el.wsPingText.textContent = `${rtt}ms`;
                    if (el.termWifiRssi) el.termWifiRssi.textContent = `PING: ${rtt}ms`;
                  }
                }
              } else if (msg.startsWith('L,')) {
                const parts = msg.split(',');
                appendTermLog(parts.slice(2).join(','), parts[1] || 'info');
              } else if (msg === 'PONG') {
                if (pingStartTime > 0) {
                  const rtt = Math.round(performance.now() - pingStartTime);
                  el.wsPingText.textContent = `${rtt}ms`;
                  if (el.termWifiRssi) el.termWifiRssi.textContent = `PING: ${rtt}ms`;
                }
              }
            };
            ws.onclose = () => {
              if (!httpOnline) updateWsStatus(false, `CONNECTING...`);
              setTimeout(connectWebSocket, 2000);
            };
            ws.onerror = () => {
              try { ws.close(); } catch {}
            };
          } catch (e) {
            setTimeout(connectWebSocket, 2500);
          }
        };

        el.wsStatusPill.addEventListener('click', () => {
          const nextHost = prompt('Enter ESP32-S3 Rover IP or hostname (e.g. 192.168.1.150 or rover.local):', wsHost);
          if (nextHost && nextHost.trim()) {
            wsHost = nextHost.trim();
            localStorage.setItem('rover-ws-host', wsHost);
            el.titlebarHost.textContent = `rover@esp32-s3 [${wsHost}]`;
            if (ws) { try { ws.close(); } catch {} }
            connectWebSocket();
          }
        });
        connectWebSocket();

        let httpDriveInFlight = false;
        let lastHttpDriveTime = 0;
        let firstHttpLogShown = false;
        let lastReceivedLogId = 0;
        let isFetchingLogs = false;

        const fetchRoverLogs = () => {
          if (isFetchingLogs) return;
          isFetchingLogs = true;
          fetch(`/logs?after=${lastReceivedLogId}`)
            .then((r) => r.json())
            .then((data) => {
              isFetchingLogs = false;
              if (data && data.logs && Array.isArray(data.logs)) {
                data.logs.forEach((item) => {
                  if (item.id > lastReceivedLogId) {
                    lastReceivedLogId = item.id;
                    appendTermLog(item.msg, item.lvl || 'info');
                  }
                });
              }
            })
            .catch(() => { isFetchingLogs = false; });
        };
        setInterval(fetchRoverLogs, 600);
        setTimeout(fetchRoverLogs, 150);

        const sendDrivePacket = () => {
          if (ws && ws.readyState === WebSocket.OPEN) {
            // High-speed WebSocket mode (Port 81)
            const payload = `D,${Math.round(cmdLeftPct)},${Math.round(cmdRightPct)},${pwmLimit},0`;
            ws.send(payload);
            pingStartTime = performance.now();
          } else {
            // Instant, bulletproof HTTP Drive mode (Port 80)
            const now = performance.now();
            if (httpDriveInFlight && (now - lastHttpDriveTime < 90)) return;
            httpDriveInFlight = true;
            lastHttpDriveTime = now;
            const t0 = now;
            fetch(`/drive?l=${Math.round(cmdLeftPct)}&r=${Math.round(cmdRightPct)}&pwm=${pwmLimit}`)
              .then((r) => r.json())
              .then((data) => {
                httpDriveInFlight = false;
                httpOnline = true;
                telemLeftPct = data.l || 0;
                telemRightPct = data.r || 0;
                telemLeftRaw = data.rl || 0;
                telemRightRaw = data.rr || 0;
                telemEstop = data.es === 1;
                if (data.s1 !== undefined) {
                  updateSonarUI(data.s1, data.s2, data.s3, data.s4);
                }
                const rtt = Math.round(performance.now() - t0);
                if (!firstHttpLogShown) {
                  firstHttpLogShown = true;
                  appendTermLog(`[LINK] Active on ESP32-S3 REST API (Port 80 · ${rtt}ms latency)`, 'sys');
                  fetchRoverLogs();
                }
                if (!ws || ws.readyState !== WebSocket.OPEN) {
                  updateWsStatus(true, `HTTP: ONLINE`);
                  el.wsPingText.style.display = 'inline';
                  el.wsPingText.textContent = `${rtt}ms`;
                  if (el.termWifiRssi) el.termWifiRssi.textContent = `PING: ${rtt}ms`;
                }
                if (data.lid && data.lid > lastReceivedLogId) {
                  fetchRoverLogs();
                }
              })
              .catch(() => {
                httpDriveInFlight = false;
                httpOnline = false;
                if (!ws || ws.readyState !== WebSocket.OPEN) {
                  updateWsStatus(false, `CONNECTING...`);
                }
              });
          }
        };
        setInterval(sendDrivePacket, WS_RATE_MS);

        const pollSonar = () => {
          if (ws && ws.readyState === WebSocket.OPEN) return;
          fetch('/sonar')
            .then((r) => r.json())
            .then((data) => {
              if (data && data.s1 !== undefined) {
                updateSonarUI(data.s1, data.s2, data.s3, data.s4);
              }
            })
            .catch(() => {});
        };
        setInterval(pollSonar, 400);

        const presetBtns = Array.from(document.querySelectorAll('.pwm-preset-btn[data-pwm]'));
        const renderPwmControls = (val) => {
          const raw = Math.round((val / 100) * 255);
          el.pwmBigVal.textContent = String(val);
          el.pwmNumInput.value = String(val);
          el.pwmSlider.value = String(val);
          el.pwmSliderFill.style.width = `${val}%`;
          el.pwmRawHeader.textContent = `${raw}/255`;
          el.termPwmCap.textContent = `${String(val).padStart(3, '0')}%`;
          el.termPwmRaw.textContent = `${raw}/255`;
          presetBtns.forEach((btn) => {
            const p = Number(btn.getAttribute('data-pwm'));
            btn.classList.toggle('active-ink', p === 0 && val === 0);
            btn.classList.toggle('active-accent', p !== 0 && val === p);
          });
        };

        let lastLoggedPwm = 75;
        const setPwm = (nextVal) => {
          pwmLimit = clamp(Math.round(nextVal), 0, 100);
          pwmFloat = pwmLimit;
          renderPwmControls(pwmLimit);
          syncVisuals(0);
          sendDrivePacket();
          if (Math.abs(pwmLimit - lastLoggedPwm) >= 5) {
            lastLoggedPwm = pwmLimit;
            appendTermLog(`[PWM] Power ceiling set to ${pwmLimit}% (${Math.round(pwmLimit * 2.55)}/255)`, 'sys');
          }
        };

        window.addEventListener('gamepadconnected', (e) => {
          appendTermLog(`[PAD] Connected: ${e.gamepad.id.slice(0, 32)}`, 'sys');
        });
        window.addEventListener('gamepaddisconnected', (e) => {
          appendTermLog(`[PAD] Disconnected: ${e.gamepad.id.slice(0, 32)}`, 'warn');
        });

        el.pwmSlider.addEventListener('input', (e) => setPwm(Number(e.target.value)));
        el.pwmNumInput.addEventListener('input', (e) => {
          const parsed = parseInt(e.target.value, 10);
          setPwm(Number.isNaN(parsed) ? 0 : parsed);
        });
        el.btnPwmDec.addEventListener('click', () => setPwm(pwmLimit - 5));
        el.btnPwmInc.addEventListener('click', () => setPwm(pwmLimit + 5));
        presetBtns.forEach(btn => btn.addEventListener('click', () => setPwm(Number(btn.getAttribute('data-pwm')))));

        const keyEls = {};
        document.querySelectorAll('.keycap[data-key]').forEach((node) => {
          const k = node.getAttribute('data-key');
          keyEls[k] = node;
          node.addEventListener('pointerdown', (e) => { e.preventDefault(); virtualKey[k] = true; syncVisuals(0); });
          const clear = () => { if (virtualKey[k]) { virtualKey[k] = false; syncVisuals(0); } };
          node.addEventListener('pointerup', clear);
          node.addEventListener('pointerleave', clear);
          node.addEventListener('pointercancel', clear);
        });

        const faceEls = {};
        document.querySelectorAll('.face-btn[data-btn]').forEach((node) => {
          const b = node.getAttribute('data-btn');
          faceEls[b] = node;
          node.addEventListener('pointerdown', (e) => { e.preventDefault(); virtualBtn[b] = true; syncVisuals(0); });
          const clear = () => { if (virtualBtn[b]) { virtualBtn[b] = false; syncVisuals(0); } };
          node.addEventListener('pointerup', clear);
          node.addEventListener('pointerleave', clear);
          node.addEventListener('pointercancel', clear);
        });

        const dpadEls = {};
        document.querySelectorAll('.dpad-btn[data-dpad]').forEach((node) => {
          const d = node.getAttribute('data-dpad');
          dpadEls[d] = node;
          node.addEventListener('pointerdown', (e) => { e.preventDefault(); virtualDpad[d] = true; syncVisuals(0); });
          const clear = () => { if (virtualDpad[d]) { virtualDpad[d] = false; syncVisuals(0); } };
          node.addEventListener('pointerup', clear);
          node.addEventListener('pointerleave', clear);
          node.addEventListener('pointercancel', clear);
        });

        el.btnCenterServo.addEventListener('click', () => {
          camServoAngle = 90;
          syncVisuals(0);
        });

        const bindTrigger = (node, key) => {
          node.addEventListener('pointerdown', (e) => { e.preventDefault(); virtualBtn[key] = true; syncVisuals(0); });
          const clear = () => { if (virtualBtn[key]) { virtualBtn[key] = false; syncVisuals(0); } };
          node.addEventListener('pointerup', clear);
          node.addEventListener('pointerleave', clear);
          node.addEventListener('pointercancel', clear);
        };
        bindTrigger(el.btnLT, 'LT');
        bindTrigger(el.btnRT, 'RT');

        const updateLFromPointer = (clientX, clientY) => {
          const rect = el.lStickReticle.getBoundingClientRect();
          const r = rect.width / 2;
          const dx = (clientX - (rect.left + r)) / r;
          const dy = (clientY - (rect.top + r)) / r;
          const mag = Math.hypot(dx, dy);
          if (mag > 1) { virtualL.x = dx / mag; virtualL.y = dy / mag; }
          else { virtualL.x = dx; virtualL.y = dy; }
          syncVisuals(0);
        };

        el.lStickReticle.addEventListener('pointerdown', (e) => {
          e.preventDefault();
          el.lStickReticle.setPointerCapture(e.pointerId);
          virtualL.dragging = true;
          updateLFromPointer(e.clientX, e.clientY);
        });
        el.lStickReticle.addEventListener('pointermove', (e) => { if (virtualL.dragging) updateLFromPointer(e.clientX, e.clientY); });
        const releaseL = () => { virtualL.dragging = false; virtualL.x = 0; virtualL.y = 0; syncVisuals(0); };
        el.lStickReticle.addEventListener('pointerup', releaseL);
        el.lStickReticle.addEventListener('pointercancel', releaseL);

        window.addEventListener('keydown', (e) => {
          if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
          if (e.code in keyState) {
            e.preventDefault();
            if (!keyState[e.code]) { keyState[e.code] = true; syncVisuals(0); }
          } else if (e.key === '[' || e.key === '-') {
            e.preventDefault(); setPwm(pwmLimit - 5);
          } else if (e.key === ']' || e.key === '=' || e.key === '+') {
            e.preventDefault(); setPwm(pwmLimit + 5);
          }
        });
        window.addEventListener('keyup', (e) => {
          if (e.code in keyState) { keyState[e.code] = false; syncVisuals(0); }
        });

        const disconnectStream = () => {
          el.camVideo.removeAttribute('src'); el.camVideo.load();
          el.camImg.removeAttribute('src');
          el.camViewport.classList.remove('has-video', 'has-image');
          el.camDot.classList.remove('on');
          el.btnCamDisconnect.style.display = 'none';
        };
        el.camImg.addEventListener('error', () => { if (el.camViewport.classList.contains('has-image')) disconnectStream(); });
        el.btnCamUrl.addEventListener('click', () => {
          const open = el.urlDrawer.classList.toggle('open');
          el.btnCamUrl.classList.toggle('active', open);
          if (open) el.streamUrlInput.focus();
        });
        el.btnUrlCancel.addEventListener('click', () => {
          el.urlDrawer.classList.remove('open'); el.btnCamUrl.classList.remove('active');
        });
        el.btnCamDisconnect.addEventListener('click', disconnectStream);
        el.urlDrawer.addEventListener('submit', (e) => {
          e.preventDefault();
          const url = el.streamUrlInput.value.trim();
          if (!url) return;
          disconnectStream();
          el.urlDrawer.classList.remove('open'); el.btnCamUrl.classList.remove('active');
          const lower = url.toLowerCase();
          if (lower.endsWith('.mp4') || lower.endsWith('.webm') || lower.endsWith('.ogg')) {
            el.camVideo.src = url; el.camViewport.classList.add('has-video');
          } else {
            el.camImg.src = url; el.camViewport.classList.add('has-image');
          }
          el.camDot.classList.add('on');
          el.btnCamDisconnect.style.display = 'inline-flex';
        });
        el.btnCamGrid.addEventListener('click', () => {
          el.camViewport.classList.toggle('show-crosshair');
          el.btnCamGrid.classList.toggle('active');
        });

        el.btnResetInputs.addEventListener('click', () => {
          Object.keys(keyState).forEach(k => { keyState[k] = false; virtualKey[k] = false; });
          Object.keys(virtualBtn).forEach(b => virtualBtn[b] = false);
          Object.keys(virtualDpad).forEach(d => virtualDpad[d] = false);
          virtualL.x = 0; virtualL.y = 0; virtualL.dragging = false;
          camServoAngle = 90;
          setPwm(75);
          syncVisuals(0);
          if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send('CMD:RESET');
          } else {
            fetch('/reset').catch(() => {});
          }
          appendTermLog('Sent RESET command to ESP32-S3', 'sys');
        });

        const updateAxisBar = (barEl, val) => {
          const mag = clamp(Math.abs(val), 0, 1);
          if (val >= 0) {
            barEl.style.top = '50%'; barEl.style.transformOrigin = 'top center'; barEl.style.transform = `scaleY(${mag})`;
          } else {
            barEl.style.top = `${50 - mag * 50}%`; barEl.style.transformOrigin = 'bottom center'; barEl.style.transform = `scaleY(${mag})`;
          }
        };

        function syncVisuals(dt) {
          const pads = navigator.getGamepads ? navigator.getGamepads() : [];
          let activePad = null;
          for (let i = 0; i < pads.length; i++) {
            if (pads[i] && pads[i].connected) { activePad = pads[i]; break; }
          }

          const wOn = keyState.KeyW || virtualKey.KeyW;
          const aOn = keyState.KeyA || virtualKey.KeyA;
          const sOn = keyState.KeyS || virtualKey.KeyS;
          const dOn = keyState.KeyD || virtualKey.KeyD;

          keyEls.KeyW.classList.toggle('active-accent', wOn);
          keyEls.KeyA.classList.toggle('active-accent', aOn);
          keyEls.KeyS.classList.toggle('active-ink', sOn);
          keyEls.KeyD.classList.toggle('active-accent', dOn);

          const wasdLabels = [];
          if (wOn) wasdLabels.push('FWD');
          if (sOn) wasdLabels.push('BACK');
          if (aOn) wasdLabels.push('LEFT');
          if (dOn) wasdLabels.push('RIGHT');
          el.wasdStateText.textContent = wasdLabels.length ? wasdLabels.join('·') : 'IDLE';

          const rawAx0 = activePad && activePad.axes.length > 0 ? activePad.axes[0] : 0;
          const rawAx1 = activePad && activePad.axes.length > 1 ? activePad.axes[1] : 0;
          const ax0 = virtualL.dragging ? virtualL.x : rawAx0;
          const ax1 = virtualL.dragging ? virtualL.y : rawAx1;

          const padLT = activePad && activePad.buttons[6] ? activePad.buttons[6].value : 0;
          const padRT = activePad && activePad.buttons[7] ? activePad.buttons[7].value : 0;
          const ltVal = virtualBtn.LT ? 1 : padLT;
          const rtVal = virtualBtn.RT ? 1 : padRT;

          const btnA = virtualBtn.A || Boolean(activePad && activePad.buttons[0] && activePad.buttons[0].pressed);
          const btnB = virtualBtn.B || Boolean(activePad && activePad.buttons[1] && activePad.buttons[1].pressed);
          const btnX = virtualBtn.X || Boolean(activePad && activePad.buttons[2] && activePad.buttons[2].pressed);
          const btnY = virtualBtn.Y || Boolean(activePad && activePad.buttons[3] && activePad.buttons[3].pressed);

          faceEls.A.classList.toggle('active-accent', btnA);
          faceEls.X.classList.toggle('active-accent', btnX);
          faceEls.B.classList.toggle('active-ink', btnB);
          faceEls.Y.classList.toggle('active-ink', btnY);

          const dUp = virtualDpad.UP || Boolean(activePad && activePad.buttons[12] && activePad.buttons[12].pressed);
          const dDown = virtualDpad.DOWN || Boolean(activePad && activePad.buttons[13] && activePad.buttons[13].pressed);
          const dLeft = virtualDpad.LEFT || Boolean(activePad && activePad.buttons[14] && activePad.buttons[14].pressed);
          const dRight = virtualDpad.RIGHT || Boolean(activePad && activePad.buttons[15] && activePad.buttons[15].pressed);

          dpadEls.UP.classList.toggle('active-accent', dUp);
          dpadEls.DOWN.classList.toggle('active-accent', dDown);
          dpadEls.LEFT.classList.toggle('active-accent', dLeft);
          dpadEls.RIGHT.classList.toggle('active-accent', dRight);

          if (dt > 0) {
            if (dLeft) camServoAngle = clamp(camServoAngle - SERVO_SPEED * dt, 0, 180);
            if (dRight) camServoAngle = clamp(camServoAngle + SERVO_SPEED * dt, 0, 180);
          }
          if (dUp) camServoAngle = 90;

          const lbPressed = Boolean(activePad && activePad.buttons[4] && activePad.buttons[4].pressed);
          const rbPressed = Boolean(activePad && activePad.buttons[5] && activePad.buttons[5].pressed);
          if (lbPressed && !rbPressed) {
            if (!prevLB) pwmFloat = clamp(pwmFloat - 1, 0, 100);
            else if (dt > 0) pwmFloat = clamp(pwmFloat - PWM_BUMPER_SPEED * dt, 0, 100);
          } else if (rbPressed && !lbPressed) {
            if (!prevRB) pwmFloat = clamp(pwmFloat + 1, 0, 100);
            else if (dt > 0) pwmFloat = clamp(pwmFloat + PWM_BUMPER_SPEED * dt, 0, 100);
          }
          prevLB = lbPressed; prevRB = rbPressed;

          const nextPwmInt = Math.round(pwmFloat);
          if (nextPwmInt !== pwmLimit) { pwmLimit = nextPwmInt; renderPwmControls(pwmLimit); }

          el.ltFill.style.transform = `scaleY(${clamp(ltVal, 0, 1)})`;
          el.ltVal.textContent = ltVal.toFixed(2);
          el.rtFill.style.transform = `scaleY(${clamp(rtVal, 0, 1)})`;
          el.rtVal.textContent = rtVal.toFixed(2);

          el.axis0Num.textContent = ax0.toFixed(5);
          el.axis1Num.textContent = ax1.toFixed(5);
          updateAxisBar(el.axis0Bar, ax0);
          updateAxisBar(el.axis1Bar, ax1);

          const lRad = Math.max(22, el.lStickReticle.clientWidth / 2 - 6);
          el.lStickDot.style.transform = `translate3d(${(ax0 * lRad).toFixed(1)}px, ${(ax1 * lRad).toFixed(1)}px, 0)`;

          const throttle = clamp(Math.max(rtVal, wOn ? 1 : 0), 0, 1);
          const reverse = clamp(Math.max(ltVal, sOn ? 1 : 0), 0, 1);
          const netDrive = throttle - reverse;
          const keySteer = (dOn ? 1 : 0) - (aOn ? 1 : 0);
          const steer = clamp(dz(ax0) + keySteer, -1, 1);

          const sumMag = Math.abs(netDrive) + Math.abs(steer);
          const turnScale = sumMag > 1.0 ? 1.0 / sumMag : 1.0;

          // Inverted steering differential: Left on stick turns Left, Right turns Right
          cmdLeftPct = Math.round((netDrive - steer) * turnScale * pwmLimit);
          cmdRightPct = Math.round((netDrive + steer) * turnScale * pwmLimit);

          const steerPct = Math.round(steer * 100);
          el.lSteerPct.textContent = steerPct === 0 ? '0%' : steerPct < 0 ? `L${Math.abs(steerPct)}%` : `R${steerPct}%`;

          const miniHalf = Math.max(12, (el.lSteerMini.parentElement.clientWidth - 10) / 2);
          el.lSteerMini.style.transform = `translate3d(${(steer * miniHalf).toFixed(1)}px, 0, 0)`;

          const servoStr = `${String(Math.round(camServoAngle)).padStart(3, '0')}°`;
          const servoNorm = (camServoAngle - 90) / 90;
          const servoHalf = Math.max(12, (el.servoBarDot.parentElement.clientWidth - 10) / 2);
          el.servoBarDot.style.transform = `translate3d(${(servoNorm * servoHalf).toFixed(1)}px, 0, 0)`;

          if (el.servoAngleLabel) el.servoAngleLabel.textContent = servoStr;
          if (el.termServoDeg) el.termServoDeg.textContent = servoStr;

          const displayLeftPct = wsConnected ? telemLeftPct : cmdLeftPct;
          const displayRightPct = wsConnected ? telemRightPct : cmdRightPct;
          const displayLeftRaw = wsConnected ? telemLeftRaw : Math.round((cmdLeftPct / 100) * 255);
          const displayRightRaw = wsConnected ? telemRightRaw : Math.round((cmdRightPct / 100) * 255);

          el.termLeftPwm.textContent = `${fmtSigned(displayLeftPct)}%`;
          el.termLeftRaw.textContent = `RAW ${fmtSigned(displayLeftRaw)}`;
          el.termRightPwm.textContent = `${fmtSigned(displayRightPct)}%`;
          el.termRightRaw.textContent = `RAW ${fmtSigned(displayRightRaw)}`;

          el.pwmLeftBar.style.transform = `scaleX(${Math.abs(displayLeftPct) / 100})`;
          el.pwmLeftBar.style.backgroundColor = displayLeftPct < 0 ? 'var(--ink)' : 'var(--accent)';
          el.pwmRightBar.style.transform = `scaleX(${Math.abs(displayRightPct) / 100})`;
          el.pwmRightBar.style.backgroundColor = displayRightPct < 0 ? 'var(--ink)' : 'var(--accent)';
          el.pwmLeftVal.textContent = `${displayLeftPct < 0 ? '-' : ''}${Math.abs(displayLeftPct)}%`;
          el.pwmRightVal.textContent = `${displayRightPct < 0 ? '-' : ''}${Math.abs(displayRightPct)}%`;

          if (activePad) {
            el.padDot.classList.add('on');
            el.padStatusLabel.textContent = '[esp32-s3 · pad online]';
          } else {
            el.padDot.classList.remove('on');
            el.padStatusLabel.textContent = '[esp32-s3 · terminal]';
          }
        }

        renderPwmControls(pwmLimit);
        let lastFrameTime = performance.now();
        const frame = (now) => {
          const dt = Math.min(0.05, (now - lastFrameTime) / 1000);
          lastFrameTime = now;
          syncVisuals(dt);
          requestAnimationFrame(frame);
        };
        requestAnimationFrame(frame);
      })();
    </script>
  </body>
</html>

)rawliteral";

// =============================================================================
// 3. LOGGING BUFFER (Streams Serial Monitor events to Dashboard Terminal)
// =============================================================================
#define MAX_LOG_LINES 32
struct LogEntry {
  uint32_t id;
  char text[128];
  char level[8]; // "sys", "info", "warn", "err"
};

LogEntry logBuffer[MAX_LOG_LINES];
int logHead = 0;
uint32_t nextLogId = 1;

class RoverWebSocketServer;
extern RoverWebSocketServer wsServer;

void roverLog(const char* level, const char* msg);

// =============================================================================
// 4. SELF-CONTAINED CRYPTO (SHA-1 & Base64 for RFC6455 WebSocket)
// =============================================================================
namespace TinyCrypto {
  static uint32_t rol(uint32_t value, size_t bits) { return (value << bits) | (value >> (32 - bits)); }
  static void sha1(const uint8_t* data, size_t len, uint8_t hash[20]) {
    uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0;
    size_t totalBits = len * 8, paddedLen = len + 1 + 8;
    while (paddedLen % 64 != 0) paddedLen++;
    uint8_t* p = (uint8_t*)malloc(paddedLen);
    if (!p) return;
    memset(p, 0, paddedLen);
    memcpy(p, data, len);
    p[len] = 0x80;
    for (int i = 0; i < 8; i++) p[paddedLen - 1 - i] = (uint8_t)((totalBits >> (i * 8)) & 0xFF);
    for (size_t chunk = 0; chunk < paddedLen; chunk += 64) {
      uint32_t w[80];
      for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)p[chunk + i * 4] << 24) | ((uint32_t)p[chunk + i * 4 + 1] << 16) |
               ((uint32_t)p[chunk + i * 4 + 2] << 8) | ((uint32_t)p[chunk + i * 4 + 3]);
      }
      for (int i = 16; i < 80; i++) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
      uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
      for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20) { f = (b & c) | ((~b) & d); k = 0x5A827999; }
        else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
        else { f = b ^ c ^ d; k = 0xCA62C1D6; }
        uint32_t temp = rol(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rol(b, 30); b = a; a = temp;
      }
      h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }
    free(p);
    uint32_t h[5] = { h0, h1, h2, h3, h4 };
    for (int i = 0; i < 5; i++) {
      hash[i * 4]     = (uint8_t)(h[i] >> 24);
      hash[i * 4 + 1] = (uint8_t)(h[i] >> 16);
      hash[i * 4 + 2] = (uint8_t)(h[i] >> 8);
      hash[i * 4 + 3] = (uint8_t)(h[i]);
    }
  }
  static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  static String base64_encode(const uint8_t* in, size_t len) {
    String out = "";
    int val = 0, valb = -6;
    for (size_t i = 0; i < len; i++) {
      val = (val << 8) + in[i]; valb += 8;
      while (valb >= 0) { out += b64_table[(val >> valb) & 0x3F]; valb -= 6; }
    }
    if (valb > -6) out += b64_table[((val << 8) >> (valb + 8)) & 0x3F];
    while (out.length() % 4) out += '=';
    return out;
  }
}

// =============================================================================
// 5. BTS7960 MOTOR CONTROLLER CLASS (With Anti-Burn Turning Slew Limiter)
// =============================================================================
#define CH_LEFT_RPWM   0
#define CH_LEFT_LPWM   1
#define CH_RIGHT_RPWM  2
#define CH_RIGHT_LPWM  3

static void ledc_init_pin(int pin, int channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pin, channel);
#endif
}

static void ledc_write_duty(int pin, int channel, int duty) {
  duty = constrain(duty, 0, PWM_MAX_RAW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

class MotorController {
public:
  MotorController()
    : _targetLeftRaw(0), _targetRightRaw(0), _currLeftRaw(0), _currRightRaw(0),
      _pwmLimit(75), _lastCmdTime(0), _lastUpdateTime(0), _leftZeroTime(0), _rightZeroTime(0),
      _estopLatched(false), _watchdogTripped(false) {}

  void begin() {
    pinMode(PIN_LEFT_RPWM, OUTPUT); digitalWrite(PIN_LEFT_RPWM, LOW);
    pinMode(PIN_LEFT_LPWM, OUTPUT); digitalWrite(PIN_LEFT_LPWM, LOW);
    pinMode(PIN_RIGHT_RPWM, OUTPUT); digitalWrite(PIN_RIGHT_RPWM, LOW);
    pinMode(PIN_RIGHT_LPWM, OUTPUT); digitalWrite(PIN_RIGHT_LPWM, LOW);

    ledc_init_pin(PIN_LEFT_RPWM, CH_LEFT_RPWM);
    ledc_init_pin(PIN_LEFT_LPWM, CH_LEFT_LPWM);
    ledc_init_pin(PIN_RIGHT_RPWM, CH_RIGHT_RPWM);
    ledc_init_pin(PIN_RIGHT_LPWM, CH_RIGHT_LPWM);

    _lastCmdTime = millis();
    _lastUpdateTime = millis();
    roverLog("sys", "[MOTORS] BTS7960 4-Pin Driver Armed: L(GPIO 4,5), R(GPIO 6,7)");
  }

  void setDrive(int leftPct, int rightPct, int pwmLimit) {
    if (_estopLatched) { _targetLeftRaw = 0; _targetRightRaw = 0; return; }
    _lastCmdTime = millis();
    _watchdogTripped = false;

    _pwmLimit = constrain(pwmLimit, 0, 100);
    float capFactor = (float)_pwmLimit / 100.0f;
    leftPct = constrain(leftPct, -100, 100);
    rightPct = constrain(rightPct, -100, 100);

    if (MOTOR_LEFT_INVERT) leftPct = -leftPct;
    if (MOTOR_RIGHT_INVERT) rightPct = -rightPct;

    // TURNING PROTECTION: Derate power during spin turns to prevent 25GA motor overheating
    bool isSpinTurn = (leftPct * rightPct < 0);
    float turnMult = isSpinTurn ? SPIN_TURN_DERATE : 1.0f;

    float scaledL = (float)leftPct * capFactor * turnMult;
    float scaledR = (float)rightPct * capFactor * turnMult;

    _targetLeftRaw = (int)round((scaledL / 100.0f) * (float)PWM_MAX_RAW);
    _targetRightRaw = (int)round((scaledR / 100.0f) * (float)PWM_MAX_RAW);
  }

  void update() {
    unsigned long now = millis();
    if (now - _lastUpdateTime < 10) return; // 100 Hz loop
    _lastUpdateTime = now;

    if (_estopLatched) { _currLeftRaw = 0; _currRightRaw = 0; applyHardwarePwm(0, 0); return; }

    // Failsafe watchdog
    if ((now - _lastCmdTime) > WATCHDOG_TIMEOUT_MS) {
      if (!_watchdogTripped) {
        _watchdogTripped = true;
        roverLog("warn", "[WATCHDOG] Inactive >450ms -> Motors stopped.");
      }
      _targetLeftRaw = 0;
      _targetRightRaw = 0;
    }

    // Left Motor Ramping & Deadtime
    if (_targetLeftRaw != _currLeftRaw) {
      if ((_currLeftRaw > 0 && _targetLeftRaw < 0) || (_currLeftRaw < 0 && _targetLeftRaw > 0)) {
        if (_currLeftRaw > 0) _currLeftRaw = max(0, _currLeftRaw - MOTOR_SLEW_STEP);
        else _currLeftRaw = min(0, _currLeftRaw + MOTOR_SLEW_STEP);
        if (_currLeftRaw == 0) _leftZeroTime = now;
      } else {
        if (_currLeftRaw == 0 && (now - _leftZeroTime) < DIRECTION_DEADTIME_MS) { /* deadtime hold */ }
        else {
          int d = constrain(_targetLeftRaw - _currLeftRaw, -MOTOR_SLEW_STEP, MOTOR_SLEW_STEP);
          _currLeftRaw += d;
        }
      }
    }

    // Right Motor Ramping & Deadtime
    if (_targetRightRaw != _currRightRaw) {
      if ((_currRightRaw > 0 && _targetRightRaw < 0) || (_currRightRaw < 0 && _targetRightRaw > 0)) {
        if (_currRightRaw > 0) _currRightRaw = max(0, _currRightRaw - MOTOR_SLEW_STEP);
        else _currRightRaw = min(0, _currRightRaw + MOTOR_SLEW_STEP);
        if (_currRightRaw == 0) _rightZeroTime = now;
      } else {
        if (_currRightRaw == 0 && (now - _rightZeroTime) < DIRECTION_DEADTIME_MS) { /* deadtime hold */ }
        else {
          int d = constrain(_targetRightRaw - _currRightRaw, -MOTOR_SLEW_STEP, MOTOR_SLEW_STEP);
          _currRightRaw += d;
        }
      }
    }

    int outL = (abs(_currLeftRaw) < MOTOR_DEADBAND_RAW) ? 0 : _currLeftRaw;
    int outR = (abs(_currRightRaw) < MOTOR_DEADBAND_RAW) ? 0 : _currRightRaw;
    applyHardwarePwm(outL, outR);
  }

  void emergencyStop(const char* reason = "E-STOP") {
    _estopLatched = true;
    _targetLeftRaw = 0; _targetRightRaw = 0;
    _currLeftRaw = 0; _currRightRaw = 0;
    applyHardwarePwm(0, 0);
    char buf[64];
    snprintf(buf, sizeof(buf), "[E-STOP] %s -> 0 PWM", reason);
    roverLog("err", buf);
  }

  void resetEmergencyStop() {
    _estopLatched = false; _watchdogTripped = false;
    _targetLeftRaw = 0; _targetRightRaw = 0;
    _currLeftRaw = 0; _currRightRaw = 0;
    _lastCmdTime = millis();
    applyHardwarePwm(0, 0);
    roverLog("sys", "[E-STOP] Cleared and Re-Armed.");
  }

  int getActualLeftPct() const { return (int)round(((float)_currLeftRaw / 255.0f) * 100.0f); }
  int getActualRightPct() const { return (int)round(((float)_currRightRaw / 255.0f) * 100.0f); }
  int getActualLeftRaw() const { return _currLeftRaw; }
  int getActualRightRaw() const { return _currRightRaw; }
  int getPwmCap() const { return _pwmLimit; }
  bool isEmergencyStopped() const { return _estopLatched; }

private:
  int _targetLeftRaw, _targetRightRaw, _currLeftRaw, _currRightRaw, _pwmLimit;
  unsigned long _lastCmdTime, _lastUpdateTime, _leftZeroTime, _rightZeroTime;
  bool _estopLatched, _watchdogTripped;

  void applyHardwarePwm(int leftRaw, int rightRaw) {
    writeMotorPins(PIN_LEFT_RPWM, PIN_LEFT_LPWM, leftRaw);
    writeMotorPins(PIN_RIGHT_RPWM, PIN_RIGHT_LPWM, rightRaw);
  }

  void writeMotorPins(int pinRPWM, int pinLPWM, int rawVal) {
    int chRPWM = (pinRPWM == PIN_LEFT_RPWM) ? CH_LEFT_RPWM : CH_RIGHT_RPWM;
    int chLPWM = (pinLPWM == PIN_LEFT_LPWM) ? CH_LEFT_LPWM : CH_RIGHT_LPWM;
    if (rawVal > 0) {
      ledc_write_duty(pinLPWM, chLPWM, 0);
      ledc_write_duty(pinRPWM, chRPWM, rawVal);
    } else if (rawVal < 0) {
      ledc_write_duty(pinRPWM, chRPWM, 0);
      ledc_write_duty(pinLPWM, chLPWM, abs(rawVal));
    } else {
      ledc_write_duty(pinRPWM, chRPWM, 0);
      ledc_write_duty(pinLPWM, chLPWM, 0);
    }
  }
};

MotorController motors;

// =============================================================================
// 6. ULTRASONIC SENSOR MANAGER (Non-Blocking Round-Robin HC-SR04 Driver)
// =============================================================================
class SonarManager {
public:
  SonarManager() : _currentSensor(0), _lastPingTime(0) {
    for (int i = 0; i < SONAR_COUNT; i++) {
      _distances[i] = -1;
      _lastAlertState[i] = false;
    }
    _trigPins[0] = PIN_SONAR1_TRIG; _echoPins[0] = PIN_SONAR1_ECHO;
    _trigPins[1] = PIN_SONAR2_TRIG; _echoPins[1] = PIN_SONAR2_ECHO;
    _trigPins[2] = PIN_SONAR3_TRIG; _echoPins[2] = PIN_SONAR3_ECHO;
    _trigPins[3] = PIN_SONAR4_TRIG; _echoPins[3] = PIN_SONAR4_ECHO;
  }

  void begin() {
    for (int i = 0; i < SONAR_COUNT; i++) {
      pinMode(_trigPins[i], OUTPUT);
      digitalWrite(_trigPins[i], LOW);
      pinMode(_echoPins[i], INPUT);
    }
  }

  void update() {
    unsigned long now = millis();
    if (now - _lastPingTime >= SONAR_PING_INTERVAL_MS) {
      _lastPingTime = now;
      measureSensor(_currentSensor);
      _currentSensor = (_currentSensor + 1) % SONAR_COUNT;
    }
  }

  int getDistance(int index) const {
    if (index >= 0 && index < SONAR_COUNT) {
      return _distances[index];
    }
    return -1;
  }

private:
  uint8_t _trigPins[SONAR_COUNT];
  uint8_t _echoPins[SONAR_COUNT];
  int _distances[SONAR_COUNT];
  bool _lastAlertState[SONAR_COUNT];
  int _currentSensor;
  unsigned long _lastPingTime;

  void measureSensor(int idx) {
    uint8_t trig = _trigPins[idx];
    uint8_t echo = _echoPins[idx];

    // 10us HIGH pulse
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // Read echo pulse with timeout (max ~14.5 ms)
    unsigned long duration = pulseIn(echo, HIGH, SONAR_TIMEOUT_US);
    int dist = -1;
    if (duration > 0) {
      dist = (int)(duration / 58);
      if (dist < 2 || dist > SONAR_MAX_DIST_CM) {
        dist = -1;
      }
    }
    _distances[idx] = dist;

    // Check obstacle close alert (< 20 cm)
    bool isClose = (dist > 0 && dist <= OBSTACLE_ALERT_DIST_CM);
    if (isClose && !_lastAlertState[idx]) {
      _lastAlertState[idx] = true;
      char alertBuf[80];
      const char* sName = (idx == 0) ? "Sonar_1 (Center)" :
                          (idx == 1) ? "Sonar_2 (Left)" :
                          (idx == 2) ? "Sonar_3 (Right)" : "Sonar_4 (Rear)";
      snprintf(alertBuf, sizeof(alertBuf), "[ALERT] %s OBSTACLE CLOSE: %d cm!", sName, dist);
      roverLog("warn", alertBuf);
    } else if (!isClose && _lastAlertState[idx]) {
      _lastAlertState[idx] = false;
    }
  }
};

SonarManager sonars;

// =============================================================================
// 7. EMBEDDED DUAL PROTOCOL: HIGH-SPEED WEBSOCKET (Port 81) & REST API (Port 80)
// =============================================================================
class RoverWebSocketServer {
public:
  RoverWebSocketServer(uint16_t port = WS_PORT) 
    : _server(port), _handshakeDone(false), _port(port), _handshakeStartTime(0), _handshakeBuffer("") {}

  void begin() {
    _server.begin();
    _server.setNoDelay(true);
    roverLog("sys", String("[WS] WebSocket Server listening on port " + String(_port)).c_str());
  }

  void update() {
    if (_server.hasClient()) {
      WiFiClient incoming = _server.available();
      if (incoming) {
        if (!bool(_client) || (_client.connected() == 0)) {
          _client = incoming;
          _client.setNoDelay(true);
          _handshakeDone = false;
          _handshakeBuffer = "";
          _handshakeStartTime = millis();
          roverLog("info", "[WS] New client connected on port 81");
        } else {
          incoming.stop();
        }
      }
    }

    if (!bool(_client) || (_client.connected() == 0)) {
      _handshakeDone = false;
      return;
    }

    if (!_handshakeDone) {
      while (_client.available() > 0) {
        char c = (char)_client.read();
        _handshakeBuffer += c;
        if (_handshakeBuffer.endsWith("\r\n\r\n")) {
          if (doHandshake(_handshakeBuffer)) {
            _handshakeDone = true;
            _handshakeBuffer = "";
            roverLog("sys", "[WS] Control WebSocket connected on port 81! S3 Ready.");
          } else {
            roverLog("warn", "[WS] WebSocket handshake failed.");
            _client.stop();
            _handshakeDone = false;
            _handshakeBuffer = "";
          }
          break;
        }
      }
      if (!_handshakeDone && (millis() - _handshakeStartTime > 3000)) {
        roverLog("warn", "[WS] Handshake timeout.");
        _client.stop();
        _handshakeDone = false;
        _handshakeBuffer = "";
      }
      return;
    }

    while (_client.available() >= 2) {
      processFrame();
    }
  }

  void sendTelemetry(int leftPct, int rightPct, int leftRaw, int rightRaw, int s1 = -1, int s2 = -1, int s3 = -1, int s4 = -1) {
    char buf[96];
    snprintf(buf, sizeof(buf), "T,%d,%d,%d,%d,12.0,%d,%d,%d,%d,%d",
             leftPct, rightPct, leftRaw, rightRaw,
             motors.isEmergencyStopped() ? 1 : 0,
             s1, s2, s3, s4);
    sendText(buf);
  }

  void logToDashboard(const char* level, const char* text) {
    char buf[256];
    snprintf(buf, sizeof(buf), "L,%s,%s", level ? level : "info", text ? text : "");
    sendText(buf);
  }

  bool hasClient() { return _handshakeDone && bool(_client) && (_client.connected() != 0); }

private:
  WiFiServer _server;
  WiFiClient _client;
  bool _handshakeDone;
  uint16_t _port;
  unsigned long _handshakeStartTime;
  String _handshakeBuffer;

  bool doHandshake(const String& req) {
    String reqLower = req;
    reqLower.toLowerCase();
    int idx = reqLower.indexOf("sec-websocket-key:");
    if (idx == -1) return false;
    int lineEnd = req.indexOf("\r\n", idx);
    if (lineEnd == -1) lineEnd = req.indexOf("\n", idx);
    if (lineEnd == -1) return false;

    String key = req.substring(idx + 18, lineEnd);
    key.trim();
    if (key.length() == 0) return false;

    String combined = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    uint8_t hash[20];
    TinyCrypto::sha1((const uint8_t*)combined.c_str(), combined.length(), hash);
    String acceptKey = TinyCrypto::base64_encode(hash, 20);

    String resp = "HTTP/1.1 101 Switching Protocols\r\n"
                  "Upgrade: websocket\r\n"
                  "Connection: Upgrade\r\n"
                  "Sec-WebSocket-Accept: " + acceptKey + "\r\n\r\n";
    _client.print(resp);
    _client.flush();
    return true;
  }

  void processFrame() {
    uint8_t b0 = _client.read(), b1 = _client.read();
    uint8_t opcode = b0 & 0x0F;
    bool isMasked = (b1 & 0x80) != 0;
    uint64_t len = b1 & 0x7F;

    if (len == 126) {
      unsigned long w = millis();
      while (_client.available() < 2 && _client.connected() && (millis() - w < 50)) delay(1);
      if (_client.available() < 2) return;
      len = ((uint16_t)_client.read() << 8) | _client.read();
    } else if (len == 127) {
      _client.stop();
      _handshakeDone = false;
      return;
    }

    uint8_t mask[4] = {0};
    if (isMasked) {
      unsigned long w = millis();
      while (_client.available() < 4 && _client.connected() && (millis() - w < 50)) delay(1);
      if (_client.available() < 4) return;
      for (int i = 0; i < 4; i++) mask[i] = _client.read();
    }

    String msg = "";
    msg.reserve((size_t)len + 1);
    unsigned long waitStart = millis();
    while ((size_t)msg.length() < len && _client.connected() && (millis() - waitStart < 50)) {
      while (_client.available() > 0 && (size_t)msg.length() < len) {
        size_t i = msg.length();
        uint8_t c = _client.read();
        if (isMasked) c ^= mask[i % 4];
        msg += (char)c;
      }
      if ((size_t)msg.length() < len) delay(1);
    }

    if (opcode == 0x01) { // Text
      if (msg.startsWith("D,")) {
        int c1 = msg.indexOf(',', 2);
        int c2 = msg.indexOf(',', c1 + 1);
        int c3 = msg.indexOf(',', c2 + 1);
        if (c1 != -1 && c2 != -1) {
          int l = msg.substring(2, c1).toInt();
          int r = msg.substring(c1 + 1, c2).toInt();
          int p = (c3 != -1) ? msg.substring(c2 + 1, c3).toInt() : 75;
          motors.setDrive(l, r, p);
        }
      } else if (msg == "CMD:RESET") {
        motors.resetEmergencyStop();
      } else if (msg == "PING" || msg == "T") {
        sendText("PONG");
      }
    } else if (opcode == 0x08) {
      _client.stop();
      _handshakeDone = false;
    }
  }

  void sendText(const char* text) {
    if (!bool(_client) || (_client.connected() == 0) || !_handshakeDone || !text) return;
    size_t len = strlen(text);
    uint8_t header[4];
    size_t hLen = 2;
    header[0] = 0x81;
    if (len < 126) { header[1] = (uint8_t)len; }
    else { header[1] = 126; header[2] = (uint8_t)(len >> 8); header[3] = (uint8_t)(len & 0xFF); hLen = 4; }
    _client.write(header, hLen);
    _client.write((const uint8_t*)text, len);
    _client.flush();
  }
};

RoverWebSocketServer wsServer(WS_PORT);
WebServer httpServer(HTTP_PORT);

void roverLog(const char* level, const char* msg) {
  Serial.println(msg);

  LogEntry& entry = logBuffer[logHead];
  entry.id = nextLogId++;
  strncpy(entry.level, level ? level : "info", sizeof(entry.level) - 1);
  entry.level[sizeof(entry.level) - 1] = '\0';
  strncpy(entry.text, msg ? msg : "", sizeof(entry.text) - 1);
  entry.text[sizeof(entry.text) - 1] = '\0';
  logHead = (logHead + 1) % MAX_LOG_LINES;

  if (wsServer.hasClient()) {
    wsServer.logToDashboard(level, msg);
  }
}

unsigned long lastTelemTime = 0;
unsigned long lastStatusPrintTime = 0;
bool isApMode = false;

void handleRoot() {
  httpServer.sendHeader("Connection", "close");
  httpServer.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleDrive() {
  if (httpServer.hasArg("l") && httpServer.hasArg("r")) {
    int l = httpServer.arg("l").toInt();
    int r = httpServer.arg("r").toInt();
    int pwm = httpServer.hasArg("pwm") ? httpServer.arg("pwm").toInt() : 75;
    motors.setDrive(l, r, pwm);
  }
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"l\":%d,\"r\":%d,\"rl\":%d,\"rr\":%d,\"pwm\":%d,\"es\":%d,\"s1\":%d,\"s2\":%d,\"s3\":%d,\"s4\":%d,\"lid\":%u}",
           motors.getActualLeftPct(), motors.getActualRightPct(),
           motors.getActualLeftRaw(), motors.getActualRightRaw(),
           motors.getPwmCap(), motors.isEmergencyStopped() ? 1 : 0,
           sonars.getDistance(0), sonars.getDistance(1),
           sonars.getDistance(2), sonars.getDistance(3),
           nextLogId - 1);
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", buf);
}

void handleSonar() {
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"s1\":%d,\"s2\":%d,\"s3\":%d,\"s4\":%d}",
           sonars.getDistance(0), sonars.getDistance(1),
           sonars.getDistance(2), sonars.getDistance(3));
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", buf);
}

void handleLogs() {
  uint32_t afterId = httpServer.hasArg("after") ? httpServer.arg("after").toInt() : 0;
  String json = "{\"logs\":[";
  bool first = true;
  for (int i = 0; i < MAX_LOG_LINES; i++) {
    int idx = (logHead + i) % MAX_LOG_LINES;
    if (logBuffer[idx].id > afterId && logBuffer[idx].text[0] != '\0') {
      if (!first) json += ",";
      first = false;
      json += "{\"id\":" + String(logBuffer[idx].id) +
              ",\"lvl\":\"" + String(logBuffer[idx].level) +
              "\",\"msg\":\"" + String(logBuffer[idx].text) + "\"}";
    }
  }
  json += "]}";
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", json);
}

void handleReset() {
  motors.resetEmergencyStop();
  roverLog("sys", "[E-STOP] Reset & Re-Armed via Dashboard");
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "text/plain", "OK");
}

void handleStatus() {
  String json = "{\"uptime\":" + String(millis()) +
                ",\"left_pwm\":" + String(motors.getActualLeftPct()) +
                ",\"right_pwm\":" + String(motors.getActualRightPct()) +
                ",\"pwm_cap\":" + String(motors.getPwmCap()) +
                ",\"rssi\":" + String(WiFi.RSSI()) + "}";
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", json);
}

// =============================================================================
// 7. MAIN ARDUINO SETUP & LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================================");
  Serial.println("  ECO-ECO ROVER · WIRELESS ESP32-S3 DIRECT DRIVE       ");
  Serial.println("  Left Motor  : GPIO 4 (RPWM), GPIO 5 (LPWM)           ");
  Serial.println("  Right Motor : GPIO 6 (RPWM), GPIO 7 (LPWM)           ");
  Serial.println("  Driver EN   : Hardwired to 5V (Always Active)        ");
  Serial.println("========================================================");

  // 1. Initialize Motors
  motors.begin();

  // 2. Initialize 4x Ultrasonic Sonars (HC-SR04)
  sonars.begin();

  roverLog("sys", "[BOOT] ESP32-S3 Firmware Initialized");
  roverLog("sys", "[MOTORS] BTS7960 Drivers Armed on GPIO 4, 5, 6, 7");
  roverLog("sys", "[SONAR] 4x HC-SR04 Active: S1(10,11) S2(12,13) S3(14,16) S4(17,18)");

  // 3. Connect Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.printf("[WIFI] Connecting to network \"%s\"...\n", WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < WIFI_CONNECT_TIMEOUT) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    isApMode = false;
    roverLog("sys", String("[WIFI] Connected to SSID: \"" + String(WIFI_SSID) + "\"").c_str());
    roverLog("sys", String("[WIFI] Rover IP: " + WiFi.localIP().toString()).c_str());
  } else {
    isApMode = true;
    WiFi.mode(WIFI_AP);
    IPAddress apIP(AP_IP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    roverLog("warn", String("[WIFI] SoftAP Active: \"" + String(AP_SSID) + "\" @ " + WiFi.softAPIP().toString()).c_str());
  }

  // 3. Start mDNS (http://rover.local)
  if (MDNS.begin(MDNS_HOSTNAME)) {
    roverLog("sys", String("[mDNS] Responder Active: http://" + String(MDNS_HOSTNAME) + ".local").c_str());
    MDNS.addService("http", "tcp", HTTP_PORT);
    MDNS.addService("ws", "tcp", WS_PORT);
  }

  // 5. Start Web Server & WebSocket Server
  httpServer.on("/", HTTP_GET, handleRoot);
  httpServer.on("/drive", HTTP_GET, handleDrive);
  httpServer.on("/sonar", HTTP_GET, handleSonar);
  httpServer.on("/logs", HTTP_GET, handleLogs);
  httpServer.on("/reset", HTTP_GET, handleReset);
  httpServer.on("/status", HTTP_GET, handleStatus);
  httpServer.begin();

  wsServer.begin();

  roverLog("sys", "[SERVER] HTTP REST API active on Port 80");
  roverLog("sys", "[READY] Rover is online & ready to drive!");
  Serial.println("========================================================\n");
}

void loop() {
  unsigned long now = millis();

  // Handle incoming HTTP browser requests (serves dashboard & REST motor drive)
  httpServer.handleClient();

  // Handle high-speed WebSocket motor control packets (25Hz)
  wsServer.update();

  // Update Motor PWMs (100 Hz / every 10ms with slew-rate limiter & deadtime)
  motors.update();

  // Update Non-blocking Ultrasonic Sensors (round-robin sweep across 4 sensors)
  sonars.update();

  // Stream drive state changes to Dashboard Terminal & Serial Monitor
  static int lastLoggedLeft = 0;
  static int lastLoggedRight = 0;
  int curL = motors.getActualLeftPct();
  int curR = motors.getActualRightPct();
  if (abs(curL - lastLoggedLeft) >= 20 || abs(curR - lastLoggedRight) >= 20 || 
      (curL == 0 && curR == 0 && (lastLoggedLeft != 0 || lastLoggedRight != 0))) {
    lastLoggedLeft = curL;
    lastLoggedRight = curR;
    if (curL == 0 && curR == 0) {
      roverLog("info", "[DRIVE] Motors Idle / Stopped (0%)");
    } else {
      char dBuf[80];
      const char* act = (curL > 0 && curR > 0) ? "FWD" : 
                        (curL < 0 && curR < 0) ? "REV" : 
                        (curL > curR) ? "RIGHT" : "LEFT";
      snprintf(dBuf, sizeof(dBuf), "[DRIVE] %s (L:%+d%% R:%+d%% Cap:%d%%)", act, curL, curR, motors.getPwmCap());
      roverLog("info", dBuf);
    }
  }

  // Send real-time telemetry to the dashboard via WebSocket if connected
  if (now - lastTelemTime >= 40) {
    lastTelemTime = now;
    if (wsServer.hasClient()) {
      wsServer.sendTelemetry(
        curL,
        curR,
        motors.getActualLeftRaw(),
        motors.getActualRightRaw(),
        sonars.getDistance(0),
        sonars.getDistance(1),
        sonars.getDistance(2),
        sonars.getDistance(3)
      );
    }
  }

  // Stream periodic Sonar telemetry to Dashboard Terminal & Serial Monitor (every 3s)
  static unsigned long lastSonarPrintTime = 0;
  if (now - lastSonarPrintTime >= 3000) {
    lastSonarPrintTime = now;
    int s1 = sonars.getDistance(0);
    int s2 = sonars.getDistance(1);
    int s3 = sonars.getDistance(2);
    int s4 = sonars.getDistance(3);
    char sBuf[128];
    char v1[12], v2[12], v3[12], v4[12];
    if (s1 > 0) snprintf(v1, sizeof(v1), "%dcm", s1); else snprintf(v1, sizeof(v1), "CLEAR");
    if (s2 > 0) snprintf(v2, sizeof(v2), "%dcm", s2); else snprintf(v2, sizeof(v2), "CLEAR");
    if (s3 > 0) snprintf(v3, sizeof(v3), "%dcm", s3); else snprintf(v3, sizeof(v3), "CLEAR");
    if (s4 > 0) snprintf(v4, sizeof(v4), "%dcm", s4); else snprintf(v4, sizeof(v4), "CLEAR");
    snprintf(sBuf, sizeof(sBuf), "[SONAR] S1:%s | S2:%s | S3:%s | S4:%s", v1, v2, v3, v4);
    roverLog("info", sBuf);
  }

  // Stream system monitoring to dashboard terminal & Serial Monitor (every 5s)
  if (now - lastStatusPrintTime >= 5000) {
    lastStatusPrintTime = now;
    char monBuf[128];
    snprintf(monBuf, sizeof(monBuf), "[MONITOR] Heap: %u KB | WiFi: %d dBm | Motors: %+d%% / %+d%%",
             ESP.getFreeHeap() / 1024, WiFi.RSSI(), curL, curR);
    roverLog("sys", monBuf);
  }
}
