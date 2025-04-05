# 📲 Toy Firmware Flow with LED Indicators, Hotspot & Offline Mode

---

## ✅ 1. On Device Start (Boot)
- Show **Rainbow Gradient** LED
- Connect to Wi-Fi using saved SSID & Password
  - If Wi-Fi fails OR MQTT can't connect:
    - Enable **Wi-Fi Hotspot**
    - Show **Blinking Yellow** LED
    - Launch local setup page (e.g., 192.168.4.1) for user to configure:
      - Wi-Fi SSID & Password
      - Server address / token
    - On successful configuration:
      - Save credentials
      - Reboot device and restart connection

---

## 🌐 2. Wi-Fi & MQTT Connection Status
- **Solid White**: Connected to server (Standby)
- **Solid Red**: File system or Wi-Fi init failed → Try firmware upgrade
- **Blinking Yellow**: Cannot connect to server → Check config or server logs
- **Blinking Blue**: In pairing mode → Waiting for user to configure
- **Blinking Green**: Waiting for audio command response from server
- **Rainbow Gradient**: Boot initialization or audio playback
- **Blinking Red**: Low battery

---

## ⚙️ 3. Post-Connection Configuration
- Send:
  - `login` event
  - `updatetoken`
  - `updateconfig` (get server IP, ports)
  - `data_config` (device version, ID)
- Server will respond with:
  - Audio/speech host
  - Default recording settings
  - Thresholds and keys

---

## 🔁 4. Real-Time Monitoring
- Send periodic updates (battery, temperature, RSSI)
- Show **Solid White** (Standby)
- Detect voice input or button press

---

## 🎤 5. Audio Interaction
- **Solid Green**: Recording in progress
- **Solid Pink**: Continuous voice detection in conversation mode
- If network or server fails:
  - Use **default voice messages** from internal storage
    - Example: "Sorry, I didn’t hear that." or "Try again later."
  - Play through built-in speaker
  - Show **Rainbow Gradient** while playing

---

## ▶️ 6. Playback & Acknowledgement
- After receiving `audioplay_cmd` or `updatestarvoice`:
  - Download or use internal audio
  - Play message
  - Acknowledge with `callAck`

---

## 🔘 7. Event Handling
- Button press → Send `press_small_btn` event
- Any command execution → Always reply with `callAck`

---

## 🔄 8. Recovery & Reconnection
- If Wi-Fi drops:
  - Retry connection
  - After 3 failed attempts → Enable **Hotspot** mode (Blinking Yellow)
- If server doesn’t respond:
  - Retry MQTT
  - Show **Blinking Green**
  - Fall back to default audio if needed

---

> This firmware flow ensures the toy works seamlessly online & offline, has clear light-based feedback, and is user-configurable via hotspot fallback.
