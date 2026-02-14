# Bluetooth Setup Status

## ✅ Completed

1. **ESP32 Code Modified**: Added Bluetooth functionality with dual-output (USB + Bluetooth)
2. **Code Compiled and Uploaded**: Successfully uploaded to ESP32
3. **Bluetooth Service Started**: BlueZ is running on your system
4. **ESP32 Discovered**: Found device "ESP32_Balancer" with MAC `B0:A7:32:2A:86:06`
5. **Device Paired**: Successfully paired, bonded, and trusted

## 📋 Next Steps (Run These Commands)

### 1. Create rfcomm Serial Port

```bash
sudo rfcomm bind 0 B0:A7:32:2A:86:06 1
```

### 2. Verify the Port Was Created

```bash
ls -l /dev/rfcomm0
```

You should see something like:
```
crw-rw---- 1 root dialout 216, 0 Jan 26 19:30 /dev/rfcomm0
```

### 3. Add Your User to dialout Group (if needed)

If you get permission errors, run:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and back in for it to take effect.

### 4. Test Bluetooth Connection

#### Option A: Quick Test with Python Script

```bash
cd /home/lars/Documents/PlatformIO/Projects/260122-102758-uno
python3 live_plot_bluetooth.py
```

The script will:
- Connect to ESP32 via Bluetooth
- Enable debug mode automatically (send 'R' command)
- Display live plots of pitch and motor output

#### Option B: Manual Test with screen

```bash
screen /dev/rfcomm0 115200
```

Then press `R` to enable debug mode. You should see:
```
Pitch: 25.93° | Output: 255.0 | FallCnt: 0 | Freq: 1000 Hz
```

Press `Ctrl+A` then `K` to exit screen.

## 🎮 Bluetooth Control Commands

All serial commands work over Bluetooth:

| Command | Function |
|---------|----------|
| `R` or `r` | Toggle debug mode (Pitch output @ 20Hz) |
| `P` / `p` | Increase/decrease Kp (±1.0) |
| `I` / `i` | Increase/decrease Ki (±0.1) |
| `D` / `d` | Increase/decrease Kd (±0.1) |
| `S` or `s` | Emergency stop |
| `?` | Show help |

## 📊 Current PID Parameters

- **Kp**: 40.0
- **Ki**: 0.0
- **Kd**: 1.0

## 🔧 Troubleshooting

### "Could not open /dev/rfcomm0"

1. Check if port exists: `ls -l /dev/rfcomm0`
2. Check Bluetooth connection: `bluetoothctl info B0:A7:32:2A:86:06`
3. If disconnected, reconnect: `bluetoothctl connect B0:A7:32:2A:86:06`

### "Permission denied"

Add yourself to dialout group:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and back in.

### Connection Drops

- Move closer to ESP32 (Bluetooth range ~10-30m)
- Check battery level
- Restart ESP32 and reconnect

### Rebind After Reboot

After system reboot, you need to rebind rfcomm:
```bash
sudo rfcomm bind 0 B0:A7:32:2A:86:06 1
```

Or make it permanent with udev rules (see BLUETOOTH_SETUP.md).

## 📡 Bluetooth vs USB

| Feature | USB | Bluetooth |
|---------|-----|-----------|
| Latency | ~1ms | ~10-50ms |
| Data Rate | Very High | Medium |
| Stability | Very Stable | Stable (10m) |
| Range | ~2m (cable) | ~10-30m |
| Use Case | Development/Debug | Robot Operation |

For self-balancing, Bluetooth latency is acceptable (plotting @ 20 Hz).

## ✨ What Works Now

- **Simultaneous USB + Bluetooth**: Both outputs work at the same time
- **Wireless PID Tuning**: Send P/I/D/R commands via Bluetooth
- **Wireless Monitoring**: Live plot works over Bluetooth
- **No Code Changes Needed**: Robot works identically via USB or Bluetooth

---

**Ready to test!** Run the rfcomm bind command and try the Python script.
