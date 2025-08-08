# M5Stack UHF RFID Writer

Complete raw JRD-4035 protocol implementation for M5Stack Core2.

## Features

- **100% Raw Protocol**: Direct JRD-4035 communication, no M5Stack UHF library needed
- **Real RSSI Values**: Authentic signal strength from antenna frames (-5 to -100 dBm)
- **Multi-Tag Scanning**: Continuous detection with live updates
- **Variable EPC Support**: 96-496 bit EPCs with automatic PC word handling
- **Advanced Error Diagnostics**: Complete error code mapping and recovery

## Hardware

- M5Stack Core2 
- JRD-4035 UHF RFID Module
- Wiring: RX=GPIO33, TX=GPIO32 (PORT-A)

## Quick Start

1. **Install M5Unified library** (Arduino IDE > Tools > Manage Libraries)
2. **Clone repository**: `git clone https://github.com/stephanebhiri/m5stack-uhf-rfid-writer.git`  
3. **Upload firmware**: Open `.ino` file, select M5Stack-Core2 board, upload

## Controls

| Button | Action |
|--------|--------|
| A | Single scan with RSSI |
| A (long) | Continuous multi-tag mode |
| B | Write random 96-bit EPC |

## Technical Details

### Raw Functions

```cpp
rawInventoryWithRssi()  // Universal inventory parser
rawSelect()            // M5Stack compatible selection
rawWrite()             // Direct memory bank writing  
rawRead()              // TID/EPC reading
```

### RSSI Conversion

```cpp
// Real mapping from antenna frames
uint8_t rssi_raw = 0xD3;  // From frame
int8_t rssi_dbm = -18;    // Converted value

// Distance examples:
// 5cm  → 0xF0 → -12 dBm
// 50cm → 0xD3 → -18 dBm  
// 2m   → 0x80 → -55 dBm
```

### Frame Format (Reverse Engineered)

```
M5Stack Response Format:
[0]     = RSSI byte (0xD3 = 211 decimal)
[1-2]   = PC word (0x3030 = 96-bit EPC)
[3-14]  = EPC data (12 bytes)
```

## Architecture

### Core Protocol Implementation

```cpp
// JRD-4035 command structure
+------+------+------+------+------+--------+------+------+
| 0xBB | Type | CMD  | PL_H | PL_L | Data   | CS   | 0x7E |
+------+------+------+------+------+--------+------+------+
```

### Key Commands

| Command | Code | Function |
|---------|------|----------|
| INVENTORY | 0x22 | Tag scanning |
| SELECT | 0x0C | Tag selection |
| WRITE | 0x49 | Memory write |
| READ | 0x39 | Memory read |

### DisplayManager

Unified UI system for consistent display across all modes:

```cpp
DisplayManager::showContinuousMode(tagCount, powerText);
DisplayManager::showTagEntry(index, epc, rssi, tid);
DisplayManager::showWriteResult(success, message);
```

## Configuration

```cpp
// Hardware pins
static constexpr int RX_PIN = 33;
static constexpr int TX_PIN = 32;

// Protocol settings
static constexpr uint32_t CMD_TIMEOUT = 500;
static constexpr size_t RX_BUFFER_SIZE = 1024;

// RF power
static constexpr uint16_t TX_PWR_DBM10 = 2600; // 26.0 dBm
```

## Comparison: M5Stack Library vs Raw Implementation

| Feature | M5Stack Lib | This Implementation |
|---------|-------------|-------------------|
| EPC Support | 96-bit only | 96-496 bits |
| RSSI | Always -50 dBm | Real values |
| Multi-Tag | Single only | Continuous scan |
| Protocol Access | Limited | Full control |
| Dependencies | +UHF Library | Self-contained |

## Troubleshooting

### Debug Output

Enable Serial Monitor (115200 baud):

```
📤 Sending inventory cmd: BB 00 22 00 00 22 7E
📥 Raw response (24 bytes): BB 02 22 00 11 D3 30 30 AC 71...
RSSI raw=0xD3 (211 decimal) -> -18 dBm
✅ Parsed: EPC=AC713762957EBF1C72299475, RSSI=-18 dBm
```

### Common Issues

**No tags detected:**
- Check antenna connection
- Verify GPIO pins (33/32)
- Test with known working tag

**Wrong RSSI values:**  
- Calibrate `_rssibyte_to_dbm()` in `universal_inventory.h`
- Test at different distances
- Adjust mapping coefficients

**Select failures:**
- Ensure 12-byte EPC format
- Check tag lock status
- Try different access passwords

## RSSI Calibration

Edit `universal_inventory.h` to adjust RSSI mapping:

```cpp
static inline int8_t _rssibyte_to_dbm(uint8_t v) {
  if (v > 200) {
    dbm = -10 - ((255 - v) * 20) / 55;  // Strong signal
  } else if (v > 100) {
    dbm = -30 - ((200 - v) * 40) / 100; // Medium signal
  } else {
    dbm = -70 - ((100 - v) * 25) / 100; // Weak signal
  }
}
```

## Error Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 0x09 | No Tag | Tag not detected or moved |
| 0x16 | Access Denied | Wrong password |
| 0x17 | Invalid Parameter | Bad command format |
| 0xA3 | Memory Overrun | Write beyond capacity |
| 0xA4 | Memory Locked | Bank is locked |

## Use Cases

- Industrial inventory with distance estimation
- Access control with proximity detection
- IoT multi-tag monitoring
- Research and development
- Protocol analysis and debugging

## Files

- `m5stack-uhf-rfid-writer.ino` - Main firmware
- `universal_inventory.h` - Raw protocol parser
- `docs/protocol.md` - Technical protocol documentation
- `docs/wiring.md` - Hardware connection guide

## License

MIT License

## Author

Stéphane Bhiri