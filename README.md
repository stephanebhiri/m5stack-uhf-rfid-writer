# M5Stack UHF RFID Writer

Advanced UHF RFID EPC writing solution for M5Stack Core2 with JRD-4035 UHF modules.

## Features

- 🔧 **Variable Length EPC Writing**: Support for 96-bit to 496-bit EPCs
- 🎯 **Smart PC Word Management**: Automatic PC word calculation and writing
- 🔍 **Multi-Select Methods**: EPC, TID, and Raw select protocols
- 📊 **Capacity Probing**: Automatic tag memory capacity detection
- 🛡️ **Robust Error Handling**: Comprehensive error codes and recovery
- 📡 **Optimized Communication**: Enhanced UART buffer and timeout management

## Hardware Requirements

- **M5Stack Core2** 
- **JRD-4035 UHF RFID Module** or compatible EL-UHF-RMT01
- **Wiring**: RX=GPIO33, TX=GPIO32 (PORT-A remapped)

## Quick Start

### Installation

1. Install Arduino IDE and M5Stack libraries
2. Install the `M5Unified` library
3. Download and install the `UNIT_UHF_RFID` library
4. Upload the code to your M5Stack Core2

### Basic Usage

1. **Scan Tags**: Press Button A to scan and analyze tags
2. **Write 96-bit EPC**: Press Button B for standard 96-bit writing
3. **Write Auto-length**: Press Button C for variable-length writing

## Technical Details

### EPC Memory Structure

```
+--------+--------+--------+--------+
| CRC-16 | PC     | EPC Data...     |
+--------+--------+--------+--------+
  16-bit   16-bit   96-496 bits
```

### PC Word Format

The PC (Protocol Control) word is automatically calculated:
- Bits [15:11]: EPC length in words
- Bits [10:0]: Protocol defaults (0x3000)

### Supported EPC Lengths

- **96 bits** (12 bytes) - Standard
- **128 bits** (16 bytes) - Extended
- **192 bits** (24 bytes) - Long
- **256 bits** (32 bytes) - Very Long
- **384 bits** (48 bytes) - Ultra Long
- **496 bits** (62 bytes) - Maximum

## Advanced Features

### Multi-Select Protocol

The system uses multiple tag selection methods for maximum reliability:

1. **Library Select** - Using M5Stack UHF library
2. **Raw EPC Select** - Direct EPC-based selection
3. **TID Select** - Using Tag Identifier for locked tags

### Error Handling

Complete error code mapping:
- `0x09` - No tag detected
- `0x16` - Access denied (wrong password)
- `0xA3` - Memory overrun
- `0xA4` - Memory locked
- `0xB3` - Insufficient power

### Capacity Detection

Automatic probing of tag memory capacity:
```cpp
size_t capacity = probeEpcCapacity(ACCESS_PWD);
// Returns maximum writable bytes
```

## Configuration

### Key Parameters

```cpp
static constexpr int RX_PIN = 33;           // UART RX Pin
static constexpr int TX_PIN = 32;           // UART TX Pin  
static constexpr uint16_t TX_PWR_DBM10 = 2600; // 26.00 dBm
static constexpr uint32_t ACCESS_PWD = 0x00000000; // Access password
```

### Buffer Optimization

```cpp
static constexpr size_t RX_BUFFER_SIZE = 1024;  // UART buffer
static constexpr uint32_t CMD_TIMEOUT = 500;    // Command timeout
```

## Protocol Details

### Command Structure

All commands follow EL-UHF-RMT01/JRD-4035 protocol:

```
+------+------+------+------+------+--------+------+------+
| 0xBB | Type | CMD  | PL_H | PL_L | Data   | CS   | 0x7E |
+------+------+------+------+------+--------+------+------+
```

### Write EPC Command (0x49)

```cpp
writeEpcWithPc(epc_data, epc_length, access_password);
```

## Troubleshooting

### Common Issues

**Tag Not Detected**
- Ensure proper antenna connection
- Check TX power settings
- Verify tag is within range

**Write Failures**  
- Verify access password
- Check tag memory capacity
- Ensure tag is not locked

**Select Errors**
- Try different select methods
- Check EPC/TID data integrity
- Restart multi-inventory mode

### Debug Output

Enable serial monitoring at 115200 baud for detailed debugging:
- Command/response hex dumps
- Error code analysis  
- Tag capacity detection results

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes with proper testing
4. Submit a pull request

## License

MIT License - see LICENSE file for details.

## Acknowledgments

- M5Stack community for hardware support
- EL-UHF-RMT01/JRD-4035 protocol documentation
- Arduino UHF RFID library contributors

## Author

**Stéphane Bhiri** - Advanced RFID Systems Development

---

*This project provides professional-grade UHF RFID writing capabilities for IoT and inventory management applications.*