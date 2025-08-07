# Wiring Diagram

## M5Stack Core2 to JRD-4035 UHF Module

### Connection Table

| M5Stack Core2 | JRD-4035 Module | Function |
|---------------|-----------------|----------|
| GPIO33        | RX              | Data to module |
| GPIO32        | TX              | Data from module |
| 5V            | VCC             | Power supply |
| GND           | GND             | Ground |

### PORT-A Remapped Configuration

The code uses PORT-A remapped UART configuration:
- **RX Pin**: GPIO33 (receives data FROM the UHF module)
- **TX Pin**: GPIO32 (sends data TO the UHF module)

### Power Requirements

- **Voltage**: 5V DC
- **Current**: 800mA (typical), 1.2A (peak during write operations)
- **Power**: ~6W max

### Physical Connection

```
M5Stack Core2        JRD-4035
     PORT-A          UHF Module
┌─────────────┐    ┌─────────────┐
│    GPIO33   │────│     RX      │
│    GPIO32   │────│     TX      │
│      5V     │────│    VCC      │
│     GND     │────│    GND      │
└─────────────┘    └─────────────┘
```

### Antenna Connection

Connect a 915MHz UHF antenna to the module's antenna port for optimal performance:
- **Frequency**: 865-928MHz (region dependent)
- **Impedance**: 50Ω
- **Gain**: 2-9dBi recommended

### Troubleshooting Connections

1. **No Communication**: Check TX/RX pin assignment and baud rate (115200)
2. **Power Issues**: Ensure 5V supply can handle 1.2A peak current
3. **Range Issues**: Check antenna connection and TX power settings
4. **Interference**: Keep antenna away from other RF sources

### Advanced Configuration

For custom pin assignments, modify these constants in the code:

```cpp
static constexpr int RX_PIN = 33;  // Your RX pin
static constexpr int TX_PIN = 32;  // Your TX pin
```