# UHF RFID Protocol Documentation

## EL-UHF-RMT01/JRD-4035 Protocol

### Frame Structure

All commands follow this structure:

```
+------+------+------+------+------+--------+------+------+
| 0xBB | Type | CMD  | PL_H | PL_L | Data   | CS   | 0x7E |
+------+------+------+------+------+--------+------+------+
```

- **Header**: `0xBB` (Start of frame)
- **Type**: `0x00` (Command type)
- **CMD**: Command code
- **PL_H/PL_L**: Payload length (16-bit, big-endian)
- **Data**: Command-specific data
- **CS**: Checksum (sum of Type through Data bytes, masked to 8-bit)
- **Trailer**: `0x7E` (End of frame)

### Key Commands

#### Multi-Poll/Inventory (0x27)
Scan for tags in the field:

```cpp
// Start inventory
uint8_t cmd[] = {0xBB, 0x00, 0x27, 0x00, 0x03, 0x22, 0x27, 0x10, 0x83, 0x7E};

// Stop inventory  
uint8_t stop[] = {0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E};
```

#### Select (0x0C)
Select a specific tag for operations:

```cpp
// Parameters:
// - Target: Session (0x01 = S0)
// - Action: 0x00 (standard)
// - MemBank: 0x01=EPC, 0x02=TID, 0x03=User
// - Pointer: Bit offset in memory bank
// - Length: Selection mask length in bits
// - Truncate: 0x00=No, 0x01=Yes
// - Mask: Data to match
```

#### Read (0x39)
Read tag memory:

```cpp
// Parameters:
// - AccessPwd: 32-bit access password
// - MemBank: 0x01=EPC, 0x02=TID, 0x03=User, 0x00=Reserved
// - WordPtr: Starting word address
// - WordCount: Number of words to read
```

#### Write (0x49)
Write tag memory:

```cpp
// Parameters:
// - AccessPwd: 32-bit access password  
// - MemBank: 0x01=EPC, 0x02=TID, 0x03=User, 0x00=Reserved
// - WordPtr: Starting word address
// - WordCount: Number of words to write
// - Data: Word-aligned data to write
```

### EPC Memory Layout

```
Word 0: CRC-16 (calculated by tag)
Word 1: PC (Protocol Control)
Word 2: EPC data start
...
Word N: EPC data end
```

#### PC Word Structure

```
Bit  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
   |  EPC Length   | XI| RFU |    UMI    |    NSI    | T |  RFU  |
   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
```

- **EPC Length [15:11]**: EPC length in words (not including CRC/PC)
- **XI [10]**: XPC Indicator (usually 0)
- **RFU [9:6]**: Reserved for Future Use
- **UMI [5:4]**: User Memory Indicator  
- **NSI [3:2]**: Numbering System Identifier
- **T [1]**: Toggle bit
- **RFU [0]**: Reserved

For standard applications: `PC = (epc_words << 11) | 0x3000`

### Error Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 0x09 | No Tag | No tag in field or tag moved |
| 0x16 | Access Denied | Wrong password or tag locked |
| 0xA3 | Memory Overrun | Trying to write beyond tag capacity |
| 0xA4 | Memory Locked | Memory bank is locked |
| 0xB3 | Insufficient Power | Tag not receiving enough power |

### Response Formats

#### Success Response
```
0xBB 0x00 [CMD] [PL_H] [PL_L] [STATUS] [DATA...] [CS] 0x7E
```

#### Error Response  
```
0xBB 0x00 0xFF [PL_H] [PL_L] [ERROR_CODE] [CS] 0x7E
```

### Implementation Notes

1. **Checksum Calculation**: Sum all bytes from Type to end of Data, mask to 8-bit
2. **Timeout Handling**: Use 200-500ms timeouts for reliable communication
3. **Buffer Management**: Clear RX buffer before/after commands to avoid corruption
4. **Multi-Inventory**: Always stop before starting new operations
5. **Select Reliability**: Try multiple select methods (EPC, TID, Raw) for robustness

### Advanced Features

#### Fast TID Reading
TID (Tag Identifier) provides unique tag identification:
- Bank 2, Words 0-3 (64 bits minimum)
- Contains manufacturer code and unique serial

#### Capacity Probing
Determine maximum EPC size by incrementally reading:
```cpp
// Try reading 6, 8, 12, 16, 24, 31 words
// Stop when error 0xA3 (overrun) occurs
size_t capacity = probeEpcCapacity(password);
```

#### Power Management
- Set TX power: 10-30 dBm (1000-3000 in units of 0.01 dBm)
- Monitor current during write operations
- Use delays between operations for tag recovery