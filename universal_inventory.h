#pragma once
#include <Arduino.h>

// ---------------------------------------------------------
// Universal inventory parser for EL-UHF/JRD-4035
// - Production-ready with performance optimizations
// - Sliding-window parser for multi-tag detection
// - Dynamic EPC support (96-496 bits)
// - Conditional debug output via DEBUG_RSSI
// - Robust validation without performance penalty
// ---------------------------------------------------------

// Debug control - comment out for production
#define DEBUG_RSSI

// Protocol constants
static constexpr uint8_t CMD_INVENTORY = 0x22;
static constexpr uint8_t CMD_MULTI_POLL = 0x27;
static constexpr uint8_t FRAME_HEADER = 0xBB;
static constexpr uint8_t FRAME_TRAILER = 0x7E;
static constexpr uint8_t CMD_ERROR = 0xFF;

// Forward declaration - function defined in main .ino
extern bool sendCmdRaw(const uint8_t* frame, size_t len,
                       uint8_t* resp, size_t& rlen,
                       uint32_t tout_ms = 200);

struct RawTagData {
  String  epc;        // EPC in uppercase hex
  int8_t  rssi_dbm;   // approx. dBm (negative)
  uint8_t antenna;    // if present in frame; else 0
  uint8_t phase;      // if present; else 0
};

// Helpers
static inline uint8_t _cs8(const uint8_t* p, size_t n) {
  uint32_t s=0; for (size_t i=0;i<n;i++) s+=p[i]; return uint8_t(s & 0xFF);
}

static inline String _toHex(const uint8_t* b, size_t n) {
  String s; s.reserve(n*2);
  const char* H="0123456789ABCDEF";
  for (size_t i=0;i<n;i++){ s += H[b[i]>>4]; s += H[b[i]&0xF]; }
  return s;
}

// RSSI mapping profiles
enum RssiProfile {
  RSSI_LINEAR,    // Simple linear mapping
  RSSI_CURVED,    // Progressive curve mapping
  RSSI_CUSTOM     // User-defined mapping
};

// Convert JRD-4035 RSSI byte to dBm with selectable profile
static inline int8_t _rssibyte_to_dbm(uint8_t v, RssiProfile profile = RSSI_CURVED) {
#ifdef DEBUG_RSSI
  Serial.printf("🔬 RSSI raw=0x%02X (%d decimal) ", v, v);
#endif
  
  int16_t dbm;
  
  switch (profile) {
    case RSSI_LINEAR:
      // Linear mapping 0-255 -> -95 to -10 dBm
      dbm = -95 + ((int16_t)v * 85) / 255;
      break;
      
    case RSSI_CURVED:
      // Progressive curve mapping based on UHF characteristics
      if (v > 200) {
        dbm = -10 - ((255 - v) * 20) / 55;  // -10 to -30 dBm
      } else if (v > 100) {
        dbm = -30 - ((200 - v) * 40) / 100; // -30 to -70 dBm  
      } else {
        dbm = -70 - ((100 - v) * 25) / 100; // -70 to -95 dBm
      }
      break;
      
    case RSSI_CUSTOM:
      // User can modify this section for custom mapping
      dbm = -50 - ((255 - v) * 45) / 255;
      break;
  }
  
  // Clamp to reasonable UHF range
  if (dbm > -5) dbm = -5;
  if (dbm < -100) dbm = -100;
  
#ifdef DEBUG_RSSI
  Serial.printf("-> %d dBm\n", dbm);
#endif
  return (int8_t)dbm;
}

// Universal parser with sliding window detection
static uint8_t _parseInventoryPayload(const uint8_t* payload, size_t plen,
                                      RawTagData* out, uint8_t maxItems)
{
  if (maxItems == 0 || plen < 3) return 0;  // Need minimum PC word

#ifdef DEBUG_RSSI
  Serial.printf("🔎 Universal parser: plen=%d, maxItems=%d\n", plen, maxItems);
#endif
  
  uint8_t found = 0;
  size_t pos = 0;
  
  // Check for M5Stack format first: {RSSI|PC|EPC_12bytes}
  if (plen >= 15) {
    uint8_t rssi_byte = payload[0];
    uint16_t pc = (payload[1] << 8) | payload[2];
    uint8_t epc_words = (pc >> 11) & 0x1F;
    
    // Valid PC word and reasonable EPC length?
    if (epc_words >= 6 && epc_words <= 31) {
      const uint8_t* epc_ptr = &payload[3];
      
      // Check EPC not all zeros
      bool all_zero = true;
      for (int i = 0; i < 12 && i < (epc_words * 2); i++) {
        if (epc_ptr[i] != 0) {
          all_zero = false;
          break;
        }
      }
      
      if (!all_zero) {
        size_t epc_bytes = min(12, epc_words * 2);  // M5Stack limits to 12 bytes
        out[0].epc = _toHex(epc_ptr, epc_bytes);
        out[0].rssi_dbm = _rssibyte_to_dbm(rssi_byte);
        out[0].antenna = 0;
        out[0].phase = 0;
        
#ifdef DEBUG_RSSI
        Serial.printf("✅ M5Stack format: EPC=%s, RSSI=%d dBm\n", 
                     out[0].epc.c_str(), out[0].rssi_dbm);
#endif
        return 1;
      }
    }
  }
  
  // Fallback: Universal sliding window parser for raw protocol
  while (pos + 2 < plen && found < maxItems) {
    // Look for valid PC word
    uint16_t pc = (payload[pos] << 8) | payload[pos + 1];
    uint8_t epc_words = (pc >> 11) & 0x1F;
    
    // Valid EPC length (6-31 words = 96-496 bits)?
    if (epc_words >= 6 && epc_words <= 31) {
      size_t epc_bytes = epc_words * 2;
      
      // Check if we have enough data
      if (pos + 2 + epc_bytes <= plen) {
        const uint8_t* epc_ptr = &payload[pos + 2];
        
        // Validate EPC not all zeros
        bool all_zero = true;
        for (size_t i = 0; i < epc_bytes; i++) {
          if (epc_ptr[i] != 0) {
            all_zero = false;
            break;
          }
        }
        
        if (!all_zero) {
          out[found].epc = _toHex(epc_ptr, epc_bytes);
          
          // Try to find RSSI - look backwards and forwards
          int8_t rssi_dbm = -50;  // Default if not found
          if (pos > 0) {
            // RSSI might be before PC word
            rssi_dbm = _rssibyte_to_dbm(payload[pos - 1]);
          } else if (pos + 2 + epc_bytes < plen) {
            // RSSI might be after EPC
            rssi_dbm = _rssibyte_to_dbm(payload[pos + 2 + epc_bytes]);
          }
          
          out[found].rssi_dbm = rssi_dbm;
          out[found].antenna = 0;
          out[found].phase = 0;
          
#ifdef DEBUG_RSSI
          Serial.printf("✅ Universal format: EPC=%s (%d bits), RSSI=%d dBm\n", 
                       out[found].epc.c_str(), epc_words * 16, out[found].rssi_dbm);
#endif
          found++;
          pos += 2 + epc_bytes;
          continue;
        }
      }
    }
    
    // Move to next byte
    pos++;
  }
  
#ifdef DEBUG_RSSI
  if (found == 0) {
    Serial.println("❌ No valid tags found in payload");
  }
#endif
  
  return found;
}

// Send inventory request with automatic fallback
static uint8_t rawInventoryWithRssi(RawTagData* out, uint8_t maxItems) {
  if (maxItems == 0) return 0;

#ifdef DEBUG_RSSI
  Serial.println("🔍 Universal Raw Inventory START");
#endif

  // Try M5Stack format first (CMD 0x22)
  uint8_t tx[] = {FRAME_HEADER, 0x00, CMD_INVENTORY, 0x00, 0x00, CMD_INVENTORY, FRAME_TRAILER};
  size_t tx_len = sizeof(tx);

#ifdef DEBUG_RSSI
  Serial.printf("📤 Sending CMD 0x%02X: ", CMD_INVENTORY);
  for (size_t i = 0; i < tx_len; i++) {
    Serial.printf("%02X ", tx[i]);
  }
  Serial.println();
#endif

  uint8_t rx[512];
  size_t rlen = sizeof(rx);
  
  if (!sendCmdRaw(tx, tx_len, rx, rlen, 200)) {
#ifdef DEBUG_RSSI
    Serial.println("❌ sendCmdRaw failed");
#endif
    return 0;
  }

#ifdef DEBUG_RSSI
  Serial.printf("📥 Response (%d bytes): ", rlen);
  for (size_t i = 0; i < rlen; i++) {
    Serial.printf("%02X ", rx[i]);
  }
  Serial.println();
#endif

  // Validate frame structure
  if (rlen < 7 || rx[0] != FRAME_HEADER || rx[rlen-1] != FRAME_TRAILER) {
#ifdef DEBUG_RSSI
    Serial.println("❌ Invalid frame structure");
#endif
    return 0;
  }

  // Check for error response
  if (rx[2] == CMD_ERROR) {
#ifdef DEBUG_RSSI
    if (rlen > 5) {
      Serial.printf("❌ Error code: 0x%02X\n", rx[5]);
    }
#endif
    
    // Try fallback to CMD 0x27 if error 0x17 (invalid parameter)
    if (rlen > 5 && rx[5] == 0x17) {
#ifdef DEBUG_RSSI
      Serial.println("🔄 Fallback to CMD 0x27");
#endif
      tx[2] = CMD_MULTI_POLL;
      tx[5] = CMD_MULTI_POLL;  // Update checksum
      
      rlen = sizeof(rx);
      if (!sendCmdRaw(tx, tx_len, rx, rlen, 200)) {
        return 0;
      }
      
      if (rlen < 7 || rx[0] != FRAME_HEADER || rx[rlen-1] != FRAME_TRAILER || rx[2] == CMD_ERROR) {
        return 0;
      }
    } else {
      return 0;
    }
  }

  // Validate command response
  if (!(rx[2] == CMD_INVENTORY || rx[2] == CMD_MULTI_POLL)) {
#ifdef DEBUG_RSSI
    Serial.printf("❌ Unexpected CMD response: 0x%02X\n", rx[2]);
#endif
    return 0;
  }

  // Extract payload
  uint16_t pl = (uint16_t(rx[3]) << 8) | rx[4];
  
  if (pl == 0) {
#ifdef DEBUG_RSSI
    Serial.println("⚠️ Empty payload - no tags found");
#endif
    return 0;
  }
  
  // Flexible length check - allow extra padding
  if (5 + pl + 2 > rlen) {
#ifdef DEBUG_RSSI
    Serial.printf("❌ Length mismatch: expected min %d, got %d\n", 5 + pl + 2, rlen);
#endif
    return 0;
  }

  const uint8_t* payload = &rx[5];

#ifdef DEBUG_RSSI
  Serial.printf("🔎 Parsing payload (%d bytes): ", pl);
  for (size_t i = 0; i < pl; i++) {
    Serial.printf("%02X ", payload[i]);
  }
  Serial.println();
#endif

  uint8_t found = _parseInventoryPayload(payload, pl, out, maxItems);
  
#ifdef DEBUG_RSSI
  Serial.printf("✅ Found %d tags\n", found);
#endif
  
  return found;
}