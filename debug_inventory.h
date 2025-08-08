#pragma once
#include <Arduino.h>

// ---------------------------------------------------------
// DEBUG inventory parser for EL-UHF/JRD-4035 
// - Heavy debug logging for reverse engineering
// - M5Stack format specialization {RSSI|PC|EPC_12bytes}
// - Single tag parsing for analysis
// - Verbose Serial output for protocol debugging
// ---------------------------------------------------------

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

// Convert JRD-4035 RSSI byte to dBm using real formula
// Based on typical UHF reader mapping: higher byte = stronger signal
static inline int8_t _rssibyte_to_dbm(uint8_t v) {
  Serial.printf("🔬 RSSI raw=0x%02X (%d decimal) ", v, v);
  
  // JRD-4035 specific mapping (reverse engineering needed)
  // Common formulas:
  // Option 1: Linear mapping 0-255 -> -95 to -10 dBm
  int16_t dbm = -95 + ((int16_t)v * 85) / 255;
  
  // Option 2: More realistic UHF mapping 
  // High values (>200) = close/strong signal (-10 to -30)
  // Low values (<50) = far/weak signal (-70 to -95)
  if (v > 200) {
    dbm = -10 - ((255 - v) * 20) / 55;  // -10 to -30 dBm
  } else if (v > 100) {
    dbm = -30 - ((200 - v) * 40) / 100; // -30 to -70 dBm  
  } else {
    dbm = -70 - ((100 - v) * 25) / 100; // -70 to -95 dBm
  }
  
  // Clamp to reasonable range
  if (dbm > -5) dbm = -5;
  if (dbm < -100) dbm = -100;
  
  Serial.printf("-> %d dBm\n", dbm);
  return (int8_t)dbm;
}

// Parse M5Stack format: {RSSI|PC|EPC_12bytes} - one tag per response
static uint8_t _parseInventoryPayload(const uint8_t* payload, size_t plen,
                                      RawTagData* out, uint8_t maxItems)
{
  if (maxItems == 0 || plen < 15) return 0;  // Need minimum 15 bytes: RSSI + PC + 12 EPC

  Serial.printf("🔎 M5Stack format parser: plen=%d\n", plen);
  
  // M5Stack fixed format: [0]=RSSI, [1-2]=PC, [3-14]=EPC(12bytes)
  if (plen >= 15) {
    uint8_t rssi_byte = payload[0];        // RSSI at position 0
    uint16_t pc = (payload[1] << 8) | payload[2];  // PC at positions 1-2
    const uint8_t* epc_ptr = &payload[3];  // EPC starts at position 3
    
    Serial.printf("📊 RSSI=0x%02X, PC=0x%04X\n", rssi_byte, pc);
    
    // Validate PC word (should have reasonable EPC length)
    uint8_t epc_words = (pc >> 11) & 0x1F;
    if (epc_words >= 6 && epc_words <= 15) {  // 96-240 bits reasonable range
      
      // Check EPC not all zeros
      bool all_zero = true;
      for (int i = 0; i < 12; i++) {
        if (epc_ptr[i] != 0) {
          all_zero = false;
          break;
        }
      }
      
      if (!all_zero) {
        out[0].epc = _toHex(epc_ptr, 12);  // Always 12 bytes for M5Stack
        out[0].rssi_dbm = _rssibyte_to_dbm(rssi_byte);
        out[0].antenna = 0;
        out[0].phase = 0;
        
        Serial.printf("✅ Parsed: EPC=%s, RSSI=%d dBm\n", out[0].epc.c_str(), out[0].rssi_dbm);
        return 1;
      }
    }
  }
  
  Serial.println("❌ M5Stack format validation failed");
  return 0;
}

// Send one inventory request (CMD 0x22) and parse response.
// Returns number of tags filled into `out` (<= maxItems).
static uint8_t rawInventoryWithRssi(RawTagData* out, uint8_t maxItems) {
  if (maxItems == 0) return 0;

  Serial.println("🔍 Raw Inventory START");

  // Use proven M5Stack POLLING_ONCE format (error 0x17 avec 0x27)
  // 0xBB 0x00 0x22 0x00 0x00 0x22 0x7E  
  uint8_t tx[] = {0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E};
  size_t i = sizeof(tx);

  Serial.printf("📤 Sending inventory cmd: ");
  for (size_t j = 0; j < i; j++) {
    Serial.printf("%02X ", tx[j]);
  }
  Serial.println();

  uint8_t rx[512];
  size_t  rlen = sizeof(rx);
  if (!sendCmdRaw(tx, i, rx, rlen, 200)) {
    Serial.println("❌ sendCmdRaw failed");
    return 0;
  }

  Serial.printf("📥 Raw response (%d bytes): ", rlen);
  for (size_t j = 0; j < rlen; j++) {
    Serial.printf("%02X ", rx[j]);
  }
  Serial.println();

  // Minimum frame check
  if (rlen < 7) {
    Serial.printf("❌ Response too short: %d bytes\n", rlen);
    return 0;
  }
  
  if (rx[0] != 0xBB) {
    Serial.printf("❌ Bad header: 0x%02X\n", rx[0]);
    return 0;
  }
  
  if (rx[rlen-1] != 0x7E) {
    Serial.printf("❌ Bad trailer: 0x%02X\n", rx[rlen-1]);
    return 0;
  }

  // Check it's a reply to 0x27 (some fw answer 0x22 or 0x27 — accept both)
  if (!(rx[2] == 0x27 || rx[2] == 0x22)) {
    Serial.printf("❌ Unexpected CMD response: 0x%02X (expected 0x27 or 0x22)\n", rx[2]);
    if (rx[2] == 0xFF && rlen > 5) {
      Serial.printf("❌ Error code: 0x%02X\n", rx[5]);
    }
    return 0;
  }

  uint16_t pl = (uint16_t(rx[3]) << 8) | rx[4];
  Serial.printf("✅ Payload length: %d bytes\n", pl);
  
  if (pl == 0) {
    Serial.println("⚠️ Empty payload - no tags found");
    return 0;
  }
  
  if (5 + pl + 2 != rlen) {
    Serial.printf("❌ Length mismatch: expected %d, got %d\n", 5 + pl + 2, rlen);
    return 0;
  }

  const uint8_t* payload = &rx[5];
  size_t plen = pl;

  Serial.printf("🔎 Parsing payload (%d bytes): ", plen);
  for (size_t j = 0; j < plen; j++) {
    Serial.printf("%02X ", payload[j]);
  }
  Serial.println();

  // Try to parse multiple tags out of payload
  uint8_t found = _parseInventoryPayload(payload, plen, out, maxItems);
  Serial.printf("✅ Parser found %d tags\n", found);
  
  return found;
}