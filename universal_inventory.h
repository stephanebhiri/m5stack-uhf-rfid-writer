#pragma once
#include <Arduino.h>

// ---------------------------------------------------------
// Minimal "universal" inventory parser for EL-UHF/JRD-4035
// - Sends a single-shot INVENTORY (CMD 0x27)
// - Parses response by PC word -> EPC length
// - Extracts EPC (hex) and RSSI (dBm-ish)
// - Robust to extra bytes / multi-tags in one payload
// ---------------------------------------------------------

// You already have these in ton .ino :
extern bool sendCmdRaw(const uint8_t* frame, size_t len,
                       uint8_t* resp, size_t& rlen,
                       uint32_t tout_ms /* = CMD_TIMEOUT */);

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

// Try to convert module RSSI byte to ~dBm.
// Many firmwares give RSSI as a *positive* magnitude (0..255),
// larger=stronger. A common mapping is dBm = - (rssibyte).
static inline int8_t _rssibyte_to_dbm(uint8_t v) {
  // Clamp to a sane range:
  // 0  -> ~0 dBm (impossible in UHF), re-map to -25
  // 255-> very strong -> -10
  if (v == 0) return -25;
  int8_t dbm = -int8_t(v);
  if (dbm > -10) dbm = -10;
  if (dbm < -95) dbm = -95;
  return dbm;
}

// Scan payload and extract tags by locating valid PC/EPC blocks.
// Payload = resp[5 .. 5+PL-1]
static uint8_t _parseInventoryPayload(const uint8_t* payload, size_t plen,
                                      RawTagData* out, uint8_t maxItems)
{
  uint8_t count = 0;

  // Slide a window through payload to find {PC(2)|EPC(N)|RSSI(1)} blocks.
  // N = EPC bytes = 2 * EPC_words = 2 * ((PC >> 11) & 0x1F)
  for (size_t i = 0; i + 3 < plen && count < maxItems; ++i) {
    uint16_t pc = (uint16_t(payload[i]) << 8) | payload[i+1];
    uint8_t  epc_words = (pc >> 11) & 0x1F;
    if (epc_words == 0) continue;

    size_t epc_len = size_t(epc_words) * 2; // bytes
    size_t need = 2 + epc_len + 1;          // PC + EPC + RSSI

    if (i + need <= plen) {
      // Candidate block fits. Additional sanity:
      // EPC typically not all 0x00 / 0xFF
      bool all_zero=true, all_ff=true;
      for (size_t k = 0; k < epc_len; ++k) {
        uint8_t b = payload[i+2+k];
        if (b != 0x00) all_zero=false;
        if (b != 0xFF) all_ff=false;
      }
      if (all_zero || all_ff) continue;

      // Extract
      const uint8_t* epc_ptr = &payload[i+2];
      uint8_t rssi_byte = payload[i+2+epc_len];

      out[count].epc       = _toHex(epc_ptr, epc_len);
      out[count].rssi_dbm  = _rssibyte_to_dbm(rssi_byte);
      out[count].antenna   = 0;   // not present in all firmwares
      out[count].phase     = 0;   // same

      ++count;

      // Jump ahead to the end of this block to avoid re-detecting
      i += (need - 1);
    }
  }

  return count;
}

// Send one inventory request (CMD 0x27) and parse response.
// Returns number of tags filled into `out` (<= maxItems).
static uint8_t rawInventoryWithRssi(RawTagData* out, uint8_t maxItems) {
  if (maxItems == 0) return 0;

  // Single inventory command:
  // 0xBB 0x00 0x27 0x00 0x03 [0x22 0x00 0x01] CS 0x7E
  // Many firmwares accept 0x22/0x00/0x01 as "fast inventory once".
  uint8_t tx[3+2+3+2]; // header+PL + params + CS+trail
  size_t i=0;
  tx[i++] = 0xBB;
  tx[i++] = 0x00;
  tx[i++] = 0x27;      // INVENTORY
  tx[i++] = 0x00;
  tx[i++] = 0x03;      // PL=3
  tx[i++] = 0x22;
  tx[i++] = 0x00;
  tx[i++] = 0x01;
  tx[i++] = _cs8(&tx[1], 3+3); // Type..params
  tx[i++] = 0x7E;

  uint8_t rx[512];
  size_t  rlen = sizeof(rx);
  if (!sendCmdRaw(tx, i, rx, rlen, 200)) {
    return 0;
  }

  // Minimum frame check
  if (rlen < 7 || rx[0] != 0xBB || rx[rlen-1] != 0x7E) return 0;

  // Check it's a reply to 0x27 (some fw answer 0x22 or 0x27 — accept both)
  if (!(rx[2] == 0x27 || rx[2] == 0x22)) return 0;

  uint16_t pl = (uint16_t(rx[3]) << 8) | rx[4];
  if (pl == 0 || 5 + pl + 2 != rlen) return 0;

  const uint8_t* payload = &rx[5];
  size_t plen = pl;

  // Try to parse multiple tags out of payload
  return _parseInventoryPayload(payload, plen, out, maxItems);
}