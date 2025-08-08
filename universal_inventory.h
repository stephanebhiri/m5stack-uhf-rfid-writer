#pragma once
#include <Arduino.h>

/*
  ---------------------------------------------------------
  Universal inventory parser for EL-UHF / JRD-4035
  - Sliding-window parser (multi-tag)
  - EPC dynamic (96..496 bits) with safe bounds
  - Multi-frame handling for CMD 0x27 (no re-send spam)
  - Optional debug via DEBUG_RSSI
  - Heap-friendly (string reserve)
  ---------------------------------------------------------
  Expects an implementation of:
    bool sendCmdRaw(const uint8_t* frame, size_t len,
                    uint8_t* resp, size_t& rlen,
                    uint32_t tout_ms = 200);
*/

// #define DEBUG_RSSI  // opt-in

// Protocol constants
static constexpr uint8_t CMD_INVENTORY   = 0x22;
static constexpr uint8_t CMD_MULTI_POLL  = 0x27;
static constexpr uint8_t FRAME_HEADER    = 0xBB;
static constexpr uint8_t FRAME_TRAILER   = 0x7E;
static constexpr uint8_t CMD_ERROR       = 0xFF;

// Forward declaration - implemented in .ino
extern bool sendCmdRaw(const uint8_t* frame, size_t len,
                       uint8_t* resp, size_t& rlen,
                       uint32_t tout_ms);

// ---------- Data model ----------
struct RawTagData {
  uint8_t epc_raw[62];    // up to 31 words * 2 bytes
  uint8_t epc_len;        // parsed bytes copied in epc_raw
  uint8_t epc_len_total;  // bytes implied by PC word (may be > epc_len)
  String  epc;            // uppercase hex (reserved to avoid fragmentation)
  int8_t  rssi_dbm;       // approx dBm
  uint8_t antenna;        // 0 if not present
  uint8_t phase;          // 0 if not present
};

// ---------- Helpers ----------
static inline uint8_t _cs8(const uint8_t* p, size_t n) {
  uint32_t s = 0; for (size_t i=0;i<n;i++) s += p[i]; return uint8_t(s & 0xFF);
}
static inline String _toHex(const uint8_t* b, size_t n) {
  String s; s.reserve(n*2);
  static const char* H="0123456789ABCDEF";
  for (size_t i=0;i<n;i++){ s += H[b[i]>>4]; s += H[b[i]&0xF]; }
  return s;
}
static constexpr inline bool _looks_like_rssi(uint8_t b) {
  return b != 0x00 && b != 0xFF; // avoid padding bytes
}

// ---------- RSSI mapping ----------
enum RssiProfile { RSSI_LINEAR, RSSI_CURVED, RSSI_CUSTOM };

static inline int8_t _rssibyte_to_dbm(uint8_t v, RssiProfile profile = RSSI_CURVED) {
#ifdef DEBUG_RSSI
  Serial.printf("🔬 RSSI raw=0x%02X (%u) ", v, v);
#endif
  int16_t dbm;
  switch (profile) {
    case RSSI_LINEAR:
      dbm = -95 + ((int16_t)v * 85) / 255; break;
    case RSSI_CURVED:
      if (v > 200)      dbm = -10 - ((255 - v) * 20) / 55;   // -10..-30
      else if (v > 100) dbm = -30 - ((200 - v) * 40) / 100;  // -30..-70
      else              dbm = -70 - ((100 - v) * 25) / 100;  // -70..-95
      break;
    case RSSI_CUSTOM:
      dbm = -50 - ((255 - v) * 45) / 255; break;
  }
  if (dbm > -5)   dbm = -5;
  if (dbm < -100) dbm = -100;
#ifdef DEBUG_RSSI
  Serial.printf("-> %d dBm\n", dbm);
#endif
  return (int8_t)dbm;
}

// ---------- Frame IO (multi-frame support) ----------
/* Read ONE complete frame currently waiting on Serial2 without sending anything */
static bool _readOneFrame(uint8_t* buf, size_t& len, uint32_t tout_ms) {
  if (!buf || len < 7) return false;
  size_t idx = 0; bool started = false; uint32_t t0 = millis();
  while (millis() - t0 < tout_ms && idx < len) {
    if (Serial2.available()) {
      uint8_t b = (uint8_t)Serial2.read();
      if (!started) {
        if (b == FRAME_HEADER) { started = true; buf[idx++] = b; }
      } else {
        buf[idx++] = b;
        if (idx >= 5) {
          uint16_t pl = (uint16_t(buf[3]) << 8) | buf[4];
          size_t expected = 5 + pl + 2;
          if (idx >= expected) break;
        }
      }
    }
  }
  if (idx < 7 || buf[0] != FRAME_HEADER) {
#ifdef DEBUG_RSSI
    if (idx > 0) Serial.printf("⚠️ Incomplete frame: got %u bytes\n", (unsigned)idx);
#endif
    return false;
  }
  uint16_t pl = (uint16_t(buf[3]) << 8) | buf[4];
  size_t expected = 5 + pl + 2;
  if (idx != expected || buf[idx-1] != FRAME_TRAILER) {
#ifdef DEBUG_RSSI
    Serial.printf("⚠️ Frame validation failed: got %u, expected %u\n", (unsigned)idx, (unsigned)expected);
#endif
    return false;
  }
  if (_cs8(&buf[1], expected - 3) != buf[idx-2]) {
#ifdef DEBUG_RSSI
    Serial.println("⚠️ Checksum validation failed");
#endif
    return false;
  }
  len = idx;
  return true;
}

/* Send ONCE, then read zero or more additional frames (CMD 0x27 use-case) */
static bool sendCmdRawMultiFrame(const uint8_t* frame, size_t len,
                                 uint8_t* resp, size_t& rlen,
                                 uint32_t tout_ms = 200) {
  if (!frame || !resp || rlen < 7) return false;

  // 1) Send ONCE and get the first frame
  size_t first_len = rlen;
  if (!sendCmdRaw(frame, len, resp, first_len, tout_ms)) return false;
  size_t total = first_len;

#ifdef DEBUG_RSSI
  Serial.printf("📥 First frame: %u bytes\n", (unsigned)first_len);
#endif

  // 2) Read subsequent frames until timeout or buffer full
  uint32_t t0 = millis();
  while (millis() - t0 < tout_ms && total + 7 < rlen) {
    size_t slot = rlen - total;
    if (!_readOneFrame(resp + total, slot, 40)) break;
    total += slot;
#ifdef DEBUG_RSSI
    Serial.printf("📥 Next frame: %u bytes (total: %u)\n", (unsigned)slot, (unsigned)total);
#endif
  }

  rlen = total;
  return total > 0;
}

// ---------- Payload parser ----------
static uint8_t _parseInventoryPayload(const uint8_t* payload, size_t plen,
                                      RawTagData* out, uint8_t maxItems)
{
  if (!payload || !out || maxItems == 0 || plen < 3) return 0;

#ifdef DEBUG_RSSI
  Serial.printf("🔎 Parse payload plen=%u, cap=%u\n", (unsigned)plen, (unsigned)maxItems);
#endif

  uint8_t found = 0;
  size_t pos = 0;

  // Fast-path: M5Stack style {RSSI | PC | 12 EPC bytes}
  if (plen >= 15) {
    uint8_t  rssi_byte = payload[0];
    uint16_t pc        = (uint16_t(payload[1]) << 8) | payload[2];
    uint8_t  epc_words = (pc >> 11) & 0x1F;

    if (pc != 0x0000 && epc_words >= 6 && epc_words <= 31) {
      const uint8_t* epc_ptr = &payload[3];
      size_t total_bytes = epc_words * 2;
      if (total_bytes > sizeof(out[0].epc_raw)) total_bytes = sizeof(out[0].epc_raw);
      size_t epc_bytes = min<size_t>(12, total_bytes);

      bool all_zero = true;
      for (size_t i=0;i<epc_bytes;i++){ if (epc_ptr[i] != 0){ all_zero=false; break; } }
      if (!all_zero) {
        memcpy(out[0].epc_raw, epc_ptr, epc_bytes);
        out[0].epc_len       = epc_bytes;
        out[0].epc_len_total = uint8_t(min<size_t>(epc_words * 2, sizeof(out[0].epc_raw)));
        out[0].epc           = _toHex(epc_ptr, epc_bytes);
        out[0].rssi_dbm      = _rssibyte_to_dbm(rssi_byte);
        out[0].antenna = 0; out[0].phase = 0;
#ifdef DEBUG_RSSI
        Serial.printf("✅ M5 format EPC=%s RSSI=%d dBm\n", out[0].epc.c_str(), out[0].rssi_dbm);
#endif
        return 1;
      }
    }
  }

  // Sliding-window fallback (raw protocol)
  while (pos + 2 <= plen - 1 && found < maxItems) {
    uint16_t pc = (uint16_t(payload[pos]) << 8) | payload[pos + 1];
    uint8_t  epc_words = (pc >> 11) & 0x1F;

    if (epc_words >= 6 && epc_words <= 31) {
      size_t epc_bytes_total = epc_words * 2;
      if (pos + 2 + epc_bytes_total <= plen) {
        const uint8_t* epc_ptr = &payload[pos + 2];

        bool all_zero = true;
        for (size_t i=0;i<epc_bytes_total;i++){ if (epc_ptr[i] != 0){ all_zero=false; break; } }

        if (!all_zero) {
          size_t to_copy = min(epc_bytes_total, sizeof(out[found].epc_raw));
          memcpy(out[found].epc_raw, epc_ptr, to_copy);
          out[found].epc_len       = (uint8_t)to_copy;
          out[found].epc_len_total = (uint8_t)min<size_t>(epc_bytes_total, sizeof(out[found].epc_raw));
          out[found].epc           = _toHex(epc_ptr, to_copy);

          // Heuristic RSSI around the EPC (prefer valid-looking byte)
          int8_t rssi_dbm = -70;
          if (pos > 0 && _looks_like_rssi(payload[pos - 1])) {
            rssi_dbm = _rssibyte_to_dbm(payload[pos - 1]);
          } else if (pos + 2 + epc_bytes_total < plen && _looks_like_rssi(payload[pos + 2 + epc_bytes_total])) {
            rssi_dbm = _rssibyte_to_dbm(payload[pos + 2 + epc_bytes_total]);
          }
          out[found].rssi_dbm = rssi_dbm;
          out[found].antenna = 0; out[found].phase = 0;

#ifdef DEBUG_RSSI
          Serial.printf("✅ RAW EPC=%s (%u bits) RSSI=%d dBm\n",
                        out[found].epc.c_str(), (unsigned)(epc_words*16), rssi_dbm);
#endif
          found++;
          pos += 2 + epc_bytes_total;
          continue;
        }
      }
    }
    pos++;
  }

#ifdef DEBUG_RSSI
  if (found == 0) Serial.println("❌ No valid tags in this payload");
#endif
  return found;
}

// ---------- Init to avoid heap fragmentation ----------
static void _initRawTagData(RawTagData* out, uint8_t maxItems) {
  if (!out) return;
  for (uint8_t i=0;i<maxItems;i++) {
    out[i].epc.reserve(124); // 62 bytes * 2 hex chars
    out[i].epc_len = 0;
    out[i].epc_len_total = 0;
    out[i].rssi_dbm = -70;
    out[i].antenna = 0;
    out[i].phase = 0;
  }
}

// ---------- Public inventory API ----------
static uint8_t rawInventoryWithRssi(RawTagData* out, uint8_t maxItems) {
  if (!out || maxItems == 0) return 0;
  _initRawTagData(out, maxItems);

#ifdef DEBUG_RSSI
  Serial.println("🔍 Inventory START");
#endif

  // Build request (payload length = 0)
  uint8_t tx[] = { FRAME_HEADER, 0x00, CMD_INVENTORY, 0x00, 0x00, 0x00, FRAME_TRAILER };
  tx[5] = _cs8(&tx[1], 4);
  const size_t tx_len = sizeof(tx);

  // Buffer to receive (can hold several frames)
  uint8_t rx[512];
  size_t rlen = sizeof(rx);

  // First try 0x22
  if (!sendCmdRaw(tx, tx_len, rx, rlen, 200)) return 0;

  // If error 0x17, fallback to 0x27 with multi-frame read
  bool used_multi = false;
  if (rlen >= 6 && rx[2] == CMD_ERROR && rx[5] == 0x17) {
#ifdef DEBUG_RSSI
    Serial.println("🔄 Fallback to 0x27 (multi-poll)");
#endif
    tx[2] = CMD_MULTI_POLL;
    tx[5] = _cs8(&tx[1], 4);
    rlen = sizeof(rx);
    if (!sendCmdRawMultiFrame(tx, tx_len, rx, rlen, 200)) return 0;
    used_multi = true;
  }

  // Parse one or multiple frames concatenated in rx
  uint8_t total_found = 0;
  size_t off = 0;

  while (off + 7 <= rlen && total_found < maxItems) {
    if (rx[off] != FRAME_HEADER) { off++; continue; }
    uint16_t pl = (uint16_t(rx[off+3]) << 8) | rx[off+4];
    size_t flen = 5 + pl + 2;
    if (off + flen > rlen) break;
    if (rx[off + flen - 1] != FRAME_TRAILER) { off++; continue; }

    uint8_t cmd = rx[off+2];
    if (cmd != CMD_ERROR && (cmd == CMD_INVENTORY || cmd == CMD_MULTI_POLL || used_multi)) {
      if (pl > 0) {
        total_found += _parseInventoryPayload(&rx[off+5], pl, out + total_found, maxItems - total_found);
      }
    }
    off += flen;
  }

#ifdef DEBUG_RSSI
  Serial.printf("✅ Inventory DONE, found=%u\n", (unsigned)total_found);
#endif
  return total_found;
}