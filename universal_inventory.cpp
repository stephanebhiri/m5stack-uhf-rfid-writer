#include "universal_inventory.h"

HardwareSerial* gUhf = nullptr;  // define the global here

// Utils
static inline uint8_t cs8_local(const uint8_t* p, size_t n) {
  uint32_t s=0; for (size_t i=0;i<n;i++) s+=p[i]; return uint8_t(s&0xFF);
}

static void clearRx(uint32_t timeout_ms=50) {
  if (!gUhf) return;
  uint32_t t0=millis();
  while (millis()-t0<timeout_ms) {
    while (gUhf->available()) { gUhf->read(); t0=millis(); }
    delay(1);
  }
}

// sendCmdRaw implémentée dans le .ino car utilise Serial2 directement

// Multi-frame frame reader (uses attached port)
static bool readOneFrame(HardwareSerial* port, uint8_t* buf, size_t& len, uint32_t tout_ms) {
  if (!port || !buf || len < 7) return false;
  size_t idx = 0; bool started = false; uint32_t t0 = millis();
  while (millis() - t0 < tout_ms && idx < len) {
    if (port->available()) {
      uint8_t b = (uint8_t)port->read();
      if (!started) {
        if (b == 0xBB) { started = true; buf[idx++] = b; }
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
  if (idx < 7 || buf[0] != 0xBB) return false;
  uint16_t pl = (uint16_t(buf[3]) << 8) | buf[4];
  size_t expected = 5 + pl + 2;
  if (idx != expected || buf[idx-1] != 0x7E) return false;
  if (cs8_local(&buf[1], expected - 3) != buf[idx-2]) return false;
  len = idx;
  return true;
}

bool sendCmdRawMultiFrame(const uint8_t* frame, size_t len,
                          uint8_t* resp, size_t& rlen,
                          uint32_t tout_ms) {
  if (!gUhf || !frame || !resp || rlen < 7) return false;

  // first frame via existing sendCmdRaw (from the sketch)
  size_t first_len = rlen;
  if (!sendCmdRaw(frame, len, resp, first_len, tout_ms)) return false;
  size_t total = first_len;

  // subsequent frames until timeout
  uint32_t t0 = millis();
  while (millis() - t0 < tout_ms && total + 7 < rlen) {
    size_t slot = rlen - total;
    if (!readOneFrame(gUhf, resp + total, slot, 40)) break;
    total += slot;
  }

  rlen = total;
  return total > 0;
}

// API
void uhfAttachSerial(HardwareSerial* port) { gUhf = port; }

void uhfStopMultiInventory() {
  static const uint8_t stop[] = {0xBB,0x00,0x28,0x00,0x00,0x28,0x7E};
  if (!gUhf) return;
  clearRx(20);
  gUhf->write(stop, sizeof(stop));
  gUhf->flush();
  delay(60);
  clearRx(30);
}

bool uhfSelectEpc(const uint8_t* epc, size_t epc_len) {
  if (!epc || epc_len==0) return false;
  size_t clip = epc_len>31 ? 31 : epc_len;   // max 31 bytes
  uint8_t f[96]; size_t i=0;
  f[i++]=0xBB; f[i++]=0x00; f[i++]=0x0C;            // SELECT
  const uint16_t pl = 11 + clip;
  f[i++]=pl>>8; f[i++]=pl&0xFF;
  f[i++]=0x01;                 // target S0
  f[i++]=0x00;                 // action
  f[i++]=0x01;                 // bank EPC
  f[i++]=0x00; f[i++]=0x00; f[i++]=0x00; f[i++]=0x20; // pointer=0x20 bits
  f[i++]=uint8_t(clip*8);      // length in bits
  f[i++]=0x00;                 // truncate=No
  memcpy(&f[i], epc, clip); i+=clip;
  f[i++]=cs8_local(&f[1], i-1); f[i++]=0x7E;
  uint8_t resp[64]; size_t r=sizeof(resp);
  bool ok = sendCmdRaw(f, i, resp, r, 300);
  return ok && r>=6 && resp[2]==0x0C;
}

bool uhfSelectTid64(const uint8_t tid[8]) {
  if (!tid) return false;
  uint8_t f[40]; size_t i=0;
  f[i++]=0xBB; f[i++]=0x00; f[i++]=0x0C;
  f[i++]=0x00; f[i++]=0x13; // PL=19
  f[i++]=0x01; f[i++]=0x00; f[i++]=0x02; // target, action, bank=TID
  f[i++]=0x00; f[i++]=0x00; f[i++]=0x00; f[i++]=0x00; // pointer=0
  f[i++]=0x40; // 64 bits
  f[i++]=0x00; // truncate
  memcpy(&f[i], tid, 8); i+=8;
  f[i++]=cs8_local(&f[1], i-1); f[i++]=0x7E;
  uint8_t resp[64]; size_t r=sizeof(resp);
  bool ok = sendCmdRaw(f, i, resp, r, 300);
  return ok && r>=6 && resp[2]==0x0C;
}

int uhfRead(uint8_t bank, uint16_t word_ptr, uint8_t* data,
            size_t max_len, uint8_t word_count, uint32_t pwd) {
  if (!data || max_len==0) return -1;
  uint8_t f[32]; size_t i=0;
  f[i++]=0xBB; f[i++]=0x00; f[i++]=0x39;
  f[i++]=0x00; f[i++]=0x09; // PL=9
  f[i++]=uint8_t(pwd>>24); f[i++]=uint8_t(pwd>>16);
  f[i++]=uint8_t(pwd>>8);  f[i++]=uint8_t(pwd);
  f[i++]=bank;
  f[i++]=uint8_t(word_ptr>>8); f[i++]=uint8_t(word_ptr);
  f[i++]=word_count;
  f[i++]=cs8_local(&f[1], i-1); f[i++]=0x7E;
  uint8_t resp[128]; size_t r=sizeof(resp);
  if (!sendCmdRaw(f, i, resp, r, 500)) return -1;
  if (resp[2]==0x39 && r>7) {
    uint16_t pl=(resp[3]<<8)|resp[4];
    size_t n = min((size_t)pl, max_len);
    n = min(n, r-7);
    memcpy(data, &resp[5], n);
    return (int)n;
  }
  if (resp[2]==0xFF && r>5) return -1;
  return -1;
}

bool uhfWrite(uint8_t bank, uint16_t word_ptr, const uint8_t* data,
              size_t data_len, uint32_t pwd) {
  if (!data || data_len==0 || data_len>62) return false;
  uint8_t f[128]; size_t i=0;
  f[i++]=0xBB; f[i++]=0x00; f[i++]=0x49;
  uint16_t pl = 4+1+2+1 + data_len;
  f[i++]=pl>>8; f[i++]=pl&0xFF;
  f[i++]=uint8_t(pwd>>24); f[i++]=uint8_t(pwd>>16);
  f[i++]=uint8_t(pwd>>8);  f[i++]=uint8_t(pwd);
  f[i++]=bank;
  f[i++]=uint8_t(word_ptr>>8); f[i++]=uint8_t(word_ptr);
  f[i++]=uint8_t(data_len/2);
  memcpy(&f[i], data, data_len); i+=data_len;
  f[i++]=cs8_local(&f[1], i-1); f[i++]=0x7E;
  uint8_t resp[64]; size_t r=sizeof(resp);
  if (!sendCmdRaw(f, i, resp, r, 1000)) return false;
  return resp[2]==0x49;
}

bool uhfWritePcWord(uint16_t pc, uint32_t pwd) {
  uint8_t f[32]; size_t i=0;
  f[i++]=0xBB; f[i++]=0x00; f[i++]=0x49;
  f[i++]=0x00; f[i++]=0x0B; // PL=11
  f[i++]=uint8_t(pwd>>24); f[i++]=uint8_t(pwd>>16);
  f[i++]=uint8_t(pwd>>8);  f[i++]=uint8_t(pwd);
  f[i++]=0x01; // EPC
  f[i++]=0x00; f[i++]=0x01; // word 1
  f[i++]=0x00; f[i++]=0x01; // 1 word
  f[i++]=uint8_t(pc>>8); f[i++]=uint8_t(pc);
  f[i++]=cs8_local(&f[1], i-1); f[i++]=0x7E;
  uint8_t resp[64]; size_t r=sizeof(resp);
  if (!sendCmdRaw(f, i, resp, r, 500)) return false;
  return resp[2]==0x49;
}

bool uhfReadEpcViaPc(uint8_t* epc_buf, size_t& epc_len, uint16_t& out_pc, uint32_t pwd) {
  epc_len=0; out_pc=0;
  uint8_t pc_bytes[2]={0};
  int n = uhfRead(0x01, 1, pc_bytes, sizeof(pc_bytes), 1, pwd);
  if (n<2) return false;
  out_pc = (uint16_t(pc_bytes[0])<<8) | pc_bytes[1];
  uint8_t words = (out_pc>>11)&0x1F;
  if (words==0 || words>31) return false;
  size_t need = size_t(words)*2; if (need>62) need=62;
  n = uhfRead(0x01, 2, epc_buf, need, words, pwd);
  if (n<=0) return false;
  epc_len = size_t(n);
  return true;
}

bool uhfWritePcAndEpc(uint16_t new_pc, const uint8_t* epc, uint8_t words, uint32_t pwd) {
  // 1) Écrire PC
  if (!uhfWritePcWord(new_pc, pwd)) return false;
  // 2) Écrire EPC (à partir de word 2)
  const size_t bytes = size_t(words)*2;
  return uhfWrite(0x01, 2, epc, bytes, pwd);
}