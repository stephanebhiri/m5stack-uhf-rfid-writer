// Core2 + UHF (JRD-4035) — WRITE EPC avec gestion longueurs variables
// PORT-A remappé UART : Serial2.begin(..., RX=33, TX=32)
// Version robuste avec buffer optimisé et gestion PC word

#include <M5Unified.h>
#include "UNIT_UHF_RFID.h"
#include "universal_inventory.h"

// === Codes d'erreur ===
enum WriteError {
  WRITE_OK = 0,
  WRITE_NO_TAG = 0x09,
  WRITE_ACCESS_DENIED = 0x16,
  WRITE_MEMORY_OVERRUN = 0xA3,
  WRITE_MEMORY_LOCKED = 0xA4,
  WRITE_INSUFFICIENT_POWER = 0xB3,
  WRITE_UNKNOWN_ERROR = 0xFF
};

// === Configuration ===
static constexpr int RX_PIN = 33;
static constexpr int TX_PIN = 32;
static constexpr uint16_t TX_PWR_DBM10 = 2600;     // 26.00 dBm
static constexpr uint32_t ACCESS_PWD   = 0x00000000; // Adapter si besoin
static constexpr size_t RX_BUFFER_SIZE = 1024;     // Buffer UART augmenté
static constexpr uint32_t CMD_TIMEOUT  = 500;      // Timeout commandes ms
static constexpr uint32_t LONG_PRESS_DURATION = 1000;  // 1 seconde pour appui long
static constexpr uint16_t BEEP_FREQ = 1000;     // Fréquence bip 1kHz (plus discret)
static constexpr uint8_t BEEP_DURATION = 25;    // Durée bip 25ms (4x plus court)

// === Debug toggle ===
#define DEBUG_UHF_FRAMES 0  // Mettre à 1 pour activer le debug hex

Unit_UHF_RFID uhf;

// === Variables globales pour le mode continu ===
bool continuous_scan_active = false;
uint32_t button_press_start = 0;
bool button_was_long_pressed = false;
String last_detected_epc = "";
uint32_t last_beep_time = 0;

// === Structure pour multi-tags en continu ===
struct ContinuousTag {
  String epc;
  String tid;
  int rssi;
  uint32_t last_seen;
  bool is_new;
};

// === Structure pour tag avec RSSI brut ===
// RawTagData maintenant définie dans universal_inventory.h

static constexpr size_t MAX_CONTINUOUS_TAGS = 16;  // Max tags à afficher
ContinuousTag continuous_tags[MAX_CONTINUOUS_TAGS];
uint8_t continuous_tags_count = 0;
uint32_t last_display_update = 0;

// === DisplayManager - Interface utilisateur unifiée ===
class DisplayManager {
private:
  static constexpr int HEADER_SIZE = 2;
  static constexpr int BODY_SIZE = 1;
  static constexpr int SEPARATOR_Y = 45;

public:
  // Affichage de statut générique
  static void showStatus(const String& line1, const String& line2 = "", const String& line3 = "", const String& line4 = "", const String& line5 = "") {
    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextSize(HEADER_SIZE);
    M5.Display.setTextColor(WHITE);
    
    M5.Display.println(line1);
    if (line2.length() > 0) M5.Display.println(line2);
    if (line3.length() > 0) M5.Display.println(line3);
    if (line4.length() > 0) M5.Display.println(line4);
    if (line5.length() > 0) M5.Display.println(line5);
  }

  // Affichage du mode continu avec en-tête et liste de tags
  static void showContinuousMode(int tagCount, const String& powerText) {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(WHITE);
    
    // En-tête avec statistiques
    M5.Display.setCursor(0, 0);
    M5.Display.setTextSize(HEADER_SIZE);
    M5.Display.printf("CONTINUOUS [%d] %s\n", tagCount, powerText.c_str());
    
    // Contrôles disponibles
    M5.Display.setTextSize(1);
    M5.Display.println("A:stop B:power");
    
    // Ligne de séparation
    M5.Display.drawLine(0, SEPARATOR_Y, 320, SEPARATOR_Y, WHITE);
    
    // Zone de contenu tags
    M5.Display.setCursor(0, 50);
    M5.Display.setTextSize(BODY_SIZE);
    
    if (tagCount == 0) {
      M5.Display.println("No tags detected...");
    }
  }

  // Affichage d'un tag dans la liste continue
  static void showTagEntry(int index, const String& epc, int rssi, const String& tid, bool isRecent) {
    // Cas spécial pour "more..." (index négatif)
    if (index < 0) {
      M5.Display.setTextColor(YELLOW);
      M5.Display.println(epc); // epc contient le message "more..."
      return;
    }
    
    // Couleur selon l'âge du tag
    uint16_t color = isRecent ? GREEN : YELLOW;
    M5.Display.setTextColor(color);
    
    // Format compact optimisé
    M5.Display.printf("%d. %s\n", index + 1, epc.c_str());
    M5.Display.setTextColor(WHITE);
    M5.Display.printf("   RSSI:%d TID:%s\n", rssi, tid.c_str());
    M5.Display.println(); // Ligne vide
  }

  // Affichage de scan simple avec détails
  static void showScanResult(int tagCount, const String& selectedEpc, int selectedIndex, const String& powerText, const String& tidInfo = "") {
    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextSize(HEADER_SIZE);
    M5.Display.setTextColor(WHITE);
    
    M5.Display.printf("Scan: %d tags %s\n", tagCount, powerText.c_str());
    
    if (tagCount > 0) {
      M5.Display.setTextSize(1);
      M5.Display.printf("Selected: %d/%d\n", selectedIndex + 1, tagCount);
      M5.Display.printf("EPC: %s\n", selectedEpc.c_str());
      
      if (tidInfo.length() > 0) {
        M5.Display.printf("TID: %s\n", tidInfo.c_str());
      }
      
      M5.Display.println("\nA:scan B:write");
    }
  }

  // Affichage des résultats d'écriture
  static void showWriteResult(bool success, const String& message, const String& details = "", const String& newEpc = "") {
    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextSize(HEADER_SIZE);
    
    if (success) {
      M5.Display.setTextColor(GREEN);
      M5.Display.println("WRITE SUCCESS");
    } else {
      M5.Display.setTextColor(RED);
      M5.Display.println("WRITE FAILED");
    }
    
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(1);
    M5.Display.println(message);
    
    if (details.length() > 0) {
      M5.Display.println(details);
    }
    
    if (success && newEpc.length() > 0) {
      M5.Display.println("New EPC:");
      M5.Display.setTextColor(GREEN);
      M5.Display.println(newEpc);
    }
  }
};

// Wrapper pour compatibilité avec l'ancien code
void displayStatus(const String& line1, const String& line2 = "", const String& line3 = "", const String& line4 = "", const String& line5 = "") {
  DisplayManager::showStatus(line1, line2, line3, line4, line5);
}

// === Variables pour gestion TX Power ===
uint8_t current_power_index = 1;  // Index power (0=20dBm, 1=26dBm, 2=30dBm)


// === Structure pour stocker les infos d'un tag ===
struct TagInfo {
  uint8_t tid[8];
  uint8_t epc[62];  // Max 496 bits
  size_t epc_len;
  uint16_t pc_word;
  bool has_tid;
};

TagInfo current_tag;


// === Utils protocole EL-UHF-RMT01/JRD-4035 ===
static uint8_t cs8(const uint8_t* p, size_t n) {
  uint32_t s = 0;
  for (size_t i = 0; i < n; i++) s += p[i];
  return uint8_t(s & 0xFF);
}

#if DEBUG_UHF_FRAMES
static void hexDump(const char* label, const uint8_t* data, size_t len) {
  Serial.print(label);
  Serial.print(": ");
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
    if (i > 0 && (i + 1) % 16 == 0) {
      Serial.println();
      Serial.print("    ");  // Indentation
    }
  }
  Serial.println();
}
#else
#define hexDump(label, data, len) ((void)0)
#endif

// Vidage complet du buffer RX avec timeout dynamique
static void clearRxBuffer(HardwareSerial& s, uint32_t timeout = 50) {
  uint32_t start = millis();
  while (millis() - start < timeout) {
    while (s.available()) {
      s.read();
      start = millis();  // Reset timeout à chaque octet reçu
    }
    delay(1);
  }
}

// Stop multi-inventaire amélioré
static void stopMultiInv(HardwareSerial& s) {
  const uint8_t stop[] = {0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E};
  clearRxBuffer(s, 20);
  s.write(stop, sizeof(stop));
  s.flush();
  delay(60);
  clearRxBuffer(s, 30);
}

// === Envoi/recv bruts améliorés ===
static bool sendCmdRaw(const uint8_t* frame, size_t len, uint8_t* resp, size_t& rlen, uint32_t tout_ms = CMD_TIMEOUT) {
  clearRxBuffer(Serial2, 20);
  
  hexDump("TX", frame, len);  // Debug envoi
  
  Serial2.write(frame, len);
  Serial2.flush();
  
  uint32_t t0 = millis();
  size_t idx = 0;
  bool frame_started = false;
  
  while (millis() - t0 < tout_ms && idx < rlen) {
    if (Serial2.available()) {
      uint8_t b = (uint8_t)Serial2.read();
      
      // Attendre le début de trame 0xBB
      if (!frame_started) {
        if (b == 0xBB) {
          frame_started = true;
          resp[idx++] = b;
        }
      } else {
        resp[idx++] = b;
        // Si on a reçu 0x7E, on a probablement la trame complète
        if (b == 0x7E && idx >= 7) {
          // Vérifier que c'est une trame valide
          if (idx >= 5) {
            uint16_t pl = (resp[3] << 8) | resp[4];
            size_t expected_len = 5 + pl + 2;
            if (idx >= expected_len) {
              break;  // Trame complète reçue
            }
          }
        }
      }
    }
  }
  
  rlen = idx;
  
  // === Validation stricte de la trame ===
  if (idx < 7) return false;  // Trame trop courte
  if (resp[0] != 0xBB) return false;  // Mauvais header
  if (resp[idx-1] != 0x7E) return false;  // Mauvais trailer
  
  // Vérifier longueur exacte
  uint16_t pl = (resp[3] << 8) | resp[4];
  size_t expected_len = 5 + pl + 2;  // Header(1) + Type(1) + CMD(1) + PL(2) + Data(pl) + CS(1) + Trailer(1)
  
  if (idx != expected_len) {
    Serial.printf("Frame length mismatch: got %d, expected %d\n", idx, expected_len);
    return false;
  }
  
  // Vérifier checksum
  uint8_t calculated_cs = cs8(&resp[1], expected_len - 3);  // Type + CMD + PL + Data
  uint8_t received_cs = resp[idx - 2];  // CS avant trailer
  
  if (calculated_cs != received_cs) {
    Serial.printf("Checksum mismatch: calc=0x%02X, recv=0x%02X\n", calculated_cs, received_cs);
    hexDump("BAD_RX", resp, idx);  // Debug trame corrompue
    return false;
  }
  
  hexDump("RX", resp, idx);  // Debug réception
  return true;
}

// === Inventory raw avec RSSI - Utilise le parser GPT-5 ===
static uint8_t rawInventoryWithRssi(RawTagData* tags, uint8_t max_tags) {
  return ::rawInventoryWithRssi(tags, max_tags);
}

// === SELECT raw pour sélectionner un tag spécifique ===
static bool rawSelect(const uint8_t* epc, size_t epc_len, uint32_t access_pwd = 0) {
  if (!epc || epc_len == 0 || epc_len > 31) {
    Serial.println("❌ Invalid EPC for select");
    return false;
  }
  
  // Commande SELECT 0x0C
  // Format: Target(1) + Action(1) + MemBank(1) + Pointer(4) + Length(1) + Truncate(1) + Mask(variable)
  uint8_t select_cmd[64];
  size_t idx = 0;
  
  select_cmd[idx++] = 0xBB;    // Header
  select_cmd[idx++] = 0x00;    // Type  
  select_cmd[idx++] = 0x0C;    // CMD = SELECT
  
  uint16_t pl = 4 + 1 + 1 + 4 + 1 + 1 + epc_len;  // Access(4) + Target + Action + MemBank + Pointer(4) + Length + Truncate + EPC
  select_cmd[idx++] = (pl >> 8);   // PL_H
  select_cmd[idx++] = (pl & 0xFF); // PL_L
  
  // Access Password (4 bytes)
  select_cmd[idx++] = (access_pwd >> 24);
  select_cmd[idx++] = (access_pwd >> 16); 
  select_cmd[idx++] = (access_pwd >> 8);
  select_cmd[idx++] = access_pwd;
  
  // SELECT parameters
  select_cmd[idx++] = 0x01;    // Target = Session S0
  select_cmd[idx++] = 0x00;    // Action = Assert SL or inventoried -> A
  select_cmd[idx++] = 0x01;    // MemBank = EPC (0x01)
  
  // Pointer (4 bytes) = bit offset in memory bank (32 bits for PC word = skip PC)
  select_cmd[idx++] = 0x00;    // Pointer[31:24]
  select_cmd[idx++] = 0x00;    // Pointer[23:16] 
  select_cmd[idx++] = 0x00;    // Pointer[15:8]
  select_cmd[idx++] = 0x20;    // Pointer[7:0] = 32 bits (skip PC word)
  
  select_cmd[idx++] = epc_len * 8;  // Length = EPC length in bits
  select_cmd[idx++] = 0x00;    // Truncate = No
  
  // EPC Mask
  memcpy(&select_cmd[idx], epc, epc_len);
  idx += epc_len;
  
  // Checksum
  select_cmd[idx] = cs8(&select_cmd[1], idx - 1);
  idx++;
  select_cmd[idx++] = 0x7E;    // Trailer
  
  Serial.printf("Selecting EPC: ");
  for (size_t i = 0; i < epc_len; i++) {
    Serial.printf("%02X", epc[i]);
  }
  Serial.printf(" (%d bytes)\n", epc_len);
  
  uint8_t resp[32];
  size_t rlen = sizeof(resp);
  
  if (!sendCmdRaw(select_cmd, idx, resp, rlen, 500)) {
    Serial.println("❌ SELECT command failed");
    return false;
  }
  
  // Analyser la réponse
  if (resp[0] == 0xBB && resp[2] == 0x0C) {
    Serial.println("✅ SELECT successful");
    return true;
  } else if (resp[2] == 0xFF && rlen > 5) {
    Serial.printf("❌ SELECT error: 0x%02X\n", resp[5]);
    return false;
  }
  
  Serial.println("❌ SELECT unexpected response");
  return false;
}

// === WRITE raw pour écriture directe ===
static bool rawWrite(uint8_t bank, uint16_t word_ptr, const uint8_t* data, size_t data_len, uint32_t access_pwd = 0) {
  if (!data || data_len == 0 || data_len > 62) {
    Serial.println("❌ Invalid data for write");
    return false;
  }
  
  // Commande WRITE 0x49
  uint8_t write_cmd[128];
  size_t idx = 0;
  
  write_cmd[idx++] = 0xBB;     // Header
  write_cmd[idx++] = 0x00;     // Type
  write_cmd[idx++] = 0x49;     // CMD = WRITE
  
  uint16_t pl = 4 + 1 + 2 + 1 + data_len;  // Access(4) + Bank(1) + WordPtr(2) + WordCount(1) + Data
  write_cmd[idx++] = (pl >> 8);    // PL_H
  write_cmd[idx++] = (pl & 0xFF);  // PL_L
  
  // Access Password (4 bytes)
  write_cmd[idx++] = (access_pwd >> 24);
  write_cmd[idx++] = (access_pwd >> 16);
  write_cmd[idx++] = (access_pwd >> 8); 
  write_cmd[idx++] = access_pwd;
  
  // WRITE parameters
  write_cmd[idx++] = bank;              // Bank (0x01=EPC, 0x02=TID, 0x03=User)
  write_cmd[idx++] = (word_ptr >> 8);   // WordPtr high
  write_cmd[idx++] = (word_ptr & 0xFF); // WordPtr low
  write_cmd[idx++] = data_len / 2;      // WordCount (data_len in bytes / 2)
  
  // Data to write
  memcpy(&write_cmd[idx], data, data_len);
  idx += data_len;
  
  // Checksum
  write_cmd[idx] = cs8(&write_cmd[1], idx - 1);
  idx++;
  write_cmd[idx++] = 0x7E;     // Trailer
  
  Serial.printf("Writing %d bytes to bank %d, word %d\n", data_len, bank, word_ptr);
  
  uint8_t resp[32];
  size_t rlen = sizeof(resp);
  
  if (!sendCmdRaw(write_cmd, idx, resp, rlen, 1000)) {
    Serial.println("❌ WRITE command failed");
    return false;
  }
  
  // Analyser la réponse
  if (resp[0] == 0xBB && resp[2] == 0x49) {
    Serial.println("✅ WRITE successful");
    return true;
  } else if (resp[2] == 0xFF && rlen > 5) {
    Serial.printf("❌ WRITE error: 0x%02X\n", resp[5]);
    return false;
  }
  
  Serial.println("❌ WRITE unexpected response");
  return false;
}

// === READ raw pour lecture directe ===
static int rawRead(uint8_t bank, uint16_t word_ptr, uint8_t* data, size_t max_len, uint8_t word_count, uint32_t access_pwd = 0) {
  if (!data || max_len == 0) {
    Serial.println("❌ Invalid buffer for read");
    return -1;
  }
  
  // Commande READ 0x39
  uint8_t read_cmd[32];
  size_t idx = 0;
  
  read_cmd[idx++] = 0xBB;     // Header
  read_cmd[idx++] = 0x00;     // Type
  read_cmd[idx++] = 0x39;     // CMD = READ
  
  uint16_t pl = 4 + 1 + 2 + 1;  // Access(4) + Bank(1) + WordPtr(2) + WordCount(1)
  read_cmd[idx++] = (pl >> 8);    // PL_H
  read_cmd[idx++] = (pl & 0xFF);  // PL_L
  
  // Access Password (4 bytes)
  read_cmd[idx++] = (access_pwd >> 24);
  read_cmd[idx++] = (access_pwd >> 16);
  read_cmd[idx++] = (access_pwd >> 8);
  read_cmd[idx++] = access_pwd;
  
  // READ parameters
  read_cmd[idx++] = bank;              // Bank (0x01=EPC, 0x02=TID, 0x03=User) 
  read_cmd[idx++] = (word_ptr >> 8);   // WordPtr high
  read_cmd[idx++] = (word_ptr & 0xFF); // WordPtr low
  read_cmd[idx++] = word_count;        // WordCount
  
  // Checksum
  read_cmd[idx] = cs8(&read_cmd[1], idx - 1);
  idx++;
  read_cmd[idx++] = 0x7E;     // Trailer
  
  Serial.printf("Reading %d words from bank %d, word %d\n", word_count, bank, word_ptr);
  
  uint8_t resp[128];
  size_t rlen = sizeof(resp);
  
  if (!sendCmdRaw(read_cmd, idx, resp, rlen, 500)) {
    Serial.println("❌ READ command failed");
    return -1;
  }
  
  // Analyser la réponse
  if (resp[0] == 0xBB && resp[2] == 0x39 && rlen > 7) {
    uint16_t resp_pl = (resp[3] << 8) | resp[4];
    size_t bytes_to_copy = min(resp_pl, (uint16_t)max_len);
    bytes_to_copy = min(bytes_to_copy, rlen - 7);  // 5 header + CS + trailer
    
    memcpy(data, &resp[5], bytes_to_copy);
    Serial.printf("✅ READ successful: %d bytes\n", bytes_to_copy);
    return bytes_to_copy;
  } else if (resp[2] == 0xFF && rlen > 5) {
    Serial.printf("❌ READ error: 0x%02X\n", resp[5]);
    return -1;
  }
  
  Serial.println("❌ READ unexpected response");
  return -1;
}

// === Analyse des erreurs ===
static WriteError parseWriteError(uint8_t error_code) {
  switch(error_code) {
    case 0x09: return WRITE_NO_TAG;
    case 0x10: return WRITE_UNKNOWN_ERROR;  // Invalid parameter/PC word
    case 0x16: return WRITE_ACCESS_DENIED;
    case 0xA3: return WRITE_MEMORY_OVERRUN;
    case 0xA4: return WRITE_MEMORY_LOCKED;
    case 0xB3: return WRITE_INSUFFICIENT_POWER;
    default: return WRITE_UNKNOWN_ERROR;
  }
}

static const char* errorToString(WriteError err) {
  switch(err) {
    case WRITE_OK: return "OK";
    case WRITE_NO_TAG: return "No tag";
    case WRITE_ACCESS_DENIED: return "Access denied";
    case WRITE_MEMORY_OVERRUN: return "Memory overrun";
    case WRITE_MEMORY_LOCKED: return "Memory locked";
    case WRITE_INSUFFICIENT_POWER: return "Low power";
    default: return "Unknown error";
  }
}

// === Lecture du TID ===
static bool readTid(uint8_t tid[8]) {
  // READ bank 2 (TID), word 0, 4 words
  uint8_t frame[32];
  size_t i = 0;
  
  frame[i++] = 0xBB;
  frame[i++] = 0x00;  // Type
  frame[i++] = 0x39;  // CMD = Read
  frame[i++] = 0x00;  // PL high
  frame[i++] = 0x09;  // PL low = 9
  
  // AccessPwd
  frame[i++] = 0x00;
  frame[i++] = 0x00;
  frame[i++] = 0x00;
  frame[i++] = 0x00;
  
  frame[i++] = 0x02;  // Bank = TID
  frame[i++] = 0x00;  // WordPtr high
  frame[i++] = 0x00;  // WordPtr low
  frame[i++] = 0x00;  // DL high
  frame[i++] = 0x04;  // DL low = 4 words = 8 bytes
  
  uint8_t cs = cs8(&frame[1], i - 1);
  frame[i++] = cs;
  frame[i++] = 0x7E;
  
  uint8_t resp[256];
  size_t rlen = sizeof(resp);
  
  if (!sendCmdRaw(frame, i, resp, rlen)) {
    return false;
  }
  
  // Parser la réponse
  if (resp[2] == 0x39) {
    // Format avec UL
    if (rlen >= 7) {
      uint8_t ul = resp[5];  // UL = taille PC+EPC
      size_t data_offset = 6 + ul;
      
      // Vérifier format alternatif (DATA only)
      uint16_t pl = (resp[3] << 8) | resp[4];
      if (pl == 8) {  // 4 words * 2 = 8 bytes
        // Format DATA only
        memcpy(tid, &resp[5], 8);
        return true;
      } else if (data_offset + 8 <= rlen - 2) {
        // Format avec UL
        memcpy(tid, &resp[data_offset], 8);
        return true;
      }
    }
  }
  
  return false;
}

// === Select par EPC simple (utilise la lib) ===
static bool selectByEpc(Unit_UHF_RFID& uhf_ref, const uint8_t* epc, size_t epc_len) {
  // La lib attend un non-const, faire une copie temporaire
  uint8_t epc_copy[62];
  memcpy(epc_copy, epc, min(epc_len, sizeof(epc_copy)));
  return uhf_ref.select(epc_copy);
}

// === Select par TID (format corrigé) ===
static bool selectByTid(const uint8_t tid[8]) {
  uint8_t frame[32];
  size_t i = 0;
  
  frame[i++] = 0xBB;
  frame[i++] = 0x00;
  frame[i++] = 0x0C;  // CMD = Select
  frame[i++] = 0x00;
  frame[i++] = 0x13;  // PL = 19
  
  frame[i++] = 0x01;  // Target = 0x01 pour Session S0
  frame[i++] = 0x00;  // Action = 0x00
  frame[i++] = 0x02;  // MemBank = 0x02 (TID)
  
  // Pointer en bits (0 = début du TID)
  frame[i++] = 0x00;
  frame[i++] = 0x00;
  frame[i++] = 0x00;
  frame[i++] = 0x00;
  
  frame[i++] = 0x40;  // Length = 0x40 = 64 bits
  frame[i++] = 0x00;  // Truncate = 0x00 (No)
  
  // TID data (8 bytes)
  memcpy(&frame[i], tid, 8);
  i += 8;
  
  uint8_t cs = cs8(&frame[1], i - 1);
  frame[i++] = cs;
  frame[i++] = 0x7E;
  
  uint8_t resp[128];
  size_t rlen = sizeof(resp);
  
  bool result = sendCmdRaw(frame, i, resp, rlen);
  
  // Debug
  if (result) {
    Serial.print("Select TID resp: ");
    for (size_t k = 0; k < min(rlen, (size_t)20); k++) {
      Serial.printf("%02X ", resp[k]);
    }
    Serial.println();
  }
  
  return result && resp[2] == 0x0C;
}

// === Select par EPC format raw (alternative) ===
static bool selectByEpcRaw(const uint8_t* epc, size_t epc_len) {
  uint8_t frame[64];
  size_t i = 0;
  
  frame[i++] = 0xBB;
  frame[i++] = 0x00;
  frame[i++] = 0x0C;  // CMD = Select
  
  uint16_t pl = 11 + epc_len;  // 11 bytes params + EPC data
  frame[i++] = (pl >> 8);
  frame[i++] = (pl & 0xFF);
  
  frame[i++] = 0x01;  // Target = Session S0
  frame[i++] = 0x00;  // Action = 0x00
  frame[i++] = 0x01;  // MemBank = 0x01 (EPC)
  
  // Pointer = 0x20 bits (32 bits = 4 bytes offset = début de l'EPC après CRC+PC)
  frame[i++] = 0x00;
  frame[i++] = 0x00;
  frame[i++] = 0x00;
  frame[i++] = 0x20;  // 32 bits offset
  
  frame[i++] = (epc_len * 8);  // Length en bits
  frame[i++] = 0x00;  // Truncate = No
  
  // EPC data
  memcpy(&frame[i], epc, epc_len);
  i += epc_len;
  
  uint8_t cs = cs8(&frame[1], i - 1);
  frame[i++] = cs;
  frame[i++] = 0x7E;
  
  uint8_t resp[128];
  size_t rlen = sizeof(resp);
  
  bool result = sendCmdRaw(frame, i, resp, rlen, 200);
  
  // Debug
  if (result) {
    Serial.print("Select EPC resp: ");
    for (size_t k = 0; k < min(rlen, (size_t)20); k++) {
      Serial.printf("%02X ", resp[k]);
    }
    Serial.println();
  }
  
  return result && resp[2] == 0x0C;
}


// === Écriture PC word ===
static bool writePcWord(uint16_t pc_word, uint32_t accessPwd) {
  uint8_t frame[32];
  size_t i = 0;
  
  frame[i++] = 0xBB;
  frame[i++] = 0x00;
  frame[i++] = 0x49;  // WRITE
  frame[i++] = 0x00;
  frame[i++] = 0x0B;  // PL = 11 (9 + 2 bytes data)
  
  // AccessPwd
  frame[i++] = (accessPwd >> 24);
  frame[i++] = (accessPwd >> 16);
  frame[i++] = (accessPwd >> 8);
  frame[i++] = accessPwd;
  
  frame[i++] = 0x01;  // Bank = EPC
  frame[i++] = 0x00;
  frame[i++] = 0x01;  // WordPtr = 1 (PC word)
  frame[i++] = 0x00;
  frame[i++] = 0x01;  // WordCount = 1
  
  frame[i++] = (pc_word >> 8);
  frame[i++] = (pc_word & 0xFF);
  
  uint8_t cs = cs8(&frame[1], i - 1);
  frame[i++] = cs;
  frame[i++] = 0x7E;
  
  uint8_t resp[128];
  size_t rlen = sizeof(resp);
  
  if (!sendCmdRaw(frame, i, resp, rlen)) {
    Serial.println("PC Write: No response from module");
    return false;
  }
  
  if (resp[2] != 0x49) {
    Serial.printf("PC Write failed: CMD=0x%02X, Status=0x%02X", resp[2], rlen >= 6 ? resp[5] : 0xFF);
    if (rlen >= 6) {
      uint16_t pl = (resp[3] << 8) | resp[4];
      Serial.printf(", PL=%u", pl);
      if (pl > 0 && rlen >= 6 + pl) {
        Serial.printf(", Data:");
        for (uint16_t j = 0; j < pl; j++) {
          Serial.printf(" %02X", resp[5 + j]);
        }
        // Pour les erreurs, ne pas essayer d'interpréter comme PC
        if (pl >= 2 && resp[2] == 0x49) {  // Seulement si succès
          uint16_t current_pc = (resp[5] << 8) | resp[6];
          Serial.printf(" (Current PC: 0x%04X)", current_pc);
        }
      }
    }
    Serial.println();
    return false;
  }
  
  Serial.printf("PC Write success: PC=0x%04X written\n", pc_word);
  return true;
}


// === WRITE EPC 96-bit SEULEMENT avec logs détaillés ===
static WriteError writeEpcWithPc(const uint8_t* epc, size_t epc_bytes, uint32_t accessPwd) {
  Serial.println("\n========== WRITE EPC 96-BIT DETAILED LOG ==========");
  
  // Validation des paramètres
  if (epc_bytes != 12) {
    Serial.printf("ERROR: Only 96-bit EPC supported, got %u bytes\n", epc_bytes);
    Serial.println("WRITE ABORTED: Invalid EPC length");
    return WRITE_MEMORY_OVERRUN;
  }
  
  if (!epc) {
    Serial.println("ERROR: NULL EPC pointer");
    Serial.println("WRITE ABORTED: Invalid parameters");
    return WRITE_UNKNOWN_ERROR;
  }
  
  // Log des paramètres d'entrée
  Serial.printf("Input Parameters:\n");
  Serial.printf("  EPC length: %u bytes (%u bits)\n", epc_bytes, epc_bytes * 8);
  Serial.printf("  Access password: 0x%08X\n", accessPwd);
  Serial.printf("  Target EPC: ");
  for (size_t j = 0; j < epc_bytes; j++) {
    Serial.printf("%02X", epc[j]);
    if (j < epc_bytes - 1) Serial.print(" ");
  }
  Serial.println();
  
  // === ÉTAPE 1: Lire le PC word actuel ===
  Serial.println("\n--- STEP 1: Reading current PC word ---");
  
  // Commande READ: Bank=EPC, WordPtr=1, WordCount=1 (pour lire PC)
  uint8_t read_cmd[] = {
    0xBB, 0x00, 0x39,     // Header + READ command
    0x00, 0x09,           // PL = 9 bytes
    0x00, 0x00, 0x00, 0x00,  // Access password
    0x01,                 // Bank = EPC  
    0x00, 0x01,          // WordPtr = 1 (PC word)
    0x00, 0x01,          // WordCount = 1
    0x00,                 // Checksum (à calculer)
    0x7E                  // Trailer
  };
  
  // Calculer checksum
  read_cmd[14] = cs8(&read_cmd[1], 13);
  
  uint8_t resp[32];
  size_t rlen = sizeof(resp);
  
  if (sendCmdRaw(read_cmd, sizeof(read_cmd), resp, rlen) && resp[2] == 0x39) {
    // Succès de lecture
    if (rlen >= 7) {
      uint16_t current_pc = (resp[5] << 8) | resp[6];
      uint8_t current_epc_words = (current_pc >> 11) & 0x1F;
      uint8_t current_epc_bits = current_epc_words * 16;
      
      Serial.printf("Current PC word: 0x%04X\n", current_pc);
      Serial.printf("Current EPC length: %u words (%u bits)\n", current_epc_words, current_epc_bits);
      
      // Pour 96-bit on a besoin de 6 words (PC bits [15:11] = 6)
      if (current_epc_words == 6) {
        Serial.println("✅ PC word already correct for 96-bit EPC, skipping PC write");
      } else {
        Serial.printf("PC word needs update: %u words → 6 words (96-bit)\n", current_epc_words);
        
        uint16_t new_pc = (current_pc & 0x07FF) | 0x3000;  // Garder les bits de protocole, changer la longueur
        Serial.printf("New PC word: 0x%04X\n", new_pc);
        
        if (!writePcWord(new_pc, accessPwd)) {
          Serial.println("❌ PC WRITE FAILED");
          Serial.println("WRITE ABORTED: Cannot set PC word");
          Serial.println("==================================================");
          return WRITE_UNKNOWN_ERROR;
        }
        Serial.println("✅ PC write successful");
      }
    } else {
      Serial.println("⚠️  Invalid PC read response, using default 0x3000");
      goto use_default_pc;
    }
  } else {
    Serial.println("⚠️  Cannot read current PC, using default 0x3000");
    use_default_pc:
    uint16_t default_pc = 0x3000;
    
    if (!writePcWord(default_pc, accessPwd)) {
      Serial.println("❌ PC WRITE FAILED");  
      Serial.println("WRITE ABORTED: Cannot set PC word");
      Serial.println("==================================================");
      return WRITE_UNKNOWN_ERROR;
    }
    Serial.println("✅ PC write successful");
  }
  
  delay(50);
  
  // === ÉTAPE 2: Construire la commande d'écriture EPC ===
  Serial.println("\n--- STEP 2: Building EPC write command ---");
  uint16_t pl = 9 + 12;  // 9 bytes params + 12 bytes EPC
  uint8_t buf[32];
  size_t i = 0;
  
  buf[i++] = 0xBB;           // Header
  buf[i++] = 0x00;           // Type
  buf[i++] = 0x49;           // CMD = WRITE
  buf[i++] = (pl >> 8);      // PL high
  buf[i++] = (pl & 0xFF);    // PL low
  
  // Access Password (4 bytes)
  buf[i++] = (accessPwd >> 24);
  buf[i++] = (accessPwd >> 16);
  buf[i++] = (accessPwd >> 8);
  buf[i++] = accessPwd;
  
  buf[i++] = 0x01;           // Bank = EPC
  buf[i++] = 0x00;           // WordPtr high
  buf[i++] = 0x02;           // WordPtr low = 2 (début EPC après CRC+PC)
  buf[i++] = 0x00;           // WordCount high  
  buf[i++] = 0x06;           // WordCount low = 6 (96-bit)
  
  // Copier l'EPC
  memcpy(&buf[i], epc, 12);
  i += 12;
  
  // Checksum et fin
  uint8_t cs = cs8(&buf[1], i - 1);
  buf[i++] = cs;
  buf[i++] = 0x7E;           // Trailer
  
  Serial.printf("EPC write frame built: %u bytes total\n", i);
  Serial.printf("  PL=%u (9 params + 12 EPC bytes)\n", pl);
  Serial.printf("  Bank=EPC (0x01), WordPtr=2, WordCount=6\n");
  Serial.printf("  Checksum: 0x%02X\n", cs);
  Serial.printf("  Frame: ");
  for (size_t j = 0; j < i; j++) {
    Serial.printf("%02X ", buf[j]);
  }
  Serial.println();
  
  // === ÉTAPE 3: Envoyer la commande d'écriture EPC ===
  Serial.println("\n--- STEP 3: Sending EPC write command ---");
  Serial.println("Sending EPC write command to module...");
  
  uint8_t epc_resp[64];
  size_t epc_rlen = sizeof(epc_resp);
  
  if (!sendCmdRaw(buf, i, epc_resp, epc_rlen)) {
    Serial.println("❌ EPC WRITE FAILED: No response or timeout");
    Serial.println("Possible causes:");
    Serial.println("  - Module not responding");
    Serial.println("  - Tag lost or moved out of range");
    Serial.println("  - UART communication error");
    Serial.println("==================================================");
    return WRITE_UNKNOWN_ERROR;
  }
  
  Serial.printf("Received response: %u bytes\n", epc_rlen);
  Serial.printf("Response: ");
  for (size_t j = 0; j < epc_rlen; j++) {
    Serial.printf("%02X ", epc_resp[j]);
  }
  Serial.println();
  
  // === ÉTAPE 4: Analyser la réponse ===
  Serial.println("\n--- STEP 4: Analyzing response ---");
  
  if (epc_resp[2] == 0x49) {
    Serial.println("✅ EPC WRITE SUCCESS");
    Serial.printf("Response CMD=0x%02X (matches request 0x49)\n", epc_resp[2]);
    
    // Analyser les détails de la réponse
    if (epc_rlen >= 5) {
      uint16_t resp_pl = (epc_resp[3] << 8) | epc_resp[4];
      Serial.printf("Response payload: %u bytes\n", resp_pl);
      
      if (resp_pl > 0 && epc_rlen >= 6 + resp_pl) {
        Serial.printf("Response data: ");
        for (size_t j = 5; j < 5 + resp_pl; j++) {
          Serial.printf("%02X ", epc_resp[j]);
        }
        Serial.println();
      }
    }
  } else if (epc_resp[2] == 0xFF) {
    Serial.println("❌ EPC WRITE ERROR");
    Serial.printf("Response CMD=0xFF (Error response)\n");
    
    if (epc_rlen > 5) {
      uint8_t error_code = epc_resp[5];
      uint16_t resp_pl = (epc_resp[3] << 8) | epc_resp[4];
      
      Serial.printf("Error code: 0x%02X\n", error_code);
      Serial.printf("Error payload: %u bytes\n", resp_pl);
      
      // Analyser le code d'erreur
      WriteError parsed_error = parseWriteError(error_code);
      Serial.printf("Parsed error: %s\n", errorToString(parsed_error));
      
      // Détails spécifiques selon l'erreur
      switch (error_code) {
        case 0x09:
          Serial.println("DIAGNOSIS: No tag in field or tag lost during write");
          Serial.println("SOLUTION: Re-scan and retry, check tag position");
          break;
        case 0x10:
          Serial.println("DIAGNOSIS: Invalid parameter (PC word issue)");
          Serial.println("SOLUTION: Check tag compatibility with 96-bit EPC");
          break;
        case 0x16:
          Serial.println("DIAGNOSIS: Access denied (wrong password or locked)");
          Serial.println("SOLUTION: Check access password or unlock tag");
          break;
        case 0xA3:
          Serial.println("DIAGNOSIS: Memory overrun (trying to write beyond capacity)");
          Serial.println("SOLUTION: This shouldn't happen with 96-bit, check tag specs");
          break;
        case 0xA4:
          Serial.println("DIAGNOSIS: Memory locked (EPC bank is write-protected)");
          Serial.println("SOLUTION: Unlock EPC bank first");
          break;
        case 0xB3:
          Serial.println("DIAGNOSIS: Insufficient power for write operation");
          Serial.printf("SOLUTION: Increase TX power (current: %s)\n", getCurrentPowerText());
          break;
        default:
          Serial.printf("DIAGNOSIS: Unknown error code 0x%02X\n", error_code);
          Serial.println("SOLUTION: Check module documentation");
          break;
      }
      
      // Afficher données d'erreur supplémentaires
      if (resp_pl > 1 && epc_rlen >= 6 + resp_pl) {
        Serial.printf("Error data: ");
        for (size_t j = 6; j < 5 + resp_pl; j++) {
          Serial.printf("%02X ", epc_resp[j]);
        }
        Serial.println();
      }
      
      Serial.println("==================================================");
      return parsed_error;
    } else {
      Serial.println("❌ Malformed error response (missing error code)");
      Serial.println("==================================================");
      return WRITE_UNKNOWN_ERROR;
    }
  } else {
    Serial.printf("❌ UNEXPECTED RESPONSE: CMD=0x%02X (expected 0x49)\n", epc_resp[2]);
    Serial.println("DIAGNOSIS: Module returned unexpected command code");
    Serial.println("SOLUTION: Check module firmware, retry operation");
    Serial.println("==================================================");
    return WRITE_UNKNOWN_ERROR;
  }
  
  // === ÉTAPE 5: Vérification post-écriture ===
  Serial.println("\n--- STEP 5: Post-write verification ---");
  delay(50);  // Délai pour stabilisation
  
  Serial.println("Performing verification scan...");
  stopMultiInv(Serial2);
  delay(100);
  
  RawTagData verify_tags[8];
  uint8_t n = rawInventoryWithRssi(verify_tags, 8);
  if (n > 0) {
    bool found = false;
    String target_epc = bytesToHex(epc, 12);
    target_epc.toUpperCase();
    
    Serial.printf("Found %u tags during verification:\n", n);
    for (uint8_t j = 0; j < n; j++) {
      String found_epc = verify_tags[j].epc;
      found_epc.toUpperCase();
      Serial.printf("  Tag %u: %s", j+1, found_epc.c_str());
      
      if (found_epc == target_epc) {
        Serial.println(" ✅ MATCH!");
        found = true;
      } else {
        Serial.println(" (different)");
      }
    }
    
    if (found) {
      Serial.println("✅ VERIFICATION SUCCESS: New EPC confirmed by scan");
    } else {
      Serial.println("⚠️  VERIFICATION WARNING: New EPC not found in scan");
      Serial.println("   This might be normal if multiple tags are present");
    }
  } else {
    Serial.println("⚠️  VERIFICATION WARNING: No tags found during verification scan");
    Serial.println("   Tag might have moved or be out of range");
  }
  
  Serial.println("==================================================");
  Serial.println("✅ WRITE OPERATION COMPLETED SUCCESSFULLY");
  Serial.println("==================================================\n");
  
  return WRITE_OK;
}


// === Hex utilities ===
static bool hexToBytes(const String& hex, uint8_t* out, size_t max_bytes) {
  size_t len = hex.length();
  if (len % 2 != 0 || len / 2 > max_bytes) return false;
  
  auto nyb = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    c |= 0x20;  // to lowercase
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };
  
  size_t bytes = len / 2;
  for (size_t i = 0; i < bytes; i++) {
    int a = nyb(hex[2*i]);
    int b = nyb(hex[2*i+1]);
    if (a < 0 || b < 0) return false;
    out[i] = (a << 4) | b;
  }
  
  return true;
}

static String bytesToHex(const uint8_t* data, size_t len) {
  String result = "";
  for (size_t i = 0; i < len; i++) {
    char buf[3];
    sprintf(buf, "%02X", data[i]);
    result += buf;
  }
  return result;
}

// Générer un EPC aléatoire de la longueur spécifiée
static void generateRandomEpc(uint8_t* epc, size_t epc_bytes) {
  // Utiliser un seed basé sur millis() + ADC pour plus d'entropie
  uint32_t seed = millis() + analogRead(0) + esp_random();
  randomSeed(seed);
  
  for (size_t i = 0; i < epc_bytes; i++) {
    epc[i] = random(0, 256);
  }
  
  // Préfixe reconnaissable pour identifier tes tags
  if (epc_bytes >= 4) {
    epc[0] = 0xAC;        // Préfixe custom
    epc[1] = 0x71;        // "AC71" = signature
    epc[2] = (millis() >> 8) & 0xFF;   // Timestamp pour unicité
    epc[3] = millis() & 0xFF;          // Timestamp low
    // Le reste (positions 4+) reste complètement aléatoire
  }
}

// === Set Select Mode (commande 0x12) ===
static bool setSelectMode(uint8_t mode) {
  uint8_t frame[10];
  size_t i = 0;
  
  frame[i++] = 0xBB;
  frame[i++] = 0x00;
  frame[i++] = 0x12;  // CMD = Set Select Mode
  frame[i++] = 0x00;
  frame[i++] = 0x01;  // PL = 1
  frame[i++] = mode;  // 0x00=EPC, 0x01=TID, 0x02=User
  
  uint8_t cs = cs8(&frame[1], i - 1);
  frame[i++] = cs;
  frame[i++] = 0x7E;
  
  uint8_t resp[32];
  size_t rlen = sizeof(resp);
  
  return sendCmdRaw(frame, i, resp, rlen, 200) && resp[2] == 0x12;
}

// === Fonctions de gestion TX Power ===

// Obtenir le power actuel en texte
static const char* getCurrentPowerText() {
  const char* power_names[] = {"20dBm", "26dBm", "30dBm"};
  return power_names[current_power_index];
}

// Cycle TX Power : 20dBm → 26dBm → 30dBm → 20dBm
static void cycleTxPower() {
  const uint16_t powers[] = {2000, 2600, 3000};  // 20, 26, 30 dBm
  const char* power_names[] = {"20dBm", "26dBm", "30dBm"};
  
  current_power_index = (current_power_index + 1) % 3;
  uint16_t new_power = powers[current_power_index];
  
  // TX Power sera géré par nos commandes raw si nécessaire
  // uhf.setTxPower(new_power);
  
  // Bip de confirmation + feedback
  M5.Speaker.tone(1200 + (current_power_index * 300), 80, 0, false);
  Serial.printf("TX Power cycled to: %s\n", power_names[current_power_index]);
}

// === Fonctions pour le mode scan continu ===

// Bip court et discret
static void shortBeep() {
  uint32_t now = millis();
  if (now - last_beep_time > 200) {  // Minimum 200ms entre les bips
    M5.Speaker.tone(BEEP_FREQ, BEEP_DURATION, 0, false);  // Volume faible
    last_beep_time = now;
    Serial.println("♪ TAG DETECTED");
  }
}

// === Fonctions de gestion multi-tags ===

// Réinitialiser la liste des tags continus
static void clearContinuousTags() {
  continuous_tags_count = 0;
  for (uint8_t i = 0; i < MAX_CONTINUOUS_TAGS; i++) {
    continuous_tags[i].epc = "";
    continuous_tags[i].tid = "";
    continuous_tags[i].rssi = 0;
    continuous_tags[i].last_seen = 0;
    continuous_tags[i].is_new = false;
  }
}

// Ajouter ou mettre à jour un tag dans la liste
static bool addOrUpdateContinuousTag(const String& epc, const String& tid = "N/A", int rssi = 0) {
  uint32_t now = millis();
  
  // Chercher si le tag existe déjà
  for (uint8_t i = 0; i < continuous_tags_count; i++) {
    if (continuous_tags[i].epc == epc) {
      continuous_tags[i].last_seen = now;
      continuous_tags[i].is_new = false;
      // Mettre à jour TID et RSSI si fournis
      if (tid != "N/A") continuous_tags[i].tid = tid;
      if (rssi != 0) continuous_tags[i].rssi = rssi;
      return false;  // Tag déjà connu
    }
  }
  
  // Nouveau tag - l'ajouter si place disponible
  if (continuous_tags_count < MAX_CONTINUOUS_TAGS) {
    continuous_tags[continuous_tags_count].epc = epc;
    continuous_tags[continuous_tags_count].tid = tid;
    continuous_tags[continuous_tags_count].rssi = rssi;
    continuous_tags[continuous_tags_count].last_seen = now;
    continuous_tags[continuous_tags_count].is_new = true;
    continuous_tags_count++;
    return true;  // Nouveau tag ajouté
  }
  
  return false;  // Plus de place
}

// Nettoyer les vieux tags (pas vus depuis x secondes)
static void cleanupOldTags() {
  uint32_t now = millis();
  uint8_t write_idx = 0;
  
  for (uint8_t read_idx = 0; read_idx < continuous_tags_count; read_idx++) {
    if (now - continuous_tags[read_idx].last_seen < 500) {  // 500ms
      if (read_idx != write_idx) {
        continuous_tags[write_idx] = continuous_tags[read_idx];
      }
      write_idx++;
    }
  }
  
  continuous_tags_count = write_idx;
}

// Démarrage du mode inventory continu - Version simplifiée
static bool startContinuousInventory() {
  // Arrêter toute opération en cours
  stopMultiInv(Serial2);
  delay(100);
  
  Serial.println("Starting continuous mode (using repeated polling)");
  
  // Réinitialiser la liste des tags
  clearContinuousTags();
  
  // Le mode continu sera géré par des appels répétés à pollingOnce()
  // dans processContinuousScan() - plus simple et plus fiable
  return true;
}

// Lecture des tags en mode continu - Version multi-tags
static void processContinuousScan() {
  if (!continuous_scan_active) return;
  
  // Nettoyer les vieux tags de temps en temps
 static uint32_t last_cleanup = 0;
  if (millis() - last_cleanup > 500) {  // Toutes les 250ms
    cleanupOldTags();
    last_cleanup = millis();
  }
  
  // Scanner avec inventory raw pour récupérer RSSI réel
  RawTagData raw_tags[16];  // Buffer pour tags avec RSSI
  uint8_t n = rawInventoryWithRssi(raw_tags, 16);
  
  if (n > 0) {
    bool new_tags_found = false;
    
    // Traiter TOUS les tags détectés avec leur RSSI réel !
    for (uint8_t i = 0; i < n; i++) {
      String epc_str = raw_tags[i].epc;
      int rssi_value = raw_tags[i].rssi_dbm;  // RSSI réel depuis trame brute !
      
      // Récupérer TID si possible pour chaque tag
      String tid_str = "N/A";
      uint8_t epc_bytes[31];
      if (hexToBytes(epc_str, epc_bytes, 31)) {
        if (rawSelect(epc_bytes, epc_hex.length() / 2)) {
          uint8_t tid_buf[8];
          if (readTid(tid_buf)) {
            tid_str = bytesToHex(tid_buf, 8).substring(0, 8); // Premier 8 chars du TID
          }
        }
      }
      
      // Ajouter/mettre à jour ce tag avec RSSI RÉEL
      if (addOrUpdateContinuousTag(epc_str, tid_str, rssi_value)) {
        new_tags_found = true;
        Serial.printf("NEW TAG: %s RSSI: %d dBm TID: %s\n", epc_str.c_str(), rssi_value, tid_str.c_str());
      }
    }
    
    // Bip seulement s'il y a de nouveaux tags (évite la cacophonie)
    if (new_tags_found) {
      shortBeep();
    }
    
    // Mettre à jour l'affichage si nécessaire
    if (millis() - last_display_update > 50) {  // Max ms
      updateMultiTagDisplay();
      last_display_update = millis();
    }
    
    // Mettre à jour current_tag avec le premier tag (pour compatibilité write)
    String first_epc = raw_tags[0].epc;
    current_tag.epc_len = first_epc.length() / 2;
    hexToBytes(first_epc, current_tag.epc, sizeof(current_tag.epc));
  }
  
  delay(25);  // Petit délai pour éviter la surcharge
}

// Affichage multi-tags unifié avec DisplayManager
static void updateMultiTagDisplay() {
  // Initialiser l'écran avec l'en-tête via DisplayManager
  DisplayManager::showContinuousMode(continuous_tags_count, getCurrentPowerText());
  
  // Afficher chaque tag via DisplayManager
  if (continuous_tags_count > 0) {
    for (uint8_t i = 0; i < continuous_tags_count; i++) {
      // Tronquer l'EPC si nécessaire pour l'affichage
      String display_epc = continuous_tags[i].epc;
      if (display_epc.length() > 26) {
        display_epc = display_epc.substring(0, 24) + "..";
      }
      
      // Utiliser DisplayManager pour l'affichage unifié
      String tid_short = continuous_tags[i].tid.substring(0, min(8U, continuous_tags[i].tid.length()));
      DisplayManager::showTagEntry(i, display_epc, continuous_tags[i].rssi, tid_short, continuous_tags[i].is_new);
      
      // Marquer comme plus nouveau après affichage
      continuous_tags[i].is_new = false;
      
      // Limite d'affichage - utiliser DisplayManager pour cohérence
      if (i >= 6) {  // Limite réduite pour laisser place aux détails
        DisplayManager::showTagEntry(-1, String("+ ") + String(continuous_tags_count - 7) + " more...", 0, "", false);
        break;
      }
    }
  }
}


// === Setup ===
void setup() {
  auto cfg = M5.config();
  cfg.output_power = true;
  M5.begin(cfg);
  Serial.begin(115200);
  
  displayStatus("UHF Init...", "RX=33 TX=32");
  
  // Configuration UART optimisée
  Serial2.setRxBufferSize(RX_BUFFER_SIZE);  // AVANT begin()
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.setTimeout(300);
  
  // uhf.begin(&Serial2, 115200, RX_PIN, TX_PIN, false);  // Remplacé par raw
  
  stopMultiInv(Serial2);
  
  // Vérifier la connexion
  String info = "ERROR";
  uint32_t t0 = millis();
  while (info == "ERROR" && millis() - t0 < 3000) {
    // info = uhf.getVersion();  // Remplacé par test raw
    // Test basique avec inventory
    RawTagData test_tags[1];
    if (rawInventoryWithRssi(test_tags, 1) >= 0) {
      info = "Raw UHF OK";
    } else {
      info = "ERROR";
    }
    delay(120);
  }
  
  Serial.println("Version: " + info);
  
  if (info == "ERROR") {
    displayStatus("UHF ERROR", "Not detected!");
    return;
  }
  
  // uhf.setTxPower(TX_PWR_DBM10);  // Géré par raw
  
  // Mode "need select" (optionnel - commenter si problème)
  // setSelectMode(0x01);  // 0x00=pas de select requis, 0x01=select requis
  
  displayStatus("UHF Ready", "A: Scan | A long: Continuous", "B: Write 96b");
}

// === Main loop ===
void loop() {
  M5.update();
  
  // === Gestion du mode scan continu (appui long sur A) ===
  
  // Détecter début/fin appui long
  if (M5.BtnA.isPressed()) {
    if (button_press_start == 0) {
      button_press_start = millis();
      button_was_long_pressed = false;
    } else if (!button_was_long_pressed && millis() - button_press_start > LONG_PRESS_DURATION) {
      // Appui long détecté - basculer le mode continu
      button_was_long_pressed = true;
      continuous_scan_active = !continuous_scan_active;
      
      if (continuous_scan_active) {
        displayStatus("CONTINUOUS SCAN", "Active...", "Press A short to stop");
        if (startContinuousInventory()) {
          M5.Speaker.tone(1000, 200, 0, false);  // Bip de démarrage
          Serial.println("=== CONTINUOUS SCAN STARTED ===");
        } else {
          continuous_scan_active = false;
          displayStatus("ERROR", "Failed to start", "continuous mode");
        }
      } else {
        stopMultiInv(Serial2);
        displayStatus("CONTINUOUS SCAN", "Stopped", "Back to normal mode");
        M5.Speaker.tone(800, 200, 0, false);  // Bip d'arrêt
        Serial.println("=== CONTINUOUS SCAN STOPPED ===");
      }
    }
  } else {
    button_press_start = 0;
    button_was_long_pressed = false;
  }
  
  // Traitement du mode continu
  processContinuousScan();
  
  // === Mode scan simple (appui court) ===
  if (M5.BtnA.wasPressed() && !button_was_long_pressed) {
    // Si en mode continu, A court l'arrête
    if (continuous_scan_active) {
      continuous_scan_active = false;
      stopMultiInv(Serial2);
      displayStatus("SCAN STOPPED", "Continuous mode", "disabled");
      M5.Speaker.tone(800, 100, 0, false);  // Bip d'arrêt
      return;
    }
    
    // Sinon, scan normal
    stopMultiInv(Serial2);
    
    // Utiliser notre parser universel au lieu de uhf.pollingOnce()
    RawTagData raw_tags[8];
    uint8_t n = rawInventoryWithRssi(raw_tags, 8);
    
    if (n == 0) {
      displayStatus("Scan: 0", "No tags");
      return;
    }
    
    // Lire le premier tag (avec RSSI réel !)
    String epc_str = raw_tags[0].epc;
    current_tag.epc_len = epc_str.length() / 2;
    hexToBytes(epc_str, current_tag.epc, sizeof(current_tag.epc));
    
    // Sélectionner et lire TID
    if (rawSelect(current_tag.epc, current_tag.epc_len)) {
      current_tag.has_tid = readTid(current_tag.tid);
      
      String tid_str = current_tag.has_tid ? bytesToHex(current_tag.tid, 8) : "N/A";
      
      displayStatus(
      "Tag found",
      "EPC: " + String(current_tag.epc_len * 8) + "b RSSI: " + String(raw_tags[0].rssi_dbm) + "dBm",
      "Data: " + String(epc_str.substring(0, 24)),
      "TID: " + tid_str.substring(0, 16)
      );
    } else {
      displayStatus("Scan OK", "Select failed");
    }
  }
  
  // Bouton B : Write 96 bits fixe
  if (M5.BtnB.wasPressed()) {
    // En mode continu : Cycle TX Power au lieu d'arrêter
    if (continuous_scan_active) {
      cycleTxPower();
      return;  // Ne pas continuer vers l'écriture
    }
    if (current_tag.epc_len == 0) {
      displayStatus("Error", "Scan first!");
      return;
    }
    
    Serial.println("\n=== WRITE 96 BITS ===");
    Serial.print("Current EPC: ");
    for (size_t i = 0; i < current_tag.epc_len; i++) {
      Serial.printf("%02X ", current_tag.epc[i]);
    }
    Serial.println();
    
    stopMultiInv(Serial2);
    delay(50);  // Petit délai après stop
    
    // Essayer plusieurs méthodes de select
    bool selected = false;
    
    // Méthode 1: Select EPC avec la lib (le plus fiable)
    selected = rawSelect(current_tag.epc, current_tag.epc_len);
    Serial.println("Select by lib: " + String(selected));
    
    if (!selected) {
      // Méthode 2: Select EPC raw
      selected = selectByEpcRaw(current_tag.epc, current_tag.epc_len);
      Serial.println("Select by EPC raw: " + String(selected));
    }
    
    if (!selected && current_tag.has_tid) {
      // Méthode 3: Select par TID
      selected = selectByTid(current_tag.tid);
      Serial.println("Select by TID: " + String(selected));
    }
    
    if (!selected) {
      // Dernière tentative : stop + scan universel + select immédiat
      stopMultiInv(Serial2);
      delay(100);
      RawTagData rescan_tags[1];
      uint8_t n = rawInventoryWithRssi(rescan_tags, 1);
      if (n > 0) {
        Serial.println("Re-scan found EPC: " + rescan_tags[0].epc);
        uint8_t epc_bytes[31];
        if (hexToBytes(rescan_tags[0].epc, epc_bytes, 31)) {
          selected = rawSelect(epc_bytes, epc_len);
          Serial.println("Select after poll: " + String(selected));
        }
      }
    }
    
    if (!selected) {
      displayStatus("Write fail", "Select error", "Check tag");
      Serial.println("All select methods failed!");
      return;
    }
    
    // Nouveau EPC 96 bits aléatoire
    uint8_t new_epc[12];
    generateRandomEpc(new_epc, 12);
    String newEpcHex = bytesToHex(new_epc, 12);
    
    Serial.println("Target new EPC: " + newEpcHex);
    Serial.print("New EPC bytes: ");
    for (size_t i = 0; i < 12; i++) {
      Serial.printf("%02X ", new_epc[i]);
    }
    Serial.println();
    
    WriteError err = writeEpcWithPc(new_epc, 12, ACCESS_PWD);
    Serial.println("Write result: " + String(errorToString(err)));
    
    if (err == WRITE_OK) {
      delay(100);
      
      // Vérifier avec notre parser universel
      stopMultiInv(Serial2);
      RawTagData verify_tags[8];
      uint8_t n = rawInventoryWithRssi(verify_tags, 8);
      bool found = false;
      
      Serial.println("\nVerification scan:");
      for (uint8_t i = 0; i < n; i++) {
        String s = verify_tags[i].epc;
        Serial.println("  Found: " + s);
        s.toUpperCase();
        if (s == newEpcHex) {
          found = true;
          Serial.println("  -> MATCH! Write successful!");
        }
      }
      
      if (!found) {
        Serial.println("  -> New EPC not found in verification");
      }
      
      Serial.println("====================\n");
      
      // Affichage unifié avec DisplayManager
      String verifyMsg = found ? "Verification: PASSED" : "Verification: FAILED";
      DisplayManager::showWriteResult(true, verifyMsg, "", newEpcHex);
    } else {
      Serial.println("Write failed with error: " + String(errorToString(err)));
      DisplayManager::showWriteResult(false, errorToString(err), "Check tag position and power");
    }
  }
}
