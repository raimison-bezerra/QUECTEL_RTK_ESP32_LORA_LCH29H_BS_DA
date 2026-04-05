#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <math.h>

// -- Pinos -----------------------------------------------------
#define RX_PIN 48
#define TX_PIN 47

// -- BLE UUIDs -------------------------------------------------
#define BLE_SERVICE_UUID     "12345678-1234-1234-1234-123456789abc"
#define BLE_CHAR_CONFIG_UUID "12345678-1234-1234-1234-123456789001"
#define BLE_CHAR_STATUS_UUID "12345678-1234-1234-1234-123456789002"
#define BLE_CHAR_LOG_UUID    "12345678-1234-1234-1234-123456789003"

// -- LoRa ------------------------------------------------------
#define RF_FREQUENCY              915000000
#define LORA_BANDWIDTH            0
#define LORA_SPREADING_FACTOR     7
#define LORA_CODINGRATE           1
#define LORA_PREAMBLE_LENGTH      8
#define LORA_SYMBOL_TIMEOUT       0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON      false
#define TX_OUTPUT_POWER           14
#define BUFFER_SIZE               255

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
HardwareSerial rtkSerial(2);

// -- Estado do rover -------------------------------------------
struct RoverState {
  float    lat          = 0.0;
  float    lon          = 0.0;
  float    alt          = 0.0;
  int      satelites    = 0;
  float    epeH         = 0.0;
  int      fixQuality   = 0;   // 0=no fix,1=GPS,2=DGPS,4=RTK Fixed,5=RTK Float
  uint32_t rtcmRecv     = 0;   // mensagens RTCM recebidas via LoRa
  uint32_t bytesRecv    = 0;
  unsigned long ultimoRTCM = 0;
  String   firmware     = "";
};
RoverState rover;

// -- Satelites SNR ---------------------------------------------
struct Satelite { char id[6] = ""; int snr = 0; bool ativo = false; };
#define MAX_SATS 40
Satelite sats[MAX_SATS];
unsigned long ultimoGSV = 0;

// -- LoRa ------------------------------------------------------
static RadioEvents_t RadioEvents;
bool loraRecebendo = true;

// -- NMEA buffers ----------------------------------------------
String nmeaLinha = "";

// -- BLE -------------------------------------------------------
BLEServer*         bleServer  = nullptr;
BLECharacteristic* charStatus = nullptr;
BLECharacteristic* charLog    = nullptr;
bool bleConectado = false;

unsigned long ultimoDisplay = 0;
unsigned long ultimoStatus  = 0;
unsigned long ultimoSNR     = 0;

bool inicializado = false;

// -------------------------------------------------------------
void VextON(void) { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

// -- Log -------------------------------------------------------
void logMsg(String msg) {
  Serial.println(msg);
  if (bleConectado && charLog) {
    if (msg.length() > 500) msg = msg.substring(0, 500);
    charLog->setValue(msg.c_str());
    charLog->notify();
  }
}

// -- Callbacks LoRa --------------------------------------------
// Flag de controle RX - padrao oficial Heltec
bool loraIdle = true;

void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snrVal) {
  if (size > 0 && payload[0] == 0xD3) {
    // Injeta RTCM no modulo GPS imediatamente
    rtkSerial.write(payload, size);
    rover.rtcmRecv++;
    rover.bytesRecv += size;
    rover.ultimoRTCM = millis();

    // Extrai e loga tipo RTCM
    uint16_t tipo = 0;
    if (size >= 5) {
      tipo = ((uint16_t)(payload[3] & 0xFF) << 4) |
             ((uint16_t)(payload[4] & 0xF0) >> 4);
    }
    logMsg("[RTCM] " + String(tipo)
           + " " + String(size) + "B"
           + " RSSI:" + String(rssi)
           + " #" + String(rover.rtcmRecv));
  } else if (size > 0) {
    // Nao e RTCM - loga primeiro byte para diagnostico
    logMsg("[LORA] nao-RTCM 0x" + String(payload[0], HEX)
           + " " + String(size) + "B RSSI:" + String(rssi));
  }

  // Padrao oficial: Sleep aqui, loop faz Rx() novamente
  Radio.Sleep();
  loraIdle = true;
}

void OnRxTimeout(void) {
  Radio.Sleep();
  loraIdle = true;
}

void OnRxError(void) {
  logMsg("[LORA] RX erro");
  Radio.Sleep();
  loraIdle = true;
}

// -- Helpers NMEA ----------------------------------------------
String nmeaChecksum(String s) {
  uint8_t cs = 0;
  for (int i = 1; i < (int)s.length(); i++) {
    if (s[i] == '*') break;
    cs ^= (uint8_t)s[i];
  }
  char buf[4]; sprintf(buf, "%02X", cs);
  return String(buf);
}

void enviarNMEA(String cmd) {
  String full = cmd + "*" + nmeaChecksum(cmd) + "\r\n";
  rtkSerial.print(full);
  logMsg("[TX] " + cmd);
}

// -- Configurar LC29H DA (rover) -------------------------------
void consultarFirmware() {
  enviarNMEA("$PQTMVERNO");
}

void configurarRover() {
  // Habilita NMEA necessario
  enviarNMEA("$PAIR062,3,1"); delay(200); // GGA - posicao + fix quality
  enviarNMEA("$PAIR062,0,1"); delay(200); // GLL - lat/lon
  enviarNMEA("$PAIR062,1,1"); delay(200); // RMC - posicao + data
  enviarNMEA("$PAIR062,4,1"); delay(200); // GSA - satelites em uso
  enviarNMEA("$PAIR062,5,1"); delay(200); // GSV - SNR satelites

  // Habilita EPE (precisao estimada) - essencial para mostrar qualidade RTK
  enviarNMEA("$PAIR430,1");   delay(200); // habilita PQTMEPE a 1Hz
  logMsg("[ROVER] NMEA + EPE habilitados");
}

// -- Status JSON via BLE ----------------------------------------
void enviarStatusBLE() {
  if (!bleConectado || !charStatus) return;

  StaticJsonDocument<512> doc;
  doc["lat"]        = serialized(String(rover.lat, 7));
  doc["lon"]        = serialized(String(rover.lon, 7));
  doc["alt"]        = serialized(String(rover.alt, 2));
  doc["sats"]       = rover.satelites;
  doc["svinAccM"]   = serialized(String(rover.epeH, 3));
  doc["fixQuality"] = rover.fixQuality;
  doc["rtcmRecv"]   = rover.rtcmRecv;
  doc["bytesRecv"]  = rover.bytesRecv;
  doc["firmware"]   = rover.firmware;

  // Tempo desde ultimo RTCM recebido
  if (rover.ultimoRTCM > 0) {
    doc["rtcmAge"] = (millis() - rover.ultimoRTCM) / 1000;
  }

  String out; serializeJson(doc, out);
  charStatus->setValue(out.c_str());
  charStatus->notify();

  // SNR separado
  if (millis() - ultimoSNR > 5000 && millis() - ultimoGSV < 8000) {
    ultimoSNR = millis();
    delay(100);
    StaticJsonDocument<1024> snrDoc;
    JsonArray arr = snrDoc.createNestedArray("snr");
    for (int i = 0; i < MAX_SATS; i++) {
      if (sats[i].ativo && sats[i].snr > 0) {
        JsonObject o = arr.createNestedObject();
        o["id"]  = sats[i].id;
        o["snr"] = sats[i].snr;
      }
    }
    if (arr.size() > 0) {
      String snrOut; serializeJson(snrDoc, snrOut);
      charStatus->setValue(snrOut.c_str());
      charStatus->notify();
    }
  }
}

// -- BLE Callbacks ---------------------------------------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConectado = true;
    logMsg("[BLE] Conectado");
  }
  void onDisconnect(BLEServer* s) override {
    bleConectado = false;
    BLEDevice::startAdvertising();
    logMsg("[BLE] Desconectado");
  }
};

void iniciarBLE() {
  BLEDevice::init("ROVER-RTK");
  BLEDevice::setMTU(512);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  // Config write (para receber comandos futuros)
  svc->createCharacteristic(
    BLE_CHAR_CONFIG_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );

  charStatus = svc->createCharacteristic(
    BLE_CHAR_STATUS_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  charStatus->addDescriptor(new BLE2902());

  charLog = svc->createCharacteristic(
    BLE_CHAR_LOG_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  charLog->addDescriptor(new BLE2902());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  logMsg("[BLE] ROVER-RTK anunciando");
}

// -- Display ----------------------------------------------------
void atualizarDisplay() {
  display.clear();

  // Linha 0 - status RTK
  String fixStr;
  switch (rover.fixQuality) {
    case 4: fixStr = "RTK FIXED";  break;
    case 5: fixStr = "RTK FLOAT";  break;
    case 2: fixStr = "DGPS";       break;
    case 1: fixStr = "GPS";        break;
    default: fixStr = "SEM FIX";
  }
  fixStr += bleConectado ? " [B]" : "";
  display.drawString(0, 0, fixStr);
  display.drawLine(0, 11, 128, 11);

  // Linhas 1-3 - posicao
  if (rover.lat != 0.0) {
    display.drawString(0, 13, "Lat: " + String(rover.lat, 6));
    display.drawString(0, 24, "Lon: " + String(rover.lon, 6));
    display.drawString(0, 35, "Alt: " + String(rover.alt, 1) + "m");
  } else {
    display.drawString(0, 13, "Aguardando GPS...");
    display.drawString(0, 24, "Leve ao ar livre!");
  }

  // Linha 4 - sats + precisao
  String l4 = "Sats:" + String(rover.satelites);
  if (rover.epeH > 0) l4 += " EPE:" + String(rover.epeH, 2) + "m";
  display.drawString(0, 46, l4);

  // Linha 5 - RTCM recebidos
  String l5 = "RTCM:" + String(rover.rtcmRecv);
  if (rover.ultimoRTCM > 0) {
    l5 += " " + String((millis() - rover.ultimoRTCM) / 1000) + "s";
  }
  display.drawString(0, 55, l5);

  display.display();
}

// -- Parser CSV -------------------------------------------------
void splitCSV(String linha, String* c, int max) {
  int campo = 0, ini = 0;
  for (int i = 0; i <= (int)linha.length() && campo < max; i++) {
    if (i == (int)linha.length() || linha[i] == ',') {
      c[campo++] = linha.substring(ini, i);
      ini = i + 1;
    }
  }
}

// -- Parser NMEA ------------------------------------------------
void processarNMEA(String linha) {
  linha.trim();
  if (linha.length() == 0) return;
  // Log seletivo - ignora sentencas de alto volume (GSV, GLL, VTG, GSA)
  bool ehAltoVolume = (linha.indexOf("GSV") == 3 ||
                       linha.indexOf("GLL") == 3 ||
                       linha.indexOf("VTG") == 3 ||
                       linha.indexOf("GSA") == 3);
  if (!ehAltoVolume) {
    logMsg("[NMEA] " + linha);
  }

  // GGA - posicao + qualidade fix
  if (linha.length() > 6 && linha.indexOf("GGA") == 3) {
    String c[15]; splitCSV(linha, c, 15);
    if (c[2].length() > 0) {
      float rLat = c[2].toFloat(); int dLat = (int)(rLat / 100);
      rover.lat = dLat + (rLat - dLat * 100) / 60.0;
      if (c[3] == "S") rover.lat = -rover.lat;
      float rLon = c[4].toFloat(); int dLon = (int)(rLon / 100);
      rover.lon = dLon + (rLon - dLon * 100) / 60.0;
      if (c[5] == "W") rover.lon = -rover.lon;
      rover.alt       = c[9].toFloat();
      rover.satelites = c[7].toInt();
      rover.fixQuality = c[6].toInt();
      // Log mudanca de fix
      static int ultimoFix = -1;
      if (rover.fixQuality != ultimoFix) {
        String fixNome;
        switch (rover.fixQuality) {
          case 4: fixNome = "RTK FIXED!";  break;
          case 5: fixNome = "RTK FLOAT";   break;
          case 2: fixNome = "DGPS";        break;
          case 1: fixNome = "GPS";         break;
          default: fixNome = "SEM FIX";
        }
        logMsg("[FIX] Mudou para: " + fixNome);
        ultimoFix = rover.fixQuality;
      }
    }
  }

  // GLL - lat/lon (fallback)
  if (linha.length() > 6 && linha.indexOf("GLL") == 3) {
    String c[8]; splitCSV(linha, c, 8);
    String status = c[6]; status.replace("*", ""); status.trim();
    if (c[1].length() > 0 && status != "V") {
      float rLat = c[1].toFloat(); int dLat = (int)(rLat / 100);
      rover.lat = dLat + (rLat - dLat * 100) / 60.0;
      if (c[2] == "S") rover.lat = -rover.lat;
      float rLon = c[3].toFloat(); int dLon = (int)(rLon / 100);
      rover.lon = dLon + (rLon - dLon * 100) / 60.0;
      if (c[4] == "W") rover.lon = -rover.lon;
    }
  }

  // GSA - satelites em uso
  if (linha.length() > 6 && linha.indexOf("GSA") == 3) {
    String c[20]; splitCSV(linha, c, 20);
    int count = 0;
    for (int i = 3; i <= 14; i++) {
      if (c[i].length() > 0 && c[i] != "0") count++;
    }
    if (count > 0) rover.satelites = count;
  }

  // PQTMEPE - precisao estimada
  if (linha.startsWith("$PQTMEPE")) {
    String c[8]; splitCSV(linha, c, 8);
    if (c[4].length() > 0) {
      rover.epeH = c[4].toFloat();
    }
  }

  // PQTMVERNO - firmware
  if (linha.startsWith("$PQTMVERNO,") && linha.indexOf("ERROR") < 0) {
    String c[5]; splitCSV(linha, c, 5);
    if (c[1].length() > 0) {
      rover.firmware = c[1];
      logMsg("[FW] " + rover.firmware);
    }
  }

  // GSV - SNR dos satelites
  if (linha.length() > 4 && linha[0] == '$' && linha.indexOf("GSV") == 3) {
    String c[20]; splitCSV(linha, c, 20);
    char prefix[3] = {linha[1], linha[2], '\0'};
    int msgNum = c[2].toInt();
    if (msgNum == 1) {
      for (int i = 0; i < MAX_SATS; i++)
        if (strncmp(sats[i].id, prefix, 2) == 0) sats[i].ativo = false;
    }
    for (int bloco = 0; bloco < 4; bloco++) {
      int bi = 4 + bloco * 4;
      if (bi >= 20 || c[bi].length() == 0) break;
      int snr = c[bi + 3].toInt();
      String svid = String(prefix) + c[bi];
      int slot = -1;
      for (int i = 0; i < MAX_SATS; i++) {
        if (strcmp(sats[i].id, svid.c_str()) == 0) { slot = i; break; }
        if (!sats[i].ativo && slot < 0) slot = i;
      }
      if (slot >= 0) {
        svid.toCharArray(sats[slot].id, 6);
        sats[slot].snr   = snr;
        sats[slot].ativo = true;
      }
    }
    // Conta satelites visiveis
    int total = 0;
    for (int i = 0; i < MAX_SATS; i++)
      if (sats[i].ativo && sats[i].snr > 0) total++;
    if (total > 0) rover.satelites = total;
    ultimoGSV = millis();
  }

  // PAIR001 - ACK/NACK
  if (linha.startsWith("$PAIR001")) {
    String c[5]; splitCSV(linha, c, 5);
    String r = c[2];
    int sp = r.indexOf('*'); if (sp >= 0) r = r.substring(0, sp);
    r.trim();
    String status = r == "0" ? "OK" : r == "1" ? "Processando" : "ERRO r=" + r;
    logMsg("[ACK] CMD:" + c[1] + " -> " + status);
  }
}

// -- Setup ------------------------------------------------------
void setup() {
  VextON(); delay(100);
  Serial.begin(115200);
  Serial.println("=== ROVER RTK iniciando ===");
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  display.init();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0,  "ROVER RTK");
  display.drawString(0, 16, "Iniciando...");
  display.display();

  // UART LC29H DA
  rtkSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(500);

  // BLE
  iniciarBLE();

  // LoRa RX - inicializa apos BLE para evitar conflito de init
  RadioEvents.RxDone    = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError   = OnRxError;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(
    MODEM_LORA, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
    0, LORA_PREAMBLE_LENGTH,
    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
    0, true, 0, 0,
    LORA_IQ_INVERSION_ON, true
  );
  // loraIdle=true dispara Radio.Rx(0) no loop()

  display.clear();
  display.drawString(0, 0,  "ROVER RTK");
  display.drawString(0, 16, "BLE: ROVER-RTK");
  display.drawString(0, 32, "LoRa: aguardando");
  display.drawString(0, 48, "Leve ao ar livre!");
  display.display();
  delay(2000);
}

// -- Loop -------------------------------------------------------
void loop() {
  // Padrao oficial Heltec: loop controla RX
  if (loraIdle) {
    loraIdle = false;
    Radio.Rx(0);
  }
  Radio.IrqProcess();

  // UART: processa NMEA do LC29H DA
  int bytesLidos = 0;
  while (rtkSerial.available() && bytesLidos < 64) {
    uint8_t b = rtkSerial.read();
    bytesLidos++;
    if (b == '\n') {
      processarNMEA(nmeaLinha);
      nmeaLinha = "";
    } else if (b != '\r') {
      nmeaLinha += (char)b;
      if (nmeaLinha.length() > 120) nmeaLinha = "";
    }
  }

  // Inicializacao lazy (3s apos boot)
  if (!inicializado && millis() > 3000) {
    inicializado = true;
    logMsg("[INIT] Configurando modulo LC29H DA...");
    consultarFirmware(); delay(500);
    configurarRover();
  }

  // Display: 1s
  if (millis() - ultimoDisplay > 1000) {
    atualizarDisplay();
    ultimoDisplay = millis();
  }

  // Status BLE: 3s
  if (millis() - ultimoStatus > 3000) {
    enviarStatusBLE();
    ultimoStatus = millis();
  }

  // Diagnostico LoRa: loga estado a cada 10s
  static unsigned long ultimoDiag = 0;
  if (millis() - ultimoDiag > 10000) {
    ultimoDiag = millis();
    logMsg("[DIAG] RTCM recv:" + String(rover.rtcmRecv)
           + " loraIdle:" + String(loraIdle)
           + " fix:" + String(rover.fixQuality)
           + " sats:" + String(rover.satelites));
  }
}
