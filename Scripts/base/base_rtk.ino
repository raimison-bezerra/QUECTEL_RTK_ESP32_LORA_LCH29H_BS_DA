#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// -- Pinos ------------------------------------------------------------
#define RX_PIN 48
#define TX_PIN 47

// -- BLE UUIDs --------------------------------------------------------
#define BLE_SERVICE_UUID     "12345678-1234-1234-1234-123456789abc"
#define BLE_CHAR_CONFIG_UUID "12345678-1234-1234-1234-123456789001"
#define BLE_CHAR_STATUS_UUID "12345678-1234-1234-1234-123456789002"
#define BLE_CHAR_LOG_UUID    "12345678-1234-1234-1234-123456789003"

// -- LoRa -------------------------------------------------------------
#define LORA_BANDWIDTH             0
#define LORA_SPREADING_FACTOR      7
#define LORA_CODINGRATE            1
#define LORA_PREAMBLE_LENGTH       8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false
#define TX_OUTPUT_POWER            14
#define BUFFER_SIZE                255

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
HardwareSerial rtkSerial(2);

// -- Config editavel via BLE ------------------------------------------
// Referencia datasheet secao 2.2.1:
// PQTMCFGSVIN,W,<Mode>,<MinDur>,<3D_AccLimit>,...
// Mode 1 = Survey-in | Mode 2 = Fixed
// 3D_AccLimit em METROS (nao em 0.1mm como imaginavamos!)
struct Config {
  uint8_t  svinMode     = 1;      // 1=Survey-in, 2=Fixed
  uint32_t svinMinDur   = 3600;   // AMOSTRAS (nao segundos!) que atingem o limite de precisao
  float    svinAccLimit = 15.0;   // metros - limite de precisao por amostra
  uint8_t  rtcmMode     = 0;      // 0=MSM4, 1=MSM7 (PAIR432)
  bool     rtcmARP      = true;   // habilita 1005 (PAIR434)
  bool     rtcmEph      = true;   // habilita efemerides (PAIR436)
  uint32_t loraFreq     = 915000000;
};
Config cfg;

// -- Estado da base ---------------------------------------------------
struct BaseState {
  bool     svinAtivo      = false;
  bool     svinCompleto   = false;
  float    svinAccM       = 0.0;
  uint32_t svinSegundos   = 0;      // tempo decorrido no SVIN
  uint8_t  svinPct        = 0;      // porcentagem calculada (0-100)
  float    lat            = 0.0;
  float    lon            = 0.0;
  float    alt            = 0.0;
  int      satelites      = 0;
  uint32_t rtcmEnviados   = 0;
  uint32_t bytesEnviados  = 0;
  unsigned long ultimoEnvio  = 0;
  unsigned long svinInicio   = 0;   // millis() quando SVIN foi iniciado
  String   firmware       = "";
  // Coordenadas ECEF calculadas pelo SVIN (para uso no modo Fixed)
  double   ecefX          = 0.0;
  double   ecefY          = 0.0;
  double   ecefZ          = 0.0;
  bool     ecefDisponivel = false;
};
BaseState base;

// -- Satelites SNR ----------------------------------------------------
struct Satelite { char id[6] = ""; int snr = 0; bool ativo = false; };
#define MAX_SATS 40
Satelite sats[MAX_SATS];
unsigned long ultimoGSV = 0;

// -- LoRa -------------------------------------------------------------
static RadioEvents_t RadioEvents;
uint8_t txBuffer[BUFFER_SIZE];
bool loraPronto = true;

// -- RTCM / NMEA buffers ----------------------------------------------
uint8_t  rtcmBuffer[BUFFER_SIZE];
uint16_t rtcmLen      = 0;
uint16_t rtcmEsperado = 0;
bool     coletandoRTCM = false;
String   nmeaLinha     = "";

// -- BLE --------------------------------------------------------------
BLEServer*         bleServer  = nullptr;
BLECharacteristic* charStatus = nullptr;
BLECharacteristic* charLog    = nullptr;
bool bleConectado = false;

unsigned long ultimoDisplay  = 0;
unsigned long ultimoStatus   = 0;
unsigned long ultimoInit     = 0;
bool          inicializado   = false;

// ---------------------------------------------------------------------
void VextON(void) { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

void atualizarDisplay();

// -- Log unificado Serial + BLE ---------------------------------------
void logMsg(String msg) {
  Serial.println(msg);
  if (bleConectado && charLog) {
    if (msg.length() > 500) msg = msg.substring(0, 500);
    charLog->setValue(msg.c_str());
    charLog->notify();
  }
}

// -- Callbacks LoRa ---------------------------------------------------
void OnTxDone(void) {
  loraPronto = true;
}
void OnTxTimeout(void) {
  logMsg("[LORA] TX Timeout");
  Radio.Sleep();
  loraPronto = true;
}

// -- Helpers NMEA -----------------------------------------------------
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

// -- LoRa -------------------------------------------------------------
void configurarLoRa() {
  Radio.SetChannel(cfg.loraFreq);
  Radio.SetTxConfig(
    MODEM_LORA, TX_OUTPUT_POWER, 0,
    LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
    LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
    LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0,
    LORA_IQ_INVERSION_ON, 3000
  );
  logMsg("[LORA] Freq: " + String(cfg.loraFreq) + " Hz");
}

// -- Configurar LC29H BS (comandos corretos do datasheet) -------------

// Secao 2.2.4: Consulta versao do firmware
void consultarFirmware() {
  enviarNMEA("$PQTMVERNO");
}

// Secao 2.2.1: Consulta config atual do SVIN
void consultarSVIN() {
  enviarNMEA("$PQTMCFGSVIN,R");
}

// Consulta estado atual do RTCM
void consultarModoRTCM() {
  enviarNMEA("$PAIR435"); delay(200);
}

// Habilita mensagens NMEA e RTCM
// PAIR062: type 0=GLL,1=RMC,2=VTG,3=GGA,4=GSA,5=GSV
// Habilita apenas NMEA - pode ser chamada a qualquer momento
void habilitarNMEA() {
  enviarNMEA("$PAIR062,3,1"); delay(150); // GGA
  enviarNMEA("$PAIR062,0,1"); delay(150); // GLL
  enviarNMEA("$PAIR062,1,1"); delay(150); // RMC
  enviarNMEA("$PAIR062,4,1"); delay(150); // GSA
  enviarNMEA("$PAIR062,5,1"); delay(150); // GSV
  logMsg("[NMEA] Sentencas habilitadas");
}

// Habilita RTCM MSM4 - chamar SOMENTE apos SVIN completar ou modo Fixed
// O LC29H BS rejeita PAIR432 sem posicao fixada
void habilitarMSM4() {
  logMsg("[RTCM] Habilitando MSM4...");
  if (cfg.rtcmMode == 0) {
    enviarNMEA("$PAIR432,1,1,1,1,0,0,1"); // 1074/1084/1094/1124
  } else {
    enviarNMEA("$PAIR432,2,2,2,2,0,0,1"); // 1077/1087/1097/1127
  }
  delay(300);
  enviarNMEA("$PAIR434,1"); delay(200); // 1005 ARP
  logMsg("[RTCM] MSM4 + 1005 enviados");
}

// Mantida por compatibilidade com chamadas existentes
void habilitarRTCM() {
  habilitarNMEA();
}

// Secao 2.2.1: PQTMCFGSVIN - Survey-in ou Fixed
// ATENCAO: MinDur = numero de AMOSTRAS (nao segundos!) que atingem AccLimit
// AccLimit = precisao maxima em METROS por amostra
// Fonte: https://indystry.cc/how-to-use-lc29h-the-cheapest-gps-rtk-module/
void ativarSVIN() {
  String cmd;
  if (cfg.svinMode == 1) {
    // Survey-in: $PQTMCFGSVIN,W,1,<MinDur>,<3D_AccLimit>,<ECEF_X>,<ECEF_Y>,<ECEF_Z>
    // Datasheet: campos ECEF obrigatorios - usar 0,0,0 no modo Survey-in
    cmd = "$PQTMCFGSVIN,W,1,"
          + String(cfg.svinMinDur) + ","
          + String((int)cfg.svinAccLimit)  // sem casas decimais
          + ",0,0,0";
  } else {
    // Fixed mode: requer coordenadas ECEF reais
    logMsg("[BASE] Modo Fixed requer coordenadas ECEF - use Survey-in primeiro");
    return;
  }
  enviarNMEA(cmd);
  delay(300);

  // Secao 2.2.2: salva na NVM
  enviarNMEA("$PQTMSAVEPAR");
  delay(500);

  base.svinAtivo    = true;
  base.svinCompleto = false;
  base.svinSegundos = 0;
  base.svinPct      = 0;
  base.svinInicio   = millis();
  logMsg("[BASE] SVIN iniciado | amostras:" + String(cfg.svinMinDur)
         + " accLim:" + String(cfg.svinAccLimit, 1) + "m");
}

// -- Converte Lat/Lon/Alt (graus/metros) para ECEF WGS84 --------------
void latLonAltParaECEF(double lat, double lon, double alt,
                        double &X, double &Y, double &Z) {
  const double a  = 6378137.0;           // semi-eixo maior WGS84
  const double e2 = 6.6943799901414e-3;  // excentricidade2
  double latR = lat * DEG_TO_RAD;
  double lonR = lon * DEG_TO_RAD;
  double N = a / sqrt(1.0 - e2 * sin(latR) * sin(latR));
  X = (N + alt) * cos(latR) * cos(lonR);
  Y = (N + alt) * cos(latR) * sin(lonR);
  Z = (N * (1.0 - e2) + alt) * sin(latR);
}

// -- Ativa modo Fixed com coordenadas ECEF salvas ---------------------
// Datasheet 2.2.1: $PQTMCFGSVIN,W,2,<MinDur>,<AccLimit>,<X>,<Y>,<Z>
// No modo Fixed, MinDur e AccLimit sao ignorados - usar 0,0
void ativarModoFixed() {
  if (!base.ecefDisponivel) {
    logMsg("[FIXED] Sem coordenadas ECEF - complete o SVIN primeiro!");
    return;
  }

  // Verifica se ECEF e plausivel (nao pode ser 0,0,0)
  if (abs(base.ecefX) < 1000.0) {
    logMsg("[FIXED] Coordenadas ECEF invalidas! Refaca o SVIN ao ar livre.");
    return;
  }

  // Formato: $PQTMCFGSVIN,W,2,0,0,X,Y,Z (sem casas decimais excessivas)
  char cmd[150];
  snprintf(cmd, sizeof(cmd),
    "$PQTMCFGSVIN,W,2,0,0,%.4f,%.4f,%.4f",
    base.ecefX, base.ecefY, base.ecefZ);

  logMsg("[FIXED] Enviando: " + String(cmd));
  enviarNMEA(String(cmd));
  delay(300);

  // Salva na NVM - o ACK chega pelo processarNMEA no loop principal
  enviarNMEA("$PQTMSAVEPAR");
  delay(300);
  cfg.svinMode = 2;
  logMsg("[FIXED] Modo Fixed enviado - aguardando confirmacao...");
  logMsg("[FIXED] X=" + String(base.ecefX, 4)
       + " Y=" + String(base.ecefY, 4)
       + " Z=" + String(base.ecefZ, 4));

  // Aguarda 1s para o modulo processar e habilita MSM4
  delay(1000);
  habilitarMSM4();
}

// -- Aplicar config via BLE --------------------------------------------
void aplicarConfig(String json) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, json)) {
    logMsg("[CFG] JSON invalido"); return;
  }

  bool reiniciarSVIN = false;
  bool reiniciarLoRa = false;
  bool reiniciarRTCM = false;

  if (doc.containsKey("svinMode"))     { cfg.svinMode     = doc["svinMode"];                   reiniciarSVIN = true; }
  if (doc.containsKey("svinMinDur"))   { cfg.svinMinDur   = doc["svinMinDur"].as<uint32_t>();  reiniciarSVIN = true; }
  if (doc.containsKey("svinAccLimit")) { cfg.svinAccLimit = doc["svinAccLimit"].as<float>();   reiniciarSVIN = true; }
  if (doc.containsKey("rtcmMode"))     { cfg.rtcmMode     = doc["rtcmMode"];                   reiniciarRTCM = true; }
  if (doc.containsKey("rtcmARP"))      { cfg.rtcmARP      = doc["rtcmARP"];                   reiniciarRTCM = true; }
  if (doc.containsKey("rtcmEph"))      { cfg.rtcmEph      = doc["rtcmEph"];                   reiniciarRTCM = true; }
  if (doc.containsKey("loraFreq"))     { cfg.loraFreq     = doc["loraFreq"].as<uint32_t>();    reiniciarLoRa = true; }

  logMsg("[CFG] Config aplicada!");
  if (reiniciarLoRa) configurarLoRa();
  if (reiniciarRTCM) habilitarRTCM();
  if (reiniciarSVIN) ativarSVIN();

  // Comando especial: ativar modo Fixed com coordenadas do SVIN
  if (doc.containsKey("ativarFixed") && doc["ativarFixed"] == true) {
    ativarModoFixed();
  }
  // Comando especial: voltar para Survey-in
  if (doc.containsKey("voltarSVIN") && doc["voltarSVIN"] == true) {
    logMsg("[SVIN] Reiniciando Survey-in...");
    // Primeiro desabilita o modo Fixed/SVIN atual
    enviarNMEA("$PQTMCFGSVIN,W,0,0,0,0,0,0"); // Modo 0 = desabilita
    delay(500);
    // Restaura parametros padrao
    enviarNMEA("$PQTMRESTOREPAR");
    delay(1000);
    // Reabilita apenas NMEA (MSM4 sera reabilitado apos SVIN completar)
    habilitarNMEA();
    delay(300);
    // Inicia novo SVIN
    cfg.svinMode      = 1;
    base.svinCompleto = false;
    base.svinAtivo    = false;
    base.ecefDisponivel = false;
    base.lat = 0; base.lon = 0; base.alt = 0;
    ativarSVIN();
  }
}

// -- Converte ECEF para Lat/Lon/Alt (WGS84) ---------------------------
void ecefParaLatLon(double X, double Y, double Z,
                    double &lat, double &lon, double &alt) {
  const double a  = 6378137.0;
  const double e2 = 6.6943799901414e-3;
  lon = atan2(Y, X) * 180.0 / M_PI;
  double p   = sqrt(X*X + Y*Y);
  double lat0 = atan2(Z, p * (1.0 - e2));
  for (int i = 0; i < 10; i++) {
    double N = a / sqrt(1.0 - e2 * sin(lat0) * sin(lat0));
    lat0 = atan2(Z + e2 * N * sin(lat0), p);
  }
  lat = lat0 * 180.0 / M_PI;
  double N = a / sqrt(1.0 - e2 * sin(lat0) * sin(lat0));
  alt = p / cos(lat0) - N;
}

// -- Status JSON via BLE -----------------------------------------------
void enviarStatusBLE() {
  if (!bleConectado || !charStatus) return;

  // Se em modo Fixed e lat/lon zerados, calcula a partir do ECEF
  if (cfg.svinMode == 2 && base.ecefDisponivel &&
      base.lat == 0.0 && base.ecefX != 0.0) {
    double lat, lon, alt;
    ecefParaLatLon(base.ecefX, base.ecefY, base.ecefZ, lat, lon, alt);
    base.lat = (float)lat;
    base.lon = (float)lon;
    base.alt = (float)alt;
  }

  StaticJsonDocument<512> doc;
  // Calcula porcentagem SVIN:
  // Progresso = max(% por tempo, % por precisao)
  // % tempo  = segundos_decorridos / minDur * 100
  // % acc    = (accInicial - accAtual) / (accInicial - accAlvo) * 100
  if (base.svinAtivo && !base.svinCompleto && base.svinInicio > 0) {
    uint32_t elapsed = (millis() - base.svinInicio) / 1000;
    base.svinSegundos = elapsed;
    uint8_t pctTempo = (cfg.svinMinDur > 0)
      ? (uint8_t)min(100UL, (unsigned long)(elapsed * 100 / cfg.svinMinDur))
      : 0;
    uint8_t pctAcc = 0;
    if (base.svinAccM > 0 && cfg.svinAccLimit > 0) {
      // Precisao melhora de ~50m (inicio tipico) ate o alvo
      float accInicial = 50.0;
      float progresso  = (accInicial - base.svinAccM) / (accInicial - cfg.svinAccLimit);
      pctAcc = (uint8_t)(constrain(progresso * 100.0, 0.0, 100.0));
    }
    base.svinPct = max(pctTempo, pctAcc);
  }
  if (base.svinCompleto) base.svinPct = 100;

  doc["svinAtivo"]    = base.svinAtivo;
  doc["svinCompleto"] = base.svinCompleto;
  doc["svinPct"]      = base.svinPct;
  doc["svinSeg"]      = base.svinSegundos;
  doc["svinAccM"]     = serialized(String(base.svinAccM, 2));
  doc["lat"]          = serialized(String(base.lat, 7));
  doc["lon"]          = serialized(String(base.lon, 7));
  doc["alt"]          = serialized(String(base.alt, 2));
  doc["sats"]         = base.satelites;
  doc["rtcmEnv"]      = base.rtcmEnviados;
  doc["bytesEnv"]     = base.bytesEnviados;
  doc["firmware"]     = base.firmware;
  doc["cfgSvinMode"]  = cfg.svinMode;
  doc["cfgSvinDur"]   = cfg.svinMinDur;
  doc["cfgSvinAcc"]   = cfg.svinAccLimit;
  doc["cfgRtcmMode"]    = cfg.rtcmMode;
  doc["cfgLoraFreq"]    = cfg.loraFreq;
  doc["ecefDisponivel"] = base.ecefDisponivel;
  if (base.ecefDisponivel) {
    doc["ecefX"] = serialized(String(base.ecefX, 4));
    doc["ecefY"] = serialized(String(base.ecefY, 4));
    doc["ecefZ"] = serialized(String(base.ecefZ, 4));
  }

  String out; serializeJson(doc, out);
  charStatus->setValue(out.c_str());
  charStatus->notify();

  // SNR dos satelites - enviado pelo charStatus com delay para nao colidir
  if (millis() - ultimoGSV < 5000) {
    delay(100); // delay para nao colidir com o status anterior
    StaticJsonDocument<1024> snrDoc;
    JsonArray arr = snrDoc.createNestedArray("snr");
    for (int i = 0; i < MAX_SATS; i++) {
      if (sats[i].ativo && sats[i].snr > 0) {
        JsonObject o = arr.createNestedObject();
        o["id"]  = sats[i].id;
        o["snr"] = sats[i].snr;
      }
    }
    String snrOut; serializeJson(snrDoc, snrOut);
    charStatus->setValue(snrOut.c_str());
    charStatus->notify();
  }
}

// -- BLE Callbacks -----------------------------------------------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConectado = true;
    logMsg("[BLE] Conectado");
    atualizarDisplay();
  }
  void onDisconnect(BLEServer* s) override {
    bleConectado = false;
    logMsg("[BLE] Desconectado");
    BLEDevice::startAdvertising();
    atualizarDisplay();
  }
};

class ConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String val = c->getValue().c_str();
    logMsg("[BLE] Config: " + val);
    aplicarConfig(val);
  }
};

void iniciarBLE() {
  BLEDevice::init("BASE-RTK");
  BLEDevice::setMTU(512);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  BLECharacteristic* charConfig = svc->createCharacteristic(
    BLE_CHAR_CONFIG_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  charConfig->setCallbacks(new ConfigCallbacks());

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
  logMsg("[BLE] BASE-RTK anunciando");
}

// -- Display -----------------------------------------------------------
void atualizarDisplay() {
  display.clear();

  if (base.svinCompleto) {
    display.drawString(0, 0, "BASE FIXO" + String(bleConectado ? " [BLE]" : ""));
  } else if (base.svinAtivo) {
    display.drawString(0, 0, "SVIN " + String(base.svinPct) + "% "
      + String(base.svinSegundos) + "s"
      + String(bleConectado ? " [B]" : ""));
    // Barra de progresso
    int barW = (int)(base.svinPct * 126 / 100);
    display.fillRect(0, 9, barW, 3);
  } else {
    display.drawString(0, 0, "Aguardando SVIN"
      + String(bleConectado ? " [BLE]" : ""));
  }
  display.drawLine(0, 12, 128, 12);

  if (base.lat != 0.0) {
    display.drawString(0, 13, "Lat: " + String(base.lat, 6));
    display.drawString(0, 24, "Lon: " + String(base.lon, 6));
    display.drawString(0, 35, "Alt: " + String(base.alt, 1) + "m");
  } else {
    display.drawString(0, 13, "Sem posicao...");
    display.drawString(0, 24, "Leve ao ar livre!");
  }

  display.drawString(0, 46,
    "Sats:" + String(base.satelites) +
    " RTCM:" + String(base.rtcmEnviados));

  String bt = "TX:" + String(base.bytesEnviados) + "B";
  if (base.ultimoEnvio > 0)
    bt += " " + String((millis() - base.ultimoEnvio) / 1000) + "s";
  display.drawString(0, 55, bt);
  display.display();
}

// -- Parser CSV --------------------------------------------------------
void splitCSV(String linha, String* c, int max) {
  int campo = 0, ini = 0;
  for (int i = 0; i <= (int)linha.length() && campo < max; i++) {
    if (i == (int)linha.length() || linha[i] == ',') {
      c[campo++] = linha.substring(ini, i);
      ini = i + 1;
    }
  }
}

// -- Parser NMEA -------------------------------------------------------
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

  // -- GxGLL - Lat/Lon (sentenca enviada pelo LC29H BS no modo base) --
  // Formato: $GNGLL,<lat>,<N/S>,<lon>,<E/W>,<hora>,<status>,<mode>
  if (linha.length() > 6 && linha.indexOf("GLL") == 3) {
    String c[8]; splitCSV(linha, c, 8);
    // c[6] = status: A=valido, V=invalido
    String status = c[6]; status.replace("*",""); status.trim();
    if (c[1].length() > 0 && status != "V") {
      float rLat = c[1].toFloat(); int dLat = (int)(rLat/100);
      base.lat = dLat + (rLat - dLat*100)/60.0;
      if (c[2]=="S") base.lat = -base.lat;
      float rLon = c[3].toFloat(); int dLon = (int)(rLon/100);
      base.lon = dLon + (rLon - dLon*100)/60.0;
      if (c[4]=="W") base.lon = -base.lon;
    }
  }

  // -- GxRMC - Posicao + satelites validos --------------------------
  // Formato: $GNRMC,<hora>,<status>,<lat>,<N/S>,<lon>,<E/W>,<vel>,<curso>,<data>,...
  if (linha.length() > 6 && linha.indexOf("RMC") == 3) {
    String c[12]; splitCSV(linha, c, 12);
    // c[2] = status: A=valido, V=invalido
    if (c[2] == "A" && c[3].length() > 0) {
      float rLat = c[3].toFloat(); int dLat = (int)(rLat/100);
      base.lat = dLat + (rLat - dLat*100)/60.0;
      if (c[4]=="S") base.lat = -base.lat;
      float rLon = c[5].toFloat(); int dLon = (int)(rLon/100);
      base.lon = dLon + (rLon - dLon*100)/60.0;
      if (c[6]=="W") base.lon = -base.lon;
    }
  }

  // -- GGA - mantido como fallback caso firmware futuro envie -------
  if (linha.length() > 6 && linha.indexOf("GGA") == 3) {
    String c[15]; splitCSV(linha, c, 15);
    if (c[2].length() > 0) {
      float rLat = c[2].toFloat(); int dLat = (int)(rLat/100);
      base.lat = dLat + (rLat - dLat*100)/60.0;
      if (c[3]=="S") base.lat = -base.lat;
      float rLon = c[4].toFloat(); int dLon = (int)(rLon/100);
      base.lon = dLon + (rLon - dLon*100)/60.0;
      if (c[5]=="W") base.lon = -base.lon;
      base.alt       = c[9].toFloat();
      base.satelites = c[7].toInt();
    }
  }

  // -- PQTMCFGSVIN resposta ----------------------------------------
  // Set: $PQTMCFGSVIN,OK
  // Get: $PQTMCFGSVIN,OK,<Mode>,<MinDur>,<3D_AccLimit>,<X>,<Y>,<Z>
  // Err: $PQTMCFGSVIN,ERROR,<code>
  if (linha.startsWith("$PQTMCFGSVIN")) {
    if (linha.indexOf(",ERROR") >= 0) {
      logMsg("[SVIN] ERRO! Verifique parametros.");
    } else if (linha.indexOf(",OK") >= 0) {
      String c[10]; splitCSV(linha, c, 10);
      // Resposta do GET tem mais de 2 campos
      if (c[2].length() > 0 && c[2] != "") {
        // Resposta GET: c[1]=OK, c[2]=Mode, c[3]=MinDur, c[4]=AccLimit
        int    modo   = c[2].toInt();
        uint32_t dur  = c[3].toInt();
        float  acc    = c[4].toFloat();
        String modoStr = modo == 0 ? "Desabilitado"
                       : modo == 1 ? "Survey-in"
                       : modo == 2 ? "Fixed" : "Desconhecido";
        logMsg("[SVIN] Modo=" + modoStr
               + " MinDur=" + String(dur) + "s"
               + " AccLimit=" + String(acc, 2) + "m");
        // Atualiza config local com valores reais do modulo
        cfg.svinMode     = modo;
        cfg.svinMinDur   = dur;
        cfg.svinAccLimit = acc;
        if (modo == 1 || modo == 2) base.svinAtivo = true;
      } else {
        // Resposta simples do SET
        logMsg("[SVIN] Comando aceito!");
        base.svinAtivo = true;
      }
    }
  }

  // -- PQTMSAVEPAR resposta -----------------------------------------
  if (linha.startsWith("$PQTMSAVEPAR")) {
    if (linha.indexOf(",OK") >= 0) logMsg("[NVM] Configuracao salva!");
    else logMsg("[NVM] Erro ao salvar");
  }

  // -- PQTMVERNO - versao do firmware ------------------------------
  // Datasheet 2.2.4: $PQTMVERNO,<VerStr>,<BuildDate>,<BuildTime>
  if (linha.startsWith("$PQTMVERNO,") && linha.indexOf("ERROR") < 0) {
    String c[5]; splitCSV(linha, c, 5);
    if (c[1].length() > 0) {
      base.firmware = c[1];
      logMsg("[FW] " + base.firmware);
    }
  }

  // -- PAIR001 - ACK/NACK dos comandos PAIR ------------------------
  // Datasheet 2.3.1: $PAIR001,<CommandID>,<r>
  // r: 0=enviado, 1=processando, 2=falhou, 3=nao suportado, 4=param erro
  // PQTMEPE - Estimated Position Error (progresso do SVIN)
  // $PQTMEPE,<mode>,<EPE_N>,<EPE_E>,<EPE_H>,<EPE_3D>,<EPE_5D>
  if (linha.startsWith("$PQTMEPE")) {
    String c[8]; splitCSV(linha, c, 8);
    if (c[4].length() > 0) {
      float epeH = c[4].toFloat(); // erro horizontal em metros
      base.svinAccM = epeH;

      // Calcula progresso baseado no EPE_H convergindo para o alvo
      if (cfg.svinAccLimit > 0 && epeH > 0) {
        float accInicial = 200.0; // EPE inicial tipico ~200m
        float progresso  = (accInicial - epeH) / (accInicial - cfg.svinAccLimit);
        base.svinPct = (uint8_t)(constrain(progresso * 100.0, 0.0, 99.0));
      }

      // Marca SVIN como ativo se recebendo EPE com valores
      if (epeH > 0 && !base.svinCompleto) {
        base.svinAtivo = true;
      }

      // SVIN completo quando EPE_H <= alvo configurado
      if (epeH <= cfg.svinAccLimit && !base.svinCompleto && base.svinAtivo) {
        base.svinCompleto = true; // marca ANTES para nao repetir
        base.svinAtivo    = false;
        base.svinPct      = 100;
        logMsg("[SVIN] CONCLUIDO! EPE_H=" + String(epeH, 2) + "m");
        // Calcula coordenadas ECEF
        if (base.lat != 0.0 && base.lon != 0.0) {
          latLonAltParaECEF(base.lat, base.lon, base.alt,
                            base.ecefX, base.ecefY, base.ecefZ);
          base.ecefDisponivel = true;
          // Ativa modo Fixed automaticamente com as coordenadas calculadas
          // Fixed mantem o modulo em modo base gerando MSM4 continuamente
          logMsg("[SVIN] Ativando modo Fixed automaticamente...");
          delay(500);
          ativarModoFixed();
          logMsg("[ECEF] X=" + String(base.ecefX, 4)
               + " Y=" + String(base.ecefY, 4)
               + " Z=" + String(base.ecefZ, 4));
        }
      }
    }
  }

  // PAIR433 - resposta GET RTCM mode
  if (linha.startsWith("$PAIR433,") && !linha.startsWith("$PAIR433*")) {
    String c[3]; splitCSV(linha, c, 3);
    String mode = c[1];
    int sp1 = mode.indexOf('*'); if (sp1>=0) mode = mode.substring(0,sp1);
    mode.trim();
    String modeStr = mode == "-1" ? "DESABILITADO"
                   : mode == "0"  ? "MSM4 ativo"
                   : mode == "1"  ? "MSM7 ativo" : "modo=" + mode;
    logMsg("[RTCM] Modo atual: " + modeStr);
  }

  // PAIR435 - resposta GET ARP
  if (linha.startsWith("$PAIR435,") && !linha.startsWith("$PAIR435*")) {
    String c[3]; splitCSV(linha, c, 3);
    String en = c[1];
    int sp2 = en.indexOf('*'); if (sp2>=0) en = en.substring(0,sp2);
    en.trim();
    logMsg("[RTCM] ARP(1005): " + String(en == "1" ? "HABILITADO" : "DESABILITADO"));
  }

  // PAIR437 - resposta GET EPH
  if (linha.startsWith("$PAIR437,") && !linha.startsWith("$PAIR437*")) {
    String c[3]; splitCSV(linha, c, 3);
    String en = c[1];
    int sp3 = en.indexOf('*'); if (sp3>=0) en = en.substring(0,sp3);
    en.trim();
    logMsg("[RTCM] Efemerides: " + String(en == "1" ? "HABILITADAS" : "DESABILITADAS"));
  }

  // PAIR001 - ACK/NACK
  // Datasheet 2.3.1: $PAIR001,<CommandID>,<r>
  // Nota: o campo CommandID na resposta pode ser o ID interno do chipset (ex:2410)
  // ou o numero do comando PAIR (ex:432). Ambos sao logados.
  if (linha.startsWith("$PAIR001")) {
    String c[5]; splitCSV(linha, c, 5);
    String cmdId = c[1];
    // Remove checksum que pode estar colado no campo r (ex: "0*38" -> "0")
    String r = c[2];
    int starPos = r.indexOf('*');
    if (starPos >= 0) r = r.substring(0, starPos);
    r.trim();

    // Mapeamento PAIR -> nome (pelo numero do comando)
    struct { const char* id; const char* nome; } mapa[] = {
      {"432","PAIR432 RTCM_MODE"},
      {"433","PAIR433 GET_MODE"},
      {"434","PAIR434 RTCM_ARP(1005)"},
      {"435","PAIR435 GET_ARP"},
      {"436","PAIR436 RTCM_EPH"},
      {"437","PAIR437 GET_EPH"},
    };
    String nome = "CMD_ID=" + cmdId;
    for (auto& m : mapa)
      if (cmdId == m.id) { nome = m.nome; break; }

    String status;
    if      (r=="0") status = "OK";
    else if (r=="1") status = "Processando";
    else if (r=="2") status = "FALHOU";
    else if (r=="3") status = "NAO SUPORTADO";
    else if (r=="4") status = "ERRO PARAMETRO";
    else             status = "r=" + r;

    logMsg("[ACK] " + nome + " -> " + status);


  }

  // -- GxRMC - posicao alternativa (fallback ao GGA) --------------
  if (linha.length() > 6 && linha.indexOf("RMC") == 3) {
    String c[12]; splitCSV(linha, c, 12);
    // $GNRMC,<time>,<status>,<lat>,<N/S>,<lon>,<E/W>,...
    // status: A=valido, V=invalido
    if (c[2] == "A" && c[3].length() > 0) {
      float rLat = c[3].toFloat(); int dLat = (int)(rLat/100);
      base.lat = dLat + (rLat - dLat*100)/60.0;
      if (c[4] == "S") base.lat = -base.lat;
      float rLon = c[5].toFloat(); int dLon = (int)(rLon/100);
      base.lon = dLon + (rLon - dLon*100)/60.0;
      if (c[6] == "W") base.lon = -base.lon;
      // posicao atualizada (visivel no status BLE)
    }
  }

  // -- GxGSA - satelites em uso ------------------------------------
  if (linha.length() > 6 && linha.indexOf("GSA") == 3) {
    String c[20]; splitCSV(linha, c, 20);
    // $GNGSA,<mode>,<fix>,<sv1>...<sv12>,<PDOP>,<HDOP>,<VDOP>
    // Conta satelites nao-vazios entre c[3] e c[14]
    int count = 0;
    for (int i = 3; i <= 14; i++) {
      if (c[i].length() > 0 && c[i] != "0" && c[i] != "") count++;
    }
    if (count > 0) base.satelites = count;
  }

  // -- GxGSV - SNR dos satelites ------------------------------------
  if (linha.length() > 4 && linha[0]=='$' && linha.indexOf("GSV")==3) {
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
      int  snr  = c[bi+3].toInt();
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
    ultimoGSV = millis();

    // Conta total de satelites visiveis somando todos os sistemas
    int total = 0;
    for (int i = 0; i < MAX_SATS; i++) {
      if (sats[i].ativo && sats[i].snr > 0) total++;
    }
    if (total > 0) base.satelites = total;
  }
}

// -- RTCM via LoRa ----------------------------------------------------
void enviarRTCMLoRa(uint8_t* dados, uint16_t len) {
  if (len == 0 || len > BUFFER_SIZE) return;

  // Timeout de seguranca: se loraPronto travou, reseta apos 3s
  static unsigned long ultimoTx = 0;
  if (!loraPronto && millis() - ultimoTx > 3000) {
    logMsg("[LORA] AVISO: timeout TX - resetando LoRa");
    Radio.Sleep();
    delay(10);
    configurarLoRa();
    loraPronto = true;
  }

  if (!loraPronto) return; // ainda aguardando TX anterior

  // Extrai tipo da mensagem RTCM para log
  uint16_t tipo = 0;
  if (len >= 4) {
    tipo = ((uint16_t)(dados[3] & 0xFF) << 4) | ((uint16_t)(dados[4] & 0xF0) >> 4);
  }

  loraPronto = false;
  ultimoTx   = millis();
  memcpy(txBuffer, dados, len);
  Radio.Send(txBuffer, len);
  base.rtcmEnviados++;
  base.bytesEnviados += len;
  base.ultimoEnvio    = millis();
  logMsg("[LORA] TX tipo:" + String(tipo) + " " + String(len) + "B msg#" + String(base.rtcmEnviados));
}

void processarByteRTCM(uint8_t b) {
  if (!coletandoRTCM) {
    if (b == 0xD3) {
      coletandoRTCM = true;
      rtcmLen = 0;
      rtcmEsperado = 0;
      rtcmBuffer[rtcmLen++] = b;
    }
  } else {
    if (rtcmLen < BUFFER_SIZE) rtcmBuffer[rtcmLen++] = b;
    // Calcula tamanho esperado quando byte 3 e 4 chegam
    if (rtcmLen == 3) {
      rtcmEsperado = (((uint16_t)(rtcmBuffer[1] & 0x03)) << 8 | rtcmBuffer[2]) + 6;
      // Sanidade: mensagem RTCM valida tem entre 6 e 255 bytes
      if (rtcmEsperado < 6 || rtcmEsperado > BUFFER_SIZE) {
        coletandoRTCM = false; rtcmLen = 0; rtcmEsperado = 0;
        return;
      }
    }
    if (rtcmLen >= 3 && rtcmEsperado > 0 && rtcmLen == rtcmEsperado) {
      // Transmite sempre - base gera RTCM mesmo durante SVIN
      enviarRTCMLoRa(rtcmBuffer, rtcmLen);
      coletandoRTCM = false; rtcmLen = 0; rtcmEsperado = 0;
    }
    // Reseta se buffer cheio (mensagem corrompida)
    if (rtcmLen >= BUFFER_SIZE) {
      logMsg("[LORA] AVISO: buffer RTCM cheio - descartando");
      coletandoRTCM = false; rtcmLen = 0; rtcmEsperado = 0;
    }
  }
}

// -- Setup -------------------------------------------------------------
void setup() {
  VextON(); delay(100);
  Serial.begin(115200);
  Serial.println("=== BASE RTK + BLE iniciando ===");
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  display.init();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0,  "BASE RTK");
  display.drawString(0, 16, "Iniciando...");
  display.display();

  // LoRa - inicializa antes do BLE
  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init(&RadioEvents);
  configurarLoRa();

  // UART LC29H
  rtkSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(500);

  // BLE (roda no Core 1)
  iniciarBLE();

  display.clear();
  display.drawString(0, 0,  "BASE RTK");
  display.drawString(0, 16, "BLE: BASE-RTK");
  display.drawString(0, 32, "Aguardando modulo...");
  display.display();
  // Configuracao do modulo feita no loop apos 3s (garante que UART estabilizou)
}

// -- Loop --------------------------------------------------------------
void loop() {
  while (rtkSerial.available()) {
    uint8_t b = rtkSerial.read();
    if (b == 0xD3 || coletandoRTCM) {
      processarByteRTCM(b);
    } else {
      if (b == '\n') {
        processarNMEA(nmeaLinha);
        nmeaLinha = "";
      } else if (b != '\r') {
        nmeaLinha += (char)b;
        if (nmeaLinha.length() > 120) nmeaLinha = "";
      }
    }
  }

  // Inicializa modulo 3s apos boot - garante UART estavel e respostas capturadas
  if (!inicializado && millis() > 3000) {
    inicializado = true;
    logMsg("[INIT] Configurando modulo LC29H...");
    consultarFirmware(); delay(500);
    // PQTMRESTOREPAR removido - apagaria o modo Fixed salvo na NVM
    consultarSVIN();     delay(500);
    // habilitarNMEA removida - consolidada em habilitarRTCM     delay(500);
    habilitarRTCM();     delay(500);
    ativarSVIN();
  }

  Radio.IrqProcess();

  if (millis() - ultimoDisplay > 1000) { atualizarDisplay(); ultimoDisplay = millis(); }
  if (millis() - ultimoStatus  > 3000) { enviarStatusBLE();  ultimoStatus  = millis(); }

  // Envia SNR dedicado a cada 5s se ha dados de satelites recentes
  static unsigned long ultimoSNR = 0;
  if (bleConectado && millis() - ultimoGSV < 8000 && millis() - ultimoSNR > 5000) {
    ultimoSNR = millis();
    delay(150);
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
