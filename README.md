# RoraiGeo Firmware

Firmware para o sistema RTK GNSS de baixo custo **RoraiGeo**, desenvolvido para o Heltec WiFi LoRa 32 V3 (ESP32-S3) com módulos GNSS Quectel LC29H.

---

## Hardware necessário

| Componente | Estação Base | Rover |
|---|---|---|
| Microcontrolador | Heltec WiFi LoRa 32 V3 | Heltec WiFi LoRa 32 V3 |
| Módulo GNSS | Quectel LC29H (BS) | Quectel LC29H (DA) |
| Antena GNSS | L1+L5 dual-band | L1+L5 dual-band |
| Antena LoRa | 915 MHz | 915 MHz |
| LED RGB | — | Catodo comum (GPIO 5/6/7) |

### Pinagem

| Sinal | GPIO |
|---|---|
| GPS RX | 48 |
| GPS TX | 47 |
| LED R (rover) | 5 |
| LED G (rover) | 6 |
| LED B (rover) | 7 |

---

## Estrutura do repositório

```
firmware/
├── base_rtk/
│   └── base_rtk.ino       # Firmware da estação base
└── rover_rtk/
    └── rover_rtk.ino      # Firmware do rover
```

---

## Dependências — Arduino IDE

Instale as seguintes bibliotecas via **Library Manager** ou manualmente:

- **Heltec ESP32 Dev-Boards** — board package oficial Heltec
  - Adicione em *File → Preferences → Additional boards manager URLs*:
    ```
    https://resource.heltec.cn/download/package_heltec_esp32_index.json
    ```
- **ArduinoJson** `^6.x` — serialização JSON para BLE
- **NimBLE-Arduino** (incluso no Heltec SDK)

### Configurações do Arduino IDE

| Parâmetro | Valor |
|---|---|
| Board | Heltec WiFi LoRa 32 (V3) |
| Upload Speed | 921600 |
| CPU Frequency | 240MHz |
| Flash Size | 8MB |
| Partition Scheme | 8M with spiffs |

---

## Firmware Base (`base_rtk.ino`)

### Funcionalidades

- **Survey-In (SVIN)** automático com precisão configurável (padrão AccLimit = 15m)
- **Modo Fixed** com coordenadas ECEF salvas na NVM do módulo
- **Transmissão RTCM via LoRa 915MHz** com fila de 6 slots e prioridade MSM4
- **BLE 5.0** com características de Status, Config e Log
- **Display OLED** com status em tempo real

### Mensagens RTCM transmitidas

| Tipo | Descrição |
|---|---|
| 1005 | ARP — posição da base |
| 1074 / 1084 / 1094 / 1124 | MSM4 — observações GPS/GLO/GAL/BDS |
| 1019 / 1020 / 1042 / 1046 | Efemérides GPS/GLO/BDS/GAL |

### Fluxo de operação

```
Boot → habilitarNMEA() → consultarSVIN()
  ├── Modo 1 (Survey-In): aguarda EPE_H ≤ AccLimit → ativarFixed() → restart
  ├── Modo 2 (Fixed):     habilitarMSM4() → transmite RTCM via LoRa
  └── Modo 0 (Off):       ativarSVIN()
```

### UUIDs BLE

| Característica | UUID |
|---|---|
| Service | `12345678-1234-1234-1234-123456789abc` |
| Config | `12345678-1234-1234-1234-123456789001` |
| Status | `12345678-1234-1234-1234-123456789002` |
| Log | `12345678-1234-1234-1234-123456789003` |

---

## Firmware Rover (`rover_rtk.ino`)

### Funcionalidades

- **Recepção LoRa** contínua com padrão oficial Heltec (`loraIdle` flag)
- **Parser RTCM 1005** — extração de coordenadas ECEF da base e conversão para lat/lon
- **Injeção RTCM** direta no LC29H DA via UART
- **Parser NMEA** — GGA, RMC, GSA, GSV, PQTMEPE
- **BLE 5.0** — envia JSON com posição, fix quality e referência da base
- **LED RGB** de status (Vermelho = sem fix, Azul = Float, Verde = Fixed)
- **Diagnóstico** periódico a cada 10s

### LED RGB

| Estado | Cor | R/G/B |
|---|---|---|
| Sem GPS (fix=0) | Vermelho | 1/0/0 |
| RTK Float (fix=5) | Azul | 0/0/1 |
| RTK Fixed (fix=4) | Verde | 0/1/0 |

### JSON BLE enviado

```json
{
  "lat": 2.8197215,
  "lon": -60.7803955,
  "alt": 100.7,
  "sats": 28,
  "epeH": 0.012,
  "fixQuality": 4,
  "rtcmRecv": 4889,
  "baseLat": 2.8196826,
  "baseLon": -60.7803459,
  "rtcmAge": 0
}
```

---

## Link LoRa

| Parâmetro | Valor |
|---|---|
| Frequência | 915 MHz (faixa ISM Brasil) |
| Chip | Semtech SX1262 |
| Modulação | LoRa SF7 BW125kHz |
| Potência TX | 14 dBm |
| Payload máximo | 255 bytes |
| Alcance típico | 2–5 km campo aberto |

---

## Resultados de campo

- **Local:** Boa Vista, Roraima — Brasil (lat ~2.82°N, lon ~-60.78°W)
- **Satélites visíveis:** 28–36 simultâneos
- **RSSI LoRa:** -50 a -65 dBm
- **RTK Float:** atingido em ~2–5 minutos
- **RTK Fixed:** atingido em céu aberto com base estável
- **Precisão comprovada:** coordenadas repetíveis a 7 casas decimais em Fixed

---

## Licença

**Creative Commons Atribuição-NãoComercial 4.0 Internacional (CC BY-NC 4.0)**

Copyright © 2026 Raimison Bezerra de Almeida — Boa Vista, Roraima, Brasil.

Você pode:
- ✅ Compartilhar — copiar e redistribuir em qualquer formato
- ✅ Adaptar — remixar, transformar e criar a partir do material

Desde que:
- 📌 **Atribuição** — deve dar crédito ao autor original (Raimison Bezerra de Almeida)
- 🚫 **NãoComercial** — não pode usar para fins comerciais sem autorização prévia

Para uso comercial, entre em contato com o autor.

[![CC BY-NC 4.0](https://licensebuttons.net/l/by-nc/4.0/88x31.png)](https://creativecommons.org/licenses/by-nc/4.0/deed.pt_BR)

> Este projeto foi desenvolvido como Trabalho de Conclusão de Curso (TCC).  

## Autor

**Raimison Bezerra de Almeida**  
raimison01@gmail.com  
Boa Vista, Roraima — Brasil  
Ano: 2026