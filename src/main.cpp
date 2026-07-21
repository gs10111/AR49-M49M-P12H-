// main.cpp — leitura do encoder absoluto Broadcom AR49-M49M-P12H via SPI no ESP32.
// (PlatformIO/Arduino: setup()/loop() são chamados pelo main do framework.)
//
// ┌──────────────────────────────────────────────────────────────────────────────┐
// │ ⚠️  LEVEL SHIFTING OBRIGATÓRIO NO MISO (5 V → 3,3 V)                            │
// │ Saída do encoder (SPI-DO) tem V_OH ≥ 4,4 V; GPIO do ESP32 NÃO é 5 V-tolerante. │
// │ CLK/MOSI/NCS podem ir diretos (encoder aceita V_IH = 2,8 V; 3,3 V > 2,8 V).     │
// └──────────────────────────────────────────────────────────────────────────────┘
//
// Tabela de ligação (conector 8 vias do encoder, variante SPI):
//   Via 1 = +5V     Via 2 = GND    Via 3 = NCS    Via 4 = CLK
//   Via 5 = DO/MISO Via 6 = DIN/MOSI  Vias 7,8 = NC
//
//   Encoder            ESP32 — perfil DiEletrons DMB-DI231765 (default)   | bancada
//   -----------------------------------------------------------------------|--------
//   CLK   (via 4) ───► GPIO21 (SCK)          direto                        | GPIO18
//   NCS   (via 3) ───► GPIO22 (NCS)          direto                        | GPIO5
//   MOSI  (via 6) ───► GPIO33 (MOSI)         direto                        | GPIO23
//   MISO  (via 5) ──[LEVEL SHIFTER 5V→3V3]─► GPIO35 (MISO, só-entrada)     | GPIO19
//   +5V   (via 1) ───► 5V     GND (via 2) ─► GND
#include <Arduino.h>
#include "Encoder.h"

// --- Perfil de pinos (selecionado por build flag em platformio.ini) -------------
// -D AR49_BOARD_DIELETRONS -> placa DMB-DI231765 (default do env esp32dev)
// sem a flag               -> bancada / VSPI padrão
// Assim o firmware não "chuta" pinos: o perfil casa com o hardware alvo do build,
// evitando ler GPIO errado (sintoma clássico: eco OC = 0x00).
#if defined(AR49_BOARD_DIELETRONS)
// Placa DiEletrons DMB-DI231765 (reaproveitando as 4 linhas do "CND"):
constexpr int8_t PIN_SCK  = 21;
constexpr int8_t PIN_NCS  = 22;
constexpr int8_t PIN_MOSI = 33;
constexpr int8_t PIN_MISO = 35;   // GPIO34/35/36/39 são só-entrada (ok p/ MISO), via level shifter
#else
// Bancada / protótipo (VSPI padrão):
constexpr int8_t PIN_SCK  = 18;
constexpr int8_t PIN_MISO = 19;   // via level shifter!
constexpr int8_t PIN_MOSI = 23;
constexpr int8_t PIN_NCS  = 5;
#endif

// Clock SPI: 1 MHz é conservador. O encoder aceita até 10 MHz — para subir, troque
// para 10000000UL APÓS validar a fiação e o eco 0xA6 em bancada.
constexpr uint32_t SPI_CLOCK_HZ = 1000000UL;

AR49Encoder encoder(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NCS, SPI_CLOCK_HZ);

void setup() {
  Serial.begin(115200);
  delay(500);            // power-on do encoder antes da 1ª leitura
  encoder.begin();
  // encoder.setStrictCrc(true);  // habilite só APÓS validar o CRC contra o encoder físico
  Serial.println(F("AR49-M49M-P12H: iniciando leituras (OC 0xA6)..."));
}

void loop() {
  AR49Reading r;
  const bool ok = encoder.read(r);

  if (!r.ocEchoOk) {
    // Falha primária de comunicação: fiação, level shifter ou modo SPI errado.
    Serial.printf("[COMM] eco OC = 0x%02X (esperado 0xA6). Cheque MISO/level-shift/Modo SPI.\n",
                  r.ocEcho);
  } else {
    Serial.printf("MT=%lu  ST=%lu  ang=%.4f deg  voltas=%.4f  nErr=%d nWar=%d  "
                  "CRC rx=0x%02X calc=0x%02X %s  %s\n",
                  (unsigned long)r.multiTurn, (unsigned long)r.singleTurn,
                  encoder.angleDeg(r), encoder.totalTurns(r),
                  r.nErr ? 1 : 0, r.nWar ? 1 : 0,
                  r.crcRecv, r.crcCalc, r.crcOk ? "OK" : "BAD",
                  ok ? "" : "(valid=false)");
  }

  delay(100);  // ~10 leituras/s
}
