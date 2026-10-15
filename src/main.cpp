// main.cpp — leitura do encoder absoluto Broadcom AR49-M49M-P12H via SPI no ESP32.
// (PlatformIO/Arduino: setup()/loop() são chamados pelo main do framework.)
//
// ┌──────────────────────────────────────────────────────────────────────────────┐
// │ ⚠️  LEVEL SHIFTING OBRIGATÓRIO NO MISO (5 V → 3,3 V)                            │
// │ Saída do encoder (SPI-DO) tem V_OH ≥ 4,4 V; GPIO do ESP32 NÃO é 5 V-tolerante. │
// │ CLK/MOSI/NCS podem ir diretos (encoder aceita V_IH = 2,8 V; 3,3 V > 2,8 V).     │
// │ Na PAEAT-DI261941 o TXB0104 (CI4) já faz a conversão nas 4 linhas.             │
// └──────────────────────────────────────────────────────────────────────────────┘
//
// Pinagem do conector JST 8 vias do encoder (DS101 Table 10, variante SPI):
//   Via 1 = +5V     Via 2 = 0V     Via 3 = NCS    Via 4 = CLK
//   Via 5 = DO/MISO Via 6 = DIN/MOSI  Vias 7,8 = NC
//
//   Encoder         PAEAT-DI261941 (default)      | DMB-DI231765 | bancada
//   ----------------------------------------------|--------------|--------
//   CLK  (via 4) ── IO18 -> CI4 A1/B1 -> CN3.3     | GPIO21       | GPIO18
//   NCS  (via 3) ── IO5  -> CI4 A4/B4 -> (*)       | GPIO22       | GPIO5
//   MOSI (via 6) ── IO23 -> CI4 A2/B2 -> CN3.4     | GPIO33       | GPIO23
//   MISO (via 5) ── IO19 <- CI4 A3/B3 <- CN3.5     | GPIO35 (LS)  | GPIO19 (LS)
//   +5V  (via 1) ── CN3.1     0V (via 2) ── CN3.2
//
// PAEAT rev. A: a ordem do CN3 NÃO é a do JST do encoder -> o cabo precisa ser cruzado
// (CN3.3->via 4, CN3.4->via 6). (*) CN3.6 (NCS) ficou sem furo: NCS por fio de
// retrabalho até a via 3 ou, só em bancada, via 3 fixa em 0V.
#include <Arduino.h>
#include "Encoder.h"

// --- Perfil de pinos (selecionado por build flag em platformio.ini) -------------
// -D AR49_BOARD_PAEAT      -> placa PAEAT-DI261941 (default do env esp32dev)
// -D AR49_BOARD_DIELETRONS -> placa antiga DMB-DI231765
// sem flag                 -> bancada / VSPI padrão
// Assim o firmware não "chuta" pinos: o perfil casa com o hardware alvo do build,
// evitando ler GPIO errado (sintoma clássico: eco OC = 0x00).
// PIN_OE / PIN_WDI = -1 => o hardware do perfil não tem esse sinal.
#if defined(AR49_BOARD_PAEAT)
// Placa PAEAT-DI261941 rev. A (encoder absoluto para trena):
constexpr int8_t PIN_SCK  = 18;
constexpr int8_t PIN_MISO = 19;   // já em 3,3 V (lado A do TXB0104)
constexpr int8_t PIN_MOSI = 23;
constexpr int8_t PIN_NCS  = 5;
constexpr int8_t PIN_OE   = 4;    // OE do TXB0104: R8 puxa p/ 0V => precisa ir a HIGH
constexpr int8_t PIN_WDI  = 14;   // WDI do watchdog STWD100 (CI2); RST->EN via jumper J1
#elif defined(AR49_BOARD_DIELETRONS)
// Placa DiEletrons DMB-DI231765 (reaproveitando as 4 linhas do "CND"):
constexpr int8_t PIN_SCK  = 21;
constexpr int8_t PIN_NCS  = 22;
constexpr int8_t PIN_MOSI = 33;
constexpr int8_t PIN_MISO = 35;   // GPIO34/35/36/39 são só-entrada (ok p/ MISO), via level shifter
constexpr int8_t PIN_OE   = -1;
constexpr int8_t PIN_WDI  = -1;
#else
// Bancada / protótipo (VSPI padrão):
constexpr int8_t PIN_SCK  = 18;
constexpr int8_t PIN_MISO = 19;   // via level shifter!
constexpr int8_t PIN_MOSI = 23;
constexpr int8_t PIN_NCS  = 5;
constexpr int8_t PIN_OE   = -1;
constexpr int8_t PIN_WDI  = -1;
#endif

// Clock SPI: 1 MHz é conservador. O encoder aceita até 10 MHz — para subir, troque
// para 10000000UL APÓS validar a fiação e o eco 0xA6 em bancada.
constexpr uint32_t SPI_CLOCK_HZ = 1000000UL;

// Power on standby do encoder: máx. 500 ms (DS101 Table 2) + margem.
constexpr uint32_t ENCODER_POWERUP_MS = 600;
constexpr uint32_t READ_PERIOD_MS     = 100;  // ~10 leituras/s

AR49Encoder encoder(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NCS, SPI_CLOCK_HZ);

// Alimenta o watchdog externo (borda em WDI zera a contagem). O timeout do STWD100
// depende do sufixo (3,4 ms / 6,3 ms / 102 ms / 1,6 s) e não foi confirmado para o
// NYWY3F => o firmware nunca bloqueia: chama isto a cada volta do loop().
static void kickWatchdog() {
  if (PIN_WDI < 0) return;
  static bool level = false;
  level = !level;
  digitalWrite(PIN_WDI, level ? HIGH : LOW);
}

void setup() {
  if (PIN_WDI >= 0) pinMode(PIN_WDI, OUTPUT);
  kickWatchdog();
  if (PIN_OE >= 0) {
    pinMode(PIN_OE, OUTPUT);
    digitalWrite(PIN_OE, HIGH);  // habilita o level shifter (sem isso MISO fica em Hi-Z)
  }

  // Buffer de TX: Serial.printf não bloqueia o loop (e o watchdog) esperando a UART.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);

  // Espera o power-on do encoder alimentando o watchdog (delay() longo resetaria o ESP32).
  const uint32_t t0 = millis();
  while (millis() - t0 < ENCODER_POWERUP_MS) kickWatchdog();

  encoder.begin();
  // encoder.setStrictCrc(true);  // habilite só APÓS validar o CRC contra o encoder físico
  Serial.println(F("AR49-M49M-P12H: iniciando leituras (OC 0xA6)..."));
}

void loop() {
  kickWatchdog();

  static uint32_t lastRead = 0;
  if (millis() - lastRead < READ_PERIOD_MS) return;
  lastRead = millis();

  AR49Reading r;
  const bool ok = encoder.read(r);

  if (!r.ocEchoOk) {
    // Falha primária de comunicação: fiação, level shifter (OE), NCS ou modo SPI errado.
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
}
