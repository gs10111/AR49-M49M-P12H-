// Encoder.cpp — implementação do adaptador de I/O do AR49-M49M-P12H.
// SPI por hardware (Modo 0, MSB-first) + NCS controlado por software para cumprir a
// temporização tL/tH/tNCS do datasheet. A interpretação do frame é do AR49Frame.h.
#include "Encoder.h"
#include <Arduino.h>
#include <SPI.h>

namespace {
// Tamanho máximo do frame suportado (bytes). Padrão M49M "Basic" = 9 bytes (65 bits úteis);
// margem para larguras configuráveis maiores (ex.: CRC de 16 bits).
constexpr uint8_t AR49_MAX_FRAME_BYTES = 16;

// Folga de temporização. A 1 MHz, tclk = 1 µs; tL ≥ 350 ns, tH ≥ tclk/2, tNCS ≥ 350 ns.
// 1 µs cobre todos com folga sem introduzir delay longo em laço.
constexpr uint32_t AR49_TL_US   = 1;  // NCS↓ -> 1º clock
constexpr uint32_t AR49_TH_US   = 1;  // último SCK -> NCS↑
constexpr uint32_t AR49_TNCS_US = 1;  // NCS alto entre transações
}  // namespace

AR49Encoder::AR49Encoder(int8_t sck, int8_t miso, int8_t mosi, int8_t ncs,
                         uint32_t clockHz, SPIClass* bus)
    : _sck(sck), _miso(miso), _mosi(mosi), _ncs(ncs), _clockHz(clockHz), _bus(bus) {}

void AR49Encoder::begin() {
  pinMode(_ncs, OUTPUT);
  digitalWrite(_ncs, HIGH);        // NCS ocioso em nível alto (transação inativa)
  if (_bus == nullptr) _bus = &SPI;  // padrão: barramento global (VSPI no ESP32)
  // SS = -1: o chip-select é o NCS por software; o driver SPI não gerencia SS.
  _bus->begin(_sck, _miso, _mosi, -1);
}

bool AR49Encoder::read(AR49Reading& out) {
  const uint16_t usefulBits =
      static_cast<uint16_t>(8 + _cfg.mtBits + _cfg.stBits + 2 + _cfg.crcBits);
  const uint8_t nbytes = static_cast<uint8_t>((usefulBits + 7u) / 8u);
  if (nbytes > AR49_MAX_FRAME_BYTES) {  // configuração inválida: não estoura o buffer
    out = AR49Reading{};
    return false;
  }

  // MOSI é obrigatório: o encoder só responde depois de receber OC 0xA6 no 1º byte;
  // os bytes seguintes (0x00) apenas geram clock para ler o resto do frame.
  uint8_t tx[AR49_MAX_FRAME_BYTES] = {0};
  uint8_t rx[AR49_MAX_FRAME_BYTES] = {0};
  tx[0] = ar49::AR49_OC_POSITION_READ;

  // Modo 0 (CPOL=0, CPHA=0), MSB-first, clock configurável (AN102: dados amostrados na
  // borda de subida do SCK). NCS por software delimita a transação com a temporização exigida.
  _bus->beginTransaction(SPISettings(_clockHz, MSBFIRST, SPI_MODE0));
  digitalWrite(_ncs, LOW);
  delayMicroseconds(AR49_TL_US);        // tL: NCS↓ antes do 1º clock
  _bus->transferBytes(tx, rx, nbytes);  // full-duplex: envia OC 0xA6…, recebe o frame
  delayMicroseconds(AR49_TH_US);        // tH: segura antes de subir NCS
  digitalWrite(_ncs, HIGH);
  _bus->endTransaction();
  delayMicroseconds(AR49_TNCS_US);      // tNCS: NCS alto mínimo entre transações

  out = ar49::decodeFrame(rx, nbytes, _cfg);
  return out.valid;
}

double AR49Encoder::angleDeg(const AR49Reading& r) const {
  return ar49::angleDeg(r, _cfg.stBits);
}

double AR49Encoder::totalTurns(const AR49Reading& r) const {
  return ar49::totalTurns(r, _cfg.stBits);
}

void AR49Encoder::setResolution(uint8_t mtBits, uint8_t stBits) {
  _cfg.mtBits = mtBits;
  _cfg.stBits = stBits;
}

void AR49Encoder::setCrcBits(uint8_t crcBits) { _cfg.crcBits = crcBits; }

void AR49Encoder::setStrictCrc(bool strict) { _cfg.strictCrc = strict; }

uint32_t AR49Encoder::getBits(const uint8_t* buf, uint16_t startBit, uint8_t len) {
  return ar49::getBits(buf, startBit, len);
}
