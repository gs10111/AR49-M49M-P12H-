// Encoder.h — driver ESP32/Arduino do encoder absoluto Broadcom AR49-M49M-P12H (SPI 4 fios).
//
// ┌─────────────────────────────────────────────────────────────────────────────┐
// │ ⚠️  AVISO DE HARDWARE — LEVEL SHIFTING OBRIGATÓRIO NO MISO                     │
// │                                                                               │
// │ O encoder opera em V_DD = 5 V e sua saída (SPI-DO/MISO) tem V_OH ≥ 4,4 V.     │
// │ Ligar isso DIRETO num GPIO do ESP32 (3,3 V, NÃO 5 V-tolerante) danifica o pino.│
// │ => Use um level shifter 5 V→3,3 V na linha MISO.                              │
// │ CLK/MOSI/NCS podem ir diretos: o encoder exige V_IH = 2,8 V e 3,3 V > 2,8 V.   │
// │ (Na placa DiEletrons os buffers 74HC245 em 3,3 V TAMBÉM não são 5 V-tolerantes.)│
// └─────────────────────────────────────────────────────────────────────────────┘
//
// Arquitetura (SOLID): a decodificação do frame (getBits/crc6/decodeFrame/ângulo/voltas)
// mora em AR49Frame.h — PURA, sem Arduino, testada no host. Esta classe é só o ADAPTADOR
// de I/O: monta a transação SPI, controla NCS por software (temporização tL/tH/tNCS) e
// delega a interpretação ao núcleo. Assim o hardware e o protocolo evoluem separados.
//
// Referências: Broadcom datasheet DS101 + application note AN102.
#pragma once
#include "AR49Frame.h"

class SPIClass;  // forward-declare: <SPI.h> só é necessário no .cpp

class AR49Encoder {
public:
  // pinos: SCK/MISO/MOSI/NCS. clockHz padrão 1 MHz (≤10 MHz permitido pelo encoder).
  // bus: passe um SPIClass* próprio; nullptr => usa o SPI global (VSPI no ESP32).
  AR49Encoder(int8_t sck, int8_t miso, int8_t mosi, int8_t ncs,
              uint32_t clockHz = 1000000, SPIClass* bus = nullptr);

  void begin();                    // configura NCS (idle alto) e inicia o barramento SPI
  bool read(AR49Reading& out);     // executa a leitura OC 0xA6; devolve out.valid

  double angleDeg(const AR49Reading& r) const;   // 0..360° dentro da volta
  double totalTurns(const AR49Reading& r) const; // MT + fração da volta

  void setResolution(uint8_t mtBits, uint8_t stBits);  // padrão 24, 25 (Table 7)
  void setCrcBits(uint8_t crcBits);                     // 6 (padrão) ou 16
  void setStrictCrc(bool strict);                       // true => valid exige crcOk

  // Extração MSB-first genérica, exposta para testes/uso avançado (delega ao núcleo puro).
  static uint32_t getBits(const uint8_t* buf, uint16_t startBit, uint8_t len);

  // Nota: OC 0x9C lê status detalhado do encoder — não implementado aqui de propósito;
  // o núcleo do driver usa APENAS OC 0xA6 (leitura de posição).

private:
  int8_t    _sck, _miso, _mosi, _ncs;
  uint32_t  _clockHz;
  SPIClass* _bus;
  AR49Config _cfg;  // resolução/CRC/rigor (padrões M49M "Basic")
};
