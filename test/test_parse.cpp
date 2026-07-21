// test_parse.cpp — teste de HOST (PC, g++), sem dependência de Arduino.
// Inclui o núcleo PURO real (AR49Frame.h) e verifica o código do driver de verdade.
//
// Compilar/rodar:
//   g++ -O2 -std=c++17 -Wall -Wextra -o test_parse test/test_parse.cpp && ./test_parse
//
// Os valores esperados de CRC vêm de uma implementação de referência INDEPENDENTE
// (scratchpad/ref_crc.py), ancorada no caso all-zeros=0x3F (derivável à mão). Isso
// evita teste tautológico (não recomputamos o CRC do mesmo jeito que o driver).
#include "../lib/AR49/AR49Frame.h"
#include <cstdio>
#include <cstdint>

static int g_fail = 0;

static void check(bool cond, const char* what) {
  std::printf("[%s] %s\n", cond ? "PASS" : "FAIL", what);
  if (!cond) ++g_fail;
}

template <typename A, typename B>
static void check_eq(A got, B exp, const char* what) {
  bool ok = (got == static_cast<A>(exp));
  std::printf("[%s] %s (got=%lld exp=%lld)\n", ok ? "PASS" : "FAIL", what,
              static_cast<long long>(got), static_cast<long long>(exp));
  if (!ok) ++g_fail;
}

// packField: escreve `value` em `len` bits a partir de `startBit`, MSB-first.
// Scaffolding do teste para montar frames sintéticos (independente do driver).
static void packField(uint8_t* buf, uint16_t startBit, uint8_t len, uint32_t value) {
  for (uint8_t i = 0; i < len; ++i) {
    const uint16_t b = static_cast<uint16_t>(startBit + i);
    const uint8_t bitInByte = static_cast<uint8_t>(7 - (b & 7));
    const uint32_t bit = (value >> (len - 1 - i)) & 1u;  // MSB-first
    if (bit) buf[b >> 3] |= static_cast<uint8_t>(1u << bitInByte);
    else     buf[b >> 3] &= static_cast<uint8_t>(~(1u << bitInByte));
  }
}

// Layout do frame M49M "Basic" (padrões 24/25/6), bits contíguos MSB-first:
//   OC(8) | MT(24) | ST(25) | nErr(1) | nWar(1) | CRC(6) | enchimento(7)
static constexpr uint16_t OC_START   = 0;
static constexpr uint16_t MT_START   = 8;
static constexpr uint16_t ST_START   = 32;   // 8 + 24
static constexpr uint16_t NERR_START = 57;   // 8 + 24 + 25
static constexpr uint16_t NWAR_START = 58;
static constexpr uint16_t CRC_START  = 59;
static constexpr uint16_t CRC_COVER_START = MT_START;  // CRC cobre MT+ST+nErr+nWar
static constexpr uint16_t CRC_COVER_BITS  = 24 + 25 + 1 + 1;  // 51 bits

// Monta um frame de 9 bytes com os campos dados (CRC preenchido separadamente).
static void buildFrame(uint8_t buf[9], uint8_t oc, uint32_t mt, uint32_t st,
                       uint8_t nerr, uint8_t nwar) {
  for (int i = 0; i < 9; ++i) buf[i] = 0;
  packField(buf, OC_START, 8, oc);
  packField(buf, MT_START, 24, mt);
  packField(buf, ST_START, 25, st);
  packField(buf, NERR_START, 1, nerr);
  packField(buf, NWAR_START, 1, nwar);
}

// ---------------------------------------------------------------------------
// Fatia 2: crc6 (poly X^6+X+1, init 0, saída invertida) — valores da ref independente
// ---------------------------------------------------------------------------
static void test_crc6() {
  uint8_t buf[9];
  // Âncora derivável à mão: campos todos zero -> remainder 0 -> invertido -> 0x3F
  buildFrame(buf, 0x00, 0, 0, 0, 0);
  check_eq(ar49::crc6(buf, CRC_COVER_START, CRC_COVER_BITS), 0x3Fu, "crc6 all-zeros = 0x3F");
  // MT=7, ST=2^25/4, saudável -> 0x22 (ref_crc.py). OC não entra no CRC:
  buildFrame(buf, 0xA6, 7, 1u << 23, 1, 1);
  check_eq(ar49::crc6(buf, CRC_COVER_START, CRC_COVER_BITS), 0x22u, "crc6 MT=7 ST=2^25/4 = 0x22");
  // Todos os bits de dado em 1 -> 0x15 (ref_crc.py)
  buildFrame(buf, 0xA6, 0xFFFFFFu, 0x1FFFFFFu, 1, 1);
  check_eq(ar49::crc6(buf, CRC_COVER_START, CRC_COVER_BITS), 0x15u, "crc6 all-ones = 0x15");
  // Padrão arbitrário -> 0x06 (ref_crc.py)
  buildFrame(buf, 0xA6, 0xABCDEFu, 0x1555555u, 1, 0);
  check_eq(ar49::crc6(buf, CRC_COVER_START, CRC_COVER_BITS), 0x06u, "crc6 padrão ABCDEF = 0x06");
}

// ---------------------------------------------------------------------------
// Fatia 1: getBits MSB-first
// ---------------------------------------------------------------------------
static void test_getBits() {
  // Buffer conhecido: byte0=0xA6 (1010 0110), byte1=0x12, byte2=0x34
  const uint8_t buf[] = {0xA6, 0x12, 0x34};
  check_eq(ar49::getBits(buf, 0, 8), 0xA6u, "getBits eco OC (bits 0..7)");
  check_eq(ar49::getBits(buf, 8, 8), 0x12u, "getBits byte1 (bits 8..15)");
  check_eq(ar49::getBits(buf, 8, 4), 0x1u, "getBits nibble alto byte1");
  check_eq(ar49::getBits(buf, 12, 4), 0x2u, "getBits nibble baixo byte1");
  check_eq(ar49::getBits(buf, 8, 16), 0x1234u, "getBits 16 bits byte1..2");
  // Faixa cruzando byte: bits 4..11 = 0110 (baixo de 0xA6) + 0001 (alto de 0x12) = 0x61
  check_eq(ar49::getBits(buf, 4, 8), 0x61u, "getBits cruzando fronteira de byte");
  check_eq(ar49::getBits(buf, 0, 1), 0x1u, "getBits 1 bit (MSB de 0xA6)");
}

// ---------------------------------------------------------------------------
// Fatia 3: decodeFrame — round-trip de todos os campos + crcOk
// ---------------------------------------------------------------------------
static void test_decode() {
  uint8_t buf[9];
  AR49Config cfg;  // padrões: 24/25/6, strict=false

  // Frame saudável: MT=7, ST=2^25/4, nErr=1, nWar=1, OC=0xA6, CRC=0x22 (ref independente)
  buildFrame(buf, 0xA6, 7, 1u << 23, 1, 1);
  packField(buf, CRC_START, 6, 0x22);
  AR49Reading r = ar49::decodeFrame(buf, sizeof(buf), cfg);
  check_eq(r.multiTurn, 7u, "decode multiTurn");
  check_eq(r.singleTurn, (1u << 23), "decode singleTurn");
  check(r.nErr == true, "decode nErr saudável (bit=1 -> true)");
  check(r.nWar == true, "decode nWar saudável (bit=1 -> true)");
  check_eq(r.ocEcho, 0xA6u, "decode ocEcho");
  check(r.ocEchoOk, "decode ocEchoOk");
  check_eq(r.crcRecv, 0x22u, "decode crcRecv");
  check_eq(r.crcCalc, 0x22u, "decode crcCalc");
  check(r.crcOk, "decode crcOk");
  check(r.valid, "decode valid");

  // Frame com falha reportada pelo encoder: nErr=0 (ativo-baixo) -> problema
  buildFrame(buf, 0xA6, 1, 1, 0, 1);
  packField(buf, CRC_START, 6, ar49::crc6(buf, CRC_COVER_START, CRC_COVER_BITS));
  r = ar49::decodeFrame(buf, sizeof(buf), cfg);
  check(r.nErr == false, "decode nErr=0 -> false (problema)");
  check(r.nWar == true, "decode nWar=1 -> true (ok)");
  check(r.crcOk, "decode crcOk com CRC recalculado");
}

static void check_close(double got, double exp, const char* what) {
  const double d = got - exp;
  const bool ok = (d < 1e-9) && (d > -1e-9);
  std::printf("[%s] %s (got=%.6f exp=%.6f)\n", ok ? "PASS" : "FAIL", what, got, exp);
  if (!ok) ++g_fail;
}

// ---------------------------------------------------------------------------
// Fatia 4: angleDeg=90.0000° e totalTurns=7.25 para MT=7, ST=2^25/4
// ---------------------------------------------------------------------------
static void test_angle_turns() {
  uint8_t buf[9];
  AR49Config cfg;
  buildFrame(buf, 0xA6, 7, 1u << 23, 1, 1);  // ST=2^25/4
  packField(buf, CRC_START, 6, 0x22);
  AR49Reading r = ar49::decodeFrame(buf, sizeof(buf), cfg);
  check_close(ar49::angleDeg(r, cfg.stBits), 90.0, "angleDeg = 90.0000");
  check_close(ar49::totalTurns(r, cfg.stBits), 7.25, "totalTurns = 7.25");
}

static void flipBit(uint8_t* buf, uint16_t bit) {
  buf[bit >> 3] ^= static_cast<uint8_t>(1u << (7 - (bit & 7)));
}

// ---------------------------------------------------------------------------
// Fatia 5: corromper 1 bit de dado invalida o CRC
// ---------------------------------------------------------------------------
static void test_crc_corruption() {
  uint8_t buf[9];
  AR49Config cfg;
  buildFrame(buf, 0xA6, 7, 1u << 23, 1, 1);
  packField(buf, CRC_START, 6, 0x22);  // CRC correto (ref independente)
  check(ar49::decodeFrame(buf, sizeof(buf), cfg).crcOk, "crcOk antes da corrupção");

  flipBit(buf, ST_START + 5);  // vira 1 bit dentro da cobertura do CRC (ST)
  AR49Reading r = ar49::decodeFrame(buf, sizeof(buf), cfg);
  check(r.crcOk == false, "1 bit corrompido -> crcOk = false");
  check(r.valid == true, "não-estrito: valid continua true apesar do CRC ruim");

  cfg.strictCrc = true;
  check(ar49::decodeFrame(buf, sizeof(buf), cfg).valid == false,
        "estrito: CRC ruim -> valid = false");
}

// ---------------------------------------------------------------------------
// Fatia 6: ocEcho e a regra de valid (independente do CRC)
// ---------------------------------------------------------------------------
static void test_flags_valid() {
  uint8_t buf[9];
  AR49Config cfg;

  // OC errado (falha de fiação/level-shifter/modo SPI) -> valid=false mesmo não-estrito
  buildFrame(buf, 0x00, 1, 1, 1, 1);
  packField(buf, CRC_START, 6, ar49::crc6(buf, CRC_COVER_START, CRC_COVER_BITS));
  AR49Reading r = ar49::decodeFrame(buf, sizeof(buf), cfg);
  check(r.ocEchoOk == false, "ocEcho != 0xA6 -> ocEchoOk false");
  check(r.valid == false, "ocEcho ruim -> valid false (mesmo não-estrito)");

  // Ambos os sinais em falha: nErr=0, nWar=0
  buildFrame(buf, 0xA6, 0, 0, 0, 0);
  packField(buf, CRC_START, 6, ar49::crc6(buf, CRC_COVER_START, CRC_COVER_BITS));
  r = ar49::decodeFrame(buf, sizeof(buf), cfg);
  check(r.nErr == false && r.nWar == false, "nErr=0 nWar=0 -> ambos false");
  check(r.valid == true, "OC ok + CRC ok -> valid true (não-estrito)");

  // Frame incompleto (nbytes insuficiente) -> valid false, não lê fora
  check(ar49::decodeFrame(buf, 3, cfg).valid == false, "frame curto -> valid false");
}

int main() {
  test_getBits();
  test_crc6();
  test_decode();
  test_angle_turns();
  test_crc_corruption();
  test_flags_valid();
  std::printf("\n%s (%d falha(s))\n", g_fail ? "RESULTADO: FAIL" : "RESULTADO: PASS", g_fail);
  return g_fail ? 1 : 0;
}
