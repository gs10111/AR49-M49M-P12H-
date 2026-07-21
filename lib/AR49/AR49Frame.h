// AR49Frame.h — núcleo PURO de decodificação do encoder Broadcom AR49-M49M-P12H.
//
// Este cabeçalho NÃO depende de Arduino/SPI: só de <cstdint>/<cstddef>. É o domínio
// (SOLID: responsabilidade única = interpretar o frame; sem I/O). O adaptador de
// hardware (AR49Encoder, em Encoder.h/.cpp) faz a transação SPI e delega a decodificação
// para cá. O teste de host (test/test_parse.cpp) inclui SÓ este arquivo e exercita o
// código REAL — não uma cópia.
//
// Referências: Broadcom datasheet DS101 + application note AN102.
#pragma once
#include <cstdint>
#include <cstddef>

namespace ar49 {

// --- Constantes travadas pelo datasheet/AN (sem números mágicos soltos) ---
constexpr uint8_t AR49_OC_POSITION_READ = 0xA6;  // operation command de leitura de posição
constexpr uint8_t AR49_DEFAULT_MT_BITS  = 24;    // multivolta (Table 7, padrão M49M)
constexpr uint8_t AR49_DEFAULT_ST_BITS  = 25;    // volta única (Table 7, padrão M49M)
constexpr uint8_t AR49_DEFAULT_CRC_BITS = 6;     // CRC "Basic" (há opção de 16)
constexpr uint8_t AR49_CRC6_POLY_TAPS   = 0x03;  // G(X)=X^6+X^1+1 => taps X^1+X^0
constexpr uint8_t AR49_OC_ECHO_BITS     = 8;     // eco do comando nos 8 primeiros clocks

// getBits: extrai `len` bits (<=32) a partir de `startBit`, MSB-first — o bit 0 é o
// MSB do byte 0, como o encoder transmite (AN102: "MSB first"). Extração genérica para
// não depender de deslocamentos hard-coded frágeis por campo.
inline uint32_t getBits(const uint8_t* buf, uint16_t startBit, uint8_t len) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < len; ++i) {
    const uint16_t b = static_cast<uint16_t>(startBit + i);
    const uint8_t bitInByte = static_cast<uint8_t>(7 - (b & 7));  // MSB-first dentro do byte
    const uint32_t bit = (buf[b >> 3] >> bitInByte) & 1u;
    v = (v << 1) | bit;
  }
  return v;
}

// crc6: CRC de 6 bits sobre `nbits` bits a partir de `startBit`, MSB-first.
// DS101: G(X)=X^6+X^1+1 (taps 0x03), valor inicial 0, "Invert of X6+X1+X0" => saída
// invertida nos 6 bits. Registrador direto (sem enchimento de zeros), bit a bit.
// Cobre MT+ST+nErr+nWar (51 bits nos padrões). Retorna em uint16_t para acomodar o
// modo de 16 bits no futuro (ver setCrcBits); aqui só o de 6 bits é implementado.
inline uint16_t crc6(const uint8_t* buf, uint16_t startBit, uint16_t nbits) {
  uint8_t crc = 0;  // init 0, registrador de 6 bits nos bits baixos
  for (uint16_t i = 0; i < nbits; ++i) {
    const uint8_t bit = static_cast<uint8_t>(getBits(buf, static_cast<uint16_t>(startBit + i), 1));
    const uint8_t msb = (crc >> 5) & 1u;                 // bit que sai (termo X^6)
    crc = static_cast<uint8_t>((crc << 1) & 0x3F);
    if (msb ^ bit) crc ^= AR49_CRC6_POLY_TAPS;           // realimenta X^1+X^0
  }
  return static_cast<uint16_t>((~crc) & 0x3F);           // saída invertida (6 bits)
}

} // namespace ar49

// --- Tipos públicos (nomes fixados pelo spec; escopo global para o driver reusar) ---

// Resultado de uma leitura decodificada. nErr/nWar já normalizados: true = SAUDÁVEL
// (o encoder emite ativo-baixo; aqui guardamos o sentido lógico "ok").
struct AR49Reading {
  uint32_t multiTurn;  // MT: contagem de voltas (0..2^mtBits-1)
  uint32_t singleTurn; // ST: posição dentro da volta (0..2^stBits-1)
  bool     nErr;       // true = sem erro (saudável)
  bool     nWar;       // true = sem aviso (saudável)
  uint8_t  ocEcho;     // eco do comando nos 8 primeiros clocks (esperado 0xA6)
  bool     ocEchoOk;   // ocEcho == 0xA6 (verificação primária de fiação/modo SPI)
  uint16_t crcRecv;    // CRC recebido no frame
  uint16_t crcCalc;    // CRC recalculado sobre MT+ST+nErr+nWar
  bool     crcOk;      // crcRecv == crcCalc
  bool     valid;      // ocEchoOk && (crcOk || !strictCrc)
};

// Configuração de decodificação (resolução/CRC/rigor). Padrões = M49M "Basic".
struct AR49Config {
  uint8_t mtBits  = ar49::AR49_DEFAULT_MT_BITS;
  uint8_t stBits  = ar49::AR49_DEFAULT_ST_BITS;
  uint8_t crcBits = ar49::AR49_DEFAULT_CRC_BITS;
  bool    strictCrc = false;  // se true, valid exige crcOk
};

namespace ar49 {

// decodeFrame: interpreta o frame bruto (MSB-first) em campos, por faixas de bits
// CONTÍGUAS a partir do bit 8, calcula o CRC e resolve `valid`. Genérico nas larguras
// (nada de deslocamentos hard-coded). `nbytes` é validado contra o tamanho mínimo do
// frame — frame curto => valid=false (robustez; não lê fora do buffer útil).
inline AR49Reading decodeFrame(const uint8_t* buf, size_t nbytes, const AR49Config& cfg) {
  AR49Reading r{};
  const uint16_t mtStart = AR49_OC_ECHO_BITS;                        // 8
  const uint16_t stStart = static_cast<uint16_t>(mtStart + cfg.mtBits);
  const uint16_t nErrPos = static_cast<uint16_t>(stStart + cfg.stBits);
  const uint16_t nWarPos = static_cast<uint16_t>(nErrPos + 1);
  const uint16_t crcPos  = static_cast<uint16_t>(nWarPos + 1);
  const uint16_t usefulBits = static_cast<uint16_t>(crcPos + cfg.crcBits);
  const size_t   neededBytes = (usefulBits + 7u) / 8u;

  r.valid = false;
  if (nbytes < neededBytes) return r;  // frame incompleto: não decodifica

  r.ocEcho   = static_cast<uint8_t>(getBits(buf, 0, AR49_OC_ECHO_BITS));
  r.ocEchoOk = (r.ocEcho == AR49_OC_POSITION_READ);
  r.multiTurn  = getBits(buf, mtStart, cfg.mtBits);
  r.singleTurn = getBits(buf, stStart, cfg.stBits);
  r.nErr = (getBits(buf, nErrPos, 1) != 0);  // ativo-baixo: 1=ok => true=saudável
  r.nWar = (getBits(buf, nWarPos, 1) != 0);
  r.crcRecv = static_cast<uint16_t>(getBits(buf, crcPos, cfg.crcBits));

  // CRC cobre MT+ST+nErr+nWar. Só o CRC de 6 bits é implementado; para outras larguras
  // não afirmamos crcOk (evita falso-positivo silencioso) — mantenha strictCrc=false.
  const uint16_t coverBits = static_cast<uint16_t>(cfg.mtBits + cfg.stBits + 2);
  if (cfg.crcBits == AR49_DEFAULT_CRC_BITS) {
    r.crcCalc = crc6(buf, mtStart, coverBits);
    r.crcOk   = (r.crcRecv == r.crcCalc);
  } else {
    r.crcCalc = 0;
    r.crcOk   = false;  // largura de CRC não suportada => não verificável
  }

  r.valid = r.ocEchoOk && (r.crcOk || !cfg.strictCrc);
  return r;
}

// angleDeg: posição dentro da volta em graus. 0..360° = 360·ST / 2^stBits.
inline double angleDeg(const AR49Reading& r, uint8_t stBits) {
  const double scale = static_cast<double>(uint64_t{1} << stBits);  // 2^stBits
  return 360.0 * static_cast<double>(r.singleTurn) / scale;
}

// totalTurns: voltas acumuladas com fração da volta atual = MT + ST / 2^stBits.
inline double totalTurns(const AR49Reading& r, uint8_t stBits) {
  const double scale = static_cast<double>(uint64_t{1} << stBits);
  return static_cast<double>(r.multiTurn) + static_cast<double>(r.singleTurn) / scale;
}

} // namespace ar49
