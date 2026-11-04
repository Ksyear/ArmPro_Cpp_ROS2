// 버스 서보 패킷 인코딩·해석 (순수 함수). 패킷: 0x55 0x55, ID, 길이, 명령, 인자…, 체크섬.
//   패킷: 0x55 0x55 ID LEN CMD P1…Pn CHK,  LEN = 3 + n,  CHK = 255 − ((ID + LEN + CMD + ΣP) mod 256)
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace armpi_hardware {
namespace servo_proto {

constexpr uint8_t kHeader = 0x55;
constexpr uint8_t kMoveTimeWrite = 1;
constexpr uint8_t kMoveStop = 12;
constexpr uint8_t kIdRead = 14;
constexpr uint8_t kTempRead = 26;
constexpr uint8_t kVinRead = 27;
constexpr uint8_t kPosRead = 28;
constexpr uint8_t kLoadOrUnloadWrite = 31;

constexpr int kMaxPosition = 1000;
constexpr int kMaxDurationMs = 30000;

uint8_t checksum(uint8_t id, uint8_t len, uint8_t cmd, const std::vector<uint8_t>& params);
std::vector<uint8_t> encode(uint8_t id, uint8_t cmd, const std::vector<uint8_t>& params = {});

// 위치 0..1000, 시간 0..30000 ms 로 자른 뒤 little-endian
std::vector<uint8_t> encode_move_time_write(uint8_t id, int position, int duration_ms);
std::vector<uint8_t> encode_move_stop(uint8_t id);
// load=true: 토크 켜기(1), false: 토크 끄기(0). 제어 노드는 토크 끄기를 쓰지 않는다.
std::vector<uint8_t> encode_load_or_unload(uint8_t id, bool load);
// 읽기 요청(매개변수 없음): 0x55 0x55 ID 3 CMD CHK
std::vector<uint8_t> encode_read(uint8_t id, uint8_t cmd);

struct Response {
  uint8_t id{0};
  uint8_t cmd{0};
  std::vector<uint8_t> params;
};

enum class ParseError { kOk, kTooShort, kBadHeader, kBadLength, kBadChecksum, kUnexpectedId, kUnexpectedCmd };
const char* to_string(ParseError e);

// 헤더부터 체크섬까지 완전한 패킷 1개를 해석(남는 바이트가 있으면 kBadLength).
ParseError parse_packet(const std::vector<uint8_t>& bytes, Response* out);
// 응답 해석 + ID·명령 확인
ParseError parse_response(const std::vector<uint8_t>& bytes, uint8_t expected_id, uint8_t expected_cmd,
                          Response* out);

// POS_READ: 2 B little-endian. 서보 문서대로 signed int16 으로 읽는다
// (0..1000 범위에서는 부호 없는 해석과 같다).
std::optional<int> decode_position(const Response& r);
// VIN_READ: 2 B little-endian mV
std::optional<int> decode_vin_mv(const Response& r);
// TEMP_READ: 1 B (°C)
std::optional<int> decode_temp_c(const Response& r);

}  // namespace servo_proto
}  // namespace armpi_hardware
