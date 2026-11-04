#include "armpi_hardware/servo_protocol.hpp"

#include <algorithm>

namespace armpi_hardware {
namespace servo_proto {

uint8_t checksum(uint8_t id, uint8_t len, uint8_t cmd, const std::vector<uint8_t>& params) {
  unsigned sum = static_cast<unsigned>(id) + len + cmd;
  for (uint8_t p : params) sum += p;
  return static_cast<uint8_t>(255u - (sum % 256u));
}

std::vector<uint8_t> encode(uint8_t id, uint8_t cmd, const std::vector<uint8_t>& params) {
  const uint8_t len = static_cast<uint8_t>(3 + params.size());
  std::vector<uint8_t> out;
  out.reserve(6 + params.size());
  out.push_back(kHeader);
  out.push_back(kHeader);
  out.push_back(id);
  out.push_back(len);
  out.push_back(cmd);
  out.insert(out.end(), params.begin(), params.end());
  out.push_back(checksum(id, len, cmd, params));
  return out;
}

std::vector<uint8_t> encode_move_time_write(uint8_t id, int position, int duration_ms) {
  position = std::max(0, std::min(kMaxPosition, position));
  duration_ms = std::max(0, std::min(kMaxDurationMs, duration_ms));
  return encode(id, kMoveTimeWrite,
                {static_cast<uint8_t>(position & 0xFF), static_cast<uint8_t>((position >> 8) & 0xFF),
                 static_cast<uint8_t>(duration_ms & 0xFF), static_cast<uint8_t>((duration_ms >> 8) & 0xFF)});
}

std::vector<uint8_t> encode_move_stop(uint8_t id) { return encode(id, kMoveStop); }

std::vector<uint8_t> encode_load_or_unload(uint8_t id, bool load) {
  return encode(id, kLoadOrUnloadWrite, {static_cast<uint8_t>(load ? 1 : 0)});
}

std::vector<uint8_t> encode_read(uint8_t id, uint8_t cmd) { return encode(id, cmd); }

const char* to_string(ParseError e) {
  switch (e) {
    case ParseError::kOk: return "ok";
    case ParseError::kTooShort: return "too_short";
    case ParseError::kBadHeader: return "bad_header";
    case ParseError::kBadLength: return "bad_length";
    case ParseError::kBadChecksum: return "bad_checksum";
    case ParseError::kUnexpectedId: return "unexpected_id";
    case ParseError::kUnexpectedCmd: return "unexpected_cmd";
  }
  return "unknown";
}

ParseError parse_packet(const std::vector<uint8_t>& b, Response* out) {
  if (b.size() < 6) return ParseError::kTooShort;
  if (b[0] != kHeader || b[1] != kHeader) return ParseError::kBadHeader;
  const uint8_t len = b[3];
  if (len < 3) return ParseError::kBadLength;
  if (b.size() != static_cast<size_t>(len) + 3) return b.size() < static_cast<size_t>(len) + 3 ? ParseError::kTooShort
                                                                                              : ParseError::kBadLength;
  // checksum = 255 − (sum(data[2:-1]) mod 256)
  unsigned sum = 0;
  for (size_t i = 2; i + 1 < b.size(); ++i) sum += b[i];
  if (static_cast<uint8_t>(255u - (sum % 256u)) != b.back()) return ParseError::kBadChecksum;
  if (out) {
    out->id = b[2];
    out->cmd = b[4];
    out->params.assign(b.begin() + 5, b.end() - 1);
  }
  return ParseError::kOk;
}

ParseError parse_response(const std::vector<uint8_t>& bytes, uint8_t expected_id, uint8_t expected_cmd,
                          Response* out) {
  Response r;
  const ParseError e = parse_packet(bytes, &r);
  if (e != ParseError::kOk) return e;
  if (r.id != expected_id) return ParseError::kUnexpectedId;
  if (r.cmd != expected_cmd) return ParseError::kUnexpectedCmd;
  if (out) *out = std::move(r);
  return ParseError::kOk;
}

std::optional<int> decode_position(const Response& r) {
  if (r.cmd != kPosRead || r.params.size() != 2) return std::nullopt;
  const auto raw = static_cast<uint16_t>(r.params[0] | (r.params[1] << 8));
  return static_cast<int>(static_cast<int16_t>(raw));
}

std::optional<int> decode_vin_mv(const Response& r) {
  if (r.cmd != kVinRead || r.params.size() != 2) return std::nullopt;
  return static_cast<int>(r.params[0] | (r.params[1] << 8));
}

std::optional<int> decode_temp_c(const Response& r) {
  if (r.cmd != kTempRead || r.params.size() != 1) return std::nullopt;
  return static_cast<int>(r.params[0]);
}

}  // namespace servo_proto
}  // namespace armpi_hardware
