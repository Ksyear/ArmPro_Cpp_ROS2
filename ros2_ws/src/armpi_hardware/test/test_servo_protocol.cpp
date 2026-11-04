// 버스 서보 패킷 인코딩·체크섬·응답 해석 시험. 기대 바이트열은 아래 ref_packet()(패킷 규격을 시험 안에서 따로 구현)으로 만든다.
#include <gtest/gtest.h>

#include <vector>

#include "armpi_hardware/servo_protocol.hpp"

using namespace armpi_hardware;
namespace sp = armpi_hardware::servo_proto;

namespace {
// 패킷 규격: 0x55 0x55, ID, 길이(= 인자 수 + 3), 명령, 인자…, 체크섬 = 255 − (ID + 길이 + 명령 + 인자 합) mod 256
std::vector<uint8_t> ref_packet(int id, int cmd, std::vector<int> params) {
  std::vector<uint8_t> b{0x55, 0x55, static_cast<uint8_t>(id), static_cast<uint8_t>(params.size() + 3),
                         static_cast<uint8_t>(cmd)};
  int sum = id + static_cast<int>(params.size()) + 3 + cmd;
  for (int v : params) { b.push_back(static_cast<uint8_t>(v)); sum += v & 0xFF; }
  b.push_back(static_cast<uint8_t>(255 - (sum % 256)));
  return b;
}
int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

TEST(ServoProtocol, MoveTimeWriteFollowsPacketLayout) {
  int n = 0;
  for (int id : {0, 1, 2, 3, 4, 5, 6, 127, 253}) {
    for (int pos : {-5, 0, 1, 255, 256, 500, 825, 999, 1000, 1200}) {
      for (int dur : {-1, 0, 20, 1000, 1500, 30000, 40000}) {
        const int pc = clampi(pos, 0, 1000), dc = clampi(dur, 0, 30000);
        const auto want = ref_packet(id, sp::kMoveTimeWrite, {pc & 0xFF, pc >> 8, dc & 0xFF, dc >> 8});
        EXPECT_EQ(sp::encode_move_time_write(static_cast<uint8_t>(id), pos, dur), want)
            << "id=" << id << " pos=" << pos << " dur=" << dur;
        ++n;
      }
    }
  }
  EXPECT_EQ(n, 9 * 10 * 7);
}

TEST(ServoProtocol, HandComputedChecksum) {
  // 0x55 0x55 01 07 01 F4 01 E8 03 → 1+7+1+244+1+232+3 = 489, 489 mod 256 = 233, 255−233 = 22 = 0x16
  const auto b = sp::encode_move_time_write(1, 500, 1000);
  const std::vector<uint8_t> want{0x55, 0x55, 0x01, 0x07, 0x01, 0xF4, 0x01, 0xE8, 0x03, 0x16};
  EXPECT_EQ(b, want);
}

TEST(ServoProtocol, ReadStopLoadPackets) {
  for (int id : {1, 2, 3, 4, 5, 6}) {
    for (int cmd : {sp::kPosRead, sp::kVinRead, sp::kTempRead, sp::kIdRead}) {
      EXPECT_EQ(sp::encode_read(static_cast<uint8_t>(id), static_cast<uint8_t>(cmd)), ref_packet(id, cmd, {}));
    }
    EXPECT_EQ(sp::encode_move_stop(static_cast<uint8_t>(id)), ref_packet(id, sp::kMoveStop, {}));
    EXPECT_EQ(sp::encode_load_or_unload(static_cast<uint8_t>(id), true), ref_packet(id, sp::kLoadOrUnloadWrite, {1}));
    EXPECT_EQ(sp::encode_load_or_unload(static_cast<uint8_t>(id), false), ref_packet(id, sp::kLoadOrUnloadWrite, {0}));
  }
}

TEST(ServoProtocol, CommandNumbers) {
  EXPECT_EQ(sp::kMoveTimeWrite, 1);
  EXPECT_EQ(sp::kMoveStop, 12);
  EXPECT_EQ(sp::kIdRead, 14);
  EXPECT_EQ(sp::kTempRead, 26);
  EXPECT_EQ(sp::kVinRead, 27);
  EXPECT_EQ(sp::kPosRead, 28);
  EXPECT_EQ(sp::kLoadOrUnloadWrite, 31);
}

namespace {
std::vector<uint8_t> response(uint8_t id, uint8_t cmd, std::vector<uint8_t> params) {
  return sp::encode(id, cmd, params);  // 응답도 같은 틀
}
}  // namespace

TEST(ServoProtocol, ParsePositionResponse) {
  auto pkt = response(4, sp::kPosRead, {0x39, 0x03});  // 825
  sp::Response r;
  ASSERT_EQ(sp::parse_response(pkt, 4, sp::kPosRead, &r), sp::ParseError::kOk);
  EXPECT_EQ(r.id, 4);
  EXPECT_EQ(sp::decode_position(r).value(), 825);
  // 서보 문서: 음수 위치는 signed int16
  auto neg = response(4, sp::kPosRead, {0xFB, 0xFF});
  ASSERT_EQ(sp::parse_response(neg, 4, sp::kPosRead, &r), sp::ParseError::kOk);
  EXPECT_EQ(sp::decode_position(r).value(), -5);
}

TEST(ServoProtocol, ParseVinAndTemp) {
  sp::Response r;
  ASSERT_EQ(sp::parse_response(response(2, sp::kVinRead, {0xE8, 0x1C}), 2, sp::kVinRead, &r), sp::ParseError::kOk);
  EXPECT_EQ(sp::decode_vin_mv(r).value(), 7400);
  ASSERT_EQ(sp::parse_response(response(2, sp::kTempRead, {41}), 2, sp::kTempRead, &r), sp::ParseError::kOk);
  EXPECT_EQ(sp::decode_temp_c(r).value(), 41);
  EXPECT_FALSE(sp::decode_position(r).has_value());  // 명령이 다르면 해석 안 함
}

TEST(ServoProtocol, ParseErrors) {
  sp::Response r;
  auto good = response(1, sp::kPosRead, {0xF4, 0x01});
  auto bad_chk = good;
  bad_chk.back() ^= 0x01;
  EXPECT_EQ(sp::parse_response(bad_chk, 1, sp::kPosRead, &r), sp::ParseError::kBadChecksum);
  auto bad_hdr = good;
  bad_hdr[0] = 0xAA;
  EXPECT_EQ(sp::parse_response(bad_hdr, 1, sp::kPosRead, &r), sp::ParseError::kBadHeader);
  auto truncated = good;
  truncated.pop_back();
  EXPECT_EQ(sp::parse_response(truncated, 1, sp::kPosRead, &r), sp::ParseError::kTooShort);
  auto extra = good;
  extra.push_back(0x00);
  EXPECT_EQ(sp::parse_response(extra, 1, sp::kPosRead, &r), sp::ParseError::kBadLength);
  EXPECT_EQ(sp::parse_response(good, 2, sp::kPosRead, &r), sp::ParseError::kUnexpectedId);
  EXPECT_EQ(sp::parse_response(good, 1, sp::kVinRead, &r), sp::ParseError::kUnexpectedCmd);
  EXPECT_EQ(sp::parse_response({0x55, 0x55}, 1, sp::kPosRead, &r), sp::ParseError::kTooShort);
  std::vector<uint8_t> len2{0x55, 0x55, 0x01, 0x02, 0x1C, 0x00};
  EXPECT_EQ(sp::parse_response(len2, 1, sp::kPosRead, &r), sp::ParseError::kBadLength);
}

TEST(ServoProtocol, ClampsPositionAndDuration) {
  const auto lo = sp::encode_move_time_write(3, -5, -1);
  EXPECT_EQ(lo[5], 0);
  EXPECT_EQ(lo[6], 0);
  EXPECT_EQ(lo[7], 0);
  EXPECT_EQ(lo[8], 0);
  const auto hi = sp::encode_move_time_write(3, 5000, 99999);
  EXPECT_EQ(hi[5] | (hi[6] << 8), 1000);
  EXPECT_EQ(hi[7] | (hi[8] << 8), 30000);
}
