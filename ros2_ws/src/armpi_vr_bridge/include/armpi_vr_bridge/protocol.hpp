// Protocol v3 형식은 armpi_protocol 라이브러리에 있다(수신기와 C++ 클라이언트가 같이 씀).
// 수신기 코드는 아래 이름들을 armpi_vr_bridge 이름공간에서 그대로 쓴다(필요한 이름만 명시적으로 가져옴).
#pragma once

#include "armpi_protocol/protocol.hpp"

namespace armpi_vr_bridge {

using armpi_protocol::Ack;
using armpi_protocol::canonical_uuid;
using armpi_protocol::Channel;
using armpi_protocol::ChannelState;
using armpi_protocol::encode_ack;
using armpi_protocol::kMaxPacketBytes;
using armpi_protocol::kMaxSequence;
using armpi_protocol::kProtocolVersion;
using armpi_protocol::Packet;
using armpi_protocol::PacketError;
using armpi_protocol::PacketType;
using armpi_protocol::parse_packet;
using armpi_protocol::ParseResult;
using armpi_protocol::Reason;
using armpi_protocol::to_string;

}  // namespace armpi_vr_bridge
