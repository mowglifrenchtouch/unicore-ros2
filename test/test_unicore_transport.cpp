// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "unicore_gnss/unicore_transport.hpp"
#include <gtest/gtest.h>

namespace unicore_gnss
{
namespace
{

uint32_t crc32_unicore_binary(const std::string& bytes)
{
  uint32_t crc = 0U;
  for (const unsigned char byte : bytes)
  {
    crc ^= static_cast<uint32_t>(byte);
    for (int bit = 0; bit < 8; ++bit)
    {
      const bool lsb = (crc & 1U) != 0U;
      crc >>= 1U;
      if (lsb)
      {
        crc ^= 0xEDB88320U;
      }
    }
  }
  return crc;
}

void append_le16(std::string& out, uint16_t value)
{
  out.push_back(static_cast<char>(value & 0xFFU));
  out.push_back(static_cast<char>((value >> 8U) & 0xFFU));
}

void append_le32(std::string& out, uint32_t value)
{
  out.push_back(static_cast<char>(value & 0xFFU));
  out.push_back(static_cast<char>((value >> 8U) & 0xFFU));
  out.push_back(static_cast<char>((value >> 16U) & 0xFFU));
  out.push_back(static_cast<char>((value >> 24U) & 0xFFU));
}

struct BinaryHeaderOverrides
{
  uint8_t cpu_idle{17U};
  uint8_t time_ref{1U};
  uint8_t time_status{2U};
  uint16_t week{2294U};
  uint32_t milliseconds{472312000U};
  uint32_t version{16U};
  uint8_t reserved{0U};
  uint8_t leap_seconds{18U};
  uint16_t delay_ms{97U};
};

std::string make_binary_frame(uint16_t message_id,
                              const std::vector<uint8_t>& payload,
                              bool valid_crc = true,
                              const BinaryHeaderOverrides& header = {})
{
  std::string frame;
  frame.push_back(static_cast<char>(0xAAU));
  frame.push_back(static_cast<char>(0x44U));
  frame.push_back(static_cast<char>(0xB5U));
  frame.push_back(static_cast<char>(header.cpu_idle));
  append_le16(frame, message_id);
  append_le16(frame, static_cast<uint16_t>(payload.size()));
  frame.push_back(static_cast<char>(header.time_ref));
  frame.push_back(static_cast<char>(header.time_status));
  append_le16(frame, header.week);
  append_le32(frame, header.milliseconds);
  append_le32(frame, header.version);
  frame.push_back(static_cast<char>(header.reserved));
  frame.push_back(static_cast<char>(header.leap_seconds));
  append_le16(frame, header.delay_ms);
  for (const uint8_t byte : payload)
  {
    frame.push_back(static_cast<char>(byte));
  }

  uint32_t crc = crc32_unicore_binary(frame);
  if (!valid_crc)
  {
    crc ^= 0xFFFFFFFFU;
  }
  append_le32(frame, crc);
  return frame;
}

std::string bytes_from_hex(std::string_view hex)
{
  if ((hex.size() % 2U) != 0U)
  {
    throw std::runtime_error("hex string must have an even length");
  }

  auto nibble = [](char ch) -> uint8_t
  {
    if (ch >= '0' && ch <= '9')
    {
      return static_cast<uint8_t>(ch - '0');
    }
    if (ch >= 'a' && ch <= 'f')
    {
      return static_cast<uint8_t>(10 + (ch - 'a'));
    }
    if (ch >= 'A' && ch <= 'F')
    {
      return static_cast<uint8_t>(10 + (ch - 'A'));
    }
    throw std::runtime_error("invalid hex digit");
  };

  std::string bytes;
  bytes.reserve(hex.size() / 2U);
  for (std::size_t index = 0U; index < hex.size(); index += 2U)
  {
    const uint8_t high = nibble(hex[index]);
    const uint8_t low = nibble(hex[index + 1U]);
    bytes.push_back(static_cast<char>((high << 4U) | low));
  }
  return bytes;
}

}  // namespace

TEST(UnicoreTransport, ExtractsValidBinaryFrame)
{
  UnicoreTransport transport({true, true, 256U});
  const std::string frame = make_binary_frame(2118U, {0x01U, 0x02U, 0x03U, 0x04U});

  transport.append(reinterpret_cast<const uint8_t*>(frame.data()), frame.size());
  const auto events = transport.drain();

  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, UnicoreTransportEventKind::kBinaryFrame);
  ASSERT_TRUE(events[0].binary_frame.has_value());
  EXPECT_EQ(events[0].binary_frame->message_id, 2118U);
  EXPECT_EQ(events[0].binary_frame->payload_length, 4U);
  EXPECT_EQ(events[0].binary_frame->header_length, kUnicoreBinaryHeaderSize);
  EXPECT_TRUE(events[0].binary_frame->crc_valid);
  ASSERT_EQ(events[0].binary_frame->payload.size(), 4U);
  EXPECT_EQ(events[0].binary_frame->payload[0], 0x01U);
  EXPECT_EQ(transport.binary_counters().frames_total, 1U);
  EXPECT_EQ(transport.binary_counters().sync_candidates, 1U);
  ASSERT_TRUE(transport.binary_counters().last_header_length.has_value());
  ASSERT_TRUE(transport.binary_counters().last_payload_length.has_value());
  EXPECT_EQ(*transport.binary_counters().last_header_length, kUnicoreBinaryHeaderSize);
  EXPECT_EQ(*transport.binary_counters().last_payload_length, 4U);
}

TEST(UnicoreTransport, RejectsBadCrcInStrictMode)
{
  UnicoreTransport transport({true, true, 256U});
  const std::string frame = make_binary_frame(2118U, {0x01U, 0x02U}, false);

  transport.append(reinterpret_cast<const uint8_t*>(frame.data()), frame.size());
  const auto events = transport.drain();

  EXPECT_TRUE(events.empty());
  EXPECT_EQ(transport.binary_counters().frames_total, 0U);
  EXPECT_EQ(transport.binary_counters().crc_errors, 1U);
  EXPECT_EQ(transport.binary_counters().resync_count, 1U);
  EXPECT_EQ(transport.binary_counters().frame_parse_errors_by_reason.at("crc_mismatch"), 1U);
}

TEST(UnicoreTransport, ResynchronizesAfterGarbageBeforeSync)
{
  UnicoreTransport transport({true, true, 256U});
  const std::string frame = make_binary_frame(1021U, {0x10U, 0x20U});
  std::string data = "xyz";
  data += frame;

  transport.append(reinterpret_cast<const uint8_t*>(data.data()), data.size());
  const auto events = transport.drain();

  ASSERT_EQ(events.size(), 1U);
  ASSERT_TRUE(events[0].binary_frame.has_value());
  EXPECT_EQ(events[0].binary_frame->message_id, 1021U);
  EXPECT_EQ(transport.binary_counters().frames_total, 1U);
  EXPECT_EQ(transport.binary_counters().resync_count, 1U);
  EXPECT_EQ(transport.binary_counters().sync_candidates, 1U);
}

TEST(UnicoreTransport, WaitsForIncompleteFrame)
{
  UnicoreTransport transport({true, true, 256U});
  const std::string frame = make_binary_frame(2125U, {0xAAU, 0xBBU, 0xCCU});
  const std::size_t split = frame.size() - 2U;

  transport.append(reinterpret_cast<const uint8_t*>(frame.data()), split);
  EXPECT_TRUE(transport.drain().empty());
  EXPECT_EQ(transport.binary_counters().frames_total, 0U);

  transport.append(reinterpret_cast<const uint8_t*>(frame.data() + split), frame.size() - split);
  const auto events = transport.drain();
  ASSERT_EQ(events.size(), 1U);
  ASSERT_TRUE(events[0].binary_frame.has_value());
  EXPECT_EQ(events[0].binary_frame->message_id, 2125U);
}

TEST(UnicoreBinaryDispatcher, RecordsUnknownMessageIds)
{
  UnicoreBinaryDispatcher dispatcher;
  UnicoreBinaryFrame frame;
  frame.message_id = 65000U;

  const auto result = dispatcher.dispatch(frame);
  const auto counters = dispatcher.counters();

  EXPECT_FALSE(result.known_message);
  EXPECT_TRUE(result.message_name.empty());
  EXPECT_EQ(counters.dispatched_frames, 1U);
  EXPECT_EQ(counters.unknown_frames, 1U);
  ASSERT_TRUE(counters.last_message_id.has_value());
  EXPECT_EQ(*counters.last_message_id, 65000U);
  ASSERT_EQ(counters.recent_unknown_message_ids.size(), 1U);
  EXPECT_EQ(counters.recent_unknown_message_ids.front(), 65000U);
}

TEST(UnicoreBinaryDispatcher, RecognizesObsvmcmpMessageId)
{
  UnicoreBinaryDispatcher dispatcher;
  UnicoreBinaryFrame frame;
  frame.message_id = 138U;

  const auto result = dispatcher.dispatch(frame);

  EXPECT_TRUE(result.known_message);
  EXPECT_EQ(result.message_name, "OBSVMCMP");
}

TEST(UnicoreTransport, KeepsAsciiPathWhenBinaryDisabled)
{
  UnicoreTransport transport({false, true, 256U});
  const std::string line = "$GPHDT,123.4,T*31\r\n";

  transport.append(reinterpret_cast<const uint8_t*>(line.data()), line.size());
  const auto events = transport.drain();

  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].kind, UnicoreTransportEventKind::kAsciiLine);
  EXPECT_EQ(events[0].ascii_line, "$GPHDT,123.4,T*31");
  EXPECT_EQ(transport.binary_counters().frames_total, 0U);
}

TEST(UnicoreTransport, ExtractsLargeObsvmcmpFrameBelowConfiguredLimit)
{
  UnicoreTransport transport({true, true, 2048U});
  std::vector<uint8_t> payload(4U + 24U * 50U, 0x00U);
  payload[0] = 50U;
  const std::string frame = make_binary_frame(138U, payload);

  transport.append(reinterpret_cast<const uint8_t*>(frame.data()), frame.size());
  const auto events = transport.drain();

  ASSERT_EQ(events.size(), 1U);
  ASSERT_TRUE(events[0].binary_frame.has_value());
  EXPECT_EQ(events[0].binary_frame->message_id, 138U);
  EXPECT_EQ(events[0].binary_frame->payload_length, payload.size());
  EXPECT_TRUE(events[0].binary_frame->crc_valid);
}

TEST(UnicoreTransport, ExtractsObservedBestnavStyleFrame)
{
  UnicoreTransport transport({true, true, 512U});
  std::vector<uint8_t> payload(120U, 0x00U);
  payload[0] = 0x34U;

  BinaryHeaderOverrides header;
  header.cpu_idle = 0x61U;
  header.time_ref = 0x00U;
  header.time_status = 0xA0U;
  header.week = 0x0972U;
  header.milliseconds = 0x0E5DEAE0U;

  const std::string frame = make_binary_frame(2118U, payload, true, header);
  const std::string observed_prefix =
      std::string("\xAA\x44\xB5\x61\x46\x08\x78\x00\x00\xA0\x72\x09\xE0\xEA\x5D\x0E", 16U);

  ASSERT_GE(frame.size(), 148U);
  EXPECT_EQ(frame.substr(0U, observed_prefix.size()), observed_prefix);

  transport.append(reinterpret_cast<const uint8_t*>(frame.data()), frame.size());
  const auto events = transport.drain();

  ASSERT_EQ(events.size(), 1U);
  ASSERT_TRUE(events.front().binary_frame.has_value());
  EXPECT_EQ(events.front().binary_frame->message_id, 2118U);
  EXPECT_EQ(events.front().binary_frame->payload_length, 120U);
  EXPECT_TRUE(events.front().binary_frame->crc_valid);
  EXPECT_EQ(events.front().binary_frame->payload.size(), 120U);
  EXPECT_EQ(transport.binary_counters().frames_total, 1U);
  EXPECT_EQ(transport.binary_counters().sync_candidates, 1U);
  ASSERT_TRUE(transport.binary_counters().last_header_length.has_value());
  ASSERT_TRUE(transport.binary_counters().last_payload_length.has_value());
  EXPECT_EQ(*transport.binary_counters().last_header_length, kUnicoreBinaryHeaderSize);
  EXPECT_EQ(*transport.binary_counters().last_payload_length, 120U);
}

TEST(UnicoreTransport, ExtractsCapturedBestnavFrameFromFieldDump)
{
  UnicoreTransport transport({true, true, 512U});
  const std::string frame = bytes_from_hex(
      "aa44b5614608780000a07209c82ba4120000000000120c00000000001000000008cc66e025fa4540"
      "1052859c789e01400000b0c5dd2b65406aa349423d000000325ac03f62b68f3fdb10104030000000"
      "0000000000005442211c1c0004121151000000000800000000000000000000009c6414e86d43803f"
      "8ef677e58cd36340c1c1bfecd94386bfe13edb3c8758ab3c961ec655");

  ASSERT_EQ(frame.size(), 148U);
  EXPECT_EQ(static_cast<unsigned char>(frame[0]), 0xAAU);
  EXPECT_EQ(static_cast<unsigned char>(frame[1]), 0x44U);
  EXPECT_EQ(static_cast<unsigned char>(frame[2]), 0xB5U);
  EXPECT_EQ(static_cast<unsigned char>(frame[3]), 0x61U);
  EXPECT_EQ(static_cast<unsigned char>(frame[4]), 0x46U);
  EXPECT_EQ(static_cast<unsigned char>(frame[5]), 0x08U);
  EXPECT_EQ(static_cast<unsigned char>(frame[6]), 0x78U);
  EXPECT_EQ(static_cast<unsigned char>(frame[7]), 0x00U);

  transport.append(reinterpret_cast<const uint8_t*>(frame.data()), frame.size());
  const auto events = transport.drain();

  ASSERT_EQ(events.size(), 1U);
  ASSERT_TRUE(events.front().binary_frame.has_value());
  EXPECT_EQ(events.front().binary_frame->message_id, 2118U);
  EXPECT_EQ(events.front().binary_frame->payload_length, 120U);
  EXPECT_TRUE(events.front().binary_frame->crc_valid);
  EXPECT_EQ(transport.binary_counters().frames_total, 1U);
  EXPECT_EQ(transport.binary_counters().crc_errors, 0U);
  EXPECT_EQ(transport.binary_counters().sync_candidates, 1U);
  ASSERT_TRUE(transport.binary_counters().last_header_length.has_value());
  ASSERT_TRUE(transport.binary_counters().last_payload_length.has_value());
  EXPECT_EQ(*transport.binary_counters().last_header_length, 24U);
  EXPECT_EQ(*transport.binary_counters().last_payload_length, 120U);
}

}  // namespace unicore_gnss
