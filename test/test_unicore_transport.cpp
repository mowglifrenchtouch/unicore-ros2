// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <string>
#include <vector>

#include "mowgli_unicore_gnss/unicore_transport.hpp"
#include <gtest/gtest.h>

namespace mowgli_unicore_gnss
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

std::string make_binary_frame(uint16_t message_id,
                              const std::vector<uint8_t>& payload,
                              bool valid_crc = true)
{
  std::string frame;
  frame.push_back(static_cast<char>(0xAAU));
  frame.push_back(static_cast<char>(0x44U));
  frame.push_back(static_cast<char>(0xB5U));
  frame.push_back(static_cast<char>(17U));          // cpu idle
  append_le16(frame, message_id);
  append_le16(frame, static_cast<uint16_t>(payload.size()));
  frame.push_back(static_cast<char>(1U));           // time ref
  frame.push_back(static_cast<char>(2U));           // time status
  append_le16(frame, 2294U);
  append_le32(frame, 472312000U);
  append_le32(frame, 16U);
  frame.push_back(static_cast<char>(0U));           // reserved
  frame.push_back(static_cast<char>(18U));          // leap second
  append_le16(frame, 97U);                          // delay ms
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
  EXPECT_TRUE(events[0].binary_frame->crc_valid);
  ASSERT_EQ(events[0].binary_frame->payload.size(), 4U);
  EXPECT_EQ(events[0].binary_frame->payload[0], 0x01U);
  EXPECT_EQ(transport.binary_counters().frames_total, 1U);
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

}  // namespace mowgli_unicore_gnss
