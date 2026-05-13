// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include "unicore_gnss/unicore_transport.hpp"

#include <algorithm>
#include <array>
#include <string_view>
#include <unordered_map>

namespace unicore_gnss
{

namespace
{

constexpr std::array<unsigned char, 3> kBinarySync{{0xAAU, 0x44U, 0xB5U}};
constexpr std::size_t kRecentUnknownIdsLimit = 8U;

}  // namespace

UnicoreTransport::UnicoreTransport(UnicoreTransportOptions options) : options_(options) {}

void UnicoreTransport::set_options(const UnicoreTransportOptions& options)
{
  options_ = options;
}

void UnicoreTransport::append(const uint8_t* data, std::size_t size)
{
  if (data == nullptr || size == 0U)
  {
    return;
  }
  buffer_.append(reinterpret_cast<const char*>(data), size);
}

std::vector<UnicoreTransportEvent> UnicoreTransport::drain()
{
  std::vector<UnicoreTransportEvent> events;

  while (!buffer_.empty())
  {
    if (!options_.enable_binary)
    {
      const std::size_t newline = buffer_.find('\n');
      if (newline == std::string::npos)
      {
        break;
      }

      std::string line = buffer_.substr(0U, newline);
      buffer_.erase(0U, newline + 1U);
      if (!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }
      if (!line.empty())
      {
        UnicoreTransportEvent event;
        event.kind = UnicoreTransportEventKind::kAsciiLine;
        event.ascii_line = std::move(line);
        events.push_back(std::move(event));
      }
      continue;
    }

    const std::size_t sync = find_binary_sync(buffer_);
    const std::size_t newline = buffer_.find('\n');

    if (sync == 0U)
    {
      if (try_extract_binary_frame(events))
      {
        continue;
      }
      break;
    }

    if (newline != std::string::npos && (sync == std::string::npos || newline < sync))
    {
      std::string line = buffer_.substr(0U, newline);
      buffer_.erase(0U, newline + 1U);
      if (!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }
      if (!line.empty())
      {
        UnicoreTransportEvent event;
        event.kind = UnicoreTransportEventKind::kAsciiLine;
        event.ascii_line = std::move(line);
        events.push_back(std::move(event));
      }
      continue;
    }

    if (sync != std::string::npos && sync > 0U)
    {
      buffer_.erase(0U, sync);
      ++binary_counters_.resync_count;
      continue;
    }

    if (!buffer_.empty() && (buffer_.front() == '$' || buffer_.front() == '#'))
    {
      break;
    }

    if (begins_with_binary_sync_prefix(buffer_))
    {
      break;
    }

    buffer_.erase(0U, 1U);
    ++binary_counters_.resync_count;
  }

  return events;
}

void UnicoreTransport::clear()
{
  buffer_.clear();
}

std::size_t UnicoreTransport::buffered_bytes() const
{
  return buffer_.size();
}

UnicoreBinaryTransportCounters UnicoreTransport::binary_counters() const
{
  return binary_counters_;
}

std::size_t UnicoreTransport::find_binary_sync(std::string_view buffer)
{
  if (buffer.size() < kBinarySync.size())
  {
    return std::string::npos;
  }

  for (std::size_t i = 0U; i + kBinarySync.size() <= buffer.size(); ++i)
  {
    if (static_cast<unsigned char>(buffer[i]) == kBinarySync[0] &&
        static_cast<unsigned char>(buffer[i + 1U]) == kBinarySync[1] &&
        static_cast<unsigned char>(buffer[i + 2U]) == kBinarySync[2])
    {
      return i;
    }
  }
  return std::string::npos;
}

bool UnicoreTransport::begins_with_binary_sync_prefix(std::string_view buffer)
{
  const std::size_t count = std::min(buffer.size(), kBinarySync.size() - 1U);
  if (count == 0U)
  {
    return false;
  }

  for (std::size_t i = 0U; i < count; ++i)
  {
    if (static_cast<unsigned char>(buffer[i]) != kBinarySync[i])
    {
      return false;
    }
  }
  return true;
}

uint16_t UnicoreTransport::read_le16(const unsigned char* data)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) |
                               (static_cast<uint16_t>(data[1]) << 8U));
}

uint32_t UnicoreTransport::read_le32(const unsigned char* data)
{
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
}

uint32_t UnicoreTransport::crc32_unicore_binary(std::string_view data)
{
  uint32_t crc = 0U;
  for (const unsigned char byte : data)
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

bool UnicoreTransport::try_extract_binary_frame(std::vector<UnicoreTransportEvent>& events)
{
  if (buffer_.size() < kUnicoreBinaryHeaderSize + kUnicoreBinaryCrcSize)
  {
    return false;
  }

  const auto* raw = reinterpret_cast<const unsigned char*>(buffer_.data());
  const uint16_t payload_length = read_le16(raw + 6U);
  const std::size_t frame_size =
      kUnicoreBinaryHeaderSize + static_cast<std::size_t>(payload_length) + kUnicoreBinaryCrcSize;
  if (frame_size > options_.binary_max_frame_size)
  {
    buffer_.erase(0U, 1U);
    ++binary_counters_.resync_count;
    return true;
  }

  if (buffer_.size() < frame_size)
  {
    return false;
  }

  const uint32_t expected_crc = read_le32(raw + frame_size - kUnicoreBinaryCrcSize);
  const uint32_t actual_crc = crc32_unicore_binary(std::string_view(buffer_.data(), frame_size - 4U));
  const bool crc_ok = expected_crc == actual_crc;
  if (!crc_ok)
  {
    ++binary_counters_.crc_errors;
    if (options_.strict_binary_crc)
    {
      buffer_.erase(0U, 1U);
      ++binary_counters_.resync_count;
      return true;
    }
  }

  // N4 R1.4 binary fields are little-endian, matching the documented byte offsets.
  UnicoreBinaryFrame frame;
  frame.cpu_idle = raw[3U];
  frame.message_id = read_le16(raw + 4U);
  frame.payload_length = payload_length;
  frame.time_ref = raw[8U];
  frame.time_status = raw[9U];
  frame.week = read_le16(raw + 10U);
  frame.milliseconds = read_le32(raw + 12U);
  frame.version = read_le32(raw + 16U);
  frame.leap_seconds = raw[21U];
  frame.delay_ms = read_le16(raw + 22U);
  frame.crc_valid = crc_ok;
  frame.crc32 = expected_crc;
  frame.payload.assign(raw + kUnicoreBinaryHeaderSize, raw + frame_size - kUnicoreBinaryCrcSize);

  buffer_.erase(0U, frame_size);
  ++binary_counters_.frames_total;

  UnicoreTransportEvent event;
  event.kind = UnicoreTransportEventKind::kBinaryFrame;
  event.binary_frame = std::move(frame);
  events.push_back(std::move(event));
  return true;
}

UnicoreBinaryDispatchResult UnicoreBinaryDispatcher::dispatch(const UnicoreBinaryFrame& frame)
{
  UnicoreBinaryDispatchResult result;
  result.message_name = known_message_name(frame.message_id);
  result.known_message = !result.message_name.empty();

  ++counters_.dispatched_frames;
  counters_.last_message_id = frame.message_id;

  if (!result.known_message)
  {
    ++counters_.unknown_frames;
    const auto it = std::find(counters_.recent_unknown_message_ids.begin(),
                              counters_.recent_unknown_message_ids.end(),
                              frame.message_id);
    if (it == counters_.recent_unknown_message_ids.end())
    {
      counters_.recent_unknown_message_ids.push_back(frame.message_id);
      if (counters_.recent_unknown_message_ids.size() > kRecentUnknownIdsLimit)
      {
        counters_.recent_unknown_message_ids.erase(counters_.recent_unknown_message_ids.begin());
      }
    }
  }

  return result;
}

UnicoreBinaryDispatchCounters UnicoreBinaryDispatcher::counters() const
{
  return counters_;
}

const char* UnicoreBinaryDispatcher::known_message_name(uint16_t message_id)
{
  static const std::unordered_map<uint16_t, const char*> kKnownMessageIds = {
      {138U, "OBSVMCMP"},
      {218U, "HWSTATUS"},
      {220U, "AGC"},
      {240U, "BESTNAVXYZ"},
      {242U, "BESTNAVXYZH"},
      {509U, "RTKSTATUS"},
      {511U, "JAMSTATUS"},
      {519U, "FREQJAMSTATUS"},
      {954U, "STADOP"},
      {972U, "UNIHEADING"},
      {1021U, "PVTSLN"},
      {1041U, "BESTSAT"},
      {2116U, "SPPNAVH"},
      {2117U, "ADRNAVH"},
      {2118U, "BESTNAV"},
      {2119U, "BESTNAVH"},
      {2124U, "SATSINFO"},
      {2125U, "RTCMSTATUS"},
  };

  const auto it = kKnownMessageIds.find(message_id);
  return it != kKnownMessageIds.end() ? it->second : "";
}

}  // namespace unicore_gnss
