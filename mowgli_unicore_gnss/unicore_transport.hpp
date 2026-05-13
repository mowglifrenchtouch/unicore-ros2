// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mowgli_unicore_gnss
{

constexpr std::size_t kUnicoreBinaryHeaderSize = 24U;
constexpr std::size_t kUnicoreBinaryCrcSize = 4U;

enum class UnicoreTransportEventKind : uint8_t
{
  kAsciiLine,
  kBinaryFrame,
};

struct UnicoreBinaryFrame
{
  uint8_t cpu_idle{0U};
  uint16_t message_id{0U};
  uint16_t payload_length{0U};
  uint8_t time_ref{0U};
  uint8_t time_status{0U};
  uint16_t week{0U};
  uint32_t milliseconds{0U};
  uint32_t version{0U};
  uint8_t leap_seconds{0U};
  uint16_t delay_ms{0U};
  bool crc_valid{false};
  uint32_t crc32{0U};
  std::vector<uint8_t> payload;
};

struct UnicoreTransportEvent
{
  UnicoreTransportEventKind kind{UnicoreTransportEventKind::kAsciiLine};
  std::string ascii_line;
  std::optional<UnicoreBinaryFrame> binary_frame;
};

struct UnicoreTransportOptions
{
  bool enable_binary{false};
  bool strict_binary_crc{true};
  std::size_t binary_max_frame_size{4096U};
};

struct UnicoreBinaryTransportCounters
{
  std::size_t frames_total{0U};
  std::size_t crc_errors{0U};
  std::size_t resync_count{0U};
};

struct UnicoreBinaryDispatchResult
{
  bool known_message{false};
  std::string message_name;
};

struct UnicoreBinaryDispatchCounters
{
  std::size_t dispatched_frames{0U};
  std::size_t unknown_frames{0U};
  std::optional<uint16_t> last_message_id;
  std::vector<uint16_t> recent_unknown_message_ids;
};

class UnicoreTransport
{
public:
  explicit UnicoreTransport(UnicoreTransportOptions options = {});

  void set_options(const UnicoreTransportOptions& options);
  void append(const uint8_t* data, std::size_t size);
  std::vector<UnicoreTransportEvent> drain();
  void clear();
  std::size_t buffered_bytes() const;
  UnicoreBinaryTransportCounters binary_counters() const;

private:
  static std::size_t find_binary_sync(std::string_view buffer);
  static bool begins_with_binary_sync_prefix(std::string_view buffer);
  static uint16_t read_le16(const unsigned char* data);
  static uint32_t read_le32(const unsigned char* data);
  static uint32_t crc32_unicore_binary(std::string_view data);

  bool try_extract_binary_frame(std::vector<UnicoreTransportEvent>& events);

  UnicoreTransportOptions options_;
  std::string buffer_;
  UnicoreBinaryTransportCounters binary_counters_{};
};

class UnicoreBinaryDispatcher
{
public:
  UnicoreBinaryDispatchResult dispatch(const UnicoreBinaryFrame& frame);
  UnicoreBinaryDispatchCounters counters() const;

private:
  static const char* known_message_name(uint16_t message_id);

  UnicoreBinaryDispatchCounters counters_{};
};

}  // namespace mowgli_unicore_gnss
