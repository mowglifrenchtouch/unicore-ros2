// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace mowgli_unicore_gnss
{

enum class DiagnosticFeedState
{
  kDisabled,
  kStale,
  kLive,
};

inline DiagnosticFeedState diagnostic_feed_state(bool enabled, bool fresh_data_available)
{
  if (!enabled)
  {
    return DiagnosticFeedState::kDisabled;
  }
  return fresh_data_available ? DiagnosticFeedState::kLive : DiagnosticFeedState::kStale;
}

inline const char* diagnostic_feed_state_name(DiagnosticFeedState state)
{
  switch (state)
  {
    case DiagnosticFeedState::kDisabled:
      return "disabled";
    case DiagnosticFeedState::kStale:
      return "stale";
    case DiagnosticFeedState::kLive:
      return "live";
  }
  return "unknown";
}

}  // namespace mowgli_unicore_gnss
