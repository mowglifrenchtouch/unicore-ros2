// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include "mowgli_unicore_gnss/diagnostic_state.hpp"

#include <gtest/gtest.h>

namespace mowgli_unicore_gnss
{

TEST(DiagnosticState, DisabledWhenFeatureTurnedOff)
{
  EXPECT_EQ(diagnostic_feed_state(false, false), DiagnosticFeedState::kDisabled);
  EXPECT_STREQ(diagnostic_feed_state_name(DiagnosticFeedState::kDisabled), "disabled");
}

TEST(DiagnosticState, StaleWhenEnabledWithoutFreshData)
{
  EXPECT_EQ(diagnostic_feed_state(true, false), DiagnosticFeedState::kStale);
  EXPECT_STREQ(diagnostic_feed_state_name(DiagnosticFeedState::kStale), "stale");
}

TEST(DiagnosticState, LiveWhenEnabledWithFreshData)
{
  EXPECT_EQ(diagnostic_feed_state(true, true), DiagnosticFeedState::kLive);
  EXPECT_STREQ(diagnostic_feed_state_name(DiagnosticFeedState::kLive), "live");
}

}  // namespace mowgli_unicore_gnss
