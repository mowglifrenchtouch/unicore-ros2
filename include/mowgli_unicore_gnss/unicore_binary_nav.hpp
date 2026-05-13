// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>

#include "unicore_gnss/unicore_transport.hpp"
#include "unicore_gnss/um982_parser.hpp"

namespace unicore_gnss
{

class UnicoreBinaryNavParser
{
public:
  std::optional<ParsedSentence> parse(const UnicoreBinaryFrame& frame) const;

private:
  static std::optional<ParsedSentence> parse_agcb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_bestsatb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_bestnavb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_freqjamstatusb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_hwstatusb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_jamstatusb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_obsvmcmpb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_pvtslnb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_rtkstatusb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_rtcmstatusb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_satsinfob(const UnicoreBinaryFrame& frame);
};

}  // namespace unicore_gnss
