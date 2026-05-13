// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include "unicore_gnss/um982_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

namespace unicore_gnss
{

namespace
{

  // PVTSLNA layout — empirically verified against UM982 firmware
// (real sample: `#PVTSLNA,78,GPS,FINE,2416,519196000,0,0,18,20;NARROW_FLOAT,
// 197.2120,43.59,0.90,0.2666,0.1021,0.2038,...`).
//
// Header is 10 comma-separated tokens (PVTSLNA, port, time_sys,
// time_status, gnss_week, gnss_seconds, status_a, status_b, leap_sec,
// rx_sw_version) terminated by `;`. The first data token (position-type,
// e.g. NARROW_INT / NARROW_FLOAT / PSRDIFF / NONE) is glued to the last
// header token via `;` — so a `,`-only split surfaces it as
// `"<rx_sw_version>;<position_type>"` at index 9. parse_pvtslna() peels
// the prefix off via find(';').
//
// After the position-type token, data continues: altitude (idx 10),
// latitude (11), longitude (12), height_std/lat_std/lon_std (13-15).
// The 7-field-header constant we used previously came from older
// Python references (sunshineharry/UM982Driver, lostDeers/UM982Driver-
// ros2) that omitted the same prefix and matched a different firmware
// revision. The data-field offsets (altitude at +3 etc.) are stable.
constexpr std::size_t kPvtslnaPositionTypeIndex = 9;
constexpr std::size_t kPvtslnaBestposHeightIndex = 10;
constexpr std::size_t kPvtslnaBestposLatitudeIndex = 11;
constexpr std::size_t kPvtslnaBestposLongitudeIndex = 12;
constexpr std::size_t kPvtslnaBestposHeightStdIndex = 13;
constexpr std::size_t kPvtslnaBestposLatitudeStdIndex = 14;
constexpr std::size_t kPvtslnaBestposLongitudeStdIndex = 15;
constexpr std::size_t kPvtslnaUndulationIndex = 21;
constexpr std::size_t kPvtslnaBestposTrackedSvsIndex = 22;
constexpr std::size_t kPvtslnaBestposSolutionSvsIndex = 23;
constexpr std::size_t kPvtslnaHdopIndex = 39;

// BESTNAVA layout per Table 7-85 in N4 R1.4:
//   #BESTNAVA,<ascii_header>;p_sol_status,pos_type,lat,lon,hgt,...
constexpr std::size_t kBestnavaPositionSolutionStatusIndex = 9;
constexpr std::size_t kBestnavaPositionTypeIndex = 10;
constexpr std::size_t kBestnavaLatitudeIndex = 11;
constexpr std::size_t kBestnavaLongitudeIndex = 12;
constexpr std::size_t kBestnavaHeightIndex = 13;
constexpr std::size_t kBestnavaUndulationIndex = 14;
constexpr std::size_t kBestnavaLatitudeStdIndex = 16;
constexpr std::size_t kBestnavaLongitudeStdIndex = 17;
constexpr std::size_t kBestnavaHeightStdIndex = 18;
constexpr std::size_t kBestnavaBaseStationIdIndex = 19;
constexpr std::size_t kBestnavaDiffAgeIndex = 20;
constexpr std::size_t kBestnavaSolutionAgeIndex = 21;
constexpr std::size_t kBestnavaTrackedSvsIndex = 22;
constexpr std::size_t kBestnavaSolutionSvsIndex = 23;
constexpr std::size_t kBestnavaExtendedSolutionStatusIndex = 27;
constexpr std::size_t kBestnavaGalileoBds3SignalMaskIndex = 28;
constexpr std::size_t kBestnavaGpsGloBds2SignalMaskIndex = 29;
constexpr std::size_t kBestnavaVelocitySolutionStatusIndex = 30;
constexpr std::size_t kBestnavaVelocityTypeIndex = 31;
constexpr std::size_t kBestnavaVelocityLatencyIndex = 32;
constexpr std::size_t kBestnavaVelocityAgeIndex = 33;
constexpr std::size_t kBestnavaHorizontalSpeedIndex = 34;
constexpr std::size_t kBestnavaTrackGroundIndex = 35;
constexpr std::size_t kBestnavaVerticalSpeedIndex = 36;
constexpr std::size_t kBestnavaVerticalSpeedStdIndex = 37;
constexpr std::size_t kBestnavaHorizontalSpeedStdIndex = 38;

// RTKSTATUSA layout per Table 7-121 in N4 R1.4:
//   #RTKSTATUSA,<ascii_header>;gpsSource,reserved,bdsSource1,...
constexpr std::size_t kRtkstatusaGpsSourceIndex = 9;
constexpr std::size_t kRtkstatusaBdsSource1Index = 11;
constexpr std::size_t kRtkstatusaBdsSource2Index = 12;
constexpr std::size_t kRtkstatusaGlonassSourceIndex = 14;
constexpr std::size_t kRtkstatusaGalileoSource1Index = 16;
constexpr std::size_t kRtkstatusaGalileoSource2Index = 17;
constexpr std::size_t kRtkstatusaQzssSourceIndex = 18;
constexpr std::size_t kRtkstatusaPositionTypeIndex = 20;
constexpr std::size_t kRtkstatusaCalculateStatusIndex = 21;
constexpr std::size_t kRtkstatusaIonDetectedIndex = 22;
constexpr std::size_t kRtkstatusaDualRtkFlagIndex = 23;
constexpr std::size_t kRtkstatusaAdrObservationCountIndex = 24;

// RTCMSTATUSA layout per Table 7-126 in N4 R1.4:
//   #RTCMSTATUSA,<ascii_header>;msg_id,msg_num,base_id,sats_num,l1,...,l6
constexpr std::size_t kRtcmstatusaMessageIdIndex = 9;
constexpr std::size_t kRtcmstatusaMessageCountIndex = 10;
constexpr std::size_t kRtcmstatusaBaseStationIdIndex = 11;
constexpr std::size_t kRtcmstatusaSatelliteCountIndex = 12;
constexpr std::size_t kRtcmstatusaL1CountIndex = 13;
constexpr std::size_t kRtcmstatusaL6CountIndex = 18;

// BESTSATA layout per Table 7-92 in N4 R1.4:
//   #BESTSATA,<ascii_header>;#entries,constellation,satellite_id,status,signal_mask,...
constexpr std::size_t kBestsataEntryCountIndex = 9;
constexpr std::size_t kBestsataFirstEntryIndex = 10;
constexpr std::size_t kBestsataEntryFieldCount = 4;

// SATSINFOA layout per Table 7-109 in N4 R1.4:
//   #SATSINFOA,<ascii_header>;sat_count,version,res,res,res,freq_flag,...
constexpr std::size_t kSatsinfoaSatelliteCountIndex = 9;
constexpr std::size_t kSatsinfoaVersionIndex = 10;
constexpr std::size_t kSatsinfoaFrequencyFlagIndex = 14;
constexpr std::size_t kSatsinfoaFirstSatelliteIndex = 15;
constexpr std::size_t kSatsinfoaBaseSatelliteFieldCount = 7;
constexpr std::size_t kSatsinfoaExtraFrequencyFieldCount = 4;

// AGCA layout per Table 7-130 in N4 R1.4:
//   #AGCA,<ascii_header>;ant1_l1,ant1_l2,ant1_l5,res,res,ant2_l1,ant2_l2,ant2_l5,...
constexpr std::size_t kAgcaAnt1L1Index = 9;
constexpr std::size_t kAgcaAnt1L2Index = 10;
constexpr std::size_t kAgcaAnt1L5Index = 11;
constexpr std::size_t kAgcaAnt2L1Index = 14;
constexpr std::size_t kAgcaAnt2L2Index = 15;
constexpr std::size_t kAgcaAnt2L5Index = 16;

// HWSTATUSA layout per Table 7-128 in N4 R1.4:
//   #HWSTATUSA,<ascii_header>;reserved,dc09,dc10,dc18,clockflag,clockdrift,...
constexpr std::size_t kHwstatusaDc09Index = 10;
constexpr std::size_t kHwstatusaDc10Index = 11;
constexpr std::size_t kHwstatusaDc18Index = 12;
constexpr std::size_t kHwstatusaClockFlagIndex = 13;
constexpr std::size_t kHwstatusaClockDriftIndex = 14;
constexpr std::size_t kHwstatusaHwFlagIndex = 16;
constexpr std::size_t kHwstatusaPllLockIndex = 18;

// JAMSTATUSA layout per Table 7-124 in N4 R1.4:
//   #JAMSTATUSA,<ascii_header>;pos_type,cw_ratio,cw_flag,...
constexpr std::size_t kJamstatusaPositionTypeIndex = 9;
constexpr std::size_t kJamstatusaCwRatioIndex = 10;
constexpr std::size_t kJamstatusaCwFlagIndex = 11;

// FREQJAMSTATUSA layout per Table 7-125 in N4 R1.4:
//   #FREQJAMSTATUSA,<ascii_header>;pos_type,l1_ratio,l1_flag,l2_ratio,l2_flag,l5_ratio,l5_flag,...
constexpr std::size_t kFreqjamstatusaPositionTypeIndex = 9;
constexpr std::size_t kFreqjamstatusaL1RatioIndex = 10;
constexpr std::size_t kFreqjamstatusaL1FlagIndex = 11;
constexpr std::size_t kFreqjamstatusaL2RatioIndex = 12;
constexpr std::size_t kFreqjamstatusaL2FlagIndex = 13;
constexpr std::size_t kFreqjamstatusaL5RatioIndex = 14;
constexpr std::size_t kFreqjamstatusaL5FlagIndex = 15;
// N4 R1.4 PVTSLNA layout:
//   #PVTSLNA,<ascii_header>;bestpos_type,bestpos_hgt,bestpos_lat,...
//
// The Unicore ASCII header is comma-separated and the first data field is
// attached to the last header token through `;`, so a comma-only split
// yields `<delay_ms>;<bestpos_type>` at index 9. The following indices are
// then aligned with Table 7-82 from the N4 R1.4 manual.

uint32_t crc32_unicore(std::string_view text)
{
  uint32_t crc = 0U;
  for (const unsigned char ch : text)
  {
    crc ^= static_cast<uint32_t>(ch);
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

std::string sentence_suffix(std::string_view type)
{
  if (type.size() <= 3U)
  {
    return std::string(type);
  }
  return std::string(type.substr(type.size() - 3U));
}

std::string_view field_after_semicolon(std::string_view field)
{
  const std::size_t semi = field.find(';');
  if (semi == std::string_view::npos)
  {
    return field;
  }
  return field.substr(semi + 1U);
}

std::string trim_ascii_quotes(std::string_view text)
{
  if (text.size() >= 2U && text.front() == '"' && text.back() == '"')
  {
    return std::string(text.substr(1U, text.size() - 2U));
  }
  return std::string(text);
}

std::string normalize_constellation_name(std::string_view text)
{
  if (text == "GPS") return "GPS";
  if (text == "GLONASS" || text == "GLO") return "GLO";
  if (text == "GALILEO" || text == "GAL") return "GAL";
  if (text == "BEIDOU" || text == "BDS") return "BDS";
  if (text == "QZSS") return "QZSS";
  if (text == "IRNSS" || text == "NAVIC") return "IRNSS";
  if (text == "SBAS") return "SBAS";
  return std::string(text);
}

std::string constellation_name_from_system_id(int system_id)
{
  switch (system_id)
  {
    case 0:
      return "GPS";
    case 1:
      return "GLO";
    case 2:
      return "SBAS";
    case 3:
      return "GAL";
    case 4:
      return "BDS";
    case 5:
      return "QZSS";
    case 6:
      return "IRNSS";
    default:
      return "UNKNOWN";
  }
}

std::vector<std::string> bestsat_signal_bands(std::string_view constellation, uint32_t signal_mask)
{
  std::vector<std::string> bands;
  if (constellation == "GPS")
  {
    if ((signal_mask & 0x01U) != 0U) bands.emplace_back("L1");
    if ((signal_mask & 0x02U) != 0U) bands.emplace_back("L2");
    if ((signal_mask & 0x04U) != 0U) bands.emplace_back("L5");
  }
  else if (constellation == "GLO")
  {
    if ((signal_mask & 0x01U) != 0U) bands.emplace_back("L1");
    if ((signal_mask & 0x02U) != 0U) bands.emplace_back("L2");
    if ((signal_mask & 0x04U) != 0U) bands.emplace_back("L3");
  }
  else if (constellation == "BDS")
  {
    if ((signal_mask & 0x01U) != 0U) bands.emplace_back("B1");
    if ((signal_mask & 0x02U) != 0U) bands.emplace_back("B2");
    if ((signal_mask & 0x04U) != 0U) bands.emplace_back("B3");
  }
  else if (constellation == "GAL")
  {
    if ((signal_mask & 0x01U) != 0U) bands.emplace_back("E1");
    if ((signal_mask & 0x02U) != 0U) bands.emplace_back("E5");
    if ((signal_mask & 0x04U) != 0U) bands.emplace_back("E5");
    if ((signal_mask & 0x08U) != 0U) bands.emplace_back("E6");
  }
  return bands;
}

std::string signal_band_from_frequency_id(std::string_view constellation, int frequency_id)
{
  if (constellation == "GPS" || constellation == "QZSS")
  {
    if (frequency_id == 0 || frequency_id == 3 || frequency_id == 11) return "L1";
    if (frequency_id == 9 || frequency_id == 17) return "L2";
    if (frequency_id == 6 || frequency_id == 14) return "L5";
    if (constellation == "QZSS" &&
        (frequency_id == 18 || frequency_id == 22 || frequency_id == 24 || frequency_id == 25))
    {
      return "L6";
    }
  }
  else if (constellation == "GLO")
  {
    if (frequency_id == 0) return "L1";
    if (frequency_id == 5) return "L2";
    if (frequency_id == 6 || frequency_id == 7) return "L3";
  }
  else if (constellation == "GAL")
  {
    if (frequency_id == 1 || frequency_id == 2) return "E1";
    if (frequency_id == 12 || frequency_id == 17) return "E5";
    if (frequency_id == 18 || frequency_id == 22) return "E6";
  }
  else if (constellation == "BDS")
  {
    if (frequency_id == 0 || frequency_id == 4 || frequency_id == 8 || frequency_id == 23)
    {
      return "B1";
    }
    if (frequency_id == 5 || frequency_id == 17 || frequency_id == 12 ||
        frequency_id == 28 || frequency_id == 13)
    {
      return "B2";
    }
    if (frequency_id == 6 || frequency_id == 21)
    {
      return "B3";
    }
  }
  else if (constellation == "SBAS")
  {
    if (frequency_id == 0) return "L1";
    if (frequency_id == 6) return "L5";
  }
  else if (constellation == "IRNSS")
  {
    if (frequency_id == 6 || frequency_id == 14) return "L5";
  }
  return "";
}

}  // namespace

std::optional<ParsedSentence> Um982Parser::parse_line(const std::string& line) const
{
  const std::string trimmed = trim(line);
  if (trimmed.empty())
  {
    return std::nullopt;
  }

  if (trimmed.front() == '$')
  {
    if (!validate_nmea_checksum(trimmed))
    {
      ++counters_.nmea_checksum_errors;
      return std::nullopt;
    }

    const std::size_t star = trimmed.find('*');
    const auto fields = split_fields(std::string_view(trimmed).substr(1U, star - 1U));
    if (fields.empty())
    {
      return std::nullopt;
    }

    const std::string suffix = sentence_suffix(fields.front());
    if (suffix == "GGA")
    {
      auto parsed = parse_gga(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (suffix == "HDT")
    {
      auto parsed = parse_hdt(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (suffix == "HPR")
    {
      auto parsed = parse_hpr(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (suffix == "GSV")
    {
      // GSV talker prefix is the first two chars of the sentence id
      // ($GPGSV, $GLGSV, $GAGSV, $GBGSV, $GQGSV, $GIGSV, $GNGSV).
      const std::string_view first = fields.front();
      const std::string talker = first.size() >= 5U ? std::string(first.substr(0U, 2U))
                                                    : std::string("GN");
      auto parsed = parse_gsv(talker, fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    return std::nullopt;
  }

  if (trimmed.front() == '#')
  {
    if (!validate_unicore_crc(trimmed))
    {
      ++counters_.unicore_crc_errors;
      return std::nullopt;
    }

    const std::size_t star = trimmed.find('*');
    const auto fields = split_fields(std::string_view(trimmed).substr(1U, star - 1U));
    if (fields.empty())
    {
      return std::nullopt;
    }

    if (fields.front() == "PVTSLNA")
    {
      auto parsed = parse_pvtslna(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "BESTNAVA")
    {
      auto parsed = parse_bestnava(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "RTKSTATUSA")
    {
      auto parsed = parse_rtkstatusa(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "RTCMSTATUSA")
    {
      auto parsed = parse_rtcmstatusa(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "BESTSATA")
    {
      auto parsed = parse_bestsata(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "SATSINFOA")
    {
      auto parsed = parse_satsinfoa(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "AGCA")
    {
      auto parsed = parse_agca(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "HWSTATUSA")
    {
      auto parsed = parse_hwstatusa(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "JAMSTATUSA")
    {
      auto parsed = parse_jamstatusa(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
    if (fields.front() == "FREQJAMSTATUSA")
    {
      auto parsed = parse_freqjamstatusa(fields);
      parsed.has_value() ? ++counters_.parsed_sentences : ++counters_.parse_errors;
      return parsed;
    }
  }

  return std::nullopt;
}

ParserCounters Um982Parser::counters() const
{
  return counters_;
}

bool Um982Parser::validate_nmea_checksum(std::string_view line)
{
  if (line.size() < 4U || line.front() != '$')
  {
    return false;
  }

  const std::size_t star = line.find('*');
  if (star == std::string_view::npos || star + 2U >= line.size())
  {
    return false;
  }

  uint8_t checksum = 0U;
  for (std::size_t i = 1U; i < star; ++i)
  {
    checksum ^= static_cast<uint8_t>(line[i]);
  }

  char* end = nullptr;
  const auto received = static_cast<unsigned long>(
      std::strtoul(std::string(line.substr(star + 1U, 2U)).c_str(), &end, 16));
  return end != nullptr && *end == '\0' && checksum == static_cast<uint8_t>(received);
}

bool Um982Parser::validate_unicore_crc(std::string_view line)
{
  if (line.size() < 11U || line.front() != '#')
  {
    return false;
  }

  const std::size_t star = line.find('*');
  if (star == std::string_view::npos || star + 8U >= line.size())
  {
    return false;
  }

  const uint32_t calculated = crc32_unicore(line.substr(1U, star - 1U));
  char* end = nullptr;
  const uint32_t received = static_cast<uint32_t>(
      std::strtoul(std::string(line.substr(star + 1U, 8U)).c_str(), &end, 16));
  return end != nullptr && *end == '\0' && calculated == received;
}

std::vector<std::string_view> Um982Parser::split_fields(std::string_view payload)
{
  std::vector<std::string_view> fields;
  std::size_t start = 0U;
  while (start <= payload.size())
  {
    const std::size_t comma = payload.find(',', start);
    if (comma == std::string_view::npos)
    {
      fields.emplace_back(payload.substr(start));
      break;
    }
    fields.emplace_back(payload.substr(start, comma - start));
    start = comma + 1U;
  }
  return fields;
}

std::string Um982Parser::trim(std::string_view text)
{
  std::size_t start = 0U;
  std::size_t end = text.size();
  while (start < end && std::isspace(static_cast<unsigned char>(text[start])) != 0)
  {
    ++start;
  }
  while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1U])) != 0)
  {
    --end;
  }
  return std::string(text.substr(start, end - start));
}

bool Um982Parser::parse_double(std::string_view field, double& value)
{
  if (field.empty())
  {
    return false;
  }

  char* end = nullptr;
  const std::string text(field);
  value = std::strtod(text.c_str(), &end);
  return end != nullptr && *end == '\0' && std::isfinite(value);
}

bool Um982Parser::parse_int(std::string_view field, int& value)
{
  if (field.empty())
  {
    return false;
  }

  char* end = nullptr;
  const std::string text(field);
  const long parsed = std::strtol(text.c_str(), &end, 10);
  if (end == nullptr || *end != '\0')
  {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool Um982Parser::parse_uint32(std::string_view field, int base, uint32_t& value)
{
  if (field.empty())
  {
    return false;
  }

  char* end = nullptr;
  const std::string text(field);
  const unsigned long parsed = std::strtoul(text.c_str(), &end, base);
  if (end == nullptr || *end != '\0' || parsed > std::numeric_limits<uint32_t>::max())
  {
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool Um982Parser::parse_latlon(std::string_view value_field,
                               std::string_view hemi_field,
                               bool is_latitude,
                               double& degrees)
{
  double raw = 0.0;
  if (!parse_double(value_field, raw))
  {
    return false;
  }
  if (hemi_field.size() != 1U)
  {
    return false;
  }

  const double divisor = is_latitude ? 100.0 : 100.0;
  const double whole = std::floor(raw / divisor);
  const double minutes = raw - (whole * divisor);
  degrees = whole + minutes / 60.0;

  const char hemi = static_cast<char>(std::toupper(static_cast<unsigned char>(hemi_field.front())));
  if (hemi == 'S' || hemi == 'W')
  {
    degrees = -degrees;
  }
  return hemi == 'N' || hemi == 'S' || hemi == 'E' || hemi == 'W';
}

std::optional<ParsedSentence> Um982Parser::parse_gga(const std::vector<std::string_view>& fields)
{
  if (fields.size() < 10U)
  {
    return std::nullopt;
  }

  double latitude = 0.0;
  double longitude = 0.0;
  if (!parse_latlon(fields[2], fields[3], true, latitude) ||
      !parse_latlon(fields[4], fields[5], false, longitude))
  {
    return std::nullopt;
  }

  int quality = 0;
  int satellites = -1;
  double hdop = -1.0;
  double altitude_msl = 0.0;
  const bool quality_ok = parse_int(fields[6], quality);
  const bool satellites_ok = fields.size() > 7U && parse_int(fields[7], satellites);
  const bool hdop_ok = fields.size() > 8U && parse_double(fields[8], hdop);
  const bool altitude_ok = parse_double(fields[9], altitude_msl);

  double geoid_separation = 0.0;
  const bool geoid_ok = fields.size() > 11U && parse_double(fields[11], geoid_separation);

  ParsedSentence sentence;
  sentence.sentence_type = "GGA";
  sentence.fix = FixData{};
  sentence.fix->source = FixSource::kGga;
  sentence.fix->valid_fix = quality_ok && quality > 0;
  sentence.fix->latitude_deg = latitude;
  sentence.fix->longitude_deg = longitude;
  sentence.fix->altitude_m = altitude_ok ? altitude_msl + (geoid_ok ? geoid_separation : 0.0) : 0.0;
  sentence.fix->fix_quality = quality_ok ? quality : 0;
  sentence.fix->satellites = satellites_ok ? satellites : -1;
  sentence.fix->hdop = hdop_ok ? hdop : -1.0;
  sentence.fix->has_covariance = false;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_hdt(const std::vector<std::string_view>& fields)
{
  if (fields.size() < 2U)
  {
    return std::nullopt;
  }

  double heading = 0.0;
  if (!parse_double(fields[1], heading))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "HDT";
  sentence.heading = HeadingData{};
  sentence.heading->source = HeadingSource::kHdt;
  sentence.heading->heading_deg = heading;
  sentence.heading->variance_deg2 = 0.0;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_hpr(const std::vector<std::string_view>& fields)
{
  if (fields.size() < 5U)
  {
    return std::nullopt;
  }

  double heading = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  if (!parse_double(fields[2], heading) || !parse_double(fields[3], pitch) ||
      !parse_double(fields[4], roll))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "HPR";
  sentence.heading = HeadingData{};
  sentence.heading->source = HeadingSource::kHpr;
  sentence.heading->heading_deg = heading;
  sentence.heading->pitch_deg = pitch;
  sentence.heading->roll_deg = roll;
  sentence.heading->variance_deg2 = 0.0;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_pvtslna(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kPvtslnaBestposLongitudeStdIndex)
  {
    return std::nullopt;
  }

  double bestpos_height_msl = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude_std = 0.0;
  double latitude_std = 0.0;
  double longitude_std = 0.0;

  if (!parse_double(fields[kPvtslnaBestposHeightIndex], bestpos_height_msl) ||
      !parse_double(fields[kPvtslnaBestposLatitudeIndex], latitude) ||
      !parse_double(fields[kPvtslnaBestposLongitudeIndex], longitude) ||
      !parse_double(fields[kPvtslnaBestposHeightStdIndex], altitude_std) ||
      !parse_double(fields[kPvtslnaBestposLatitudeStdIndex], latitude_std) ||
      !parse_double(fields[kPvtslnaBestposLongitudeStdIndex], longitude_std))
  {
    return std::nullopt;
  }

  // Map the position-type field (string or numeric) to an NMEA GGA
  // quality code. The Unicore header/data separator `;` glues the last
  // header token to the first data token in a `,`-only split — so peel
  // the prefix off if present (real-world: `"20;NARROW_FLOAT"` → `"NARROW_FLOAT"`).
  // Quality 0 means "no fix" — accept the position anyway because the
  // receiver may still emit covariance on the PVTSLNA stream during
  // cold-start, and downstream consumers gate on NavSatStatus.status
  // not on valid_fix.
  
  std::string_view pos_type_field = fields[kPvtslnaPositionTypeIndex];
  const std::size_t semi = pos_type_field.find(';');
  if (semi != std::string_view::npos)
  {
    pos_type_field = pos_type_field.substr(semi + 1U);
  }
  const int quality = position_type_to_gga_quality(pos_type_field);

  double undulation = 0.0;
  const bool undulation_ok = fields.size() > kPvtslnaUndulationIndex &&
                             parse_double(fields[kPvtslnaUndulationIndex], undulation);

  int tracked_satellites = -1;
  if (fields.size() > kPvtslnaBestposTrackedSvsIndex)
  {
    (void)parse_int(fields[kPvtslnaBestposTrackedSvsIndex], tracked_satellites);
  }

  int solution_satellites = -1;
  if (fields.size() > kPvtslnaBestposSolutionSvsIndex)
  {
    (void)parse_int(fields[kPvtslnaBestposSolutionSvsIndex], solution_satellites);
  }

  double hdop = -1.0;
  if (fields.size() > kPvtslnaHdopIndex)
  {
    (void)parse_double(fields[kPvtslnaHdopIndex], hdop);
  }

  ParsedSentence sentence;
  sentence.sentence_type = "PVTSLNA";
  sentence.fix = FixData{};
  sentence.fix->source = FixSource::kPvtslna;
  sentence.fix->valid_fix = quality > 0;
  sentence.fix->fix_quality = quality;
  sentence.fix->latitude_deg = latitude;
  sentence.fix->longitude_deg = longitude;
  // PVTSLN bestpos_hgt is height above mean sea level. Convert to
  // ellipsoid height when undulation is available so NavSatFix matches
  // the existing GGA path.
  sentence.fix->altitude_m = bestpos_height_msl + (undulation_ok ? undulation : 0.0);
  sentence.fix->satellites = solution_satellites >= 0 ? solution_satellites : tracked_satellites;
  sentence.fix->hdop = hdop;
  sentence.fix->has_covariance = true;
  sentence.fix->covariance.fill(0.0);
  sentence.fix->covariance[0] = longitude_std * longitude_std;
  sentence.fix->covariance[4] = latitude_std * latitude_std;
  sentence.fix->covariance[8] = altitude_std * altitude_std;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_bestnava(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kBestnavaHorizontalSpeedStdIndex)
  {
    // Backward-compatible fallback for the older velocity-only test/sample
    // shape used before BESTNAVA was decoded according to N4 R1.4.
    if (fields.size() < 6U)
    {
      return std::nullopt;
    }

    double horizontal_speed = 0.0;
    double track_deg = 0.0;
    double up_speed = 0.0;
    double vertical_std = 0.0;
    double horizontal_std = 0.0;

    if (!parse_double(fields[fields.size() - 5U], horizontal_speed) ||
        !parse_double(fields[fields.size() - 4U], track_deg) ||
        !parse_double(fields[fields.size() - 3U], up_speed) ||
        !parse_double(fields[fields.size() - 2U], vertical_std) ||
        !parse_double(fields[fields.size() - 1U], horizontal_std))
    {
      return std::nullopt;
    }

    const double track_rad = track_deg * M_PI / 180.0;

    ParsedSentence sentence;
    sentence.sentence_type = "BESTNAVA";
    sentence.velocity = VelocityData{};
    sentence.velocity->east_mps = horizontal_speed * std::sin(track_rad);
    sentence.velocity->north_mps = horizontal_speed * std::cos(track_rad);
    sentence.velocity->up_mps = up_speed;
    sentence.velocity->horizontal_std_mps = horizontal_std;
    sentence.velocity->vertical_std_mps = vertical_std;
    return sentence;
  }

  double latitude = 0.0;
  double longitude = 0.0;
  double height_msl = 0.0;
  double undulation = 0.0;
  double latitude_std = 0.0;
  double longitude_std = 0.0;
  double height_std = 0.0;
  double diff_age = 0.0;
  double sol_age = 0.0;
  double velocity_latency = 0.0;
  double velocity_age = 0.0;
  double horizontal_speed = 0.0;
  double track_deg = 0.0;
  double vertical_speed = 0.0;
  double vertical_std = 0.0;
  double horizontal_std = 0.0;
  int tracked_satellites = -1;
  int solution_satellites = -1;
  uint32_t extended_solution_status = 0U;

  if (!parse_double(fields[kBestnavaLatitudeIndex], latitude) ||
      !parse_double(fields[kBestnavaLongitudeIndex], longitude) ||
      !parse_double(fields[kBestnavaHeightIndex], height_msl) ||
      !parse_double(fields[kBestnavaUndulationIndex], undulation) ||
      !parse_double(fields[kBestnavaLatitudeStdIndex], latitude_std) ||
      !parse_double(fields[kBestnavaLongitudeStdIndex], longitude_std) ||
      !parse_double(fields[kBestnavaHeightStdIndex], height_std) ||
      !parse_double(fields[kBestnavaDiffAgeIndex], diff_age) ||
      !parse_double(fields[kBestnavaSolutionAgeIndex], sol_age) ||
      !parse_int(fields[kBestnavaTrackedSvsIndex], tracked_satellites) ||
      !parse_int(fields[kBestnavaSolutionSvsIndex], solution_satellites) ||
      !parse_uint32(fields[kBestnavaExtendedSolutionStatusIndex], 16, extended_solution_status) ||
      !parse_double(fields[kBestnavaVelocityLatencyIndex], velocity_latency) ||
      !parse_double(fields[kBestnavaVelocityAgeIndex], velocity_age) ||
      !parse_double(fields[kBestnavaHorizontalSpeedIndex], horizontal_speed) ||
      !parse_double(fields[kBestnavaTrackGroundIndex], track_deg) ||
      !parse_double(fields[kBestnavaVerticalSpeedIndex], vertical_speed) ||
      !parse_double(fields[kBestnavaVerticalSpeedStdIndex], vertical_std) ||
      !parse_double(fields[kBestnavaHorizontalSpeedStdIndex], horizontal_std))
  {
    return std::nullopt;
  }

  std::string_view position_solution_status = field_after_semicolon(
      fields[kBestnavaPositionSolutionStatusIndex]);
  const std::string_view position_type = fields[kBestnavaPositionTypeIndex];
  const std::string_view velocity_solution_status = fields[kBestnavaVelocitySolutionStatusIndex];
  const std::string_view velocity_type = fields[kBestnavaVelocityTypeIndex];

  uint32_t galileo_bds3_signal_mask = 0U;
  uint32_t gps_glonass_bds2_signal_mask = 0U;
  (void)parse_uint32(fields[kBestnavaGalileoBds3SignalMaskIndex], 16, galileo_bds3_signal_mask);
  (void)parse_uint32(fields[kBestnavaGpsGloBds2SignalMaskIndex], 16, gps_glonass_bds2_signal_mask);

  const double track_rad = track_deg * M_PI / 180.0;

  ParsedSentence sentence;
  sentence.sentence_type = "BESTNAVA";

  sentence.bestnav = BestNavData{};
  sentence.bestnav->solution_status = std::string(position_solution_status);
  sentence.bestnav->position_type = std::string(position_type);
  sentence.bestnav->fix_quality = position_type_to_gga_quality(position_type);
  sentence.bestnav->latitude_deg = latitude;
  sentence.bestnav->longitude_deg = longitude;
  sentence.bestnav->height_msl_m = height_msl;
  sentence.bestnav->undulation_m = undulation;
  sentence.bestnav->latitude_std_m = latitude_std;
  sentence.bestnav->longitude_std_m = longitude_std;
  sentence.bestnav->height_std_m = height_std;
  sentence.bestnav->base_station_id = trim_ascii_quotes(fields[kBestnavaBaseStationIdIndex]);
  sentence.bestnav->diff_age_sec = diff_age;
  sentence.bestnav->sol_age_sec = sol_age;
  sentence.bestnav->satellites_tracked = tracked_satellites;
  sentence.bestnav->satellites_used = solution_satellites;
  sentence.bestnav->extended_solution_status = static_cast<int>(extended_solution_status);
  sentence.bestnav->galileo_bds3_signal_mask =
      static_cast<int>(galileo_bds3_signal_mask);
  sentence.bestnav->gps_glonass_bds2_signal_mask =
      static_cast<int>(gps_glonass_bds2_signal_mask);
  sentence.bestnav->velocity_solution_status = std::string(velocity_solution_status);
  sentence.bestnav->velocity_type = std::string(velocity_type);
  sentence.bestnav->velocity_latency_sec = velocity_latency;
  sentence.bestnav->velocity_age_sec = velocity_age;
  sentence.bestnav->horizontal_speed_mps = horizontal_speed;
  sentence.bestnav->track_over_ground_deg = track_deg;
  sentence.bestnav->vertical_speed_mps = vertical_speed;
  sentence.bestnav->vertical_speed_std_mps = vertical_std;
  sentence.bestnav->horizontal_speed_std_mps = horizontal_std;

  sentence.velocity = VelocityData{};
  sentence.velocity->east_mps = horizontal_speed * std::sin(track_rad);
  sentence.velocity->north_mps = horizontal_speed * std::cos(track_rad);
  sentence.velocity->up_mps = vertical_speed;
  sentence.velocity->horizontal_std_mps = horizontal_std;
  sentence.velocity->vertical_std_mps = vertical_std;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_rtkstatusa(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kRtkstatusaAdrObservationCountIndex)
  {
    return std::nullopt;
  }

  uint32_t gps_source_mask = 0U;
  uint32_t bds_source_mask_1 = 0U;
  uint32_t bds_source_mask_2 = 0U;
  uint32_t glonass_source_mask = 0U;
  uint32_t galileo_source_mask_1 = 0U;
  uint32_t galileo_source_mask_2 = 0U;
  uint32_t qzss_source_mask = 0U;
  int calculate_status = -1;
  int ion_detected = -1;
  int dual_rtk_flag = -1;
  int adr_observation_count = -1;

  if (!parse_uint32(field_after_semicolon(fields[kRtkstatusaGpsSourceIndex]), 16, gps_source_mask) ||
      !parse_uint32(fields[kRtkstatusaBdsSource1Index], 16, bds_source_mask_1) ||
      !parse_uint32(fields[kRtkstatusaBdsSource2Index], 16, bds_source_mask_2) ||
      !parse_uint32(fields[kRtkstatusaGlonassSourceIndex], 16, glonass_source_mask) ||
      !parse_uint32(fields[kRtkstatusaGalileoSource1Index], 16, galileo_source_mask_1) ||
      !parse_uint32(fields[kRtkstatusaGalileoSource2Index], 16, galileo_source_mask_2) ||
      !parse_uint32(fields[kRtkstatusaQzssSourceIndex], 16, qzss_source_mask) ||
      !parse_int(fields[kRtkstatusaCalculateStatusIndex], calculate_status) ||
      !parse_int(fields[kRtkstatusaIonDetectedIndex], ion_detected) ||
      !parse_int(fields[kRtkstatusaDualRtkFlagIndex], dual_rtk_flag) ||
      !parse_int(fields[kRtkstatusaAdrObservationCountIndex], adr_observation_count))
  {
    return std::nullopt;
  }

  const std::string_view position_type = fields[kRtkstatusaPositionTypeIndex];

  ParsedSentence sentence;
  sentence.sentence_type = "RTKSTATUSA";
  sentence.rtk_status = RtkStatusData{};
  sentence.rtk_status->gps_source_mask = gps_source_mask;
  sentence.rtk_status->bds_source_mask_1 = bds_source_mask_1;
  sentence.rtk_status->bds_source_mask_2 = bds_source_mask_2;
  sentence.rtk_status->glonass_source_mask = glonass_source_mask;
  sentence.rtk_status->galileo_source_mask_1 = galileo_source_mask_1;
  sentence.rtk_status->galileo_source_mask_2 = galileo_source_mask_2;
  sentence.rtk_status->qzss_source_mask = qzss_source_mask;
  sentence.rtk_status->position_type = std::string(position_type);
  sentence.rtk_status->fix_quality = position_type_to_gga_quality(position_type);
  sentence.rtk_status->calculate_status = calculate_status;
  sentence.rtk_status->ion_detected = ion_detected;
  sentence.rtk_status->dual_rtk_flag = dual_rtk_flag;
  sentence.rtk_status->adr_observation_count = adr_observation_count;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_rtcmstatusa(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kRtcmstatusaL6CountIndex)
  {
    return std::nullopt;
  }

  int message_id = -1;
  int message_count = -1;
  int base_station_id = -1;
  int satellite_count = -1;
  std::array<int, 6> observable_count{{-1, -1, -1, -1, -1, -1}};

  if (!parse_int(field_after_semicolon(fields[kRtcmstatusaMessageIdIndex]), message_id) ||
      !parse_int(fields[kRtcmstatusaMessageCountIndex], message_count) ||
      !parse_int(fields[kRtcmstatusaBaseStationIdIndex], base_station_id) ||
      !parse_int(fields[kRtcmstatusaSatelliteCountIndex], satellite_count))
  {
    return std::nullopt;
  }

  for (std::size_t i = 0U; i < observable_count.size(); ++i)
  {
    if (!parse_int(fields[kRtcmstatusaL1CountIndex + i], observable_count[i]))
    {
      return std::nullopt;
    }
  }

  ParsedSentence sentence;
  sentence.sentence_type = "RTCMSTATUSA";
  sentence.rtcm_status = RtcmStatusData{};
  sentence.rtcm_status->message_id = message_id;
  sentence.rtcm_status->message_count = message_count;
  sentence.rtcm_status->base_station_id = base_station_id;
  sentence.rtcm_status->satellite_count = satellite_count;
  sentence.rtcm_status->observable_count = observable_count;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_bestsata(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kBestsataEntryCountIndex)
  {
    return std::nullopt;
  }

  int entry_count = -1;
  if (!parse_int(field_after_semicolon(fields[kBestsataEntryCountIndex]), entry_count) ||
      entry_count < 0)
  {
    return std::nullopt;
  }

  const std::size_t required_size =
      kBestsataFirstEntryIndex + static_cast<std::size_t>(entry_count) * kBestsataEntryFieldCount;
  if (fields.size() < required_size)
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "BESTSATA";
  sentence.bestsat = BestSatData{};
  sentence.bestsat->entry_count = entry_count;
  sentence.bestsat->entries.reserve(static_cast<std::size_t>(entry_count));

  std::size_t cursor = kBestsataFirstEntryIndex;
  for (int i = 0; i < entry_count; ++i)
  {
    uint32_t signal_mask = 0U;
    if (!parse_uint32(fields[cursor + 3U], 16, signal_mask))
    {
      return std::nullopt;
    }

    BestSatEntry entry;
    entry.constellation = normalize_constellation_name(fields[cursor]);
    entry.satellite_id = std::string(fields[cursor + 1U]);
    entry.status = std::string(fields[cursor + 2U]);
    entry.signal_mask = static_cast<int>(signal_mask);
    entry.common_view = (signal_mask & 0x10U) != 0U;
    entry.used_signal_bands = bestsat_signal_bands(entry.constellation, signal_mask);
    sentence.bestsat->entries.push_back(entry);
    cursor += kBestsataEntryFieldCount;
  }

  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_satsinfoa(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kSatsinfoaFrequencyFlagIndex)
  {
    return std::nullopt;
  }

  int satellite_count = -1;
  int version = -1;
  int frequency_flag = -1;
  if (!parse_int(field_after_semicolon(fields[kSatsinfoaSatelliteCountIndex]), satellite_count) ||
      !parse_int(fields[kSatsinfoaVersionIndex], version) ||
      !parse_int(fields[kSatsinfoaFrequencyFlagIndex], frequency_flag) ||
      satellite_count < 0)
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "SATSINFOA";
  sentence.satsinfo = SatsInfoData{};
  sentence.satsinfo->satellite_count = satellite_count;
  sentence.satsinfo->version = version;
  sentence.satsinfo->frequency_flag = frequency_flag;
  sentence.satsinfo->entries.reserve(static_cast<std::size_t>(satellite_count));

  std::size_t cursor = kSatsinfoaFirstSatelliteIndex;
  for (int sat_index = 0; sat_index < satellite_count; ++sat_index)
  {
    if (cursor + (kSatsinfoaBaseSatelliteFieldCount - 1U) >= fields.size())
    {
      return std::nullopt;
    }

    int prn = -1;
    int azimuth = -1;
    int elevation = -1;
    int system_id = -1;
    int cn0 = -1;
    int frequency_id = -1;
    int frequency_count = -1;
    if (!parse_int(fields[cursor], prn) ||
        !parse_int(fields[cursor + 1U], azimuth) ||
        !parse_int(fields[cursor + 2U], elevation) ||
        !parse_int(fields[cursor + 3U], system_id) ||
        !parse_int(fields[cursor + 4U], cn0) ||
        !parse_int(fields[cursor + 5U], frequency_id) ||
        !parse_int(fields[cursor + 6U], frequency_count) ||
        frequency_count <= 0)
    {
      return std::nullopt;
    }

    SatsInfoEntry entry;
    entry.constellation = constellation_name_from_system_id(system_id);
    entry.prn = prn;
    entry.azimuth_deg = azimuth;
    entry.elevation_deg = elevation;

    SatsInfoSignal primary_signal;
    primary_signal.constellation = entry.constellation;
    primary_signal.band = signal_band_from_frequency_id(entry.constellation, frequency_id);
    primary_signal.system_id = system_id;
    primary_signal.frequency_id = frequency_id;
    primary_signal.cn0_db_hz = static_cast<double>(cn0);
    entry.signals.push_back(primary_signal);
    cursor += kSatsinfoaBaseSatelliteFieldCount;

    // N4 R1.4 only documents a generic 4-byte "next frequency information"
    // block for ASCII SATSINFOA. The live sample in the manual expands each
    // additional frequency as:
    //   <system_id>,<cn0>,<frequency_id>,<frequency_count>
    // where the last byte repeats the total per-satellite frequency count.
    // We rely on that representation here and keep the repeated count only
    // as a consistency hint, not as a hard requirement.
    for (int freq_index = 1; freq_index < frequency_count; ++freq_index)
    {
      if (cursor + (kSatsinfoaExtraFrequencyFieldCount - 1U) >= fields.size())
      {
        return std::nullopt;
      }

      int extra_system_id = -1;
      int extra_cn0 = -1;
      int extra_frequency_id = -1;
      int repeated_frequency_count = -1;
      if (!parse_int(fields[cursor], extra_system_id) ||
          !parse_int(fields[cursor + 1U], extra_cn0) ||
          !parse_int(fields[cursor + 2U], extra_frequency_id) ||
          !parse_int(fields[cursor + 3U], repeated_frequency_count))
      {
        return std::nullopt;
      }

      SatsInfoSignal extra_signal;
      extra_signal.constellation = constellation_name_from_system_id(extra_system_id);
      extra_signal.band = signal_band_from_frequency_id(extra_signal.constellation,
                                                        extra_frequency_id);
      extra_signal.system_id = extra_system_id;
      extra_signal.frequency_id = extra_frequency_id;
      extra_signal.cn0_db_hz = static_cast<double>(extra_cn0);
      entry.signals.push_back(extra_signal);
      cursor += kSatsinfoaExtraFrequencyFieldCount;
      (void)repeated_frequency_count;
    }

    sentence.satsinfo->entries.push_back(entry);
  }

  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_agca(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kAgcaAnt2L5Index)
  {
    return std::nullopt;
  }

  AgcData agc;
  if (!parse_int(field_after_semicolon(fields[kAgcaAnt1L1Index]), agc.antenna1[0]) ||
      !parse_int(fields[kAgcaAnt1L2Index], agc.antenna1[1]) ||
      !parse_int(fields[kAgcaAnt1L5Index], agc.antenna1[2]) ||
      !parse_int(fields[kAgcaAnt2L1Index], agc.antenna2[0]) ||
      !parse_int(fields[kAgcaAnt2L2Index], agc.antenna2[1]) ||
      !parse_int(fields[kAgcaAnt2L5Index], agc.antenna2[2]))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "AGCA";
  sentence.agc = agc;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_hwstatusa(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kHwstatusaPllLockIndex)
  {
    return std::nullopt;
  }

  double dc09 = 0.0;
  double dc10 = 0.0;
  double dc18 = 0.0;
  int clock_flag = -1;
  double clock_drift = 0.0;
  uint32_t hw_flag = 0U;
  uint32_t pll_lock = 0U;
  if (!parse_double(fields[kHwstatusaDc09Index], dc09) ||
      !parse_double(fields[kHwstatusaDc10Index], dc10) ||
      !parse_double(fields[kHwstatusaDc18Index], dc18) ||
      !parse_int(fields[kHwstatusaClockFlagIndex], clock_flag) ||
      !parse_double(fields[kHwstatusaClockDriftIndex], clock_drift) ||
      !parse_uint32(fields[kHwstatusaHwFlagIndex], 16, hw_flag) ||
      !parse_uint32(fields[kHwstatusaPllLockIndex], 16, pll_lock))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "HWSTATUSA";
  sentence.hw_status = HwStatusData{};
  sentence.hw_status->dc09_v = dc09;
  sentence.hw_status->dc10_v = dc10;
  sentence.hw_status->dc18_v = dc18;
  sentence.hw_status->clock_flag = clock_flag;
  sentence.hw_status->clock_drift_mps = clock_drift;
  sentence.hw_status->hw_flag = static_cast<int>(hw_flag);
  sentence.hw_status->pll_lock = static_cast<int>(pll_lock);
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_jamstatusa(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kJamstatusaCwFlagIndex)
  {
    return std::nullopt;
  }

  int cw_ratio = -1;
  int cw_flag = -1;
  if (!parse_int(fields[kJamstatusaCwRatioIndex], cw_ratio) ||
      !parse_int(fields[kJamstatusaCwFlagIndex], cw_flag))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "JAMSTATUSA";
  sentence.jam_status = JamStatusData{};
  sentence.jam_status->position_type =
      std::string(field_after_semicolon(fields[kJamstatusaPositionTypeIndex]));
  sentence.jam_status->cw_ratio = cw_ratio;
  sentence.jam_status->cw_flag = cw_flag;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_freqjamstatusa(
    const std::vector<std::string_view>& fields)
{
  if (fields.size() <= kFreqjamstatusaL5FlagIndex)
  {
    return std::nullopt;
  }

  FreqJamStatusData freq_jam;
  freq_jam.position_type = std::string(field_after_semicolon(fields[kFreqjamstatusaPositionTypeIndex]));
  if (!parse_int(fields[kFreqjamstatusaL1RatioIndex], freq_jam.cw_ratio[0]) ||
      !parse_int(fields[kFreqjamstatusaL1FlagIndex], freq_jam.cw_flag[0]) ||
      !parse_int(fields[kFreqjamstatusaL2RatioIndex], freq_jam.cw_ratio[1]) ||
      !parse_int(fields[kFreqjamstatusaL2FlagIndex], freq_jam.cw_flag[1]) ||
      !parse_int(fields[kFreqjamstatusaL5RatioIndex], freq_jam.cw_ratio[2]) ||
      !parse_int(fields[kFreqjamstatusaL5FlagIndex], freq_jam.cw_flag[2]))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "FREQJAMSTATUSA";
  sentence.freq_jam_status = freq_jam;
  return sentence;
}

std::optional<ParsedSentence> Um982Parser::parse_gsv(
    std::string_view talker, const std::vector<std::string_view>& fields)
{
  // GSV layout: $<talker>GSV,<total_msgs>,<msg_num>,<sats_in_view>,...
  // The total satellite count for this constellation is field 3, and
  // is repeated identically across every fragment of the burst — only
  // the first message of the burst is needed by the node aggregator.
  if (fields.size() < 4U)
  {
    return std::nullopt;
  }
  int total_in_view = 0;
  if (!parse_int(fields[3], total_in_view))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "GSV";
  sentence.gsv = GsvData{};
  sentence.gsv->talker = std::string(talker);
  sentence.gsv->satellites_in_view = total_in_view;
  return sentence;
}

int Um982Parser::position_type_to_gga_quality(std::string_view text)
{
  // String form from Table 0-4 in the N4 R1.4 manual.
  if (text == "NONE") return 0;
  if (text == "FIXEDPOS" || text == "FIXEDHEIGHT") return 1;
  if (text == "SINGLE") return 1;
  if (text == "PSRDIFF" || text == "DGPS") return 2;
  if (text == "WAAS" || text == "SBAS") return 9;
  if (text == "L1_FLOAT" || text == "IONOFREE_FLOAT" || text == "NARROW_FLOAT" ||
      text == "RTK_FLOAT")
  {
    return 5;
  }
  if (text == "L1_INT" || text == "WIDE_INT" || text == "NARROW_INT" || text == "RTK_FIXED") return 4;
  if (text == "INS") return 1;
  if (text == "INS_PSRSP") return 1;
  if (text == "INS_PSRDIFF") return 2;
  if (text == "INS_RTKFLOAT") return 5;
  if (text == "INS_RTKFIXED") return 4;
  if (text == "PPP_CONVERGING" || text == "PPP") return 1;

  // Numeric form from Table 0-4 in the N4 R1.4 manual.
  int code = 0;
  if (parse_int(text, code))
  {
    switch (code)
    {
      case 0:  return 0;   // NONE
      case 1:
      case 2:  return 1;   // FIXEDPOS / FIXEDHEIGHT
      case 16: return 1;   // SINGLE
      case 17: return 2;   // PSRDIFF
      case 18: return 9;   // WAAS / SBAS
      case 32:
      case 33:
      case 34: return 5;   // L1_FLOAT / IONOFREE_FLOAT / NARROW_FLOAT
      case 48:
      case 49:
      case 50: return 4;   // L1_INT / WIDE_INT / NARROW_INT
      case 52: return 1;   // INS
      case 53: return 1;   // INS_PSRSP
      case 54: return 2;   // INS_PSRDIFF
      case 55: return 5;   // INS_RTKFLOAT
      case 56: return 4;   // INS_RTKFIXED
      case 68:
      case 69: return 1;   // PPP_CONVERGING / PPP
      default: return 0;
    }
  }

  return 0;
}

}  // namespace unicore_gnss
