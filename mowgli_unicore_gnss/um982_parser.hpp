// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mowgli_unicore_gnss
{

enum class FixSource : uint8_t
{
  kGga,
  kPvtslna,
  kPvtslnb,
};

enum class HeadingSource : uint8_t
{
  kHdt,
  kHpr,
  kPvtslnb,
};

struct FixData
{
  FixSource source{FixSource::kGga};
  bool valid_fix{false};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  int fix_quality{0};
  int satellites{-1};
  double hdop{-1.0};
  bool has_covariance{false};
  std::array<double, 9> covariance{};
};

struct HeadingData
{
  HeadingSource source{HeadingSource::kHdt};
  double heading_deg{0.0};
  std::optional<double> pitch_deg;
  std::optional<double> roll_deg;
  std::optional<double> baseline_m;
  double variance_deg2{0.0};
};

struct VelocityData
{
  double east_mps{0.0};
  double north_mps{0.0};
  double up_mps{0.0};
  double horizontal_std_mps{0.0};
  double vertical_std_mps{0.0};
};

struct BestNavData
{
  std::string solution_status;
  std::string position_type;
  int fix_quality{0};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double height_msl_m{0.0};
  double undulation_m{0.0};
  double latitude_std_m{-1.0};
  double longitude_std_m{-1.0};
  double height_std_m{-1.0};
  std::string base_station_id;
  double diff_age_sec{-1.0};
  double sol_age_sec{-1.0};
  int satellites_tracked{-1};
  int satellites_used{-1};
  int extended_solution_status{-1};
  int galileo_bds3_signal_mask{-1};
  int gps_glonass_bds2_signal_mask{-1};
  std::string velocity_solution_status;
  std::string velocity_type;
  double velocity_latency_sec{-1.0};
  double velocity_age_sec{-1.0};
  double horizontal_speed_mps{0.0};
  double track_over_ground_deg{0.0};
  double vertical_speed_mps{0.0};
  double vertical_speed_std_mps{-1.0};
  double horizontal_speed_std_mps{-1.0};
};

struct RtkStatusData
{
  uint32_t gps_source_mask{0U};
  uint32_t bds_source_mask_1{0U};
  uint32_t bds_source_mask_2{0U};
  uint32_t glonass_source_mask{0U};
  uint32_t galileo_source_mask_1{0U};
  uint32_t galileo_source_mask_2{0U};
  uint32_t qzss_source_mask{0U};
  std::string position_type;
  int fix_quality{0};
  int calculate_status{-1};
  int ion_detected{-1};
  int dual_rtk_flag{-1};
  int adr_observation_count{-1};
};

struct RtcmStatusData
{
  int message_id{-1};
  int message_count{-1};
  int base_station_id{-1};
  int satellite_count{-1};
  std::array<int, 6> observable_count{{-1, -1, -1, -1, -1, -1}};
};

struct BestSatEntry
{
  std::string constellation;
  std::string satellite_id;
  std::string status;
  int signal_mask{-1};
  bool common_view{false};
  std::vector<std::string> used_signal_bands;
};

struct BestSatData
{
  int entry_count{-1};
  std::vector<BestSatEntry> entries;
};

struct SatsInfoSignal
{
  std::string constellation;
  std::string band;
  int system_id{-1};
  int frequency_id{-1};
  double cn0_db_hz{-1.0};
};

struct SatsInfoEntry
{
  std::string constellation;
  int prn{-1};
  int azimuth_deg{-1};
  int elevation_deg{-1};
  std::vector<SatsInfoSignal> signals;
};

struct SatsInfoData
{
  int satellite_count{-1};
  int version{-1};
  int frequency_flag{-1};
  std::vector<SatsInfoEntry> entries;
};

struct RawObservationEntry
{
  std::string constellation;
  std::string satellite_id;
  std::string signal_band;
  int system_id{-1};
  int signal_type{-1};
  int glonass_frequency_channel{-1};
  uint32_t tracking_status{0U};
  bool pseudorange_valid{false};
  bool carrier_phase_valid{false};
  double pseudorange_m{-1.0};
  double carrier_phase_cycles{-1.0};
  double doppler_hz{-1.0};
  double cn0_db_hz{-1.0};
  double lock_time_sec{-1.0};
  double pseudorange_std_m{-1.0};
  double carrier_phase_std_cycles{-1.0};
};

struct RawObservationData
{
  int observation_count{-1};
  std::vector<RawObservationEntry> entries;
};

struct AgcData
{
  // L1/L2/L5 AGC register values for the master antenna.
  std::array<int, 3> antenna1{{-1, -1, -1}};
  // L1/L2/L5 AGC register values for the slave antenna when present.
  std::array<int, 3> antenna2{{-1, -1, -1}};
};

struct HwStatusData
{
  double dc09_v{-1.0};
  double dc10_v{-1.0};
  double dc18_v{-1.0};
  int clock_flag{-1};
  double clock_drift_mps{-1.0};
  int hw_flag{-1};
  int pll_lock{-1};
};

struct JamStatusData
{
  std::string position_type;
  int cw_ratio{-1};
  int cw_flag{-1};
};

struct FreqJamStatusData
{
  std::string position_type;
  // L1/L2/L5 frequency-family jamming status from FREQJAMSTATUSA.
  std::array<int, 3> cw_ratio{{-1, -1, -1}};
  std::array<int, 3> cw_flag{{-1, -1, -1}};
};

struct GsvData
{
  // Two-letter NMEA talker prefix carried verbatim from the GSV
  // sentence ("GP", "GL", "GA", "GB", "GQ", "GI", "GN"). Used as a
  // constellation key in the node's per-constellation tally.
  std::string talker;
  int satellites_in_view{0};
};

struct ParserCounters
{
  std::size_t parsed_sentences{0U};
  std::size_t parse_errors{0U};
  std::size_t nmea_checksum_errors{0U};
  std::size_t unicore_crc_errors{0U};
};

struct ParsedSentence
{
  std::string sentence_type;
  std::optional<FixData> fix;
  std::optional<HeadingData> heading;
  std::optional<VelocityData> velocity;
  std::optional<BestNavData> bestnav;
  std::optional<RtkStatusData> rtk_status;
  std::optional<RtcmStatusData> rtcm_status;
  std::optional<BestSatData> bestsat;
  std::optional<SatsInfoData> satsinfo;
  std::optional<RawObservationData> raw_observations;
  std::optional<AgcData> agc;
  std::optional<HwStatusData> hw_status;
  std::optional<JamStatusData> jam_status;
  std::optional<FreqJamStatusData> freq_jam_status;
  std::optional<GsvData> gsv;
};

class Um982Parser
{
public:
  std::optional<ParsedSentence> parse_line(const std::string& line) const;
  ParserCounters counters() const;

private:
  static bool validate_nmea_checksum(std::string_view line);
  static bool validate_unicore_crc(std::string_view line);
  static std::vector<std::string_view> split_fields(std::string_view payload);
  static std::string trim(std::string_view text);
  static bool parse_double(std::string_view field, double& value);
  static bool parse_int(std::string_view field, int& value);
  static bool parse_uint32(std::string_view field, int base, uint32_t& value);
  static bool parse_latlon(std::string_view value_field,
                           std::string_view hemi_field,
                           bool is_latitude,
                           double& degrees);

  static std::optional<ParsedSentence> parse_gga(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_hdt(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_hpr(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_pvtslna(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_bestnava(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_rtkstatusa(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_rtcmstatusa(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_bestsata(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_satsinfoa(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_agca(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_hwstatusa(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_jamstatusa(const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_freqjamstatusa(
      const std::vector<std::string_view>& fields);
  static std::optional<ParsedSentence> parse_gsv(std::string_view talker,
                                                 const std::vector<std::string_view>& fields);

  // Map a Unicore BESTPOSA-style position-type field (string like
  // "NARROW_INT" or numeric code like "50") to the equivalent NMEA GGA
  // quality value (0-9). Returns 0 when the input is empty or unknown,
  // matching NMEA quality 0 = no fix.
  static int position_type_to_gga_quality(std::string_view text);

  mutable ParserCounters counters_{};
};

}  // namespace mowgli_unicore_gnss
