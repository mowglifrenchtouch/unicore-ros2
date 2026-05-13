#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import binascii
import json
import math
import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass, field
from typing import BinaryIO, Dict, Iterable, List, Optional, Sequence, Tuple

KNOWN_UNICORE_ASCII_TYPES = {
    "AGCA",
    "BESTNAVA",
    "BESTSATA",
    "FREQJAMSTATUSA",
    "HWSTATUSA",
    "JAMSTATUSA",
    "PVTSLNA",
    "RTCMSTATUSA",
    "RTKSTATUSA",
    "SATSINFOA",
    "VERSIONA",
}
KNOWN_NMEA_SUFFIXES = {"GGA", "GSV", "HDT", "HPR"}
KNOWN_NMEA_TYPES = {"GNHPR2", "GPHPR2"}
KNOWN_BINARY_IDS = {
    138: "OBSVMCMPB",
    218: "HWSTATUSB",
    220: "AGCB",
    509: "RTKSTATUSB",
    511: "JAMSTATUSB",
    519: "FREQJAMSTATUSB",
    972: "UNIHEADINGB",
    1021: "PVTSLNB",
    1041: "BESTSATB",
    2118: "BESTNAVB",
    2124: "SATSINFOB",
    2125: "RTCMSTATUSB",
}
PRIMARY_CONSTELLATIONS = ("GPS", "GLO", "GAL", "BDS", "QZSS", "IRNSS", "SBAS")
PRIMARY_SIGNALS = ("L1", "L2", "L3", "L5", "L6", "E1", "E5", "E6", "B1", "B2", "B3")
RTCM_STALE_TIMEOUT_SEC = 5.0
DEFAULT_COMMAND_INTERVAL_SEC = 0.15
DEFAULT_READ_TIMEOUT_SEC = 0.1
DEFAULT_BINARY_MAX_FRAME_SIZE = 4096
UNICORE_BINARY_HEADER_LENGTH = 24
LOW_FREQUENCY_RATIO = 0.70
DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC = 1.0
DEFAULT_REBOOT_WAIT_SEC = 8.0
DEFAULT_POST_CONFIG_SETTLE_SEC = 2.0
DEFAULT_UNICORE_COM_PORT = "COM1"
PROFILE_PERIOD_DEFAULTS = {
    "normal": {"main": 0.2, "bestnav": 0.2, "diagnostic": 1.0, "satellite": 1.0, "rf": 1.0, "raw": 5.0},
    "debug": {"main": 0.2, "bestnav": 0.2, "diagnostic": 1.0, "satellite": 1.0, "rf": 1.0, "raw": 5.0},
    "survey": {"main": 1.0, "bestnav": 1.0, "diagnostic": 1.0, "satellite": 2.0, "rf": 2.0, "raw": 5.0},
    "high_precision": {"main": 0.1, "bestnav": 0.1, "diagnostic": 1.0, "satellite": 1.0, "rf": 1.0, "raw": 5.0},
}
PROFILE_FEATURE_DEFAULTS = {
    "normal": {"satellites": False, "rf": False, "hardware": False, "jamming": False},
    "debug": {"satellites": True, "rf": True, "hardware": True, "jamming": True},
    "survey": {"satellites": True, "rf": True, "hardware": True, "jamming": True},
    "high_precision": {"satellites": True, "rf": True, "hardware": True, "jamming": True},
}
UNSUPPORTED_RESPONSE_HINTS = (
    "unsupported",
    "not support",
    "unknown command",
    "invalid command",
    "command error",
    "parsing failed",
    "grammar error",
    "syntax error",
    "failed",
    "denied",
)
ERROR_RESPONSE_HINTS = (
    "response can't found device",
    "can't found device",
    "device (null)",
)
POSITIVE_RESPONSE_HINTS = ("ok", "ack", "accepted", "success")
MAX_BINARY_CRC_FAILURE_HISTORY = 512

LOG_SYNTAX_NMEA_ONTIME = "nmea_log_ontime"
LOG_SYNTAX_UNICORE_DIRECT_PERIOD = "unicore_direct_period"
LOG_SYNTAX_UNICORE_ONCHANGED = "unicore_onchanged"


@dataclass(frozen=True)
class LogCommandSpec:
    syntax_kind: str
    output_names: Tuple[str, ...]
    track_in_summary: bool = True


LOG_COMMAND_SPECS = {
    "GPGGA": LogCommandSpec(LOG_SYNTAX_NMEA_ONTIME, ("GPGGA",)),
    "PVTSLNA": LogCommandSpec(LOG_SYNTAX_NMEA_ONTIME, ("PVTSLNA",)),
    "PVTSLNB": LogCommandSpec(LOG_SYNTAX_NMEA_ONTIME, ("PVTSLNB",)),
    "BESTNAVA": LogCommandSpec(LOG_SYNTAX_UNICORE_DIRECT_PERIOD, ("BESTNAVA",)),
    "BESTNAVB": LogCommandSpec(LOG_SYNTAX_UNICORE_DIRECT_PERIOD, ("BESTNAVB",)),
    "RTKSTATUSA": LogCommandSpec(LOG_SYNTAX_UNICORE_DIRECT_PERIOD, ("RTKSTATUSA",)),
    "RTKSTATUSB": LogCommandSpec(LOG_SYNTAX_UNICORE_DIRECT_PERIOD, ("RTKSTATUSB",)),
    "RTCMSTATUSA": LogCommandSpec(LOG_SYNTAX_UNICORE_ONCHANGED, ("RTCMSTATUSA",), track_in_summary=False),
    "RTCMSTATUSB": LogCommandSpec(LOG_SYNTAX_UNICORE_ONCHANGED, ("RTCMSTATUSB",), track_in_summary=False),
    "GPHPR": LogCommandSpec(LOG_SYNTAX_UNICORE_DIRECT_PERIOD, ("GNHPR", "GPHPR")),
    "GPHPR2": LogCommandSpec(LOG_SYNTAX_UNICORE_ONCHANGED, ("GNHPR2", "GPHPR2"), track_in_summary=False),
}
PLANNED_LOG_MESSAGES = {
    "GPGGA",
    "PVTSLNA",
    "PVTSLNB",
    "BESTNAVA",
    "BESTNAVB",
    "GPHPR",
    "GPHPR2",
    "RTKSTATUSA",
    "RTKSTATUSB",
    "RTCMSTATUSA",
    "RTCMSTATUSB",
    "BESTSATA",
    "BESTSATB",
    "SATSINFOA",
    "SATSINFOB",
    "GPGSV",
    "GLGSV",
    "GAGSV",
    "GBGSV",
    "AGCA",
    "AGCB",
    "HWSTATUSA",
    "HWSTATUSB",
    "JAMSTATUSA",
    "JAMSTATUSB",
    "FREQJAMSTATUSA",
    "FREQJAMSTATUSB",
    "OBSVMCMPA",
    "OBSVMCMPB",
}


def crc32_unicore(text: bytes) -> int:
    return binascii.crc32(text) & 0xFFFFFFFF


def nmea_checksum(body: bytes) -> int:
    value = 0
    for byte in body:
        value ^= byte
    return value


def format_count_mapping(counts: Dict[str, int]) -> str:
    if not counts:
        return "none"
    return ", ".join(f"{name}={count}" for name, count in sorted(counts.items()))


def sentence_suffix(sentence_type: str) -> str:
    return sentence_type[-3:] if len(sentence_type) > 3 else sentence_type


def field_after_semicolon(text: str) -> str:
    return text.split(";", 1)[1] if ";" in text else text


def normalize_constellation_name(text: str) -> str:
    if text in {"GPS"}:
        return "GPS"
    if text in {"GLONASS", "GLO"}:
        return "GLO"
    if text in {"GALILEO", "GAL"}:
        return "GAL"
    if text in {"BEIDOU", "BDS"}:
        return "BDS"
    if text in {"QZSS"}:
        return "QZSS"
    if text in {"IRNSS", "NAVIC"}:
        return "IRNSS"
    if text in {"SBAS"}:
        return "SBAS"
    return text


def constellation_name_from_system_id(system_id: int) -> str:
    mapping = {
        0: "GPS",
        1: "GLO",
        2: "SBAS",
        3: "GAL",
        4: "BDS",
        5: "QZSS",
        6: "IRNSS",
    }
    return mapping.get(system_id, "UNKNOWN")


def bestsat_constellation_name_from_system_id(system_id: int) -> str:
    mapping = {
        0: "GPS",
        1: "GLO",
        2: "SBAS",
        5: "GAL",
        6: "BDS",
        7: "QZSS",
        9: "IRNSS",
    }
    return mapping.get(system_id, "UNKNOWN")


def bestsat_signal_bands(constellation: str, signal_mask: int) -> List[str]:
    bands: List[str] = []
    if constellation == "GPS":
        if signal_mask & 0x01:
            bands.append("L1")
        if signal_mask & 0x02:
            bands.append("L2")
        if signal_mask & 0x04:
            bands.append("L5")
    elif constellation == "GLO":
        if signal_mask & 0x01:
            bands.append("L1")
        if signal_mask & 0x02:
            bands.append("L2")
        if signal_mask & 0x04:
            bands.append("L3")
    elif constellation == "BDS":
        if signal_mask & 0x01:
            bands.append("B1")
        if signal_mask & 0x02:
            bands.append("B2")
        if signal_mask & 0x04:
            bands.append("B3")
    elif constellation == "GAL":
        if signal_mask & 0x01:
            bands.append("E1")
        if signal_mask & 0x02:
            bands.append("E5")
        if signal_mask & 0x04:
            bands.append("E5")
        if signal_mask & 0x08:
            bands.append("E6")
    return bands


def signal_band_from_frequency_id(constellation: str, frequency_id: int) -> str:
    if constellation in {"GPS", "QZSS"}:
        if frequency_id in {0, 3, 11}:
            return "L1"
        if frequency_id in {9, 17}:
            return "L2"
        if frequency_id in {6, 14}:
            return "L5"
        if constellation == "QZSS" and frequency_id in {18, 22, 24, 25}:
            return "L6"
    elif constellation == "GLO":
        if frequency_id == 0:
            return "L1"
        if frequency_id == 5:
            return "L2"
        if frequency_id in {6, 7}:
            return "L3"
    elif constellation == "GAL":
        if frequency_id in {1, 2}:
            return "E1"
        if frequency_id in {12, 17}:
            return "E5"
        if frequency_id in {18, 22}:
            return "E6"
    elif constellation == "BDS":
        if frequency_id in {0, 4, 8, 23}:
            return "B1"
        if frequency_id in {5, 12, 13, 17, 28}:
            return "B2"
        if frequency_id in {6, 21}:
            return "B3"
    elif constellation == "SBAS":
        if frequency_id == 0:
            return "L1"
        if frequency_id == 6:
            return "L5"
    elif constellation == "IRNSS":
        if frequency_id in {6, 14}:
            return "L5"
    return ""


def position_type_to_quality(value: str) -> int:
    text = value.strip().strip('"')
    string_map = {
        "NONE": 0,
        "FIXEDPOS": 1,
        "FIXEDHEIGHT": 1,
        "SINGLE": 1,
        "PSRDIFF": 2,
        "DGPS": 2,
        "WAAS": 9,
        "SBAS": 9,
        "L1_FLOAT": 5,
        "IONOFREE_FLOAT": 5,
        "NARROW_FLOAT": 5,
        "RTK_FLOAT": 5,
        "L1_INT": 4,
        "WIDE_INT": 4,
        "NARROW_INT": 4,
        "RTK_FIXED": 4,
        "INS": 1,
        "INS_PSRSP": 1,
        "INS_PSRDIFF": 2,
        "INS_RTKFLOAT": 5,
        "INS_RTKFIXED": 4,
        "PPP_CONVERGING": 1,
        "PPP": 1,
    }
    if text in string_map:
        return string_map[text]
    try:
        code = int(text, 10)
    except ValueError:
        return 0
    numeric_map = {
        0: 0,
        1: 1,
        2: 1,
        16: 1,
        17: 2,
        18: 9,
        32: 5,
        33: 5,
        34: 5,
        48: 4,
        49: 4,
        50: 4,
        52: 1,
        53: 1,
        54: 2,
        55: 5,
        56: 4,
        68: 1,
        69: 1,
    }
    return numeric_map.get(code, 0)


def quality_label(quality: int) -> str:
    mapping = {
        0: "no-fix",
        1: "single",
        2: "dgps",
        4: "rtk-fixed",
        5: "rtk-float",
        9: "sbas",
    }
    return mapping.get(quality, f"quality-{quality}")


def normalize_profile(profile: str) -> str:
    return profile if profile in PROFILE_PERIOD_DEFAULTS else "normal"


def normalize_output_format(output_format: str) -> str:
    return output_format if output_format in {"ascii", "binary", "hybrid"} else "ascii"


def output_has_ascii(output_format: str) -> bool:
    return normalize_output_format(output_format) in {"ascii", "hybrid"}


def output_has_binary(output_format: str) -> bool:
    return normalize_output_format(output_format) in {"binary", "hybrid"}


def profile_supports_raw(profile: str) -> bool:
    return normalize_profile(profile) in {"survey", "high_precision"}


def clamp_min_period(value: float, minimum: float) -> float:
    return value if value >= minimum else minimum


def period_to_rate(period: float) -> float:
    return 0.0 if period <= 0 else 1.0 / period


def default_period(profile: str, kind: str) -> float:
    return PROFILE_PERIOD_DEFAULTS[normalize_profile(profile)][kind]


def default_feature_enabled(profile: str, feature: str) -> bool:
    return PROFILE_FEATURE_DEFAULTS[normalize_profile(profile)][feature]


def signalgroup_command_for_model(model: Optional[str]) -> Optional[str]:
    if model == "UM980":
        return "CONFIG SIGNALGROUP 2"
    if model in {"UM981", "UM982"}:
        # Driver-side assumption for the NebulasIV UM98x family.
        return "CONFIG SIGNALGROUP 3 6"
    return None


def log_command_spec_for_message(message: str) -> LogCommandSpec:
    return LOG_COMMAND_SPECS.get(message, LogCommandSpec(LOG_SYNTAX_NMEA_ONTIME, (message,)))


def build_planned_log_command(message: str, period: Optional[float] = None) -> str:
    spec = log_command_spec_for_message(message)
    if spec.syntax_kind == LOG_SYNTAX_UNICORE_ONCHANGED:
        return f"{message} ONCHANGED"
    if period is None:
        raise ValueError(f"period is required for {message}")
    if spec.syntax_kind == LOG_SYNTAX_UNICORE_DIRECT_PERIOD:
        return f"{message} {period:g}"
    return f"LOG {message} ONTIME {period:g}"


def parse_planned_log_command(command: str) -> Optional[Tuple[str, str, Optional[float]]]:
    parts = command.split()
    if len(parts) >= 4 and parts[0] == "LOG" and parts[2] == "ONTIME":
        if parts[1] not in PLANNED_LOG_MESSAGES:
            return None
        try:
            period = float(parts[3])
        except ValueError:
            return None
        return parts[1], LOG_SYNTAX_NMEA_ONTIME, period
    if len(parts) == 2 and parts[1].upper() == "ONCHANGED":
        if parts[0] not in PLANNED_LOG_MESSAGES:
            return None
        return parts[0], LOG_SYNTAX_UNICORE_ONCHANGED, None
    if len(parts) == 2:
        if parts[0] not in PLANNED_LOG_MESSAGES:
            return None
        try:
            period = float(parts[1])
        except ValueError:
            return None
        return parts[0], LOG_SYNTAX_UNICORE_DIRECT_PERIOD, period
    return None


def build_log_command_variants(
    message: str,
    period: Optional[float],
    com_port: str,
    syntax_kind: Optional[str] = None,
) -> List[Tuple[str, str]]:
    variants: List[Tuple[str, str]] = []
    syntax = syntax_kind or log_command_spec_for_message(message).syntax_kind
    rate = period_to_rate(period or 0.0)

    if syntax == LOG_SYNTAX_UNICORE_ONCHANGED:
        variants.append((LOG_SYNTAX_UNICORE_ONCHANGED, f"{message} ONCHANGED"))
        variants.append(("com_onchanged", f"{message} {com_port} ONCHANGED"))
    elif syntax == LOG_SYNTAX_UNICORE_DIRECT_PERIOD:
        if period is None:
            raise ValueError(f"period is required for {message}")
        variants.append((LOG_SYNTAX_UNICORE_DIRECT_PERIOD, f"{message} {period:g}"))
        variants.append(("bare_ontime", f"{message} ONTIME {period:g}"))
        variants.append(("com_period", f"{message} {com_port} {period:g}"))
        if rate > 0:
            variants.append(("com_rate", f"{message} {com_port} {rate:g}"))
        variants.append((LOG_SYNTAX_NMEA_ONTIME, f"LOG {message} ONTIME {period:g}"))
    else:
        if period is None:
            raise ValueError(f"period is required for {message}")
        variants.append((LOG_SYNTAX_NMEA_ONTIME, f"LOG {message} ONTIME {period:g}"))
        variants.append(("bare_ontime", f"{message} ONTIME {period:g}"))
        variants.append(("bare_period", f"{message} {period:g}"))
        variants.append(("com_period", f"{message} {com_port} {period:g}"))
        if rate > 0:
            variants.append(("com_rate", f"{message} {com_port} {rate:g}"))
    deduped: List[Tuple[str, str]] = []
    seen = set()
    for label, command in variants:
        if command in seen:
            continue
        seen.add(command)
        deduped.append((label, command))
    return deduped


def unique_ints(values: Iterable[int]) -> List[int]:
    ordered: List[int] = []
    seen = set()
    for value in values:
        if value in seen:
            continue
        ordered.append(value)
        seen.add(value)
    return ordered


def build_probe_bauds(preferred_baud: Optional[int], target_baud: int, after_factory_reset: bool = False) -> List[int]:
    if after_factory_reset:
        return unique_ints([115200, 460800, 921600, target_baud])
    ordered: List[int] = []
    if preferred_baud is not None:
        ordered.append(preferred_baud)
    ordered.extend([target_baud, 115200, 460800, 921600])
    return unique_ints(ordered)


def extract_version_line(lines: Sequence[str]) -> Optional[str]:
    for line in lines:
        upper = line.upper()
        if "#VERSIONA" in upper or '"UM980"' in upper or '"UM981"' in upper or '"UM982"' in upper:
            return line
    return None


def detect_receiver_model_from_lines(lines: Sequence[str]) -> Optional[str]:
    version_line = extract_version_line(lines)
    if version_line is None:
        return None
    upper = version_line.upper()
    if "UM980" in upper:
        return "UM980"
    if "UM981" in upper:
        return "UM981"
    if "UM982" in upper:
        return "UM982"
    return "unknown"


def classify_command_response(command: str, lines: Sequence[str]) -> str:
    if not lines:
        return "no_response"
    collapsed = " ".join(lines).lower()
    if any(hint in collapsed for hint in ERROR_RESPONSE_HINTS):
        return "error"
    if any(hint in collapsed for hint in UNSUPPORTED_RESPONSE_HINTS):
        return "unsupported"
    if command.upper().startswith("VERSION") and extract_version_line(lines) is not None:
        return "ok"
    if any(hint in collapsed for hint in POSITIVE_RESPONSE_HINTS):
        return "ok"
    return "ok"


def parse_float(text: str) -> Optional[float]:
    if text == "":
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def parse_int(text: str, base: int = 10) -> Optional[int]:
    if text == "":
        return None
    try:
        return int(text, base)
    except ValueError:
        return None


def parse_latlon(value_text: str, hemisphere: str, is_latitude: bool) -> Optional[float]:
    raw = parse_float(value_text)
    if raw is None or len(hemisphere) != 1:
        return None
    divisor = 100.0
    whole = math.floor(raw / divisor)
    minutes = raw - whole * divisor
    degrees = whole + minutes / 60.0
    hemi = hemisphere.upper()
    if hemi in {"S", "W"}:
        degrees = -degrees
    if hemi not in {"N", "S", "E", "W"}:
        return None
    return degrees


def to_fixed(value: Optional[float], digits: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def horizontal_position_delta_m(lhs: Dict[str, float], rhs: Dict[str, float]) -> Optional[float]:
    lat1 = lhs.get("latitude_deg")
    lon1 = lhs.get("longitude_deg")
    lat2 = rhs.get("latitude_deg")
    lon2 = rhs.get("longitude_deg")
    if None in {lat1, lon1, lat2, lon2}:
        return None
    earth_radius = 6378137.0
    deg_to_rad = math.pi / 180.0
    avg_lat_rad = ((lat1 + lat2) * 0.5) * deg_to_rad
    d_lat = (lat2 - lat1) * deg_to_rad
    d_lon = (lon2 - lon1) * deg_to_rad
    north = d_lat * earth_radius
    east = d_lon * earth_radius * math.cos(avg_lat_rad)
    return math.sqrt(north * north + east * east)


def read_bits_le(data: bytes, bit_offset: int, bit_count: int) -> int:
    value = 0
    for bit in range(bit_count):
        absolute = bit_offset + bit
        byte_index = absolute // 8
        bit_index = absolute % 8
        if ((data[byte_index] >> bit_index) & 0x01) != 0:
            value |= 1 << bit
    return value


def sign_extend(value: int, bit_count: int) -> int:
    if bit_count <= 0:
        return value
    sign_bit = 1 << (bit_count - 1)
    mask = (1 << bit_count) - 1
    value &= mask
    return value - (1 << bit_count) if value & sign_bit else value


def obsvmcmp_psr_std(index: int) -> float:
    table = (
        0.050,
        0.075,
        0.113,
        0.169,
        0.253,
        0.380,
        0.570,
        0.854,
        1.281,
        2.375,
        4.750,
        9.500,
        19.000,
        38.000,
        76.000,
        152.000,
    )
    return table[index & 0x0F]


def obsvmcmp_system_id(tracking_status: int) -> int:
    return (tracking_status >> 16) & 0x07


def obsvmcmp_signal_type(tracking_status: int) -> int:
    return (tracking_status >> 21) & 0x1F


def obsvmcmp_l2c_flag(tracking_status: int) -> bool:
    return ((tracking_status >> 26) & 0x01) != 0


def obsvmcmp_carrier_phase_valid(tracking_status: int) -> bool:
    return ((tracking_status >> 19) & 0x01) != 0


def obsvmcmp_pseudorange_valid(tracking_status: int) -> bool:
    return ((tracking_status >> 20) & 0x01) != 0


def obsvmcmp_signal_band(constellation: str, signal_type: int, l2c_flag: bool) -> str:
    _ = l2c_flag
    if constellation in {"GPS", "QZSS"}:
        if signal_type <= 3:
            return "L1"
        if signal_type <= 8:
            return "L2"
        if signal_type <= 15:
            return "L5"
        if constellation == "QZSS":
            return "L6"
    elif constellation == "GLO":
        if signal_type <= 1:
            return "L1"
        if signal_type <= 4:
            return "L2"
        return "L3"
    elif constellation == "GAL":
        if signal_type <= 2:
            return "E1"
        if signal_type <= 5:
            return "E5"
        return "E6"
    elif constellation == "BDS":
        if signal_type <= 2:
            return "B1"
        if signal_type <= 5:
            return "B2"
        return "B3"
    elif constellation == "SBAS":
        return "L1" if signal_type <= 1 else "L5"
    elif constellation == "IRNSS":
        return "L5"
    return ""


@dataclass
class MessageCounter:
    count: int = 0
    first_ts: Optional[float] = None
    last_ts: Optional[float] = None

    def add(self, ts: float) -> None:
        self.count += 1
        if self.first_ts is None:
            self.first_ts = ts
        self.last_ts = ts

    def hz(self, capture_duration_sec: float) -> float:
        if self.count <= 0:
            return 0.0
        if self.first_ts is not None and self.last_ts is not None and self.last_ts > self.first_ts:
            window = self.last_ts - self.first_ts
            return self.count / window if window > 0 else float(self.count)
        return self.count / max(capture_duration_sec, 1e-9)


@dataclass
class BinaryFrame:
    message_id: int
    payload: bytes
    crc_valid: bool
    header_length: int
    payload_length: int
    week: int
    milliseconds: int
    version: int


@dataclass
class BinaryCrcFailure:
    message_id: int
    payload_length: int
    header_hex: str
    expected_crc: int
    computed_crc: int
    crc_offset: int
    frame_length: int


@dataclass
class ProfilePlan:
    log_commands: List[str]
    profile_commands: List[str]
    expected_messages: Dict[str, "ExpectedMessagePlan"]


@dataclass
class ExpectedMessagePlan:
    observed_names: Tuple[str, ...]
    expected_hz: Optional[float]
    required: bool = True


@dataclass
class CommandResult:
    command: str
    baud: int
    status: str
    lines: List[str] = field(default_factory=list)
    logical_message: Optional[str] = None
    syntax_label: Optional[str] = None


@dataclass
class CaptureState:
    total_bytes: int = 0
    nmea_checksum_ok: int = 0
    nmea_checksum_bad: int = 0
    unicore_ascii_crc_ok: int = 0
    unicore_ascii_crc_bad: int = 0
    binary_frames_total: int = 0
    binary_crc_errors: int = 0
    binary_resync_count: int = 0
    binary_sync_candidates: int = 0
    binary_last_header_length: Optional[int] = None
    binary_last_payload_length: Optional[int] = None
    binary_frame_parse_errors_by_reason: Dict[str, int] = field(default_factory=dict)
    binary_crc_failures_recent: List[BinaryCrcFailure] = field(default_factory=list)
    ascii_lines_total: int = 0
    binary_known_but_unparsed: int = 0
    message_counters: Dict[str, MessageCounter] = field(default_factory=dict)
    unknown_ascii_types: Dict[str, int] = field(default_factory=dict)
    unknown_binary_ids: Dict[int, int] = field(default_factory=dict)
    sent_commands: List[str] = field(default_factory=list)
    command_results: List[CommandResult] = field(default_factory=list)
    accepted_log_commands: Dict[str, str] = field(default_factory=dict)
    rejected_log_commands: Dict[str, List[str]] = field(default_factory=dict)
    log_command_syntax_by_message: Dict[str, str] = field(default_factory=dict)
    text_logs: List[str] = field(default_factory=list)
    nav_ascii: Dict[str, Dict[str, float]] = field(default_factory=dict)
    nav_binary: Dict[str, Dict[str, float]] = field(default_factory=dict)
    gsv_counts: Dict[str, int] = field(default_factory=dict)
    latest_rtcm_ascii_ts: Optional[float] = None
    latest_rtcm_binary_ts: Optional[float] = None
    detected_baud: Optional[int] = None
    capture_baud: Optional[int] = None
    detected_model: Optional[str] = None

    def record_message(self, name: str, ts: float) -> None:
        self.message_counters.setdefault(name, MessageCounter()).add(ts)

    def record_unknown_ascii(self, name: str) -> None:
        self.unknown_ascii_types[name] = self.unknown_ascii_types.get(name, 0) + 1

    def record_unknown_binary(self, message_id: int) -> None:
        self.unknown_binary_ids[message_id] = self.unknown_binary_ids.get(message_id, 0) + 1

    def update_ascii_snapshot(self, key: str, values: Dict[str, float]) -> None:
        self.nav_ascii[key] = values

    def update_binary_snapshot(self, key: str, values: Dict[str, float]) -> None:
        self.nav_binary[key] = values


class PosixSerialPort:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.fd: Optional[int] = None
        self._original_attrs: Optional[List[object]] = None

    def open(self) -> None:
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.fd = fd
        attrs = termios.tcgetattr(fd)
        self._original_attrs = attrs[:]
        tty.setraw(fd)
        raw_attrs = termios.tcgetattr(fd)
        speed_name = f"B{self.baud}"
        speed = getattr(termios, speed_name, None)
        if speed is None:
            raise RuntimeError(f"Unsupported baud rate on this platform: {self.baud}")
        raw_attrs[4] = speed
        raw_attrs[5] = speed
        raw_attrs[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(fd, termios.TCSANOW, raw_attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)

    def close(self) -> None:
        if self.fd is None:
            return
        try:
            if self._original_attrs is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self._original_attrs)
        finally:
            os.close(self.fd)
            self.fd = None

    def read(self, timeout_sec: float = DEFAULT_READ_TIMEOUT_SEC) -> bytes:
        if self.fd is None:
            return b""
        ready, _, _ = select.select([self.fd], [], [], timeout_sec)
        if not ready:
            return b""
        try:
            return os.read(self.fd, 4096)
        except (BlockingIOError, OSError):
            return b""

    def write_line(self, command: str) -> None:
        if self.fd is None:
            raise RuntimeError("Serial port is not open")
        data = (command + "\r\n").encode("ascii", errors="ignore")
        os.write(self.fd, data)


class TransportParser:
    def __init__(self, binary_max_frame_size: int) -> None:
        self.buffer = bytearray()
        self.binary_max_frame_size = max(256, binary_max_frame_size)
        self.binary_crc_errors = 0
        self.resync_count = 0
        self.sync_candidates = 0
        self.last_header_length: Optional[int] = None
        self.last_payload_length: Optional[int] = None
        self.frame_parse_errors_by_reason: Dict[str, int] = {}
        self.recent_crc_failures: List[BinaryCrcFailure] = []
        self._counted_sync_candidate = False

    def feed(self, data: bytes, ts: float) -> List[Tuple[str, object]]:
        if data:
            self.buffer.extend(data)
        events: List[Tuple[str, object]] = []

        while self.buffer:
            sync_index = self._find_sync()
            newline_index = self._find_newline()

            if sync_index == 0:
                if not self._counted_sync_candidate:
                    self.sync_candidates += 1
                    self._counted_sync_candidate = True
                frame = self._try_extract_binary()
                if frame is None:
                    break
                if frame is False:
                    continue
                events.append(("binary", (ts, frame)))
                continue

            if newline_index != -1 and (sync_index == -1 or newline_index < sync_index):
                line = bytes(self.buffer[:newline_index]).rstrip(b"\r")
                del self.buffer[:newline_index + 1]
                self._counted_sync_candidate = False
                if line:
                    events.append(("line", (ts, line)))
                continue

            if sync_index > 0:
                del self.buffer[:sync_index]
                self.resync_count += 1
                self._counted_sync_candidate = False
                continue

            if self.buffer and self.buffer[0] in (ord("$"), ord("#")):
                break

            if self._begins_with_binary_sync_prefix():
                break

            del self.buffer[0]
            self.resync_count += 1
            self._counted_sync_candidate = False

        return events

    def _record_parse_reason(self, reason: str) -> None:
        self.frame_parse_errors_by_reason[reason] = self.frame_parse_errors_by_reason.get(reason, 0) + 1

    def _record_crc_failure(self, frame_bytes: bytes, expected_crc: int, actual_crc: int) -> None:
        failure = BinaryCrcFailure(
            message_id=int.from_bytes(frame_bytes[4:6], byteorder="little", signed=False),
            payload_length=int.from_bytes(frame_bytes[6:8], byteorder="little", signed=False),
            header_hex=frame_bytes[:UNICORE_BINARY_HEADER_LENGTH].hex(),
            expected_crc=expected_crc,
            computed_crc=actual_crc,
            crc_offset=len(frame_bytes) - 4,
            frame_length=len(frame_bytes),
        )
        self.recent_crc_failures.append(failure)
        if len(self.recent_crc_failures) > MAX_BINARY_CRC_FAILURE_HISTORY:
            del self.recent_crc_failures[:-MAX_BINARY_CRC_FAILURE_HISTORY]

    def _find_sync(self) -> int:
        pattern = b"\xAA\x44\xB5"
        size = len(self.buffer)
        if size < len(pattern):
            return -1
        for index in range(size - len(pattern) + 1):
            if self.buffer[index:index + 3] == pattern:
                return index
        return -1

    def _find_newline(self) -> int:
        try:
            return self.buffer.index(0x0A)
        except ValueError:
            return -1

    def _begins_with_binary_sync_prefix(self) -> bool:
        prefix = b"\xAA\x44\xB5"
        count = min(len(self.buffer), len(prefix) - 1)
        return count > 0 and self.buffer[:count] == prefix[:count]

    def _try_extract_binary(self) -> Optional[object]:
        if len(self.buffer) < UNICORE_BINARY_HEADER_LENGTH + 4:
            return None
        payload_length = int.from_bytes(self.buffer[6:8], byteorder="little", signed=False)
        frame_size = UNICORE_BINARY_HEADER_LENGTH + payload_length + 4
        if frame_size > self.binary_max_frame_size:
            del self.buffer[0]
            self.resync_count += 1
            self._record_parse_reason("frame_too_large")
            self._counted_sync_candidate = False
            return False
        if len(self.buffer) < frame_size:
            return None
        frame_bytes = bytes(self.buffer[:frame_size])
        expected_crc = int.from_bytes(frame_bytes[-4:], byteorder="little", signed=False)
        actual_crc = crc32_unicore(frame_bytes[:-4])
        crc_ok = expected_crc == actual_crc
        if not crc_ok:
            self.binary_crc_errors += 1
            self._record_crc_failure(frame_bytes, expected_crc, actual_crc)
            del self.buffer[0]
            self.resync_count += 1
            self._record_parse_reason("crc_mismatch")
            self._counted_sync_candidate = False
            return False

        del self.buffer[:frame_size]
        self._counted_sync_candidate = False
        self.last_header_length = UNICORE_BINARY_HEADER_LENGTH
        self.last_payload_length = payload_length
        return BinaryFrame(
            message_id=int.from_bytes(frame_bytes[4:6], byteorder="little", signed=False),
            payload=frame_bytes[UNICORE_BINARY_HEADER_LENGTH:-4],
            crc_valid=True,
            header_length=UNICORE_BINARY_HEADER_LENGTH,
            payload_length=payload_length,
            week=int.from_bytes(frame_bytes[10:12], byteorder="little", signed=False),
            milliseconds=int.from_bytes(frame_bytes[12:16], byteorder="little", signed=False),
            version=int.from_bytes(frame_bytes[16:20], byteorder="little", signed=False),
        )


class LiveValidator:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.state = CaptureState()
        self.text_output: Optional[BinaryIO] = None
        self.raw_output: Optional[BinaryIO] = None
        self.plan = self._build_profile_plan()
        self.serial = PosixSerialPort(args.port, args.baud)
        self.transport = TransportParser(args.binary_max_frame_size)
        self._reported_crc_failures = 0

    def run(self) -> int:
        if self.args.text_output:
            self.text_output = open(self.args.text_output, "wb")
        if self.args.raw_output:
            self.raw_output = open(self.args.raw_output, "wb")

        try:
            self._prepare_receiver()
            capture_start_wall = time.time()
            capture_start_mono = time.monotonic()
            while time.monotonic() - capture_start_mono < self.args.duration:
                chunk = self.serial.read(DEFAULT_READ_TIMEOUT_SEC)
                ts = time.time()
                if chunk:
                    self.state.total_bytes += len(chunk)
                    if self.raw_output is not None:
                        self.raw_output.write(chunk)
                    self._process_bytes(chunk, ts)
            duration_sec = max(time.monotonic() - capture_start_mono, 1e-9)
            summary = self._build_summary(duration_sec, capture_start_wall)
            if self.args.summary:
                with open(self.args.summary, "w", encoding="utf-8") as handle:
                    json.dump(summary, handle, indent=2, sort_keys=True)
                    handle.write("\n")
            self._print_summary(summary)
            conclusion = summary["conclusion"]["level"]
            return 0 if conclusion == "PASS" else (1 if conclusion == "WARN" else 2)
        finally:
            self.serial.close()
            if self.raw_output is not None:
                self.raw_output.close()
            if self.text_output is not None:
                self.text_output.close()

    def _needs_pre_capture_sequence(self) -> bool:
        return any(
            (
                self.args.factory_reset,
                self.args.reset,
                self.args.send_version,
                self.args.discover_log_syntax,
                self.args.apply_profile_config,
                self.args.apply_profile_logs,
                self.args.save_config,
            )
        )

    def _open_serial(self, baud: int) -> None:
        self.serial.close()
        self.serial.baud = baud
        self.serial.open()
        self.state.capture_baud = baud
        self._log_console(f"[live-validate] Opened {self.args.port} @ {baud}")

    def _prepare_receiver(self) -> None:
        if not self._needs_pre_capture_sequence():
            self._open_serial(self.args.baud)
            return

        detected_baud = self._probe_receiver_baud(
            build_probe_bauds(self.args.baud, self.args.baud, after_factory_reset=False)
        )
        if detected_baud is None:
            raise RuntimeError(
                f"Unable to detect a responding Unicore receiver on {self.args.port} before configuration/reset"
            )
        self._open_serial(detected_baud)

        if self.args.factory_reset:
            self._log_console(
                "[live-validate] WARNING: factory reset requested; receiver configuration will be erased and rebooted."
            )
            self._send_command("FRESET", response_timeout_sec=0.4)
            self.serial.close()
            time.sleep(self.args.reboot_wait)
            detected_baud = self._probe_receiver_baud(
                build_probe_bauds(None, self.args.baud, after_factory_reset=True)
            )
            if detected_baud is None:
                raise RuntimeError("Receiver did not respond after FRESET")
            if detected_baud != self.args.baud:
                self._open_serial(detected_baud)
                self._send_command(
                    f"CONFIG {self.args.com_port} {self.args.baud}",
                    response_timeout_sec=DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC,
                )
                self.serial.close()
                time.sleep(0.5)
                self._open_serial(self.args.baud)
                version_lines = self._query_version_lines()
                if extract_version_line(version_lines) is None:
                    reprobed = self._probe_receiver_baud([self.args.baud])
                    if reprobed is None:
                        raise RuntimeError(f"Receiver did not respond at target baud {self.args.baud} after CONFIG COM1")
                    detected_baud = reprobed
                else:
                    detected_baud = self.args.baud
        elif self.args.reset:
            current_baud = detected_baud
            self._send_command("RESET", response_timeout_sec=0.4)
            self.serial.close()
            time.sleep(self.args.reboot_wait)
            detected_baud = self._probe_receiver_baud([current_baud])
            if detected_baud is None:
                self._log_console(
                    f"[live-validate] WARN: receiver did not come back at {current_baud}; probing fallback bauds."
                )
                detected_baud = self._probe_receiver_baud(
                    build_probe_bauds(current_baud, self.args.baud, after_factory_reset=False)
                )
                if detected_baud is None:
                    raise RuntimeError("Receiver did not respond after RESET")

        self._open_serial(detected_baud)
        self.state.capture_baud = detected_baud

        apply_logs = self.args.apply_profile_logs or self.args.discover_log_syntax

        if self.args.send_version or self.args.factory_reset or self.args.reset or self.args.apply_profile_config or apply_logs or self.args.save_config:
            self._query_version()

        should_clear_logs = apply_logs and (
            self.args.unlog_first
            or self.args.factory_reset
            or self.args.reset
            or self.args.discover_log_syntax
            or self.args.apply_profile_config
            or self.args.save_config
        )

        if should_clear_logs:
            self._clear_output_schedule()

        if self.args.apply_profile_config:
            for command in self._build_base_config_commands(self.state.detected_model):
                self._send_command(command)
            for command in self.plan.profile_commands:
                self._send_command(command)

        if apply_logs:
            for command in self.plan.log_commands:
                self._apply_log_command_with_fallback(command)

        if self.args.save_config:
            self._send_command("SAVECONFIG", response_timeout_sec=DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC)

        if any((self.args.apply_profile_config, apply_logs, self.args.factory_reset, self.args.reset, self.args.save_config)):
            self._log_console(
                f"[live-validate] Waiting {DEFAULT_POST_CONFIG_SETTLE_SEC:.1f}s for receiver settle before capture..."
            )
            time.sleep(DEFAULT_POST_CONFIG_SETTLE_SEC)
            self._drain_serial(0.25)

    def _probe_receiver_baud(self, candidates: Sequence[int]) -> Optional[int]:
        for baud in unique_ints(candidates):
            try:
                self._open_serial(baud)
            except Exception as exc:
                self._log_console(f"[live-validate] Probe {baud} failed to open: {exc}")
                continue
            try:
                version_lines = self._query_version_lines()
                version_line = extract_version_line(version_lines)
                if version_line is None:
                    continue
                self.state.detected_baud = baud
                self.state.detected_model = detect_receiver_model_from_lines(version_lines)
                self._log_console(f"[live-validate] Current baud detected: {baud}")
                return baud
            finally:
                self.serial.close()
        return None

    def _drain_serial(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(duration_sec, 0.0)
        while time.monotonic() < deadline:
            chunk = self.serial.read(min(DEFAULT_READ_TIMEOUT_SEC, max(deadline - time.monotonic(), 0.01)))
            if not chunk:
                continue

    def _collect_response_lines(self, timeout_sec: float) -> List[str]:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        line_buffer = bytearray()
        lines: List[str] = []
        last_rx = time.monotonic()
        while time.monotonic() < deadline:
            chunk = self.serial.read(min(DEFAULT_READ_TIMEOUT_SEC, max(deadline - time.monotonic(), 0.01)))
            now = time.monotonic()
            if not chunk:
                if lines and now - last_rx >= 0.12:
                    break
                continue
            last_rx = now
            line_buffer.extend(chunk)
            while True:
                newline_index = line_buffer.find(b"\n")
                if newline_index < 0:
                    break
                raw_line = bytes(line_buffer[:newline_index]).rstrip(b"\r")
                del line_buffer[:newline_index + 1]
                if not raw_line:
                    continue
                text = raw_line.decode("ascii", errors="replace")
                lines.append(text)
                self._write_text_log(time.time(), f"<<< {text}")
        return lines

    def _record_command_result(self, result: CommandResult) -> None:
        self.state.command_results.append(result)
        line_preview = result.lines[0] if result.lines else ""
        suffix = f" | {line_preview}" if line_preview else ""
        self._log_console(
            f"[live-validate] command {result.command} @ {result.baud}: {result.status}{suffix}"
        )

    def _send_command(
        self,
        command: str,
        response_timeout_sec: float = DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC,
        *,
        record: bool = True,
        logical_message: Optional[str] = None,
        syntax_label: Optional[str] = None,
    ) -> CommandResult:
        self._drain_serial(0.10)
        self.serial.write_line(command)
        self.state.sent_commands.append(command)
        self._write_text_log(time.time(), f">>> {command}")
        lines = self._collect_response_lines(response_timeout_sec)
        result = CommandResult(
            command=command,
            baud=self.serial.baud,
            status=classify_command_response(command, lines),
            lines=lines,
            logical_message=logical_message,
            syntax_label=syntax_label,
        )
        if record:
            self._record_command_result(result)
        if self.args.command_interval > 0:
            time.sleep(self.args.command_interval)
        return result

    def _apply_log_command_with_fallback(self, canonical_command: str) -> CommandResult:
        parsed = parse_planned_log_command(canonical_command)
        if parsed is None:
            return self._send_command(canonical_command)

        message, syntax_kind, period = parsed
        rejected: List[str] = []
        last_result: Optional[CommandResult] = None

        for syntax_label, candidate in build_log_command_variants(message, period, self.args.com_port, syntax_kind):
            result = self._send_command(
                candidate,
                response_timeout_sec=DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC,
                record=False,
                logical_message=message,
                syntax_label=syntax_label,
            )
            last_result = result
            if result.status in {"unsupported", "error"}:
                rejected.append(candidate)
                continue

            self.state.accepted_log_commands[message] = candidate
            self.state.log_command_syntax_by_message[message] = syntax_label
            self.state.rejected_log_commands[message] = rejected
            self._record_command_result(result)
            return result

        self.state.rejected_log_commands[message] = rejected
        fallback_result = last_result or CommandResult(
            command=canonical_command,
            baud=self.serial.baud,
            status="unsupported",
            lines=[],
            logical_message=message,
            syntax_label=None,
        )
        self._record_command_result(fallback_result)
        return fallback_result

    def _query_version_lines(self) -> List[str]:
        first = self._send_command("VERSION", response_timeout_sec=DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC)
        lines = list(first.lines)
        if extract_version_line(lines) is None:
            second = self._send_command("VERSIONA", response_timeout_sec=DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC)
            lines.extend(second.lines)
        return lines

    def _query_version(self) -> None:
        lines = self._query_version_lines()
        self.state.detected_model = detect_receiver_model_from_lines(lines) or self.state.detected_model

    def _clear_output_schedule(self) -> None:
        result = self._send_command("UNLOGALL", response_timeout_sec=DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC)
        if result.status in {"unsupported", "no_response", "error"}:
            self._send_command("UNLOG", response_timeout_sec=DEFAULT_COMMAND_RESPONSE_TIMEOUT_SEC)

    def _build_profile_plan(self) -> ProfilePlan:
        log_commands = self._build_log_commands()
        profile_commands = self._build_profile_commands()
        expected_messages: Dict[str, ExpectedMessagePlan] = {}
        for command in log_commands:
            parsed = parse_planned_log_command(command)
            if parsed is None:
                continue
            message, syntax_kind, period = parsed
            spec = log_command_spec_for_message(message)
            if not spec.track_in_summary or not spec.output_names:
                continue
            expected_hz = None
            if syntax_kind != LOG_SYNTAX_UNICORE_ONCHANGED and period is not None:
                expected_hz = 1.0 / period if period > 0 else 0.0
            expected_messages[message] = ExpectedMessagePlan(
                observed_names=spec.output_names,
                expected_hz=expected_hz,
                required=spec.track_in_summary,
            )

        return ProfilePlan(log_commands=log_commands,
                           profile_commands=profile_commands,
                           expected_messages=expected_messages)

    def _build_base_config_commands(self, model: Optional[str]) -> List[str]:
        commands = [
            "MODE ROVER SURVEY MOW",
            "CONFIG NMEAVERSION V411",
            "CONFIG RTK TIMEOUT 10",
            "CONFIG RTK RELIABILITY 3 1",
            "CONFIG DGPS TIMEOUT 600",
            "CONFIG UNDULATION AUTO",
            "CONFIG SBAS DISABLE",
            "CONFIG AGNSS DISABLE",
            "CONFIG PPS DISABLE",
            "MASK 10",
            "UNMASK GPS",
            "UNMASK GLO",
            "UNMASK GAL",
            "UNMASK BDS",
            "MASK QZSS",
            "MASK IRNSS",
        ]
        signalgroup = self.args.signalgroup_override or signalgroup_command_for_model(model)
        if signalgroup is not None:
            commands.insert(6, signalgroup)
        else:
            self._log_console(
                "[live-validate] WARN: receiver model is unknown or unmapped; skipping SIGNALGROUP command."
            )
        return commands

    def _build_profile_commands(self) -> List[str]:
        if normalize_profile(self.args.profile) != "high_precision":
            return []
        return [
            "CONFIG PVTALG MULTI",
            "CONFIG RTCMDECAUTO ENABLE",
            "CONFIG RTCMPHASERATE POSITIVE",
            "CONFIG RTCMCLOCKOFFSET ENABLE",
        ]

    def _build_log_commands(self) -> List[str]:
        profile = normalize_profile(self.args.profile)
        output_format = normalize_output_format(self.args.format)
        periods = PROFILE_PERIOD_DEFAULTS[profile]
        enable_satellites = default_feature_enabled(profile, "satellites")
        enable_rf = default_feature_enabled(profile, "rf")
        enable_hardware = default_feature_enabled(profile, "hardware")
        enable_jamming = default_feature_enabled(profile, "jamming")
        enable_raw = self.args.enable_raw and profile_supports_raw(profile)
        raw_period = clamp_min_period(periods["raw"], 1.0)

        commands: List[str] = []

        def add_ascii(message: str, period: float) -> None:
            if output_has_ascii(output_format):
                commands.append(build_planned_log_command(message, period))

        def add_paired(ascii_message: str, binary_message: str, period: float) -> None:
            if output_has_ascii(output_format):
                commands.append(build_planned_log_command(ascii_message, period))
            if output_has_binary(output_format):
                commands.append(build_planned_log_command(binary_message, period))

        add_ascii("GPGGA", periods["main"])
        add_paired("PVTSLNA", "PVTSLNB", periods["main"])
        add_paired("BESTNAVA", "BESTNAVB", periods["bestnav"])
        add_ascii("GPHPR", periods["main"])
        if output_has_ascii(output_format):
            commands.append(build_planned_log_command("GPHPR2"))
        add_paired("RTKSTATUSA", "RTKSTATUSB", periods["diagnostic"])
        add_paired("RTCMSTATUSA", "RTCMSTATUSB", periods["diagnostic"])

        if enable_satellites:
            add_paired("BESTSATA", "BESTSATB", periods["satellite"])
            add_paired("SATSINFOA", "SATSINFOB", periods["satellite"])
            add_ascii("GPGSV", periods["satellite"])
            add_ascii("GLGSV", periods["satellite"])
            add_ascii("GAGSV", periods["satellite"])
            add_ascii("GBGSV", periods["satellite"])

        if enable_rf:
            add_paired("AGCA", "AGCB", periods["rf"])
        if enable_hardware:
            add_paired("HWSTATUSA", "HWSTATUSB", periods["rf"])
        if enable_jamming:
            add_paired("JAMSTATUSA", "JAMSTATUSB", periods["rf"])
            add_paired("FREQJAMSTATUSA", "FREQJAMSTATUSB", periods["rf"])

        if enable_raw and profile in {"survey", "high_precision"}:
            if output_has_ascii(output_format):
                commands.append(f"LOG OBSVMCMPA ONTIME {raw_period:g}")
            if output_has_binary(output_format):
                commands.append(f"LOG OBSVMCMPB ONTIME {raw_period:g}")

        return commands

    def _process_bytes(self, chunk: bytes, ts: float) -> None:
        events = self.transport.feed(chunk, ts)
        self.state.binary_crc_errors = self.transport.binary_crc_errors
        self.state.binary_resync_count = self.transport.resync_count
        self.state.binary_sync_candidates = self.transport.sync_candidates
        self.state.binary_last_header_length = self.transport.last_header_length
        self.state.binary_last_payload_length = self.transport.last_payload_length
        self.state.binary_frame_parse_errors_by_reason = dict(
            self.transport.frame_parse_errors_by_reason
        )
        self.state.binary_crc_failures_recent = list(self.transport.recent_crc_failures)
        while self._reported_crc_failures < len(self.transport.recent_crc_failures):
            failure = self.transport.recent_crc_failures[self._reported_crc_failures]
            self._write_text_log(
                ts,
                "[BIN-CRC] "
                f"id={failure.message_id} len={failure.payload_length} "
                f"header={failure.header_hex} expected_crc=0x{failure.expected_crc:08x} "
                f"computed_crc=0x{failure.computed_crc:08x} crc_offset={failure.crc_offset} "
                f"frame_len={failure.frame_length}",
            )
            self._reported_crc_failures += 1
        for kind, payload in events:
            if kind == "line":
                line_ts, raw_line = payload
                self._handle_line(line_ts, raw_line)
            else:
                frame_ts, frame = payload
                self._handle_binary_frame(frame_ts, frame)

    def _looks_binary_garbage(self, raw_line: bytes) -> bool:
        if not raw_line:
            return False
        if 0x00 in raw_line:
            return True
        printable = 0
        for byte in raw_line:
            if byte in (0x09, 0x0D, 0x20):
                printable += 1
            elif 0x21 <= byte <= 0x7E:
                printable += 1
        return printable < max(1, int(len(raw_line) * 0.85))

    def _handle_line(self, ts: float, raw_line: bytes) -> None:
        self.state.ascii_lines_total += 1
        if self._looks_binary_garbage(raw_line):
            self._write_text_log(ts, f"[BIN-GARBAGE] {raw_line.hex()}")
            return
        text = raw_line.decode("ascii", errors="replace")
        self._write_text_log(ts, text)
        if text.startswith("$"):
            self._handle_nmea_line(ts, text)
        elif text.startswith("#"):
            self._handle_unicore_ascii_line(ts, text)
        else:
            self.state.record_unknown_ascii(text.split(",", 1)[0])

    def _handle_nmea_line(self, ts: float, line: str) -> None:
        if not self._validate_nmea(line):
            self.state.nmea_checksum_bad += 1
            token = self._line_token(line)
            if token:
                self.state.record_unknown_ascii(token)
            return
        self.state.nmea_checksum_ok += 1
        body = line[1:line.find("*")]
        fields = body.split(",")
        if not fields:
            return
        sentence_type = fields[0]
        self.state.record_message(sentence_type, ts)
        suffix = sentence_suffix(sentence_type)
        if suffix not in KNOWN_NMEA_SUFFIXES and sentence_type not in KNOWN_NMEA_TYPES:
            self.state.record_unknown_ascii(sentence_type)
            return

        if suffix == "GGA" and len(fields) >= 10:
            latitude = parse_latlon(fields[2], fields[3], True)
            longitude = parse_latlon(fields[4], fields[5], False)
            quality = parse_int(fields[6]) or 0
            satellites = parse_int(fields[7]) or -1
            hdop = parse_float(fields[8])
            altitude_msl = parse_float(fields[9])
            geoid = parse_float(fields[11]) if len(fields) > 11 else None
            self.state.update_ascii_snapshot(
                "GGA",
                {
                    "fix_quality": float(quality),
                    "latitude_deg": latitude if latitude is not None else math.nan,
                    "longitude_deg": longitude if longitude is not None else math.nan,
                    "altitude_m": (altitude_msl or 0.0) + (geoid or 0.0),
                    "satellites": float(satellites),
                    "hdop": hdop if hdop is not None else math.nan,
                },
            )
        elif suffix == "GSV" and len(fields) >= 4:
            talker = sentence_type[:2]
            total_in_view = parse_int(fields[3])
            if total_in_view is not None:
                self.state.gsv_counts[talker] = total_in_view

    def _handle_unicore_ascii_line(self, ts: float, line: str) -> None:
        sentence_type = self._line_token(line)
        known_header = sentence_type in KNOWN_UNICORE_ASCII_TYPES
        if sentence_type and known_header:
            self.state.record_message(sentence_type, ts)

        if not self._validate_unicore_ascii(line):
            self.state.unicore_ascii_crc_bad += 1
            if sentence_type and not known_header:
                self.state.record_unknown_ascii(sentence_type)
            return

        self.state.unicore_ascii_crc_ok += 1
        body = line[1:line.find("*")]
        fields = body.split(",")
        if not fields:
            return
        sentence_type = fields[0]
        if sentence_type and not known_header:
            self.state.record_message(sentence_type, ts)
        if sentence_type not in KNOWN_UNICORE_ASCII_TYPES:
            self.state.record_unknown_ascii(sentence_type)
            return

        try:
            if sentence_type == "PVTSLNA":
                snapshot = self._parse_pvtslna(fields)
                if snapshot:
                    self.state.update_ascii_snapshot("PVTSLNA", snapshot)
            elif sentence_type == "BESTNAVA":
                bestnav, velocity = self._parse_bestnava(fields)
                if bestnav:
                    self.state.update_ascii_snapshot("BESTNAVA", bestnav)
                if velocity:
                    self.state.update_ascii_snapshot("BESTNAVA_VELOCITY", velocity)
            elif sentence_type == "BESTSATA":
                snapshot = self._parse_bestsata(fields)
                if snapshot:
                    self.state.update_ascii_snapshot("BESTSATA", snapshot)
            elif sentence_type == "SATSINFOA":
                snapshot = self._parse_satsinfoa(fields)
                if snapshot:
                    self.state.update_ascii_snapshot("SATSINFOA", snapshot)
            elif sentence_type == "RTKSTATUSA":
                snapshot = self._parse_rtkstatusa(fields)
                if snapshot:
                    self.state.update_ascii_snapshot("RTKSTATUSA", snapshot)
            elif sentence_type == "RTCMSTATUSA":
                snapshot = self._parse_rtcmstatusa(fields)
                if snapshot:
                    self.state.latest_rtcm_ascii_ts = ts
                    self.state.update_ascii_snapshot("RTCMSTATUSA", snapshot)
            elif sentence_type == "AGCA":
                snapshot = self._parse_agca(fields)
                if snapshot:
                    self.state.update_ascii_snapshot("AGCA", snapshot)
            elif sentence_type == "HWSTATUSA":
                snapshot = self._parse_hwstatusa(fields)
                if snapshot:
                    self.state.update_ascii_snapshot("HWSTATUSA", snapshot)
            elif sentence_type == "JAMSTATUSA":
                snapshot = self._parse_jamstatusa(fields)
                if snapshot:
                    self.state.update_ascii_snapshot("JAMSTATUSA", snapshot)
            elif sentence_type == "FREQJAMSTATUSA":
                snapshot = self._parse_freqjamstatusa(fields)
                if snapshot:
                    self.state.update_ascii_snapshot("FREQJAMSTATUSA", snapshot)
        except Exception:
            self.state.record_unknown_ascii(sentence_type)

    def _handle_binary_frame(self, ts: float, frame: BinaryFrame) -> None:
        self.state.binary_frames_total += 1
        name = KNOWN_BINARY_IDS.get(frame.message_id, f"ID_{frame.message_id}")
        self.state.record_message(name, ts)
        self._write_text_log(
            ts,
            f"[BIN] id={frame.message_id} name={name} payload={frame.payload_length} crc=ok",
        )
        if frame.message_id not in KNOWN_BINARY_IDS:
            self.state.record_unknown_binary(frame.message_id)
            return

        parsed = False
        if frame.message_id == 1021:
            snapshot = self._parse_pvtslnb(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("PVTSLNB", snapshot)
                parsed = True
        elif frame.message_id == 2118:
            bestnav, velocity = self._parse_bestnavb(frame.payload)
            if bestnav:
                self.state.update_binary_snapshot("BESTNAVB", bestnav)
                parsed = True
            if velocity:
                self.state.update_binary_snapshot("BESTNAVB_VELOCITY", velocity)
        elif frame.message_id == 1041:
            snapshot = self._parse_bestsatb(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("BESTSATB", snapshot)
                parsed = True
        elif frame.message_id == 2124:
            snapshot = self._parse_satsinfob(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("SATSINFOB", snapshot)
                parsed = True
        elif frame.message_id == 2125:
            snapshot = self._parse_rtcmstatusb(frame.payload)
            if snapshot:
                self.state.latest_rtcm_binary_ts = ts
                self.state.update_binary_snapshot("RTCMSTATUSB", snapshot)
                parsed = True
        elif frame.message_id == 509:
            snapshot = self._parse_rtkstatusb(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("RTKSTATUSB", snapshot)
                parsed = True
        elif frame.message_id == 220:
            snapshot = self._parse_agcb(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("AGCB", snapshot)
                parsed = True
        elif frame.message_id == 218:
            snapshot = self._parse_hwstatusb(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("HWSTATUSB", snapshot)
                parsed = True
        elif frame.message_id == 511:
            snapshot = self._parse_jamstatusb(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("JAMSTATUSB", snapshot)
                parsed = True
        elif frame.message_id == 519:
            snapshot = self._parse_freqjamstatusb(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("FREQJAMSTATUSB", snapshot)
                parsed = True
        elif frame.message_id == 138:
            snapshot = self._parse_obsvmcmpb(frame.payload)
            if snapshot:
                self.state.update_binary_snapshot("OBSVMCMPB", snapshot)
                parsed = True
        if not parsed:
            self.state.binary_known_but_unparsed += 1

    def _validate_nmea(self, line: str) -> bool:
        if len(line) < 4 or not line.startswith("$"):
            return False
        star = line.find("*")
        if star == -1 or star + 2 >= len(line):
            return False
        expected = parse_int(line[star + 1:star + 3], 16)
        if expected is None:
            return False
        return nmea_checksum(line[1:star].encode("ascii", errors="ignore")) == expected

    def _validate_unicore_ascii(self, line: str) -> bool:
        if len(line) < 11 or not line.startswith("#"):
            return False
        star = line.find("*")
        if star == -1 or star + 8 >= len(line):
            return False
        expected = parse_int(line[star + 1:star + 9], 16)
        if expected is None:
            return False
        return crc32_unicore(line[1:star].encode("ascii", errors="ignore")) == expected

    def _line_token(self, line: str) -> str:
        if not line:
            return ""
        start = 1 if line[0] in {"$", "#"} else 0
        body = line[start:]
        star = body.find("*")
        if star != -1:
            body = body[:star]
        return body.split(",", 1)[0]

    def _parse_pvtslna(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 39:
            return None
        pos_type = field_after_semicolon(fields[9])
        quality = position_type_to_quality(pos_type)
        altitude_msl = parse_float(fields[10])
        latitude = parse_float(fields[11])
        longitude = parse_float(fields[12])
        altitude_std = parse_float(fields[13])
        latitude_std = parse_float(fields[14])
        longitude_std = parse_float(fields[15])
        undulation = parse_float(fields[21])
        tracked = parse_int(fields[22])
        solution = parse_int(fields[23])
        hdop = parse_float(fields[39])
        if None in {altitude_msl, latitude, longitude, altitude_std, latitude_std, longitude_std}:
            return None
        return {
            "fix_quality": float(quality),
            "latitude_deg": float(latitude),
            "longitude_deg": float(longitude),
            "altitude_m": float(altitude_msl) + (undulation or 0.0),
            "altitude_std_m": float(altitude_std),
            "latitude_std_m": float(latitude_std),
            "longitude_std_m": float(longitude_std),
            "satellites": float(solution if solution is not None and solution >= 0 else (tracked or -1)),
            "hdop": hdop if hdop is not None else math.nan,
            "position_type": pos_type,
        }

    def _parse_bestnava(
        self, fields: Sequence[str]
    ) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, float]]]:
        if len(fields) <= 38:
            return None, None
        solution_status = field_after_semicolon(fields[9])
        position_type = fields[10]
        latitude = parse_float(fields[11])
        longitude = parse_float(fields[12])
        height_msl = parse_float(fields[13])
        undulation = parse_float(fields[14])
        latitude_std = parse_float(fields[16])
        longitude_std = parse_float(fields[17])
        height_std = parse_float(fields[18])
        diff_age = parse_float(fields[20])
        sol_age = parse_float(fields[21])
        tracked = parse_int(fields[22])
        used = parse_int(fields[23])
        ext_status = parse_int(fields[27], 16)
        gal_bds3_mask = parse_int(fields[28], 16)
        gps_glo_bds2_mask = parse_int(fields[29], 16)
        velocity_status = fields[30]
        velocity_type = fields[31]
        velocity_latency = parse_float(fields[32])
        velocity_age = parse_float(fields[33])
        horizontal_speed = parse_float(fields[34])
        track_deg = parse_float(fields[35])
        vertical_speed = parse_float(fields[36])
        vertical_std = parse_float(fields[37])
        horizontal_std = parse_float(fields[38])
        if None in {
            latitude,
            longitude,
            height_msl,
            undulation,
            latitude_std,
            longitude_std,
            height_std,
            diff_age,
            sol_age,
            horizontal_speed,
            track_deg,
            vertical_speed,
            vertical_std,
            horizontal_std,
        }:
            return None, None
        bestnav = {
            "solution_status": solution_status,
            "position_type": position_type,
            "fix_quality": float(position_type_to_quality(position_type)),
            "latitude_deg": float(latitude),
            "longitude_deg": float(longitude),
            "height_msl_m": float(height_msl),
            "undulation_m": float(undulation),
            "altitude_m": float(height_msl) + float(undulation),
            "latitude_std_m": float(latitude_std),
            "longitude_std_m": float(longitude_std),
            "height_std_m": float(height_std),
            "diff_age_sec": float(diff_age),
            "sol_age_sec": float(sol_age),
            "satellites_tracked": float(tracked if tracked is not None else -1),
            "satellites_used": float(used if used is not None else -1),
            "extended_solution_status": float(ext_status if ext_status is not None else -1),
            "galileo_bds3_signal_mask": float(gal_bds3_mask if gal_bds3_mask is not None else -1),
            "gps_glonass_bds2_signal_mask": float(gps_glo_bds2_mask if gps_glo_bds2_mask is not None else -1),
        }
        velocity = {
            "velocity_solution_status": velocity_status,
            "velocity_type": velocity_type,
            "velocity_latency_sec": float(velocity_latency),
            "velocity_age_sec": float(velocity_age),
            "horizontal_speed_mps": float(horizontal_speed),
            "track_over_ground_deg": float(track_deg),
            "vertical_speed_mps": float(vertical_speed),
            "vertical_speed_std_mps": float(vertical_std),
            "horizontal_speed_std_mps": float(horizontal_std),
        }
        return bestnav, velocity

    def _parse_rtkstatusa(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 24:
            return None
        gps_mask = parse_int(field_after_semicolon(fields[9]), 16)
        bds1 = parse_int(fields[11], 16)
        bds2 = parse_int(fields[12], 16)
        glo = parse_int(fields[14], 16)
        gal1 = parse_int(fields[16], 16)
        gal2 = parse_int(fields[17], 16)
        qzss = parse_int(fields[18], 16)
        pos_type = fields[20]
        calc = parse_int(fields[21])
        ion = parse_int(fields[22])
        dual = parse_int(fields[23])
        adr = parse_int(fields[24])
        if None in {gps_mask, bds1, bds2, glo, gal1, gal2, qzss, calc, ion, dual, adr}:
            return None
        return {
            "gps_source_mask": float(gps_mask),
            "bds_source_mask_1": float(bds1),
            "bds_source_mask_2": float(bds2),
            "glonass_source_mask": float(glo),
            "galileo_source_mask_1": float(gal1),
            "galileo_source_mask_2": float(gal2),
            "qzss_source_mask": float(qzss),
            "position_type": pos_type,
            "fix_quality": float(position_type_to_quality(pos_type)),
            "calculate_status": float(calc),
            "ion_detected": float(ion),
            "dual_rtk_flag": float(dual),
            "adr_observation_count": float(adr),
        }

    def _parse_rtcmstatusa(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 18:
            return None
        values = [parse_int(field_after_semicolon(fields[9]))]
        values.extend(parse_int(fields[index]) for index in range(10, 19))
        if any(value is None for value in values):
            return None
        message_id, message_count, base_id, sat_count, l1, l2, l3, l4, l5, l6 = values
        return {
            "message_id": float(message_id),
            "message_count": float(message_count),
            "base_station_id": float(base_id),
            "satellite_count": float(sat_count),
            "l1": float(l1),
            "l2": float(l2),
            "l3": float(l3),
            "l4": float(l4),
            "l5": float(l5),
            "l6": float(l6),
        }

    def _parse_bestsata(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 9:
            return None
        entry_count = parse_int(field_after_semicolon(fields[9]))
        if entry_count is None or entry_count < 0:
            return None
        required = 10 + entry_count * 4
        if len(fields) < required:
            return None
        used_by_constellation = {key: 0 for key in PRIMARY_CONSTELLATIONS}
        signal_count_by_band = {key: 0 for key in PRIMARY_SIGNALS}
        cursor = 10
        for _ in range(entry_count):
            constellation = normalize_constellation_name(fields[cursor])
            signal_mask = parse_int(fields[cursor + 3], 16)
            if signal_mask is None:
                return None
            used_by_constellation[constellation] = used_by_constellation.get(constellation, 0) + 1
            for band in bestsat_signal_bands(constellation, signal_mask):
                signal_count_by_band[band] = signal_count_by_band.get(band, 0) + 1
            cursor += 4
        snapshot: Dict[str, float] = {"entry_count": float(entry_count), "used_total": float(entry_count)}
        snapshot.update({f"used_{key}": float(value) for key, value in used_by_constellation.items()})
        snapshot.update({f"signal_{key}": float(value) for key, value in signal_count_by_band.items()})
        return snapshot

    def _parse_satsinfoa(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 14:
            return None
        satellite_count = parse_int(field_after_semicolon(fields[9]))
        version = parse_int(fields[10])
        frequency_flag = parse_int(fields[14])
        if satellite_count is None or version is None or frequency_flag is None or satellite_count < 0:
            return None
        cursor = 15
        visible_by_constellation = {key: 0 for key in PRIMARY_CONSTELLATIONS}
        signal_count_by_band = {key: 0 for key in PRIMARY_SIGNALS}
        cn0_values: List[float] = []
        for _ in range(satellite_count):
            if cursor + 6 >= len(fields):
                return None
            prn = parse_int(fields[cursor])
            azimuth = parse_int(fields[cursor + 1])
            elevation = parse_int(fields[cursor + 2])
            system_id = parse_int(fields[cursor + 3])
            cn0 = parse_int(fields[cursor + 4])
            frequency_id = parse_int(fields[cursor + 5])
            frequency_count = parse_int(fields[cursor + 6])
            if None in {prn, azimuth, elevation, system_id, cn0, frequency_id, frequency_count}:
                return None
            constellation = constellation_name_from_system_id(system_id)
            visible_by_constellation[constellation] = visible_by_constellation.get(constellation, 0) + 1
            band = signal_band_from_frequency_id(constellation, frequency_id)
            if band:
                signal_count_by_band[band] = signal_count_by_band.get(band, 0) + 1
            if cn0 > 0:
                cn0_values.append(float(cn0))
            cursor += 7
            for _extra in range(1, max(frequency_count, 0)):
                if cursor + 3 >= len(fields):
                    return None
                extra_system_id = parse_int(fields[cursor])
                extra_cn0 = parse_int(fields[cursor + 1])
                extra_frequency_id = parse_int(fields[cursor + 2])
                repeated_count = parse_int(fields[cursor + 3])
                if None in {extra_system_id, extra_cn0, extra_frequency_id, repeated_count}:
                    return None
                extra_constellation = constellation_name_from_system_id(extra_system_id)
                extra_band = signal_band_from_frequency_id(extra_constellation, extra_frequency_id)
                if extra_band:
                    signal_count_by_band[extra_band] = signal_count_by_band.get(extra_band, 0) + 1
                if extra_cn0 > 0:
                    cn0_values.append(float(extra_cn0))
                cursor += 4
        snapshot: Dict[str, float] = {
            "satellite_count": float(satellite_count),
            "version": float(version),
            "frequency_flag": float(frequency_flag),
            "visible_total": float(satellite_count),
            "cn0_mean_db_hz": float(sum(cn0_values) / len(cn0_values)) if cn0_values else math.nan,
            "cn0_max_db_hz": float(max(cn0_values)) if cn0_values else math.nan,
        }
        snapshot.update({f"visible_{key}": float(value) for key, value in visible_by_constellation.items()})
        snapshot.update({f"signal_{key}": float(value) for key, value in signal_count_by_band.items()})
        return snapshot

    def _parse_agca(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 16:
            return None
        values = [
            parse_int(field_after_semicolon(fields[9])),
            parse_int(fields[10]),
            parse_int(fields[11]),
            parse_int(fields[14]),
            parse_int(fields[15]),
            parse_int(fields[16]),
        ]
        if any(value is None for value in values):
            return None
        ant1_l1, ant1_l2, ant1_l5, ant2_l1, ant2_l2, ant2_l5 = values
        valid_main = [value for value in (ant1_l1, ant1_l2, ant1_l5) if value is not None and value >= 0]
        valid_aux = [value for value in (ant2_l1, ant2_l2, ant2_l5) if value is not None and value >= 0]
        return {
            "agc_main_mean": float(sum(valid_main) / len(valid_main)) if valid_main else math.nan,
            "agc_aux_mean": float(sum(valid_aux) / len(valid_aux)) if valid_aux else math.nan,
        }

    def _parse_hwstatusa(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 18:
            return None
        dc09 = parse_float(fields[10])
        dc10 = parse_float(fields[11])
        dc18 = parse_float(fields[12])
        clock_flag = parse_int(fields[13])
        clock_drift = parse_float(fields[14])
        hw_flag = parse_int(fields[16], 16)
        pll_lock = parse_int(fields[18], 16)
        if None in {dc09, dc10, dc18, clock_flag, clock_drift, hw_flag, pll_lock}:
            return None
        return {
            "dc09_v": float(dc09),
            "dc10_v": float(dc10),
            "dc18_v": float(dc18),
            "clock_flag": float(clock_flag),
            "clock_drift_mps": float(clock_drift),
            "hw_flag": float(hw_flag),
            "pll_lock": float(pll_lock),
        }

    def _parse_jamstatusa(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 11:
            return None
        position_type = field_after_semicolon(fields[9])
        cw_ratio = parse_int(fields[10])
        cw_flag = parse_int(fields[11])
        if None in {cw_ratio, cw_flag}:
            return None
        return {
            "position_type": position_type,
            "cw_ratio": float(cw_ratio),
            "cw_flag": float(cw_flag),
        }

    def _parse_freqjamstatusa(self, fields: Sequence[str]) -> Optional[Dict[str, float]]:
        if len(fields) <= 15:
            return None
        position_type = field_after_semicolon(fields[9])
        values = [parse_int(fields[index]) for index in range(10, 16)]
        if any(value is None for value in values):
            return None
        l1_ratio, l1_flag, l2_ratio, l2_flag, l5_ratio, l5_flag = values
        return {
            "position_type": position_type,
            "l1_ratio": float(l1_ratio),
            "l1_flag": float(l1_flag),
            "l2_ratio": float(l2_ratio),
            "l2_flag": float(l2_flag),
            "l5_ratio": float(l5_ratio),
            "l5_flag": float(l5_flag),
        }

    def _parse_bestnavb(
        self, payload: bytes
    ) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, float]]]:
        if len(payload) < 116:
            return None, None
        solution_status_code = int.from_bytes(payload[0:4], "little")
        position_type_code = int.from_bytes(payload[4:8], "little")
        latitude = self._read_f64(payload, 8)
        longitude = self._read_f64(payload, 16)
        height_msl = self._read_f64(payload, 24)
        undulation = self._read_f32(payload, 32)
        latitude_std = self._read_f32(payload, 40)
        longitude_std = self._read_f32(payload, 44)
        height_std = self._read_f32(payload, 48)
        diff_age = self._read_f32(payload, 56)
        sol_age = self._read_f32(payload, 60)
        satellites_tracked = payload[64]
        satellites_used = payload[65]
        ext_status = payload[69]
        gal_bds3_mask = payload[70]
        gps_glo_bds2_mask = payload[71]
        velocity_solution_status = int.from_bytes(payload[72:76], "little")
        velocity_type_code = int.from_bytes(payload[76:80], "little")
        velocity_latency = self._read_f32(payload, 80)
        velocity_age = self._read_f32(payload, 84)
        horizontal_speed = self._read_f64(payload, 88)
        track_deg = self._read_f64(payload, 96)
        vertical_speed = self._read_f32(payload, 104)
        vertical_std = self._read_f32(payload, 108)
        horizontal_std = self._read_f32(payload, 112)
        if None in {
            latitude,
            longitude,
            height_msl,
            undulation,
            latitude_std,
            longitude_std,
            height_std,
            diff_age,
            sol_age,
            velocity_latency,
            velocity_age,
            horizontal_speed,
            track_deg,
            vertical_speed,
            vertical_std,
            horizontal_std,
        }:
            return None, None
        bestnav = {
            "solution_status_code": float(solution_status_code),
            "position_type_code": float(position_type_code),
            "fix_quality": float(self._position_code_to_quality(position_type_code)),
            "latitude_deg": float(latitude),
            "longitude_deg": float(longitude),
            "height_msl_m": float(height_msl),
            "undulation_m": float(undulation),
            "altitude_m": float(height_msl) + float(undulation),
            "latitude_std_m": float(latitude_std),
            "longitude_std_m": float(longitude_std),
            "height_std_m": float(height_std),
            "diff_age_sec": float(diff_age),
            "sol_age_sec": float(sol_age),
            "satellites_tracked": float(satellites_tracked),
            "satellites_used": float(satellites_used),
            "extended_solution_status": float(ext_status),
            "galileo_bds3_signal_mask": float(gal_bds3_mask),
            "gps_glonass_bds2_signal_mask": float(gps_glo_bds2_mask),
        }
        velocity = {
            "velocity_solution_status_code": float(velocity_solution_status),
            "velocity_type_code": float(velocity_type_code),
            "velocity_latency_sec": float(velocity_latency),
            "velocity_age_sec": float(velocity_age),
            "horizontal_speed_mps": float(horizontal_speed),
            "track_over_ground_deg": float(track_deg),
            "vertical_speed_mps": float(vertical_speed),
            "vertical_speed_std_mps": float(vertical_std),
            "horizontal_speed_std_mps": float(horizontal_std),
        }
        return bestnav, velocity

    def _parse_pvtslnb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 140:
            return None
        bestpos_type_code = int.from_bytes(payload[0:4], "little")
        bestpos_height_msl = self._read_f32(payload, 4)
        latitude = self._read_f64(payload, 8)
        longitude = self._read_f64(payload, 16)
        altitude_std = self._read_f32(payload, 24)
        latitude_std = self._read_f32(payload, 28)
        longitude_std = self._read_f32(payload, 32)
        undulation = self._read_f32(payload, 64)
        bestpos_svs = payload[68]
        bestpos_solnsvs = payload[69]
        hdop = self._read_f32(payload, 124)
        if None in {bestpos_height_msl, latitude, longitude, altitude_std, latitude_std, longitude_std, undulation, hdop}:
            return None
        return {
            "bestpos_type_code": float(bestpos_type_code),
            "fix_quality": float(self._position_code_to_quality(bestpos_type_code)),
            "latitude_deg": float(latitude),
            "longitude_deg": float(longitude),
            "altitude_m": float(bestpos_height_msl) + float(undulation),
            "altitude_std_m": float(altitude_std),
            "latitude_std_m": float(latitude_std),
            "longitude_std_m": float(longitude_std),
            "undulation_m": float(undulation),
            "satellites": float(bestpos_solnsvs if bestpos_solnsvs > 0 else bestpos_svs),
            "hdop": float(hdop),
        }

    def _parse_bestsatb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 4:
            return None
        entry_count = int.from_bytes(payload[0:4], "little")
        required = 4 + entry_count * 16
        if len(payload) < required:
            return None
        used_by_constellation = {key: 0 for key in PRIMARY_CONSTELLATIONS}
        signal_count_by_band = {key: 0 for key in PRIMARY_SIGNALS}
        cursor = 4
        for _ in range(entry_count):
            system_id = int.from_bytes(payload[cursor:cursor + 4], "little")
            signal_mask = int.from_bytes(payload[cursor + 12:cursor + 16], "little")
            constellation = bestsat_constellation_name_from_system_id(system_id)
            used_by_constellation[constellation] = used_by_constellation.get(constellation, 0) + 1
            for band in bestsat_signal_bands(constellation, signal_mask):
                signal_count_by_band[band] = signal_count_by_band.get(band, 0) + 1
            cursor += 16
        snapshot: Dict[str, float] = {"entry_count": float(entry_count), "used_total": float(entry_count)}
        snapshot.update({f"used_{key}": float(value) for key, value in used_by_constellation.items()})
        snapshot.update({f"signal_{key}": float(value) for key, value in signal_count_by_band.items()})
        return snapshot

    def _parse_satsinfob(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 6:
            return None
        satellite_count = payload[0]
        version = payload[1]
        frequency_flag = payload[5]
        cursor = 6
        visible_by_constellation = {key: 0 for key in PRIMARY_CONSTELLATIONS}
        signal_count_by_band = {key: 0 for key in PRIMARY_SIGNALS}
        cn0_values: List[float] = []
        for _ in range(satellite_count):
            if cursor + 7 >= len(payload):
                return None
            prn = payload[cursor]
            azimuth = int.from_bytes(payload[cursor + 1:cursor + 3], "little", signed=True)
            elevation = payload[cursor + 3]
            _ = prn, azimuth, elevation
            system_id = payload[cursor + 4]
            cn0 = payload[cursor + 5]
            frequency_id = payload[cursor + 6]
            frequency_count = payload[cursor + 7]
            if frequency_count == 0:
                return None
            constellation = constellation_name_from_system_id(system_id)
            visible_by_constellation[constellation] = visible_by_constellation.get(constellation, 0) + 1
            band = signal_band_from_frequency_id(constellation, frequency_id)
            if band:
                signal_count_by_band[band] = signal_count_by_band.get(band, 0) + 1
            if cn0 > 0:
                cn0_values.append(float(cn0))
            record_size = 4 + frequency_count * 4
            if cursor + record_size > len(payload):
                return None
            for freq_index in range(frequency_count):
                offset = cursor + 4 + freq_index * 4
                signal_system_id = payload[offset]
                signal_cn0 = payload[offset + 1]
                signal_frequency_id = payload[offset + 2]
                signal_constellation = constellation_name_from_system_id(signal_system_id)
                extra_band = signal_band_from_frequency_id(signal_constellation, signal_frequency_id)
                if extra_band:
                    signal_count_by_band[extra_band] = signal_count_by_band.get(extra_band, 0) + 1
                if signal_cn0 > 0:
                    cn0_values.append(float(signal_cn0))
            cursor += record_size
        snapshot: Dict[str, float] = {
            "satellite_count": float(satellite_count),
            "version": float(version),
            "frequency_flag": float(frequency_flag),
            "visible_total": float(satellite_count),
            "cn0_mean_db_hz": float(sum(cn0_values) / len(cn0_values)) if cn0_values else math.nan,
            "cn0_max_db_hz": float(max(cn0_values)) if cn0_values else math.nan,
        }
        snapshot.update({f"visible_{key}": float(value) for key, value in visible_by_constellation.items()})
        snapshot.update({f"signal_{key}": float(value) for key, value in signal_count_by_band.items()})
        return snapshot

    def _parse_rtcmstatusb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 22:
            return None
        return {
            "message_id": float(int.from_bytes(payload[0:4], "little")),
            "message_count": float(int.from_bytes(payload[4:8], "little")),
            "base_station_id": float(int.from_bytes(payload[8:12], "little")),
            "satellite_count": float(int.from_bytes(payload[12:16], "little")),
            "l1": float(payload[16]),
            "l2": float(payload[17]),
            "l3": float(payload[18]),
            "l4": float(payload[19]),
            "l5": float(payload[20]),
            "l6": float(payload[21]),
        }

    def _parse_rtkstatusb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 56:
            return None
        position_type_code = int.from_bytes(payload[44:48], "little")
        calculate_status = int.from_bytes(payload[48:52], "little")
        ion_detected = payload[52]
        dual_rtk_flag = payload[53]
        adr_observation_count = payload[54]
        return {
            "gps_source_mask": float(int.from_bytes(payload[0:4], "little")),
            "bds_source_mask_1": float(int.from_bytes(payload[8:12], "little")),
            "bds_source_mask_2": float(int.from_bytes(payload[12:16], "little")),
            "glonass_source_mask": float(int.from_bytes(payload[20:24], "little")),
            "galileo_source_mask_1": float(int.from_bytes(payload[28:32], "little")),
            "galileo_source_mask_2": float(int.from_bytes(payload[32:36], "little")),
            "qzss_source_mask": float(int.from_bytes(payload[36:40], "little")),
            "position_type_code": float(position_type_code),
            "fix_quality": float(self._position_code_to_quality(position_type_code)),
            "calculate_status": float(calculate_status),
            "ion_detected": float(ion_detected),
            "dual_rtk_flag": float(dual_rtk_flag),
            "adr_observation_count": float(adr_observation_count),
        }

    def _parse_agcb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 20:
            return None
        ant1 = [self._read_i16(payload, offset) for offset in (0, 2, 4)]
        ant2 = [self._read_i16(payload, offset) for offset in (10, 12, 14)]
        if any(value is None for value in ant1 + ant2):
            return None
        valid_main = [value for value in ant1 if value is not None and value >= 0]
        valid_aux = [value for value in ant2 if value is not None and value >= 0]
        return {
            "agc_main_mean": float(sum(valid_main) / len(valid_main)) if valid_main else math.nan,
            "agc_aux_mean": float(sum(valid_aux) / len(valid_aux)) if valid_aux else math.nan,
        }

    def _parse_hwstatusb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 40:
            return None
        dc09 = self._read_f32(payload, 4)
        dc10 = self._read_f32(payload, 8)
        dc18 = self._read_f32(payload, 12)
        clock_flag = int.from_bytes(payload[16:20], "little")
        clock_drift = self._read_f32(payload, 20)
        hw_flag = payload[28]
        pll_lock = int.from_bytes(payload[30:32], "little")
        if None in {dc09, dc10, dc18, clock_drift}:
            return None
        return {
            "dc09_v": float(dc09),
            "dc10_v": float(dc10),
            "dc18_v": float(dc18),
            "clock_flag": float(clock_flag),
            "clock_drift_mps": float(clock_drift),
            "hw_flag": float(hw_flag),
            "pll_lock": float(pll_lock),
        }

    def _parse_jamstatusb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 8:
            return None
        position_type_code = int.from_bytes(payload[0:4], "little")
        return {
            "position_type_code": float(position_type_code),
            "fix_quality": float(self._position_code_to_quality(position_type_code)),
            "cw_ratio": float(payload[4]),
            "cw_flag": float(payload[5]),
        }

    def _parse_freqjamstatusb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 12:
            return None
        position_type_code = int.from_bytes(payload[0:4], "little")
        return {
            "position_type_code": float(position_type_code),
            "l1_ratio": float(payload[4]),
            "l1_flag": float(payload[5]),
            "l2_ratio": float(payload[6]),
            "l2_flag": float(payload[7]),
            "l5_ratio": float(payload[8]),
            "l5_flag": float(payload[9]),
        }

    def _parse_obsvmcmpb(self, payload: bytes) -> Optional[Dict[str, float]]:
        if len(payload) < 4:
            return None
        observation_count = int.from_bytes(payload[0:4], "little")
        required = 4 + observation_count * 24
        if len(payload) < required:
            return None
        raw_by_constellation = {key: 0 for key in PRIMARY_CONSTELLATIONS}
        raw_by_signal = {key: 0 for key in PRIMARY_SIGNALS}
        cn0_values: List[float] = []
        cursor = 4
        for _ in range(observation_count):
            record = payload[cursor:cursor + 24]
            tracking_status = read_bits_le(record, 0, 32)
            system_id = obsvmcmp_system_id(tracking_status)
            constellation = constellation_name_from_system_id(system_id)
            signal_type = obsvmcmp_signal_type(tracking_status)
            signal_band = obsvmcmp_signal_band(constellation, signal_type, obsvmcmp_l2c_flag(tracking_status))
            raw_by_constellation[constellation] = raw_by_constellation.get(constellation, 0) + 1
            if signal_band:
                raw_by_signal[signal_band] = raw_by_signal.get(signal_band, 0) + 1
            cn0_code = read_bits_le(record, 165, 5)
            if cn0_code > 0:
                cn0_values.append(20.0 + float(cn0_code))
            cursor += 24
        snapshot: Dict[str, float] = {
            "observation_count": float(observation_count),
            "raw_cn0_mean_db_hz": float(sum(cn0_values) / len(cn0_values)) if cn0_values else math.nan,
            "raw_cn0_max_db_hz": float(max(cn0_values)) if cn0_values else math.nan,
        }
        snapshot.update({f"raw_constellation_{key}": float(value) for key, value in raw_by_constellation.items()})
        snapshot.update({f"raw_signal_{key}": float(value) for key, value in raw_by_signal.items()})
        return snapshot

    def _position_code_to_quality(self, code: int) -> int:
        return {
            0: 0,
            1: 1,
            2: 1,
            16: 1,
            17: 2,
            18: 9,
            32: 5,
            33: 5,
            34: 5,
            48: 4,
            49: 4,
            50: 4,
            52: 1,
            53: 1,
            54: 2,
            55: 5,
            56: 4,
            68: 1,
            69: 1,
        }.get(code, 0)

    def _read_i16(self, payload: bytes, offset: int) -> Optional[int]:
        if offset + 2 > len(payload):
            return None
        return int.from_bytes(payload[offset:offset + 2], "little", signed=True)

    def _read_f32(self, payload: bytes, offset: int) -> Optional[float]:
        if offset + 4 > len(payload):
            return None
        return self._unpack_float(payload[offset:offset + 4], 32)

    def _read_f64(self, payload: bytes, offset: int) -> Optional[float]:
        if offset + 8 > len(payload):
            return None
        return self._unpack_float(payload[offset:offset + 8], 64)

    def _unpack_float(self, data: bytes, bits: int) -> Optional[float]:
        import struct

        fmt = "<f" if bits == 32 else "<d"
        try:
            value = struct.unpack(fmt, data)[0]
        except struct.error:
            return None
        return value if math.isfinite(value) else None

    def _build_summary(self, duration_sec: float, capture_start_wall: float) -> Dict[str, object]:
        warnings: List[str] = []
        failures: List[str] = []
        expected = self.plan.expected_messages
        message_frequencies: Dict[str, Dict[str, object]] = {}
        discovery_only = self.args.discover_log_syntax and self.args.duration <= 0.0

        for name, expected_spec in sorted(expected.items()):
            observed_count = 0
            observed_hz = 0.0
            for observed_name in expected_spec.observed_names:
                observed = self.state.message_counters.get(observed_name, MessageCounter())
                observed_count += observed.count
                observed_hz += observed.hz(duration_sec)
            message_frequencies[name] = {
                "count": observed_count,
                "expected_hz": round(expected_spec.expected_hz, 3) if expected_spec.expected_hz is not None else None,
                "observed_hz": round(observed_hz, 3),
                "observed_names": list(expected_spec.observed_names),
            }
            if discovery_only:
                continue
            if expected_spec.required and observed_count == 0:
                if name in {"PVTSLNA", "PVTSLNB", "BESTNAVA", "BESTNAVB"}:
                    failures.append(f"Expected nav log {name} not observed")
                else:
                    warnings.append(f"Expected log {name} not observed")
            elif expected_spec.expected_hz is not None and expected_spec.expected_hz > 0 and observed_hz < expected_spec.expected_hz * LOW_FREQUENCY_RATIO:
                warnings.append(
                    f"Log {name} is slower than requested ({observed_hz:.2f} Hz < {expected_spec.expected_hz:.2f} Hz)"
                )

        if self.state.total_bytes == 0 and not discovery_only:
            failures.append("No bytes captured from the serial port")

        if self.args.format in {"hybrid", "binary"} and self.state.binary_frames_total == 0 and not discovery_only:
            failures.append(f"Format {self.args.format} requested but no Unicore binary frames were observed")
            if self.state.binary_sync_candidates > 0:
                warnings.append(
                    "Binary sync candidates were observed but no complete frame passed CRC validation"
                )

        if self.args.format == "binary" and self.state.message_counters.get("PVTSLNB", MessageCounter()).count == 0 and not discovery_only:
            failures.append("Binary-only run did not observe PVTSLNB")

        if self.state.binary_crc_errors > 0:
            warnings.append(f"Binary CRC errors observed: {self.state.binary_crc_errors}")
        if self.state.nmea_checksum_bad > 0:
            warnings.append(f"NMEA checksum errors observed: {self.state.nmea_checksum_bad}")
        if self.state.unicore_ascii_crc_bad > 0:
            warnings.append(f"Unicore ASCII CRC errors observed: {self.state.unicore_ascii_crc_bad}")
        if self.state.binary_resync_count > 0:
            warnings.append(f"Binary transport resynchronizations observed: {self.state.binary_resync_count}")
        if self.state.binary_known_but_unparsed > 0:
            warnings.append(
                f"Known binary frames seen but not fully parsed: {self.state.binary_known_but_unparsed}"
            )
        for item in self.state.command_results:
            if item.logical_message and item.logical_message in self.state.accepted_log_commands:
                continue
            if item.status == "unsupported":
                warnings.append(f"Command {item.command} was reported unsupported at {item.baud} baud")
            elif item.status == "error":
                warnings.append(f"Command {item.command} returned an explicit device error at {item.baud} baud")
            elif item.status == "no_response" and item.command not in {
                "RESET",
                "FRESET",
                "VERSION",
                "VERSIONA",
                "UNLOG",
                "UNLOGALL",
            }:
                warnings.append(f"Command {item.command} returned no explicit response at {item.baud} baud")
        for message, rejected in sorted(self.state.rejected_log_commands.items()):
            if message in self.state.accepted_log_commands:
                continue
            if rejected:
                warnings.append(
                    f"No accepted LOG syntax found for {message}; attempted: {', '.join(rejected)}"
                )
        if self.state.unknown_binary_ids:
            warnings.append(
                "Unknown binary message IDs: "
                + ", ".join(f"{message_id}x{count}" for message_id, count in sorted(self.state.unknown_binary_ids.items()))
            )
        if self.state.unknown_ascii_types:
            filtered_unknowns = {
                key: value for key, value in self.state.unknown_ascii_types.items() if key not in {"VERSIONA"}
            }
            if filtered_unknowns:
                warnings.append(
                    "Unknown ASCII/NMEA logs: "
                    + ", ".join(f"{name}x{count}" for name, count in sorted(filtered_unknowns.items()))
                )

        nav_ascii = self._preferred_ascii_nav()
        nav_binary = self._preferred_binary_nav()
        fix_quality = None
        if nav_binary and self.args.format == "binary":
            fix_quality = int(nav_binary.get("fix_quality", 0))
        elif nav_ascii:
            fix_quality = int(nav_ascii.get("fix_quality", 0))
        elif nav_binary:
            fix_quality = int(nav_binary.get("fix_quality", 0))
        if (fix_quality is None or fix_quality <= 0) and not discovery_only:
            failures.append("No valid GNSS fix detected in the captured streams")

        ascii_visible, ascii_used, ascii_cn0_mean, ascii_cn0_max, ascii_diff_age = self._extract_state_metrics(
            self.state.nav_ascii
        )
        binary_visible, binary_used, binary_cn0_mean, binary_cn0_max, binary_diff_age = self._extract_state_metrics(
            self.state.nav_binary
        )

        hybrid_comparison = {
            "position_delta_m": horizontal_position_delta_m(nav_ascii or {}, nav_binary or {})
            if nav_ascii and nav_binary
            else None,
            "altitude_delta_m": (
                abs((nav_binary or {}).get("altitude_m", math.nan) - (nav_ascii or {}).get("altitude_m", math.nan))
                if nav_ascii and nav_binary
                else None
            ),
            "fix_type_match": (
                int((nav_ascii or {}).get("fix_quality", -1)) == int((nav_binary or {}).get("fix_quality", -2))
                if nav_ascii and nav_binary
                else None
            ),
            "visible_satellites_delta": (
                binary_visible - ascii_visible if ascii_visible is not None and binary_visible is not None else None
            ),
            "used_satellites_delta": (
                binary_used - ascii_used if ascii_used is not None and binary_used is not None else None
            ),
            "cn0_mean_delta_db_hz": (
                binary_cn0_mean - ascii_cn0_mean
                if ascii_cn0_mean is not None and binary_cn0_mean is not None
                else None
            ),
            "diff_age_delta_sec": (
                binary_diff_age - ascii_diff_age
                if ascii_diff_age is not None and binary_diff_age is not None
                else None
            ),
        }
        if self.args.format == "hybrid" and not discovery_only:
            position_delta = hybrid_comparison["position_delta_m"]
            if position_delta is not None and position_delta > 1.0:
                warnings.append(f"Hybrid ASCII/binary position delta is high: {position_delta:.3f} m")
            if hybrid_comparison["fix_type_match"] is False:
                warnings.append("Hybrid ASCII/binary fix types do not match")

        rtcm_age = None
        if self.args.format == "binary":
            rtcm_age = time.time() - self.state.latest_rtcm_binary_ts if self.state.latest_rtcm_binary_ts else None
        elif self.args.format == "hybrid":
            ages = []
            if self.state.latest_rtcm_ascii_ts:
                ages.append(time.time() - self.state.latest_rtcm_ascii_ts)
            if self.state.latest_rtcm_binary_ts:
                ages.append(time.time() - self.state.latest_rtcm_binary_ts)
            rtcm_age = min(ages) if ages else None
        else:
            rtcm_age = time.time() - self.state.latest_rtcm_ascii_ts if self.state.latest_rtcm_ascii_ts else None
        rtcm_alive = rtcm_age is not None and rtcm_age <= RTCM_STALE_TIMEOUT_SEC
        if any(name.startswith("RTCMSTATUS") for name in expected) and not rtcm_alive and not discovery_only:
            warnings.append("RTCM status stream is stale or missing")

        summary = {
            "config": {
                "port": self.args.port,
                "baud": self.args.baud,
                "com_port": self.args.com_port,
                "signalgroup_override": self.args.signalgroup_override,
                "detected_baud": self.state.detected_baud,
                "capture_baud": self.state.capture_baud,
                "detected_model": self.state.detected_model,
                "duration_sec": self.args.duration,
                "profile": self.args.profile,
                "format": self.args.format,
                "enable_raw": self.args.enable_raw,
                "send_version": self.args.send_version,
                "factory_reset": self.args.factory_reset,
                "reset": self.args.reset,
                "save_config": self.args.save_config,
                "apply_profile_config": self.args.apply_profile_config,
                "apply_profile_logs": self.args.apply_profile_logs,
                "unlog_first": self.args.unlog_first,
                "discover_log_syntax": self.args.discover_log_syntax,
                "reboot_wait_sec": self.args.reboot_wait,
                "capture_started_unix_sec": capture_start_wall,
                "effective_raw_expected": any(
                    command.startswith("LOG OBSVMCMP")
                    for command in self.plan.log_commands
                ),
                "discovery_only": discovery_only,
            },
            "capture": {
                "total_bytes": self.state.total_bytes,
                "ascii_lines_total": self.state.ascii_lines_total,
                "nmea_checksum_ok": self.state.nmea_checksum_ok,
                "nmea_checksum_bad": self.state.nmea_checksum_bad,
                "unicore_ascii_crc_ok": self.state.unicore_ascii_crc_ok,
                "unicore_ascii_crc_bad": self.state.unicore_ascii_crc_bad,
                "binary_frames_total": self.state.binary_frames_total,
                "binary_crc_errors": self.state.binary_crc_errors,
                "binary_resync_count": self.state.binary_resync_count,
                "binary_sync_candidates": self.state.binary_sync_candidates,
                "binary_header_len": self.state.binary_last_header_length,
                "binary_payload_len": self.state.binary_last_payload_length,
                "binary_frame_parse_errors_by_reason": self.state.binary_frame_parse_errors_by_reason,
                "binary_crc_failures_recent": [
                    {
                        "message_id": item.message_id,
                        "payload_length": item.payload_length,
                        "header_hex": item.header_hex,
                        "expected_crc": item.expected_crc,
                        "computed_crc": item.computed_crc,
                        "crc_offset": item.crc_offset,
                        "frame_length": item.frame_length,
                    }
                    for item in self.state.binary_crc_failures_recent
                ],
                "binary_known_but_unparsed": self.state.binary_known_but_unparsed,
            },
            "commands": {
                "sent": self.state.sent_commands,
                "planned_logs": self.plan.log_commands,
                "planned_profile_commands": self.plan.profile_commands,
                "accepted_log_commands": self.state.accepted_log_commands,
                "rejected_log_commands": self.state.rejected_log_commands,
                "log_command_syntax_by_message": self.state.log_command_syntax_by_message,
                "results": [
                    {
                        "command": item.command,
                        "baud": item.baud,
                        "status": item.status,
                        "lines": item.lines,
                        "logical_message": item.logical_message,
                        "syntax_label": item.syntax_label,
                    }
                    for item in self.state.command_results
                ],
            },
            "expected_messages": message_frequencies,
            "unknown_ascii_logs": self.state.unknown_ascii_types,
            "unknown_binary_ids": self.state.unknown_binary_ids,
            "state": {
                "fix_quality": fix_quality,
                "fix_label": quality_label(fix_quality or 0),
                "ascii_visible_satellites": ascii_visible,
                "ascii_used_satellites": ascii_used,
                "ascii_cn0_mean_db_hz": ascii_cn0_mean,
                "ascii_cn0_max_db_hz": ascii_cn0_max,
                "ascii_diff_age_sec": ascii_diff_age,
                "binary_visible_satellites": binary_visible,
                "binary_used_satellites": binary_used,
                "binary_cn0_mean_db_hz": binary_cn0_mean,
                "binary_cn0_max_db_hz": binary_cn0_max,
                "binary_diff_age_sec": binary_diff_age,
                "rtcm_alive": rtcm_alive,
                "rtcm_age_sec": rtcm_age,
            },
            "hybrid_comparison": hybrid_comparison,
            "warnings": warnings,
            "failures": failures,
            "conclusion": {
                "level": "FAIL" if failures else ("WARN" if warnings else "PASS"),
                "warnings_count": len(warnings),
                "failures_count": len(failures),
            },
        }
        return summary

    def _preferred_ascii_nav(self) -> Optional[Dict[str, float]]:
        for key in ("PVTSLNA", "BESTNAVA", "GGA"):
            snapshot = self.state.nav_ascii.get(key)
            if snapshot and int(snapshot.get("fix_quality", 0)) > 0:
                return snapshot
        return self.state.nav_ascii.get("BESTNAVA") or self.state.nav_ascii.get("PVTSLNA") or self.state.nav_ascii.get("GGA")

    def _preferred_binary_nav(self) -> Optional[Dict[str, float]]:
        for key in ("PVTSLNB", "BESTNAVB"):
            snapshot = self.state.nav_binary.get(key)
            if snapshot and int(snapshot.get("fix_quality", 0)) > 0:
                return snapshot
        return self.state.nav_binary.get("BESTNAVB") or self.state.nav_binary.get("PVTSLNB")

    def _extract_state_metrics(
        self, snapshots: Dict[str, Dict[str, float]]
    ) -> Tuple[Optional[int], Optional[int], Optional[float], Optional[float], Optional[float]]:
        satsinfo = snapshots.get("SATSINFOA") or snapshots.get("SATSINFOB")
        bestsat = snapshots.get("BESTSATA") or snapshots.get("BESTSATB")
        bestnav = snapshots.get("BESTNAVA") or snapshots.get("BESTNAVB")
        visible = int(satsinfo["visible_total"]) if satsinfo and "visible_total" in satsinfo else None
        used = None
        if bestsat and "used_total" in bestsat:
            used = int(bestsat["used_total"])
        elif bestnav and "satellites_used" in bestnav:
            used = int(bestnav["satellites_used"])
        cn0_mean = (
            float(satsinfo["cn0_mean_db_hz"])
            if satsinfo and "cn0_mean_db_hz" in satsinfo and math.isfinite(satsinfo["cn0_mean_db_hz"])
            else None
        )
        cn0_max = (
            float(satsinfo["cn0_max_db_hz"])
            if satsinfo and "cn0_max_db_hz" in satsinfo and math.isfinite(satsinfo["cn0_max_db_hz"])
            else None
        )
        diff_age = (
            float(bestnav["diff_age_sec"])
            if bestnav and "diff_age_sec" in bestnav and math.isfinite(bestnav["diff_age_sec"])
            else None
        )
        return visible, used, cn0_mean, cn0_max, diff_age

    def _print_summary(self, summary: Dict[str, object]) -> None:
        state = summary["state"]
        print()
        print("Unicore Live Validation")
        print(
            f"Port: {summary['config']['port']} requested={summary['config']['baud']} baud "
            f"detected={summary['config']['detected_baud'] or 'n/a'} "
            f"capture={summary['config']['capture_baud'] or 'n/a'}"
        )
        print(
            f"Profile: {summary['config']['profile']}  Format: {summary['config']['format']}  "
            f"Duration: {summary['config']['duration_sec']} s"
        )
        if summary["config"]["factory_reset"] or summary["config"]["reset"] or summary["config"]["save_config"]:
            print(
                f"Control: factory_reset={summary['config']['factory_reset']}  "
                f"reset={summary['config']['reset']}  save_config={summary['config']['save_config']}"
            )
        print(
            f"Capture: {summary['capture']['total_bytes']} B, "
            f"{summary['capture']['ascii_lines_total']} ASCII lines, "
            f"{summary['capture']['binary_frames_total']} binary frames"
        )
        print(
            f"Checks: NMEA bad={summary['capture']['nmea_checksum_bad']}  "
            f"ASCII CRC bad={summary['capture']['unicore_ascii_crc_bad']}  "
            f"binary CRC bad={summary['capture']['binary_crc_errors']}  "
            f"resync={summary['capture']['binary_resync_count']}"
        )
        print(
            f"Binary framing: sync={summary['capture']['binary_sync_candidates']}  "
            f"header={summary['capture']['binary_header_len'] or 'n/a'}  "
            f"payload={summary['capture']['binary_payload_len'] or 'n/a'}  "
            f"errors={format_count_mapping(summary['capture']['binary_frame_parse_errors_by_reason'])}"
        )
        if summary["capture"]["binary_crc_failures_recent"]:
            print("Recent binary CRC failures")
            failures_to_show = summary["capture"]["binary_crc_failures_recent"]
            preview = failures_to_show[:3]
            if len(failures_to_show) > 6:
                preview += [{"ellipsis": True}]
            if len(failures_to_show) > 3:
                preview += failures_to_show[-3:]
            for item in preview:
                if item.get("ellipsis"):
                    print("  ...")
                    continue
                print(
                    f"  id={item['message_id']} len={item['payload_length']} "
                    f"crc_expected=0x{item['expected_crc']:08x} "
                    f"crc_computed=0x{item['computed_crc']:08x} "
                    f"crc_offset={item['crc_offset']} frame_len={item['frame_length']} "
                    f"header={item['header_hex']}"
                )
        print(
            f"Fix: {state['fix_label']}  visible={state['ascii_visible_satellites'] or state['binary_visible_satellites'] or 'n/a'}  "
            f"used={state['ascii_used_satellites'] or state['binary_used_satellites'] or 'n/a'}  "
            f"diff_age={to_fixed(state['ascii_diff_age_sec'] or state['binary_diff_age_sec'])} s"
        )
        print(
            f"CN0: ascii mean/max={to_fixed(state['ascii_cn0_mean_db_hz'])}/{to_fixed(state['ascii_cn0_max_db_hz'])}  "
            f"binary mean/max={to_fixed(state['binary_cn0_mean_db_hz'])}/{to_fixed(state['binary_cn0_max_db_hz'])}"
        )
        print(
            f"RTCM: {'alive' if state['rtcm_alive'] else 'stale'}  age={to_fixed(state['rtcm_age_sec'])} s"
        )
        if summary["commands"]["results"]:
            print()
            print("Command responses")
            for item in summary["commands"]["results"]:
                preview = item["lines"][0] if item["lines"] else ""
                suffix = f" | {preview}" if preview else ""
                print(
                    f"  @{item['baud']:6d} {item['command']:<24s} {item['status']}{suffix}"
                )
        if summary["commands"]["accepted_log_commands"] or summary["commands"]["rejected_log_commands"]:
            print()
            print("Log syntax")
            for message in sorted(summary["commands"]["accepted_log_commands"].keys()):
                print(
                    f"  {message:14s} accepted={summary['commands']['accepted_log_commands'][message]} "
                    f"via={summary['commands']['log_command_syntax_by_message'].get(message, 'n/a')}"
                )
            for message, rejected in sorted(summary["commands"]["rejected_log_commands"].items()):
                if message in summary["commands"]["accepted_log_commands"]:
                    continue
                print(
                    f"  {message:14s} rejected={', '.join(rejected) if rejected else 'none'}"
                )
        print()
        print("Expected messages")
        for name, item in summary["expected_messages"].items():
            expected_hz = item["expected_hz"]
            expected_text = f"{expected_hz:6.2f} Hz" if expected_hz is not None else "onchange"
            print(
                f"  {name:14s} count={item['count']:3d}  observed={item['observed_hz']:6.2f} Hz  "
                f"expected={expected_text}"
            )
        if summary["config"]["format"] == "hybrid":
            hybrid = summary["hybrid_comparison"]
            print()
            print(
                "Hybrid comparison: "
                f"pos_delta={to_fixed(hybrid['position_delta_m'])} m  "
                f"alt_delta={to_fixed(hybrid['altitude_delta_m'])} m  "
                f"visible_delta={hybrid['visible_satellites_delta'] if hybrid['visible_satellites_delta'] is not None else 'n/a'}  "
                f"used_delta={hybrid['used_satellites_delta'] if hybrid['used_satellites_delta'] is not None else 'n/a'}  "
                f"cn0_delta={to_fixed(hybrid['cn0_mean_delta_db_hz'])} dB-Hz"
            )
        if summary["unknown_binary_ids"]:
            print()
            print(
                "Unknown binary IDs: "
                + ", ".join(f"{message_id}x{count}" for message_id, count in sorted(summary["unknown_binary_ids"].items()))
            )
        if summary["unknown_ascii_logs"]:
            filtered = {
                key: value
                for key, value in summary["unknown_ascii_logs"].items()
                if key not in {"VERSIONA"}
            }
            if filtered:
                print()
                print(
                    "Unknown ASCII logs: "
                    + ", ".join(f"{name}x{count}" for name, count in sorted(filtered.items()))
                )
        if summary["warnings"]:
            print()
            print("Warnings")
            for warning in summary["warnings"]:
                print(f"  - {warning}")
        if summary["failures"]:
            print()
            print("Failures")
            for failure in summary["failures"]:
                print(f"  - {failure}")
        print()
        print(f"Conclusion: {summary['conclusion']['level']}")

    def _write_text_log(self, ts: float, text: str) -> None:
        if self.text_output is None:
            return
        line = f"{ts:.6f} {text}\n".encode("utf-8", errors="replace")
        self.text_output.write(line)

    def _log_console(self, text: str) -> None:
        print(text, file=sys.stderr)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture and validate a live Unicore N4 serial stream without ROS 2."
    )
    parser.add_argument("--port", required=True, help="Serial device path, e.g. /dev/gps or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--com-port", default=DEFAULT_UNICORE_COM_PORT, help="Receiver logical COM port for CONFIG <port> <baud>")
    parser.add_argument("--duration", type=float, default=30.0, help="Capture duration in seconds")
    parser.add_argument(
        "--profile",
        choices=("normal", "debug", "survey", "high_precision"),
        default="normal",
        help="Expected runtime profile",
    )
    parser.add_argument(
        "--format",
        choices=("ascii", "hybrid", "binary"),
        default="ascii",
        help="Expected receiver output format",
    )
    parser.add_argument(
        "--enable-raw",
        action="store_true",
        help="Expect and validate raw observations (effective only in survey/high_precision)",
    )
    parser.add_argument(
        "--factory-reset",
        action="store_true",
        help="Send FRESET before validation, reprobe default bauds, then reopen at the target baud",
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Send RESET before validation, wait for reboot, then reopen the receiver",
    )
    parser.add_argument(
        "--save-config",
        action="store_true",
        help="Send SAVECONFIG explicitly after configuration/log scheduling",
    )
    parser.add_argument(
        "--signalgroup-override",
        help="Optional SIGNALGROUP command override, e.g. 'CONFIG SIGNALGROUP 3 6'",
    )
    parser.add_argument("--send-version", action="store_true", help="Send VERSION during the pre-capture setup sequence")
    parser.add_argument(
        "--apply-profile-config",
        action="store_true",
        help="Send the built-in UM98x profile CONFIG sequence before capture",
    )
    parser.add_argument(
        "--apply-profile-logs",
        action="store_true",
        help="Send the built-in UM98x profile LOG schedule before capture",
    )
    parser.add_argument(
        "--discover-log-syntax",
        action="store_true",
        help="Probe alternate LOG syntaxes for each requested message before capture",
    )
    parser.add_argument(
        "--unlog-first",
        action="store_true",
        help="Send UNLOG before profile logs when --apply-profile-logs is used",
    )
    parser.add_argument("--command-interval", type=float, default=DEFAULT_COMMAND_INTERVAL_SEC)
    parser.add_argument(
        "--reboot-wait",
        type=float,
        default=DEFAULT_REBOOT_WAIT_SEC,
        help="Seconds to wait for reboot after RESET/FRESET before reprobe",
    )
    parser.add_argument("--binary-max-frame-size", type=int, default=DEFAULT_BINARY_MAX_FRAME_SIZE)
    parser.add_argument("--raw-output", help="Optional path to store the raw byte capture")
    parser.add_argument("--text-output", help="Optional path to store decoded text/frame logs")
    parser.add_argument("--summary", help="Optional path to write the JSON summary")
    args = parser.parse_args(argv)
    if args.factory_reset and args.reset:
        parser.error("--factory-reset and --reset are mutually exclusive")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    validator = LiveValidator(args)
    return validator.run()


if __name__ == "__main__":
    raise SystemExit(main())
