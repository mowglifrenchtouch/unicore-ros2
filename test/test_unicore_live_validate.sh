#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."

python3 - <<'PY'
from pathlib import Path

for relative in ("tools/unicore_live_validate.py", "tools/um982_live_validate.py", "tools/unicore_crc_probe.py"):
    path = Path(relative)
    compile(path.read_text(encoding="utf-8"), str(path), "exec")
PY

help_output="$(python3 tools/unicore_live_validate.py --help)"
[[ "$help_output" == *"--port"* ]]
[[ "$help_output" == *"--format {ascii,hybrid,binary}"* ]]
[[ "$help_output" == *"--factory-reset"* ]]
[[ "$help_output" == *"--reset"* ]]
[[ "$help_output" == *"--save-config"* ]]
[[ "$help_output" == *"--com-port"* ]]
[[ "$help_output" == *"--signalgroup-override"* ]]
[[ "$help_output" == *"--discover-log-syntax"* ]]
[[ "$help_output" == *"--enable-ggah"* ]]

legacy_help_output="$(python3 tools/um982_live_validate.py --help)"
[[ "$legacy_help_output" == *"--port"* ]]

crc_probe_output="$(python3 tools/unicore_crc_probe.py --hex aa44b5614608780000a07209c82ba4120000000000120c00000000001000000008cc66e025fa45401052859c789e01400000b0c5dd2b65406aa349423d000000325ac03f62b68f3fdb101040300000000000000000005442211c1c0004121151000000000800000000000000000000009c6414e86d43803f8ef677e58cd36340c1c1bfecd94386bfe13edb3c8758ab3c961ec655)"
[[ "$crc_probe_output" == *"msg_id=2118 payload_len=120 frame_len=148"* ]]
[[ "$crc_probe_output" == *"sync+header+payload"* ]]
[[ "$crc_probe_output" == *"crc=0x55c61e96 expected_le"* ]]

python3 - <<'PY'
from tools.um982_live_validate import classify_command_response
from tools.unicore_live_validate import crc32_unicore

assert classify_command_response("BESTSATA 2", ["response can't found device (null)"]) == "error"
frame = bytes.fromhex(
    "aa44b5614608780000a07209c82ba4120000000000120c00000000001000000008cc66e025fa4540"
    "1052859c789e01400000b0c5dd2b65406aa349423d000000325ac03f62b68f3fdb10104030000000"
    "0000000000005442211c1c0004121151000000000800000000000000000000009c6414e86d43803f"
    "8ef677e58cd36340c1c1bfecd94386bfe13edb3c8758ab3c961ec655"
)
assert crc32_unicore(frame[:-4]) == 0x55C61E96
PY

[[ -f launch/unicore_launch.py ]]
rg -n "from unicore_launch import make_unicore_node" launch/um982_launch.py >/dev/null
[[ -f config/unicore.yaml ]]
[[ -f config/um982.yaml ]]
[[ -f start_unicore.sh ]]
rg -n "start_unicore.sh" start_um982.sh >/dev/null

! rg -n "configure_receiver.sh" tools/unicore_live_validate.py >/dev/null 2>&1

normal_plan="$(python3 -c "from tools.unicore_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$normal_plan" == *"LOG PVTSLNA ONTIME 0.2"* ]]
[[ "$normal_plan" == *"BESTNAVA 0.2"* ]]
[[ "$normal_plan" == *"GPHPR 0.2"* ]]
[[ "$normal_plan" == *"GPHPR2 ONCHANGED"* ]]
[[ "$normal_plan" == *"RTKSTATUSA 1"* ]]
[[ "$normal_plan" == *"RTCMSTATUSA ONCHANGED"* ]]
[[ "$normal_plan" != *"GPGGAH"* ]]
[[ "$normal_plan" != *"BESTSATB"* ]]
[[ "$normal_plan" != *"OBSVMCMPB"* ]]

ggah_plan="$(python3 -c "from tools.unicore_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null','--enable-ggah']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$ggah_plan" == *"LOG GPGGAH ONTIME 0.2"* ]]

survey_plan="$(python3 -c "from tools.unicore_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null','--profile','survey','--format','hybrid','--enable-raw']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$survey_plan" == *"LOG PVTSLNA ONTIME 1"* ]]
[[ "$survey_plan" == *"LOG PVTSLNB ONTIME 1"* ]]
[[ "$survey_plan" == *"BESTNAVB 1"* ]]
[[ "$survey_plan" == *"RTKSTATUSB 1"* ]]
[[ "$survey_plan" == *"RTCMSTATUSB ONCHANGED"* ]]
[[ "$survey_plan" == *"LOG BESTSATB ONTIME 2"* ]]
[[ "$survey_plan" == *"LOG SATSINFOB ONTIME 2"* ]]
[[ "$survey_plan" == *"GPGSV 2"* ]]
[[ "$survey_plan" != *"GLGSV"* ]]
[[ "$survey_plan" != *"GAGSV"* ]]
[[ "$survey_plan" != *"GBGSV"* ]]
[[ "$survey_plan" == *"LOG OBSVMCMPB ONTIME 5"* ]]

debug_raw_plan="$(python3 -c "from tools.unicore_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null','--profile','debug','--format','hybrid','--enable-raw']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$debug_raw_plan" != *"OBSVMCMPA"* ]]
[[ "$debug_raw_plan" != *"OBSVMCMPB"* ]]

python3 - <<'PY'
import contextlib
import io

from tools.unicore_live_validate import (
    CommandResult,
    TransportParser,
    build_planned_log_command,
    build_probe_bauds,
    build_log_command_variants,
    classify_command_response,
    crc32_unicore,
    detect_receiver_model_from_lines,
    extract_version_line,
    LiveValidator,
    parse_args,
)
from types import MethodType

def make_nmea(body: str) -> str:
    checksum = 0
    for char in body.encode("ascii"):
        checksum ^= char
    return f"${body}*{checksum:02X}"

assert build_probe_bauds(None, 921600, after_factory_reset=True) == [115200, 460800, 921600]
assert build_probe_bauds(921600, 921600, after_factory_reset=False) == [921600, 115200, 460800]
assert classify_command_response("VERSION", ['#VERSIONA,"UM982","R4.10Build15434"']) == "ok"
assert classify_command_response("UNLOGALL", ["unsupported command"]) == "unsupported"
assert classify_command_response("BESTSATA 2", ["response can't found device (null)"]) == "error"
assert classify_command_response("RESET", []) == "no_response"
assert extract_version_line(['#VERSIONA,"UM980","R4.10Build15434"']) is not None
assert detect_receiver_model_from_lines(['#VERSIONA,"UM981","R4.10Build15434"']) == "UM981"

validator = LiveValidator(parse_args(["--port", "/dev/null"]))
validator._handle_unicore_ascii_line(0.0, "#PVTSLNA,foo*00000000")
assert validator.state.message_counters["PVTSLNA"].count == 1
assert validator.state.unicore_ascii_crc_bad == 1
assert "PVTSLNA" not in validator.state.unknown_ascii_types

validator._handle_line(0.0, b"\xaa\x44\xb5\x00junk\n".rstrip(b"\n"))
assert validator.state.unknown_ascii_types == {}

assert build_planned_log_command("BESTNAVA", 0.2) == "BESTNAVA 0.2"
assert build_planned_log_command("RTKSTATUSA", 1.0) == "RTKSTATUSA 1"
assert build_planned_log_command("RTCMSTATUSA", 1.0) == "RTCMSTATUSA ONCHANGED"
assert build_planned_log_command("GPHPR", 1.0) == "GPHPR 1"
assert build_planned_log_command("GPHPR2", 1.0) == "GPHPR2 ONCHANGED"
assert build_planned_log_command("GPGSV", 2.0) == "GPGSV 2"
assert build_planned_log_command("GPGGA", 0.2) == "LOG GPGGA ONTIME 0.2"
assert build_planned_log_command("GPGGAH", 0.2) == "LOG GPGGAH ONTIME 0.2"
assert build_planned_log_command("PVTSLNA", 0.2) == "LOG PVTSLNA ONTIME 0.2"

validator = LiveValidator(parse_args(["--port", "/dev/null", "--profile", "debug"]))
assert "GPGSV" in validator.plan.expected_messages
assert validator.plan.expected_messages["GPGSV"].observed_names == (
    "GPGSV",
    "GLGSV",
    "GAGSV",
    "GBGSV",
    "GQGSV",
)
assert not any(
    unwanted in validator.plan.log_commands for unwanted in ("GLGSV 1", "GAGSV 1", "GBGSV 1")
)

validator = LiveValidator(parse_args(["--port", "/dev/null"]))
gngga = make_nmea("GNGGA,123519,4807.038,N,01131.000,E,4,12,0.8,545.4,M,46.9,M,,")
validator.state.total_bytes = len(gngga)
validator._handle_nmea_line(0.0, gngga)
summary = validator._build_summary(1.0, 0.0)
assert summary["expected_messages"]["GPGGA"]["count"] == 1
assert summary["expected_messages"]["GPGGA"]["observed_aliases"] == ["GNGGA"]
assert summary["expected_messages"]["GPGGA"]["observed_by_name"]["GNGGA"] == 1

validator = LiveValidator(parse_args(["--port", "/dev/null", "--enable-ggah"]))
gnggah = make_nmea("GNGGAH,123519,4807.038,N,01131.000,E,4,12,0.8,545.4,M,46.9,M,,")
validator.state.total_bytes = len(gnggah)
validator._handle_nmea_line(0.0, gnggah)
summary = validator._build_summary(1.0, 0.0)
assert summary["expected_messages"]["GPGGAH"]["count"] == 1
assert summary["expected_messages"]["GPGGAH"]["observed_aliases"] == ["GNGGAH"]
assert summary["expected_messages"]["GPGGAH"]["observed_by_name"]["GNGGAH"] == 1
assert "GNGGAH" not in validator.state.unknown_ascii_types

payload = bytes([0x34] + [0x00] * 119)
frame = bytearray(
    bytes.fromhex("aa44b5614608780000a07209e0ea5d0e")
    + bytes([0x10, 0x00, 0x00, 0x00, 0x00, 0x12, 0x61, 0x00])
    + payload
)
frame.extend(int.to_bytes(crc32_unicore(bytes(frame)), 4, "little"))
parser = TransportParser(512)
events = parser.feed(bytes(frame), 0.0)
assert len(events) == 1
kind, payload_event = events[0]
assert kind == "binary"
_, binary_frame = payload_event
assert binary_frame.message_id == 2118
assert binary_frame.header_length == 24
assert binary_frame.payload_length == 120
assert parser.sync_candidates == 1
assert parser.last_header_length == 24
assert parser.last_payload_length == 120
assert parser.binary_crc_errors == 0
assert parser.frame_parse_errors_by_reason == {}

bad_frame = bytearray(frame)
bad_frame[-1] ^= 0xFF
parser = TransportParser(512)
events = parser.feed(bytes(bad_frame), 0.0)
assert events == []
assert parser.binary_crc_errors == 1
assert parser.recent_crc_failures
assert parser.recent_crc_failures[-1].message_id == 2118
assert parser.recent_crc_failures[-1].payload_length == 120

variants = build_log_command_variants("BESTNAVA", 0.2, "COM1")
assert variants[0] == ("unicore_direct_period", "BESTNAVA 0.2")
assert variants[1] == ("bare_ontime", "BESTNAVA ONTIME 0.2")
assert variants[2] == ("com_period", "BESTNAVA COM1 0.2")
assert variants[3] == ("com_rate", "BESTNAVA COM1 5")
assert variants[4] == ("nmea_log_ontime", "LOG BESTNAVA ONTIME 0.2")

variants = build_log_command_variants("RTCMSTATUSA", None, "COM1")
assert variants[0] == ("unicore_onchanged", "RTCMSTATUSA ONCHANGED")
assert variants[1] == ("com_onchanged", "RTCMSTATUSA COM1 ONCHANGED")

def install_fake_sender(target, statuses):
    def fake_send(self, command, response_timeout_sec=0.0, *, record=True, logical_message=None, syntax_label=None):
        status = statuses.get(command, "unsupported")
        result = CommandResult(
            command=command,
            baud=self.serial.baud,
            status=status,
            lines=["mock response"],
            logical_message=logical_message,
            syntax_label=syntax_label,
        )
        if record:
            self._record_command_result(result)
        return result

    target._send_command = MethodType(fake_send, target)

validator = LiveValidator(parse_args(["--port", "/dev/null", "--discover-log-syntax"]))
install_fake_sender(validator, {"LOG GPGGA ONTIME 0.2": "ok"})
result = validator._apply_log_command_with_fallback("LOG GPGGA ONTIME 0.2")
assert result.status == "ok"
assert validator.state.accepted_log_commands["GPGGA"] == "LOG GPGGA ONTIME 0.2"
assert validator.state.log_command_syntax_by_message["GPGGA"] == "nmea_log_ontime"
assert validator.state.rejected_log_commands["GPGGA"] == []
assert len(validator.state.command_results) == 1

validator = LiveValidator(parse_args(["--port", "/dev/null", "--discover-log-syntax"]))
install_fake_sender(
    validator,
    {"BESTNAVA 0.2": "ok"},
)
result = validator._apply_log_command_with_fallback("BESTNAVA 0.2")
assert result.command == "BESTNAVA 0.2"
assert validator.state.accepted_log_commands["BESTNAVA"] == "BESTNAVA 0.2"
assert validator.state.log_command_syntax_by_message["BESTNAVA"] == "unicore_direct_period"
assert validator.state.rejected_log_commands["BESTNAVA"] == []
assert len(validator.state.command_results) == 1

validator = LiveValidator(parse_args(["--port", "/dev/null", "--discover-log-syntax"]))
install_fake_sender(
    validator,
    {
        "RTCMSTATUSA ONCHANGED": "unsupported",
        "RTCMSTATUSA COM1 ONCHANGED": "unsupported",
    },
)
result = validator._apply_log_command_with_fallback("RTCMSTATUSA ONCHANGED")
assert result.status == "unsupported"
assert "RTCMSTATUSA" not in validator.state.accepted_log_commands
assert validator.state.rejected_log_commands["RTCMSTATUSA"] == [
    "RTCMSTATUSA ONCHANGED",
    "RTCMSTATUSA COM1 ONCHANGED",
]
assert len(validator.state.command_results) == 1

validator = LiveValidator(parse_args(["--port", "/dev/null", "--discover-log-syntax"]))
install_fake_sender(
    validator,
    {
        "LOG BESTSATA ONTIME 2": "error",
        "BESTSATA ONTIME 2": "unsupported",
        "BESTSATA 2": "ok",
    },
)
result = validator._apply_log_command_with_fallback("LOG BESTSATA ONTIME 2")
assert result.command == "BESTSATA 2"
assert validator.state.accepted_log_commands["BESTSATA"] == "BESTSATA 2"
assert validator.state.rejected_log_commands["BESTSATA"] == [
    "LOG BESTSATA ONTIME 2",
    "BESTSATA ONTIME 2",
]

try:
    with contextlib.redirect_stderr(io.StringIO()):
        parse_args(["--port", "/dev/null", "--factory-reset", "--reset"])
    raise AssertionError("mutually exclusive reset options should fail")
except SystemExit:
    pass
PY

echo "unicore_live_validate: OK"
