#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."

python3 - <<'PY'
from pathlib import Path

path = Path("tools/um982_live_validate.py")
compile(path.read_text(encoding="utf-8"), str(path), "exec")
PY

help_output="$(python3 tools/um982_live_validate.py --help)"
[[ "$help_output" == *"--port"* ]]
[[ "$help_output" == *"--format {ascii,hybrid,binary}"* ]]
[[ "$help_output" == *"--factory-reset"* ]]
[[ "$help_output" == *"--reset"* ]]
[[ "$help_output" == *"--save-config"* ]]
[[ "$help_output" == *"--com-port"* ]]
[[ "$help_output" == *"--signalgroup-override"* ]]
[[ "$help_output" == *"--discover-log-syntax"* ]]

! rg -n "configure_receiver.sh" tools/um982_live_validate.py >/dev/null 2>&1

normal_plan="$(python3 -c "from tools.um982_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$normal_plan" == *"LOG PVTSLNA ONTIME 0.2"* ]]
[[ "$normal_plan" == *"BESTNAVA 0.2"* ]]
[[ "$normal_plan" == *"GPHPR 0.2"* ]]
[[ "$normal_plan" == *"GPHPR2 ONCHANGED"* ]]
[[ "$normal_plan" == *"RTKSTATUSA 1"* ]]
[[ "$normal_plan" == *"RTCMSTATUSA ONCHANGED"* ]]
[[ "$normal_plan" != *"BESTSATB"* ]]
[[ "$normal_plan" != *"OBSVMCMPB"* ]]

survey_plan="$(python3 -c "from tools.um982_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null','--profile','survey','--format','hybrid','--enable-raw']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$survey_plan" == *"LOG PVTSLNA ONTIME 1"* ]]
[[ "$survey_plan" == *"LOG PVTSLNB ONTIME 1"* ]]
[[ "$survey_plan" == *"BESTNAVB 1"* ]]
[[ "$survey_plan" == *"RTKSTATUSB 1"* ]]
[[ "$survey_plan" == *"RTCMSTATUSB ONCHANGED"* ]]
[[ "$survey_plan" == *"LOG BESTSATB ONTIME 2"* ]]
[[ "$survey_plan" == *"LOG SATSINFOB ONTIME 2"* ]]
[[ "$survey_plan" == *"LOG OBSVMCMPB ONTIME 5"* ]]

debug_raw_plan="$(python3 -c "from tools.um982_live_validate import parse_args, LiveValidator; args=parse_args(['--port','/dev/null','--profile','debug','--format','hybrid','--enable-raw']); validator=LiveValidator(args); print('\\n'.join(validator.plan.log_commands))")"
[[ "$debug_raw_plan" != *"OBSVMCMPA"* ]]
[[ "$debug_raw_plan" != *"OBSVMCMPB"* ]]

python3 - <<'PY'
import contextlib
import io

from tools.um982_live_validate import (
    CommandResult,
    build_planned_log_command,
    build_probe_bauds,
    build_log_command_variants,
    classify_command_response,
    detect_receiver_model_from_lines,
    extract_version_line,
    LiveValidator,
    parse_args,
)
from types import MethodType

assert build_probe_bauds(None, 921600, after_factory_reset=True) == [115200, 460800, 921600]
assert build_probe_bauds(921600, 921600, after_factory_reset=False) == [921600, 115200, 460800]
assert classify_command_response("VERSION", ['#VERSIONA,"UM982","R4.10Build15434"']) == "ok"
assert classify_command_response("UNLOGALL", ["unsupported command"]) == "unsupported"
assert classify_command_response("RESET", []) == "no_response"
assert extract_version_line(['#VERSIONA,"UM980","R4.10Build15434"']) is not None
assert detect_receiver_model_from_lines(['#VERSIONA,"UM981","R4.10Build15434"']) == "UM981"

validator = LiveValidator(parse_args(["--port", "/dev/null"]))
validator._handle_unicore_ascii_line(0.0, "#PVTSLNA,foo*00000000")
assert validator.state.message_counters["PVTSLNA"].count == 1
assert validator.state.unicore_ascii_crc_bad == 1
assert "PVTSLNA" not in validator.state.unknown_ascii_types

assert build_planned_log_command("BESTNAVA", 0.2) == "BESTNAVA 0.2"
assert build_planned_log_command("RTKSTATUSA", 1.0) == "RTKSTATUSA 1"
assert build_planned_log_command("RTCMSTATUSA", 1.0) == "RTCMSTATUSA ONCHANGED"
assert build_planned_log_command("GPHPR", 1.0) == "GPHPR 1"
assert build_planned_log_command("GPHPR2", 1.0) == "GPHPR2 ONCHANGED"
assert build_planned_log_command("GPGGA", 0.2) == "LOG GPGGA ONTIME 0.2"
assert build_planned_log_command("PVTSLNA", 0.2) == "LOG PVTSLNA ONTIME 0.2"

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

try:
    with contextlib.redirect_stderr(io.StringIO()):
        parse_args(["--port", "/dev/null", "--factory-reset", "--reset"])
    raise AssertionError("mutually exclusive reset options should fail")
except SystemExit:
    pass
PY

echo "um982_live_validate: OK"
