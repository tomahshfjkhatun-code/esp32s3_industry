import argparse
import pathlib
import re
import subprocess
import sys

parser = argparse.ArgumentParser(description="Compile-time execution of 702 production logic; no firmware build, device or network.")
parser.add_argument("compiler")
args = parser.parse_args()
root = pathlib.Path(__file__).resolve().parents[1]

def source(path):
    text = (root / path).read_text(encoding="utf-8")
    return re.sub(r'(?m)^#(?:pragma once|include "[^"]+")\s*$', "", text)

protocol_header = source("src/application/can/702_can_protocol.h")
protocol_header = re.sub(r"(?m)^(bool|EncodeResult)( \w+\()", r"constexpr \1\2", protocol_header)
protocol_source = source("src/application/can/702_can_protocol.cpp")
protocol_source = re.sub(r"(?m)^(bool|void|EncodeResult)( \w+\()", r"constexpr \1\2", protocol_source)
protocol_source = protocol_source.replace("template<class T> bool Fresh", "template<class T> constexpr bool Fresh")

axis_header = source("src/application/control/axis_control.h")
axis_header = re.sub(r"(?m)^(    )(static )?(bool|void|AxisStatus|int8_t)( \w+\([^\n]+;)$", r"\1\2constexpr \3\4", axis_header)
axis_source = source("src/application/control/axis_control.cpp")
axis_source = re.sub(r"(?m)^((?:bool|void|AxisStatus|int8_t) AxisControl::)", r"constexpr \1", axis_source)

pump_header = source("src/application/control/pump_control.h")
start = pump_header.index("class PumpControl final")
end = pump_header.index("class PumpRuntime final")
block = pump_header[start:end]
names = ["explicit PumpControl(", "bool Configure(", "static bool ValidCan(", "bool ReceiveCan(", "void ReceiveIo(", "void ReceiveOutput(", "void ReportFault(", "void Inhibit(", "Drive Step(", "bool ResetFault(", "Snapshot Status(", "bool Fresh(", "void Expire(", "void UpdateDrive(", "void CheckAxes("]
for name in names:
    prefix = name.replace("static ", "static constexpr ") if name.startswith("static ") else name.replace("explicit ", "explicit constexpr ") if name.startswith("explicit ") else "constexpr " + name
    if block.count(name) != 1:
        sys.exit("Unsupported declaration: " + name)
    block = block.replace(name, prefix)
pump_header = pump_header[:start] + block + pump_header[end:]
pump_source = source("src/application/control/pump_control.cpp")
pump_source = pump_source[:pump_source.index("bool PumpControl::Publish")] + "\n}\n"
pump_source = re.sub(r"(?m)^(PumpControl::PumpControl|(?:bool|void|Drive|Snapshot) PumpControl::)", r"constexpr \1", pump_source)

signal_header = source("src/application/io/702_signal_map.h")
io_header = source("src/application/io/702_io_model.h")
io_header = re.sub(r"(?m)^(    )?(bool|void|control::Drive)( \w+\([^\n]+;)$", r"\1constexpr \2\3", io_header)
io_source = source("src/application/io/702_io_model.cpp")
io_source = re.sub(r"(?m)^(bool|void|int32_t|control::Drive)( [\w:]+\()", r"constexpr \1\2", io_source)

runtime_start = pump_header.index("class PumpRuntime final")
runtime_block = pump_header[runtime_start:]
runtime_block = runtime_block.replace("    PumpRuntime(", "    constexpr PumpRuntime(")
runtime_block = runtime_block.replace("    bool Configure(", "    constexpr bool Configure(")
runtime_block = runtime_block.replace("    Snapshot Status(", "    constexpr Snapshot Status(")
pump_header = pump_header[:runtime_start] + runtime_block
runtime_text = source("src/application/control/pump_control.cpp")
runtime_source = runtime_text[runtime_text.index("PumpRuntime::PumpRuntime"):runtime_text.index("bool PumpRuntime::ReceiveCan")]
runtime_source += runtime_text[runtime_text.index("Snapshot PumpRuntime::Status"):runtime_text.index("bool PumpRuntime::ResetFault")]
runtime_source = re.sub(r"(?m)^(PumpRuntime::PumpRuntime|(?:bool|Snapshot) PumpRuntime::)", r"constexpr \1", runtime_source)
runtime_source = "\nnamespace ind::application::control {\n" + runtime_source + "\n}\n"

# Hardware objects are replaced by cached fixtures; the production Read body is unchanged.
service_header = source("src/application/io/702_io_service.h")
io_types = service_header[service_header.index("template<class T> struct IoValue"):service_header.index("// 钩子")]
service_text = source("src/application/io/702_io_service.cpp")
read_source = service_text[service_text.index("control::ReadResult Io702::Read("):service_text.index("bool Io702::ReadOutput(")]
read_source = read_source.replace("control::ReadResult Io702::Read(", "constexpr control::ReadResult Io702::Read(", 1)
older_source = service_text[service_text.index("uint32_t Older("):service_text.index("control::Drive StopDrive(")]
expire_source = service_text[service_text.index("template<class T> void Expire"):service_text.index("uint32_t Older(")]
expire_source = expire_source.replace("template<class T> void Expire", "template<class T> constexpr void Expire")
convert_source = service_text[service_text.index("void Io702::Convert("):service_text.index("IoStatus Io702::get_702_io()")]
convert_source = convert_source.replace("void Io702::Convert(", "constexpr void Io702::Convert(", 1)
io_fixture = """
namespace ind::application::io {
enum class IoResult { Disconnected };
struct PwmStatus {};
struct NetworkConfig {};
struct IoHooks {};
""" + io_types + """
class Io702 {
public:
    IoConfig pump_config_;
    IoStatus io_status_;
    constexpr IoStatus get_702_io() const { return io_status_; }
    constexpr const PumpConfig& Config(control::Side pump_side) const { return pump_side == control::Side::Left ? pump_config_.left_pump : pump_config_.right_pump; }
    constexpr control::ReadResult Read(control::Side pump_side, control::IoSample* pump_sample, uint32_t* sample_at);
    constexpr void Convert(PumpIo& pump_status, const PumpConfig& pump_config, uint32_t now_ms) const;
};
constexpr uint32_t NowMs() { return 100; }
""" + older_source.replace("uint32_t Older(", "constexpr uint32_t Older(", 1) + expire_source + convert_source + read_source + "\n}\n"

app_header = source("src/application/702_app.h")
app_config = app_header[app_header.index("struct AlarmRoutes"):app_header.index("struct CanStatus")]
config_source = source("src/application/702_config.cpp")
config_source = re.sub(r"(?m)^(io::PositionConfig|control::AxisSettings|control::Settings|AppConfig)( \w+\()", r"constexpr \1\2", config_source)
config_source = "\nnamespace ind::application {\n" + app_config + "\n}\n" + config_source

cases = source("tests/702_prepare_cases.cpp") + source("tests/axis_control_cases.cpp") + source("tests/702_axes_cases.cpp")
translation = protocol_header + protocol_source + axis_header + axis_source + pump_header + pump_source + runtime_source + io_header + io_source + signal_header + io_fixture + config_source + cases
result = subprocess.run([args.compiler, "-std=gnu++17", "-Wall", "-Wextra", "-Werror", "-fsyntax-only", "-x", "c++", "-"], input=translation, text=True, encoding="utf-8", errors="replace", capture_output=True)
print(result.stdout + result.stderr, end="")
if result.returncode == 0:
    print("PASS:", cases.count("static_assert("), "compile-time scenarios; production function bodies unchanged, constexpr added only to compiler input.")
sys.exit(result.returncode)
