"""Exercise actual sketch UDP functions and installed MbedUdp.cpp in ARM emulation.

The socket transport is mocked; the problematic UDP available/read/parse code
comes directly from the installed board package. No hardware is accessed.
"""
import argparse
import os
import re
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(tempfile.gettempdir()) / "codex-portenta-test-dependencies"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--reproduce-old-drain", action="store_true")
parser.add_argument("--compiler", type=Path, default=Path(os.environ["LOCALAPPDATA"]) /
                    "Arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-g++.exe")
parser.add_argument("--udp-source", type=Path, default=Path(os.environ["LOCALAPPDATA"]) /
                    "Arduino15/packages/arduino/hardware/mbed_portenta/4.6.0/libraries/SocketWrapper/src/MbedUdp.cpp")
args = parser.parse_args()
test_dir = Path(__file__).resolve().parent
project = test_dir.parents[1]
output = Path(tempfile.gettempdir()) / "codex-portenta-udp-tests"
output.mkdir(exist_ok=True)
main = (project / "Portenta_Core_V3.ino").read_text(encoding="utf-8")

def function(name):
    definition = re.search(r"(?m)^[A-Za-z_][^\n;{}]*\b" + re.escape(name) +
                           r"\([^;{}]*\)\s*\{", main)
    if not definition:
        raise ValueError("Missing function definition: " + name)
    start, opening = definition.start(), definition.end() - 1
    depth, end = 1, opening + 1
    while depth:
        if main[end] == "{": depth += 1
        elif main[end] == "}": depth -= 1
        end += 1
    return main[start:end]

functions = "\n".join(function(name) for name in
                      ["controllerLossRequiresStop", "computeChecksum", "processUDPPacket", "activate_UDP_Failsafe",
                       "reportControllerRecovery", "serviceControllerPackets",
                       "sendVisionPacket", "manageVisionPort"])
if args.reproduce_old_drain:
    marker = "if (!esp32IP_known || cameraUDP.remoteIP() != esp32IP || size != sizeof(VisionPacket)) {"
    assert marker in functions
    functions = functions.replace(marker, marker + "\nwhile(cameraUDP.available()) cameraUDP.read();", 1)
(output / "AppUdpFunctions.inc").write_text(functions, encoding="utf-8")
state_start = main.index("enum State")
state_end = main.index(";", main.index("}", state_start)) + 1
(output / "AppUdpState.inc").write_text(main[state_start:state_end], encoding="utf-8")
vendor = args.udp_source.read_text(encoding="utf-8")
vendor = vendor.replace('#include "MbedUdp.h"', '#include "FakeMbedUdp.h"')
(output / "MbedUdpActual.cpp").write_text(vendor, encoding="utf-8")
elf_path = output / "udp_tests.elf"
command = [str(args.compiler), "-std=c++14", "-O2", "-mcpu=cortex-m4", "-mthumb",
           "-mfloat-abi=soft", "-fno-exceptions", "-fno-rtti", "-ffunction-sections",
           "-fdata-sections", "-nostartfiles", "--specs=nosys.specs",
           "-Wl,-e,test_main,-Ttext=0x10000,--gc-sections",
           "-I" + str(test_dir), "-I" + str(project), "-I" + str(output),
           str(test_dir / "udp_tests.cpp"), str(output / "MbedUdpActual.cpp"), "-o", str(elf_path)]
if args.reproduce_old_drain:
    command.insert(1, "-DREPRODUCE_OLD_DRAIN")
compiled = subprocess.run(command, capture_output=True, text=True)
if compiled.returncode:
    sys.stderr.write(compiled.stderr)
    raise SystemExit(compiled.returncode)

emulator = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
emulator.mem_map(0x10000, 0x400000)
emulator.mem_map(0x20000000, 0x100000)
with elf_path.open("rb") as source:
    elf = ELFFile(source)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
            emulator.mem_write(segment["p_vaddr"], segment.data())
    symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
done = symbols["test_done"] & ~1
finished = False
def stop(uc, address, size, data):
    global finished
    if address == done:
        finished = True
        uc.emu_stop()
emulator.hook_add(UC_HOOK_CODE, stop, begin=done, end=done)
emulator.reg_write(UC_ARM_REG_SP, 0x200ff000)
emulator.reg_write(UC_ARM_REG_LR, done | 1)
emulator.emu_start(symbols["test_main"] | 1, 0, timeout=20_000_000, count=2_000_000)
if args.reproduce_old_drain:
    if finished:
        raise SystemExit("Old drain unexpectedly returned")
    print("Reproduced old UDP discard hang using installed MbedUdp.cpp")
else:
    if not finished:
        raise SystemExit("UDP tests hung or exceeded the instruction limit")
    checks, failures, first = struct.unpack("<III", emulator.mem_read(symbols["test_result"], 12))
    print(f"UDP regression tests: {checks} checks, {failures} failures")
    if failures:
        raise SystemExit(f"First failing check: {first}")
