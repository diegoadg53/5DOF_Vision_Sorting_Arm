"""Execute real Nucleo feedback/command functions and Portenta A parsing in ARM emulation.

SPI/UART transactions, clocks and register accesses are mocked. Nothing is flashed.
Dependencies: unicorn and pyelftools (also accepted under the Codex temp dependency folder).
"""
from pathlib import Path
import argparse
import os
import struct
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(tempfile.gettempdir()) / "codex-portenta-test-dependencies"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--compiler", type=Path, default=Path(
    "C:/ST/STM32CubeIDE_2.1.0/STM32CubeIDE/plugins/"
    "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/"
    "tools/bin/arm-none-eabi-gcc.exe"))
parser.add_argument("--portenta-compiler", type=Path, default=Path(os.environ["LOCALAPPDATA"]) /
    "Arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-g++.exe")
args = parser.parse_args()
test_dir = Path(__file__).resolve().parent
project = test_dir.parent
portenta = project.parent / "Portenta_Core_V3"
output = Path(tempfile.gettempdir()) / "codex-nucleo-feedback-tests"
output.mkdir(exist_ok=True)

firmware_image = project / "Debug/NucleoTest_AngularSensor.elf"
if firmware_image.exists():
    with firmware_image.open("rb") as source:
        for segment in ELFFile(source).iter_segments():
            address, count = segment["p_paddr"], segment["p_filesz"]
            if segment["p_type"] == "PT_LOAD" and count and 0x08000000 <= address < 0x08080000:
                assert address + count <= 0x08040000, "Firmware overlaps calibration journal"

def build(command):
    r = subprocess.run(command, capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stderr)
        raise SystemExit(r.returncode)

def emulate(path, uart=False):
    u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    u.mem_map(0x10000, 0x400000)
    u.mem_map(0x20000000, 0x100000)
    u.mem_map(0x40000000, 0x40000)
    u.mem_map(0xE0000000, 0x100000)
    u.mem_map(0x08000000, 0x80000)
    u.mem_write(0x08000000, b"\xff" * 0x80000)
    with path.open("rb") as f:
        elf = ELFFile(f)
        for s in elf.iter_segments():
            if s["p_type"] == "PT_LOAD" and s["p_filesz"]:
                u.mem_write(s["p_vaddr"], s.data())
        symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
    done = symbols["test_done"] & ~1
    finished = False
    def stop(uc, address, size, data):
        nonlocal finished
        if address == done:
            finished = True
            uc.emu_stop()
    u.hook_add(UC_HOOK_CODE, stop, begin=done, end=done)
    if uart:
        def cycle_counter(uc, access, address, size, value, data):
            stopped = struct.unpack("<I", u.mem_read(symbols["test_dwt_stopped"], 4))[0]
            if not stopped:
                count = struct.unpack("<I", u.mem_read(address, 4))[0]
                step = struct.unpack("<I", u.mem_read(symbols["test_dwt_step"], 4))[0]
                u.mem_write(address, struct.pack("<I", (count + step) & 0xffffffff))
        u.hook_add(UC_HOOK_MEM_READ, cycle_counter, begin=0xE0001004, end=0xE0001004)
        def bus_released(base, port, pin):
            cr1 = struct.unpack("<I",u.mem_read(base+12,4))[0]
            otype = struct.unpack("<I",u.mem_read(port+4,4))[0]
            return (cr1 & 12) == 4 and bool(otype & (1 << pin))
        def word(name):
            return struct.unpack("<I", u.mem_read(symbols[name], 4))[0]
        def receive(uc, access, address, size, value, data):
            enabled = bus_released(0x40005000,0x40020800,12)
            index, count = word("uart_rx_index"), word("uart_rx_count")
            active = enabled and word("uart_rx_active") and index < count
            if address == 0x40005000:  # SR: TXE/TC, and RXNE only when a byte exists.
                u.mem_write(address, struct.pack("<I", 0xC0 | (0x20 if active else 0)))
            elif address == 0x40005004 and active:  # DR read consumes one byte.
                byte = u.mem_read(symbols["uart_rx_stream"] + index, 1)[0]
                u.mem_write(address, struct.pack("<I", byte))
                u.mem_write(symbols["uart_rx_index"], struct.pack("<I", index + 1))
                if index + 1 == count:
                    u.mem_write(symbols["uart_rx_active"], struct.pack("<I", 0))
        u.hook_add(UC_HOOK_MEM_READ, receive, begin=0x40005000, end=0x40005004)
        def receive_drive(uc, access, address, size, value, bus):
            base = [0x40011000, 0x40004800, 0x40004C00][bus]
            ports = [(0x40020000,9),(0x40020400,10),(0x40020000,0)]
            enabled = bus_released(base,*ports[bus])
            index = struct.unpack("<I",u.mem_read(symbols["servo_rx_index"]+4*bus,4))[0]
            count = struct.unpack("<I",u.mem_read(symbols["servo_rx_count"]+4*bus,4))[0]
            active = enabled and index < count
            if address == base:
                u.mem_write(base,struct.pack("<I",0xC0 | (0x20 if active else 0)))
            elif address == base+4 and active:
                byte = u.mem_read(symbols["servo_rx_stream"]+16*bus+index,1)[0]
                u.mem_write(base+4,struct.pack("<I",byte))
                u.mem_write(symbols["servo_rx_index"]+4*bus,struct.pack("<I",index+1))
        for bus, base in enumerate([0x40011000,0x40004800,0x40004C00]):
            u.hook_add(UC_HOOK_MEM_READ,receive_drive,user_data=bus,begin=base,end=base+4)
    u.reg_write(UC_ARM_REG_SP, 0x200FF000)
    u.reg_write(UC_ARM_REG_LR, done | 1)
    u.emu_start(symbols["test_main"] | 1, 0, timeout=30_000_000, count=30_000_000)
    if not finished:
        raise SystemExit(f"{path.name}: execution limit exceeded")
    checks, failures, first = struct.unpack("<III", u.mem_read(symbols["test_result"], 12))
    if "test_diag_sample" in symbols:
        (output / "diagnostic_frame.bin").write_bytes(u.mem_read(symbols["test_diag_sample"],152))
    print(f"{path.stem}: {checks} checks, {failures} failures")
    if failures:
        if "test_failure_codes" in symbols:
            codes = struct.unpack("<32I", u.mem_read(symbols["test_failure_codes"], 128))
            print("Failing checks:", ", ".join(map(str, codes[:min(failures, 32)])))
        raise SystemExit(f"First failing check: {first}")

elf = output / "nucleo_feedback.elf"
includes = [project / p for p in ["Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
    "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include"]]
common = ["-O1", "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft", "-ffunction-sections",
    "-fdata-sections", "-nostartfiles", "--specs=nosys.specs",
    "-Wl,-e,test_main,-Ttext=0x10000,--gc-sections"]
build([str(args.compiler), "-std=gnu11", "-DSTM32F446xx", "-DUSE_HAL_DRIVER", *common,
    *("-I" + str(p) for p in includes), str(test_dir / "feedback_tests.c"), "-lm", "-o", str(elf)])
emulate(elf, uart=True)

# Portable calibration engine and real journal against a small, fault-injected NOR model.
assert (project / "Core/Inc/JointCalibrationProtocol.h").read_bytes() == (
    portenta / "JointCalibrationProtocol.h").read_bytes(), "Calibration protocol copies diverged"
assert (project / "Core/Inc/JointDiagnosticProtocol.h").read_bytes() == (
    portenta / "JointDiagnosticProtocol.h").read_bytes(), "Diagnostic protocol copies diverged"
elf = output / "joint_calibration.elf"
build([str(args.compiler), "-std=gnu11", *common, "-I" + str(project / "Core/Inc"),
       str(test_dir / "calibration_tests.c"), "-lm", "-o", str(elf)])
emulate(elf)

elf = output / "joint_wizard.elf"
build([str(args.portenta_compiler), "-std=c++14", "-fno-exceptions", "-fno-rtti", *common,
       "-I" + str(portenta), str(portenta / "tests/trajectory/joint_wizard_tests.cpp"),
       "-lm", "-o", str(elf)])
emulate(elf)

# Extract the exact existing Portenta A validation body; do not modify its sketch.
main = (portenta / "Portenta_Core_V3.ino").read_text(encoding="utf-8")
start = main.index("  case 'A':", main.index("void processNucleoCMD()"))
end = main.index("  case 'R':", start)
body = main[start:end].split("\n", 1)[1].rsplit("    break;", 1)[0]
fresh_start = main.index("bool feedbackFresh() {")
fresh_end = main.index("\n}", fresh_start) + 2
source = r'''
#include "ArmTrajectory.h"
#include <stdio.h>
#include <string.h>
ArmFeedback armFeedback;
AutoConfig autoConfig;
float real_q[5], real_gripper;
bool new_q_angles;
char nucleoBuffer[128];
static uint32_t now;
uint32_t millis() { return now; }
extern "C" { volatile uint32_t test_result[3] = {}; }
extern "C" void *_sbrk(int n) {
  static char heap[32768]; static int used;
  if(n < 0 || used+n > int(sizeof(heap))) return (void*)-1;
  void *p=heap+used; used+=n; return p;
}
void receive() { float a[12]; char *d=nucleoBuffer+2; int n=0;
''' + body + "\n}\n" + main[fresh_start:fresh_end] + r'''
void check(bool ok, uint32_t code) {
  ++test_result[0]; if(!ok) { ++test_result[1]; if(!test_result[2]) test_result[2]=code; }
}
extern "C" __attribute__((used,noinline)) void test_done() { for(;;) asm volatile("nop"); }
extern "C" void test_main() {
  now=1000; strcpy(nucleoBuffer,"A:0,0,0,-0.46,0,0"); receive();
  check(armFeedback.valid && armFeedback.sequence==1 && feedbackFresh(),501);
  now=1100; strcpy(nucleoBuffer,"A:nan,nan,nan,nan,nan,0"); receive();
  check(armFeedback.sequence==1 && armFeedback.timestampMs==1000,502);
  check(armFeedback.q[3]==-.46f,503);
  now=1400; check(!feedbackFresh(),504);
  strcpy(nucleoBuffer,"A:19.93,0.02,-13.86,-33.26,-89.91,0"); receive();
  check(feedbackFresh() && armFeedback.sequence==2,505);
  check(armFeedback.q[0]==19.93f && armFeedback.q[3]==-33.26f,506);
  strcpy(nucleoBuffer,"A:19.93,0.02,nan,-33.26,-89.91,0"); receive();
  check(armFeedback.sequence==2 && armFeedback.q[2]==-13.86f,507);
  test_done();
}
'''
(output / "portenta_interop.cpp").write_text(source, encoding="utf-8")
interop = output / "portenta_interop.elf"
build([str(args.portenta_compiler), "-std=c++14", "-fno-rtti", "-fno-exceptions", *common,
    "-I" + str(portenta / "tests/trajectory"), "-I" + str(portenta),
    "-I" + str(Path.home() / "Documents/Arduino/libraries/BasicLinearAlgebra"),
    str(output / "portenta_interop.cpp"), "-lm", "-o", str(interop)])
emulate(interop)
