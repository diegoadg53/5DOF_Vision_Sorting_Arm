"""Run real ARM C++ controller/IK code in Unicorn; no hardware is accessed.

Dependencies: unicorn and pyelftools. Outputs are placed in the user's temp
directory, outside the sketch. Arduino fixtures are used only in this build.
"""
import argparse
import os
import re
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

dependencies = Path(tempfile.gettempdir()) / "codex-portenta-test-dependencies"
sys.path.insert(0, str(dependencies))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--vision-diagnostics", action="store_true",
                    help="Run focused ONLY VISION and serial queue regression checks")
parser.add_argument("--settings-only", action="store_true")
parser.add_argument("--classification-only", action="store_true",
                    help="Run classification orientation, joint-lock and workspace checks")
parser.add_argument("--compiler", type=Path, default=Path(os.environ["LOCALAPPDATA"]) /
                    "Arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-g++.exe")
parser.add_argument("--bla", type=Path, default=Path.home() / "Documents/Arduino/libraries/BasicLinearAlgebra")
args = parser.parse_args()
test_dir = Path(__file__).resolve().parent
project = test_dir.parents[1]
output_dir = Path(tempfile.gettempdir()) / "codex-portenta-trajectory-tests"
output_dir.mkdir(exist_ok=True)
elf_path = output_dir / "controller_tests.elf"
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

(output_dir / "AppHomeFunctions.inc").write_text(
    "\n".join(function(name) for name in ["feedbackFresh", "validAutoJoints", "sendAutoHome"]),
    encoding="utf-8")
def enum_source(prefix):
    start = main.index(prefix)
    end = main.index(";", main.index("}", start)) + 1
    return main[start:end]

(output_dir / "AppAutoTypes.inc").write_text(
    "\n".join(enum_source(prefix) for prefix in
              ["enum class AutoRun", "enum class CalPhase", "enum State", "enum PS3ButtonsMask"]), encoding="utf-8")
(output_dir / "AppAutoFunctions.inc").write_text(
    "\n".join(function(name) for name in
              ["printArmProgress", "printIkDiagnostic", "printPlanRejection", "validPose",
               "armConfigurationIssue", "armConfigurationReady", "autoMovesArm", "autoRespectsId", "autoReadinessIssue",
               "autoReady", "printReadinessFailure", "latchEmergency", "usesCamera",
               "controllerLossRequiresStop", "serviceImmediateControllerFaults",
               "serviceGlobalFaults", "beginStreamEnd", "selectNextPart", "serviceAutomaticMode",
               "serviceAlgorithmSelection", "finishCalibrationStream", "printVelocityCalibrationReport",
               "velocityLaneOccupied", "serviceVelocityCalibration", "serviceHomographyCalibration",
               "snapshotRuntimeSettings", "installRuntimeSettings", "printRuntimeStorageStatus",
               "loadRuntimeSettings", "printTuningRow", "applyTuningDraft", "printPickupTuningRow",
               "servicePickupTuning", "savePickupTuning", "serviceTuningMenu"]),
    encoding="utf-8")
command = [str(args.compiler), "-std=c++14", "-O2", "-mcpu=cortex-m4", "-mthumb",
           "-mfloat-abi=soft", "-fno-exceptions", "-fno-rtti", "-ffunction-sections",
           "-fdata-sections", "-nostartfiles", "--specs=nosys.specs",
           "-Wl,-e,test_main,-Ttext=0x10000,--gc-sections",
           "-I" + str(test_dir), "-I" + str(project), "-I" + str(args.bla),
           "-I" + str(output_dir),
           str(test_dir / "controller_tests.cpp"), str(project / "ArmTrajectory.cpp"),
           str(project / "DOF5_wServo_Kinematics.cpp"), str(project / "BeltTracker.cpp"),
           "-lm", "-o", str(elf_path)]
if args.settings_only:
    command.insert(1, "-DTEST_SETTINGS_ONLY")
if args.classification_only:
    command.insert(1,"-DTEST_CLASSIFICATION_ONLY")
if args.vision_diagnostics:
    command.insert(1, "-DTEST_VISION_DIAGNOSTICS_ONLY")
result = subprocess.run(command, capture_output=True, text=True)
if result.returncode:
    sys.stderr.write(result.stderr)
    raise SystemExit(result.returncode)

emulator = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
emulator.mem_map(0x10000, 0x400000)
emulator.mem_map(0x20000000, 0x100000)
with elf_path.open("rb") as source:
    elf = ELFFile(source)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
            emulator.mem_write(segment["p_vaddr"], segment.data())
    symbols = {symbol.name: symbol["st_value"]
               for symbol in elf.get_section_by_name(".symtab").iter_symbols()}

done = symbols["test_done"] & ~1
finished = False

def stop_at_done(uc, address, size, data):
    global finished
    if address == done:
        finished = True
        uc.emu_stop()

emulator.hook_add(UC_HOOK_CODE, stop_at_done, begin=done, end=done)
emulator.reg_write(UC_ARM_REG_SP, 0x200ff000)
emulator.reg_write(UC_ARM_REG_LR, done | 1)
# Full AUTO cycles plus entry/hold boundary sweeps need more emulator work.
# These limits belong only to this desktop runner, not the robot's watchdogs.
emulator.emu_start(symbols["test_main"] | 1, 0, timeout=180_000_000, count=3_000_000_000)
if not finished:
    progress = struct.unpack("<III", emulator.mem_read(symbols["test_result"], 12))
    print(f"Checks/failures/first at execution limit: {progress}")
    print(bytes(emulator.mem_read(symbols["test_context"],192)).split(b"\0",1)[0])
    pc = emulator.reg_read(UC_ARM_REG_PC)
    print(f"PC={pc:#x}; nearest symbol=" + str(max(
        ((address, name) for name,address in symbols.items() if address <= pc), default=None)))
    raise SystemExit("Controller tests did not finish within the execution limit")
checks, failures, first = struct.unpack("<III", emulator.mem_read(symbols["test_result"], 12))
print(f"ARM controller tests: {checks} checks, {failures} failures")
if failures:
    if "runtime_settings_debug" in symbols:
        print(bytes(emulator.mem_read(symbols["runtime_settings_debug"],1024)).split(b"\0",1)[0].decode("utf-8",errors="replace"))
    if "failure_codes" in symbols:
        print("Failing checks:", struct.unpack("<" + "I" * min(failures, 128),
              emulator.mem_read(symbols["failure_codes"], 4 * min(failures, 128))))
    if "test_context" in symbols:
        context = bytes(emulator.mem_read(symbols["test_context"], 192)).split(b"\0", 1)[0]
        if context:
            print(context.decode("utf-8", errors="replace"))
    raise SystemExit(f"First failing check: {first}; see controller_tests.cpp")
