"""Record Nucleo diagnostic schema 1; analyse without sending motion commands.

Offline analysis uses only Python's standard library. Live capture additionally
requires pyserial. Run --help for commands. Angles are degrees, times are ms.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import struct
import time
import zlib

SIZE = 152
HEADER = struct.Struct("<HBBHHIII")
SAMPLE = struct.Struct("<6I17f3h6H3I2BH2x")
CONFIG = struct.Struct("<4I23fI16x")
assert HEADER.size == 20 and SAMPLE.size == CONFIG.size == 128
STAGES = ["READY", "INITIAL_HOME", "BASELINE_HOME", "PLAN", "APPROACH", "DESCEND",
          "WAIT", "CLOSE", "CLASSIFICATION_WAYPOINT", "CLASSIFICATION_POSE", "OPEN",
          "FINAL_HOME", "HOME_DWELL", "PAUSED", "CANCEL_HOME", "COMPLETE", "FAULT"]
AXES = ("pitch", "roll", "elbow")
RAW = ("motor_l", "joint_l", "motor_r", "joint_r", "elbow")
CONFIG_VALUES = [f"{gain}_{axis}" for gain in ("kp", "ki", "kd") for axis in ("l", "r", "elbow")]
CONFIG_VALUES += ["friction_l", "friction_r", "friction_elbow", "gravity_lr", "gravity_elbow",
                  "weight_ratio", "deadband_deg", "slew_lr_deg_per_tick", "slew_elbow_deg_per_tick",
                  "offset_l", "offset_r", "offset_elbow", "offset_q4", "offset_q5"]


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = self.skipped_bytes = self.bad_header = 0
        self.last_sequence = {}

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while len(self.buffer) >= 2:
            at = self.buffer.find(b"\xa5\x5a")
            if at < 0:
                # Preserve a possible first sync byte across reads.
                keep = 1 if self.buffer[-1] == 0xa5 else 0
                self.skipped_bytes += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-1:] if keep else b""
                break
            if at:
                self.skipped_bytes += at
                del self.buffer[:at]
            if len(self.buffer) < HEADER.size:
                break
            h = HEADER.unpack_from(self.buffer)
            if h[1] != 1 or h[2] not in (1, 2) or h[3] != SIZE:
                self.bad_header += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < SIZE:
                break
            packet = bytes(self.buffer[:SIZE])
            if zlib.crc32(packet[:-4]) != struct.unpack_from("<I", packet, SIZE-4)[0]:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:SIZE]
            item = dict(type=h[2], flags=h[4], sequence=h[5], run=h[6], timestamp_ms=h[7])
            previous=self.last_sequence.get(h[6])
            difference=None if previous is None else elapsed(h[5],previous)
            item["sequence_gap_before"]=difference-1 if difference is not None and 1<difference<0x80000000 else 0
            item["sequence_anomaly"]=difference is not None and difference!=1
            self.last_sequence[h[6]]=h[5]
            if h[2] == 2:
                v = CONFIG.unpack_from(packet, HEADER.size)
                item.update(build=f"{v[0]:08X}", flash_schema=f"{v[1]:08X}",
                            calibration_source=v[2], calibration_generation=v[3],
                            sample_interval_ms=v[27], configuration=dict(zip(CONFIG_VALUES, v[4:27])))
            else:
                v = SAMPLE.unpack_from(packet, HEADER.size)
                item.update(zip(("cycle", "command", "command_ms", "control_dt_ms", "dropped",
                                 "calibration_generation"), v[:6]))
                for label, start, names in (("target", 6, AXES), ("slew", 9, AXES),
                                             ("actual", 12, AXES), ("raw", 15, RAW),
                                             ("offset", 20, ("l", "r", "elbow")),
                                             ("requested", 23, ("l", "r", "elbow")),
                                             ("age_ms", 26, (*RAW, "wrist")),
                                             ("submitted", 32, ("l", "r", "elbow"))):
                    item.update({f"{label}_{name}": v[start+i] for i, name in enumerate(names)})
                item.update(state=v[35], valid_mask=v[36], stage_code=v[37])
                item["stage"] = STAGES[v[37]] if v[37] < len(STAGES) else f"UNKNOWN_{v[37]}"
                # Safety/hold flags invalidate quantitative tracking metrics.
                item["valid"] = (v[36] & 0x1f) == 0x1f and not (h[4] & (2 | 4 | 8))
                item["valid"] &= all(math.isfinite(item[f"actual_{a}"]) for a in AXES)
                for label in ("target", "slew", "actual"):
                    item[f"{label}_l"] = item[f"{label}_roll"] - item[f"{label}_pitch"]
                    item[f"{label}_r"] = item[f"{label}_roll"] + item[f"{label}_pitch"]
                for axis in (*AXES, "l", "r"):
                    item[f"error_{axis}"] = item[f"actual_{axis}"] - item[f"target_{axis}"]
                    item[f"abs_error_{axis}"] = abs(item[f"error_{axis}"])
                    item[f"slew_error_{axis}"] = item[f"actual_{axis}"] - item[f"slew_{axis}"]
            out.append(item)
        return out


def elapsed(now, then):
    return (now-then) & 0xffffffff


def rms(values):
    return math.sqrt(statistics.fmean(x*x for x in values)) if values else None


def percentile(values, fraction):
    if not values:
        return None
    return sorted(values)[min(len(values)-1, math.ceil(fraction*len(values))-1)]


def summarize_axis(rows, axis, tolerance, hold_ms, maximum_gap_ms, steady_ms):
    valid = [r for r in rows if r["valid"] and r["flags"] & 1]
    if not valid:
        return dict(status="NO_VALID_SAMPLES", samples=0, settled=False, settling_ms=None)
    command_ms = rows[0]["command_ms"]
    errors = [r[f"error_{axis}"] for r in valid]
    slew_errors = [r[f"slew_error_{axis}"] for r in valid]
    last = rows[-1]["timestamp_ms"]
    steady = [r for r in valid if elapsed(last, r["timestamp_ms"]) <= steady_ms]
    tail = [r[f"error_{axis}"] for r in steady]
    terminal = tail
    # A settling result must be supported continuously through the end of
    # this command segment; gaps and invalid samples reset the candidate.
    candidate = None
    previous = None
    gap_seen = False
    for r in rows:
        gap = (previous is not None and elapsed(r["timestamp_ms"], previous) > maximum_gap_ms) or \
              r.get("sequence_anomaly",False)
        if gap:
            gap_seen = True
        previous = r["timestamp_ms"]
        if gap or not r["valid"] or not r["flags"] & 1 or abs(r[f"error_{axis}"]) > tolerance:
            candidate = None
        elif candidate is None:
            candidate = r["timestamp_ms"]
    settled = candidate is not None and elapsed(last, candidate) >= hold_ms
    paused = any(r["stage_code"] == 13 for r in rows)
    invalid = len(valid) != len(rows)
    # Direction is relative to the first observed position, not inferred velocity.
    target = valid[0][f"target_{axis}"]
    delta = target-valid[0][f"actual_{axis}"]
    overshoot = max(0.0, max(errors)) if delta > 0 else max(0.0, -min(errors)) if delta < 0 else None
    complete = not paused and not invalid and not gap_seen
    steady = [r for r in steady if settled and complete and
              elapsed(r["timestamp_ms"],command_ms)>=elapsed(candidate,command_ms)]
    tail = [r[f"error_{axis}"] for r in steady]
    settling_status = "SETTLED" if settled and complete else "PAUSED" if paused else \
        "INCOMPLETE" if not complete else "INSUFFICIENT_HOLD" if candidate is not None else "OUTSIDE_BAND"
    return dict(status="OK" if complete else "PAUSED_OR_INCOMPLETE", samples=len(valid),
                target_deg=target, signed_mean_error_deg=statistics.fmean(errors),
                mae_deg=statistics.fmean(abs(e) for e in errors), rmse_deg=rms(errors),
                slew_tracking_rmse_deg=rms(slew_errors), peak_abs_error_deg=max(map(abs, errors)),
                final_error_deg=errors[-1], final_abs_error_deg=abs(errors[-1]),
                steady_bias_deg=statistics.fmean(tail) if tail else None,
                terminal_bias_deg=statistics.fmean(terminal) if terminal else None,
                terminal_rmse_deg=rms(terminal), settling_status=settling_status,
                steady_rmse_deg=rms(tail), steady_peak_to_peak_deg=max(tail)-min(tail) if tail else None,
                overshoot_deg=overshoot, settled=settled and complete,
                settling_ms=elapsed(candidate,command_ms) if settled and complete else None,
                observation_ms=elapsed(last,command_ms), has_gap=gap_seen, paused=paused)


def write_csv(path, rows):
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w",newline="",encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def make_plots(samples, homes, output, maximum_gap_ms):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Plots skipped: install matplotlib; CSV/JSON reports are complete.")
        return
    for run in sorted({r["run"] for r in samples}):
        rows=[r for r in samples if r["run"]==run]
        times=[elapsed(r["timestamp_ms"],rows[0]["timestamp_ms"])/1000 for r in rows]
        fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
        for ax,axis in zip(axes,AXES):
            ax.step(times,[r[f"target_{axis}"] for r in rows],where="post",label="Requested",color="black",alpha=.65)
            ax.plot(times,[r[f"slew_{axis}"] for r in rows],label="Slew reference",color="orange",linewidth=1)
            actual=[r[f"actual_{axis}"] if r["valid"] and not r["sequence_anomaly"] and
                    (i==0 or elapsed(r["timestamp_ms"],rows[i-1]["timestamp_ms"])<=maximum_gap_ms)
                    else math.nan for i,r in enumerate(rows)]
            ax.plot(times,actual,
                    label="Encoder",color="tab:blue",linewidth=1)
            ax.set_ylabel(f"{axis.title()} (deg)"); ax.grid(alpha=.25)
        axes[0].legend(loc="best",ncol=3); axes[-1].set_xlabel("Nucleo elapsed time (s)")
        fig.suptitle(f"Run {run}: requested, slewed and measured joint coordinates")
        fig.tight_layout(); fig.savefig(output/f"run_{run}_angles.png",dpi=150); plt.close(fig)
        fig,ax=plt.subplots(figsize=(10,5))
        for axis in (*AXES,"l","r"):
            h=[r for r in homes if r["run"]==run and r["axis"]==axis and r["drift_from_baseline_deg"] is not None]
            if h: ax.plot([r["cycle"] for r in h],[r["drift_from_baseline_deg"] for r in h],"o-",
                          label={"l":"Left bevel","r":"Right bevel"}.get(axis,axis))
        ax.axhline(0,color="black",linewidth=.7); ax.grid(alpha=.25)
        ax.set(xlabel="Completed cycle (0 = initial HOME)",ylabel="Encoder coordinate change from baseline (deg)",
               title=f"Run {run}: HOME drift (external level not measured)")
        ax.legend(); fig.tight_layout(); fig.savefig(output/f"run_{run}_home_drift.png",dpi=150); plt.close(fig)


def analyse(source, output, tolerance=1.0, hold_ms=500, maximum_gap_ms=60, steady_ms=1000, plots=False):
    output.mkdir(parents=True, exist_ok=True)
    decoder = Decoder()
    packets = []
    with source.open("rb") as f:
        while data := f.read(65536):
            packets.extend(decoder.feed(data))
    samples = [p for p in packets if p["type"] == 1]
    if not samples:
        raise ValueError("No valid schema-1 diagnostic samples found")
    # Unwrap only continuously observed raw channels. Never guess turns across
    # a missing/invalid interval; a segment number exposes each reset.
    raw_history = {}
    raw_segments = Counter()
    for r in samples:
        for i,name in enumerate(RAW):
            key=(r["run"],name)
            value=r[f"raw_{name}"]
            fresh=bool(r["valid_mask"] & (1<<i)) and r[f"age_ms_{name}"]<=50 and math.isfinite(value)
            previous=raw_history.get(key)
            if not fresh:
                raw_history.pop(key,None)
                r[f"unwrapped_{name}"]=None
            elif previous is None or elapsed(r["timestamp_ms"],previous[0])>maximum_gap_ms:
                raw_segments[key]+=1
                r[f"unwrapped_{name}"]=value
                raw_history[key]=(r["timestamp_ms"],value,value)
            else:
                unwrapped=previous[2]+((value-previous[1]+180)%360-180)
                r[f"unwrapped_{name}"]=unwrapped
                raw_history[key]=(r["timestamp_ms"],value,unwrapped)
            r[f"unwrap_segment_{name}"]=raw_segments[key]
    # Retain raw packets for audit; CSV includes calibrated coordinate errors.
    write_csv(output/"samples.csv", samples)
    configurations = [p for p in packets if p["type"] == 2]
    groups = defaultdict(list)
    for r in samples:
        if r["command"]:
            groups[r["run"],r["command"]].append(r)
    segments = []
    for (run,command), rows in groups.items():
        cycle=Counter(r["cycle"] for r in rows).most_common(1)[0][0]
        stage = Counter(r["stage"] for r in rows).most_common(1)[0][0]
        for axis in (*AXES, "l", "r"):
            m = summarize_axis(rows,axis,tolerance,hold_ms,maximum_gap_ms,steady_ms)
            m.update(run=run,cycle=cycle,command=command,stage=stage,axis=axis)
            segments.append(m)
    write_csv(output/"targets.csv", segments)
    homes = []
    baseline = {}
    for run,cycle in sorted({(r["run"],r["cycle"]) for r in samples}):
        dwell = [r for r in samples if r["run"]==run and r["cycle"]==cycle and r["stage_code"] in (2,12)]
        if not dwell:
            continue
        last = dwell[-1]["timestamp_ms"]
        window = [r for r in dwell if elapsed(last,r["timestamp_ms"])<=steady_ms]
        tail = [r for r in window if r["valid"] and r["flags"] & 1]
        if not tail:
            continue
        span=elapsed(tail[-1]["timestamp_ms"],tail[0]["timestamp_ms"])
        steady_valid = span>=hold_ms and len(tail)==len(window) and all(
            not r["sequence_anomaly"] and elapsed(r["timestamp_ms"],window[i-1]["timestamp_ms"])<=maximum_gap_ms
            for i,r in enumerate(window) if i)
        for axis in (*AXES, "l", "r"):
            values = [r[f"actual_{axis}"] for r in tail]
            mean = statistics.fmean(values)
            if cycle==0 and steady_valid:
                baseline[run,axis] = mean
            reference = baseline.get((run,axis))
            homes.append(dict(run=run,cycle=cycle,axis=axis,samples=len(values),
                mean_deg=mean,std_deg=statistics.pstdev(values),
                peak_to_peak_deg=max(values)-min(values),
                drift_from_baseline_deg=None if reference is None or not steady_valid else mean-reference,
                steady_window_valid=steady_valid,steady_span_ms=span,
                dwell_span_ms=elapsed(dwell[-1]["timestamp_ms"],dwell[0]["timestamp_ms"])))
    write_csv(output/"home_drift.csv", homes)
    gaps = 0
    previous = {}
    for p in packets:
        if p["run"] in previous:
            difference = elapsed(p["sequence"],previous[p["run"]])
            if 1<difference<0x80000000:
                gaps += difference-1
        previous[p["run"]] = p["sequence"]
    offsets = {(r["run"],r["offset_l"],r["offset_r"],r["offset_elbow"],
                r["calibration_generation"]) for r in samples}
    loop_dt = [r["control_dt_ms"] for r in samples if r["control_dt_ms"]]
    summary = dict(schema=1, source=str(source.resolve()), samples=len(samples),
        configurations=configurations, bad_crc=decoder.bad_crc, bad_header=decoder.bad_header,
        skipped_bytes=decoder.skipped_bytes, trailing_bytes=len(decoder.buffer),
        sequence_gaps=gaps, device_dropped=max(r["dropped"] for r in samples),
        invalid_samples=sum(not r["valid"] for r in samples),
        end_seen=any(r["flags"] & 16 for r in samples),
        normal_completion=any(r["flags"] & 16 and r["stage_code"]==15 and
                              not r["flags"] & (2|32) for r in samples),
        logger_lease_ended=any(r["flags"] & 32 for r in samples),
        offset_versions=[list(v) for v in sorted(offsets)],
        sampled_control_dt_ms=dict(median=statistics.median(loop_dt) if loop_dt else None,
                                   p95=percentile(loop_dt,.95),maximum=max(loop_dt) if loop_dt else None),
        metric_settings=dict(tolerance_deg=tolerance,hold_ms=hold_ms,maximum_gap_ms=maximum_gap_ms,
                             steady_window_ms=steady_ms),
        limitations=["Errors are against sensor coordinates; physical absolute accuracy requires an independent reference.",
                     "50 Hz samples give approximately 20 ms timing resolution; sequence/CRC gaps reduce confidence.",
                     "Motor UART submission counters show MCU transmission attempts, not actuator acknowledgements.",
                     "Runtime control-dt statistics are sampled, not an exhaustive log of every control iteration.",
                     "Only pitch, roll, elbow and derived bevel coordinates are measured in this report."])
    summary["quality_warnings"]=[]
    versions=defaultdict(set)
    for version in offsets: versions[version[0]].add(version[1:])
    for bad,message in ((not configurations,"Configuration metadata missing"),
                        (not summary["end_seen"],"Capture has no END marker"),
                        (summary["logger_lease_ended"],"Recorder keepalive expired"),
                        (any(len(v)>1 for v in versions.values()),"Offsets/calibration generation changed within a run"),
                        (summary["bad_crc"]>0,"CRC failures"),
                        (any(p["sequence_anomaly"] for p in packets),"Missing/duplicate/out-of-order frames"),
                        (summary["invalid_samples"]>0,"Invalid/held/faulted feedback samples")):
        if bad: summary["quality_warnings"].append(message)
    summary["quality_ok"]=not summary["quality_warnings"]
    (output/"summary.json").write_text(json.dumps(summary,indent=2,allow_nan=False),encoding="utf-8")
    if plots: make_plots(samples,homes,output,maximum_gap_ms)
    print(f"{len(samples)} samples; CRC errors={decoder.bad_crc}, sequence gaps={gaps}; END={summary['end_seen']}")
    for warning in summary["quality_warnings"]: print(f"Quality warning: {warning}")
    print(f"Reports: {output.resolve()}")
    return summary,segments,homes


def record(args):
    try:
        import serial
    except ImportError as e:
        raise SystemExit("Live capture requires pyserial: python -m pip install pyserial") from e
    output = args.output or Path("joint_motion_runs")/datetime.now().strftime("%Y%m%d-%H%M%S")
    output.mkdir(parents=True,exist_ok=True)
    raw = output/"capture.bin"
    if raw.exists():
        raise SystemExit(f"Refusing to overwrite {raw}; choose a new output folder")
    decoder = Decoder()
    start=time.monotonic(); finished_at=None; last_marker=None
    print("Open Portenta JOINT MOTION TEST and press A. Controller B remains emergency stop.")
    try:
        with serial.Serial(args.port,115200,timeout=.1,write_timeout=.1) as port, raw.open("wb") as f:
            while time.monotonic()-start < args.duration:
                data=port.read(4096)
                if data:
                    f.write(data)
                    for p in decoder.feed(data):
                        if p["type"]==1:
                            marker=(p["run"],p["cycle"],p["stage"])
                            if marker!=last_marker:
                                print(f"run={marker[0]} cycle={marker[1]} stage={marker[2]}")
                                last_marker=marker
                            if p["flags"] & 16:
                                finished_at=time.monotonic()
                if finished_at is not None and time.monotonic()-finished_at>.3:
                    break
    except KeyboardInterrupt:
        print("Capture interrupted. This does not stop robot motion; use the controller.")
    analyse(raw,output,args.tolerance,args.hold_ms,args.max_gap_ms,args.steady_ms,args.plots)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="command",required=True)
    for name in ("record","analyse"):
        p=sub.add_parser(name)
        p.add_argument("--output",type=Path)
        p.add_argument("--tolerance",type=float,default=1.0)
        p.add_argument("--hold-ms",type=int,default=500)
        p.add_argument("--max-gap-ms",type=int,default=60)
        p.add_argument("--steady-ms",type=int,default=1000)
        p.add_argument("--plots",action="store_true",help="Export PNG charts if matplotlib is installed")
        if name=="record":
            p.add_argument("--port",required=True)
            p.add_argument("--duration",type=float,default=900)
        else:
            p.add_argument("source",type=Path)
    args=parser.parse_args()
    if args.tolerance<=0 or min(args.hold_ms,args.max_gap_ms,args.steady_ms)<=0:
        parser.error("Tolerance and time windows must be positive")
    if args.command=="record":
        record(args)
    else:
        analyse(args.source,args.output or args.source.parent,args.tolerance,args.hold_ms,
                args.max_gap_ms,args.steady_ms,args.plots)


if __name__=="__main__":
    main()
