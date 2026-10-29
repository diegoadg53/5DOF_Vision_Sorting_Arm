import contextlib
import importlib.util
import io
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

TOOL=Path(__file__).resolve().parents[1]/"joint_motion_report.py"
spec=importlib.util.spec_from_file_location("joint_motion_report",TOOL)
j=importlib.util.module_from_spec(spec); spec.loader.exec_module(j)


def sample(t, actual=(0,0,0), target=(0,0,0), slew=None, sequence=1, command=1,
           command_ms=0, cycle=0, stage=2, flags=1, mask=63, run=91, dropped=0):
    raw=(359.0,201.1,10.0,24.8,276.2)
    values=(cycle,command,command_ms,10,dropped,0,*target,*(slew or target),*actual,
            *raw,-201.1,-24.8,-276.2,0,0,0,*([0]*6),0,0,0,3,mask,stage)
    b=j.HEADER.pack(0x5aa5,1,1,152,flags,sequence,run,t&0xffffffff)+j.SAMPLE.pack(*values)
    return b+struct.pack("<I",zlib.crc32(b))


class ReportTests(unittest.TestCase):
    def test_real_firmware_packet(self):
        p=Path(tempfile.gettempdir())/"codex-nucleo-feedback-tests/diagnostic_frame.bin"
        self.assertTrue(p.exists(),"Run Nucleo Tests/run_tests.py first for the C-to-Python fixture")
        items=j.Decoder().feed(p.read_bytes())
        self.assertEqual(len(items),1)
        r=items[0]
        self.assertEqual((r["run"],r["cycle"],r["command"],r["stage_code"]),(91,1,1,4))
        self.assertEqual([r[f"target_{a}"] for a in j.AXES],[20,10,5])
        self.assertEqual([r[f"slew_{a}"] for a in j.AXES],[10,5,2])
        self.assertAlmostEqual(r["actual_pitch"],0,places=2)
        self.assertAlmostEqual(r["raw_joint_l"],201.1,delta=.012) # One encoder quantisation step.
        self.assertTrue(r["valid"])
        self.assertEqual(r["control_dt_ms"],12)

    def test_fragmented_stream_and_ascii(self):
        d=j.Decoder(); result=[]
        data=b"NUCLEO START\r\n"+sample(20)+sample(40,sequence=2)
        for byte in data: result+=d.feed(bytes([byte]))
        self.assertEqual(len(result),2)
        self.assertEqual(d.bad_crc,0)
        self.assertEqual(d.skipped_bytes,14)

    def test_corrupted_frame_recovers(self):
        bad=bytearray(sample(0)); bad[75]^=1
        d=j.Decoder(); result=d.feed(bad+sample(20,sequence=2))
        self.assertEqual(len(result),1)
        self.assertEqual(d.bad_crc,1)
        self.assertEqual(result[0]["sequence"],2)

    def test_unknown_version_and_partial_tail(self):
        bad=bytearray(sample(0)); bad[2]=9
        d=j.Decoder(); result=d.feed(bad+sample(20)+sample(40)[:17])
        self.assertEqual(len(result),1)
        self.assertGreater(d.bad_header,0)
        self.assertEqual(len(d.buffer),17)

    def test_metrics_and_overshoot(self):
        rows=j.Decoder().feed(b"".join(sample(t,
            actual=(0 if t==0 else 5 if t==20 else 11.5 if t==40 else 10.2,0,0),
            target=(10,0,0),sequence=t//20+1) for t in range(0,801,20)))
        r=j.summarize_axis(rows,"pitch",1,500,60,1000)
        self.assertTrue(r["settled"])
        self.assertEqual(r["settling_ms"],60)
        self.assertAlmostEqual(r["overshoot_deg"],1.5)
        self.assertAlmostEqual(r["steady_bias_deg"],.2,places=5)
        self.assertAlmostEqual(r["rmse_deg"],math.sqrt((100+25+2.25+38*.2**2)/41),places=5)
        self.assertGreater(r["rmse_deg"],r["steady_rmse_deg"])

    def test_unsettled_is_not_zero_settling_time(self):
        rows=j.Decoder().feed(b"".join(sample(t,actual=(3,0,0),sequence=t//20+1)
                                      for t in range(0,801,20)))
        r=j.summarize_axis(rows,"pitch",1,500,60,1000)
        self.assertFalse(r["settled"])
        self.assertIsNone(r["settling_ms"])
        self.assertEqual(r["settling_status"],"OUTSIDE_BAND")
        self.assertIsNone(r["steady_rmse_deg"])
        self.assertEqual(r["terminal_bias_deg"],3)

    def test_short_hold_is_insufficient(self):
        rows=j.Decoder().feed(b"".join(sample(t,sequence=t//20+1) for t in range(0,301,20)))
        r=j.summarize_axis(rows,"pitch",1,500,60,1000)
        self.assertFalse(r["settled"])
        self.assertEqual(r["settling_status"],"INSUFFICIENT_HOLD")

    def test_gap_invalidates_apparent_settling(self):
        rows=j.Decoder().feed(sample(0)+sample(800,sequence=2))
        r=j.summarize_axis(rows,"pitch",1,500,60,1000)
        self.assertFalse(r["settled"])
        self.assertTrue(r["has_gap"])

    def test_invalid_feedback_not_counted_as_zero_error(self):
        d=j.Decoder()
        rows=d.feed(sample(0,mask=47)+sample(20,actual=(float("nan"),0,0),sequence=2))
        self.assertFalse(any(r["valid"] for r in rows))
        self.assertEqual(j.summarize_axis(rows,"pitch",1,500,60,1000)["status"],"NO_VALID_SAMPLES")

    def test_paused_move_is_flagged(self):
        rows=j.Decoder().feed(b"".join(sample(t,stage=13 if t==100 else 2,sequence=t//20+1)
                                      for t in range(0,801,20)))
        r=j.summarize_axis(rows,"pitch",1,500,60,1000)
        self.assertFalse(r["settled"])
        self.assertTrue(r["paused"])

    def test_millis_wrap(self):
        start=0xffffff00
        rows=j.Decoder().feed(b"".join(sample(start+t,command_ms=start,sequence=t//20+1)
                                      for t in range(0,801,20)))
        r=j.summarize_axis(rows,"pitch",1,500,60,1000)
        self.assertTrue(r["settled"])
        self.assertEqual(r["observation_ms"],800)

    def test_config_decode(self):
        b=j.HEADER.pack(0x5aa5,1,2,152,0,1,91,0)+j.CONFIG.pack(123,0x20001,100,7,*range(23),20)
        r=j.Decoder().feed(b+struct.pack("<I",zlib.crc32(b)))[0]
        self.assertEqual(r["configuration"]["offset_q5"],22)
        self.assertEqual(r["sample_interval_ms"],20)

    def test_report_outputs_home_drift_and_sequence_gaps(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp); source=p/"capture.bin"
            source.write_bytes(b"".join(sample(i*20,sequence=i+1) for i in range(51))+
                b"".join(sample(2000+i*20,actual=(-1,2,.5),cycle=1,stage=12,
                    sequence=53+i,command=2,command_ms=2000) for i in range(51))+
                sample(3040,actual=(-1,2,.5),cycle=1,stage=15,flags=17,
                       command=2,command_ms=2000,sequence=104))
            with contextlib.redirect_stdout(io.StringIO()): summary,segments,homes=j.analyse(source,p/"out")
            self.assertTrue(summary["normal_completion"])
            self.assertEqual(summary["sequence_gaps"],1)
            drift={r["axis"]:r["drift_from_baseline_deg"] for r in homes if r["cycle"]==1}
            self.assertEqual(drift,{"pitch":-1,"roll":2,"elbow":.5,"l":3,"r":1})
            self.assertEqual(len(segments),10)
            for name in ("samples.csv","targets.csv","home_drift.csv","summary.json"):
                self.assertTrue((p/"out"/name).exists())

    def test_midstream_capture_does_not_invent_a_home_baseline(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp); source=p/"capture.bin"
            source.write_bytes(sample(1000,cycle=4,stage=12))
            with contextlib.redirect_stdout(io.StringIO()): summary,_,homes=j.analyse(source,p/"out")
            self.assertFalse(summary["end_seen"])
            self.assertTrue(all(r["drift_from_baseline_deg"] is None for r in homes))

    def test_sequence_loss_prevents_false_settling(self):
        rows=j.Decoder().feed(b"".join(sample(t,sequence=t//20+1+(1 if t>=100 else 0))
                                      for t in range(0,801,20)))
        self.assertFalse(j.summarize_axis(rows,"pitch",1,500,60,1000)["settled"])

    def test_live_capture_reads_only_and_uses_nucleo_baud(self):
        payload=sample(1000)+sample(1020,flags=17,stage=15,sequence=2)
        class Port:
            def __init__(self,port,baud,**kwargs):
                self.assertions=(port,baud); self.used=False
                if (port,baud)!=("FAKE",115200): raise AssertionError("Wrong port/baud")
            def __enter__(self): return self
            def __exit__(self,*_): pass
            def read(self,_):
                if self.used: return b""
                self.used=True; return payload
            def write(self,_): raise AssertionError("Capture must not send robot commands")
        with tempfile.TemporaryDirectory() as temp:
            args=SimpleNamespace(output=Path(temp)/"run",port="FAKE",duration=2,plots=False,
                                 tolerance=1,hold_ms=500,max_gap_ms=60,steady_ms=1000)
            with patch.dict(sys.modules,{"serial":SimpleNamespace(Serial=Port)}), \
                 contextlib.redirect_stdout(io.StringIO()):
                j.record(args)
            self.assertEqual((args.output/"capture.bin").read_bytes(),payload)
            self.assertTrue((args.output/"summary.json").exists())


if __name__=="__main__":
    unittest.main()
