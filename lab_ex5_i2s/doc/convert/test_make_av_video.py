"""Contract tests. Set AV_TEST_FFMPEG to enable real FFmpeg integration tests.

All fixtures are generated locally. No downloads, installs or existing media
are used. AV_TEST_TMP may point to an owned task-artifact directory.
"""

import contextlib
import io
import importlib.util
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import tracemalloc
import unittest
from unittest import mock
import wave

import make_av_video as av


FFMPEG = os.environ.get("AV_TEST_FFMPEG") or shutil.which("ffmpeg")


class FixtureCase(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="av-test-", dir=os.environ.get("AV_TEST_TMP"))
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / "out.VID"

    def vid1(self, frames=2, fps=20):
        path = self.root / "old.VID"
        header = bytearray(512)
        struct.pack_into("<4sHHHI", header, 0, b"VID1", 320, 240, fps, frames)
        # Nonuniform rows distinguish a byte copy from a second vertical flip.
        pixels = b"".join(struct.pack("<H", y * 127) * 320 for y in range(240))
        with path.open("wb") as out:
            out.write(header)
            for n in range(frames):
                out.write(pixels if n % 2 == 0 else pixels[::-1])
        return path

    def wav(self, pcm, channels=2, width=2, rate=48000):
        path = self.root / "audio.wav"
        with wave.open(str(path), "wb") as out:
            out.setparams((channels, width, rate, 0, "NONE", "not compressed"))
            out.writeframes(pcm)
        return path

    def record(self, n):
        info = av.inspect_file(self.output)
        with self.output.open("rb") as source:
            source.seek(512 + n * info["frame_bytes"])
            return source.read(av.VIDEO_BYTES), source.read(info["audio_sectors"] * 512)


class ContractTests(FixtureCase):
    def test_header_exact_offsets_all_fps(self):
        for fps, spf, sectors in ((12, 4000, 32), (15, 3200, 25), (20, 2400, 19),
                                  (24, 2000, 16), (25, 1920, 15)):
            with self.subTest(fps=fps):
                head = av.make_header(fps, 0x01020304)
                self.assertEqual(len(head), 512)
                # Construct independently, by offsets from the hardware contract.
                expected = bytearray(512)
                expected[0:4] = b"VID2"
                for offset, fmt, value in ((4, "H", 320), (6, "H", 240),
                        (8, "H", fps), (10, "I", 0x01020304), (14, "H", sectors),
                        (16, "I", 48000), (20, "H", spf), (22, "B", 2),
                        (23, "B", 16), (24, "H", 300), (26, "H", 1)):
                    struct.pack_into("<" + fmt, expected, offset, value)
                self.assertEqual(head, expected)
                self.assertEqual(av.parse_header(head)["samples_per_frame"], spf)

    def test_header_rejects_each_invalid_field_and_reserved_bytes(self):
        for offset in (0, 4, 6, 8, 10, 14, 16, 20, 22, 23, 24, 26, 28, 511):
            with self.subTest(offset=offset):
                head = bytearray(av.make_header(20, 1))
                head[offset] = 0 if offset == 10 else head[offset] ^ 0x80
                with self.assertRaises(ValueError):
                    av.parse_header(head)
        for fps in (0, 16, 30, 60):
            with self.assertRaises(ValueError):
                av.make_header(fps, 1)
        for frames in (0, -1, 2**32):
            with self.assertRaises(ValueError):
                av.make_header(20, frames)
        with self.assertRaises(ValueError):
            av.parse_header(b"VID2")

    def test_vid1_exact_video_pcm_stereo_endian_and_tail_padding(self):
        video = self.vid1()
        samples = 2400 * 2
        pcm = b"".join(struct.pack("<hh", i - 4000, 12000 - i) for i in range(samples))
        wav = self.wav(pcm)
        av.package_vid1(video, wav, self.output)
        original = video.read_bytes()[512:]
        for n in range(2):
            pixels, audio = self.record(n)
            self.assertEqual(pixels, original[n*153600:(n+1)*153600])
            self.assertEqual(audio[:9600], pcm[n*9600:(n+1)*9600])
            self.assertEqual(audio[9600:], bytes(128))
        self.assertEqual(self.output.stat().st_size, 512 + 2 * (300 + 19) * 512)

    def test_short_audio_partial_frame_and_later_frames_are_silence(self):
        video = self.vid1(frames=3)
        pcm = struct.pack("<hh", -12345, 23456) * 2501
        av.package_vid1(video, self.wav(pcm), self.output)
        self.assertEqual(self.record(0)[1][:9600], pcm[:9600])
        self.assertEqual(self.record(1)[1], pcm[9600:] + bytes(19*512 - 404))
        self.assertEqual(self.record(2)[1], bytes(19*512))

    def test_empty_audio(self):
        av.package_vid1(self.vid1(), self.wav(b""), self.output)
        self.assertEqual(self.record(0)[1], bytes(19*512))

    def test_long_audio_truncated_to_video(self):
        pcm = struct.pack("<hh", -32768, 32767) * (3*2400)
        av.package_vid1(self.vid1(), self.wav(pcm), self.output)
        self.assertEqual(av.inspect_file(self.output)["frames"], 2)
        self.assertEqual(self.record(1)[1][:9600], pcm[9600:19200])

    def test_all_fps_record_layout_and_vid1_rate_inheritance(self):
        for fps in (12, 15, 20, 24, 25):
            with self.subTest(fps=fps):
                spf = 48000 // fps
                pcm = struct.pack("<hh", 0x1234, -0x2345) * spf
                av.package_vid1(self.vid1(frames=1, fps=fps), self.wav(pcm),
                                self.output, force=True)
                _, audio = self.record(0)
                self.assertEqual(audio[:spf*4], pcm)
                self.assertEqual(audio[spf*4:], bytes(len(audio) - spf*4))
                self.assertEqual(av.inspect_file(self.output)["fps"], fps)

    def test_mono_wrong_width_and_wrong_rate_rejected(self):
        video = self.vid1()
        for kwargs in ({"channels": 1}, {"width": 1}, {"rate": 44100}):
            with self.subTest(kwargs=kwargs), self.assertRaisesRegex(ValueError, "WAV must"):
                av.package_vid1(video, self.wav(bytes(100), **kwargs), self.output)
        self.assertFalse(self.output.exists())

    def test_vid1_bad_header_size_and_rate_override(self):
        video = self.vid1()
        audio = self.wav(b"")
        with self.assertRaisesRegex(ValueError, "fps cannot"):
            av.package_vid1(video, audio, self.output, fps=25)
        for change in (lambda b: b[:511], lambda b: b[:-1], lambda b: b + b"x",
                       lambda b: b"VID2" + b[4:],
                       lambda b: b[:4] + struct.pack("<H", 640) + b[6:]):
            video = self.vid1()
            video.write_bytes(change(video.read_bytes()))
            with self.assertRaises(ValueError):
                av.package_vid1(video, audio, self.output)
        self.assertFalse(self.output.exists())

    def test_inspect_detects_short_extra_bytes_and_nonzero_padding(self):
        av.package_demo(.05, self.output, fps=20)
        valid = self.output.read_bytes()
        for bad in (valid[:-1], valid + b"x", valid[:-1] + b"x", valid[:511]):
            self.output.write_bytes(bad)
            with self.assertRaises(ValueError):
                av.inspect_file(self.output)

    def test_output_protection_extension_alias_and_force(self):
        video = self.vid1()
        audio = self.wav(b"")
        sentinel = b"keep existing output"
        self.output.write_bytes(sentinel)
        with self.assertRaises(FileExistsError):
            av.package_vid1(video, audio, self.output)
        self.assertEqual(self.output.read_bytes(), sentinel)
        for name in ("bad.vid", "bad.bin"):
            with self.assertRaises(ValueError):
                av.package_vid1(video, audio, self.root / name)
        original = video.read_bytes()
        with self.assertRaisesRegex(ValueError, "input"):
            av.package_vid1(video, audio, video, force=True)
        alias = self.root / "alias.VID"
        os.link(video, alias)
        with self.assertRaisesRegex(ValueError, "input"):
            av.package_vid1(video, audio, alias, force=True)
        self.assertEqual(video.read_bytes(), original)
        av.package_vid1(video, audio, self.output, force=True)
        self.assertEqual(av.inspect_file(self.output)["fps"], 20)

    def test_failed_write_preserves_output_and_removes_only_owned_temp(self):
        self.output.write_bytes(b"existing")
        unrelated = self.root / ".vid2-unrelated.tmp"
        unrelated.write_bytes(b"do not touch")
        before = set(self.root.iterdir())
        with self.assertRaisesRegex(ValueError, "truncated video"):
            with av.atomic_output(self.output, True) as output:
                av.write_records(output, io.BytesIO(b"short"), io.BytesIO(), 20, 1)
        self.assertEqual(self.output.read_bytes(), b"existing")
        self.assertEqual(unrelated.read_bytes(), b"do not touch")
        self.assertEqual(set(self.root.iterdir()), before)

    def test_atomic_no_clobber_if_output_appears_after_preflight(self):
        av.validate_output(self.output)
        with self.assertRaises(FileExistsError):
            with av.atomic_output(self.output, False) as output:
                av.write_records(output, io.BytesIO(bytes(153600)), io.BytesIO(), 20, 1)
                self.output.write_bytes(b"another writer")
        self.assertEqual(self.output.read_bytes(), b"another writer")
        self.assertEqual(list(self.root.iterdir()), [self.output])

    def test_bounded_reads_short_reads_and_partial_stereo_sample(self):
        class SmallReader(io.BytesIO):
            def read(self, count=-1):
                if not 0 <= count <= 153600:
                    raise AssertionError("unbounded or oversized read")
                return super().read(min(count, 17))
        output = io.BytesIO()
        av.write_records(output, SmallReader(bytes(153600)),
                         SmallReader(struct.pack("<hh", 11, -22) * 2400), 20, 1)
        self.assertEqual(output.getvalue()[512+153600:][:4], struct.pack("<hh", 11, -22))
        with self.assertRaisesRegex(ValueError, "stereo sample"):
            av.write_records(io.BytesIO(), io.BytesIO(bytes(153600)), io.BytesIO(b"x"), 20, 1)

    def test_streaming_memory_does_not_scale_with_movie_length(self):
        class ZeroSource:
            def read(self, count):
                if not 0 <= count <= 153600:
                    raise AssertionError("unbounded input read")
                return bytes(count)

        class CountingSink:
            count = 0

            def write(self, data):
                self.count += len(data)

        sink = CountingSink()
        tracemalloc.start()
        try:
            av.write_records(sink, ZeroSource(), ZeroSource(), 20, 2000)
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        # Simulate 326MB of output while keeping the Python peak below 2MB.
        self.assertEqual(sink.count, 512 + 2000 * 319 * 512)
        self.assertLess(peak, 2_000_000)

    def test_demo_synchronized_flash_beep_and_fpga_row_order(self):
        av.package_demo(1.1, self.output)
        self.assertEqual(av.inspect_file(self.output)["fps"], 12)
        self.assertEqual(av.inspect_file(self.output)["frames"], 14)
        for frame, flash in ((0, True), (1, False), (11, False), (12, True), (13, False)):
            pixels, pcm = self.record(frame)
            self.assertEqual(pixels[-8*320*2:], b"\xff\xff" * (8*320))
            color = 0xF800 if frame == 0 else (0x07E0 if frame == 12 else 0)
            self.assertEqual(pixels[:2], struct.pack("<H", color))
            self.assertEqual(any(pcm[:9600]), flash)
            self.assertEqual(pcm[9600:16000], bytes(6400))
            self.assertEqual(pcm[16000:], bytes(384))
            self.assertEqual(pcm[0:2], pcm[2:4])
        for seconds in (0, -1, float("nan"), float("inf"), 1e308):
            with self.assertRaises(ValueError):
                av.package_demo(seconds, self.root / "invalid.VID")

    def test_cli_demo_and_inspect_without_external_programs(self):
        with mock.patch.object(av.shutil, "which", side_effect=AssertionError("external lookup")):
            with contextlib.redirect_stdout(io.StringIO()) as captured:
                self.assertEqual(av.main(["--demo", "0.05", "--out", str(self.output)]), 0)
            default_info = json.loads(captured.getvalue())
            self.assertEqual(default_info["fps"], 12)
            self.assertEqual(default_info["samples_per_frame"], 4000)
            self.assertEqual(default_info["audio_sectors"], 32)
            with contextlib.redirect_stdout(io.StringIO()) as captured:
                self.assertEqual(av.main(["--inspect", str(self.output)]), 0)
            self.assertEqual(json.loads(captured.getvalue())["flags"], 1)
        with mock.patch.object(av, "package_source", return_value={}) as package:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(av.main(["--input", "source.mkv", "--out", str(self.output)]), 0)
            self.assertEqual(package.call_args.args[2], 12)
        with contextlib.redirect_stdout(io.StringIO()) as help_output:
            with self.assertRaises(SystemExit):
                av.main(["--help"])
        self.assertIn("25MHz", help_output.getvalue())
        self.assertIn("higher throughput", help_output.getvalue())

    def test_cli_errors_and_missing_binary(self):
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(av.main(["--inspect", str(self.output)]), 1)
            for args in (["--vid1", "file"], ["--demo", "1", "--wav", "file"],
                         ["--demo", "1", "--fps", "30"]):
                with self.assertRaises(SystemExit) as raised:
                    av.main(args)
                self.assertEqual(raised.exception.code, 2)
        with mock.patch.object(av.shutil, "which", return_value=None):
            with self.assertRaisesRegex(ValueError, "supply --ffmpeg"):
                av.resolve_program(None, "ffmpeg")


class CardDiagnosticCompatibilityTests(FixtureCase):
    def setUp(self):
        super().setUp()
        tool = Path(__file__).resolve().parent.parent / "tools" / "check_sd_card.py"
        spec = importlib.util.spec_from_file_location("_card_check_local_test", tool)
        self.checker = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.checker)

    def test_vid2_local_fixture_return_contract_without_card_access(self):
        av.package_demo(.1, self.output)
        previous_path = list(sys.path)
        with mock.patch.object(self.checker, "boot_sector_info", side_effect=AssertionError("card access")):
            with mock.patch.object(self.checker.os, "listdir", side_effect=AssertionError("card scan")):
                message, info = self.checker.check_vid(self.output)
        self.assertTrue(message.startswith("OK"), message)
        self.assertEqual(info, (320, 240, 12, 2))
        self.assertEqual(sys.path, previous_path)

    def test_vid2_corrupt_header_size_and_audio_padding_report_failure(self):
        av.package_demo(.05, self.output, fps=20)
        valid = self.output.read_bytes()
        invalid_header = bytearray(valid)
        invalid_header[24] = 0
        for bad in (invalid_header, valid[:-1], valid + b"x", valid[:-1] + b"x"):
            with self.subTest(size=len(bad)):
                self.output.write_bytes(bad)
                message, info = self.checker.check_vid(self.output)
                self.assertIn("VID2", message)
                self.assertFalse(message.startswith("OK"))
                self.assertIsNone(info)

    def test_existing_vid1_path_and_local_comparison_still_work(self):
        video = self.vid1(fps=25)
        message, info = self.checker.check_vid(video)
        self.assertEqual(message, "OK")
        self.assertEqual(info, (320, 240, 25, 2))
        message, info = self.checker.check_vid(video, video)
        self.assertIn("完全相同", message)
        self.assertEqual(info, (320, 240, 25, 2))


@unittest.skipUnless(FFMPEG, "set AV_TEST_FFMPEG to an installed executable")
class FFmpegTests(FixtureCase):
    def run_ffmpeg(self, *args):
        result = subprocess.run([FFMPEG, "-nostdin", "-hide_banner", "-loglevel", "error",
                                 "-y", *map(str, args)], capture_output=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))

    def source(self, audio_samples=9600, audio_delay="0.2", global_offset="5", mono=False,
               video_delay="0", rate=48000):
        # Paths deliberately contain spaces and shell metacharacters.
        source = self.root / "real source & fixture.mkv"
        raw_video = self.root / "pixels.rgb"
        raw_audio = self.root / "samples.pcm"
        top = bytes((255, 0, 0)) * (320*120)
        bottom = bytes((0, 0, 255)) * (320*120)
        raw_video.write_bytes((top + bottom) * 12)
        if mono:
            pcm = struct.pack("<h", 6000) * audio_samples
        else:
            pcm = b"".join(struct.pack("<hh", 1000+i % 1000, -2000-i % 1000)
                           for i in range(audio_samples))
        raw_audio.write_bytes(pcm)
        self.run_ffmpeg("-itsoffset", video_delay, "-f", "rawvideo", "-pixel_format", "rgb24",
                        "-video_size", "320x240", "-framerate", "20", "-i", raw_video,
                        "-itsoffset", audio_delay, "-f", "s16le", "-ar", str(rate),
                        "-ac", "1" if mono else "2", "-i", raw_audio,
                        "-map", "0:v:0", "-map", "1:a:0", "-c:v", "ffv1", "-level", "3",
                        "-pix_fmt", "bgr0", "-c:a", "pcm_s16le", "-fps_mode", "passthrough",
                        "-output_ts_offset", global_offset, source)
        return source, pcm

    def test_real_input_shared_nonzero_origin_audio_delay_padding_vflip_bytes(self):
        source, pcm = self.source()
        info = av.package_source(source, self.output, ffmpeg=FFMPEG)
        self.assertEqual(info["fps"], 12)
        self.assertEqual(info["frames"], 7)
        all_audio = bytearray()
        for n in range(7):
            pixels, audio = self.record(n)
            # Input red top/blue bottom -> encoded blue top/red bottom.
            self.assertEqual(pixels[:320*120*2], b"\x1f\x00" * (320*120))
            self.assertEqual(pixels[320*120*2:], b"\x00\xf8" * (320*120))
            self.assertEqual(len(audio), 16384)
            self.assertEqual(audio[16000:], bytes(384))
            all_audio.extend(audio[:16000])
        expected = (bytes(9600*4) + pcm).ljust(7*16000, b"\0")
        self.assertEqual(all_audio, expected)

    def test_real_long_audio_truncated(self):
        source, pcm = self.source(audio_samples=48000, audio_delay="0", global_offset="0")
        info = av.package_source(source, self.output, fps=20, ffmpeg=FFMPEG)
        self.assertEqual(info["frames"], 12)
        for n in range(12):
            self.assertEqual(self.record(n)[1][:9600], pcm[n*9600:(n+1)*9600])

    def test_real_mono_converted_to_stereo(self):
        source, _ = self.source(audio_samples=28800, audio_delay="0", mono=True)
        av.package_source(source, self.output, fps=20, ffmpeg=FFMPEG)
        pcm = self.record(0)[1][:9600]
        pairs = list(struct.iter_unpack("<hh", pcm))
        self.assertTrue(all(left == right and left > 0 for left, right in pairs))

    def test_real_44100_audio_resampled_to_48000(self):
        source, _ = self.source(audio_samples=26460, audio_delay="0", rate=44100)
        info = av.package_source(source, self.output, fps=20, ffmpeg=FFMPEG)
        self.assertEqual(info["sample_rate"], 48000)
        self.assertEqual(info["frames"], 12)
        for n in range(12):
            pairs = list(struct.iter_unpack("<hh", self.record(n)[1][:9600]))
            self.assertTrue(all(left > 0 and right < 0 for left, right in pairs))

    def test_real_video_only_rejected_without_inventing_audio(self):
        source, _ = self.source()
        silent = self.root / "no_audio.mkv"
        self.run_ffmpeg("-i", source, "-an", "-c:v", "copy", silent)
        before = set(self.root.iterdir())
        with self.assertRaisesRegex(ValueError, "failed"):
            av.package_source(silent, self.output, ffmpeg=FFMPEG)
        self.assertEqual(set(self.root.iterdir()), before)
        self.assertFalse(self.output.exists())

    def test_real_all_supported_fps(self):
        source, _ = self.source(audio_samples=28800, audio_delay="0", global_offset="0")
        for fps, frames in ((12, 7), (15, 9), (20, 12), (24, 14), (25, 15)):
            with self.subTest(fps=fps):
                info = av.package_source(source, self.output, fps=fps, ffmpeg=FFMPEG, force=True)
                self.assertEqual(info["fps"], fps)
                self.assertEqual(info["frames"], frames)
                audio_bytes = (48000 // fps) * 4
                for n in range(frames):
                    _, audio = self.record(n)
                    self.assertEqual(audio[audio_bytes:], bytes(len(audio) - audio_bytes))
                if fps == 12:
                    self.assertEqual(info["samples_per_frame"], 4000)
                    self.assertEqual(info["audio_sectors"], 32)
                    self.assertEqual(info["frame_bytes"], 169984)

    def test_real_late_video_uses_shared_audio_origin(self):
        source, pcm = self.source(audio_samples=38400, audio_delay="0",
                                  global_offset="3", video_delay="0.2")
        info = av.package_source(source, self.output, fps=20, ffmpeg=FFMPEG)
        self.assertEqual(info["frames"], 16)
        self.assertEqual(self.record(0)[1][:9600], pcm[:9600])
        self.assertEqual(self.record(15)[1][:9600], pcm[15*9600:16*9600])

    def test_real_failure_preserves_existing_and_unrelated_files(self):
        source = self.root / "bad source ; &.mp4"
        source.write_bytes(b"not a video")
        self.output.write_bytes(b"existing")
        unrelated = self.root / ".vid2-decode-unrelated"
        unrelated.mkdir()
        (unrelated / "keep").write_bytes(b"keep")
        before = set(self.root.iterdir())
        with self.assertRaisesRegex(ValueError, "failed"):
            av.package_source(source, self.output, ffmpeg=FFMPEG, force=True)
        self.assertEqual(set(self.root.iterdir()), before)
        self.assertEqual(self.output.read_bytes(), b"existing")
        self.assertEqual((unrelated / "keep").read_bytes(), b"keep")


if __name__ == "__main__":
    unittest.main(verbosity=2)
