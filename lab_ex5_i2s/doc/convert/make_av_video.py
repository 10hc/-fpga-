#!/usr/bin/env python3
"""Stream sector-aligned VID2 files using only stdlib (FFmpeg for source video).

VID2 stores a 512-byte header followed by fixed video/audio records. Source
video is vertically flipped for WRITE_V_FLIP=1; existing VID1 pixels are copied
unchanged. FFmpeg uses one input and one shared timestamp origin for both tracks.
Use the 12fps default for measured SPI DIV0 throughput (36 clocks/byte,
2.78MB/s). 15/20/24/25fps require higher throughput and are experimental.
See --help, or the task's standalone make_av_video.md, for examples.
"""

import argparse
import contextlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import wave


SECTOR = 512
WIDTH, HEIGHT = 320, 240
VIDEO_SECTORS = 300
VIDEO_BYTES = VIDEO_SECTORS * SECTOR
SAMPLE_RATE = 48000
FPS_VALUES = (12, 15, 20, 24, 25)
DEFAULT_FPS = 12
HEADER_STRUCT = struct.Struct("<4sHHHIHIHBBHH")


def audio_layout(fps):
    if fps not in FPS_VALUES:
        raise ValueError("fps must be 12, 15, 20, 24 or 25")
    spf = SAMPLE_RATE // fps
    return spf, (spf * 4 + SECTOR - 1) // SECTOR


def make_header(fps, frames):
    spf, audio_sectors = audio_layout(fps)
    if not 1 <= frames <= 0xFFFFFFFF:
        raise ValueError("frame count must be in 1..4294967295")
    return HEADER_STRUCT.pack(
        b"VID2", WIDTH, HEIGHT, fps, frames, audio_sectors,
        SAMPLE_RATE, spf, 2, 16, VIDEO_SECTORS, 1,
    ).ljust(SECTOR, b"\0")


def parse_header(header):
    """Validate every contract field, including reserved bytes."""
    if len(header) != SECTOR:
        raise ValueError("truncated 512-byte VID2 header")
    fields = HEADER_STRUCT.unpack_from(header)
    magic, width, height, fps, frames, sectors, rate, spf, channels, bits, vs, flags = fields
    if magic != b"VID2":
        raise ValueError("expected VID2 magic")
    expected = make_header(fps, frames)
    if header != expected:
        raise ValueError("invalid VID2 fields or nonzero reserved header bytes")
    return dict(magic="VID2", width=width, height=height, fps=fps, frames=frames,
                audio_sectors=sectors, sample_rate=rate, samples_per_frame=spf,
                channels=channels, bits=bits, video_sectors=vs, flags=flags,
                frame_bytes=(vs + sectors) * SECTOR)


def inspect_file(path):
    """Check header, exact length and each audio sector tail, with bounded reads."""
    with open(path, "rb") as source:
        info = parse_header(source.read(SECTOR))
        expected = SECTOR + info["frames"] * info["frame_bytes"]
        actual = os.fstat(source.fileno()).st_size
        if actual != expected:
            raise ValueError(f"VID2 size mismatch: expected {expected}, got {actual}")
        valid_audio = info["samples_per_frame"] * 4
        padding = info["audio_sectors"] * SECTOR - valid_audio
        if padding:
            for n in range(info["frames"]):
                source.seek(SECTOR + n * info["frame_bytes"] + VIDEO_BYTES + valid_audio)
                if source.read(padding) != bytes(padding):
                    raise ValueError(f"nonzero audio sector padding in frame {n}")
    info.update(file_bytes=actual, duration_seconds=info["frames"] / info["fps"])
    return info


def read_up_to(source, count):
    """Also handle pipes/readers that return fewer bytes than requested."""
    result = bytearray()
    while len(result) < count:
        chunk = source.read(count - len(result))
        if not chunk:
            break
        result.extend(chunk)
    return bytes(result)


def write_records(output, video, audio, fps, frames):
    """Read one frame at a time; truncate long PCM and zero-fill short PCM."""
    spf, sectors = audio_layout(fps)
    audio_bytes = spf * 4
    padding = bytes(sectors * SECTOR - audio_bytes)
    output.write(make_header(fps, frames))
    for n in range(frames):
        pixels = read_up_to(video, VIDEO_BYTES)
        if len(pixels) != VIDEO_BYTES:
            raise ValueError(f"truncated video frame {n}")
        pcm = read_up_to(audio, audio_bytes)
        if len(pcm) % 4:
            raise ValueError("PCM ends inside a stereo sample")
        output.write(pixels)
        output.write(pcm)
        output.write(bytes(audio_bytes - len(pcm)))
        output.write(padding)


def validate_output(path, inputs=(), force=False):
    path = Path(path)
    if path.suffix != ".VID":
        raise ValueError("output extension must be .VID (uppercase)")
    if not path.parent.is_dir():
        raise ValueError(f"output directory does not exist: {path.parent}")
    for source in inputs:
        same = path.resolve() == Path(source).resolve()
        if path.exists() and Path(source).exists():
            same = same or os.path.samefile(path, source)
        if same:
            raise ValueError("output must not replace an input, even with --force")
    if path.exists() and not force:
        raise FileExistsError(f"output exists (use --force): {path}")
    return path


@contextlib.contextmanager
def atomic_output(path, force):
    """Publish only a complete file; clean only the unique file created here."""
    fd, name = tempfile.mkstemp(prefix=".vid2-", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "wb") as output:
            yield output
            output.flush()
            os.fsync(output.fileno())
        inspect_file(temporary)
        if force:
            os.replace(temporary, path)
        else:
            # Hard-link publication is atomic and refuses an output created
            # by another process since preflight. Both paths are on one volume.
            os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def vid1_info(source):
    header = source.read(SECTOR)
    if len(header) != SECTOR or header[:4] != b"VID1":
        raise ValueError("expected VID1 with a 512-byte header")
    width, height, fps, frames = struct.unpack_from("<HHHI", header, 4)
    if (width, height) != (WIDTH, HEIGHT):
        raise ValueError("VID1 must be 320x240 RGB565LE")
    make_header(fps, frames)
    expected = SECTOR + frames * VIDEO_BYTES
    if os.fstat(source.fileno()).st_size != expected:
        raise ValueError("VID1 size does not match its declared frames")
    return fps, frames


def package_vid1(video_path, wav_path, output_path, fps=None, force=False):
    """Independent tracks: WAV sample 0 is assumed to match VID1 frame 0."""
    destination = validate_output(output_path, (video_path, wav_path), force)
    with open(video_path, "rb") as video, wave.open(str(wav_path), "rb") as audio:
        source_fps, frames = vid1_info(video)
        if fps is not None and fps != source_fps:
            raise ValueError("VID1 fps cannot be changed without resampling video")
        if (audio.getnchannels(), audio.getsampwidth(), audio.getframerate(),
                audio.getcomptype()) != (2, 2, SAMPLE_RATE, "NONE"):
            raise ValueError("WAV must be uncompressed PCM16LE stereo 48000Hz; mono is rejected")
        # wave.readframes respects RIFF chunks, unlike treating a WAV as raw PCM.
        class PCMReader:
            def read(self, count):
                return audio.readframes(count // 4)

        with atomic_output(destination, force) as output:
            write_records(output, video, PCMReader(), source_fps, frames)
    return inspect_file(destination)


def package_demo(seconds, output_path, fps=DEFAULT_FPS, force=False):
    """One colored 50ms flash + 1kHz stereo beep at each integer second."""
    audio_layout(fps)
    if not math.isfinite(seconds) or not 0 < seconds <= 0xFFFFFFFF / fps:
        raise ValueError("demo seconds must be finite, positive and fit the u32 frame count")
    frames = math.ceil(seconds * fps)
    make_header(fps, frames)
    destination = validate_output(output_path, force=force)
    spf, sectors = audio_layout(fps)
    palette = (0xF800, 0x07E0, 0x001F, 0xFFFF)
    with atomic_output(destination, force) as output:
        output.write(make_header(fps, frames))
        for n in range(frames):
            first_sample = n * spf
            flash = first_sample % SAMPLE_RATE < 2400
            color = palette[(n // fps) % len(palette)] if flash else 0
            # Logical top 8 rows are white; bottom-up storage matches the FPGA.
            row = struct.pack("<H", color) * WIDTH
            video = row * (HEIGHT - 8) + b"\xff\xff" * (WIDTH * 8)
            pcm = bytearray(sectors * SECTOR)
            for k in range(spf):
                position = (first_sample + k) % SAMPLE_RATE
                value = round(12000 * math.sin(2 * math.pi * (position % 48) / 48)) if position < 2400 else 0
                struct.pack_into("<hh", pcm, 4 * k, value, value)
            output.write(video)
            output.write(pcm)
    return inspect_file(destination)


def resolve_program(value, default):
    candidate = str(value) if value else default
    found = shutil.which(candidate)
    if found:
        return found
    raise ValueError(f"{default} not found: supply --{default} with an installed executable path")


def run_logged(command, directory):
    """No shell and no unbounded stderr capture; retain only the error tail."""
    with tempfile.TemporaryFile(dir=directory) as log:
        completed = subprocess.run(command, stdin=subprocess.DEVNULL,
                                   stdout=subprocess.DEVNULL, stderr=log, check=False)
        if completed.returncode:
            log.seek(0, os.SEEK_END)
            log.seek(max(0, log.tell() - 8192))
            detail = log.read().decode("utf-8", errors="replace").strip()
            raise ValueError(f"{Path(command[0]).name} failed ({completed.returncode}): {detail}")


def package_source(source_path, output_path, fps=DEFAULT_FPS, ffmpeg=None, ffprobe=None, force=False):
    """Decode on a common timeline, spool to disk, then interleave frame by frame.

    Memory is bounded by a frame. Scratch disk needs the decoded RGB565 and PCM
    plus the output file. ffprobe is optional; if supplied, it probes the source
    before decoding. FFmpeg requires both video and audio streams.
    """
    audio_layout(fps)
    source = Path(source_path).resolve(strict=True)
    destination = validate_output(output_path, (source,), force)
    encoder = resolve_program(ffmpeg, "ffmpeg")
    with tempfile.TemporaryDirectory(prefix=".vid2-decode-", dir=destination.parent) as scratch:
        if ffprobe:
            probe = resolve_program(ffprobe, "ffprobe")
            run_logged([probe, "-v", "error", "-show_entries", "stream=index,codec_type",
                        "-of", "json", str(source)], scratch)
        video_path = Path(scratch) / "video.rgb565"
        audio_path = Path(scratch) / "audio.pcm"
        # -start_at_zero shifts both streams by the SAME input start_time.
        # Never use per-stream setpts/asetpts=PTS-STARTPTS: it loses track delay.
        # fps duplicates a delayed first video frame; aresample inserts initial
        # silence for delayed audio and trims negative timestamps at shared zero.
        command = [encoder, "-nostdin", "-hide_banner", "-loglevel", "error", "-y",
                   "-copyts", "-start_at_zero", "-i", str(source),
                   "-map", "0:v:0", "-an", "-vf",
                   f"fps=fps={fps}:start_time=0,scale=320:240:flags=bilinear,vflip",
                   "-c:v", "rawvideo", "-threads:v", "1", "-pix_fmt", "rgb565le",
                   "-fps_mode", "passthrough", "-f", "rawvideo", str(video_path),
                   "-map", "0:a:0", "-vn", "-af", "aresample=48000:async=1:first_pts=0",
                   "-ac", "2", "-ar", "48000", "-c:a", "pcm_s16le",
                   "-f", "s16le", str(audio_path)]
        run_logged(command, scratch)
        size = video_path.stat().st_size
        frames, remainder = divmod(size, VIDEO_BYTES)
        if remainder or not frames:
            raise ValueError("FFmpeg produced no complete video frames")
        with open(video_path, "rb") as video, open(audio_path, "rb") as audio:
            with atomic_output(destination, force) as output:
                write_records(output, video, audio, fps, frames)
    return inspect_file(destination)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--input", type=Path, help="source video with audio (mono is converted to stereo)")
    modes.add_argument("--vid1", type=Path, help="existing sector-header VID1; requires --wav")
    modes.add_argument("--demo", type=float, metavar="SECONDS", help="stdlib synchronized flash/beep demo")
    modes.add_argument("--inspect", type=Path, help="stdlib strict header/length/padding check")
    parser.add_argument("--wav", type=Path, help="PCM16LE stereo 48kHz; sample 0 assumed to match VID1 frame 0")
    parser.add_argument("--out", "--output", type=Path, default=Path("VIDEO_AV.VID"))
    parser.add_argument("--fps", type=int, choices=FPS_VALUES,
                        help="source/demo default: 12 (measured SPI 25MHz DIV0: "
                             "36 clocks/byte, 2.78MB/s); 15/20/24/25: experiments "
                             "requiring higher throughput; VID1: inherit (no rate conversion)")
    parser.add_argument("--ffmpeg", help="installed ffmpeg executable, or PATH lookup")
    parser.add_argument("--ffprobe", help="optional installed ffprobe for source validation")
    parser.add_argument("--force", action="store_true", help="replace output after successful conversion")
    args = parser.parse_args(argv)
    if args.vid1 and not args.wav:
        parser.error("--vid1 requires --wav")
    if args.wav and not args.vid1:
        parser.error("--wav is only valid with --vid1")
    if (args.ffmpeg or args.ffprobe) and not args.input:
        parser.error("--ffmpeg/--ffprobe are only valid with --input")
    try:
        if args.inspect:
            result = inspect_file(args.inspect)
        elif args.demo is not None:
            result = package_demo(args.demo, args.out, args.fps or DEFAULT_FPS, args.force)
        elif args.vid1:
            result = package_vid1(args.vid1, args.wav, args.out, args.fps, args.force)
        else:
            result = package_source(args.input, args.out, args.fps or DEFAULT_FPS,
                                    args.ffmpeg, args.ffprobe, args.force)
    except (OSError, ValueError, wave.Error, EOFError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
