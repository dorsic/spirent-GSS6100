#!/usr/bin/env python3
"""
GSS6100-based L1 hardware delay calibration helper.

What this program does
----------------------
This script talks to a Spirent GSS6100 over USB bulk endpoints and helps you
calibrate the effective L1 hardware delay of a GPS receiver.

The intended workflow is:
1. Connect the receiver RF input to the GSS6100 L1 output.
2. Connect the GSS6100 1PPS output and the receiver 1PPS output to a time
   interval counter / oscilloscope / TIC.
3. Keep the GSS6100 in a stable, static scenario.
4. Apply a series of small programmed timing offsets on the simulator.
5. After each step, measure the PPS time error:
      dt = receiver_1pps - simulator_1pps
   in nanoseconds.
6. Enter those measured values into this program.
7. The script fits a line and estimates the receiver's apparent L1 hardware
   delay from the intercept.

Sign convention used here
-------------------------
The script assumes the programmed simulator delay is added to the signal seen by
the receiver. You provide measured PPS error as:

    measured_ns = receiver_1pps - simulator_1pps

If your measurement instrument uses the opposite sign, flip the sign before
entering the values or use --invert-measurement-sign.

Notes
-----
- The exact SCPI command used to program timing delay differs between setups.
  This script contains a small abstraction layer. By default it does NOT send
  an unknown delay command unless you explicitly provide a command template.
- You can still use the script in "manual stimulus" mode: it will tell you what
  programmed offset to apply and record the result.
- A simple CSV log is written so you can re-fit later.

Examples
--------
Manual mode, just log offsets and measurements:
    python gss6100_l1_delay_calibrator.py manual \
        --offsets-ns -200,-100,-50,0,50,100,200

USB mode, if your firmware accepts a delay command template:
    python gss6100_l1_delay_calibrator.py usb \
        --delay-template "PDEL {ns}" \
        --offsets-ns -200,-100,-50,0,50,100,200

Quick connectivity check:
    python gss6100_l1_delay_calibrator.py identify
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

try:
    import usb.core
except ImportError:  # pragma: no cover
    usb = None  # type: ignore


class GSS6100:
    VID = 0x0525
    PID = 0xA105

    EP_OUT = 0x01
    EP_IN_RESP = 0x82
    EP_IN_EVT = 0x83

    def __init__(
        self,
        default_terminator: str = "\r",
        first_timeout_ms: int = 700,
        inter_packet_timeout_ms: int = 80,
        overall_timeout_ms: int = 2000,
    ) -> None:
        self.default_terminator = default_terminator
        self.first_timeout_ms = int(first_timeout_ms)
        self.inter_packet_timeout_ms = int(inter_packet_timeout_ms)
        self.overall_timeout_ms = int(overall_timeout_ms)
        self.dev = None
        self.connect()

    def connect(self, retries: int = 10, delay_s: float = 0.25) -> None:
        last_error: Optional[Exception] = None
        for _ in range(retries):
            try:
                dev = usb.core.find(idVendor=self.VID, idProduct=self.PID)
                if dev is None:
                    time.sleep(delay_s)
                    continue
                dev.set_configuration()
                self.dev = dev
                return
            except Exception as exc:  # pragma: no cover
                last_error = exc
                time.sleep(delay_s)
        if last_error is not None:
            raise RuntimeError(f"GSS6100 connect failed: {last_error}")
        raise RuntimeError("GSS6100 not found")

    def _ensure(self) -> None:
        if self.dev is None:
            self.connect()

    def _read_once(self, ep: int, size: int = 64, timeout_ms: int = 50) -> Optional[bytes]:
        self._ensure()
        assert self.dev is not None
        try:
            return bytes(self.dev.read(ep, size, timeout=timeout_ms))
        except usb.core.USBTimeoutError:
            return None
        except usb.core.USBError:
            self.connect()
            return None

    def _drain(self, ep: int, timeout_ms: int = 10, max_reads: int = 100) -> List[bytes]:
        out: List[bytes] = []
        for _ in range(max_reads):
            data = self._read_once(ep, timeout_ms=timeout_ms)
            if data is None:
                break
            out.append(data)
        return out

    def clear_response_queue(self) -> None:
        self._drain(self.EP_IN_RESP, timeout_ms=10, max_reads=50)

    def send(self, cmd: str, terminator: Optional[str] = None) -> None:
        self._ensure()
        assert self.dev is not None
        if terminator is None:
            terminator = self.default_terminator
        payload = cmd.encode("ascii") + terminator.encode("ascii")
        try:
            self.dev.write(self.EP_OUT, payload, timeout=1000)
        except usb.core.USBError:
            self.connect()
            assert self.dev is not None
            self.dev.write(self.EP_OUT, payload, timeout=1000)

    def query_raw(self, cmd: str, terminator: Optional[str] = None) -> bytes:
        if terminator is None:
            terminator = self.default_terminator

        self.clear_response_queue()
        self.send(cmd, terminator=terminator)

        response = bytearray()
        first = self._read_once(self.EP_IN_RESP, timeout_ms=self.first_timeout_ms)
        if first is None:
            raise TimeoutError(f"No response to {cmd!r}")
        response.extend(first)
        if b"\x00" in response:
            return bytes(response)

        deadline = time.monotonic() + self.overall_timeout_ms / 1000.0
        while time.monotonic() < deadline:
            data = self._read_once(self.EP_IN_RESP, timeout_ms=self.inter_packet_timeout_ms)
            if data is None:
                break
            response.extend(data)
            if b"\x00" in response:
                break

        return bytes(response)

    @staticmethod
    def decode(raw: bytes) -> str:
        raw = raw.split(b"\x00", 1)[0]
        return raw.decode("ascii", errors="replace").rstrip("\r\n")

    def query(self, cmd: str, terminators: Sequence[str] = ("\r", "\n", "\r\n"), retries: int = 2) -> str:
        last_error: Optional[Exception] = None
        for term in terminators:
            for _ in range(retries):
                try:
                    return self.decode(self.query_raw(cmd, term))
                except Exception as exc:
                    last_error = exc
                    time.sleep(0.04)
        if last_error is not None:
            raise last_error
        raise TimeoutError(f"No response to {cmd!r}")

    def identify(self) -> str:
        return self.query("*IDN?")

    def get_level(self) -> str:
        return self.query("LEVL ?")

    def get_status(self) -> str:
        return self.query("STAT ?")

    def get_tiop(self) -> str:
        return self.query("TIOP ?")


@dataclass
class Sample:
    programmed_offset_ns: float
    measured_pps_error_ns: float
    residual_ns: Optional[float] = None


def parse_offsets(text: str) -> List[float]:
    values: List[float] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        values.append(float(part))
    if not values:
        raise ValueError("No offsets supplied")
    return values


def fit_line(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float]:
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("Need at least two x/y pairs")
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx == 0:
        raise ValueError("Offsets must not all be equal")
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    return slope, intercept


def rms(values: Iterable[float]) -> float:
    vals = list(values)
    if not vals:
        return float("nan")
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def estimate_receiver_delay(samples: Sequence[Sample]) -> Tuple[float, float, float, List[Sample]]:
    xs = [s.programmed_offset_ns for s in samples]
    ys = [s.measured_pps_error_ns for s in samples]
    slope, intercept = fit_line(xs, ys)

    # If the ideal relation is measured = programmed + receiver_delay,
    # then the intercept is the receiver hardware delay.
    delay_ns = intercept

    enriched: List[Sample] = []
    residuals: List[float] = []
    for s in samples:
        model = slope * s.programmed_offset_ns + intercept
        res = s.measured_pps_error_ns - model
        residuals.append(res)
        enriched.append(Sample(s.programmed_offset_ns, s.measured_pps_error_ns, res))

    return delay_ns, slope, rms(residuals), enriched


def prompt_float(prompt: str) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            return float(raw)
        except ValueError:
            print("Please enter a number, for example -37.5")


def write_csv(path: Path, samples: Sequence[Sample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["programmed_offset_ns", "measured_pps_error_ns", "residual_ns"],
        )
        writer.writeheader()
        for s in samples:
            writer.writerow(asdict(s))


def print_report(delay_ns: float, slope: float, residual_rms_ns: float, samples: Sequence[Sample]) -> None:
    print()
    print("Calibration result")
    print("------------------")
    print(f"Estimated receiver L1 hardware delay : {delay_ns:+.3f} ns")
    print(f"Fit slope                           : {slope:.6f}")
    print(f"Residual RMS                        : {residual_rms_ns:.3f} ns")
    print()
    print("Samples")
    print("-------")
    for s in samples:
        res = "" if s.residual_ns is None else f"  residual={s.residual_ns:+.3f} ns"
        print(
            f"offset={s.programmed_offset_ns:+8.3f} ns  "
            f"measured={s.measured_pps_error_ns:+8.3f} ns{res}"
        )
    print()
    if abs(slope - 1.0) > 0.05:
        print("Warning: slope is far from 1.0.")
        print("This often means the sign convention is wrong, the applied delay command")
        print("does not map 1:1 to signal time, or the receiver had not settled.")
        print()


def run_identify(args: argparse.Namespace) -> int:
    g = GSS6100()
    print(g.identify())
    print(g.get_status())
    print(g.get_level())
    print(g.get_tiop())
    return 0


def maybe_send_delay(g: GSS6100, template: Optional[str], offset_ns: float, settle_s: float) -> None:
    if not template:
        return
    cmd = template.format(ns=offset_ns, us=offset_ns / 1000.0, ms=offset_ns / 1e6, s=offset_ns / 1e9)
    print(f"Applying simulator delay: {cmd}")
    g.send(cmd)
    time.sleep(settle_s)


def collect_samples(
    offsets_ns: Sequence[float],
    invert_measurement_sign: bool,
    csv_path: Path,
    usb_mode: bool,
    delay_template: Optional[str],
    settle_s: float,
) -> int:
    g = GSS6100() if usb_mode else None
    if g is not None:
        print("Connected:", g.identify())
        print("Status:   ", g.get_status())
        print("Level:    ", g.get_level())
        print("Timing:   ", g.get_tiop())
        print()

    samples: List[Sample] = []

    print("Measurement convention:")
    print("  measured_ns = receiver_1pps - simulator_1pps")
    if invert_measurement_sign:
        print("  NOTE: input values will be sign-inverted before fitting.")
    print()

    for offset in offsets_ns:
        print(f"Programmed offset target: {offset:+.3f} ns")
        if g is not None:
            maybe_send_delay(g, delay_template, offset, settle_s)
        else:
            print("Apply this offset on the simulator manually, then press Enter.")
            input()

        value = prompt_float("Enter measured PPS error [ns]: ")
        if invert_measurement_sign:
            value = -value
        samples.append(Sample(programmed_offset_ns=offset, measured_pps_error_ns=value))
        write_csv(csv_path, samples)
        print()

    delay_ns, slope, residual_rms_ns, enriched = estimate_receiver_delay(samples)
    write_csv(csv_path, enriched)
    print_report(delay_ns, slope, residual_rms_ns, enriched)
    print(f"Saved log to {csv_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Calibrate receiver L1 hardware delay with a Spirent GSS6100")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("identify", help="Query and print basic device info")

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--offsets-ns",
        required=True,
        help="Comma-separated programmed offsets in ns, e.g. -200,-100,-50,0,50,100,200",
    )
    common.add_argument(
        "--csv",
        default="l1_delay_calibration.csv",
        help="Output CSV log path",
    )
    common.add_argument(
        "--invert-measurement-sign",
        action="store_true",
        help="Invert entered measurements before fitting",
    )

    sub.add_parser("manual", parents=[common], help="Manual mode: you apply offsets yourself")

    usbp = sub.add_parser("usb", parents=[common], help="USB mode: optionally send a delay command template")
    usbp.add_argument(
        "--delay-template",
        default=None,
        help=(
            "SCPI command template for programmed timing delay. "
            "Available fields: {ns}, {us}, {ms}, {s}. "
            "Example: 'PDEL {ns}' or 'DELAY {us:.3f}'"
        ),
    )
    usbp.add_argument(
        "--settle-s",
        type=float,
        default=2.0,
        help="Settle time after each applied delay command",
    )

    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.cmd == "identify":
        return run_identify(args)

    offsets_ns = parse_offsets(args.offsets_ns)
    csv_path = Path(args.csv)

    if args.cmd == "manual":
        return collect_samples(
            offsets_ns=offsets_ns,
            invert_measurement_sign=args.invert_measurement_sign,
            csv_path=csv_path,
            usb_mode=False,
            delay_template=None,
            settle_s=0.0,
        )

    if args.cmd == "usb":
        return collect_samples(
            offsets_ns=offsets_ns,
            invert_measurement_sign=args.invert_measurement_sign,
            csv_path=csv_path,
            usb_mode=True,
            delay_template=args.delay_template,
            settle_s=float(args.settle_s),
        )

    parser.error("Unknown command")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

# Usage>

# python gss6100_l1_delay_calibrator.py manual --offsets-ns -200,-100,-50,0,50,100,200
# python gss6100_l1_delay_calibrator.py usb --delay-template "PDEL {ns}" --offsets-ns -200,-100,-50,0,50,100,200
