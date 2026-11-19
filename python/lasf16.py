# -*- coding: utf-8 -*-
"""Inspire Robots LASF16 on the USB board (LVTTL UART).

Bench, 2026-11-13: two actuators, id 1 and id 2, CH340 COM4, 921600 8N1.
Supply for motion is 8.0 V (series rating DC 8 V ±10%). At 5.3 V the
controller answers and the rod does not move. UART wires stay 3.3 V.

Close the 上位机 exe before opening this port.

    py -3.13 python/lasf16.py
    py -3.13 python/lasf16.py --move 1 1500

``--move`` enables the driver, sends one position (0–2000 counts, larger
is more extension), waits, then commands the count it started from.
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import serial


PORT = "COM4"
BAUD = 921600
# Series rating. Do not command motion below 7 V or above 8.8 V.
SUPPLY_V = 8.0


def _frame(body: list[int]) -> bytes:
    return bytes([0x55, 0xAA, *body, sum(body) & 0xFF])


def _s16(lo: int, hi: int) -> int:
    value = lo | (hi << 8)
    return value - 0x10000 if value & 0x8000 else value


@dataclass
class Status:
    actuator_id: int
    target: int
    position: int
    temp_c: int
    current_ma: int
    force_g: int
    error: int

    @property
    def stall(self) -> bool:
        return bool(self.error & 0x01)

    @property
    def overtemp(self) -> bool:
        return bool(self.error & 0x02)

    @property
    def overcurrent(self) -> bool:
        return bool(self.error & 0x04)

    @property
    def motor_fault(self) -> bool:
        return bool(self.error & 0x08)


def parse_status(data: bytes) -> Status | None:
    start = data.find(bytes([0xAA, 0x55]))
    if start < 0 or len(data) < start + 22:
        return None
    frame = data[start : start + 22]
    if (sum(frame[2:21]) & 0xFF) != frame[21]:
        return None
    if frame[2] != 0x11 or frame[4] != 0x04:
        return None
    temp = frame[11]
    if temp >= 128:
        temp -= 256
    return Status(
        actuator_id=frame[3],
        target=_s16(frame[7], frame[8]),
        position=_s16(frame[9], frame[10]),
        temp_c=temp,
        current_ma=frame[12] | (frame[13] << 8),
        force_g=_s16(frame[14], frame[16]),
        error=frame[15],
    )


class LasfBus:
    def __init__(self, port: str = PORT, baud: int = BAUD) -> None:
        self._ser = serial.Serial(port, baud, timeout=0.2, write_timeout=0.3)

    def close(self) -> None:
        self._ser.close()

    def __enter__(self) -> "LasfBus":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def _ask(self, packet: bytes, wait_s: float = 0.02, actuator_id: int | None = None) -> Status | None:
        """Read one status frame. A reply for the other rod is not this answer.

        The first read after a follow frame often arrives late. Returning the
        first 22 bytes, whichever rod they belong to, dropped motor 1 and the
        speed plot turned the gap into a spike.
        """
        self._ser.reset_input_buffer()
        self._ser.write(packet)
        self._ser.flush()
        data = bytearray()
        deadline = time.perf_counter() + wait_s
        while time.perf_counter() < deadline:
            waiting = self._ser.in_waiting
            if waiting:
                data.extend(self._ser.read(waiting))
            else:
                time.sleep(0.001)
            while True:
                start = data.find(bytes([0xAA, 0x55]))
                if start < 0 or len(data) < start + 22:
                    break
                parsed = parse_status(bytes(data[start : start + 22]))
                del data[: start + 22]
                if parsed is None:
                    continue
                if actuator_id is not None and parsed.actuator_id != actuator_id:
                    continue
                return parsed
        return None

    def status(self, actuator_id: int) -> Status:
        packet = _frame([0x03, actuator_id, 0x04, 0x00, 0x22])
        for wait_s in (0.02, 0.05):
            found = self._ask(packet, wait_s, actuator_id)
            if found is not None:
                return found
        raise TimeoutError(f"no status from id {actuator_id}")

    def status_quick(self, actuator_id: int, wait_s: float = 0.005) -> Status | None:
        """One short read. A miss must not stall the caller. The log uses 5 ms."""
        packet = _frame([0x03, actuator_id, 0x04, 0x00, 0x22])
        return self._ask(packet, wait_s, actuator_id)

    def read_status(self, ids: tuple[int, ...], wait_s: float = 0.02) -> dict[int, Status]:
        """Ask each rod, and keep a late frame instead of clearing it.

        Motor 1 is asked first. Its reply often arrives while motor 2 is being
        read. Clearing the buffer for that second request was the missed sample.
        """
        found: dict[int, Status] = {}
        data = bytearray()
        self._ser.reset_input_buffer()

        def take() -> None:
            while True:
                start = data.find(bytes([0xAA, 0x55]))
                if start < 0 or len(data) < start + 22:
                    if start > 0:
                        del data[:start]
                    break
                parsed = parse_status(bytes(data[start : start + 22]))
                del data[: start + 22]
                if parsed is not None:
                    found[parsed.actuator_id] = parsed

        def wait_for(actuator_id: int | None) -> None:
            deadline = time.perf_counter() + wait_s
            while time.perf_counter() < deadline:
                if actuator_id is not None and actuator_id in found:
                    return
                waiting = self._ser.in_waiting
                if waiting:
                    data.extend(self._ser.read(waiting))
                else:
                    time.sleep(0.001)
                take()

        for actuator_id in ids:
            if actuator_id in found:
                continue
            packet = _frame([0x03, actuator_id, 0x04, 0x00, 0x22])
            self._ser.write(packet)
            self._ser.flush()
            wait_for(actuator_id)
        return found

    def enable(self, actuator_id: int) -> Status | None:
        """Power-stage on. Required after 急停 before a position command."""
        return self._ask(_frame([0x03, actuator_id, 0x04, 0x00, 0x04]), 0.08)

    def estop(self, actuator_id: int) -> Status | None:
        """Power-stage off. A later move needs enable(), then a position."""
        return self._ask(_frame([0x03, actuator_id, 0x04, 0x00, 0x23]), 0.08)

    def clear_fault(self, actuator_id: int) -> Status | None:
        """Clear stall / overcurrent / motor-fault. Overtemperature waits for cooldown.

        At 5.3 V, bit3 stayed set until the target was written back to the
        present count. A clear alone did not drop it while the old target
        was still outstanding.
        """
        return self._ask(_frame([0x03, actuator_id, 0x04, 0x00, 0x1E]), 0.08)

    def move_counts(self, actuator_id: int, counts: int) -> Status | None:
        """Position mode with a status reply. Counts 0–2000."""
        counts = max(0, min(2000, int(counts)))
        lo = counts & 0xFF
        hi = (counts >> 8) & 0xFF
        return self._ask(_frame([0x04, actuator_id, 0x21, 0x37, lo, hi]), 0.08)

    def read_mem(self, actuator_id: int, index: int, length: int) -> bytes:
        packet = _frame([0x03, actuator_id, 0x01, index & 0xFF, length & 0xFF])
        self._ser.reset_input_buffer()
        self._ser.write(packet)
        self._ser.flush()
        time.sleep(0.03)
        data = self._ser.read(80)
        start = data.find(bytes([0xAA, 0x55]))
        if start < 0 or len(data) < start + 7:
            raise TimeoutError(f"no memory reply from id {actuator_id}")
        nbytes = data[start + 2]
        end = start + 5 + nbytes
        if len(data) < end:
            raise TimeoutError(f"short memory reply from id {actuator_id}")
        body = data[start + 2 : end - 1]
        if (sum(body) & 0xFF) != data[end - 1]:
            raise TimeoutError(f"bad memory checksum from id {actuator_id}")
        return bytes(data[start + 6 : start + 4 + nbytes])

    def write_u16(self, actuator_id: int, index: int, value: int) -> None:
        payload = [value & 0xFF, (value >> 8) & 0xFF]
        body = [4, actuator_id, 0x02, index & 0xFF, *payload]
        self._ask(bytes([0x55, 0xAA, *body, sum(body) & 0xFF]), 0.05)

    def write_u8(self, actuator_id: int, index: int, value: int) -> None:
        body = [3, actuator_id, 0x02, index & 0xFF, value & 0xFF]
        self._ask(bytes([0x55, 0xAA, *body, sum(body) & 0xFF]), 0.05)

    def broadcast(self, mode: int, targets: dict[int, int]) -> None:
        """One frame for every id. 0xF2 plans both moves. 0xF3 is follow mode.

        Follow mode expects a new frame every 10–50 ms. Neither mode returns
        a status frame, so this write does not wait.
        """
        payload: list[int] = []
        for actuator_id in sorted(targets):
            counts = max(0, min(2000, int(targets[actuator_id])))
            payload.extend(
                [actuator_id & 0xFF, counts & 0xFF, (counts >> 8) & 0xFF]
            )
        body = [1 + 3 * len(targets), 0xFF, mode & 0xFF, *payload]
        packet = bytes([0x55, 0xAA, *body, sum(body) & 0xFF])
        self._ser.write(packet)
        self._ser.flush()

    def write_speed(self, targets: dict[int, int], speed: int) -> None:
        """One position command per rod, with the speed-sheet voltage word.

        The one-page note encodes this as opcode 0x23, index 0, then position
        and a word from 300 to 1000. 1000 is described as full motor voltage.
        This is not the emergency-stop parameter of opcode 0x04.

        On the two bench rods (2026-11-15) the position was honored and the
        word was not: 700 counts took about 0.53 s at word 300 and with
        ordinary position mode 0x21. Do not treat the word as a speed command
        on these units. The write does not wait for a status frame.
        """
        speed = max(300, min(1000, int(speed)))
        slow = speed & 0xFF
        shigh = (speed >> 8) & 0xFF
        for actuator_id in sorted(targets):
            counts = max(0, min(2000, int(targets[actuator_id])))
            body = [
                0x06,
                actuator_id & 0xFF,
                0x23,
                0x00,
                counts & 0xFF,
                (counts >> 8) & 0xFF,
                slow,
                shigh,
            ]
            packet = bytes([0x55, 0xAA, *body, sum(body) & 0xFF])
            self._ser.write(packet)
        self._ser.flush()

    def _exchange(self, body: list[int], wait_s: float = 0.08) -> bytes:
        """Send one V2 register frame and return the raw reply, if any."""
        packet = bytes([0x55, 0xAA, *body, sum(body) & 0xFF])
        self._ser.reset_input_buffer()
        self._ser.write(packet)
        self._ser.flush()
        data = bytearray()
        deadline = time.perf_counter() + wait_s
        quiet = None
        while time.perf_counter() < deadline:
            waiting = self._ser.in_waiting
            if waiting:
                data.extend(self._ser.read(waiting))
                quiet = time.perf_counter()
            elif data and quiet is not None and time.perf_counter() - quiet > 0.004:
                break
            else:
                time.sleep(0.001)
        return bytes(data)

    def read_regs(self, actuator_id: int, address: int, count: int = 1) -> list[int] | None:
        """V2.0.3 register read, command 0x31. None if this firmware does not answer."""
        count = max(1, min(8, int(count)))
        raw = self._exchange(
            [0x04, actuator_id & 0xFF, 0x31, address & 0xFF, (address >> 8) & 0xFF, count]
        )
        start = raw.find(bytes([0xAA, 0x55]))
        if start < 0 or len(raw) < start + 8:
            return None
        if raw[start + 4] != 0x31:
            return None
        payload = raw[start + 7 : start + 7 + 2 * count]
        if len(payload) < 2 * count:
            return None
        return [payload[i] | (payload[i + 1] << 8) for i in range(0, 2 * count, 2)]

    def write_regs(self, actuator_id: int, address: int, values: list[int]) -> bytes:
        """V2.0.3 register write, command 0x32. Does not save to flash."""
        data: list[int] = []
        for value in values:
            word = int(value) & 0xFFFF
            data.extend([word & 0xFF, (word >> 8) & 0xFF])
        body = [
            3 + len(data),
            actuator_id & 0xFF,
            0x32,
            address & 0xFF,
            (address >> 8) & 0xFF,
            *data,
        ]
        packet = bytes([0x55, 0xAA, *body, sum(body) & 0xFF])
        if address == 0x28:
            print(
                f"USB {time.perf_counter():.3f} id={actuator_id} "
                f"speed={values[0] if values else ''} "
                f"{packet.hex(' ')}",
                flush=True,
            )
        return self._exchange(body)


def _print_status(st: Status) -> None:
    print(
        f"id={st.actuator_id} pos={st.position} tgt={st.target} "
        f"I={st.current_ma}mA T={st.temp_c}C F={st.force_g}g "
        f"err={st.error:02X}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="LASF16 status, or one unloaded move")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--move", nargs=2, type=int, metavar=("ID", "COUNTS"))
    args = parser.parse_args()
    with LasfBus(args.port) as bus:
        if args.move is None:
            for actuator_id in (1, 2):
                _print_status(bus.status(actuator_id))
            return
        actuator_id, counts = args.move
        home = bus.status(actuator_id)
        _print_status(home)
        bus.enable(actuator_id)
        bus.move_counts(actuator_id, counts)
        deadline = time.time() + 3.0
        last = home
        while time.time() < deadline:
            last = bus.status(actuator_id)
            _print_status(last)
            if last.motor_fault or last.stall or last.overcurrent:
                bus.move_counts(actuator_id, max(0, min(2000, home.position)))
                bus.estop(actuator_id)
                raise SystemExit(f"id {actuator_id} fault {last.error:02X}; stopped")
            if abs(last.position - counts) <= 15 and last.current_ma < 80:
                break
            time.sleep(0.08)
        bus.move_counts(actuator_id, max(0, min(2000, home.position)))
        time.sleep(0.4)
        _print_status(bus.status(actuator_id))


if __name__ == "__main__":
    main()
