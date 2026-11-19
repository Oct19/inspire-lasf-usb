# -*- coding: utf-8 -*-
"""NDI Polaris Vega commands over the tracker Ethernet port, or a COM port.

Track shows this unit at 169.254.105.153. The Combined API command port is
TCP 8765. Commands are ASCII, a carriage return, and no CRC on the way out.
The reply ends with a carriage return and a 4-digit CRC-16/ARC.
"RESET" checks as BE6F. This bench does not send RESET.

A passive stray marker is TX 1000. The body is a 2-digit handle count,
a 2-digit stray count, one hex character of out-of-volume flags per four
markers, then 21 characters per marker. Each axis is a signed 7-character
integer in hundredths of a millimetre. Four status characters follow.
"""
from __future__ import annotations

import socket
import subprocess
import time

import serial
from serial.tools import list_ports

TCP_PORT = 8765
STATIONARY_HOST = "169.254.105.153"
ACTUATOR_PORT = "COM4"


def crc16_arc(data: bytes) -> int:
    """CRC-16/ARC. CRC of b'RESET' is 0xBE6F."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def split_reply(raw: str) -> tuple[str, str, bool]:
    text = raw.strip("\r\n")
    if len(text) >= 4 and all(c in "0123456789abcdefABCDEF" for c in text[-4:]):
        body, digits = text[:-4], text[-4:].upper()
        return body, digits, crc16_arc(body.encode("ascii", "replace")) == int(digits, 16)
    return text, "", False


def parse_tx_strays(body: str) -> list[tuple[float, float, float]]:
    """XYZ millimetres from a TX stray reply, CRC already removed.

    A loaded tool adds a handle line before the stray count. The stray
    section is the part after that line.
    """
    if body.startswith("ERROR"):
        raise RuntimeError(body)
    head, sep, tail = body.partition("\n")
    if sep:
        text = tail
    else:
        if len(head) < 2:
            raise RuntimeError(f"short TX reply {body!r}")
        handles = int(head[:2], 16)
        text = head[2 + 2 * handles :]
    if len(text) < 2:
        return []
    index = 0

    def take(count: int) -> str:
        nonlocal index
        chunk = text[index : index + count]
        if len(chunk) < count:
            raise RuntimeError(f"short TX reply {body!r}")
        index += count
        return chunk

    stray_count = int(take(2), 16)
    if stray_count > 50:
        raise RuntimeError(f"stray count {stray_count}")
    out_of_volume = (stray_count + 3) // 4
    if out_of_volume:
        take(out_of_volume)
    points: list[tuple[float, float, float]] = []
    for _ in range(stray_count):
        raw = take(21)
        points.append(
            (
                int(raw[0:7]) / 100.0,
                int(raw[7:14]) / 100.0,
                int(raw[14:21]) / 100.0,
            )
        )
    return points


def format_xyz(point: tuple[float, float, float]) -> str:
    return f"{point[0]:.2f},{point[1]:.2f},{point[2]:.2f}"


def _arp_hosts() -> list[str]:
    hosts: list[str] = []
    try:
        done = subprocess.run(
            ["arp", "-a"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return hosts
    for line in (done.stdout or "").splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        ip, mac = parts[0], parts[1].lower()
        if not ip.startswith("169.254.") or ip.endswith(".255"):
            continue
        if mac.startswith("ff-ff-ff"):
            continue
        hosts.append(ip)
    return hosts


def tracker_hosts() -> list[str]:
    """Link-local machines on this wire. The tracker picks a new 169.254 address; Track reads that address, it does not store one."""
    global _tracker_host_cache
    now = time.time()
    cached_at, cached = _tracker_host_cache
    if cached and now - cached_at < 1.0:
        return list(cached)
    _wake_link_local()
    own: set[str] = set()
    peers: list[str] = []
    try:
        done = subprocess.run(
            ["arp", "-a"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        done = None
    for line in ((done.stdout if done else "") or "").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "Interface:" and parts[1].count(".") == 3:
            own.add(parts[1])
            continue
        if len(parts) < 2:
            continue
        ip, mac = parts[0], parts[1].lower()
        if not ip.startswith("169.254.") or ip.endswith(".255"):
            continue
        if mac.startswith("ff-ff-ff") or mac.startswith("00-00-00"):
            continue
        if ip not in peers:
            peers.append(ip)
    hosts = [ip for ip in peers if ip not in own and _tcp_open(ip)]
    _tracker_host_cache = (now, hosts)
    return list(hosts)


def _wake_link_local() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.2)
        sock.sendto(b"NDI", ("169.254.255.255", TCP_PORT))
    except OSError:
        pass
    finally:
        sock.close()


def _tcp_open(host: str, timeout: float = 0.4) -> bool:
    sock = socket.socket()
    sock.settimeout(timeout)
    try:
        sock.connect((host, TCP_PORT))
        return True
    except OSError:
        return False
    finally:
        sock.close()


_tracker_host_cache: tuple[float, list[str]] = (0.0, [])


def list_endpoints() -> list[str]:
    """Stationary tracker address, other link-local hosts, then USB serial ports."""
    names: list[str] = []

    def add(name: str) -> None:
        if name and name not in names:
            names.append(name)

    add(STATIONARY_HOST)
    for ip in _arp_hosts():
        add(ip)
    try:
        ports = list_ports.comports()
    except Exception:
        ports = []
    for port in ports:
        description = (port.description or "").lower()
        if "bluetooth" in description:
            continue
        if port.device.upper() == ACTUATOR_PORT:
            continue
        add(port.device)
    return names


class NdiLink:
    """One open tracker, either TCP 8765 or a serial COM port at 9600 8N1."""

    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        self._ser: serial.Serial | None = None
        self._sock: socket.socket | None = None
        if endpoint.upper().startswith("COM"):
            self._ser = serial.Serial(
                endpoint,
                9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2,
                write_timeout=1.0,
            )
            return
        host, _, port_text = endpoint.partition(":")
        port = int(port_text) if port_text else TCP_PORT
        self._sock = socket.create_connection((host, port), timeout=0.8)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

    def command(self, text: str, wait_s: float = 1.2) -> str:
        payload = text.encode("ascii")
        if not payload.endswith(b"\r"):
            payload += b"\r"
        if self._sock is not None:
            self._sock.sendall(payload)
            return self._read_socket(wait_s)
        assert self._ser is not None
        self._ser.reset_input_buffer()
        self._ser.write(payload)
        self._ser.flush()
        return self._read_serial(wait_s)

    def _read_socket(self, wait_s: float) -> str:
        assert self._sock is not None
        buf = bytearray()
        deadline = time.time() + wait_s
        while time.time() < deadline:
            remain = max(0.005, deadline - time.time())
            self._sock.settimeout(remain)
            try:
                chunk = self._sock.recv(512)
            except TimeoutError:
                break
            if not chunk:
                break
            buf.extend(chunk)
            cut = buf.find(b"\r")
            if cut >= 0:
                return buf[:cut].decode("ascii", "replace")
        return buf.decode("ascii", "replace")

    def _read_serial(self, wait_s: float) -> str:
        assert self._ser is not None
        buf = bytearray()
        deadline = time.time() + wait_s
        while time.time() < deadline:
            chunk = self._ser.read(self._ser.in_waiting or 1)
            if not chunk:
                if buf:
                    break
                continue
            buf.extend(chunk)
            cut = buf.find(b"\r")
            if cut >= 0:
                return buf[:cut].decode("ascii", "replace")
        return buf.decode("ascii", "replace")


def read_identity(link: NdiLink) -> tuple[str, str]:
    raw = link.command("APIREV ", wait_s=2.0)
    body, _, ok = split_reply(raw)
    if not ok or not body or body.startswith("ERROR"):
        raise RuntimeError(raw or "no APIREV reply")
    firmware = ""
    try:
        version_raw = link.command("GET Features.Firmware.Version", wait_s=2.0)
        version_body, _, version_ok = split_reply(version_raw)
        if version_ok and "=" in version_body:
            firmware = version_body.split("=", 1)[1]
    except (OSError, TimeoutError):
        firmware = ""
    return body, firmware


def expect_okay(link: NdiLink, text: str, wait_s: float) -> None:
    raw = link.command(text, wait_s=wait_s)
    body, _, ok = split_reply(raw)
    if not ok or body != "OKAY":
        raise RuntimeError(raw or "empty reply")


def load_passive_rom(link: NdiLink, rom: bytes) -> str:
    """Load one wireless ROM and enable it so the camera reports stray markers.

    The tool pose is not used. A passive tool has to be enabled before the
    Vega will return stray-marker positions.
    """
    raw = link.command("PHRQ *********1****", wait_s=2.0)
    body, _, ok = split_reply(raw)
    if not ok or body.startswith("ERROR") or len(body) < 2:
        raise RuntimeError(raw or "PHRQ failed")
    handle = body[:2]
    padded = rom + b"\x00" * ((-len(rom)) % 64)
    for address in range(0, len(padded), 64):
        chunk = padded[address : address + 64]
        expect_okay(link, f"PVWR {handle}{address:04X}{chunk.hex().upper()}", 2.0)
    expect_okay(link, f"PINIT {handle}", 5.0)
    expect_okay(link, f"PENA {handle}D", 2.0)
    return handle


def _self_check() -> None:
    if crc16_arc(b"RESET") != 0xBE6F:
        raise SystemExit("CRC of RESET is not BE6F")
    body, digits, ok = split_reply("RESETBE6F")
    if body != "RESET" or digits != "BE6F" or not ok:
        raise SystemExit("reply parse failed")
    if parse_tx_strays("00000000") != []:
        raise SystemExit("empty stray frame")
    if parse_tx_strays("0101\n000000") != []:
        raise SystemExit("loaded tool with no stray")
    one = parse_tx_strays("0101\n010+000123-000456+0007890000")
    if one != [(1.23, -4.56, 7.89)]:
        raise SystemExit(f"one stray frame {one}")
    if format_xyz(one[0]) != "1.23,-4.56,7.89":
        raise SystemExit("xyz format")
    print("ndi crc ok")


if __name__ == "__main__":
    _self_check()
