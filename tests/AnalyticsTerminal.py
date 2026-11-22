"""Check the first paint, shortcuts and mouse controls in a real pseudo-terminal."""

import fcntl
import os
import pty
import re
import select
import struct
import subprocess
import sys
import termios
import time
from contextlib import contextmanager


def expect(terminal, text):
    output = bytearray()
    deadline = time.monotonic() + 3
    while text not in output:
        timeout = max(0, deadline - time.monotonic())
        if not select.select([terminal], [], [], timeout)[0]:
            raise AssertionError(f"Missing {text!r} in terminal output {output[-2000:]!r}")
        output.extend(os.read(terminal, 8192))
    return bytes(output)


def full_frame(terminal, columns=80, rows=24):
    # FTXUI sends a full frame on startup and resize, then incremental updates.
    pending = b""
    end = b"\x1b[?2026l"
    while True:
        pending += expect(terminal, end)
        *frames, pending = pending.split(end)
        for frame in frames:
            _, start, body = frame.rpartition(b"\x1b[?2026h")
            if not start:
                continue
            plain = re.sub(rb"\x1b\[[0-?]*[ -/]*[@-~]", b"", body)
            lines = plain.replace(b"\r", b"").decode().split("\n")
            if len(lines) == rows and len(lines[0]) == columns:
                return lines


def resize_frame(terminal, columns, rows=24):
    fcntl.ioctl(terminal, termios.TIOCSWINSZ, struct.pack("HHHH", rows, columns, 0, 0))
    os.write(terminal, b"\x1b[<35;1;1M")  # Mouse motion wakes the event loop.
    return full_frame(terminal, columns, rows)


def mouse_at(terminal, frame, label, button=0):
    """Send an SGR mouse click or wheel event at the rendered label."""
    y, line = next((y, line) for y, line in enumerate(frame, 1) if label in line)
    x = line.index(label) + 1
    event = f"\x1b[<{button};{x};{y}".encode()
    os.write(terminal, event + b"M")
    if button < 3:
        os.write(terminal, event + b"m")


@contextmanager
def terminal(*command):
    master, slave = pty.openpty()
    try:
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 24, 80, 0, 0))
        with subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave,
                              env={**os.environ, "TERM": "xterm-256color"}) as app:
            try:
                yield master
                os.write(master, b"q")
                assert app.wait(timeout=3) == 0
            finally:
                if app.poll() is None:
                    app.kill()
    finally:
        os.close(slave)
        os.close(master)


def main(executable):
    redirected = subprocess.run([executable], stdin=subprocess.DEVNULL, capture_output=True, timeout=3)
    assert redirected.returncode != 0 and b"interactive terminal" in redirected.stderr
    invalid = subprocess.run([executable, "--bodies=-1"], capture_output=True, timeout=3)
    assert invalid.returncode != 0 and b"whole number" in invalid.stderr
    with terminal(executable) as master:
        # The paused dashboard must paint before the first keypress.
        expect(master, b"Press n for one tick")
        for key, result in [(b"n", b"8128"), (b"a", b"k-d tree"), (b"\t", b"Body #0"),
                            (b"c", b"Hidden"), (b" ", b"Running"), (b"n", b"Paused")]:
            os.write(master, key)
            expect(master, result)

    with terminal(executable) as master:
        frame = full_frame(master)
        mouse_at(master, frame, "n Step", button=2)  # Right-click does not step.
        mouse_at(master, frame, "n Step")
        expect(master, b"8128")
        mouse_at(master, frame, "Inspector")
        expect(master, b"Position")
        frame = resize_frame(master, 81)
        assert "Tick 1 " in frame[0]  # Left release did not step again.
        mouse_at(master, frame, "#5    Sphere")
        frame = resize_frame(master, 80)
        assert any("Body #5" in line for line in frame)
        mouse_at(master, frame, "Position", button=65)
        frame = resize_frame(master, 81)
        assert any("Body #6" in line for line in frame)
        mouse_at(master, frame, "c Contacts")
        expect(master, b"Hidden")
        frame = resize_frame(master, 44, 20)
        mouse_at(master, frame, "] Next")
        frame = resize_frame(master, 45, 20)
        assert any("Body #7" in line for line in frame)
        mouse_at(master, frame, "o Open")
        expect(master, b"Open saved benchmark")
        frame = resize_frame(master, 44, 20)
        mouse_at(master, frame, "Esc Cancel")
        expect(master, b"Body #7")

    with terminal(executable, "--bodies", "5") as master:
        frame = full_frame(master)
        assert "5 bodies" in frame[1]

        def after(keys=b""):
            nonlocal frame
            if keys:
                os.write(master, keys)
            frame = resize_frame(master, 81 if len(frame[0]) == 80 else 80)
            return "\n".join(frame)

        assert re.search(r"Examined +10│", after(b"n"))
        mouse_at(master, frame, "3 Sweep and prune")
        screen = after()
        assert "Tick 1 " in screen and re.search(r"Examined +—│", screen)
        screen = after(b"n")
        assert "Tick 2 " in screen and int(re.search(r"Examined +(\d+)│", screen)[1]) < 10
        assert "Tick 2 " in after(b"1")
        assert re.search(r"Examined +10│", after(b"n"))
        assert re.search(r"Examined +10│", after(b"1"))  # Selecting the active algorithm keeps its measurements.

        mouse_at(master, frame, "5 bodies")
        assert "Set body count" in after()
        assert "Tick 3 " in after(b"5\r")  # The same count keeps the field.
        assert "whole number" in after(b"b-1\r")
        mouse_at(master, frame, "Esc Cancel")
        assert "Tick 3 " in after()
        assert "whole number" in after(b"b4294967296\r")
        mouse_at(master, frame, "Esc Cancel")
        assert "Tick 3 " in after()

        assert "Body #4" in after(b"\t[")
        assert "Set body count" in after(b"b2")
        mouse_at(master, frame, "Enter Apply")
        screen = after()
        assert "Tick 0 " in screen and "2 bodies" in screen and "Body #1 · Box" in screen
        assert "No bodies in this scene" in after(b"b0\r")
        screen = after(b"b3\r")
        assert "3 bodies" in screen and "Body #0 · Sphere" in screen
        assert "Body #1 · Sphere" in after(b"]")
        assert "Body #2 · Box" in after(b"]")
        screen = after(b"b8192\r")
        assert "8192 bodies" in screen and "Tick 0 " in screen

    with terminal(executable, "--bodies", "5") as master:
        frame = full_frame(master)
        mouse_at(master, frame, "g Generate report")
        expect(master, b"3 Individuals report")
        os.write(master, b"3")
        expect(master, b"Waiting for the first tick")
        os.write(master, b"\x1b")
        expect(master, b"Salvage Analytics")
        os.write(master, b"n")
        expect(master, b"1 tick   Mean")
        os.write(master, b"g1")
        expect(master, b"Generated benchmark")
        frame = "\n".join(resize_frame(master, 81))
        for algorithm in ("brute-force", "kd-tree", "sweep-and-prune"):
            assert re.search(rf"{algorithm}\s+1\s+\d+\.\d+\s+\d+\.\d+", frame), frame
        os.write(master, b"\r")
        expect(master, b"Repetition 1/3   Tick 11")
        os.write(master, b"\x1b")
        expect(master, b"Algorithm")
        os.write(master, b"\x1b")
        assert "Tick 1 " in resize_frame(master, 80)[0]

        os.write(master, b"2g2")
        expect(master, b"Generated benchmark")
        frame = "\n".join(resize_frame(master, 44, 20))
        assert "kd-tree" in frame and "Variant 1/1" in frame and "brute-force" not in frame
        os.write(master, b"\x1b")
        assert "Tick 1 " in resize_frame(master, 45, 20)[0]
        os.write(master, b"g3[")
        frame = "\n".join(resize_frame(master, 44, 20))
        assert "Individuals report" in frame and "Tick 1" in frame and "Body #4 · Box" in frame
        assert "No contacts for this body" in frame and "Position" in frame

    with terminal(executable, "--bodies", "8192") as master:
        expect(master, b"Press n for one tick")
        os.write(master, b"g1")
        expect(master, b"Running benchmark")
        os.write(master, b"\x1b")
        expect(master, b"Salvage Analytics")


if __name__ == "__main__":
    main(sys.argv[1])
