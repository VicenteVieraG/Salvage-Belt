"""Check the first paint and keyboard controls in a real pseudo-terminal."""

import fcntl
import os
import pty
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
    with terminal(executable) as master:
        # The paused dashboard must paint before the first keypress.
        expect(master, b"Press n for one tick")
        for key, result in [(b"n", b"8128"), (b"a", b"k-d tree"), (b"\t", b"Body #0"),
                            (b"c", b"Hidden"), (b" ", b"Running"), (b"n", b"Paused")]:
            os.write(master, key)
            expect(master, result)


if __name__ == "__main__":
    main(sys.argv[1])
