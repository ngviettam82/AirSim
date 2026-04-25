#!/usr/bin/env python3
"""Repository text checks used by CI.

The repo intentionally preserves mixed LF/CRLF files globally, so this check
only rejects mixed endings inside an individual tracked file.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


CONFLICT_START = b"<<<<<<< "
CONFLICT_MIDDLE = b"======="
CONFLICT_END = b">>>>>>> "

LEGACY_MIXED_LINE_ENDING_FILES = {
    Path("LogViewer/LogViewer/Controls/AppSettings.xaml.cs"),
    Path("LogViewer/LogViewer/Controls/MeshViewer.xaml"),
    Path("LogViewer/LogViewer/Controls/MeshViewer.xaml.cs"),
    Path("LogViewer/LogViewer/Gestures/RotateGesture3D.cs"),
    Path("LogViewer/LogViewer/Utilities/Generated.cs"),
}


def repo_files() -> list[Path]:
    output = subprocess.check_output(
        ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"]
    )
    return [Path(name.decode("utf-8")) for name in output.split(b"\0") if name]


def is_binary(data: bytes) -> bool:
    return b"\0" in data


def line_ending_style(data: bytes) -> str:
    crlf = data.count(b"\r\n")
    lf = data.count(b"\n") - crlf
    cr = data.count(b"\r") - crlf

    if crlf and not lf and not cr:
        return "CRLF"
    if lf and not crlf and not cr:
        return "LF"
    if not crlf and not lf and not cr:
        return "none"
    return "mixed"


def has_conflict_marker(line: bytes) -> bool:
    stripped = line.rstrip(b"\r\n")
    return (
        stripped.startswith(CONFLICT_START)
        or stripped == CONFLICT_MIDDLE
        or stripped.startswith(CONFLICT_END)
    )


def main() -> int:
    failures: list[str] = []

    for path in repo_files():
        if not path.exists():
            continue

        data = path.read_bytes()
        if is_binary(data):
            continue

        style = line_ending_style(data)
        if style == "mixed" and path not in LEGACY_MIXED_LINE_ENDING_FILES:
            failures.append(f"{path}: mixed line endings")

        for line_number, line in enumerate(data.splitlines(keepends=True), 1):
            if has_conflict_marker(line):
                failures.append(f"{path}:{line_number}: unresolved merge conflict marker")

    if failures:
        print("Repository text checks failed:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1

    print("Repository text checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
