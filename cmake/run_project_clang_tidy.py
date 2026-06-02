#!/usr/bin/env python3
# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from xml.sax.saxutils import escape, quoteattr

import yaml

SOURCE_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx"}
DIAGNOSTIC_RE = re.compile(
    r"(?P<path>/[^:\n]+\.(?:c|cc|cpp|cxx|h|hh|hpp|hxx)):(?P<line>\d+):(?P<col>\d+): " r"(?P<level>warning|error): (?P<message>.*)"
)


def is_under(path: str, directory: str) -> bool:
    """Return whether path is inside directory after resolving symlinks."""
    try:
        Path(path).resolve().relative_to(Path(directory).resolve())
        return True
    except ValueError:
        return False


def load_project_compile_commands(build_dir: Path, source_dir: Path):
    """Load compile commands for source files owned by source_dir."""
    db_path = build_dir / "compile_commands.json"
    with db_path.open() as handle:
        commands = json.load(handle)

    filtered = []
    for item in commands:
        file_path = Path(item["file"]).resolve()
        if file_path.suffix in SOURCE_EXTENSIONS and is_under(str(file_path), str(source_dir)):
            filtered.append(item)
    return filtered


def clang_tidy_config(config_file: Path) -> str:
    """Return clang-tidy YAML config as an inline command-line value."""
    with config_file.open() as handle:
        data = yaml.safe_load(handle)
    return yaml.dump(data, default_flow_style=True, width=float("inf"))


def is_third_party_expansion(diagnostic) -> bool:
    """Return whether a diagnostic is caused by third-party macro/template expansion."""
    text = diagnostic["text"]
    has_third_party_frame = "/opt/ros/" in text or "/usr/include/" in text
    has_expansion_context = "expanded from macro" in text or "in instantiation of" in text
    return has_third_party_frame and has_expansion_context


def diagnostic_blocks(output: str, source_dir: Path):
    """Yield clang-tidy diagnostics whose primary location is under source_dir."""
    current = None
    ignored = 0

    def finish():
        nonlocal current, ignored
        if current is not None:
            result = current
            current = None
            if is_third_party_expansion(result):
                ignored += 1
                return None
            return result
        return None

    for line in output.splitlines():
        match = DIAGNOSTIC_RE.search(line)
        if match:
            block = finish()
            if block is not None:
                yield block

            diagnostic_path = Path(match.group("path")).resolve()
            if is_under(str(diagnostic_path), str(source_dir)):
                current = {
                    "path": str(diagnostic_path),
                    "line": int(match.group("line")),
                    "col": int(match.group("col")),
                    "message": match.group("message"),
                    "text": line,
                }
            else:
                ignored += 1
                current = None
        elif current is not None:
            current["text"] += "\n" + line

    block = finish()
    if block is not None:
        yield block
    if ignored:
        print(f"Ignored {ignored} clang-tidy diagnostic(s) outside {source_dir}")


def xunit_content(package_name: str, diagnostics, checked_files, elapsed: float) -> str:
    """Build an xUnit report for colcon test-result."""
    diagnostics = list(diagnostics)
    tests = len(diagnostics) if diagnostics else max(len(checked_files), 1)
    failures = len(diagnostics)
    testname = f"{package_name}.clang_tidy"
    xml = (
        f'<?xml version="1.0" encoding="UTF-8"?>\n'
        f"<testsuite\n"
        f'  name="{escape(testname)}"\n'
        f'  tests="{tests}"\n'
        f'  errors="0"\n'
        f'  failures="{failures}"\n'
        f'  time="{elapsed:.3f}"\n'
        f">\n"
    )

    if diagnostics:
        for diagnostic in diagnostics:
            location = f"{diagnostic['path']}:{diagnostic['line']}:{diagnostic['col']}"
            xml += (
                f"  <testcase\n"
                f"    name={quoteattr(location)}\n"
                f"    classname={quoteattr(testname)}\n"
                f"  >\n"
                f'      <failure message={quoteattr(diagnostic["message"])}>'
                f'<![CDATA[{diagnostic["text"]}]]></failure>\n'
                f"  </testcase>\n"
            )
    else:
        for file_path in checked_files or ["no project source files"]:
            xml += f"""  <testcase\n    name={quoteattr(str(file_path))}\n    classname={quoteattr(testname)}/>\n"""

    checked = "".join(f"\n* {file_path}" for file_path in checked_files)
    xml += f"  <system-out>Checked files:{escape(checked)}</system-out>\n"
    xml += "</testsuite>\n"
    return xml


def main():
    """Run project-scoped clang-tidy and write an xUnit result."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-name", required=True)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--config-file", type=Path, required=True)
    parser.add_argument("--xunit-file", type=Path, required=True)
    parser.add_argument("--extra-arg", action="append", default=[])
    args = parser.parse_args()

    start = time.time()
    clang_tidy = shutil.which("clang-tidy")
    if clang_tidy is None:
        print("Could not find clang-tidy executable", file=sys.stderr)
        return 1

    filtered_commands = load_project_compile_commands(args.build_dir, args.source_dir)
    if not filtered_commands:
        print(f"No project source compile commands found for {args.package_name}", file=sys.stderr)
        return 1

    filtered_db_dir = args.build_dir / "project_clang_tidy"
    filtered_db_dir.mkdir(parents=True, exist_ok=True)
    with (filtered_db_dir / "compile_commands.json").open("w") as handle:
        json.dump(filtered_commands, handle, indent=2)

    config = clang_tidy_config(args.config_file)
    header_filter = f"^{re.escape(str(args.source_dir.resolve()))}/(include|src)/.*"
    checked_files = [str(Path(item["file"]).resolve()) for item in filtered_commands]
    print(f"Checking {len(checked_files)} project source file(s) for package '{args.package_name}'")
    for file_path in checked_files:
        print(f"* {file_path}")

    combined_output = ""
    for file_path in checked_files:
        cmd = [
            clang_tidy,
            "-p",
            str(filtered_db_dir),
            f"--config={config}",
            "--header-filter",
            header_filter,
        ]
        for extra_arg in args.extra_arg:
            cmd.append(f"--extra-arg={extra_arg}")
        cmd.append(file_path)
        completed = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
        combined_output += completed.stdout

    if combined_output:
        print(combined_output)

    diagnostics = list(diagnostic_blocks(combined_output, args.source_dir))
    args.xunit_file.parent.mkdir(parents=True, exist_ok=True)
    args.xunit_file.write_text(xunit_content(args.package_name, diagnostics, checked_files, time.time() - start))
    return 1 if diagnostics else 0


if __name__ == "__main__":
    sys.exit(main())
