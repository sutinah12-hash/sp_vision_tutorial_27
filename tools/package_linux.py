#!/usr/bin/env python3
"""Package a clean Git revision plus locally verified Linux binaries; never start hardware."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import platform
import subprocess
import tarfile
import time


def run(*args, cwd):
    return subprocess.check_output(args, cwd=cwd, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path("build-deploy"))
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = (root / args.build_dir).resolve()
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        parser.error("Build/package on the Ubuntu x86_64 VM, not Windows")
    if run("git", "status", "--porcelain", "--untracked-files=all", cwd=root):
        parser.error("Commit or preserve changes first; packaging requires a clean Git revision")
    if args.output.exists():
        parser.error("Output already exists; choose a new name rather than overwrite it")
    cache = (build / "CMakeCache.txt").read_text()
    if f"CMAKE_HOME_DIRECTORY:INTERNAL={root}" not in cache.splitlines():
        parser.error("Build directory belongs to a different source checkout")
    revision = run("git", "rev-parse", "HEAD", cwd=root)
    epoch = int(run("git", "show", "-s", "--format=%ct", "HEAD", cwd=root))
    source_tar = subprocess.check_output(["git", "archive", "--format=tar", "HEAD"], cwd=root)
    entries = []
    with tarfile.open(fileobj=io.BytesIO(source_tar), mode="r:") as archive:
        for member in archive.getmembers():
            if member.isfile():
                entries.append((member.name, archive.extractfile(member).read(), member.mode))
            elif not member.isdir():
                parser.error(f"Unsupported nonregular source entry: {member.name}")
    binary_info = {}
    for name in ("task_1", "task_2", "task_3", "aim_simulator"):
        binary = build / name
        data = binary.read_bytes()
        if not data.startswith(b"\x7fELF"):
            parser.error(f"Not an ELF binary: {binary}")
        deps = run("ldd", str(binary), cwd=root)
        if "not found" in deps:
            parser.error(f"Unresolved dependencies for {name}: {deps}")
        dynamic = run("readelf", "-d", str(binary), cwd=root)
        if str(root) in dynamic:
            parser.error(f"{name} contains a build-directory RPATH; cannot safely relocate")
        if name.startswith("task_"):
            run(str(binary), "--help", cwd=root)
        entries.append((f"bin/{name}", data, 0o755))
        binary_info[name] = {"sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data),
                             "runpath": [line.strip() for line in dynamic.splitlines() if "RUNPATH" in line]}
    info = {
        "source_commit": revision, "source_branch": run("git", "branch", "--show-current", cwd=root),
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "os_release": Path("/etc/os-release").read_text(), "architecture": platform.machine(),
        "compiler": run("g++", "--version", cwd=root).splitlines()[0],
        "cmake": run("cmake", "--version", cwd=root).splitlines()[0],
        "opencv": run("pkg-config", "--modversion", "opencv4", cwd=root),
        "binaries": binary_info,
        "notice": "Requires compatible host libraries and full MVS/OpenVINO runtime; not a static standalone package. VM offline tests are not real-robot validation.",
    }
    entries.append(("PACKAGE_INFO.json", (json.dumps(info, indent=2, ensure_ascii=False) + "\n").encode(), 0o644))
    manifest = "".join(f"{hashlib.sha256(data).hexdigest()}  {name}\n" for name, data, _ in sorted(entries))
    entries.append(("MANIFEST.sha256", manifest.encode(), 0o644))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation: an existing student delivery can never be overwritten.
    with args.output.open("xb") as output:
        with tarfile.open(fileobj=output, mode="w:gz") as archive:
            for name, data, mode in sorted(entries):
                member = tarfile.TarInfo(f"zhaoruijie-final_project_aim/{name}")
                member.size, member.mode, member.mtime = len(data), mode, epoch
                archive.addfile(member, io.BytesIO(data))
    print(json.dumps({"package": str(args.output.resolve()), "commit": revision,
                      "bytes": args.output.stat().st_size,
                      "sha256": hashlib.sha256(args.output.read_bytes()).hexdigest()}, indent=2))


if __name__ == "__main__":
    main()
