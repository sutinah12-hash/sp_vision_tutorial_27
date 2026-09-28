#!/usr/bin/env bash
set -euo pipefail

build_dir="build-zrj-verify"
cmake -S . -B "${build_dir}"
cmake --build "${build_dir}" --parallel 1

if ldd "${build_dir}/main" | grep -q "not found"; then
  echo "FAILED: 存在缺失的运行时动态库"
  exit 1
fi

echo "PASSED: Lecture2 全量编译及运行时动态库检查通过"

