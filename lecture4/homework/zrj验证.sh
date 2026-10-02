#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
colcon build --packages-select nav_hw_interfaces qos_debugger \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

log_dir="zrj-test-logs"
mkdir -p "${log_dir}"

run_case() {
  local name="$1"
  local domain_id="$2"
  local delay_ms="$3"

  export ROS_DOMAIN_ID="${domain_id}"
  timeout 9s ros2 run qos_debugger qos_debugger_sub \
    --ros-args -p callback_delay_ms:="${delay_ms}" \
    > "${log_dir}/${name}_sub.log" 2>&1 &
  local sub_pid=$!

  sleep 1
  timeout 6s ros2 run qos_debugger qos_debugger_pub \
    > "${log_dir}/${name}_pub.log" 2>&1 &
  local pub_pid=$!

  wait "${pub_pid}" || test "$?" -eq 124
  wait "${sub_pid}" || test "$?" -eq 124
}

run_case normal 91 0
normal_warnings=$(grep -c "检测到丢包" "${log_dir}/normal_sub.log" || true)
echo "NORMAL_WARNINGS=${normal_warnings}"
test "${normal_warnings}" -eq 0

run_case stress 92 30
stress_warnings=$(grep -c "检测到丢包" "${log_dir}/stress_sub.log" || true)
echo "STRESS_WARNINGS=${stress_warnings}"
test "${stress_warnings}" -gt 0

echo "PASSED: 正常模式零丢包，压力模式成功复现并检测丢包"
