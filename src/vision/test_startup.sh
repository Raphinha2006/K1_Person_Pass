#!/usr/bin/env bash
# Load the actual GPU models without a controller or live camera input.
set -eo pipefail
binary="$1"
cd "$2"
mkdir -p logs
output="$PWD/logs/vision_startup_test.log"
ROS_DOMAIN_ID=83 "$binary" "$PWD/src/vision/config/vision.yaml" \
  "$PWD/config/no_vision_override.yaml" "$PWD/config/no_user_override.yaml" \
  --ros-args -p save_data:=false -p save_depth:=false >"$output" 2>&1 &
pid=$!
cleanup() {
  kill -INT "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
ready=false
for attempt in {1..150}; do
  if ! kill -0 "$pid" 2>/dev/null; then cat "$output"; exit 1; fi
  if grep -q 'create sub for segmentor' "$output"; then ready=true; break; fi
  sleep .1
done
if [ "$ready" != true ]; then cat "$output"; echo 'Vision initialization timed out'; exit 1; fi
sleep 2
if ! kill -0 "$pid" 2>/dev/null; then cat "$output"; exit 1; fi
kill -INT "$pid"
if ! wait "$pid"; then cat "$output"; echo 'Vision did not exit cleanly'; exit 1; fi
trap - EXIT
echo 'PASS: actual Vision initializes and exits with save_data=false; controller not started'
