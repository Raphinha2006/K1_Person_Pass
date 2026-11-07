#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
dry_run=true
case "${1:-}" in
  ""|--dry-run) ;;
  --real) dry_run=false ;;
  *) echo "Usage: $0 [--dry-run|--real]"; exit 2 ;;
esac
for node in brain_node vision_node game_controller_node person_pass_node; do
  if pgrep -f "(^|/)$node([[:space:]]|$)" >/dev/null; then
    echo "Already running: $node. Stop the other demo before starting this program."
    exit 1
  fi
done
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
cd "$ROOT"
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
mkdir -p logs
vision_pid=""
controller_pid=""
cleanup() {
  trap - EXIT
  for pid in "$controller_pid" "$vision_pid"; do
    if [ -n "$pid" ]; then kill -INT "$pid" 2>/dev/null || true; fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$ROOT/install/vision/lib/vision/vision_node" \
  "$ROOT/src/vision/config/vision.yaml" "$ROOT/config/no_vision_override.yaml" \
  "$ROOT/config/no_user_override.yaml" --ros-args -p save_data:=false -p save_depth:=false \
  >logs/vision.log 2>&1 &
vision_pid=$!
"$ROOT/install/person_pass/lib/person_pass/person_pass_node" --ros-args \
  --params-file "$ROOT/config/person_pass.yaml" -p "dry_run:=$dry_run" \
  >logs/person_pass.log 2>&1 &
controller_pid=$!
echo "Person pass started; dry_run=$dry_run; logs are in $ROOT/logs. Ctrl+C stops both processes."
wait -n "$vision_pid" "$controller_pid"
