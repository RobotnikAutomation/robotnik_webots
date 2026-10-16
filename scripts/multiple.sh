#!/usr/bin/env bash
# spawn_multiple.sh
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./spawn_multiple.sh [n] [base_name] [dx] [dy] [x0] [y0] [z] [yaw] [-- extra ros2 args]

Defaults:
  n=3 base_name=rbwatcher dx=2.0 dy=0.0 x0=0.0 y0=0.0 z=0.05 yaw=0.0
Examples:
  ./spawn_multiple.sh 5 rbwatcher 3.0 0 0 0 0.05 0
  ./spawn_multiple.sh 3 rbwatcher 1.0 1.0 0 0 0.05 0 -- run_rviz:=true

The robots are named <base_name>1, <base_name>2, ... .  All robot launchers
start without artificial delays.  The script then requires an image and both
controllers from every robot.  ROBOT_SPAWN_TIMEOUT controls the common startup
deadline in seconds (default: 90).

To pass extra ros2 launch arguments, provide all eight positional arguments
before `--`, as shown in the second example.
EOF
}

[[ "${1:-}" == "-h" || "${1:-}" == "--help" ]] && { usage; exit 0; }

n="${1:-3}"
base="${2:-rbwatcher}"
dx="${3:-2.0}"
dy="${4:-0.0}"
x0="${5:-0.0}"
y0="${6:-0.0}"
z="${7:-0.05}"
yaw="${8:-0.0}"
spawn_timeout="${ROBOT_SPAWN_TIMEOUT:-90}"

shift $(( $#>0 ? ($#>=8?8:$#) : 0 )) || true
if [[ "${1:-}" == "--" ]]; then shift; fi
extra_args=( "$@" )

if [[ ! "$n" =~ ^[1-9][0-9]*$ ]]; then
  echo "Error: n must be a positive integer." >&2
  exit 2
fi
if [[ ! "$base" =~ ^[a-zA-Z][a-zA-Z0-9_]*$ ]]; then
  echo "Error: base_name must be a valid ROS/Webots identifier." >&2
  exit 2
fi
if [[ ! "$spawn_timeout" =~ ^[1-9][0-9]*$ ]]; then
  echo "Error: ROBOT_SPAWN_TIMEOUT must be a positive integer." >&2
  exit 2
fi
python3 -c 'import sys; [float(value) for value in sys.argv[1:]]' \
  "$dx" "$dy" "$x0" "$y0" "$z" "$yaw" 2>/dev/null || {
  echo "Error: dx, dy, x0, y0, z and yaw must be numbers." >&2
  exit 2
}

# run_rviz applies to the complete scenario, not to every robot process.  The
# rbwatcher RViz configuration already contains displays for robots 1..3.
run_rviz=false
forward_args=()
for arg in "${extra_args[@]}"; do
  case "$arg" in
    run_rviz:=*) run_rviz="${arg#run_rviz:=}" ;;
    *) forward_args+=( "$arg" ) ;;
  esac
done
if [[ "$run_rviz" != "true" && "$run_rviz" != "false" ]]; then
  echo "Error: run_rviz must be true or false." >&2
  exit 2
fi

pkg="robotnik_webots"
file="spawn_robot.launch.py"

pids=()
preflight_dir=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$preflight_dir" && -d "$preflight_dir" ]]; then
    rm -r -- "$preflight_dir"
  fi
  if ((${#pids[@]})); then
    echo "Forwarding shutdown to ${#pids[@]} robot launches..."
    for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
    wait "${pids[@]}" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

readiness_topic() {
  local robot_name="$1"
  case "$base" in
    rbwatcher) echo "/$robot_name/$robot_name/front_rgbd_camera_color/image_color" ;;
    rbkairos|rbrobout) echo "/$robot_name/$robot_name/front_camera_color/image_color" ;;
    *)
      echo "Error: no readiness image topic is defined for model '$base'." >&2
      return 1
      ;;
  esac
}

all_launches_alive() {
  local launch_pid
  for launch_pid in "${pids[@]}"; do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "Error: robot launch PID $launch_pid exited during startup." >&2
      return 1
    fi
  done
}

wait_for_image() {
  local robot_name="$1"
  local ready_topic="$2"
  local deadline="$3"

  echo "Waiting for $robot_name image on $ready_topic"
  while (( SECONDS < deadline )); do
    all_launches_alive || return 1
    if timeout 3 ros2 topic echo --no-daemon --once --timeout 2 \
        --field header "$ready_topic" sensor_msgs/msg/Image >/dev/null 2>&1; then
      return 0
    fi
  done
  echo "Error: $robot_name did not publish its readiness image within ${spawn_timeout}s." >&2
  return 1
}

wait_for_active_controllers() {
  local robot_name="$1"
  local deadline="$2"
  local controllers

  echo "Waiting for $robot_name controllers to become active"
  while (( SECONDS < deadline )); do
    all_launches_alive || return 1
    controllers=$(timeout 3 ros2 control list_controllers \
      -c "/$robot_name/controller_manager" 2>/dev/null || true)
    if grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' <<<"$controllers" \
        && grep -Eq '^diffdrive_controller[[:space:]].*[[:space:]]active$' <<<"$controllers"; then
      echo "$robot_name controllers are active"
      return 0
    fi
    sleep 1
  done
  echo "Error: $robot_name controllers did not become active within ${spawn_timeout}s." >&2
  return 1
}

# A stale simulator must not be able to satisfy this run's image readiness.
# Run all DDS discovery queries concurrently so this safety check has constant
# latency instead of adding one discovery interval per robot.
preflight_dir=$(mktemp -d /tmp/robotnik_webots_preflight_XXXXXX)
preflight_pids=()
preflight_topics=()
for ((i=0;i<n;i++)); do
  name="${base}$((i+1))"
  ready_topic=$(readiness_topic "$name")
  preflight_topics+=( "$ready_topic" )
  ros2 topic info --no-daemon --spin-time 1 "$ready_topic" \
    >"$preflight_dir/$i" 2>/dev/null &
  preflight_pids+=( "$!" )
done
for preflight_pid in "${preflight_pids[@]}"; do
  wait "$preflight_pid" || true
done
for ((i=0;i<n;i++)); do
  ready_topic="${preflight_topics[$i]}"
  topic_info=$(<"$preflight_dir/$i")
  publisher_count=$(sed -n 's/^Publisher count: //p' <<<"$topic_info")
  if [[ -n "$publisher_count" ]] && (( publisher_count > 0 )); then
    echo "Error: $ready_topic already has $publisher_count publisher(s); check for a residual simulation." >&2
    rm -r -- "$preflight_dir"
    preflight_dir=""
    exit 1
  fi
done
rm -r -- "$preflight_dir"
preflight_dir=""

# Start every robot launcher immediately.  Each launcher still sequences its
# own Webots spawn response before its driver, but no robot waits for another
# robot's image or controllers.
names=()
ready_topics=()
startup_deadline=$((SECONDS + spawn_timeout))
for ((i=0;i<n;i++)); do
  name="${base}$((i+1))"
  xi=$(python3 -c 'import sys; print(float(sys.argv[1]) + int(sys.argv[3]) * float(sys.argv[2]))' "$x0" "$dx" "$i")
  yi=$(python3 -c 'import sys; print(float(sys.argv[1]) + int(sys.argv[3]) * float(sys.argv[2]))' "$y0" "$dy" "$i")
  robot_rviz=false
  if (( i == 0 )); then robot_rviz="$run_rviz"; fi
  echo "Launching $name at ($xi,$yi,$z) yaw=$yaw"
  ros2 launch "$pkg" "$file" \
    robot:="$base" robot_model:="$base" robot_id:="$name" \
    x:="$xi" y:="$yi" z:="$z" yaw:="$yaw" run_rviz:="$robot_rviz" \
    "${forward_args[@]}" &
  pids+=( "$!" )
  names+=( "$name" )
  ready_topics+=( "$(readiness_topic "$name")" )
done

# Validate the complete concurrently-started set against one common deadline.
for ((i=0;i<n;i++)); do
  name="${names[$i]}"
  ready_topic="${ready_topics[$i]}"
  wait_for_image "$name" "$ready_topic" "$startup_deadline" || exit 1
  wait_for_active_controllers "$name" "$startup_deadline" || exit 1
  echo "$name is active (image and controllers ready)"
done

# Every robot launch is expected to remain alive for the whole measurement.
# Report the first one that terminates so the benchmark supervisor can abort
# the iteration instead of measuring a partially active scenario.
set +e
wait -n "${pids[@]}"
status=$?
set -e
if (( status == 0 )); then
  status=1
fi
exit "$status"
