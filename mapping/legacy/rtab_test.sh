#!/usr/bin/env bash
set -euo pipefail

# ===== EDIT THESE IF YOUR TOPICS DIFFER =====
RGB_TOPIC="/oak/rgb/image_raw"
DEPTH_TOPIC="/oak/stereo/image_raw"
CAMINFO_TOPIC="/oak/rgb/camera_info"

# ===== FRAMES =====
BASE_FRAME="base_link"
OAK_ROOT="oak"          # try "oak-d-base-frame" if your TF tree is rooted there
ODOM_FRAME="odom"
MAP_FRAME="map"

# ===== MOUNT TF base_link -> oak root (rough test values) =====
MOUNT_X="0.10"
MOUNT_Y="0.00"
MOUNT_Z="0.20"

# ===== TRANSPORTS =====
RGB_TRANSPORT="compressed"
DEPTH_TRANSPORT="compressedDepth"

# ===== SYNC STABILITY =====
TOPIC_Q="20"
SYNC_Q="20"
WAIT_TF="0.5"

# ===== helpers =====
PIDS=()

cleanup() {
  echo ""
  echo "[rtab_test] Stopping..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

run_node() {
  local name="$1"; shift
  echo "[rtab_test] Starting $name ..."
  "$@" >"/tmp/rtab_${name}.log" 2>&1 &
  PIDS+=($!)
  echo "[rtab_test]   pid=${PIDS[-1]} log=/tmp/rtab_${name}.log"
}

assert_running() {
  # Usage: assert_running "NAME" pid
  local name="$1"; local pid="$2"
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[rtab_test] ERROR: $name died immediately."
    exit 1
  fi
}

echo "[rtab_test] ===== Mondy RTAB-Map live test ====="
echo "[rtab_test] RGB:      $RGB_TOPIC ($RGB_TRANSPORT)"
echo "[rtab_test] Depth:    $DEPTH_TOPIC ($DEPTH_TRANSPORT)"
echo "[rtab_test] CamInfo:  $CAMINFO_TOPIC"
echo "[rtab_test] Frames:   $MAP_FRAME -> $ODOM_FRAME -> $BASE_FRAME -> $OAK_ROOT -> (oak cameras)"
echo ""

# 1) Static TF (quaternion form: x y z qx qy qz qw parent child)
# Identity rotation: 0 0 0 1
run_node "TF_STATIC" ros2 run tf2_ros static_transform_publisher \
  "$MOUNT_X" "$MOUNT_Y" "$MOUNT_Z" 0 0 0 1 \
  "$BASE_FRAME" "$OAK_ROOT"

sleep 1

# Quick check: did base_link appear?
echo "[rtab_test] Checking TF frames exist (2s)..."
timeout 2 ros2 run tf2_ros tf2_echo "$BASE_FRAME" "$OAK_ROOT" || true

# 2) rgbd_sync (decompress + sync -> /rgbd_image)
run_node "RGBD_SYNC" ros2 run rtabmap_sync rgbd_sync --ros-args \
  -r /rgb/image:="$RGB_TOPIC" \
  -r /depth/image:="$DEPTH_TOPIC" \
  -r /rgb/camera_info:="$CAMINFO_TOPIC" \
  -p rgb_transport:="$RGB_TRANSPORT" \
  -p depth_transport:="$DEPTH_TRANSPORT" \
  -p approx_sync:=true \
  -p topic_queue_size:="$TOPIC_Q" \
  -p sync_queue_size:="$SYNC_Q"

sleep 2
echo "[rtab_test] /rgbd_image rate (5s window):"
ros2 topic hz /rgbd_image -w 20 || true
echo ""

# 3) rgbd_odometry (publishes /odom + TF odom->base_link)
run_node "RGBD_ODOM" ros2 run rtabmap_odom rgbd_odometry --ros-args \
  -p subscribe_rgbd:=true \
  -r /rgbd_image:=/rgbd_image \
  -p frame_id:="$BASE_FRAME" \
  -p odom_frame_id:="$ODOM_FRAME" \
  -p publish_tf:=true \
  -p wait_for_transform:="$WAIT_TF"

sleep 2
echo "[rtab_test] /odom rate (5s window):"
ros2 topic hz /odom -w 20 || true
echo ""

# 4) rtabmap (mapping)
run_node "RTABMAP" ros2 run rtabmap_slam rtabmap --ros-args \
  -p subscribe_rgbd:=true \
  -r /rgbd_image:=/rgbd_image \
  -r /odom:=/odom \
  -p frame_id:="$BASE_FRAME" \
  -p odom_frame_id:="$ODOM_FRAME" \
  -p map_frame_id:="$MAP_FRAME" \
  -p publish_tf:=true \
  -- \
  --delete_db_on_start

sleep 1

# 5) viz (GUI)
run_node "VIZ" ros2 run rtabmap_viz rtabmap_viz --ros-args \
  -p subscribe_rgbd:=true \
  -r /rgbd_image:=/rgbd_image \
  -r /odom:=/odom \
  -p frame_id:="$BASE_FRAME" \
  -p odom_frame_id:="$ODOM_FRAME" \
  -p map_frame_id:="$MAP_FRAME"

echo ""
echo "[rtab_test] Running. Watch the prefixed logs above."
echo "[rtab_test] If TF tree errors persist, try setting OAK_ROOT=\"oak-d-base-frame\"."
echo "[rtab_test] Ctrl+C here stops everything."
wait
