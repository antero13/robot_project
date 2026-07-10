#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROFILE="${1:-camera1}"

case "$PROFILE" in
  camera1)
    ENV_FILE="$SCRIPT_DIR/camera_settings.env"
    ;;
  camera2)
    ENV_FILE="$SCRIPT_DIR/camera2_settings.env"
    ;;
  *)
    if [[ -f "$PROFILE" ]]; then
      ENV_FILE="$(realpath "$PROFILE")"
    else
      echo "Unknown camera profile: $PROFILE" >&2
      echo "Use camera1, camera2, or an environment file path." >&2
      exit 2
    fi
    ;;
esac

set +u
source /opt/ros/humble/setup.bash
source /home/airobot/ros2_ws/install/setup.bash
set -u
source "$ENV_FILE"

if [[ "$PROFILE" == "camera1" || "$PROFILE" == "camera2" ]]; then
  default_camera_name="$PROFILE"
else
  default_camera_name="$(basename "$ENV_FILE" .env)"
fi
CAMERA_NAME="${CAMERA_NAME:-$default_camera_name}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/recording/${CAMERA_NAME}/image_raw}"

required_variables=(
  VIDEO_DEVICE WIDTH HEIGHT FPS DURATION PIXEL_FORMAT OUTPUT_ENCODING
  POWER_LINE_FREQUENCY AUTO_EXPOSURE
  EXPOSURE GAIN BRIGHTNESS CONTRAST SATURATION SHARPNESS OUTPUT_DIR
)
for variable in "${required_variables[@]}"; do
  if [[ -z "${!variable:-}" ]]; then
    echo "Missing $variable in $ENV_FILE" >&2
    exit 2
  fi
done

if [[ ! -e "$VIDEO_DEVICE" ]]; then
  echo "Camera device does not exist: $VIDEO_DEVICE" >&2
  exit 1
fi

if command -v fuser >/dev/null 2>&1 && fuser "$VIDEO_DEVICE" >/dev/null 2>&1; then
  echo "Camera device is already in use: $VIDEO_DEVICE" >&2
  fuser -v "$VIDEO_DEVICE" || true
  echo "Stop the mission/camera launch before recording." >&2
  exit 1
fi

mkdir -p "$OUTPUT_DIR"

TIMESTAMP="$(date +"%Y%m%d_%H%M%S")"
OUTPUT_FILE="$OUTPUT_DIR/${CAMERA_NAME}_${TIMESTAMP}.mp4"
CAMERA_PID=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" 2>/dev/null; then
    echo "Stopping $CAMERA_NAME..."
    kill -INT -- "-$CAMERA_PID" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$CAMERA_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-$CAMERA_PID" 2>/dev/null || true
    wait "$CAMERA_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

echo "Starting $CAMERA_NAME"
echo "  device:   $VIDEO_DEVICE"
echo "  topic:    $IMAGE_TOPIC"
echo "  exposure: $EXPOSURE"

setsid ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __node:="${CAMERA_NAME}_v4l2_camera" \
  -r /image_raw:="$IMAGE_TOPIC" \
  -r /camera_info:="/recording/${CAMERA_NAME}/camera_info" \
  -p video_device:="$VIDEO_DEVICE" \
  -p image_size:="[$WIDTH,$HEIGHT]" \
  -p time_per_frame:="[1,$FPS]" \
  -p pixel_format:="$PIXEL_FORMAT" \
  -p output_encoding:="$OUTPUT_ENCODING" \
  -p power_line_frequency:="$POWER_LINE_FREQUENCY" \
  -p auto_exposure:="$AUTO_EXPOSURE" \
  -p exposure_time_absolute:="$EXPOSURE" \
  -p gain:="$GAIN" \
  -p brightness:="$BRIGHTNESS" \
  -p contrast:="$CONTRAST" \
  -p saturation:="$SATURATION" \
  -p sharpness:="$SHARPNESS" &

CAMERA_PID=$!

sleep 3
if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
  echo "$CAMERA_NAME exited before publishing images." >&2
  wait "$CAMERA_PID" || true
  exit 1
fi

if ! ros2 topic info "$IMAGE_TOPIC" 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  echo "No publisher appeared on $IMAGE_TOPIC" >&2
  echo "Publisher details from /${CAMERA_NAME}_v4l2_camera:" >&2
  ros2 node info "/${CAMERA_NAME}_v4l2_camera" 2>/dev/null || true
  exit 1
fi

echo "Recording $CAMERA_NAME to $OUTPUT_FILE"
set +e
timeout --signal=INT --kill-after=3s "$DURATION" \
  ros2 run mp4_recorder record_mp4 --ros-args \
    -r __node:="${CAMERA_NAME}_mp4_recorder" \
    -p image_topic:="$IMAGE_TOPIC" \
    -p output:="$OUTPUT_FILE" \
    -p fps:="${FPS}.0"
recorder_status=$?
set -e

if [[ -s "$OUTPUT_FILE" ]]; then
  echo "Saved: $OUTPUT_FILE"
  exit 0
fi

echo "Recording failed for $CAMERA_NAME (status=$recorder_status)" >&2
exit 1
