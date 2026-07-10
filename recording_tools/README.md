# Dual-camera recording tools

The recorder uses a separate environment profile, ROS topic, and node name for each
camera. This prevents frames from two cameras from being interleaved in one MP4 file.

## Profiles

- `camera_settings.env`: camera 1, USB path `2.1`, exposure `200`
- `camera2_settings.env`: camera 2, USB path `2.2`, exposure `500`

Both profiles use the `video-index0` capture device. Do not change them to
`video-index1`, which is commonly the camera's non-capture interface.

## Install on Jetson

After pulling the repository:

```bash
mkdir -p ~/recording_tools
cp -a ~/recording_tools ~/recording_tools_backup_$(date +%Y%m%d_%H%M%S)

if [[ ! -f ~/recording_tools/camera_settings.env ]]; then
  cp ~/ros2_ws/src/robot_project/recording_tools/camera_settings.env \
    ~/recording_tools/camera_settings.env
fi

ln -sfn ~/ros2_ws/src/robot_project/recording_tools/camera2_settings.env \
  ~/recording_tools/camera2_settings.env
ln -sfn ~/ros2_ws/src/robot_project/recording_tools/record_once.sh \
  ~/recording_tools/record_once.sh
ln -sfn ~/ros2_ws/src/robot_project/recording_tools/record_both.sh \
  ~/recording_tools/record_both.sh

chmod +x ~/recording_tools/record_once.sh ~/recording_tools/record_both.sh
```

The existing camera 1 `camera_settings.env` is preserved. The linked camera 2 profile
and scripts update automatically on future Git pulls.

## Record one camera

Camera 1 remains the default, so the original command still works:

```bash
bash ~/recording_tools/record_once.sh
```

Select either camera explicitly:

```bash
bash ~/recording_tools/record_once.sh camera1
bash ~/recording_tools/record_once.sh camera2
```

Files are written as `camera1_YYYYMMDD_HHMMSS.mp4` and
`camera2_YYYYMMDD_HHMMSS.mp4` under `/home/airobot/recordings`.

## Record both cameras

```bash
bash ~/recording_tools/record_both.sh
```

The streams use isolated topics:

```text
/recording/camera1/image_raw
/recording/camera2/image_raw
```

Two 1920x1080 YUYV streams can consume substantial USB bandwidth. If simultaneous
recording fails while individual recording works, lower the resolution/FPS or use a
camera-supported compressed pixel format.

Stop the autonomous mission launch before recording. A camera cannot be opened by the
mission and the recording tool at the same time.
