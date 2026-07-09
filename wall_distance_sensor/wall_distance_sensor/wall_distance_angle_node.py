import importlib
import json
import math
import time
from collections import deque
from statistics import median
from typing import Any

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


class MockToFSensorPair:
    def __init__(self, left_distance_m: float, right_distance_m: float) -> None:
        self.left_distance_m = left_distance_m
        self.right_distance_m = right_distance_m

    def read(self) -> tuple[float, float]:
        return self.left_distance_m, self.right_distance_m

    def stop(self) -> None:
        return


class Vl53l1xSensorPair:
    def __init__(
        self,
        *,
        left_i2c_bus: int,
        right_i2c_bus: int,
        default_address: int,
        left_address: int,
        right_address: int,
        left_xshut_pin: int,
        right_xshut_pin: int,
        xshut_pin_mode: str,
        ranging_mode: int,
        distance_scale_m: float,
        logger,
    ) -> None:
        self.left_i2c_bus = left_i2c_bus
        self.right_i2c_bus = right_i2c_bus
        self.default_address = default_address
        self.left_address = left_address
        self.right_address = right_address
        self.left_xshut_pin = left_xshut_pin
        self.right_xshut_pin = right_xshut_pin
        self.xshut_pin_mode = xshut_pin_mode
        self.ranging_mode = ranging_mode
        self.distance_scale_m = distance_scale_m
        self.logger = logger
        self.gpio = None
        self.left_sensor = None
        self.right_sensor = None
        self.vl53l1x = importlib.import_module("VL53L1X")

        self._open_sensors()

    def _open_sensors(self) -> None:
        if self.left_xshut_pin >= 0 and self.right_xshut_pin >= 0:
            self._open_sensors_with_xshut()
        else:
            self.left_sensor = self._make_sensor(self.left_i2c_bus, self.left_address)
            self.right_sensor = self._make_sensor(self.right_i2c_bus, self.right_address)

        self._start_ranging(self.left_sensor)
        self._start_ranging(self.right_sensor)

    def _open_sensors_with_xshut(self) -> None:
        self._setup_gpio()
        self._set_xshut(self.left_xshut_pin, False)
        self._set_xshut(self.right_xshut_pin, False)
        time.sleep(0.05)

        self._set_xshut(self.left_xshut_pin, True)
        time.sleep(0.10)
        self.left_sensor = self._make_sensor_at_default_then_readdress(
            self.left_i2c_bus,
            self.left_address,
        )

        self._set_xshut(self.right_xshut_pin, True)
        time.sleep(0.10)
        self.right_sensor = self._make_sensor_at_default_then_readdress(
            self.right_i2c_bus,
            self.right_address,
        )

    def _setup_gpio(self) -> None:
        try:
            self.gpio = importlib.import_module("Jetson.GPIO")
        except ImportError as exc:
            raise RuntimeError(
                "Jetson.GPIO is required when left_xshut_pin and right_xshut_pin are set."
            ) from exc

        mode_name = self.xshut_pin_mode.strip().upper()
        if not hasattr(self.gpio, mode_name):
            raise ValueError(f"Unknown Jetson.GPIO pin mode: {self.xshut_pin_mode}")

        self.gpio.setmode(getattr(self.gpio, mode_name))
        for pin in (self.left_xshut_pin, self.right_xshut_pin):
            self.gpio.setup(pin, self.gpio.OUT)

    def _set_xshut(self, pin: int, enabled: bool) -> None:
        if self.gpio is None:
            return
        self.gpio.output(pin, self.gpio.HIGH if enabled else self.gpio.LOW)

    def _make_sensor_at_default_then_readdress(self, i2c_bus: int, target_address: int) -> Any:
        sensor = self._make_sensor(i2c_bus, self.default_address)
        if target_address == self.default_address:
            return sensor

        self._change_address(sensor, target_address)
        self._close_sensor(sensor)
        return self._make_sensor(i2c_bus, target_address)

    def _make_sensor(self, i2c_bus: int, address: int) -> Any:
        sensor = self.vl53l1x.VL53L1X(i2c_bus=i2c_bus, i2c_address=address)
        if hasattr(sensor, "open"):
            sensor.open()
        return sensor

    def _change_address(self, sensor: Any, target_address: int) -> None:
        for method_name in ("change_address", "set_i2c_address", "set_address"):
            method = getattr(sensor, method_name, None)
            if method is not None:
                method(target_address)
                return
        raise RuntimeError(
            "The installed VL53L1X Python driver does not expose an address-change method. "
            "Use XSHUT plus a driver with change_address support, preconfigure the addresses, "
            "or use an I2C multiplexer."
        )

    def _start_ranging(self, sensor: Any) -> None:
        method = getattr(sensor, "start_ranging", None)
        if method is None:
            raise RuntimeError("The installed VL53L1X Python driver has no start_ranging method.")
        method(self.ranging_mode)

    def read(self) -> tuple[float, float]:
        return self._read_sensor_m(self.left_sensor), self._read_sensor_m(self.right_sensor)

    def _read_sensor_m(self, sensor: Any) -> float:
        method = getattr(sensor, "get_distance", None)
        if method is None:
            raise RuntimeError("The installed VL53L1X Python driver has no get_distance method.")
        return float(method()) * self.distance_scale_m

    def stop(self) -> None:
        for sensor in (self.left_sensor, self.right_sensor):
            if sensor is not None and hasattr(sensor, "stop_ranging"):
                sensor.stop_ranging()
            self._close_sensor(sensor)
        if self.gpio is not None:
            self.gpio.cleanup()

    @staticmethod
    def _close_sensor(sensor: Any) -> None:
        if sensor is not None and hasattr(sensor, "close"):
            sensor.close()


class WallDistanceAngleNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_distance_angle_node")

        self.declare_parameter("driver_backend", "vl53l1x")
        self.declare_parameter("left_i2c_bus", 1)
        self.declare_parameter("right_i2c_bus", 0)
        self.declare_parameter("default_address", "0x29")
        self.declare_parameter("left_address", "0x29")
        self.declare_parameter("right_address", "0x29")
        self.declare_parameter("left_xshut_pin", -1)
        self.declare_parameter("right_xshut_pin", -1)
        self.declare_parameter("xshut_pin_mode", "BOARD")
        self.declare_parameter("ranging_mode", 1)
        self.declare_parameter("distance_scale_m", 0.001)
        self.declare_parameter("sensor_separation_m", 0.29)
        self.declare_parameter("safe_distance_m", 0.15)
        self.declare_parameter("min_valid_distance_m", 0.02)
        self.declare_parameter("max_valid_distance_m", 4.00)
        self.declare_parameter("filter_window_size", 3)
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("field_of_view_rad", 0.47)
        self.declare_parameter("measurement_frame_id", "front_wall_sensors")
        self.declare_parameter("left_frame_id", "front_left_tof")
        self.declare_parameter("right_frame_id", "front_right_tof")
        self.declare_parameter("distance_angle_topic", "/wall/distance_angle")
        self.declare_parameter("measurement_json_topic", "/wall/measurement_json")
        self.declare_parameter("left_range_topic", "/wall/left_range")
        self.declare_parameter("right_range_topic", "/wall/right_range")
        self.declare_parameter("mock_left_distance_m", 0.50)
        self.declare_parameter("mock_right_distance_m", 0.50)

        self.sensor_separation_m = self.get_float("sensor_separation_m")
        if self.sensor_separation_m <= 0.0:
            raise ValueError("sensor_separation_m must be greater than 0.")

        self.safe_distance_m = self.get_float("safe_distance_m")
        self.min_valid_distance_m = self.get_float("min_valid_distance_m")
        self.max_valid_distance_m = self.get_float("max_valid_distance_m")
        self.field_of_view_rad = self.get_float("field_of_view_rad")
        self.measurement_frame_id = str(self.get_parameter("measurement_frame_id").value)
        self.left_frame_id = str(self.get_parameter("left_frame_id").value)
        self.right_frame_id = str(self.get_parameter("right_frame_id").value)

        window_size = max(1, int(self.get_parameter("filter_window_size").value))
        self.left_window = deque(maxlen=window_size)
        self.right_window = deque(maxlen=window_size)

        self.distance_angle_pub = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("distance_angle_topic").value),
            10,
        )
        self.measurement_json_pub = self.create_publisher(
            String,
            str(self.get_parameter("measurement_json_topic").value),
            10,
        )
        self.left_range_pub = self.create_publisher(
            Range,
            str(self.get_parameter("left_range_topic").value),
            10,
        )
        self.right_range_pub = self.create_publisher(
            Range,
            str(self.get_parameter("right_range_topic").value),
            10,
        )

        self.sensor_pair = self.create_sensor_pair()
        update_rate_hz = max(0.1, self.get_float("update_rate_hz"))
        self.timer = self.create_timer(1.0 / update_rate_hz, self.timer_callback)
        self.get_logger().info(
            "Wall distance/angle node ready. "
            f"sensor_separation_m={self.sensor_separation_m:.3f}, "
            f"safe_distance_m={self.safe_distance_m:.3f}"
        )

    def create_sensor_pair(self):
        backend = str(self.get_parameter("driver_backend").value).strip().lower()
        if backend == "mock":
            return MockToFSensorPair(
                self.get_float("mock_left_distance_m"),
                self.get_float("mock_right_distance_m"),
            )
        if backend != "vl53l1x":
            raise ValueError("driver_backend must be 'vl53l1x' or 'mock'.")

        return Vl53l1xSensorPair(
            left_i2c_bus=int(self.get_parameter("left_i2c_bus").value),
            right_i2c_bus=int(self.get_parameter("right_i2c_bus").value),
            default_address=self.parse_int_parameter("default_address"),
            left_address=self.parse_int_parameter("left_address"),
            right_address=self.parse_int_parameter("right_address"),
            left_xshut_pin=int(self.get_parameter("left_xshut_pin").value),
            right_xshut_pin=int(self.get_parameter("right_xshut_pin").value),
            xshut_pin_mode=str(self.get_parameter("xshut_pin_mode").value),
            ranging_mode=int(self.get_parameter("ranging_mode").value),
            distance_scale_m=self.get_float("distance_scale_m"),
            logger=self.get_logger(),
        )

    def timer_callback(self) -> None:
        try:
            raw_left_m, raw_right_m = self.sensor_pair.read()
        except Exception as exc:
            self.get_logger().warning(f"Failed to read wall ToF sensors: {exc}")
            return

        left_m = self.filtered_distance(raw_left_m, self.left_window)
        right_m = self.filtered_distance(raw_right_m, self.right_window)
        if left_m is None or right_m is None:
            self.get_logger().warning(
                f"Invalid ToF distance. left={raw_left_m:.3f}m right={raw_right_m:.3f}m"
            )
            return

        angle_rad = math.atan2(right_m - left_m, self.sensor_separation_m)
        distance_m = ((left_m + right_m) * 0.5) * math.cos(angle_rad)
        min_distance_m = min(left_m, right_m)
        too_close = distance_m <= self.safe_distance_m
        stamp = self.get_clock().now().to_msg()

        self.publish_range(self.left_range_pub, self.left_frame_id, stamp, left_m)
        self.publish_range(self.right_range_pub, self.right_frame_id, stamp, right_m)
        self.publish_distance_angle(stamp, distance_m, angle_rad, min_distance_m)
        self.publish_measurement_json(
            stamp,
            left_m,
            right_m,
            distance_m,
            angle_rad,
            min_distance_m,
            too_close,
        )

    def filtered_distance(self, value: float, window: deque) -> float | None:
        if not math.isfinite(value):
            return None
        if value < self.min_valid_distance_m or value > self.max_valid_distance_m:
            return None
        window.append(float(value))
        return float(median(window))

    def publish_range(self, publisher, frame_id: str, stamp, distance_m: float) -> None:
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float(self.field_of_view_rad)
        msg.min_range = float(self.min_valid_distance_m)
        msg.max_range = float(self.max_valid_distance_m)
        msg.range = float(distance_m)
        publisher.publish(msg)

    def publish_distance_angle(
        self,
        stamp,
        distance_m: float,
        angle_rad: float,
        min_distance_m: float,
    ) -> None:
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.measurement_frame_id
        msg.vector.x = float(distance_m)
        msg.vector.y = float(angle_rad)
        msg.vector.z = float(min_distance_m)
        self.distance_angle_pub.publish(msg)

    def publish_measurement_json(
        self,
        stamp,
        left_m: float,
        right_m: float,
        distance_m: float,
        angle_rad: float,
        min_distance_m: float,
        too_close: bool,
    ) -> None:
        payload = {
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "frame_id": self.measurement_frame_id,
            "left_distance_m": float(left_m),
            "right_distance_m": float(right_m),
            "wall_distance_m": float(distance_m),
            "wall_angle_rad": float(angle_rad),
            "wall_angle_deg": float(math.degrees(angle_rad)),
            "min_distance_m": float(min_distance_m),
            "safe_distance_m": float(self.safe_distance_m),
            "too_close": bool(too_close),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.measurement_json_pub.publish(msg)

    def parse_int_parameter(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, int):
            return value
        return int(str(value), 0)

    def get_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def destroy_node(self) -> bool:
        if hasattr(self, "sensor_pair"):
            self.sensor_pair.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallDistanceAngleNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
