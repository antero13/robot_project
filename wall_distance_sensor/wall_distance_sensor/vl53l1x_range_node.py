import importlib
import math
from collections import deque
from statistics import median

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class MockDistanceSensor:
    def __init__(self, distance_m):
        self.distance_m = float(distance_m)

    def read_m(self):
        return self.distance_m

    def stop(self):
        return None


class Vl53l1xDistanceSensor:
    """Own exactly one VL53L1X native driver instance in this process."""

    def __init__(
        self,
        *,
        i2c_bus,
        address,
        ranging_mode,
        distance_scale_m,
        timing_budget_us,
        inter_measurement_period_ms,
    ):
        self.distance_scale_m = float(distance_scale_m)
        self.sensor = None
        self.module = importlib.import_module("VL53L1X")
        self.sensor = self.module.VL53L1X(
            i2c_bus=int(i2c_bus),
            i2c_address=int(address),
        )
        if hasattr(self.sensor, "open"):
            self.sensor.open()
        self._start_ranging(
            ranging_mode=int(ranging_mode),
            timing_budget_us=int(timing_budget_us),
            inter_measurement_period_ms=int(inter_measurement_period_ms),
        )

    def _start_ranging(
        self,
        *,
        ranging_mode,
        timing_budget_us,
        inter_measurement_period_ms,
    ):
        if ranging_mode not in (1, 2, 3):
            raise ValueError("ranging_mode must be 1=short, 2=medium, or 3=long")

        start_ranging = getattr(self.sensor, "start_ranging", None)
        if start_ranging is None:
            raise RuntimeError("VL53L1X driver has no start_ranging method")

        set_distance_mode = getattr(self.sensor, "set_distance_mode", None)
        set_timing = getattr(self.sensor, "set_timing", None)
        if set_distance_mode is not None:
            set_distance_mode(ranging_mode)
        if set_timing is not None:
            set_timing(timing_budget_us, inter_measurement_period_ms)

        # Pimoroni's driver uses 0 to preserve explicitly configured mode/timing.
        # Drivers without those setters receive the mode through start_ranging.
        start_ranging(
            0
            if set_distance_mode is not None and set_timing is not None
            else ranging_mode
        )

    def read_m(self):
        get_distance = getattr(self.sensor, "get_distance", None)
        if get_distance is None:
            raise RuntimeError("VL53L1X driver has no get_distance method")
        return float(get_distance()) * self.distance_scale_m

    def stop(self):
        if self.sensor is None:
            return
        if hasattr(self.sensor, "stop_ranging"):
            self.sensor.stop_ranging()
        if hasattr(self.sensor, "close"):
            self.sensor.close()
        self.sensor = None


class Vl53l1xRangeNode(Node):
    def __init__(self):
        super().__init__("vl53l1x_range_node")

        self.declare_parameter("driver_backend", "vl53l1x")
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x29)
        self.declare_parameter("ranging_mode", 3)
        self.declare_parameter("distance_scale_m", 0.001)
        self.declare_parameter("timing_budget_us", 41000)
        self.declare_parameter("inter_measurement_period_ms", 50)
        self.declare_parameter("min_valid_distance_m", 0.02)
        self.declare_parameter("max_valid_distance_m", 4.0)
        self.declare_parameter("filter_window_size", 3)
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("field_of_view_rad", 0.47)
        self.declare_parameter("frame_id", "front_tof")
        self.declare_parameter("range_topic", "/wall/range")
        self.declare_parameter("mock_distance_m", 0.5)

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.address = int(self.get_parameter("address").value)
        self.ranging_mode = int(self.get_parameter("ranging_mode").value)
        self.min_valid_distance_m = self.get_float("min_valid_distance_m")
        self.max_valid_distance_m = self.get_float("max_valid_distance_m")
        self.field_of_view_rad = self.get_float("field_of_view_rad")
        self.frame_id = str(self.get_parameter("frame_id").value)
        update_rate_hz = self.get_float("update_rate_hz")
        window_size = int(self.get_parameter("filter_window_size").value)
        self.validate_parameters(update_rate_hz, window_size)
        self.window = deque(maxlen=window_size)

        self.range_pub = self.create_publisher(
            Range,
            str(self.get_parameter("range_topic").value),
            10,
        )
        self.sensor = self.create_sensor()
        self.timer = self.create_timer(1.0 / update_rate_hz, self.timer_callback)
        self.get_logger().info(
            "Single VL53L1X range node ready: "
            f"bus={self.i2c_bus}, address=0x{self.address:02x}, "
            f"mode={self.ranging_mode}, frame={self.frame_id}"
        )

    def validate_parameters(self, update_rate_hz, window_size):
        if self.address < 0x08 or self.address > 0x77:
            raise ValueError("address must be a valid 7-bit I2C address")
        if self.ranging_mode not in (1, 2, 3):
            raise ValueError("ranging_mode must be 1, 2, or 3")
        if update_rate_hz <= 0.0:
            raise ValueError("update_rate_hz must be positive")
        if window_size <= 0:
            raise ValueError("filter_window_size must be positive")
        if not 0.0 < self.min_valid_distance_m < self.max_valid_distance_m:
            raise ValueError("invalid distance limits")

    def create_sensor(self):
        backend = str(self.get_parameter("driver_backend").value).strip().lower()
        if backend == "mock":
            return MockDistanceSensor(self.get_float("mock_distance_m"))
        if backend != "vl53l1x":
            raise ValueError("driver_backend must be 'vl53l1x' or 'mock'")
        return Vl53l1xDistanceSensor(
            i2c_bus=self.i2c_bus,
            address=self.address,
            ranging_mode=self.ranging_mode,
            distance_scale_m=self.get_float("distance_scale_m"),
            timing_budget_us=int(self.get_parameter("timing_budget_us").value),
            inter_measurement_period_ms=int(
                self.get_parameter("inter_measurement_period_ms").value
            ),
        )

    def timer_callback(self):
        try:
            raw_distance_m = self.sensor.read_m()
        except Exception as exc:
            self.get_logger().warning(f"VL53L1X read failed: {exc}")
            return

        if (
            not math.isfinite(raw_distance_m)
            or raw_distance_m < self.min_valid_distance_m
            or raw_distance_m > self.max_valid_distance_m
        ):
            self.get_logger().warning(
                f"Invalid VL53L1X distance: {raw_distance_m:.3f} m"
            )
            return

        self.window.append(float(raw_distance_m))
        distance_m = float(median(self.window))
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.field_of_view_rad
        msg.min_range = self.min_valid_distance_m
        msg.max_range = self.max_valid_distance_m
        msg.range = distance_m
        self.range_pub.publish(msg)

    def get_float(self, name):
        return float(self.get_parameter(name).value)

    def destroy_node(self):
        if hasattr(self, "sensor"):
            self.sensor.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Vl53l1xRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
