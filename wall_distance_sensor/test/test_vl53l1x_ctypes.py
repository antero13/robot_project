import unittest
from ctypes import c_int, c_int32, c_uint8, c_void_p
from types import SimpleNamespace

from wall_distance_sensor.vl53l1x_ctypes import configure_pimoroni_ctypes


class FakeFunction:
    def __init__(self):
        self.argtypes = None
        self.restype = c_int


class ConfigurePimoroniCtypesTest(unittest.TestCase):
    def test_configures_pointer_return_and_device_arguments(self):
        library = SimpleNamespace(
            initialise=FakeFunction(),
            setDistanceMode=FakeFunction(),
            getDistance=FakeFunction(),
            stopRanging=FakeFunction(),
        )
        module = SimpleNamespace(_TOF_LIBRARY=library)

        self.assertTrue(configure_pimoroni_ctypes(module))
        self.assertEqual(
            library.initialise.argtypes,
            [c_uint8, c_uint8, c_uint8, c_uint8],
        )
        self.assertIs(library.initialise.restype, c_void_p)
        self.assertEqual(library.setDistanceMode.argtypes, [c_void_p, c_int])
        self.assertEqual(library.getDistance.argtypes, [c_void_p])
        self.assertIs(library.getDistance.restype, c_int32)
        self.assertEqual(library.stopRanging.argtypes, [c_void_p])
        self.assertIsNone(library.stopRanging.restype)

    def test_ignores_non_pimoroni_driver_modules(self):
        self.assertFalse(configure_pimoroni_ctypes(SimpleNamespace()))


if __name__ == "__main__":
    unittest.main()
