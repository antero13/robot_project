from ctypes import c_int, c_int32, c_uint8, c_void_p


def configure_pimoroni_ctypes(vl53l1x_module):
    """Declare the C ABI used by Pimoroni's VL53L1X ctypes wrapper.

    Pimoroni's Python module leaves these signatures unspecified. ctypes then
    assumes that ``initialise`` returns a 32-bit C int, although the native
    function actually returns a device pointer. That truncates the handle on
    64-bit systems such as Jetson and can cause SIGSEGV on the next driver call.
    """
    library = getattr(vl53l1x_module, "_TOF_LIBRARY", None)
    if library is None:
        return False

    signatures = {
        "initialise": (
            [c_uint8, c_uint8, c_uint8, c_uint8],
            c_void_p,
        ),
        "setDeviceAddress": ([c_void_p, c_int], c_int),
        "setDistanceMode": ([c_void_p, c_int], c_int),
        "setUserRoi": (
            [c_void_p, c_int, c_int, c_int, c_int],
            c_int,
        ),
        "startRanging": ([c_void_p, c_int], c_int),
        "setMeasurementTimingBudgetMicroSeconds": (
            [c_void_p, c_int],
            c_int,
        ),
        "setInterMeasurementPeriodMilliSeconds": (
            [c_void_p, c_int],
            c_int,
        ),
        "getDistance": ([c_void_p], c_int32),
        "stopRanging": ([c_void_p], None),
    }

    configured = False
    for function_name, (argument_types, return_type) in signatures.items():
        function = getattr(library, function_name, None)
        if function is None:
            continue
        function.argtypes = argument_types
        function.restype = return_type
        configured = True

    return configured
