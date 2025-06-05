#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <structmember.h>

#include "nicolay_flow_sensor.h"

// --- Helper Functions for Error Handling ---

/**
 * @brief Converts a NicolayError code to a Python exception.
 *        Sets a Python exception and returns NULL.
 * @param error_code The NicolayError code.
 * @return NULL (to propagate the exception).
 */
static PyObject *NicolayError_Set(NicolayError error_code) {
    PyObject *exception_type = PyExc_RuntimeError; // Default
    const char *error_msg = "An unknown Nicolay error occurred.";

    if (error_code >= NICOLAY_ERR_EXCEPTION_BASE) {
        // Device-specific exceptions
        exception_type = PyExc_IOError; // More specific than RuntimeError
        int device_exception_code = NICOLAY_ERR_EXCEPTION_BASE - error_code;
        // You might want to map these to more specific Python exceptions
        // or just include the code in the message.
        switch (device_exception_code) {
        case NICOLAY_EXC_FUNC_UNKNOWN:
            error_msg = "Nicolay device error: Unknown function.";
            break;
        case NICOLAY_EXC_WRONG_NUM_DATA:
            error_msg = "Nicolay device error: Wrong number of data bytes.";
            break;
        // Add more cases for specific NICOLAY_EXC_* codes
        default:
            PyErr_Format(PyExc_IOError, "Nicolay device returned exception "
                                        "code: %d",
                         device_exception_code);
            return NULL;
        }
    } else {
        switch (error_code) {
        case NICOLAY_OK: // Should not be called with OK
            return NULL;
        case NICOLAY_ERR_SERIAL_OPEN_FAILED:
            exception_type = PyExc_OSError;
            error_msg = "Serial port open failed.";
            break;
        case NICOLAY_ERR_SERIAL_WRITE_FAILED:
            exception_type = PyExc_IOError;
            error_msg = "Serial write failed.";
            break;
        case NICOLAY_ERR_SERIAL_READ_FAILED:
            exception_type = PyExc_IOError;
            error_msg = "Serial read failed.";
            break;
        case NICOLAY_ERR_TIMEOUT:
            exception_type = PyExc_TimeoutError; // Python 3.x
            error_msg = "Response timeout.";
            break;
        case NICOLAY_ERR_INVALID_RESPONSE:
            exception_type = PyExc_ValueError;
            error_msg = "Invalid device response (e.g., wrong address).";
            break;
        case NICOLAY_ERR_CRC_MISMATCH:
            exception_type = PyExc_ValueError;
            error_msg = "CRC mismatch in response.";
            break;
        case NICOLAY_ERR_INVALID_RESPONSE_LEN:
            exception_type = PyExc_ValueError;
            error_msg = "Invalid response length.";
            break;
        case NICOLAY_ERR_INVALID_PARAMETER:
            exception_type = PyExc_ValueError;
            error_msg = "Invalid parameter passed to C function.";
            break;
        case NICOLAY_ERR_BAUDRATE_CHANGE_FAILED:
            exception_type = PyExc_OSError;
            error_msg = "Baud rate change failed.";
            break;
        default:
            error_msg = "An unexpected Nicolay error occurred.";
            break;
        }
    }
    PyErr_SetString(exception_type, error_msg);
    return NULL;
}

// --- NicolaySensor Python Type Definition ---

// This struct represents a Python object that wraps our C NicolaySensor
typedef struct {
    PyObject_HEAD
    NicolaySensor sensor; // The actual C struct instance
    int is_open;          // Flag to track if the sensor is initialized/open
} NicolaySensorObject;

// Deallocation (destructor)
static void NicolaySensor_dealloc(NicolaySensorObject *self) {
    if (self->is_open) {
        // Attempt to close the serial port if it's still open
        // Ignore the return value here as we're deallocating anyway
        nicolay_close(&self->sensor);
    }
    Py_TYPE(self)->tp_free((PyObject *)self); // Free the object's memory
}

// Initialization (__init__)
// This function corresponds to NicolaySensor(...) in Python
static int NicolaySensor_init(NicolaySensorObject *self, PyObject *args,
                              PyObject *kwds) {
    const char *port_name = NULL;
    int slave_address = NICOLAY_DEFAULT_SLAVE_ADDRESS; // Default value

    static char *kwlist[] = {"port", "slave_address", NULL};

    // Parse arguments: "s|b" means string for port, optional byte for slave_address
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|b", kwlist, &port_name,
                                     &slave_address)) {
        return -1; // Parsing failed, Python exception is already set
    }

    if (slave_address < 0 || slave_address > 255) {
        PyErr_SetString(PyExc_ValueError, "Slave address must be between 0 and 255.");
        return -1;
    }

    // Call your C library's init function
    NicolayError err = nicolay_init(&self->sensor, port_name, (uint8_t)slave_address);

    if (err != NICOLAY_OK) {
        self->is_open = 0; // Mark as not open
        NicolayError_Set(err);
        return -1; // Indicate initialization failure
    }

    self->is_open = 1; // Mark as successfully opened
    self->sensor.slave_address = (uint8_t)slave_address; // Store actual address
    // Store current baud rate (initially 115200 or whatever default Nicolay uses)
    // You might need to add a way to get the initial baud rate from your C library if it's not fixed.
    self->sensor.current_baud_rate = 115200; // Placeholder, adjust as needed

    return 0; // Success
}

// Method for nicolay_close()
static PyObject *NicolaySensor_close(NicolaySensorObject *self, PyObject *Py_UNUSED(ignored)) {
    if (!self->is_open) {
        Py_RETURN_NONE; // Already closed
    }

    NicolayError err = nicolay_close(&self->sensor);
    if (err != NICOLAY_OK) {
        return NicolayError_Set(err);
    }
    self->is_open = 0; // Mark as closed
    Py_RETURN_NONE;
}

// Method for nicolay_get_sw_version()
static PyObject *NicolaySensor_get_sw_version(NicolaySensorObject *self, PyObject *Py_UNUSED(ignored)) {
    NicolaySWVersion sw_version;
    NicolayError err = nicolay_get_sw_version(&self->sensor, &sw_version);

    if (err != NICOLAY_OK) {
        return NicolayError_Set(err);
    }

    // Return a Python dictionary or a custom object. Dictionary is simpler for now.
    return Py_BuildValue("{s:c, s:i, s:i}",
                         "index_ascii", sw_version.index_ascii,
                         "version_minor", sw_version.version_minor,
                         "version_major", sw_version.version_major);
}

// Method for nicolay_get_hw_version()
static PyObject *NicolaySensor_get_hw_version(NicolaySensorObject *self, PyObject *Py_UNUSED(ignored)) {
    NicolayHWVersion hw_version;
    NicolayError err = nicolay_get_hw_version(&self->sensor, &hw_version);

    if (err != NICOLAY_OK) {
        return NicolayError_Set(err);
    }

    return Py_BuildValue("{s:i, s:i}",
                         "version_minor", hw_version.version_minor,
                         "version_major", hw_version.version_major);
}

// Method for nicolay_test_command()
static PyObject *NicolaySensor_test_command(NicolaySensorObject *self, PyObject *Py_UNUSED(ignored)) {
    uint8_t byte1, byte2;
    NicolayError err = nicolay_test_command(&self->sensor, &byte1, &byte2);

    if (err != NICOLAY_OK) {
        return NicolayError_Set(err);
    }

    // Return as a tuple of two integers
    return Py_BuildValue("(ii)", byte1, byte2);
}

// Method for nicolay_get_pressure()
static PyObject *NicolaySensor_get_pressure(NicolaySensorObject *self, PyObject *Py_UNUSED(ignored)) {
    int16_t pressure_counts;
    NicolayError err = nicolay_get_pressure(&self->sensor, &pressure_counts);

    if (err != NICOLAY_OK) {
        return NicolayError_Set(err);
    }

    return Py_BuildValue("h", pressure_counts); // 'h' for short int (int16_t)
}

// Method for nicolay_get_flow_and_pressure()
static PyObject *NicolaySensor_get_flow_and_pressure(NicolaySensorObject *self, PyObject *Py_UNUSED(ignored)) {
    int32_t flow_mSlm;
    int16_t raw_pressure_counts;
    NicolayError err = nicolay_get_flow_and_pressure(&self->sensor, &flow_mSlm, &raw_pressure_counts);

    if (err != NICOLAY_OK) {
        return NicolayError_Set(err);
    }

    // Return as a tuple
    return Py_BuildValue("(ii)", flow_mSlm, raw_pressure_counts); // 'i' for int32_t, 'i' for int16_t
}


// Method for nicolay_set_uart_baud()
// This method takes a baud_code and returns the response_baud_code
static PyObject *NicolaySensor_set_uart_baud(NicolaySensorObject *self, PyObject *args) {
    uint8_t baud_code;
    if (!PyArg_ParseTuple(args, "B", &baud_code)) { // "B" for unsigned char (uint8_t)
        return NULL;
    }

    uint8_t response_baud_code;
    NicolayError err = nicolay_set_uart_baud(&self->sensor, baud_code, &response_baud_code);

    if (err != NICOLAY_OK) {
        return NicolayError_Set(err);
    }

    // IMPORTANT: After changing baud rate on the device,
    // your C library's platform_serial_set_baudrate must be called
    // or you must communicate to the user to reinitialize/reopen
    // the port at the new speed.
    // For now, let's assume `nicolay_set_uart_baud` in your C library
    // internally calls `platform_serial_set_baudrate` and updates `sensor->current_baud_rate`.
    // If not, you'll need to add that logic here or make it a user responsibility.
    // For this example, we'll return the response code directly.
    return Py_BuildValue("B", response_baud_code);
}


// --- List of methods for NicolaySensor type ---
static PyMethodDef NicolaySensor_methods[] = {
    {"close", (PyCFunction)NicolaySensor_close, METH_NOARGS,
     "Closes the serial connection to the Nicolay sensor."},
    {"get_sw_version", (PyCFunction)NicolaySensor_get_sw_version, METH_NOARGS,
     "Retrieves the sensor's software version."},
    {"get_hw_version", (PyCFunction)NicolaySensor_get_hw_version, METH_NOARGS,
     "Retrieves the sensor's hardware version."},
    {"test_command", (PyCFunction)NicolaySensor_test_command, METH_NOARGS,
     "Sends a test command (0x05) to the sensor. Expects (0x55, 0xAA)."},
    {"get_pressure", (PyCFunction)NicolaySensor_get_pressure, METH_NOARGS,
     "Retrieves the 16-bit pressure counts."},
    {"get_flow_and_pressure", (PyCFunction)NicolaySensor_get_flow_and_pressure, METH_NOARGS,
     "Retrieves both flow (mSlm^2) and raw pressure counts."},
    {"set_uart_baud", (PyCFunction)NicolaySensor_set_uart_baud, METH_VARARGS,
     "Sets the UART baud rate of the sensor using a baud code (0-15). "
     "Returns the baud code acknowledged by the device."},
    // Add more methods here for each nicolay_ function
    // Example: {"get_flow_measurement", (PyCFunction)NicolaySensor_get_flow_measurement, METH_NOARGS, "..."}
    // Example: {"get_heater_state", (PyCFunction)NicolaySensor_get_heater_state, METH_NOARGS, "..."}
    {NULL} // Sentinel
};

// --- Members (attributes) for NicolaySensor type ---
static PyMemberDef NicolaySensor_members[] = {
    {"slave_address", T_UBYTE, offsetof(NicolaySensorObject, sensor.slave_address), READONLY,
     "The slave address of the Nicolay sensor."},
    {"is_open", T_INT, offsetof(NicolaySensorObject, is_open), READONLY,
     "True if the sensor serial port is open, False otherwise."},
    {"current_baud_rate", T_INT, offsetof(NicolaySensorObject, sensor.current_baud_rate), READONLY,
     "The current baud rate configured for the serial port communication."},
    {NULL} // Sentinel
};


// --- Type definition for NicolaySensorObject ---
static PyTypeObject NicolaySensorType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "nicolay_flow_meter_connector.NicolaySensor", // ModuleName.ClassName
    .tp_doc = "Nicolay Flow Sensor object.",
    .tp_basicsize = sizeof(NicolaySensorObject),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_new = PyType_GenericNew, // Default object creation
    .tp_init = (initproc)NicolaySensor_init, // Our custom initializer
    .tp_dealloc = (destructor)NicolaySensor_dealloc, // Our custom deallocator
    .tp_methods = NicolaySensor_methods, // Our methods
    .tp_members = NicolaySensor_members, // Our attributes
};

// --- Module Methods ---
// These are functions directly accessible from the module (e.g., nicolay_sensor_lib.some_func())
// For this library, most functionality will be through the NicolaySensor object.
static PyMethodDef module_methods[] = {
    {NULL, NULL, 0, NULL} // Sentinel
};

// --- Module Definition Structure ---
static struct PyModuleDef nicolay_sensor_module = {
    PyModuleDef_HEAD_INIT,
    "nicolay_flow_meter_connector", // Name of the module
    "Python bindings for the Nicolay Flow Sensor C library.", // Module docstring
    -1, // Size of per-interpreter state of the module, or -1 if the module keeps state in global variables.
    module_methods // Module-level methods
};

// --- Module Initialization Function ---
// This is the entry point called when Python imports the module
PyMODINIT_FUNC PyInit_nicolay_flow_meter_connector(void) {
    PyObject *m;

    // Prepare NicolaySensorType
    if (PyType_Ready(&NicolaySensorType) < 0) {
        return NULL;
    }

    // Create the module
    m = PyModule_Create(&nicolay_sensor_module);
    if (m == NULL) {
        return NULL;
    }

    // Add NicolaySensorType to the module
    Py_INCREF(&NicolaySensorType);
    if (PyModule_AddObject(m, "NicolaySensor", (PyObject *)&NicolaySensorType) < 0) {
        Py_DECREF(&NicolaySensorType);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}
