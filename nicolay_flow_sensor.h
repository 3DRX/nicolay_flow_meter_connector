#ifndef NICOLAY_FLOW_SENSOR_H
#define NICOLAY_FLOW_SENSOR_H

#include <stddef.h> // For size_t
#include <stdint.h>

// == Platform-Specific Serial Communication Functions ==
// These functions must be implemented by the user for their specific
// microcontroller/platform. They are declared here as extern and should be
// defined in your project.

/**
 * @brief Opens and configures the serial port.
 * @param port_name String identifier for the port (e.g., "/dev/ttyUSB0",
 * "COM1"). May not be used on all MCUs.
 * @param baud_rate The baud rate to set (e.g., 115200).
 * @return A file descriptor or handle for the serial port, or -1 on error.
 */
extern int platform_serial_open(const char *port_name, int baud_rate);

/**
 * @brief Closes the serial port.
 * @param fd File descriptor or handle of the serial port.
 * @return 0 on success, -1 on error.
 */
extern int platform_serial_close(int fd);

/**
 * @brief Writes data to the serial port.
 * @param fd File descriptor or handle of the serial port.
 * @param buffer Pointer to the data to write.
 * @param len Number of bytes to write.
 * @return Number of bytes written, or -1 on error.
 */
extern int platform_serial_write(int fd, const uint8_t *buffer, size_t len);

/**
 * @brief Reads data from the serial port.
 * @param fd File descriptor or handle of the serial port.
 * @param buffer Pointer to the buffer to store read data.
 * @param max_len Maximum number of bytes to read.
 * @param timeout_ms Timeout in milliseconds for the read operation.
 * @return Number of bytes read, 0 on timeout, or -1 on error.
 */
extern int platform_serial_read(int fd, uint8_t *buffer, size_t max_len,
				int timeout_ms);

/**
 * @brief Changes the baud rate of an already open serial port.
 *        This is needed for the GET/Set_UartBaud command.
 * @param fd File descriptor or handle of the serial port.
 * @param baud_rate The new baud rate.
 * @return 0 on success, -1 on error.
 */
extern int platform_serial_set_baudrate(int fd, int new_baud_rate);

// == Protocol Constants ==
#define NICOLAY_DEFAULT_SLAVE_ADDRESS 0x01
#define NICOLAY_BROADCAST_ADDRESS 0x00
#define NICOLAY_IDENTIFY_ADDRESS 0xFF

#define NICOLAY_MAX_CMD_DATA_LEN 255
#define NICOLAY_MAX_RESPONSE_DATA_LEN 400 // For BulkRead (Cmd 29)
#define NICOLAY_FRAME_HEADER_SIZE 3 // DevAddr, FuncCode, NumData
#define NICOLAY_CRC_SIZE 1
#define NICOLAY_MAX_FRAME_LEN                                        \
	(NICOLAY_FRAME_HEADER_SIZE + NICOLAY_MAX_RESPONSE_DATA_LEN + \
	 NICOLAY_CRC_SIZE)

#define NICOLAY_DEFAULT_TIMEOUT_MS 500 // Default timeout for responses

// == Function Codes (from PDF section 7) ==
#define NICOLAY_FUNC_GET_SW_VERSION 0x01
#define NICOLAY_FUNC_GET_HW_VERSION 0x02
#define NICOLAY_FUNC_TEST_COMMAND 0x05
#define NICOLAY_FUNC_GET_PRESSURE_SENSOR_VALUE 0x06
#define NICOLAY_FUNC_GET_PRESSURE 0x07
#define NICOLAY_FUNC_GET_FLOW_AND_PRESSURE 0x09
#define NICOLAY_FUNC_GET_SENSOR_ARTICLE_NO 0x0A
#define NICOLAY_FUNC_BOARD_HARDWARE_RESET 0x0B
#define NICOLAY_FUNC_SENSOR_HARD_RESET 0x0C
#define NICOLAY_FUNC_SENSOR_SOFT_RESET 0x0D
#define NICOLAY_FUNC_START_FLOW_SENSOR 0x0E
#define NICOLAY_FUNC_GET_SENSOR_SERIAL_NO 0x0F
#define NICOLAY_FUNC_GET_FLOW_MEASUREMENT 0x10
#define NICOLAY_FUNC_GET_RAW_FLOW_MEASUREMENT 0x11
#define NICOLAY_FUNC_GET_FLOW_SENSOR_SCALE 0x12
#define NICOLAY_FUNC_GET_FLOW_SENSOR_OFFSET 0x13
#define NICOLAY_FUNC_GET_SET_HEATER_STATE 0x14
#define NICOLAY_FUNC_GET_SET_HEATER_POWER 0x15
#define NICOLAY_FUNC_GET_SCALE_TEMPERATURE 0x18
#define NICOLAY_FUNC_GET_OFFSET_TEMPERATURE 0x19
#define NICOLAY_FUNC_FORCE_TEMPERATURE_UPDATE 0x1B
#define NICOLAY_FUNC_FORCE_RAW_TEMPERATURE_UPDATE 0x1C
#define NICOLAY_FUNC_BULK_READ 0x1D
#define NICOLAY_FUNC_STREAM_SEND 0x1E
#define NICOLAY_FUNC_GET_SET_UART_BAUD 0x22

// == Error Codes for this library ==
typedef enum {
	NICOLAY_OK = 0,
	NICOLAY_ERR_SERIAL_OPEN_FAILED = -1,
	NICOLAY_ERR_SERIAL_WRITE_FAILED = -2,
	NICOLAY_ERR_SERIAL_READ_FAILED = -3,
	NICOLAY_ERR_TIMEOUT = -4,
	NICOLAY_ERR_INVALID_RESPONSE = -5, // e.g. wrong address, too short
	NICOLAY_ERR_CRC_MISMATCH = -6,
	NICOLAY_ERR_INVALID_RESPONSE_LEN = -7,
	NICOLAY_ERR_INVALID_PARAMETER = -8,
	NICOLAY_ERR_BAUDRATE_CHANGE_FAILED = -9,
	NICOLAY_ERR_EXCEPTION_BASE = -100 // Device returned an exception
	// Specific exception codes will be NICOLAY_ERR_EXCEPTION_BASE -
	// exception_code
} NicolayError;

// == Exception Error Codes from PDF (section 6.2.1) ==
#define NICOLAY_EXC_FUNC_UNKNOWN 1
#define NICOLAY_EXC_CANNOT_START_FIRMWARE 2
#define NICOLAY_EXC_DEVICE_INITIALIZING 3
#define NICOLAY_EXC_DEVICE_BUSY 4
#define NICOLAY_EXC_WRONG_NUM_DATA 5 // Follower error
#define NICOLAY_EXC_DATA_SIZE_ERROR 6 // Buffer overflow/underflow
#define NICOLAY_EXC_SUBCODE_ERROR 7
#define NICOLAY_EXC_DATA_VALUE_ERROR 8 // Plausibility
#define NICOLAY_EXC_NO_ACK_EEPROM 9
#define NICOLAY_EXC_TIMEOUT_EEPROM 10
#define NICOLAY_EXC_CHECKSUM_I2C_CMD_INVALID 11
#define NICOLAY_EXC_SENSOR_SHUTDOWN 15
#define NICOLAY_EXC_UPDATE_BOOTLOADER_NOT_STARTED 16
#define NICOLAY_EXC_HEXLINE_CHECKSUM_ERROR 17
#define NICOLAY_EXC_SYNTAX_ERROR_UPDATE 18

// == Structures ==
typedef struct {
	int serial_fd; // File descriptor for the serial port
	uint8_t slave_address; // Slave address of the sensor
	NicolayError last_error; // Last error encountered by a library function
	int current_baud_rate; // Current baud rate of the communication
} NicolaySensor;

typedef struct {
	uint8_t device_address;
	uint8_t function_code_raw; // Raw function code from response
	uint8_t num_data_bytes; // Number of data bytes in payload
	uint8_t data[NICOLAY_MAX_RESPONSE_DATA_LEN]; // Payload data
	uint16_t actual_data_len; // Actual length of data received in payload

	int is_exception; // Flag: 1 if response is an exception, 0 otherwise
	uint8_t exception_code; // Exception code if is_exception is 1
} NicolayResponse;

// Software Version Structure (for Get SW_Version)
typedef struct {
	char index_ascii; // Index as ASCII
	uint8_t version_minor; // Firmware version minor
	uint8_t version_major; // Firmware version major
} NicolaySWVersion;

// Hardware Version Structure (for Get HW_Version)
typedef struct {
	uint8_t version_minor; // Hardware version minor
	uint8_t version_major; // Hardware version major
} NicolayHWVersion;

// Pressure Sensor Value Structure (for Get Pressure Sensor Value)
// Note: This structure is complex, refer to PDF section 7.6 for details on
// interpretation
typedef struct {
	uint8_t type; // Pressure sensor type enum
	int16_t min_pressure_val; // Min pressure (e.g., -200 mbar)
	int16_t max_pressure_val; // Max pressure (e.g., +200 mbar)
	int16_t dig_out_p_min; // Digital output counts for min pressure
	int16_t dig_out_p_max; // Digital output counts for max pressure
	// Raw values from response (9 bytes total: Type, MinP_L, MinP_H, MaxP_L,
	// MaxP_H, dOutPMin_L, dOutPMin_H, dOutPMax_L, dOutPMax_H)
	uint8_t raw_bytes[9];
} NicolayPressureSensorInfo;

// == Public Function Prototypes ==

/**
 * @brief Initializes the NicolaySensor structure and opens the serial port.
 * @param sensor Pointer to the NicolaySensor structure.
 * @param port_name Name/path of the serial port (platform-specific).
 * @param slave_address The slave address of the Nicolay device.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_init(NicolaySensor *sensor, const char *port_name,
			  uint8_t slave_address);

/**
 * @brief Closes the serial port and deinitializes the sensor structure.
 * @param sensor Pointer to the NicolaySensor structure.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_close(NicolaySensor *sensor);

/**
 * @brief Calculates CRC-8 for the given data.
 *        This uses the specific CRC algorithm derived from the PDF's
 * pseudocode.
 * @param data Pointer to the data buffer.
 * @param len Length of the data in bytes.
 * @return Calculated CRC-8 value.
 */
uint8_t nicolay_calculate_crc8(const uint8_t *data, size_t len);

// --- Specific Command Functions ---

/**
 * @brief Get Software Version (Function Code 1).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param sw_version Pointer to NicolaySWVersion struct to store the result.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_sw_version(NicolaySensor *sensor,
				    NicolaySWVersion *sw_version);

/**
 * @brief Get Hardware Version (Function Code 2).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param hw_version Pointer to NicolayHWVersion struct to store the result.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_hw_version(NicolaySensor *sensor,
				    NicolayHWVersion *hw_version);

/**
 * @brief Test command (Function Code 5).
 *        Checks if the response matches the expected [0x55, 0xAA].
 * @param sensor Pointer to the NicolaySensor structure.
 * @param byte1 Pointer to store the first response data byte (should be 0x55).
 * @param byte2 Pointer to store the second response data byte (should be 0xAA).
 * @return NICOLAY_OK on success and if data matches, or an error code.
 */
NicolayError nicolay_test_command(NicolaySensor *sensor, uint8_t *byte1,
				  uint8_t *byte2);

/**
 * @brief Get Pressure (Function Code 7).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param pressure_counts Pointer to store the 16-bit pressure counts.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_pressure(NicolaySensor *sensor,
				  int16_t *pressure_counts);

/**
 * @brief Get Flow and Pressure (Function Code 9).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param flow_mSlm Pointer to store the 32-bit signed integer flow value (in
 * mSlm^2).
 * @param raw_pressure_counts Pointer to store the 16-bit signed integer raw
 * pressure counts.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_flow_and_pressure(NicolaySensor *sensor,
					   int32_t *flow_mSlm,
					   int16_t *raw_pressure_counts);

/**
 * @brief Get Sensor Article Number (Function Code 10).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param article_no Pointer to store the 32-bit article number.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_sensor_article_no(NicolaySensor *sensor,
					   uint32_t *article_no);

/**
 * @brief Board Hardware Reset (Function Code 11). No data in response.
 * @param sensor Pointer to the NicolaySensor structure.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_board_hardware_reset(NicolaySensor *sensor);

/**
 * @brief Sensor Hard Reset (Function Code 12). No data in response.
 * @param sensor Pointer to the NicolaySensor structure.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_sensor_hard_reset(NicolaySensor *sensor);

/**
 * @brief Sensor Soft Reset (Function Code 13). No data in response.
 * @param sensor Pointer to the NicolaySensor structure.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_sensor_soft_reset(NicolaySensor *sensor);

/**
 * @brief Get Sensor Serial Number (Function Code 15).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param serial_no Pointer to store the 32-bit serial number.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_sensor_serial_no(NicolaySensor *sensor,
					  uint32_t *serial_no);

/**
 * @brief Get Flow Measurement (Function Code 16).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param flow_mSlm Pointer to store the 32-bit signed integer flow value (in
 * mSlm^2).
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_flow_measurement(NicolaySensor *sensor,
					  int32_t *flow_mSlm);

/**
 * @brief Get Raw Flow Measurement (Function Code 17).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param raw_flow Pointer to store the 16-bit integer raw flow value.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_raw_flow_measurement(NicolaySensor *sensor,
					      int16_t *raw_flow);

/**
 * @brief Get Heater State (Function Code 20).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param heater_state Pointer to store the heater state (0=OFF, 1=ON).
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_heater_state(NicolaySensor *sensor,
				      uint8_t *heater_state);

/**
 * @brief Set Heater State (Function Code 20).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param heater_on 0 to turn OFF, 1 to turn ON.
 * @param response_state Pointer to store the returned state from the device.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_set_heater_state(NicolaySensor *sensor, uint8_t heater_on,
				      uint8_t *response_state);

/**
 * @brief Get Heater Power (Function Code 21).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param heater_power Pointer to store the heater power (0-100%).
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_get_heater_power(NicolaySensor *sensor,
				      uint8_t *heater_power);

/**
 * @brief Set Heater Power (Function Code 21).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param heater_power Power in percent (0-100). Values >100 are not allowed.
 * @param response_power Pointer to store the returned power from the device.
 * @return NICOLAY_OK on success, or an error code.
 */
NicolayError nicolay_set_heater_power(NicolaySensor *sensor,
				      uint8_t heater_power,
				      uint8_t *response_power);

/**
 * @brief Get/Set UART Baud Rate (Function Code 34).
 * @param sensor Pointer to the NicolaySensor structure.
 * @param baud_code The code for the new baud rate (0-15, see PDF 7.34).
 * @param response_baud_code Pointer to store the returned baud code from the
 * device.
 * @return NICOLAY_OK on success. NOTE: Baud rate changes AFTER this response.
 *         The caller must reconfigure the host serial port to the new baud
 * rate.
 */
NicolayError nicolay_set_uart_baud(NicolaySensor *sensor, uint8_t baud_code,
				   uint8_t *response_baud_code);

// Add prototypes for other functions (Get Pressure Sensor Value, Get Flow
// Sensor Scale/Offset, Temperature functions, BulkRead, etc.) following the
// same pattern. For brevity, they are not all listed here.

#endif // NICOLAY_FLOW_SENSOR_H
