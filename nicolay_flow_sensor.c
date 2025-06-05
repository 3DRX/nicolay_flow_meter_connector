#include "nicolay_flow_sensor.h"
#include <string.h>

// Forward declaration for internal helper
static NicolayError nicolay_send_receive_command(NicolaySensor *sensor,
						 uint8_t func_code,
						 const uint8_t *cmd_data,
						 uint8_t cmd_data_len,
						 NicolayResponse *response_out);

// Helper to convert baud code to actual baud rate value
static int nicolay_baud_code_to_value(uint8_t baud_code)
{
	switch (baud_code) {
	case 0:
		return 4800;
	case 1:
		return 9600;
	case 2:
		return 14400;
	case 3:
		return 19200;
	case 4:
		return 28800;
	case 5:
		return 31250;
	case 6:
		return 38400;
	case 7:
		return 57600;
	case 8:
		return 115200; // Factory Default
	case 9:
		return 128000;
	case 10:
		return 230400;
	case 11:
		return 250000;
	case 12:
		return 256000;
	case 13:
		return 384000;
	case 14:
		return 500000;
	case 15:
		return 576000;
	default:
		return -1; // Invalid code
	}
}

uint8_t nicolay_calculate_crc8(const uint8_t *data, size_t len)
{
	const uint8_t polynomial = 0x31;
	uint16_t crc =
		0x0000; // Use uint16_t for intermediate calculation as per
	// PDF's literal pseudocode

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i]; // crc is 0..255 after this
		for (int bit = 0; bit < 8; bit++) {
			uint16_t temp_crc_shifted = crc << 1;
			if (temp_crc_shifted &
			    0x0100) { // Check 9th bit (mask 0x100)
				crc = (temp_crc_shifted ^ polynomial);
			} else {
				crc = temp_crc_shifted;
			}
			crc &= 0x00FF; // Truncate to 8 bits for next iteration / next byte
		}
	}
	return (uint8_t)crc;
}

NicolayError nicolay_init(NicolaySensor *sensor, const char *port_name,
			  uint8_t slave_address)
{
	if (!sensor)
		return NICOLAY_ERR_INVALID_PARAMETER;

	sensor->slave_address = slave_address;
	sensor->last_error = NICOLAY_OK;
	sensor->current_baud_rate = 115200; // Default baud rate

	sensor->serial_fd =
		platform_serial_open(port_name, sensor->current_baud_rate);
	if (sensor->serial_fd < 0) { // Assuming negative fd is error
		sensor->last_error = NICOLAY_ERR_SERIAL_OPEN_FAILED;
		return sensor->last_error;
	}
	return NICOLAY_OK;
}

NicolayError nicolay_close(NicolaySensor *sensor)
{
	if (!sensor)
		return NICOLAY_ERR_INVALID_PARAMETER;
	if (sensor->serial_fd >= 0) {
		if (platform_serial_close(sensor->serial_fd) != 0) {
			sensor->last_error =
				NICOLAY_ERR_SERIAL_OPEN_FAILED; // Or a specific close error
			// return sensor->last_error; // Continue to invalidate fd
		}
		sensor->serial_fd = -1;
	}
	return NICOLAY_OK;
}

static NicolayError nicolay_send_receive_command(NicolaySensor *sensor,
						 uint8_t func_code,
						 const uint8_t *cmd_data,
						 uint8_t cmd_data_len,
						 NicolayResponse *response_out)
{
	if (!sensor || sensor->serial_fd < 0) {
		if (sensor)
			sensor->last_error = NICOLAY_ERR_SERIAL_OPEN_FAILED;
		return NICOLAY_ERR_SERIAL_OPEN_FAILED;
	}
	if (cmd_data_len > NICOLAY_MAX_CMD_DATA_LEN) {
		sensor->last_error = NICOLAY_ERR_INVALID_PARAMETER;
		return sensor->last_error;
	}

	uint8_t tx_buffer[NICOLAY_FRAME_HEADER_SIZE + NICOLAY_MAX_CMD_DATA_LEN +
			  NICOLAY_CRC_SIZE];
	size_t tx_len = 0;

	// Build command frame
	tx_buffer[tx_len++] = sensor->slave_address;
	tx_buffer[tx_len++] = func_code;
	tx_buffer[tx_len++] = cmd_data_len;
	if (cmd_data_len > 0 && cmd_data) {
		memcpy(&tx_buffer[tx_len], cmd_data, cmd_data_len);
		tx_len += cmd_data_len;
	}
	tx_buffer[tx_len] = nicolay_calculate_crc8(tx_buffer, tx_len);
	tx_len++;

	// Send command
	int bytes_written =
		platform_serial_write(sensor->serial_fd, tx_buffer, tx_len);
	if (bytes_written < 0 || (size_t)bytes_written != tx_len) {
		sensor->last_error = NICOLAY_ERR_SERIAL_WRITE_FAILED;
		return sensor->last_error;
	}

	// Receive response
	uint8_t rx_buffer[NICOLAY_MAX_FRAME_LEN];
	memset(rx_buffer, 0, NICOLAY_MAX_FRAME_LEN);
	memset(response_out, 0, sizeof(NicolayResponse));

	// Read header (Device Address, Function Code, Number of Data)
	int bytes_read = platform_serial_read(sensor->serial_fd, rx_buffer,
					      NICOLAY_FRAME_HEADER_SIZE,
					      NICOLAY_DEFAULT_TIMEOUT_MS);

	if (bytes_read < 0) {
		sensor->last_error = NICOLAY_ERR_SERIAL_READ_FAILED;
		return sensor->last_error;
	}
	if (bytes_read == 0) {
		sensor->last_error = NICOLAY_ERR_TIMEOUT;
		return sensor->last_error;
	}
	if (bytes_read < NICOLAY_FRAME_HEADER_SIZE) {
		sensor->last_error =
			NICOLAY_ERR_INVALID_RESPONSE; // Incomplete header
		return sensor->last_error;
	}

	response_out->device_address = rx_buffer[0];
	response_out->function_code_raw = rx_buffer[1];
	response_out->num_data_bytes =
		rx_buffer[2]; // This is the "Follower" count

	if (response_out->device_address != sensor->slave_address &&
	    sensor->slave_address != NICOLAY_BROADCAST_ADDRESS) {
		// Exception: if master address is 255 (identify), any slave address is
		// fine. For now, strict check.
		sensor->last_error =
			NICOLAY_ERR_INVALID_RESPONSE; // Address mismatch
		return sensor->last_error;
	}

	response_out->is_exception =
		(response_out->function_code_raw & 0x80) ? 1 : 0;

	uint16_t expected_payload_len = response_out->num_data_bytes;

	// Special handling for BulkRead (Cmd 0x1D) response length as per PDF note
	if (func_code == NICOLAY_FUNC_BULK_READ &&
	    response_out->num_data_bytes == 0xFF) {
		// "If Pressure sensor exist Follower = 255 (0xFF) ... Data are 400Bytes"
		expected_payload_len = 400;
	} else if (func_code == NICOLAY_FUNC_BULK_READ &&
		   response_out->num_data_bytes == 0xC8) {
		expected_payload_len = 200;
	}

	if (expected_payload_len > NICOLAY_MAX_RESPONSE_DATA_LEN) {
		sensor->last_error =
			NICOLAY_ERR_INVALID_RESPONSE_LEN; // Declared payload too large
		return sensor->last_error;
	}

	// Read data payload and CRC
	size_t bytes_to_read_more = expected_payload_len + NICOLAY_CRC_SIZE;
	if (bytes_to_read_more > 0) {
		bytes_read = platform_serial_read(
			sensor->serial_fd,
			&rx_buffer[NICOLAY_FRAME_HEADER_SIZE],
			bytes_to_read_more, NICOLAY_DEFAULT_TIMEOUT_MS);
		if (bytes_read < 0) {
			sensor->last_error = NICOLAY_ERR_SERIAL_READ_FAILED;
			return sensor->last_error;
		}
		if (bytes_read == 0 && bytes_to_read_more > 0) {
			sensor->last_error = NICOLAY_ERR_TIMEOUT;
			return sensor->last_error;
		}
		if ((size_t)bytes_read < bytes_to_read_more) {
			sensor->last_error =
				NICOLAY_ERR_INVALID_RESPONSE; // Incomplete payload/CRC
			return sensor->last_error;
		}
	}

	response_out->actual_data_len = expected_payload_len;
	memcpy(response_out->data, &rx_buffer[NICOLAY_FRAME_HEADER_SIZE],
	       expected_payload_len);

	// Verify CRC
	uint8_t received_crc =
		rx_buffer[NICOLAY_FRAME_HEADER_SIZE + expected_payload_len];
	uint8_t calculated_crc = nicolay_calculate_crc8(
		rx_buffer, NICOLAY_FRAME_HEADER_SIZE + expected_payload_len);

	if (received_crc != calculated_crc) {
		sensor->last_error = NICOLAY_ERR_CRC_MISMATCH;
		return sensor->last_error;
	}

	// Handle exception response
	if (response_out->is_exception) {
		if (response_out->num_data_bytes == 1 &&
		    expected_payload_len ==
			    1) { // num_data_bytes should be 0x01 for exception
			response_out->exception_code = response_out->data[0];
			sensor->last_error =
				(NicolayError)(NICOLAY_ERR_EXCEPTION_BASE -
					       response_out->exception_code);
			return sensor->last_error;
		} else {
			sensor->last_error =
				NICOLAY_ERR_INVALID_RESPONSE; // Malformed exception
			return sensor->last_error;
		}
	}

	// Check if returned function code (without exception bit) matches sent one
	if ((response_out->function_code_raw & 0x7F) != func_code) {
		sensor->last_error =
			NICOLAY_ERR_INVALID_RESPONSE; // Function code mismatch
		return sensor->last_error;
	}

	sensor->last_error = NICOLAY_OK;
	return NICOLAY_OK;
}

NicolayError nicolay_get_sw_version(NicolaySensor *sensor,
				    NicolaySWVersion *sw_version)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_SW_VERSION, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 3) {
		sw_version->index_ascii = (char)response.data[0];
		sw_version->version_minor = response.data[1];
		sw_version->version_major = response.data[2];
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_get_hw_version(NicolaySensor *sensor,
				    NicolayHWVersion *hw_version)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_HW_VERSION, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 2) {
		// PDF: "MajorHw. MinorHW" but data is Low/High. Assuming Minor is first
		// byte.
		hw_version->version_minor = response.data[0];
		hw_version->version_major = response.data[1];
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_test_command(NicolaySensor *sensor, uint8_t *byte1,
				  uint8_t *byte2)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_TEST_COMMAND, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 2) {
		if (byte1)
			*byte1 = response.data[0];
		if (byte2)
			*byte2 = response.data[1];
		// As per PDF example: response data is 0x55, 0xAA
		if (response.data[0] == 0x55 && response.data[1] == 0xAA) {
			return NICOLAY_OK;
		} else {
			sensor->last_error =
				NICOLAY_ERR_INVALID_RESPONSE; // Content mismatch
			return sensor->last_error;
		}
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_get_pressure(NicolaySensor *sensor,
				  int16_t *pressure_counts)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_PRESSURE, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len ==
	    2) { // PDF says 0x04 (4 bytes) for data count, but then "16-Bit
		// (Integer)_pressure counts"
		// The example response shows "Press low Press high CRC8", so 2 bytes of
		// data. Assuming "Number of following dates" in PDF image on page 5 is a
		// typo (0x04) and data is 2 bytes. If it is indeed 4 bytes, this needs
		// adjustment. Update: The table shows "DevAddr 0x07 0x04 Press low Press
		// high CRC 8" This means NumData = 0x04. But "Press low Press high" is 2
		// bytes. This is confusing. The example frame for Get Pressure (7) on page
		// 5 shows: Response: DevAddr | 0x07 | 0x04 | Press low | Press high | CRC 8
		// This table is inconsistent with "Number of data: determines the number of
		// data without CRC and headers" If NumData is 0x04, then 4 bytes of data
		// are expected. If "Press low Press high" are the only data, then NumData
		// should be 0x02. Going by "16-Bit (Integer)_pressure counts", assuming 2
		// bytes of data. If this assumption is wrong, and NumData is truly 4, this
		// will fail CRC or parsing. The generic send/receive will use NumData from
		// response header. So if device sends NumData=2, actual_data_len=2. If
		// NumData=4, actual_data_len=4.
		if (response.num_data_bytes ==
		    2) { // Trusting the "16-bit" description
			// over the example's "0x04" field
			*pressure_counts =
				(int16_t)((uint16_t)response.data[0] |
					  ((uint16_t)response.data[1] << 8));
			return NICOLAY_OK;
		} else if (response.num_data_bytes == 4) {
			// If NumData is indeed 4, what are the other 2 bytes? PDF is unclear.
			// For now, let's assume the first 2 bytes are the pressure if NumData
			// is 4.
			*pressure_counts =
				(int16_t)((uint16_t)response.data[0] |
					  ((uint16_t)response.data[1] << 8));
			// Potentially log a warning about unexpected data length if strictness is
			// needed.
			return NICOLAY_OK;
		}
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_get_flow_and_pressure(NicolaySensor *sensor,
					   int32_t *flow_mSlm,
					   int16_t *raw_pressure_counts)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_FLOW_AND_PRESSURE, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len ==
	    6) { // Flow (4 bytes) + Pressure (2 bytes)
		*flow_mSlm = (int32_t)((uint32_t)response.data[0] |
				       ((uint32_t)response.data[1] << 8) |
				       ((uint32_t)response.data[2] << 16) |
				       ((uint32_t)response.data[3] << 24));
		*raw_pressure_counts =
			(int16_t)((uint16_t)response.data[4] |
				  ((uint16_t)response.data[5] << 8));
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_get_sensor_article_no(NicolaySensor *sensor,
					   uint32_t *article_no)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_SENSOR_ARTICLE_NO, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 4) {
		*article_no = (uint32_t)((uint32_t)response.data[0] |
					 ((uint32_t)response.data[1] << 8) |
					 ((uint32_t)response.data[2] << 16) |
					 ((uint32_t)response.data[3] << 24));
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_board_hardware_reset(NicolaySensor *sensor)
{
	NicolayResponse response;
	// Expects 0 data bytes in response.
	return nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_BOARD_HARDWARE_RESET, NULL, 0, &response);
}

NicolayError nicolay_sensor_hard_reset(NicolaySensor *sensor)
{
	NicolayResponse response;
	return nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_SENSOR_HARD_RESET, NULL, 0, &response);
}

NicolayError nicolay_sensor_soft_reset(NicolaySensor *sensor)
{
	NicolayResponse response;
	return nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_SENSOR_SOFT_RESET, NULL, 0, &response);
}

NicolayError nicolay_get_sensor_serial_no(NicolaySensor *sensor,
					  uint32_t *serial_no)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_SENSOR_SERIAL_NO, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 4) {
		*serial_no = (uint32_t)((uint32_t)response.data[0] |
					((uint32_t)response.data[1] << 8) |
					((uint32_t)response.data[2] << 16) |
					((uint32_t)response.data[3] << 24));
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_get_flow_measurement(NicolaySensor *sensor,
					  int32_t *flow_mSlm)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_FLOW_MEASUREMENT, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 4) {
		*flow_mSlm = (int32_t)((uint32_t)response.data[0] |
				       ((uint32_t)response.data[1] << 8) |
				       ((uint32_t)response.data[2] << 16) |
				       ((uint32_t)response.data[3] << 24));
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_get_raw_flow_measurement(NicolaySensor *sensor,
					      int16_t *raw_flow)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_RAW_FLOW_MEASUREMENT, NULL, 0,
		&response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 2) {
		*raw_flow = (int16_t)((uint16_t)response.data[0] |
				      ((uint16_t)response.data[1] << 8));
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_get_heater_state(NicolaySensor *sensor,
				      uint8_t *heater_state)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_SET_HEATER_STATE, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 1) {
		*heater_state = response.data[0] & 0x01; // Bit 0 is the state
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_set_heater_state(NicolaySensor *sensor, uint8_t heater_on,
				      uint8_t *response_state)
{
	uint8_t cmd_data = (heater_on ? 1 : 0);
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_SET_HEATER_STATE, &cmd_data, 1,
		&response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 1) {
		if (response_state)
			*response_state = response.data[0] & 0x01;
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_get_heater_power(NicolaySensor *sensor,
				      uint8_t *heater_power)
{
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_SET_HEATER_POWER, NULL, 0, &response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 1) {
		*heater_power = response.data[0];
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_set_heater_power(NicolaySensor *sensor,
				      uint8_t heater_power,
				      uint8_t *response_power)
{
	if (heater_power > 100) { // As per PDF note
		sensor->last_error = NICOLAY_ERR_INVALID_PARAMETER;
		return sensor->last_error;
	}
	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_SET_HEATER_POWER, &heater_power, 1,
		&response);
	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 1) {
		if (response_power)
			*response_power = response.data[0];
		return NICOLAY_OK;
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

NicolayError nicolay_set_uart_baud(NicolaySensor *sensor, uint8_t baud_code,
				   uint8_t *response_baud_code)
{
	if (baud_code > 15) { // Max baud code from PDF
		sensor->last_error = NICOLAY_ERR_INVALID_PARAMETER;
		return sensor->last_error;
	}

	NicolayResponse response;
	NicolayError status = nicolay_send_receive_command(
		sensor, NICOLAY_FUNC_GET_SET_UART_BAUD, &baud_code, 1,
		&response);

	// IMPORTANT: The response comes at the *OLD* baud rate.
	// The baud rate changes *AFTER* this response is sent by the device.

	if (status != NICOLAY_OK)
		return status;

	if (response.actual_data_len == 1) {
		if (response_baud_code)
			*response_baud_code = response.data[0];

		// If the command was apparently successful and the returned code matches
		// sent code:
		if (response.data[0] == baud_code) {
			int new_baud_value =
				nicolay_baud_code_to_value(baud_code);
			if (new_baud_value != -1) {
				// User's platform function to change baud rate of the host serial port
				if (platform_serial_set_baudrate(
					    sensor->serial_fd,
					    new_baud_value) == 0) {
					sensor->current_baud_rate =
						new_baud_value;
					return NICOLAY_OK;
				} else {
					sensor->last_error =
						NICOLAY_ERR_BAUDRATE_CHANGE_FAILED;
					return sensor->last_error;
				}
			} else {
				sensor->last_error =
					NICOLAY_ERR_INVALID_PARAMETER; // Should not happen if initial check
				// passed
				return sensor->last_error;
			}
		} else {
			// Device returned a different baud code than what we tried to set.
			// This could be an error or unexpected behavior.
			sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE;
			return sensor->last_error;
		}
	}
	sensor->last_error = NICOLAY_ERR_INVALID_RESPONSE_LEN;
	return sensor->last_error;
}

// Implementations for other functions (Get Pressure Sensor Value, Get Flow
// Sensor Scale/Offset, Temperature functions, BulkRead, STREAM_send etc.) would
// follow a similar pattern:
// 1. Define any specific request data.
// 2. Call nicolay_send_receive_command().
// 3. Check status and response.actual_data_len.
// 4. Parse response.data into the provided output structures/variables,
//    remembering Low-byte first for multi-byte values.
//
// Example for a function that takes arguments:
// NicolayError nicolay_some_set_function(NicolaySensor* sensor, uint16_t
// param1, uint8_t param2) {
//     uint8_t cmd_data[3];
//     cmd_data[0] = param1 & 0xFF;        // Low byte of param1
//     cmd_data[1] = (param1 >> 8) & 0xFF; // High byte of param1
//     cmd_data[2] = param2;
//     NicolayResponse response;
//     return nicolay_send_receive_command(sensor, YOUR_FUNC_CODE, cmd_data, 3,
//     &response);
//     // Check response data if any expected
// }
