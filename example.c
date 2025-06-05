#include "nicolay_flow_sensor.h"
#include <stdio.h>

int main()
{
	NicolaySensor sensor;
	NicolayError err;

	// Initialize the sensor communication
	// Replace "SERIAL_PORT_NAME" if applicable, or pass NULL if not used by your platform_serial_open.
	// Use NICOLAY_DEFAULT_SLAVE_ADDRESS or the specific address of your device.
	err = nicolay_init(&sensor, "/dev/ttyUSB0",
			   NICOLAY_DEFAULT_SLAVE_ADDRESS);
	if (err != NICOLAY_OK) {
		printf("Failed to initialize sensor: %d\n", err);
		return -1;
	}

	printf("Sensor initialized. Current baud rate: %d\n",
	       sensor.current_baud_rate);

	// Example: Get Software Version
	NicolaySWVersion sw_version;
	err = nicolay_get_sw_version(&sensor, &sw_version);
	if (err == NICOLAY_OK) {
		printf("SW Version: Index '%c', Major %u, Minor %u (Raw: %u.%u)\n",
		       sw_version.index_ascii, sw_version.version_major,
		       sw_version.version_minor, sw_version.version_major,
		       sw_version.version_minor);
		// Example from PDF: 61,5A,0 -> Firmware version 0.90a
		// 'a' (ASCII 97) is 0x61. 'Z' (ASCII 90) is 0x5A.
		// So if index_ascii = 'a', version_major = 0, version_minor = 90 (0x5A)
		// it would print: Index 'a', Major 0, Minor 90
	} else {
		printf("Failed to get SW version: %d (Exception code: %d)\n",
		       err,
		       (err <= NICOLAY_ERR_EXCEPTION_BASE) ?
			       (NICOLAY_ERR_EXCEPTION_BASE - err) :
			       0);
	}

	while (1) {
		// Example: Get Flow and Pressure
		int32_t flow;
		int16_t pressure;
		err = nicolay_get_flow_and_pressure(&sensor, &flow, &pressure);
		if (err == NICOLAY_OK) {
			// printf("Flow: %d mSlm^2, Raw Pressure: %d counts\n",
			//        flow, pressure);
			printf("%d\n", flow);
		} else {
			printf("Failed to get flow and pressure: %d\n", err);
		}
	}

	// Example: Changing Baud Rate (Use with caution!)
	/*
    uint8_t current_baud_code = 8; // Assuming current is 115200
    uint8_t new_baud_code = 7; // Target: 57600
    uint8_t response_code;
    printf("Attempting to change baud rate from 115200 (code 8) to 57600 (code 7)...\n");
    err = nicolay_set_uart_baud(&sensor, new_baud_code, &response_code);
    if (err == NICOLAY_OK) {
        printf("Baud rate change command acknowledged. Device returned code: %u.\n", response_code);
        printf("Host serial port reconfigured to %d baud.\n", sensor.current_baud_rate);
        
        // Communication should now proceed at the new baud rate.
        // Test with another command:
        err = nicolay_get_sw_version(&sensor, &sw_version);
        if (err == NICOLAY_OK) {
             printf("SW Version at new baud rate: Index '%c', Major %u, Minor %u\n",
               sw_version.index_ascii, sw_version.version_major, sw_version.version_minor);
        } else {
            printf("Failed to get SW version at new baud rate: %d\n", err);
        }

    } else {
        printf("Failed to set baud rate: %d\n", err);
    }
    */

	// Close the sensor communication
	nicolay_close(&sensor);
	printf("Sensor closed.\n");

	return 0;
}
