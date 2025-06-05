#include "nicolay_flow_sensor.h"
#include <stdio.h>

int main()
{
	NicolaySensor sensor;
	NicolayError err;

	// Initialize the sensor communication
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
	} else {
		printf("Failed to get SW version: %d (Exception code: %d)\n",
		       err,
		       (err <= NICOLAY_ERR_EXCEPTION_BASE) ?
			       (NICOLAY_ERR_EXCEPTION_BASE - err) :
			       0);
	}

	while (1) {
		// Example: Get Flow
		int32_t flow;
		int16_t pressure;
		err = nicolay_get_flow_measurement(&sensor, &flow);
		if (err == NICOLAY_OK) {
			printf("%d mSlm^2\n", flow);
		} else {
			printf("Failed to get flow and pressure: %d\n", err);
		}
	}

	// Close the sensor communication
	nicolay_close(&sensor);
	printf("Sensor closed.\n");

	return 0;
}
