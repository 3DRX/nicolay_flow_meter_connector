example-posix: example.c nicolay_flow_sensor.c platform_serial_posix.c
	gcc example.c nicolay_flow_sensor.c platform_serial_posix.c -o example-posix
