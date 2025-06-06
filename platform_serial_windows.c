#include "nicolay_flow_sensor.h"
#include <windows.h>
#include <stdio.h>

int platform_serial_open(const char *port_name, int baud_rate)
{
	HANDLE hSerial = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0,
				     NULL, OPEN_EXISTING, 0, NULL);

	if (hSerial == INVALID_HANDLE_VALUE) {
		return -1;
	}

	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

	if (!GetCommState(hSerial, &dcbSerialParams)) {
		CloseHandle(hSerial);
		return -1;
	}

	dcbSerialParams.BaudRate = baud_rate;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;

	if (!SetCommState(hSerial, &dcbSerialParams)) {
		CloseHandle(hSerial);
		return -1;
	}

	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	if (!SetCommTimeouts(hSerial, &timeouts)) {
		CloseHandle(hSerial);
		return -1;
	}

	return (int)hSerial;
}

int platform_serial_close(int fd)
{
	HANDLE hSerial = (HANDLE)fd;
	if (!CloseHandle(hSerial)) {
		return -1;
	}
	return 0;
}

int platform_serial_write(int fd, const uint8_t *buffer, size_t len)
{
	HANDLE hSerial = (HANDLE)fd;
	DWORD bytesWritten;

	if (!WriteFile(hSerial, buffer, len, &bytesWritten, NULL)) {
		return -1;
	}

	return (int)bytesWritten;
}

int platform_serial_read(int fd, uint8_t *buffer, size_t max_len,
			 int timeout_ms)
{
	HANDLE hSerial = (HANDLE)fd;
	DWORD bytesRead;

	COMMTIMEOUTS timeouts;
	if (!GetCommTimeouts(hSerial, &timeouts)) {
		return -1;
	}

	timeouts.ReadTotalTimeoutConstant = timeout_ms;
	if (!SetCommTimeouts(hSerial, &timeouts)) {
		return -1;
	}

	if (!ReadFile(hSerial, buffer, max_len, &bytesRead, NULL)) {
		return -1;
	}

	return (int)bytesRead;
}

int platform_serial_set_baudrate(int fd, int new_baud_rate)
{
	HANDLE hSerial = (HANDLE)fd;
	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

	if (!GetCommState(hSerial, &dcbSerialParams)) {
		return -1;
	}

	dcbSerialParams.BaudRate = new_baud_rate;

	if (!SetCommState(hSerial, &dcbSerialParams)) {
		return -1;
	}

	return 0;
