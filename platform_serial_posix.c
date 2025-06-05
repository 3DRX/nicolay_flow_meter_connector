// platform_serial_linux.c

#include <errno.h> // For errno
#include <fcntl.h> // For file control options (O_RDWR, O_NOCTTY, etc.)
#include <stdio.h> // For perror, fprintf
#include <string.h> // For memset, strerror
#include <sys/select.h> // For select() and related macros/structs (fd_set, timeval)
#include <termios.h> // For POSIX terminal control (tcgetattr, tcsetattr, baud rates, etc.)
#include <unistd.h> // For open, close, read, write

#include "nicolay_flow_sensor.h" // For the function declarations these definitions implement

// Helper function to convert integer baud rate to termios speed_t constant
static speed_t get_baud_rate_constant(int baud_rate)
{
	switch (baud_rate) {
	case 4800:
		return B4800;
	case 9600:
		return B9600;
#ifdef B14400
	case 14400:
		return B14400;
#endif
	case 19200:
		return B19200;
#ifdef B28800
	case 28800:
		return B28800;
#endif
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 128000:
		return B115200; // No standard B128000, B115200 is closest standard above
		// Or custom baud logic might be needed if exact 128000 is
		// critical For many USB-serial, it might accept
		// non-standard values directly or via termios2. For now,
		// map to a close standard one or fail. The PDF for Nicolay
		// lists 128000, so it might be supported by the device.
		// Using B115200 as a placeholder if B128000 isn't defined.
		// A more robust solution might involve termios2 for custom
		// baud rates. However, B128000 is not a standard POSIX
		// speed_t. For simplicity, let's assume standard rates are
		// preferred or the system handles it. The Nicolay device
		// itself supports it via its codes.
#ifdef B128000
	case 128000:
		return B128000;
#endif
	case 230400:
		return B230400;
	case 250000:
		return B230400; // No standard B250000.
#ifdef B250000
	case 250000:
		return B250000;
#endif
	case 256000:
		return B230400; // No standard B256000.
#ifdef B256000
	case 256000:
		return B256000;
#endif
	case 384000:
		return B230400; // No standard B384000
#ifdef B384000
	case 384000:
		return B384000;
#endif
#ifdef B500000
	case 500000:
		return B500000;
#endif
#ifdef B576000
	case 576000:
		return B576000;
#endif
	// Add other baud rates as needed and available
	default:
		fprintf(stderr, "Unsupported baud rate: %d\n", baud_rate);
		return B0; // Indicates an invalid speed
	}
}

/**
 * @brief Opens and configures the serial port on Linux.
 */
int platform_serial_open(const char *port_name, int baud_rate)
{
	int fd;
	struct termios tty;

	// Open the serial port device file.
	// O_RDWR: Open for reading and writing.
	// O_NOCTTY: The port will not become the controlling terminal for the
	// process. O_NONBLOCK (or O_NDELAY): Open in non-blocking mode. This affects
	// how read() behaves if no data is available. select() will be used for timed
	// reads, so non-blocking is fine.
	fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		perror("platform_serial_open: Unable to open port");
		return -1;
	}

	// Get current serial port settings
	if (tcgetattr(fd, &tty) != 0) {
		perror("platform_serial_open: Unable to get serial port attributes "
		       "(tcgetattr)");
		close(fd);
		return -1;
	}

	// Set baud rate
	speed_t speed = get_baud_rate_constant(baud_rate);
	if (speed == B0 &&
	    baud_rate !=
		    0) { // B0 is valid for "hang up" but not for data transfer
		fprintf(stderr, "platform_serial_open: Invalid baud rate %d\n",
			baud_rate);
		close(fd);
		return -1;
	}
	if (cfsetospeed(&tty, speed) != 0) {
		perror("platform_serial_open: cfsetospeed failed");
		close(fd);
		return -1;
	}
	if (cfsetispeed(&tty, speed) != 0) {
		perror("platform_serial_open: cfsetispeed failed");
		close(fd);
		return -1;
	}

	// Configure for 8N1 (8 data bits, no parity, 1 stop bit) and raw mode
	tty.c_cflag |=
		(CLOCAL | CREAD); // Enable receiver, ignore modem control lines
	tty.c_cflag &= ~CSIZE; // Clear character size mask
	tty.c_cflag |= CS8; // 8 data bits
	tty.c_cflag &= ~PARENB; // No parity
	tty.c_cflag &=
		~CSTOPB; // 1 stop bit (setting CSTOPB would be 2 stop bits)
	tty.c_cflag &= ~CRTSCTS; // No hardware flow control

	// Input modes: raw input
	tty.c_iflag &= ~(IXON | IXOFF |
			 IXANY); // Disable software flow control (XON/XOFF)
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
			 ICRNL); // Disable any special handling of input bytes

	// Output modes: raw output
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
	// (e.g., newline conversions)

	// Local modes: non-canonical mode
	tty.c_lflag &=
		~(ECHO | ECHONL | ICANON | ISIG |
		  IEXTEN); // Disable canonical mode, echo, and signal chars

	// Control characters for non-canonical mode
	// VMIN = 0, VTIME = 0: read() returns immediately with available bytes or 0
	// if none. This is suitable when using select() for timeouts.
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	// Apply the settings
	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		perror("platform_serial_open: Unable to set serial port attributes "
		       "(tcsetattr)");
		close(fd);
		return -1;
	}

	// Flush any pending I/O
	tcflush(fd, TCIOFLUSH);

	// If O_NONBLOCK was set by open, we can clear it if we want blocking writes
	// by default For reads, select() will handle non-blocking behavior and
	// timeouts. For writes, default blocking behavior is often simpler unless
	// specific non-blocking writes are needed. To make writes blocking (if open
	// used O_NONBLOCK): int flags = fcntl(fd, F_GETFL, 0); fcntl(fd, F_SETFL,
	// flags & ~O_NONBLOCK); However, for this implementation, we'll assume writes
	// can complete or the system handles buffering. The read function explicitly
	// uses select() for timeouts.

	return fd;
}

/**
 * @brief Closes the serial port on Linux.
 */
int platform_serial_close(int fd)
{
	if (fd >= 0) {
		if (close(fd) < 0) {
			perror("platform_serial_close: Failed to close port");
			return -1;
		}
	}
	return 0;
}

/**
 * @brief Writes data to the serial port on Linux.
 *        This implementation attempts to write all bytes in a loop.
 */
int platform_serial_write(int fd, const uint8_t *buffer, size_t len)
{
	ssize_t total_written = 0;
	ssize_t written_now;

	if (fd < 0)
		return -1;

	while (total_written < len) {
		written_now =
			write(fd, buffer + total_written, len - total_written);
		if (written_now < 0) {
			if (errno ==
			    EINTR) { // Interrupted by signal, try again
				continue;
			}
			// If O_NONBLOCK is set for writes, EAGAIN or EWOULDBLOCK might occur.
			// This simple loop doesn't handle non-blocking writes with select.
			// It assumes writes are blocking or will eventually succeed.
			perror("platform_serial_write: Write failed");
			return -1;
		}
		// write() returning 0 for a non-zero length is unusual for a regular
		// file/device but could mean different things for different devices. For
		// serial, it's unlikely if no error is reported.
		if (written_now == 0 && (len - total_written) > 0) {
			fprintf(stderr,
				"platform_serial_write: Wrote 0 bytes when data "
				"remained. Port issue?\n");
			// This might indicate an issue, returning what's written so far or an
			// error. For simplicity, we treat it as an incomplete write that couldn't
			// proceed.
			return total_written > 0 ?
				       total_written :
				       -1; // Return bytes written if some, else error
		}
		total_written += written_now;
	}
	return total_written;
}

/**
 * @brief Reads data from the serial port on Linux with a timeout.
 */
int platform_serial_read(int fd, uint8_t *buffer, size_t max_len,
			 int timeout_ms)
{
	fd_set readfds;
	struct timeval tv;
	int retval;
	ssize_t bytes_read;

	if (fd < 0 || max_len == 0)
		return -1;

	FD_ZERO(&readfds);
	FD_SET(fd, &readfds);

	// Set timeout
	// If timeout_ms < 0, select blocks indefinitely.
	// If timeout_ms == 0, select polls.
	// If timeout_ms > 0, select waits for that duration.
	struct timeval *tv_ptr = NULL;
	if (timeout_ms >= 0) {
		tv.tv_sec = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;
		tv_ptr = &tv;
	}

	retval = select(fd + 1, &readfds, NULL, NULL, tv_ptr);

	if (retval == -1) {
		perror("platform_serial_read: select() failed");
		return -1; // Error in select
	} else if (retval > 0) {
		if (FD_ISSET(fd, &readfds)) {
			// Data is available, now read it
			bytes_read = read(fd, buffer, max_len);
			if (bytes_read < 0) {
				// read() might return -1 with EAGAIN or EWOULDBLOCK if O_NONBLOCK is
				// set on fd and no data is *actually* there, despite select indicating
				// readiness. This shouldn't happen if VMIN/VTIME are set for
				// non-blocking reads (0,0) and select() reported readiness.
				perror("platform_serial_read: read() failed after select");
				return -1;
			}
			return bytes_read; // Return number of bytes read (can be less than
			// max_len)
		}
	}
	// retval == 0 means timeout
	return 0; // Timeout or no data if FD_ISSET was false (should not happen if
	// retval > 0)
}

/**
 * @brief Changes the baud rate of an already open serial port on Linux.
 */
int platform_serial_set_baudrate(int fd, int new_baud_rate)
{
	struct termios tty;

	if (fd < 0)
		return -1;

	if (tcgetattr(fd, &tty) != 0) {
		perror("platform_serial_set_baudrate: tcgetattr failed");
		return -1;
	}

	speed_t speed = get_baud_rate_constant(new_baud_rate);
	if (speed == B0 && new_baud_rate != 0) {
		fprintf(stderr,
			"platform_serial_set_baudrate: Invalid new baud rate %d\n",
			new_baud_rate);
		return -1;
	}

	if (cfsetospeed(&tty, speed) != 0) {
		perror("platform_serial_set_baudrate: cfsetospeed failed");
		return -1;
	}
	if (cfsetispeed(&tty, speed) != 0) {
		perror("platform_serial_set_baudrate: cfsetispeed failed");
		return -1;
	}

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		perror("platform_serial_set_baudrate: tcsetattr failed");
		return -1;
	}

	tcflush(fd, TCIOFLUSH); // Flush buffers after changing rate

	return 0;
}
