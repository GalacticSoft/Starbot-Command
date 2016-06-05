#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include "ev314.h" // EV314 Firmware

/* EV314 Constants */

#define EV314_EXPECTED_SERIAL		  "0016534957ad"
#define EV314_TEST_NB_ITER			  27000000
#define EV314_USB_TIMEOUT             1000    // Milliseconds
#define EV314_INTERFACE_NUMBER        0
#define EV314_CONFIGURATION_NB        1
#define EV314_EP_OUT                  0x01
#define EV314_EP_IN                   0x81
#define EV314_PACKET_SIZE			  0x400
#define EV314_RESET_RESPONSE_SIZE     5
#define EV314_POWER_RESPONSE_SIZE     5
#define EV314_VENDOR_LEGO             0x0694
#define EV314_PRODUCT_EV3             0x0005
#define EV314_MAX_RETRY               3
#define EV314_RETRY_TIMEOUT           200000

/* EV314 Error codes */

#define EV314_OK                      0
#define EV314_USB_ERROR				  1
#define EV314_NOT_PRESENT             2
#define EV314_CONFIGURATION_ERROR     3
#define EV314_IN_USE                  4
#define EV314_USB_WRITE_ERROR         5
#define EV314_USB_READ_ERROR          6
#define EV314_USB_PARTIAL_TRANS		  7
#define EV314_USB_OVERFLOW			  9
#define EV314_BYTECODE_ERROR          10

/* EV314 PreProcessor Macros */

#define EV314_PROFILING_ON			  // Comment to deactivate profiling

typedef int                     EV314_error_t;
EV314_error_t 					ret;
struct timespec					profiling_start;
struct ev314_control_struct		ev314_control;
struct ev314_state_struct		ev314_state;
struct libusb_device_handle	   *EV314_hdl;

/*
* ev314_profiling_start: start timer
*
*/
void ev314_profiling_start(void) {
#ifdef EV314_PROFILING_ON
	clock_gettime(CLOCK_MONOTONIC, &profiling_start);
#endif
}

/*
* ev314_profiling_stop: stop timer and print time
*
*/
void ev314_profiling_stop(void) {
#ifdef EV314_PROFILING_ON
	struct timespec						profiling_stop;

	clock_gettime(CLOCK_MONOTONIC, &profiling_stop);

	fprintf(stderr, "** Profiling duration: %d us.\n",
		(int)((profiling_stop.tv_sec - profiling_start.tv_sec) * 1000000
			+ (profiling_stop.tv_nsec - profiling_start.tv_nsec) / 1000));

#endif
}

/*
* USB API.
*
*/
EV314_error_t EV314_init(void) {
	int status;

	status = libusb_init(NULL);

	if (status)
		return EV314_USB_ERROR;

	return EV314_OK;
}


struct libusb_device_handle* EV314_find_and_open(char* expected_serial) {
	libusb_device 									*dev, **devs;
	struct libusb_device_descriptor	desc;
	struct libusb_device_handle			*EV314_hdl = NULL;
	int															i, status, transfered;
	unsigned char	                  tmpbuf[EV314_PACKET_SIZE];
	char														serial[EV314_LENGTH_SERIAL + 1];

	if (libusb_get_device_list(NULL, &devs) < 0)
		return NULL;

	/* Go through device list loooking for an EV3 device */
	for (i = 0; (dev = devs[i]) != NULL; i++) {

		status = libusb_get_device_descriptor(dev, &desc);

		if (status >= 0) {
			if ((desc.idVendor == EV314_VENDOR_LEGO) &&
				(desc.idProduct == EV314_PRODUCT_EV3)) {

				/* Open the device */
				status = libusb_open(dev, &EV314_hdl);
				if (status < 0) {
					libusb_free_device_list(devs, 1);
					return NULL;
				}

				/* Check the serial number */

				status = libusb_get_string_descriptor_ascii(EV314_hdl, desc.iSerialNumber, (unsigned char*)serial, sizeof(serial));
				if (status == EV314_LENGTH_SERIAL) {
					if (strcmp(expected_serial, serial)) {
						libusb_close(EV314_hdl);
						EV314_hdl = NULL;
						continue;
					}
				}
				else {
					libusb_close(EV314_hdl);
					EV314_hdl = NULL;
					continue;
				}

				/* Detach possible kernel driver bound to interface */
				libusb_detach_kernel_driver(EV314_hdl, EV314_INTERFACE_NUMBER);

				/* Claiming the interface */
				status = libusb_claim_interface(EV314_hdl, EV314_INTERFACE_NUMBER);
				if (status) {
					libusb_close(EV314_hdl);
					libusb_free_device_list(devs, 1);
					return NULL;
				}

				/* Request a packet until getting a zero byte packet */
				do
				{
					status = libusb_bulk_transfer(EV314_hdl, EV314_EP_IN, tmpbuf, EV314_PACKET_SIZE, &transfered, EV314_USB_TIMEOUT);
				} while ((status == 0) && (transfered != 0));

				libusb_free_device_list(devs, 1);
				return EV314_hdl;
			}
		}
	}

	libusb_free_device_list(devs, 1);
	return NULL;
}

EV314_error_t EV314_close(struct libusb_device_handle *EV314_hdl) {

	if (EV314_hdl == NULL)
		return EV314_CONFIGURATION_ERROR;

	libusb_release_interface(EV314_hdl, EV314_INTERFACE_NUMBER);
	libusb_close(EV314_hdl);
	EV314_hdl = NULL;
	libusb_exit(NULL);

	return EV314_OK;
}

EV314_error_t EV314_send_buf(struct libusb_device_handle *EV314_hdl, unsigned char *buf, int len) {
	int	status, transfered = 0;

	if (EV314_hdl == NULL)
		return EV314_CONFIGURATION_ERROR;

	if (len > EV314_PACKET_SIZE)
		return EV314_USB_OVERFLOW;

	status = libusb_interrupt_transfer(EV314_hdl, EV314_EP_OUT, buf, len, &transfered, EV314_USB_TIMEOUT);

	if (status)
		return EV314_USB_WRITE_ERROR;

	if (transfered != len)
		return EV314_USB_PARTIAL_TRANS;

	return EV314_OK;
}

EV314_error_t EV314_recv_buf(struct libusb_device_handle *EV314_hdl, unsigned char *buf, int len) {
	int						i, status, transfered = 0;
	unsigned char	tmpbuf[EV314_PACKET_SIZE];

	if (EV314_hdl == NULL)
		return EV314_CONFIGURATION_ERROR;

	if (len > EV314_PACKET_SIZE)
		return EV314_USB_OVERFLOW;

	status = libusb_interrupt_transfer(EV314_hdl, EV314_EP_IN, tmpbuf, EV314_PACKET_SIZE, &transfered, EV314_USB_TIMEOUT);

	if (status)
		return EV314_USB_READ_ERROR;

	if (transfered != EV314_PACKET_SIZE)
		return EV314_USB_PARTIAL_TRANS;

	for (i = 0; i < len; i++)
		buf[i] = tmpbuf[i];

	return EV314_OK;
}
