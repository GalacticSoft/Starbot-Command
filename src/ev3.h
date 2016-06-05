#ifndef __EV3_H__
#define __EV3_H__

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

//#define EV314_PROFILING_ON			  // Comment to deactivate profiling

typedef int                     EV314_error_t;

void ev314_profiling_start(void);
void ev314_profiling_stop(void);

#endif // !__EV3_H__