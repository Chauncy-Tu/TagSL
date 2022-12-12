#ifndef USER_USB_H
#define USER_USB_H

#include <stdint.h>

void user_init_usbd(void);
void usb_send(uint8_t *str,uint16_t length);

#endif