#ifndef ARM_USB_H
#define ARM_USB_H





void init_usb();
void usb_start_receive();
void usb_start_transmit(unsigned char *data, int len, int do_null);
void handle_usb();






#endif


