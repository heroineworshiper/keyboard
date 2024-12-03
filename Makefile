

ARM_OBJS := \
	../stm32stuff/startup_main.o \
	../stm32stuff/arm_math2.o \
	../stm32stuff/uart.o \
	../stm32stuff/linux.o \
	../stm32stuff/misc.o \
	../stm32stuff/stm32f4xx_rcc.o \
	../stm32stuff/stm32f4xx_usart.o \
	../stm32stuff/stm32f4xx_gpio.o \
	../stm32stuff/stm32f4xx_dcmi.o \
	../stm32stuff/stm32f4xx_dma.o \
	../stm32stuff/stm32f4xx_i2c.o \
	stm32f4xx_it.o \
	../stm32stuff/stm32f4xx_iwdg.o \
	../stm32stuff/stm32f4xx_tim.o \
	../stm32stuff/stm32f4xx_adc.o \
	../stm32stuff/stm32f4xx_flash.o \
	../stm32stuff/stm32f4xx_syscfg.o \
        ../stm32stuff/system_stm32f4xx.o


USB_OBJS := \
        arm_usb.o \
	usb_dcd_int.o \
	../stm32stuff/usb_bsp.o \
	../stm32stuff/usb_core.o \
	../stm32stuff/usb_dcd.o \
	../stm32stuff/usb_hcd.o \
	../stm32stuff/usbd_core.o \
	../stm32stuff/usbd_req.o \
	../stm32stuff/usbd_ioreq.o \
	../stm32stuff/usbh_core.o \
	../stm32stuff/usbh_hcs.o \
	../stm32stuff/usbh_ioreq.o \
	../stm32stuff/usbh_stdreq.o

KEYBOARD_OBJS := \
        keyboard.o


GCC_ARM := /opt/gcc-arm-none-eabi-4_6-2012q2/bin/arm-none-eabi-gcc
OBJCOPY := /opt/gcc-arm-none-eabi-4_6-2012q2/bin/arm-none-eabi-objcopy
ARM_CFLAGS := \
	-O2 \
	-c \
	-mcpu=cortex-m4 \
	-mthumb \
	-march=armv7e-m \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mlittle-endian \
	-ffreestanding \
	-I. \
	-I../stm32stuff \
	-DENABLE_PRINT


ARM_LIBM := $(shell $(GCC_ARM) $(ARM_CFLAGS) -print-file-name=libm.a)
ARM_LIBC := $(shell $(GCC_ARM) $(ARM_CFLAGS) -print-libgcc-file-name)
ARM_LFLAGS := -mcpu=cortex-m4 \
	-mthumb \
	-march=armv7e-m \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mlittle-endian \
	-ffreestanding \
	-nostdlib \
	-nostdinc \
	$(ARM_LIBM) $(ARM_LIBC)

$(shell echo $(GCC_ARM) $(ARM_CFLAGS) > arm_gcc )

keyboard.bin: $(ARM_OBJS) $(USB_OBJS) $(KEYBOARD_OBJS)
	$(GCC_ARM) -o keyboard.elf \
		$(ARM_OBJS) \
                $(USB_OBJS) \
                $(KEYBOARD_OBJS) \
		$(ARM_LFLAGS) \
		-T../stm32stuff/main.ld
	$(OBJCOPY) -O binary keyboard.elf keyboard.bin

clean:
	rm -f $(ARM_OBJS) $(USB_OBJS) $(KEYBOARD_OBJS) keyboard.bin

$(ARM_OBJS) $(USB_OBJS) $(KEYBOARD_OBJS):
	`cat arm_gcc` -c $< -o $*.o




arm_usb.o:                           arm_usb.c
../stm32stuff/arm_math2.o:       ../stm32stuff/arm_math2.c
../stm32stuff/uart.o:            ../stm32stuff/uart.c
../stm32stuff/linux.o:           ../stm32stuff/linux.c
../stm32stuff/startup_main.o:    ../stm32stuff/startup_main.s
../stm32stuff/misc.o: 	     ../stm32stuff/misc.c
../stm32stuff/stm32f4xx_rcc.o:   ../stm32stuff/stm32f4xx_rcc.c
../stm32stuff/stm32f4xx_usart.o: ../stm32stuff/stm32f4xx_usart.c
../stm32stuff/stm32f4xx_gpio.o:  ../stm32stuff/stm32f4xx_gpio.c
../stm32stuff/stm32f4xx_dcmi.o:  ../stm32stuff/stm32f4xx_dcmi.c
../stm32stuff/stm32f4xx_dma.o:   ../stm32stuff/stm32f4xx_dma.c
../stm32stuff/stm32f4xx_i2c.o:   ../stm32stuff/stm32f4xx_i2c.c
stm32f4xx_it.o:    stm32f4xx_it.c
../stm32stuff/stm32f4xx_iwdg.o:  ../stm32stuff/stm32f4xx_iwdg.c
../stm32stuff/stm32f4xx_tim.o:   ../stm32stuff/stm32f4xx_tim.c
../stm32stuff/stm32f4xx_adc.o:   ../stm32stuff/stm32f4xx_adc.c
../stm32stuff/stm32f4xx_flash.o: ../stm32stuff/stm32f4xx_flash.c
../stm32stuff/stm32f4xx_syscfg.o: ../stm32stuff/stm32f4xx_syscfg.c
../stm32stuff/system_stm32f4xx.o: ../stm32stuff/system_stm32f4xx.c

../stm32stuff/usb_bsp.o:	      ../stm32stuff/usb_bsp.c
../stm32stuff/usb_core.o:	      ../stm32stuff/usb_core.c
../stm32stuff/usb_dcd.o:	      ../stm32stuff/usb_dcd.c
../stm32stuff/usb_hcd.o:	      ../stm32stuff/usb_hcd.c
../stm32stuff/usbd_core.o:	      ../stm32stuff/usbd_core.c
../stm32stuff/usbd_req.o:	      ../stm32stuff/usbd_req.c
../stm32stuff/usbd_ioreq.o:	      ../stm32stuff/usbd_ioreq.c
usb_dcd_int.o:                usb_dcd_int.c
../stm32stuff/usbh_core.o:	      ../stm32stuff/usbh_core.c
../stm32stuff/usbh_hcs.o:	      ../stm32stuff/usbh_hcs.c
../stm32stuff/usbh_ioreq.o:	      ../stm32stuff/usbh_ioreq.c
../stm32stuff/usbh_stdreq.o:	      ../stm32stuff/usbh_stdreq.c

keyboard.o: keyboard.c





