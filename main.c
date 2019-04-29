#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arena.h"
#include "arm_math.h"
#include "ch.h"
#include "communications.h"
#include "hal.h"
#include "i2c_bus.h"
#include "leds.h"
#include <main.h>
#include "motors_advanced.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include "chprintf.h"
#include "spi_comm.h"
//#include "chsys.h"


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    //starts the USB communication
    usb_start();
    spi_comm_start();
    //inits the arena

    init_arena();
	set_body_led(0);
	set_front_led(0);
    messagebus_init(&bus,&bus_lock,&bus_condvar);
    chThdSleepMilliseconds(3000);

    //Initialize the robot position to arena center
	gotoarenacenter();
	set_body_led(0);
    chThdSleepMilliseconds(1000);
	set_front_led(0);

    //Clean
    search_waste();


    /* Infinite loop. */
    while (1) {
    	chThdSleepMilliseconds(2000);
    }


}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}


