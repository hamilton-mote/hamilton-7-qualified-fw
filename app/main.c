#include <stdio.h>
#include <rtt_stdio.h>
#include "thread.h"
#include "xtimer.h"
#include <string.h>

#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"

#include <at30ts74.h>
#include <mma7660.h>
#include <periph/gpio.h>
#include <periph/i2c.h>


#define SAMPLE_INTERVAL (2000000UL)
#define ACC_EVERY_MASK 15
#define TYPE_FIELD 4

void send_udp(char *addr_str, uint16_t port, uint8_t *data, uint16_t datalen);

at30ts74_t tmp;
mma7660_t acc;
uint32_t sample_counter;

typedef struct {
  uint16_t type;
  int8_t flags; //which of the fields below exist, bit 0 is acc_x
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int32_t temperature; // in C*10000
  int32_t lux;
  uint64_t uptime;
} measurement_t;


void sample(measurement_t *m);

#define FLAG_HAS_ACC_X  0x01
#define FLAG_HAS_ACC_Y  0x02
#define FLAG_HAS_ACC_Z  0x04
#define FLAG_HAS_ACC    0x07
#define FLAG_HAS_TEMP   0x08
#define FLAG_HAS_LUX    0x10
#define FLAG_HAS_UPTIME 0x20
#define DO_ACC (0)

void low_power_init(void) {
    // Light sensor off
    gpio_init(GPIO_PIN(0,28), GPIO_OUT);
    gpio_write(GPIO_PIN(0, 28), 1);
    i2c_poweroff(I2C_0);

    // Temperature sensor off
    if (at30ts74_init(&tmp, I2C_0, AT30TS74_ADDR, AT30TS74_12BIT) != 0)
        printf("Failed to init TEMP\n");

    // Accelerometer off
    if (mma7660_init(&acc, I2C_0, MMA7660_ADDR) != 0)
      printf("Failed to init ACC\n");
    if (mma7660_set_mode(&acc, 0, 0, 0, 0) != 0)
      printf("Failed to set idle mode\n");
	  if (mma7660_config_samplerate(&acc, MMA7660_SR_AM1, MMA7660_SR_AW1, 1) != 0)
      printf("Failed to config SR\n");
}

void sample(measurement_t *m) {
    int32_t temp = 0;
    int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;
    uint8_t do_acc = 0;
    #if DO_ACC
    if ((sample_counter & (ACC_EVERY_MASK)) == 0) {
      do_acc = 1;
      if (mma7660_set_mode(&acc, 1, 0, 0, 0) != 0) {
        printf("Failed to set active mode\n");
        return;
      }
    }
    #endif

    if(at30ts74_read(&tmp, &temp)) {
      printf("Failed to read temp\n");
      return;
    }

    #if DO_ACC
    if (do_acc) {
      if (mma7660_read(&acc, &x, &y, &z)) {
        printf("Failed to read accel\n");
        return;
      }
    }
    if (do_acc) {
      if (mma7660_set_mode(&acc, 0, 0, 0, 0) != 0) {
        printf("Failed to set idle mode");
      }
    }
    #endif
    sample_counter++;


    m->type = TYPE_FIELD;
    m->flags = FLAG_HAS_TEMP;
    m->temperature = temp;
    m->flags |= FLAG_HAS_UPTIME;
    m->uptime = xtimer_usec_from_ticks64(xtimer_now64());
    if (do_acc) {
      m->flags |= FLAG_HAS_ACC;
      m->acc_x = x;
      m->acc_y = y;
      m->acc_z = z;
    }


    //printf("[%lu] temperature: %luC / accel %d %d %d\n", xtimer_usec_from_ticks(xtimer_now()), temp, x, y, z);
}


int main(void)
{
    measurement_t m;
    low_power_init();
    kernel_pid_t radio[GNRC_NETIF_NUMOF];
    uint8_t radio_num = gnrc_netif_get(radio);
    while (1) {
      sample(&m);
      //netopt_state_t radio_state = NETOPT_STATE_IDLE;
   	  //for (i=0; i < radio_num; i++)
   	  //    gnrc_netapi_set(radio[i], NETOPT_STATE, 0, &radio_state, sizeof(netopt_state_t));
      netopt_state_t radio_state = NETOPT_STATE_SLEEP;
      send_udp("ff02::1",4747,(uint8_t*)&m,sizeof(measurement_t));
      for (int i=0; i < radio_num; i++)
          gnrc_netapi_set(radio[i], NETOPT_STATE, 0, &radio_state, sizeof(netopt_state_t));
      xtimer_usleep(SAMPLE_INTERVAL);
    }

    return 0;
}
