// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include "controller.h"

#include "virtual_timer.h"
#include "gpio.h"
#include "software_interrupt.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

uint32_t dist_ahead;

static void ultrasonic_holler (void) {
  gpio_config(4, OUTPUT);
  gpio_clear(4);
  nrf_delay_us(2);
  gpio_set(4);
  nrf_delay_us(15);
  gpio_clear(4);
  gpio_config(4, INPUT);
}

void duration() {
  printf("entered duration\n");
  ultrasonic_holler();
  
  uint32_t timeout = 1000000L;
  uint32_t begin = read_timer();
  while (gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
  while (!gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
  uint32_t pulseBegin = read_timer();

  while (gpio_read(4)) if (read_timer() - begin >= timeout) { return; }
  uint32_t pulseEnd = read_timer();
  uint32_t dist = (pulseEnd - pulseBegin)*(10/2) / 29;
  dist_ahead = dist;
  
  // char buf[16];
	// snprintf(buf, 16, "%d", dist);
	// display_write(buf, DISPLAY_LINE_1);

  // printf("%d\n", dist);
  // return dist;  
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  virtual_timer_init();
  nrf_delay_ms(3000);
  // virtual_timer_start_repeated(1000000, ultrasonic_holler);
  virtual_timer_start_repeated(1000000, duration);
  // GPIOTE_Ultrasonic_ReceiveEdgeEvent();

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  // display_write("Hello, Human!", DISPLAY_LINE_0);
  // printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  robot_state_t state = OFF;
  // loop forever, running state machine
  while (1) {
    nrf_delay_ms(1);
    state = controller(state, dist_ahead);
  
  }
}

