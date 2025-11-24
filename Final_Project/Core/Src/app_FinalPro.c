/*
 * app_FinalPro.c
 *
 *  Created on: Nov 24, 2025
 *      Author: sadug
 */
#include "app.h"
#include <stdio.h>
#include <string.h>
#include "./BME280/bme280.h"
#include "fonts.h"
#include "ssd1306.h"

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

float temperature;
float humidity;
float pressure;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

char hum_string[50];
char temp_string[50];
char press_string[50];

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}

void App_Init(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();


  /* BME280 init */
  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;

  rslt = bme280_init(&dev);

  /* BME280 settings */
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;
  rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);

  /* Initialize OLED*/
  SSD1306_Init();
}
void App_MainLoop(void)
  {
    /* Forced mode setting, switched to SLEEP mode after measurement */
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    dev.delay_ms(40);
    /*Get Data */
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    if(rslt == BME280_OK)
    {
      temperature = comp_data.temperature / 100.0;
      humidity = comp_data.humidity / 1024.0;
      pressure = comp_data.pressure / 10000.0;

      /*Display Data */
      memset(hum_string, 0, sizeof(hum_string));
      memset(temp_string, 0, sizeof(temp_string));
      memset(press_string, 0, sizeof(press_string));

      sprintf(hum_string, "Humidity %03.1f %% ", humidity);
      sprintf(temp_string, "Temperature %03.1f C ", temperature);
      sprintf(press_string, "Pressure %03.1f hPa ", pressure);

      SSD1306_GotoXY (0, 0);
      SSD1306_Puts (hum_string, &Font_7x10, 1);
      SSD1306_GotoXY (0, 20);
      SSD1306_Puts (temp_string, &Font_7x10, 1);
      SSD1306_GotoXY (0, 40);
      SSD1306_Puts (press_string, &Font_7x10, 1);
      SSD1306_UpdateScreen();
    }

    HAL_Delay(1000);
  }


