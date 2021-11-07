#ifndef VEML7700_DRIVERS_H
#define VEML7700_DRIVERS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define SLAVE_ADDRESS             (0x10)

#define CONFIG_REG                (0x00)
#define HIGH_THRESHOLD_REG        (0x01)
#define LOW_THRESHOLD_REG         (0x02)
#define POWER_SAVING_REG          (0x03)
#define HIGH_RESOLUTION_DATA_REG  (0x04)
#define WHITE_CHANNEL_DATA_REG    (0x05)
#define INTERRUPT_STATUS_REG      (0x06)

enum als_gain_e {
  ALS_GAIN_X1   = 0b00,
  ALS_GAIN_X2   = 0b01,
  ALS_GAIN_X1_8 = 0b10,
  ALS_GAIN_X1_4 = 0b11,
};

enum als_it_e {
  ALS_IT_25_MS  = 0b1100,
  ALS_IT_50_MS  = 0b1000,
  ALS_IT_100_MS = 0b0000,
  ALS_IT_200_MS = 0b0001,
  ALS_IT_400_MS = 0b0010,
  ALS_IT_800_MS = 0b0011,
};

enum als_pers_e {
  ALS_PERS_1 = 0b00,
  ALS_PERS_2 = 0b01,
  ALS_PERS_4 = 0b10,
  ALS_PERS_8 = 0b11,
};

struct als_config_reg_s {
  enum als_gain_e gain;
  enum als_it_e integration_time;
  enum als_pers_e persistance_protect;
  bool interrupt_enable;
  bool shutdown;
};

enum power_saving_mode_e {
  PSM_MODE_1 = 0b00,
  PSM_MODE_2 = 0b01,
  PSM_MODE_3 = 0b10,
  PSM_MODE_4 = 0b11,
};

struct power_saving_reg_s {
  enum power_saving_mode_e psm;
  bool psm_enable;
};

struct interrupt_status_s {
  bool int_th_low;
  bool int_th_high;
};

typedef int8_t (*veml7700_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint16_t *data);

struct veml7700_dev {
	veml7700_com_fptr_t read;
	veml7700_com_fptr_t write;
  struct als_config_reg_s config_reg;
  uint16_t high_threshold_reg;
  uint16_t low_threshold_reg;
  struct power_saving_reg_s power_saving;
  uint16_t high_resolution_data;
  uint16_t white_channel_data;
  struct interrupt_status_s interrupt_status;
};

uint8_t veml7700_set_config(struct veml7700_dev dev);
uint8_t veml7700_data_output(struct veml7700_dev *dev);

#endif /* VEML7700_DRIVERS_H */
