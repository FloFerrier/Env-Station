#include "veml7700.h"

#include "Debug/console.h"

/* Configuration register */
#define REG_ALS_GAIN_OFFSET   (11)
#define REG_ALS_IT_OFFSET     (6)
#define REG_ALS_PERS_OFFSET   (4)
#define REG_ALS_INT_EN_OFFSET (1)
#define REG_ALS_SD_OFFSET     (0)

/* Power Saving Modes register */
#define REG_PSM_OFFSET    (1)
#define REG_PSM_EN_OFFSET (0)

/* Interrupt Status register */
#define REG_INT_TH_LOW_OFFSET (15)
#define REG_INT_TH_HIGH_OFFSET (14)

static void set_configuration_reg(struct als_config_reg_s reg_settings, uint16_t* reg_value)
{
  /* ALS_GAIN Bit [12:11] */
  *reg_value &= ~(0b11 << REG_ALS_GAIN_OFFSET);
  *reg_value |= reg_settings.gain << REG_ALS_GAIN_OFFSET;

  /* ALS_IT Bit [9:6] */
  *reg_value &= ~(0b1111 << REG_ALS_IT_OFFSET);
  *reg_value |= reg_settings.integration_time << REG_ALS_IT_OFFSET;

  /* ALS_PERS Bit [5:4] */
  *reg_value &= ~(0b11 << REG_ALS_PERS_OFFSET);
  *reg_value |= reg_settings.persistance_protect << REG_ALS_PERS_OFFSET;

  /* ALS_INT_EN Bit [1] */
  *reg_value &= ~(0b1 << REG_ALS_INT_EN_OFFSET);
  if(reg_settings.interrupt_enable != false)
  {
    *reg_value |= 0b1 << REG_ALS_INT_EN_OFFSET;
  }

  /* ALS_INT_EN Bit [0] */
  *reg_value &= ~(0b1 << REG_ALS_SD_OFFSET);
  if(reg_settings.shutdown != false)
  {
    *reg_value |= 0b1 << REG_ALS_SD_OFFSET;
  }
}

static void set_power_saving_reg(struct power_saving_reg_s reg_settings, uint16_t* reg_value)
{
  /* PSM Bit [2:1] */
  *reg_value &= ~(0b11 << REG_PSM_OFFSET);
  *reg_value |= reg_settings.psm << REG_PSM_OFFSET;

  /* PSM_EN Bit [0] */
  *reg_value &= ~(0b1 << REG_PSM_EN_OFFSET);
  if(reg_settings.psm_enable != false)
  {
    *reg_value |= 0b1 << REG_PSM_EN_OFFSET;
  }
}

static void set_interrupt_status_reg(struct interrupt_status_s reg_settings, uint16_t* reg_value)
{
  /* INT_TH_LOW Bit [15] */
  *reg_value &= ~(0b1 << REG_INT_TH_LOW_OFFSET);
  if(reg_settings.int_th_low != false)
  {
    *reg_value |= 0b1 << REG_INT_TH_LOW_OFFSET;
  }

  /* INT_TH_HIGH Bit [14] */
  *reg_value &= ~(0b1 << REG_INT_TH_HIGH_OFFSET);
  if(reg_settings.int_th_high != false)
  {
    *reg_value |= 0b1 << REG_INT_TH_HIGH_OFFSET;
  }
}

uint8_t veml7700_set_config(struct veml7700_dev dev)
{
  uint16_t reg_config_val = 0x0;
  dev.read(SLAVE_ADDRESS, CONFIG_REG, &reg_config_val);
  set_configuration_reg(dev.config_reg, &reg_config_val);
  console_debug("[VEML7700] Set reg:0x%x val:0x%x\r\n", CONFIG_REG, reg_config_val);

  uint16_t reg_high_threshold_val = dev.high_threshold_reg;
  console_debug("[VEML7700] Set reg:0x%x val:0x%x\r\n", HIGH_THRESHOLD_REG, reg_high_threshold_val);

  uint16_t reg_low_threshold_val = dev.low_threshold_reg;
  console_debug("[VEML7700] Set reg:0x%x val:0x%x\r\n", LOW_THRESHOLD_REG, reg_low_threshold_val);

  uint16_t reg_power_saving_val = 0x0;
  dev.read(SLAVE_ADDRESS, POWER_SAVING_REG, &reg_power_saving_val);
  set_power_saving_reg(dev.power_saving, &reg_power_saving_val);
  console_debug("[VEML7700] Set reg:0x%x val:0x%x\r\n", POWER_SAVING_REG, reg_power_saving_val);

  uint16_t reg_interrupt_status_val = 0x0;
  dev.read(SLAVE_ADDRESS, INTERRUPT_STATUS_REG, &reg_interrupt_status_val);
  set_interrupt_status_reg(dev.interrupt_status, &reg_interrupt_status_val);
  console_debug("[VEML7700] Set reg:0x%x val:0x%x\r\n", INTERRUPT_STATUS_REG, reg_interrupt_status_val);

  return 0;
}

uint8_t veml7700_data_output(struct veml7700_dev *dev)
{
  uint16_t reg_high_resolution_val = 0x0;
  dev->read(SLAVE_ADDRESS, HIGH_RESOLUTION_DATA_REG, &reg_high_resolution_val);
  dev->high_resolution_data = reg_high_resolution_val;
  console_debug("[VEML7700] Set reg:0x%x val:0x%x\r\n", HIGH_RESOLUTION_DATA_REG, dev->high_resolution_data);

  uint16_t reg_white_channel_val = 0x0;
  dev->read(SLAVE_ADDRESS, WHITE_CHANNEL_DATA_REG, &reg_white_channel_val);
  dev->white_channel_data = reg_white_channel_val;
  console_debug("[VEML7700] Set reg:0x%x val:0x%x\r\n", WHITE_CHANNEL_DATA_REG, dev->white_channel_data);

  return 0;
}
