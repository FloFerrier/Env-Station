#include "ina219.h"

static int8_t i2c_read(uint8_t chip_addr,
                       uint8_t reg_addr,
                       uint8_t *ptr_data,
                       uint16_t len)
{
  i2c_transfer7(I2C1, chip_addr, &reg_addr, 1, ptr_data, (size_t)len);

  return 0;
}

static int8_t i2c_write(uint8_t chip_addr,
                        uint8_t reg_addr,
                        uint8_t *ptr_data,
                        uint16_t len)
{
  uint8_t buffer[len + 1];

  memcpy(buffer, &reg_addr, 1);
  memcpy(&(buffer[1]), ptr_data, len);

  i2c_transfer7(I2C1, chip_addr, buffer, (size_t)(len + 1), NULL, 0);

  return 0;
}

static void ina219_reset(void)
{
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_CONFIG, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];
  tmp |= INA219_REG_CONFIG_MASK_RST;
  ptr_data[0] = REG_16BITS_MSB(tmp);
  ptr_data[1] = REG_16BITS_LSB(tmp);
  i2c_write(INA219_CHIP_ADDR, INA219_REG_ADDR_CONFIG, ptr_data, 2);
}

static void ina219_start(void)
{
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_CONFIG, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];
  tmp |= INA219_REG_CONFIG_MASK_MODE;
  ptr_data[0] = REG_16BITS_MSB(tmp);
  ptr_data[1] = REG_16BITS_LSB(tmp);
  i2c_write(INA219_CHIP_ADDR, INA219_REG_ADDR_CONFIG, ptr_data, 2);
}

static void ina219_stop(void)
{
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_CONFIG, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];
  tmp &= ~(INA219_REG_CONFIG_MASK_MODE);
  ptr_data[0] = REG_16BITS_MSB(tmp);
  ptr_data[1] = REG_16BITS_LSB(tmp);
  i2c_write(INA219_CHIP_ADDR, INA219_REG_ADDR_CONFIG, ptr_data, 2);
}

static void ina219_set_reg_config(void)
{
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_CONFIG, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];

  /* Configration Register
   * Bits:  15 - 14 - 13 - 12-11 - 10-7 - 6-3 - 2-0
   * Mask: RST |    |BRNG|  PG   | BADC | SADC| MODE
   */
  tmp |= (INA219_REG_CONFIG_MASK_BRNG); /* Bus Voltage Range at 32V FSR */
  tmp |= (INA219_REG_CONFIG_MASK_PG);   /* PGA/8 : Shunt Voltage Only at +/320 mV */
  /* By default for Bus ADC */   /* One sample - 12 bits conversion time: 532us */
  /* By default for Shunt ADC */   /* One sample - 12 bits conversion time: 532us */
  tmp &= ~(INA219_REG_CONFIG_MASK_MODE);   /* Power-down */
  ptr_data[0] = REG_16BITS_MSB(tmp);
  ptr_data[1] = REG_16BITS_LSB(tmp);
  i2c_write(INA219_CHIP_ADDR, INA219_REG_ADDR_CONFIG, ptr_data, 2);
}

static void ina219_set_reg_calib(uint16_t val)
{
  val = (val << 1); /* Calibration stored in FS15:FS1 */
  uint8_t ptr_data[2] = {
    REG_16BITS_MSB(val),
    REG_16BITS_LSB(val)};
  i2c_write(INA219_CHIP_ADDR, INA219_REG_ADDR_CALIBRATION, ptr_data, 2);
}

/* in uV */
static int32_t ina219_get_shunt_voltage(void)
{
  int16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_SHUNT_VOLTAGE, ptr_data, 2);
  tmp = (int16_t)((ptr_data[0] << 8) + ptr_data[1]);
  return (tmp * 10); /* Shunt Voltage LSB (in uV) */
}

static uint32_t ina219_get_bus_voltage(void)
{
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_BUS_VOLTAGE, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];
  tmp = ((tmp & INA219_REG_BUS_VOLTAGE_MASK_BUS) >> 3);
  return (tmp * 4); /* Bus Voltage LSB */
}

static uint32_t ina219_get_power(void)
{
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_POWER, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];
  return (tmp * 20 * INA219_CURRENT_LSB); /* uW */
}

static int32_t ina219_get_current(void)
{
  int16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_CURRENT, ptr_data, 2);
  tmp = (int16_t)((ptr_data[0] << 8) + ptr_data[1]);
  return (tmp * INA219_CURRENT_LSB); /* uA */
}

static bool ina219_check_calcutions(void)
{
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_BUS_VOLTAGE, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];
  tmp &= INA219_REG_BUS_VOLTAGE_MASK_OVF;
  if(tmp == 1)
  {
    return false;
  }
  return true;
}

void ina219_setup(void)
{
  ina219_reset();
  ina219_set_reg_config();
  ina219_set_reg_calib(4096); /* Calculated value with equation 1 (Datasheet) */
}

bool ina219_check_conversion(void)
{
  uint16_t tmp = 0;
  uint8_t ptr_data[2] = {0, 0};
  i2c_read(INA219_CHIP_ADDR, INA219_REG_ADDR_BUS_VOLTAGE, ptr_data, 2);
  tmp = (ptr_data[0] << 8) + ptr_data[1];
  tmp &= INA219_REG_BUS_VOLTAGE_MASK_CNVR;
  if(tmp != 0)
  {
    return false;
  }
  return true;
}

int8_t ina219_get_data(struct ina219_s *tmp)
{
  int8_t rslt = -1;
  ina219_start();
  ina219_wait_conversion(); /* Blocking mode (defined with embedded archi) */

  if(ina219_check_calcutions() == true)
  {
    /* tmp->shunt_voltage = ina219_get_shunt_voltage(); */
    ina219_get_power(); /* Clear CNVR bit */
    tmp->voltage = ina219_get_bus_voltage();
    tmp->current = ina219_get_current();
    rslt = 0;
  }

  ina219_stop();

  return rslt;
}
