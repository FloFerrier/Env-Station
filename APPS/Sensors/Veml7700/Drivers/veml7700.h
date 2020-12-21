#ifndef VEML7700_DRIVERS_H
#define VEML7700_DRIVERS_H

enum als_gain_e {
  ALS_GAIN_X1,
  ALS_GAIN_X2,
  ALS_GAIN_X1_8,
  ALS_GAIN_X1_4,
};

enum als_it_e {
  ALS_IT_25_MS,
  ALS_IT_50_MS,
  ALS_IT_100_MS,
  ALS_IT_200_MS,
  ALS_IT_400_MS,
  ALS_IT_800_MS,
};

enum als_pers_e {
  ALS_PERS_1,
  ALS_PERS_2,
  ALS_PERS_4,
  ALS_PERS_8,
};

struct als_config_s {
  enum als_gain_e gain;
  enum als_it_e integration_time;
  enum als_pers_e persistance_protect;
};

struct veml7700_dev {
  struct als_config_s config;
};

#endif /* VEML7700_DRIVERS_H */
