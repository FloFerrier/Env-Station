#include "ga1a1s202wp.h"

double xVoltToLux(uint32_t volt_value)
{
  uint32_t current = volt_value / 68; /*in microAmpere */
  return (pow((double)10, (double)(current / 10.0)));
}
