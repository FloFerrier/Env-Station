#ifndef EINK_FONT_DEFS_H
#define EINK_FONT_DEFS_H

#include <stdlib.h>
#include <stdint.h>

//ASCII
typedef struct _tFont
{
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
} sFONT;

extern sFONT Font12;
extern sFONT Font8;

#endif /* EINK_FONT_DEFS_H */