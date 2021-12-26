#ifndef EINK_PAINT_H
#define EINK_PAINT_H

#include <stdint.h>
#include <stdlib.h>

#include "font_defs.h"

typedef enum
{
    MIRROR_NONE       = 0x00,
    MIRROR_HORIZONTAL = 0x01,
    MIRROR_VERTICAL   = 0x02,
    MIRROR_ORIGIN     = 0x03,
} MIRROR_IMAGE;

#define ROTATE_0              (0)
#define ROTATE_90             (90)
#define ROTATE_180            (180)
#define ROTATE_270            (270)
#define MIRROR_IMAGE_DFT      MIRROR_NONE
#define WHITE                 (0xFF)
#define BLACK                 (0x00)

#define IMAGE_BACKGROUND    WHITE
#define FONT_FOREGROUND     BLACK
#define FONT_BACKGROUND     WHITE

typedef struct
{
    uint8_t *Image;
    uint16_t Width;
    uint16_t Height;
    uint16_t WidthMemory;
    uint16_t HeightMemory;
    uint16_t Color;
    uint16_t Rotate;
    uint16_t Mirror;
    uint16_t WidthByte;
    uint16_t HeightByte;
} PAINT;

void Paint_NewImage(uint8_t *image, uint16_t Width, uint16_t Height, uint16_t Rotate, uint16_t Color);
int8_t Paint_SetPixel(uint16_t Xpoint, uint16_t Ypoint, uint16_t Color);
int8_t Paint_DrawChar(uint16_t Xpoint, uint16_t Ypoint, const char Acsii_Char,
  sFONT* Font, uint16_t Color_Background, uint16_t Color_Foreground);
int8_t Paint_Draw_String(uint16_t Xstart, uint16_t Ystart, const char *pString, size_t uSizeString,
  sFONT* Font, uint16_t Color_Background, uint16_t Color_Foreground);

#endif /* EINK_PAINT_H */