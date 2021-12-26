#include "paint.h"

static PAINT Paint;

void Paint_NewImage(uint8_t *image, uint16_t Width, uint16_t Height, uint16_t Rotate, uint16_t Color)
{
  Paint.Image = NULL;
  Paint.Image = image;

  Paint.WidthMemory = Width;
  Paint.HeightMemory = Height;
  Paint.Color = Color;
  Paint.WidthByte = (Width % 8 == 0)? (Width / 8 ): (Width / 8 + 1);
  Paint.HeightByte = Height;
  //printf("WidthByte = %d, HeightByte = %d\r\n", Paint.WidthByte, Paint.HeightByte);
  //printf(" EPD_WIDTH / 8 = %d\r\n",  122 / 8);

  Paint.Rotate = Rotate;
  Paint.Mirror = MIRROR_NONE;

  if(Rotate == ROTATE_0 || Rotate == ROTATE_180)
  {
    Paint.Width = Width;
    Paint.Height = Height;
  }
  else
  {
    Paint.Width = Height;
    Paint.Height = Width;
  }
}

int8_t Paint_SetPixel(uint16_t Xpoint, uint16_t Ypoint, uint16_t Color)
{
  uint16_t X = 0;
  uint16_t Y = 0;
  uint32_t Addr = 0;
  uint8_t Rdata = 0;

  if(Xpoint > Paint.Width || Ypoint > Paint.Height)
  {
    //Debug("Exceeding display boundaries\r\n");
    return -1;
  }

  switch(Paint.Rotate)
  {
    case 0:
      X = Xpoint;
      Y = Ypoint;  
      break;
    case 90:
      X = Paint.WidthMemory - Ypoint - 1;
      Y = Xpoint;
      break;
    case 180:
      X = Paint.WidthMemory - Xpoint - 1;
      Y = Paint.HeightMemory - Ypoint - 1;
      break;
    case 270:
      X = Ypoint;
      Y = Paint.HeightMemory - Xpoint - 1;
      break;

    default:
      return -1;
  }

  switch(Paint.Mirror)
  {
    case MIRROR_NONE:
      break;
    case MIRROR_HORIZONTAL:
      X = Paint.WidthMemory - X - 1;
      break;
    case MIRROR_VERTICAL:
      Y = Paint.HeightMemory - Y - 1;
      break;
    case MIRROR_ORIGIN:
      X = Paint.WidthMemory - X - 1;
      Y = Paint.HeightMemory - Y - 1;
      break;
    default:
      return -1;
  }

  if(X > Paint.WidthMemory || Y > Paint.HeightMemory)
  {
    //Debug("Exceeding display boundaries\r\n");
    return -1;
  }
    
  Addr = X / 8 + Y * Paint.WidthByte;
  Rdata = Paint.Image[Addr];
  if(Color == BLACK)
  {
    Paint.Image[Addr] = Rdata & ~(0x80 >> (X % 8));
  }
  else
  {
    Paint.Image[Addr] = Rdata | (0x80 >> (X % 8));
  }

  return 0;
}

int8_t Paint_DrawChar(uint16_t Xpoint, uint16_t Ypoint, const char Acsii_Char,
  sFONT* Font, uint16_t Color_Background, uint16_t Color_Foreground)
{
  if(Xpoint > Paint.Width || Ypoint > Paint.Height)
  {
    //Debug("Paint_DrawChar Input exceeds the normal display range\r\n");
    return -1;
  }

  uint32_t Char_Offset = (Acsii_Char - ' ') * Font->Height * (Font->Width / 8 + (Font->Width % 8 ? 1 : 0));
  const uint8_t *ptr;
  ptr = &Font->table[Char_Offset];

  for(size_t Page = 0; Page < Font->Height; Page ++)
  {
    for(size_t Column = 0; Column < Font->Width; Column ++)
    {
	  if(FONT_BACKGROUND == Color_Background)
	  { //this process is to speed up the scan
        if (*ptr & (0x80 >> (Column % 8)))
		{
          if(0 != Paint_SetPixel(Xpoint + Column, Ypoint + Page, Color_Foreground))
		  {
			return -1;
		  }
          // Paint_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
		}
      }
	  else
	  {
        if(*ptr & (0x80 >> (Column % 8)))
		{
          if(0 != Paint_SetPixel(Xpoint + Column, Ypoint + Page, Color_Foreground))
		  {
			return -1;
		  }
          // Paint_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
        }
		else
		{
          if(0 != Paint_SetPixel(Xpoint + Column, Ypoint + Page, Color_Background))
		  {
			return -1;
		  }
          // Paint_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Background, DOT_PIXEL_DFT, DOT_STYLE_DFT);
        }
      }
      //One pixel is 8 bits
      if (Column % 8 == 7)
	  {
        ptr++;
	  }
    }// Write a line
    if(Font->Width % 8 != 0)
	{
      ptr++;
	}
  }// Write all

  return 0;
}

int8_t Paint_Draw_String(uint16_t Xstart, uint16_t Ystart, const char *pString, size_t uSize,
  sFONT* Font, uint16_t Color_Background, uint16_t Color_Foreground)
{
  uint16_t Xpoint = Xstart;
  uint16_t Ypoint = Ystart;

  if(Xstart > Paint.Width || Ystart > Paint.Height)
  {
    //Debug("Paint_DrawString_EN Input exceeds the normal display range\r\n");
    return -1;
  }

  for(size_t idx = 0; idx < uSize; idx++)
  {
    //if X direction filled , reposition to(Xstart,Ypoint),Ypoint is Y direction plus the Height of the character
    if((Xpoint + Font->Width ) > Paint.Width)
    {
      Xpoint = Xstart;
      Ypoint += Font->Height;
    }

    // If the Y direction is full, reposition to(Xstart, Ystart)
    if((Ypoint  + Font->Height ) > Paint.Height)
    {
      Xpoint = Xstart;
      Ypoint = Ystart;
    }

	if(0 != Paint_DrawChar(Xpoint, Ypoint, pString[idx], Font, Color_Background, Color_Foreground))
	{
	  return -1;
	}

    //The next word of the abscissa increases the font of the broadband
    Xpoint += Font->Width;
  }

  return 0;
}