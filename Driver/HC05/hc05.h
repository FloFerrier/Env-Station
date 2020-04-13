#ifndef HC05_H
#define HC05_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "Debug/printf/printf.h"

#define HC05_OK 0

/* DEFINE yours functions */
extern void HC05_Enable_Power(bool enable);
extern void HC05_Enable_Command_Mode(bool enable);
extern void HC05_putchar(char character);
extern void HC05_Receive(char *p_str);
extern void HC05_Timer(uint32_t time); // in ms

int8_t HC05_Setup(const char *p_name, const char *p_pswd);
bool HC05_Cmp_Response(const char *p_str);

int8_t HC05_Send_Data(const char *p_str);

#endif /* HC05_H */
