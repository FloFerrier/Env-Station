#include "hc05.h"

#define SLAVE  0
#define MASTER 1

#define MAX_BUFFER_SIZE 255

static int8_t HC05_Send(const char *p_str)
{
  if(p_str == NULL)
  {
    return -1;
  }

  while(*p_str != 0)
  {
    HC05_putchar(*p_str);
    p_str++;
  }

  return 0;
}

static  int8_t HC05_Command_Mode(void)
{
  static int8_t rslt = -1;
  static const char str[] = "AT\r\n";
  rslt = HC05_Send(str);
  return rslt;
}

static int8_t HC05_Restore_Default(void)
{
  /* Default state :
   * Slave Mode, PSWD: 1234, Name: H-C-2010-06-01, Baud: 38400 bauds
   */
  static int8_t rslt = -1;
  static const char str[] = "AT+ORGL\r\n";
  rslt = HC05_Send(str);
  return rslt;
}

static int8_t HC05_Set_Name(const char *p_str)
{
  static int8_t rslt = -1;
  if(p_str == NULL)
  {
    return -1;
  }

  static size_t size_str = 0;
  size_str = strnlen(p_str, 20);

  static char str[MAX_BUFFER_SIZE] = "AT+NAME=";

  strncat(str, p_str, size_str);
  strncat(str, "\r\n", 2);

  rslt = HC05_Send(str);

  return rslt;
}

static int8_t HC05_Set_Pswd(const char *p_str)
{
  static int8_t rslt = -1;
  static size_t size_str = 0;
  size_str = strnlen(p_str, 10);
  if((p_str == NULL) || (size_str != 4))
  {
    return -1;
  }

  static char str[255] = "AT+PSWD=";

  strncat(str, p_str, size_str);
  strncat(str, "\r\n", 2);

  rslt = HC05_Send(str);

  return rslt;
}

static int8_t HC05_Set_CMode(uint8_t mode)
{
  static int8_t rslt = -1;

  if(mode > 1)
  {
    return -1;
  }

  if(mode == MASTER)
  {
    rslt = HC05_Send("AT+CMODE=1\r\n");
  }
  else
  {
    rslt = HC05_Send("AT+CMODE=0\r\n");
  }

  return rslt;
}

bool HC05_Cmp_Response(const char *p_str)
{
  static bool rslt = false;
  static char buffer[MAX_BUFFER_SIZE];
  HC05_Receive(buffer);

  if(strcmp(buffer, p_str) == 0)
  {
    rslt = true;
  }

  return rslt;
}

int8_t HC05_Setup(const char *p_name, const char *p_pswd)
{
  HC05_Enable_Power(true);
  HC05_Timer(1000); // Wait 1 second
  HC05_Enable_Command_Mode(true);
  bool rslt = false;
  uint8_t try = 5;

  while((try > 0) && (rslt == false))
  {
    HC05_Command_Mode();
    rslt = HC05_Cmp_Response("OK\r\n");
    try--;
  }
  if(try == 0)
  {
    return -1;
  }

  HC05_Restore_Default();
  if(HC05_Cmp_Response("OK\r\n") != true)
  {
    return -1;
  }

  HC05_Set_Name(p_name);
  if(HC05_Cmp_Response("OK\r\n") != true)
  {
    return -1;
  }

  HC05_Set_Pswd(p_pswd);
  if(HC05_Cmp_Response("OK\r\n") != true)
  {
    return -1;
  }

  HC05_Set_CMode(MASTER);
  if(HC05_Cmp_Response("OK\r\n") != true)
  {
    return -1;
  }

  /* Re-power module */
  HC05_Enable_Power(false);
  HC05_Timer(1000);
  /* Data Mode */
  HC05_Enable_Command_Mode(false);
  HC05_Enable_Power(true);

  return 0;
}

int8_t HC05_Send_Data(const char *p_str)
{
  if(p_str == NULL)
  {
    return -1;
  }

  HC05_Send(p_str);

  return 0;
}
