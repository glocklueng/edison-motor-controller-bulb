
#ifndef _EDISON_SOCKET_H_
#define _EDISON_SOCKET_H_

#define EDISON_SOCKET_VERSION 0x01

#define EDISON_SOCKET_CMD_NOT_SET      0x00
#define EDISON_SOCKET_CMD_READ_CONFIG  0x01
#define EDISON_SOCKET_CMD_USER         0x80

typedef struct {
  uint8_t version;
  const char driverUrl[255];
} EdisonSocketConfig;

#endif
