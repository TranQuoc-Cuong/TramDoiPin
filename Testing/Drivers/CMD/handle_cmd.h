#ifndef HANDLE_CMD_H_
#define HANDLE_CMD_H_

#include "battery_control.h"

typedef void (*CmdHandler)(char *args);

typedef struct {
	const char *cmd_name;  
	CmdHandler handler;  
} CommandEntry;

void CMDProcessAll(char *full_command);

#endif /* HANDLE_CMD_H_ */
