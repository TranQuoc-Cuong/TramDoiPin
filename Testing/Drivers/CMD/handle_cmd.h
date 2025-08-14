#include "battery_control.h"
#include "servo_control.h"
#include <string.h>
#include <stdlib.h> 

void CMD_ParseSimple(char* args, ChannelState_t state);

void CMD_ParseServo(char* args);

void CMD_Process(char* command_buffer);
