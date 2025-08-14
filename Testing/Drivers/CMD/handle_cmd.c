#include "handle_cmd.h"

void CMD_ParseSimple(char* args, ChannelState_t state) {
    char* token = strtok(args, ",;"); 
    
    while (token != NULL) {
        uint8_t channel_id = atoi(token); 
        if (channel_id > 0 && channel_id <= 4) {
            Channel_SetState(channel_id, state);
        }
        token = strtok(NULL, ",;");
    }
}

void CMD_ParseServo(char* args) {
    char* channel_part = strtok(args, ".");
    char* angle_part = strtok(NULL, ";");

    if (channel_part != NULL && angle_part != NULL) {
        uint8_t angle = atoi(angle_part);

        char* channel_token = strtok(channel_part, ",");
        while (channel_token != NULL) {
            uint8_t channel_id = atoi(channel_token);
            if (channel_id > 0 && channel_id <= 4) {
                Servo_SetAngle(channel_id, angle);
            }
            channel_token = strtok(NULL, ",");
        }
    }
}

void CMD_Process(char* command_buffer) {
    if (strncmp(command_buffer, "sac:", 4) == 0) {
        CMD_ParseSimple(command_buffer + 4, STATE_CHARGING);
    } 
    else if (strncmp(command_buffer, "test:", 5) == 0) {
        CMD_ParseSimple(command_buffer + 5, STATE_TESTING);
    } 
    else if (strncmp(command_buffer, "pinoff:", 7) == 0) {
        CMD_ParseSimple(command_buffer + 7, STATE_IDLE);
    } 
    else if (strncmp(command_buffer, "servo:", 6) == 0) {
        CMD_ParseServo(command_buffer + 6);
    }
}
