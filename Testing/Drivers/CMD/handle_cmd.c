#include "handle_cmd.h"

#include <stdlib.h> 
#include <string.h>
#include <stdio.h>

#include "main.h"
#include "servo_control.h"
#include "battery_control.h"  
#include "uart_dn.h"

extern volatile uint8_t          fit_pin_initial;
extern volatile uint8_t          fit_pin_remove;
extern volatile SystemState      status_machine;
extern volatile int8_t           esp32_pin_check_status;
extern PinChannel                pin_channels[3]; 
extern uint8_t                   id_pin_percent_max;    
extern uint8_t                   id_pin_empty;          

static void SendMessageWithOneID(char* string, uint8_t id) {
	char message[50];
	
	sprintf(message, string, id);
	TransmitStringHandlerUART(message);
}

static void SendMessageWithTwoID(char* string, uint8_t id_1, uint8_t id_2) {
	char message[50];
	
	sprintf(message, string, id_1, id_2);
	TransmitStringHandlerUART(message);
}

static void SendMessageWithIDsToESP32(char* string, uint8_t id_1, uint8_t id_2) {
	if ((0 == id_1) && (0 == id_2)) {
		TransmitStringHandlerUART(string);
	} else if ((0 != id_1) && (0 == id_2)) {
		SendMessageWithOneID(string, id_1);
	} else if ((0 != id_1) && (0 != id_2)) {
		SendMessageWithTwoID(string, id_1, id_2);
	}
}

void TakePinLogic(char *args) {
	if (id_pin_percent_max < 3) {
		fit_pin_remove = id_pin_percent_max;
		status_machine = WAIT_BATTERY_REMOVAL;
		
		SendMessageWithIDsToESP32("laypin:%d.\n", fit_pin_remove + 1, 0);
	} else {
		TransmitStringHandlerUART("Khong co pin san sang de lay!\r\n");
	}
}

void SwapPinLogic(char *args) {
	if (id_pin_empty < 3 && id_pin_percent_max < 3) {
		fit_pin_remove = id_pin_percent_max;
		fit_pin_initial = id_pin_empty;
		status_machine = SWAP_WAIT_INSERTION;
		
		SendMessageWithIDsToESP32("doipin:%d,laypin:%d\n", fit_pin_initial + 1, fit_pin_remove + 1);
	} else {
		TransmitStringHandlerUART("Khong the doi pin luc nay!\r\n");
	}
}

void InitialPinLogic(char *args) {
	uint8_t number_pin_empty = 0;

	for (uint8_t i = 0; i < 3; i++){
		if(pin_channels[i].status == pin_empty) ++number_pin_empty;                
	}
	
	if (number_pin_empty == 0) {
		SendMessageWithIDsToESP32("lappin:0\n", 0, 0);
	} else {
		SendMessageWithIDsToESP32("lappin:1\n", 0, 0);
		status_machine = INSTALL_BATTERY;
	}
}

void BackLogic(char *args) {
	status_machine = IDLE_WAIT_CMD;
}

void CheckPinCorrect(char *args) {
	esp32_pin_check_status = 1;
}

void CheckPinWrong(char *args) {
	esp32_pin_check_status = 0;
}

static void HelperParseAndSetState(char *args, ChannelState state) {
	char *saveptr;
	char *token = strtok_r(args, ",;", &saveptr);
	
	while (token != NULL) {
		uint8_t channel_id = atoi(token); 
		
		if (channel_id > 0 && channel_id < 4) {
			SetStateChannelBattery(channel_id, state);
		}
		
		token = strtok_r(NULL, ",;", &saveptr);
	}
}

void ChargeHandler(char *args) {
	HelperParseAndSetState(args, STATE_CHARGING);
}

void TestHandler(char *args) {
	HelperParseAndSetState(args, STATE_TESTING);
}

void PinOffHandler(char *args) {
	HelperParseAndSetState(args, STATE_IDLE);
}

void ServoHandler(char *args) {
	char *saveptr_outer;
	char *saveptr_inner;

	char *command_pair = strtok_r(args, ";", &saveptr_outer);

	while (command_pair != NULL) {
		char* channel_part = strtok_r(command_pair, ".", &saveptr_inner);
		char* angle_part = strtok_r(NULL, ".", &saveptr_inner);

		if (channel_part && angle_part) {
			uint8_t angle = atoi(angle_part);

			char* ch_token = strtok_r(channel_part, ",", &saveptr_inner);
			
			while (ch_token) {
				uint8_t id = atoi(ch_token);
				
				if (id > 0 && id < 4) {
						SetAngleServo(id, angle);
				}
				
				ch_token = strtok_r(NULL, ",", &saveptr_inner);
				}
		}
		command_pair = strtok_r(NULL, ";", &saveptr_outer);
	}
}

static const CommandEntry command_table[] = {
    {"sac",         ChargeHandler},
    {"test",        TestHandler},
    {"pinoff",      PinOffHandler},
    {"servo",       ServoHandler},
    {"laypin",      TakePinLogic}, 
    {"doipin",      SwapPinLogic},
    {"lappin",      InitialPinLogic},
		{"back",       BackLogic},
		{"pincorrect", CheckPinCorrect},
		{"pinwrong",   CheckPinWrong},
    {NULL,          NULL} 
};

void CMDProcessAll(char *full_command) {
	char *saveptr;
	char *cmd_name = strtok_r(full_command, ":", &saveptr);
	char *cmd_args = strtok_r(NULL, "", &saveptr); 

	if (cmd_name == NULL) return;

	for (int i = 0; command_table[i].cmd_name != NULL; i++) {
		if (strcmp(cmd_name, command_table[i].cmd_name) == 0) {
			if (command_table[i].handler != NULL) {
				command_table[i].handler(cmd_args ? cmd_args : ""); 
			}
			
			return;
		}
	}
}
