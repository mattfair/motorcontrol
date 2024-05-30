/*
 * fsm.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */


#ifndef INC_FSM_H_
#define INC_FSM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum FsmState 
{
  MENU_MODE,
  CALIBRATION_MODE,
  MOTOR_MODE,
  SETUP_MODE,
  ENCODER_MODE,
  INIT_TEMP_MODE
};

#define MENU_CMD			27
#define MOTOR_CMD			'm'
#define CAL_CMD				'c'
#define ENCODER_CMD			'e'
#define SETUP_CMD			's'
#define ZERO_CMD			'z'
#define ENTER_CMD			13

typedef struct{
	enum FsmState state;
	enum FsmState next_state;
	uint8_t state_change;
	uint8_t ready;
	char cmd_buff[8];
	char bytecount;
	char cmd_id;
}FSMStruct;

void run_fsm(FSMStruct* fsmstate);
void update_fsm(FSMStruct * fsmstate, char fsm_input);
void fsm_enter_state(FSMStruct * fsmstate);
void fsm_exit_state(FSMStruct * fsmstate);
void enter_menu_state(void);
void enter_setup_state(void);
void enter_motor_mode(void);
void process_user_input(FSMStruct * fsmstate);

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_H_ */
