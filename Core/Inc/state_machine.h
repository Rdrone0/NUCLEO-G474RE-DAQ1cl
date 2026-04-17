#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

/* Runs Stage 1 (blocking init), then leaves system in READY state.
   Call once from main() after all HAL/BSP inits. */
void StateMachine_Init(void);

/* Call from the main loop — handles Stage 3 flush and VCP status prints */
void StateMachine_Process(void);

/* Called from HAL_GPIO_EXTI_Callback for GPIO_PIN_13 (B1 button, PC13) */
void StateMachine_OnButton(void);

#endif /* __STATE_MACHINE_H */
