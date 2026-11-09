#ifndef REFEREE_KEYBOARD_H
#define REFEREE_KEYBOARD_H

#include "keyBoard.h"

#include <stdint.h>

void Referee_KeyboardEdgeDetect(const keyboard_t *kb);
void Referee_OnKeyboardKeyPressed(uint8_t key, uint8_t ctrl_pressed);
void Referee_OnKeyboardCtrlKeyPressed(uint8_t key);
void Referee_OnKeyboardShiftKeyPressed(uint8_t key);
const keyboard_t *Referee_GetActiveKeyboard(void);

#endif
