#include "referee_keyboard.h"
#include "Referee_Task.h"

#include "DBusSys.h"
#include "Chassis_Task.h"
#include "arm_referee.h"
#include "referee_api.h"
#include "referee_ui.h"
#include <stdbool.h>

extern rc_info_t remoter;
extern keyboard_t kb_info;

/**
 * @brief 返回当前控制源对应的键盘状态。
 * @return 编译期开关选择的遥控器内嵌键盘或自定义控制器键盘。
 */
const keyboard_t *Referee_GetActiveKeyboard(void)
{
#if (USE_REMOTER_KEYBOARD != 0)
    return &remoter.keyboard;
#else
    return &kb_info;
#endif
}

/**
 * @brief 处理普通按键的上升沿事件并切换底盘、UI 或机械臂功能状态。
 * @param key ASCII 按键编码。
 * @param ctrl_pressed Ctrl 是否同时按下；普通键分支中仅保留接口一致性。
 */
void Referee_OnKeyboardKeyPressed(uint8_t key, uint8_t ctrl_pressed)
{
    (void)ctrl_pressed;

    if (key == (uint8_t)'W')
    {
        //这里写你要执行的操作（W 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'S')
    {
        //这里写你要执行的操作（S 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'A')
    {
        //这里写你要执行的操作（A 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'D')
    {
        //这里写你要执行的操作（D 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'Q')
    {
        //这里写你要执行的操作（Q 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'E')
    {
        //这里写你要执行的操作（E 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'B')
    {
        //这里写你要执行的操作（B 从 0->1 的瞬间触发）
        Engineer_Mode.Ctrl_Logic_Mode =
            (Engineer_Mode.Ctrl_Logic_Mode == CTRL_LOGIC_MODE_DBUS) ? CTRL_LOGIC_MODE_Keyboard : CTRL_LOGIC_MODE_DBUS;
    }
    if (key == (uint8_t)'C')
    {
        Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Normal;
        return;
    }
    if (key == (uint8_t)'R')
    {
        Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Rising;
        return;
    }
    if (key == (uint8_t)'G')
    {
        //这里写你要执行的操作（G 从 0->1 的瞬间触发）
        Referee_UI_RequestRefresh();
        return;
    }
    if (key == (uint8_t)'Z')
    {
        //这里写你要执行的操作（Z 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'X')
    {
        //这里写你要执行的操作（X 从 0->1 的瞬间触发）
        Chassis_ToggleKeyboardDirectionState();
        return;
    }
    if (key == (uint8_t)'V')
    {
        //这里写你要执行的操作（V 从 0->1 的瞬间触发）
    }

    Arm_Keyboard_Manager(key);
}

void Referee_OnKeyboardCtrlKeyPressed(uint8_t key)
{
    if (key == (uint8_t)'W')
    {
        //这里写你要执行的操作（Ctrl + W 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'S')
    {
        //这里写你要执行的操作（Ctrl + S 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'A')
    {
        //这里写你要执行的操作（Ctrl + A 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'D')
    {
        //这里写你要执行的操作（Ctrl + D 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'Q')
    {
        //这里写你要执行的操作（Ctrl + Q 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'E')
    {
        //这里写你要执行的操作（Ctrl + E 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'R')
    {
        Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Normal;
        Chassis_RequestKeyboardAutoNormalSequence();
        return;
    }
    if (key == (uint8_t)'F')
    {
        //这里写你要执行的操作（Ctrl + F 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'G')
    {
        //这里写你要执行的操作（Ctrl + G 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'Z')
    {
        //这里写你要执行的操作（Ctrl + Z 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'X')
    {
        //这里写你要执行的操作（Ctrl + X 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'C')
    {
        Engineer_Mode.Chassis_Ctrl_Mode = CHASSIS_CTRL_MODE_Downstairs;
        return;
    }
    if (key == (uint8_t)'V')
    {
        //这里写你要执行的操作（Ctrl + V 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'B')
    {
        //这里写你要执行的操作（Ctrl + B 从 0->1 的瞬间触发）
    }
    Arm_Keyboard_ctrl_Manager(key);
}

void Referee_OnKeyboardShiftKeyPressed(uint8_t key)
{
    Arm_Keyboard_shift_Manager(key);
    if (key == (uint8_t)'W')
    {
        //这里写你要执行的操作（Shift + W 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'S')
    {
        //这里写你要执行的操作（Shift + S 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'A')
    {
        //这里写你要执行的操作（Shift + A 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'D')
    {
        //这里写你要执行的操作（Shift + D 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'Q')
    {
        //这里写你要执行的操作（Shift + Q 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'E')
    {
        //这里写你要执行的操作（Shift + E 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'R')
    {
        //这里写你要执行的操作（Shift + R 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'F')
    {
        //这里写你要执行的操作（Shift + F 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'G')
    {
        //这里写你要执行的操作（Shift + G 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'Z')
    {
        //这里写你要执行的操作（Shift + Z 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'X')
    {
        //这里写你要执行的操作（Shift + X 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'C')
    {
        //这里写你要执行的操作（Shift + C 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'V')
    {
        //这里写你要执行的操作（Shift + V 从 0->1 的瞬间触发）
    }
    if (key == (uint8_t)'B')
    {
        //这里写你要执行的操作（Shift + B 从 0->1 的瞬间触发）
    }
}

static void Referee_DispatchKeyboardKeyPressed(uint8_t key, uint8_t ctrl_pressed, uint8_t shift_pressed)
{
    uint8_t handled = 0U;

    if (ctrl_pressed != 0U) {
        Referee_OnKeyboardCtrlKeyPressed(key);
        handled = 1U;
    }
    if (shift_pressed != 0U) {
        Referee_OnKeyboardShiftKeyPressed(key);
        handled = 1U;
    }
    if (handled == 0U) {
        Referee_OnKeyboardKeyPressed(key, 0U);
    }
}

void Referee_KeyboardEdgeDetect(const keyboard_t *kb)
{
    static keyboard_t last_kb;
    static uint8_t inited = 0U;
    uint8_t ctrl_pressed = 0U;
    uint8_t shift_pressed = 0U;

    if (kb == NULL) {
        return;
    }

    ctrl_pressed = kb->key_code.bit.CTRL;
    shift_pressed = kb->key_code.bit.SHIFT;

    if (inited == 0U) {
        last_kb = *kb;
        inited = 1U;
        return;
    }

    if ((kb->key_code.bit.Q != 0U) && (last_kb.key_code.bit.Q == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'Q', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.E != 0U) && (last_kb.key_code.bit.E == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'E', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.R != 0U) && (last_kb.key_code.bit.R == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'R', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.F != 0U) && (last_kb.key_code.bit.F == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'F', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.G != 0U) && (last_kb.key_code.bit.G == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'G', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.Z != 0U) && (last_kb.key_code.bit.Z == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'Z', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.X != 0U) && (last_kb.key_code.bit.X == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'X', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.C != 0U) &&
        (last_kb.key_code.bit.C == 0U) &&
        (kb->key_code.bit.A == 0U) &&
        (kb->key_code.bit.D == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'C', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.V != 0U) && (last_kb.key_code.bit.V == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'V', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.B != 0U) && (last_kb.key_code.bit.B == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'B', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.W != 0U) && (last_kb.key_code.bit.W == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'W', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.S != 0U) && (last_kb.key_code.bit.S == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'S', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.A != 0U) && (last_kb.key_code.bit.A == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'A', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.D != 0U) && (last_kb.key_code.bit.D == 0U)) {
        Referee_DispatchKeyboardKeyPressed((uint8_t)'D', ctrl_pressed, shift_pressed);
    }
    if ((kb->key_code.bit.SHIFT != 0U) && (last_kb.key_code.bit.SHIFT == 0U)) {
        Referee_OnKeyboardShiftKeyPressed((uint8_t)0U);
    }
    if ((kb->key_code.bit.CTRL != 0U) && (last_kb.key_code.bit.CTRL == 0U)) {
        Referee_OnKeyboardCtrlKeyPressed((uint8_t)0U);
    }

    last_kb = *kb;
}
