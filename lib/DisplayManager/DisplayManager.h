#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "fishbot_display.h"
#include <GlobalData.h>

// 初始化屏幕（放在 setup 中调用）
void DisplayManager_Init();

// 刷新屏幕逻辑（放在 loop 中调用）
void DisplayManager_Update();

#endif // DISPLAY_MANAGER_H