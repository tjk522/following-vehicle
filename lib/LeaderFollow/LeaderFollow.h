#ifndef LEADER_FOLLOW_H
#define LEADER_FOLLOW_H

#include <Arduino.h>

void LeaderFollow_Init();
void Task_LeaderFollow(void *pvParameters);

extern bool leader_follow_enabled;

#endif
