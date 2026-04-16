#ifndef __MOTION_PLANNER_H__
#define __MOTION_PLANNER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ==============================================================
// 运动规划层 API 声明 (大脑皮层)
// ==============================================================

/**
 * @brief  初始化运动规划器
 * @note   由于目前没有正运动学(FK)和绝对编码器，必须在开机时告诉大脑
 * 机械臂当前所处的初始物理坐标。
 * @param  start_x 开机时机械臂末端的物理 X 坐标 (mm)
 * @param  start_y 开机时机械臂末端的物理 Y 坐标 (mm)
 * @param  start_z 开机时机械臂末端的物理 Z 坐标 (mm)
 */
void Motion_Planner_Init(float start_x, float start_y, float start_z);

/**
 * @brief  笛卡尔空间直线插补 (画绝对直线)
 * @param  target_x 目标 X 坐标 (mm)
 * @param  target_y 目标 Y 坐标 (mm)
 * @param  target_z 目标 Z 坐标 (mm)
 * @param  duration_ms 动作预期耗时 (ms)
 * @retval true: 指令全部成功压入底层队列; false: 队列满或无解
 */
bool Motion_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_PLANNER_H__ */