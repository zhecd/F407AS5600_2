#ifndef __MOTION_PLANNER_H__
#define __MOTION_PLANNER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ==============================================================
// 运动规划层 API 声明
// ==============================================================

/**
 * @brief 初始化运动规划器 (同步初始物理坐标)
 */
void Motion_Planner_Init(void);

/**
 * @brief  控制机械臂末端移动到指定的物理坐标 (空间直线运动)
 * @param  x            目标 X 坐标 (水平前伸方向，单位: mm)
 * @param  y            目标 Y 坐标 (水平侧边方向，单位: mm)
 * @param  z            目标 Z 坐标 (垂直高度方向，桌面为0，单位: mm)
 * @param  duration_ms  动作预期耗时 (单位: 毫秒)
 * @retval true: 指令生成并成功存入底层队列; false: 队列满或坐标无解
 */
bool Motion_Planner_MoveToXYZ(float x, float y, float z, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_PLANNER_H__ */