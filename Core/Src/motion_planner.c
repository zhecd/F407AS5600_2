#include "motion_planner.h"
#include "robotGeometry.h" // 引入逆向解算大脑
#include "motor_core.h"    // 引入底层环形缓冲区
#include "bsp_stepper.h"   // 引入底层物理位置用于初始化
#include <stdlib.h>        // 提供 abs() 绝对值函数

// 假设你的 TIM6 中断频率是 20kHz (50微秒触发一次)
// 那么 1 毫秒 = 20 个 tick
#define TICKS_PER_MS  20

// ==============================================================
// 【核心修复】：虚拟规划坐标 (大脑的记忆)
// ==============================================================
static int32_t planned_pos_m1 = 0;
static int32_t planned_pos_m2 = 0;
static int32_t planned_pos_m3 = 0;

void Motion_Planner_Init(void)
{
    // 开机时，大脑的记忆必须和底层的物理位置对齐
    planned_pos_m1 = Motor_M1.absolute_position;
    planned_pos_m2 = Motor_M2.absolute_position;
    planned_pos_m3 = Motor_M3.absolute_position;
}

bool Motion_Planner_MoveToXYZ(float x, float y, float z, uint32_t duration_ms) 
{
    RobotAngles target_angles;
    RobotMotorUnits target_units;
    MotionFrame_t frame;

    // 1. 调用 IK 代码：将空间坐标 (X,Y,Z) 转换为三个关节的物理角度
    RobotGeometry_CalculateAngles(x, y, z, &target_angles);

    // 2. 将角度转换为目标电机的绝对步数
    RobotGeometry_AnglesToMotorUnits(&target_angles, &target_units);

    // 3. 计算步数增量 (使用大脑记录的 planned_pos，而不是底层的 absolute_position)
    frame.delta_m1 = target_units.rotUnits - planned_pos_m1;
    frame.delta_m2 = target_units.lowUnits - planned_pos_m2;
    frame.delta_m3 = target_units.highUnits - planned_pos_m3;

    // 4. 将传入的毫秒时间转化为底层的中断节拍数
    frame.total_ticks = duration_ms * TICKS_PER_MS;

    // ==============================================================
    // 【核心保护】：DDA 饱和超速锁
    // ==============================================================
    // 找出三个轴里需要走最多步数的那个轴
    uint32_t max_delta = abs(frame.delta_m1);
    if (abs(frame.delta_m2) > max_delta) max_delta = abs(frame.delta_m2);
    if (abs(frame.delta_m3) > max_delta) max_delta = abs(frame.delta_m3);
    
    // 如果要求走的步数大于时间节拍数（超出了20kHz的物理极限）
    if (max_delta > frame.total_ticks) 
    {
        // 强制拉长时间，保护系统不崩溃 (加 10 个 tick 作为安全缓冲)
        frame.total_ticks = max_delta + 10; 
    }

    // 5. 压入底层无锁环形缓冲区
    if (Motor_Buffer_Push(&frame)) 
    {
        // 只有当成功存入仓库后，大脑才更新“未来终点”的记忆！
        planned_pos_m1 = target_units.rotUnits;
        planned_pos_m2 = target_units.lowUnits;
        planned_pos_m3 = target_units.highUnits;
        return true;
    }

    return false; // 仓库满了
}