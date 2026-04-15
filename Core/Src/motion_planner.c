#include "motion_planner.h"
#include "robotGeometry.h" 
#include "motor_core.h"    
#include "bsp_stepper.h"   
#include <stdlib.h>        
#include <math.h>          // 需要用到 sqrtf 等数学函数

#define TICKS_PER_MS  20

// 笛卡尔直线插补精度：每隔多少毫米切一刀 (推荐 0.5mm - 2.0mm)
// 值越小，直线越平滑，但 CPU 计算量越大
#define LINEAR_STEP_MM 1.0f 

// ==============================================================
// 大脑的记忆变量：不但要记住电机的步数，还要记住当前的笛卡尔坐标
// ==============================================================
static int32_t planned_pos_m1 = 0;
static int32_t planned_pos_m2 = 0;
static int32_t planned_pos_m3 = 0;

static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;

void Motion_Planner_Init(float start_x, float start_y, float start_z)
{
    // 同步笛卡尔空间坐标
    current_x = start_x;
    current_y = start_y;
    current_z = start_z;
    
    // 同步关节空间绝对步数
    planned_pos_m1 = Motor_M1.absolute_position;
    planned_pos_m2 = Motor_M2.absolute_position;
    planned_pos_m3 = Motor_M3.absolute_position;
}

bool Motion_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms) 
{
    // 1. 计算三维空间的总直线距离
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float dz = target_z - current_z;
    float distance = sqrtf(dx*dx + dy*dy + dz*dz);

    // 如果距离太短（比如不到 0.1mm），直接忽略，防止除以 0
    if (distance < 0.1f) return true;

    // 2. 决定要切成多少段 (Segments)
    uint32_t segments = (uint32_t)(distance / LINEAR_STEP_MM);
    if (segments == 0) segments = 1; // 至少得有一段

    // 3. 分配每段微小直线的时间 (Ticks)
    uint32_t total_ticks_needed = duration_ms * TICKS_PER_MS;
    uint32_t ticks_per_segment = total_ticks_needed / segments;
    if (ticks_per_segment == 0) ticks_per_segment = 1;

    // 4. ★ 核心插补循环 ★
    for (uint32_t i = 1; i <= segments; i++) 
    {
        // 4.1 利用参数方程计算第 i 个微小目标点的 (x, y, z)
        float t = (float)i / (float)segments; // t 的范围是 0.0 到 1.0
        float step_x = current_x + dx * t;
        float step_y = current_y + dy * t;
        float step_z = current_z + dz * t;

        // 4.2 将微小目标点送入逆运动学解算大脑
        RobotAngles target_angles;
        RobotMotorUnits target_units;
        MotionFrame_t frame;

        RobotGeometry_CalculateAngles(step_x, step_y, step_z, &target_angles);
        RobotGeometry_AnglesToMotorUnits(&target_angles, &target_units);

        // 4.3 计算这一小步的电机增量
        frame.delta_m1 = target_units.rotUnits - planned_pos_m1;
        frame.delta_m2 = target_units.lowUnits - planned_pos_m2;
        frame.delta_m3 = target_units.highUnits - planned_pos_m3;
        frame.total_ticks = ticks_per_segment;

        // 4.4 超速安全锁 (老规矩，保护硬件)
        uint32_t max_delta = abs(frame.delta_m1);
        if (abs(frame.delta_m2) > max_delta) max_delta = abs(frame.delta_m2);
        if (abs(frame.delta_m3) > max_delta) max_delta = abs(frame.delta_m3);
        
        if (max_delta > frame.total_ticks) {
            frame.total_ticks = max_delta + 5; // 强制降速
        }

        // 4.5 ★ 阻塞式压入缓冲区 ★
        // 如果仓库满了 (返回 false)，就一直在 while 里死循环等！
        // 等待底层的 TIM6 中断取走一帧，腾出空位后，Push 就会成功并跳出 while。
        while (!Motor_Buffer_Push(&frame)) 
        {
            // 如果以后上了 FreeRTOS，这里就替换成 osDelay(1) 避免死等占用CPU
            // 在裸机下，只需要空转即可，因为 TIM6 是硬件中断，会强行打断死循环去清空缓冲区
        }

        // 4.6 压入成功，更新大脑记忆
        planned_pos_m1 = target_units.rotUnits;
        planned_pos_m2 = target_units.lowUnits;
        planned_pos_m3 = target_units.highUnits;
    }

    // 5. 整条直线插补跑完，更新笛卡尔记忆坐标为最终目标点
    current_x = target_x;
    current_y = target_y;
    current_z = target_z;

    return true;
}