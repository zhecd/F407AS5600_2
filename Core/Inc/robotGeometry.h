#ifndef ROBOT_GEOMETRY_H
#define ROBOT_GEOMETRY_H

#include "main.h"
#include <math.h>

// ================= 机械臂尺寸配置 =================

// 1. 骨架臂长 (mm)
// 因为夹爪始终平行，我们只需要解算到"手腕"位置
// 所以这里填写的仅仅是两个关节之间的物理连杆长度
#define LINK_1_LEN      140.0f  // 大臂长度
#define LINK_2_LEN      140.0f  // 小臂长度 (手腕关节到肘关节)

// 2. 始终平行的夹爪补偿 (mm)
// 这里的 R 是完全平行于地面的水平距离
// 这里的 Z 是完全垂直于地面的垂直距离
#define TOOL_OFFSET_R   45.0f   // 夹爪水平突出长度55，笔尖大约45
#define TOOL_OFFSET_Z   -40.0f  // 夹爪垂直高度差-40 (如果没有则填0)

// =========================================================
// [新增] 底座高度补偿 (Z轴偏移)
// =========================================================
// 请用尺子测量：从"桌面/纸面"到"大臂旋转轴中心"的垂直高度
// 如果你设置 Z=0，笔尖刚好碰到纸面，说明这个值设置对了
#define BASE_HEIGHT     140.0f   // <--- 请修改这个值 (单位: mm)

// ================= 传动与算法配置 =================

// 3. 传动参数
#define GEAR_RATIO       4.5f    // 减速比 9:2

// 4. 零点设置
#define OFFSET_ROT       0.0f
#define OFFSET_LOW       0.0f
#define OFFSET_HIGH      0.0f

// 5. 单位转换
#define UNITS_PER_DEGREE     (GEAR_RATIO * 10.0f)
#define RAD_TO_DEG(x)        ((x) * 57.2957795f)

// ================= 数据结构 =================

typedef struct {
    float rot;   // θ1: 底座
    float low;   // θ2: 大臂
    float high;  // θ3: 小臂
} RobotAngles;

typedef struct {
    int32_t rotUnits;
    int32_t lowUnits;
    int32_t highUnits;
} RobotMotorUnits;

// ================= 函数声明 =================

void RobotGeometry_Init(void);
void RobotGeometry_CalculateAngles(float x, float y, float z, RobotAngles* angles);
void RobotGeometry_AnglesToMotorUnits(RobotAngles* angles, RobotMotorUnits* units);

#endif // ROBOT_GEOMETRY_H