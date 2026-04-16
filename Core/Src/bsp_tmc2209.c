#include "bsp_tmc2209.h"
#include <string.h>

// 1. 保留你经过实战检验的 CRC8 计算函数
uint8_t TMC2209_CalcCRC8(uint8_t *datagram, uint8_t datagramLength)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < datagramLength; i++) {
        uint8_t currentByte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            currentByte >>= 1;
        }
    }
    return crc;
}

// 2. 带有 ORE 清除防死锁的安全发送函数
static void TMC2209_WriteRegister(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t reg_addr, uint32_t data) 
{
    uint8_t packet[8];

    packet[0] = 0x05;             // Sync
    packet[1] = motor_addr;       // 动态硬件地址
    packet[2] = reg_addr | 0x80;  // Write Bit
    packet[3] = (data >> 24) & 0xFF;
    packet[4] = (data >> 16) & 0xFF;
    packet[5] = (data >> 8) & 0xFF;
    packet[6] = data & 0xFF;
    packet[7] = TMC2209_CalcCRC8(packet, 7);

    // 发送数据
    HAL_UART_Transmit(huart, packet, 8, 100);

    // ★ 关键恢复：清除 TX/RX 物理短接导致的 ORE 溢出错误
    __HAL_UART_CLEAR_OREFLAG(huart); 
    volatile uint8_t dummy = huart->Instance->DR; 
    (void)dummy; 

    HAL_Delay(5);
}

// 3. 多轴一键初始化函数
void BSP_TMC2209_InitAll(UART_HandleTypeDef *huart) 
{
    // 【完美融合 GCONF】
    // 恢复旧代码的位 6 和位 0，同时保持位 7 (夺权) 
    // mstep_reg_select=1, pdn_disable=1, I_scale_analog=1
    uint32_t gconf_val = (1 << 7) | (1 << 6) | (1 << 0);

    // 【修改细分为 16 (极其重要！)】
    // 你旧代码里 mres=0 是 256 细分。如果用 256 细分，
    // 你的 motion_planner 会慢得像乌龟！这里必须改为 16 细分 (mres=4)。
    uint32_t chopconf_val = (1 << 28) | (4 << 24) | (1 << 15) | (3 << 7) | (5 << 4) | 3;

    // 【完美融合 电流】使用你旧代码成功过的电流值
    uint32_t ihold_irun_val = (6 << 16) | (24 << 8) | 12;

    // ★ 无敌遍历法：不管你有没有插跳线帽，地址 0 到 3 全发一遍！
    for (uint8_t addr = 0; addr <= 3; addr++) 
    {
        TMC2209_WriteRegister(huart, addr, 0x00, gconf_val);        // GCONF
        TMC2209_WriteRegister(huart, addr, 0x6C, chopconf_val);     // CHOPCONF
        TMC2209_WriteRegister(huart, addr, 0x10, ihold_irun_val);   // IHOLD_IRUN
    }
}