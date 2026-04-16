#include "bsp_tmc2209.h"

// TMC2209 专用的 CRC8 计算算法
static uint8_t TMC_CalcCRC8(uint8_t *data, uint8_t len) 
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t current_byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            current_byte >>= 1;
        }
    }
    return crc;
}

// 核心函数：向指定的 TMC2209 地址发送 32 位配置数据
static void TMC2209_WriteRegister(UART_HandleTypeDef *huart, uint8_t motor_addr, uint8_t reg_addr, uint32_t data) 
{
    uint8_t tx_buffer[8];

    tx_buffer[0] = 0x05; // 同步头 (Sync)
    tx_buffer[1] = motor_addr; // 硬件地址 (0, 1, 2)
    tx_buffer[2] = reg_addr | 0x80; // 寄存器地址 (最高位置 1 表示写入)
    
    // 填入 32 位数据 (MSB first)
    tx_buffer[3] = (data >> 24) & 0xFF;
    tx_buffer[4] = (data >> 16) & 0xFF;
    tx_buffer[5] = (data >> 8) & 0xFF;
    tx_buffer[6] = data & 0xFF;
    
    // 计算并填入 CRC8 校验码
    tx_buffer[7] = TMC_CalcCRC8(tx_buffer, 7);

    // 通过单线半双工串口发送
    HAL_UART_Transmit(huart, tx_buffer, 8, 100);
    
    // 延迟一小会儿，确保驱动有时间处理
    HAL_Delay(5); 
}

// ==============================================================
// 对外暴露的统一初始化函数
// ==============================================================
void BSP_TMC2209_InitAll(UART_HandleTypeDef *huart) 
{
    // 配置参数说明：
    // 1. IHOLD_IRUN (控制电流):
    //    IRUN (运行电流) = 16 (范围 0-31，16大概是最大电流的一半)
    //    IHOLD (保持电流) = 4  (电机停下时，电流降到极低，保持温凉)
    //    IHOLDDELAY = 1  (停下后多快开始降流)
    //    组合后的 32 位数据 (1<<16 | 16<<8 | 4) = 0x00011004
    uint32_t ihold_irun_val = 0x00011004;

    // 2. GCONF (全局配置):
    //    清零 en_spreadCycle 位，强制开启 StealthChop2 (绝对静音模式)
    //    内部 Sense 电阻启用 (根据你的硬件板子，通常是开启的)
    uint32_t gconf_val = 0x00000008; // 默认基本配置，保证 StealthChop 开启

    // 循环配置三个电机 (地址 0, 1, 2)
    for (uint8_t addr = 0; addr <= 2; addr++) 
    {
        // 1. 设置电流和休眠降流
        TMC2209_WriteRegister(huart, addr, TMC2209_REG_IHOLD_IRUN, ihold_irun_val);
        
        // 2. 确保开启 StealthChop 静音模式
        TMC2209_WriteRegister(huart, addr, TMC2209_REG_GCONF, gconf_val);
    }
}