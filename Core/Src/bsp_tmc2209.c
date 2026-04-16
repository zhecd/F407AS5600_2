/**
  * @file    bsp_tmc2209.c
  * @brief   TMC2209 核心逻辑实现
  */

#include "bsp_tmc2209.h"

// 内部使用的寄存器地址 (不对外暴露，保护封装性)
#define REG_GCONF       0x00
#define REG_IHOLD_IRUN  0x10
#define REG_CHOPCONF    0x6C

// ==============================================================
// 第一层：纯底层物理传输层 (Private)
// ==============================================================

static uint8_t CalcCRC8(uint8_t *datagram, uint8_t len) 
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
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

static void WriteReg(UART_HandleTypeDef *huart, uint8_t addr, uint8_t reg, uint32_t data) 
{
    uint8_t packet[8];
    packet[0] = 0x05;             
    packet[1] = addr;             
    packet[2] = reg | 0x80;       
    packet[3] = (data >> 24) & 0xFF;
    packet[4] = (data >> 16) & 0xFF;
    packet[5] = (data >> 8) & 0xFF;
    packet[6] = data & 0xFF;
    packet[7] = CalcCRC8(packet, 7);

    HAL_UART_Transmit(huart, packet, 8, 100);

    // 核心防死锁机制：清除 TX 物理短接 RX 造成的 ORE 溢出
    __HAL_UART_CLEAR_OREFLAG(huart); 
    volatile uint8_t dummy = huart->Instance->DR; 
    (void)dummy; 

    HAL_Delay(5);
}

// ==============================================================
// 第二层：人类逻辑到寄存器逻辑的翻译层 (Private)
// ==============================================================

// 将直观的细分数 (1~256) 翻译为 TMC 底层的 MRES 寄存器值 (8~0)
static uint32_t ConvertMicrostepsToMRES(uint16_t microsteps)
{
    switch(microsteps) {
        case 256: return 0;
        case 128: return 1;
        case 64:  return 2;
        case 32:  return 3;
        case 16:  return 4;
        case 8:   return 5;
        case 4:   return 6;
        case 2:   return 7;
        case 1:   return 8;
        default:  return 4; // 容错处理：默认返回 16 细分
    }
}

// ==============================================================
// 第三层：面向用户的极简 API 接口层 (Public)
// ==============================================================

void BSP_TMC2209_ConfigNode(UART_HandleTypeDef *huart, uint8_t node_addr, uint16_t microsteps, uint8_t irun, uint8_t ihold)
{
    // 1. GCONF (软件夺权 & 参考电压设置)
    // Bit7=1(软件细分), Bit6=1(防休眠), Bit0=1(启用模拟比例)
    uint32_t gconf = (1 << 7) | (1 << 6) | (1 << 0);
    WriteReg(huart, node_addr, REG_GCONF, gconf);

    // 2. CHOPCONF (静音斩波与细分)
    // 基础值 0x10000053，插入计算好的 MRES 细分值，并开启 256 平滑插值(Bit28)
    uint32_t mres = ConvertMicrostepsToMRES(microsteps);
    uint32_t chopconf = 0x10000053 | (1 << 28) | (mres << 24);
    WriteReg(huart, node_addr, REG_CHOPCONF, chopconf);

    // 3. IHOLD_IRUN (安全无死角电流配置)
    // 使用掩码 & 0x1F 防止用户传入大于 31 的数值导致寄存器位错乱
    uint32_t ihold_irun = (6 << 16) | ((irun & 0x1F) << 8) | (ihold & 0x1F);
    WriteReg(huart, node_addr, REG_IHOLD_IRUN, ihold_irun);
}

void BSP_TMC2209_InitBus(UART_HandleTypeDef *huart) 
{
    // 广播初始化：不论跳线帽怎么插，地址 0~3 全部注入默认的高性能参数！
    // 默认配置：16 细分，运行电流 24，保持电流 12
    for (uint8_t addr = 0; addr <= 3; addr++) 
    {
        BSP_TMC2209_ConfigNode(huart, addr, 16, TMC_IRUN_HIGH, TMC_IHOLD_STRONG);
    }
}