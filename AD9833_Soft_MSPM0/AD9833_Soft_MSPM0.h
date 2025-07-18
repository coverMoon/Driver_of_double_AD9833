/*
 * AD9833_Soft_MSPM0.h
 * 
 * AD9833 DDS 波形发生器驱动 —— 适配 TI MSPM0G3507 (DriverLib)
 * --------------------------------------------------------------
 * 本头文件是基于原 STM32 HAL 版本 (AD9833_Soft.h) 的移植。
 * 主要变动:
 *   1. 去除 HAL 依赖, 改用 TI DriverLib (`dl_gpio.h`) 操作 GPIO.
 *   2. 保持原有 API 和宏定义接口不变, 方便直接复用 .c 文件.
 *   3. 将与 MCU 相关的端口/引脚配置集中到本文件顶部, 建议使用 SysConfig
 *      或手动 `DL_GPIO_init*()` 在 `main()` 中完成引脚复用/方向配置.
 * 
 */

 #ifndef _AD9833_SOFT_MSPM0_H_
#define _AD9833_SOFT_MSPM0_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <math.h>

/* -------------------------------------------------------------------------- */
/*                             软件SPI所需宏定义                               */
/* -------------------------------------------------------------------------- */
#define AD9833_SCLK_PORT        (GPIOB)
#define AD9833_SCLK_PIN_MASK    DL_GPIO_PIN_0

#define AD9833_MOSI_PORT        (GPIOB)
#define AD9833_MOSI_PIN_MASK    DL_GPIO_PIN_16

#define AD9833_CS1_PORT         (GPIOB)
#define AD9833_CS1_PIN_MASK     DL_GPIO_PIN_6

#define AD9833_CS2_PORT         (GPIOB)
#define AD9833_CS2_PIN_MASK     DL_GPIO_PIN_7

/* -------------------------------------------------------------------------- */
/*                   GPIO 操作宏 —— DriverLib 单周期写寄存器                */
/* -------------------------------------------------------------------------- */
#define AD9833_SCLK_H()     DL_GPIO_setPins(AD9833_SCLK_PORT, AD9833_SCLK_PIN_MASK)
#define AD9833_SCLK_L()     DL_GPIO_clearPins(AD9833_SCLK_PORT, AD9833_SCLK_PIN_MASK)

#define AD9833_MOSI_H()     DL_GPIO_setPins(AD9833_MOSI_PORT, AD9833_MOSI_PIN_MASK)
#define AD9833_MOSI_L()     DL_GPIO_clearPins(AD9833_MOSI_PORT, AD9833_MOSI_PIN_MASK)

#define AD9833_CS1_H()      DL_GPIO_setPins(AD9833_CS1_PORT, AD9833_CS1_PIN_MASK)
#define AD9833_CS1_L()      DL_GPIO_clearPins(AD9833_CS1_PORT, AD9833_CS1_PIN_MASK)

#define AD9833_CS2_H()      DL_GPIO_setPins(AD9833_CS2_PORT, AD9833_CS2_PIN_MASK)
#define AD9833_CS2_L()      DL_GPIO_clearPins(AD9833_CS2_PORT, AD9833_CS2_PIN_MASK)

/* -------------------------------------------------------------------------- */
/*                          与原库保持一致的宏定义                           */
/* -------------------------------------------------------------------------- */

#define FREQ_REG_MAX 268435456ULL  // AD9833 为28位频率寄存器, 使用ULL确保类型正确

#ifndef PI      // 防止重定义
#define PI           3.14159265358979323846
#endif

// AD9833 控制寄存器位宏定义 (Control Register Bits)
#define AD9833_CTRL_B28        (1U << 13) // 1: 28位频率字分两次写入; 0: 14位独立写入
#define AD9833_CTRL_HLB        (1U << 12) // 当B28=0时: 1: 写高14位; 0: 写低14位
#define AD9833_CTRL_FSELECT    (1U << 11) // 1: 使用FREQ1; 0: 使用FREQ0
#define AD9833_CTRL_PSELECT    (1U << 10) // 1: 使用PHASE1; 0: 使用PHASE0
#define AD9833_CTRL_RESET      (1U << 8)  // 1: 复位; 0: 正常工作
#define AD9833_CTRL_SLEEP1     (1U << 7)  // 1: MCLK关闭, NCO停止累加
#define AD9833_CTRL_SLEEP12    (1U << 6)  // 1: DAC电源关闭
#define AD9833_CTRL_OPBITEN    (1U << 5)  // 1: VOUT输出MSB(方波); 0: VOUT输出DAC模拟信号
#define AD9833_CTRL_DIV2       (1U << 3)  // 当OPBITEN=1时: 1: 输出MSB; 0: 输出MSB/2
#define AD9833_CTRL_MODE       (1U << 1)  // 当OPBITEN=0时: 1: 三角波; 0: 正弦波

// AD9833 指令前缀宏定义 (Command Prefixes for Register Writes)
#define AD9833_CMD_CTRLREG     (0x0000U) // 写入控制寄存器 (D15=0, D14=0)
#define AD9833_CMD_FREQ0REG    (0x4000U) // 写入FREQ0寄存器 (D15=0, D14=1)
#define AD9833_CMD_FREQ1REG    (0x8000U) // 写入FREQ1寄存器 (D15=1, D14=0)
#define AD9833_CMD_PHASE0REG   (0xC000U) // 写入PHASE0寄存器 (D15=1, D14=1, D13=0)
#define AD9833_CMD_PHASE1REG   (0xE000U) // 写入PHASE1寄存器 (D15=1, D14=1, D13=1)

/* 无需 SPI 超时, 软件 SPI 直接 bit‑bang */
#define AD9833_SPI_TIMEOUT     (2U)

/* ------------------------- 枚举 & 结构体 ------------------------ */

/**
 * @brief   工作状态选择
 *      @arg CS1_SINGLE: 仅CS1工作，CS2的DAC关闭
 *      @arg CS2_SINGLE: 仅CS2工作，CS1的DAC关闭
 *      @arg CS1_CS2_DOUBLE: CS1和CS2都工作
 */
typedef enum{
    CS1_SINGLE = 0,
    CS2_SINGLE = 1,
    CS1_CS2_DOUBLE = 2
} workStatus;

/**
 * @brief   片选选择 (用于函数参数，区分操作哪个芯片)
 *      @arg CS1: 片选1
 *      @arg CS2: 片选2
 *      @arg CS_BOTH: 同时选择，用于广播模式
 */
typedef enum{
    CS1 = 1,
    CS2 = 2,
    CS_BOTH = 3
} chipChose;

/**
 * @brief   波形选择
 *      @arg SINE_WAVE: 正弦波
 *      @arg TRIANGLE_WAVE: 三角波
 *      @arg SQUARE_WAVE：方波
 */
typedef enum{
    SINE_WAVE = 1,
    TRIANGLE_WAVE = 2,
    SQUARE_WAVE = 3
} waveType;

/**
  * @brief DDS初始化结构体
  * @note 用来初始化单个AD9833芯片的输出状态
  *     @arg wave: 波形种类 (使用 waveType 枚举)
  *     @arg freq: 频率 (Hz)
  *     @arg phase: 相位 (弧度, 0 到 360度)
  *     @arg freqReg: 使用的频率寄存器 (0 或 1)
  *     @arg phaseReg: 使用的相位寄存器 (0 或 1)
  */
typedef struct
{
    waveType wave;
    double   freq;
    double   phase;
    uint8_t  freqReg;
    uint8_t  phaseReg;
} DDS_InitTypedef;

/**
  * @brief AD9833初始化结构体，若只需要一路输出，另一通道可全部初始化为0，也可直接默认初始化
  *     @arg hspi: 指向SPI句柄的指针
  *     @arg status: 工作状态 (使用 workStatus 枚举)
  *     @arg AD_CS1: 通道一输出参数
  *     @arg AD_CS2: 通道二输出参数
  */
typedef struct
{
    workStatus      status;
    DDS_InitTypedef AD_CS1;
    DDS_InitTypedef AD_CS2;
} AD9833_InitTypedef;

/* ------------------------------- API 声明 -------------------------------- */
void AD9833_Init(workStatus status);
void AD9833_Write(chipChose choice, uint16_t TxData);
void AD9833_PhaseSet(chipChose choice, uint8_t phase_reg_num, double phase);
void AD9833_FreqSet(chipChose choice, uint8_t freq_reg_num, double freq);
void AD9833_SetWaveformAndStart(chipChose choice, waveType wave);
void AD9833_Cmd(AD9833_InitTypedef *AD_InitStruct);
void AD9833_SelectFreqReg(chipChose choice, uint8_t freq_reg_num);
void AD9833_SelectPhaseReg(chipChose choice, uint8_t phase_reg_num);
void AD9833_Reset(chipChose choice, uint8_t reset_active);
void AD9833_Sleep(chipChose choice, uint8_t sleep1_active, uint8_t sleep12_active);
void AD9833_Cmd_Sync(AD9833_InitTypedef *AD_InitStruct);

#endif /* _AD9833_SOFT_MSPM0_H_ */