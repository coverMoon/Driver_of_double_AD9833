/**
  ******************************************************************************
  * @file           : AD9833_Soft_MSPM0.c
  * @brief          : AD9833 DDS波形发生器驱动库 (纯软件SPI, 适配TI MSPM0)
  * @author         : 秦泽宇
  * @version        : v1.2 (MSPM0 移植版)
  * @date           : 2025-07-18
  *
  ******************************************************************************
  * @attention
  *
  * 本驱动库由 STM32 HAL 版本移植而来，专为 TI MSPM0 系列单片机设计，
  * 使用 TI DriverLib 库函数进行底层GPIO操作。
  *
  * 完全通过软件模拟SPI时序进行通信，不依赖于硬件SPI外设，因此具有
  * 极高的引脚灵活性。
  *
  * 使用方法:
  * 1. 在主程序中包含 "AD9833_Soft_MSPM0.h" 头文件。
  * 2. 使用 TI SysConfig 或手动调用 DL_GPIO_init() 函数，将用于软件SPI
  * 的引脚 (SCLK, MOSI) 和片选 (CS) 配置为GPIO输出模式。
  * 3. 确保在 "AD9833_Soft_MSPM0.h" 文件顶部的用户配置区中，正确
  * 设置了引脚对应的 PORT 和 PIN_MASK 宏。
  * 4. 创建一个 `AD9833_InitTypedef` 结构体变量并填充所需参数。
  * 5. 调用 `AD9833_Cmd()` 或 `AD9833_Cmd_Sync()` 函数来初始化并启动芯片。
  *
  ******************************************************************************
  */


#include "AD9833_Soft_MSPM0.h"
#include <math.h>

// 频率换算因子: FREQ_REG_MAX / MCLK_Frequency
// MCLK 为 25MHz
const double freqScale = (double)FREQ_REG_MAX / 25000000.0;

// 影子控制寄存器，用于保存每个通道的控制寄存器状态
// 初始状态: B28=1 (28位频率写入), RESET=1 (芯片处于复位状态)
static uint16_t s_control_reg_cs1 = AD9833_CMD_CTRLREG | AD9833_CTRL_B28 | AD9833_CTRL_RESET;
static uint16_t s_control_reg_cs2 = AD9833_CMD_CTRLREG | AD9833_CTRL_B28 | AD9833_CTRL_RESET;

/**
 * @brief       通过软件模拟SPI发送一个16位数据
 * @param       TxData: 要发送的16位数据
 */
static void AD9833_Write_Software(uint16_t TxData)
{
    for (uint8_t i = 0; i < 16; i++)
    {
        // 准备数据
        if (TxData & 0x8000)
        {
            AD9833_MOSI_H();
        } else
        {
            AD9833_MOSI_L();
        }

        // 第一个边沿：时钟从高变低
        AD9833_SCLK_L();
        TxData <<= 1;
        // 第二个边沿：时钟从低变高
        AD9833_SCLK_H();
        // 锁存后移位
    }
}

/**
 * @brief       向 AD9833 写入一个 16bit 的数据
 * @note        底层软件SPI发送函数
 * @param       choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 *                  @arg CS_BOTH: 广播模式
 * @param       TxData: 要发送的16位数据
 * @retval      无
 */
void AD9833_Write(chipChose choice, const uint16_t TxData)
{
    if (choice == CS1)
    {
        AD9833_CS1_L();
        AD9833_Write_Software(TxData);
        AD9833_CS1_H();
    }
    else if (choice == CS2)
    {
        AD9833_CS2_L();
        AD9833_Write_Software(TxData);
        AD9833_CS2_H();
    }
    else if (choice == CS_BOTH)
    {
        AD9833_CS1_L();
        AD9833_CS2_L();
        AD9833_Write_Software(TxData);
        AD9833_CS1_H();
        AD9833_CS2_H();
    }
}

/**
 * @brief     	获取指定通道的影子控制寄存器的指针
 * @param     	choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 * @retval    	指向影子控制寄存器的指针，如果choice无效则返回NULL
 */
static uint16_t* AD9833_GetShadowCtrlReg(chipChose choice)
{
    if (choice == CS1)
    {
        return &s_control_reg_cs1;
    }
    else if (choice == CS2)
    {
        return &s_control_reg_cs2;
    }
    return NULL; // 无效选择
}

/**
 * @brief     	初始化 AD9833 片选线，并将芯片置于初始复位状态
 * @note      	仅初始化控制寄存器到复位和B28模式。
 *              实际的频率、相位、波形设置由其他函数完成。
 * @param     	status: 工作状态选择 (决定哪个通道的DAC关闭)
 *                  @arg CS1_SINGLE: 仅CS1工作，CS2的DAC关闭
 *                  @arg CS2_SINGLE: 仅CS2工作，CS1的DAC关闭
 *                  @arg CS1_CS2_DOUBLE: CS1和CS2都工作
 * @retval    	无
 */
void AD9833_Init(workStatus status)
{
    AD9833_CS1_H();     // 初始化时片选拉高
    AD9833_CS2_H();
    AD9833_SCLK_H();    // 确保时钟线初始为高

    // 初始化影子控制寄存器 (B28=1, RESET=1)
    s_control_reg_cs1 = AD9833_CMD_CTRLREG | AD9833_CTRL_B28 | AD9833_CTRL_RESET;
    s_control_reg_cs2 = AD9833_CMD_CTRLREG | AD9833_CTRL_B28 | AD9833_CTRL_RESET;

    switch(status)
    {
        case CS1_SINGLE:
            // CS1 正常复位, CS2 DAC关闭
            s_control_reg_cs2 |= AD9833_CTRL_SLEEP12; // 关闭CS2的DAC
            break;
        case CS2_SINGLE:
            // CS2 正常复位, CS1 DAC关闭
            s_control_reg_cs1 |= AD9833_CTRL_SLEEP12; // 关闭CS1的DAC
            break;
        case CS1_CS2_DOUBLE:
            // 两者都正常复位，DAC都工作 (SLEEP12默认为0)
            break;
        default:
            // 处理无效状态
            s_control_reg_cs1 |= AD9833_CTRL_SLEEP1 | AD9833_CTRL_SLEEP12;
            s_control_reg_cs2 |= AD9833_CTRL_SLEEP1 | AD9833_CTRL_SLEEP12;
            break;
    }

    // 将初始控制状态写入芯片
    AD9833_Write(CS1, s_control_reg_cs1);
    AD9833_Write(CS2, s_control_reg_cs2);
}

/**
 * @brief     	设置输出波形类型并使芯片退出复位开始输出
 * @note      	此函数会修改影子控制寄存器并写入。
 * @param     	choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 * @param       wave: 波形选择
 *                  @arg SINE_WAVE: 正弦波
 *                  @arg TRIANGLE_WAVE: 三角波
 *                  @arg SQUARE_WAVE: 方波
 * @retval    	无
 */
void AD9833_SetWaveformAndStart(chipChose choice, waveType wave)
{
    uint16_t* pCtrlReg = AD9833_GetShadowCtrlReg(choice);
    if (!pCtrlReg) return;

    // 清除当前波形相关的控制位 (MODE, OPBITEN, DIV2)
    *pCtrlReg &= ~(AD9833_CTRL_MODE | AD9833_CTRL_OPBITEN | AD9833_CTRL_DIV2);

    // 根据选择设置新的波形位
    switch(wave)
    {
        case SINE_WAVE:
            // OPBITEN = 0 (默认), MODE = 0 (默认)
            break;
        case TRIANGLE_WAVE:
            *pCtrlReg |= AD9833_CTRL_MODE;      // MODE = 1
            break;
        case SQUARE_WAVE:
            *pCtrlReg |= AD9833_CTRL_OPBITEN;   // OPBITEN = 1
            *pCtrlReg |= AD9833_CTRL_DIV2;      // DIV2 = 1 (输出MSB, 如果需要MSB/2, 则不设置此位或清除此位)
                                                // MODE位在OPBITEN=1时应为0
            break;
        default:
            // 默认为正弦波或不改变
            break;
    }

    // 确保芯片退出复位状态以开始输出
    *pCtrlReg &= ~AD9833_CTRL_RESET; // RESET = 0

    // 写入更新后的控制寄存器
    AD9833_Write(choice, *pCtrlReg);
}


/**
 * @brief     	向 AD9833 的指定相位寄存器写入一个12位的值
 * @note      	可直接在芯片工作过程中写入，实现相位可控
 * @param     	choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 * @param       phase_reg_num: 相位寄存器编号
 *                  @arg 0: 相位寄存器0
 *                  @arg 1: 相位寄存器1
 * @param       phase: 要写入的相位值 (角度，0到360度)
 * @retval    	无
 */
void AD9833_PhaseSet(chipChose choice, uint8_t phase_reg_num, double phase)
{
    uint16_t phase_cmd;

    // 相位换算 (0 to 360 -> 0 to 4095)
    uint16_t phase_data_raw = (uint16_t) (fmod(phase, 360.0) / 360.0 * 4096.0); // 使用fmod确保在0-360内
    phase_data_raw &= 0x0FFF; // 取低12位

    if (phase_reg_num == 0)
    {
        phase_cmd = AD9833_CMD_PHASE0REG;
    }
    else if (phase_reg_num == 1)
    {
        phase_cmd = AD9833_CMD_PHASE1REG;
    }
    else
    {
        return; // 无效的相位寄存器号
    }

    // 合并指令和数据
    AD9833_Write(choice, phase_cmd | phase_data_raw);
}

/**
 * @brief     	向 AD9833 的指定频率寄存器写入一个28位的值
 * @note      	可直接在芯片工作过程中写入，实现频率可控。
 *              需要两次16位的写操作。确保控制寄存器中的B28位已设为1。
 * @param     	choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 * @param       freq_reg_num: 频率寄存器编号 (0 或 1)
 *                  @arg 0: 频率寄存器 0
 *                  @arg 1: 频率寄存器 1
 * @param       freq: 要写入的频率值 (Hz)
 * @retval    	无
 */
void AD9833_FreqSet(chipChose choice, uint8_t freq_reg_num, double freq)
{
    uint16_t freq_cmd;

    // 频率换算
    if (freq < 0) 
		freq = 0;           // 频率不能为负
    if (freq > 12500000.0) 
		freq = 12500000.0;  // 最大频率限制

    uint32_t freq_data_raw = (uint32_t) (freq * freqScale);
    freq_data_raw &= 0x0FFFFFFF; // 取28位

    uint16_t freq_LSB = (uint16_t) (freq_data_raw & 0x3FFF);            // 低14位
    uint16_t freq_MSB = (uint16_t) ((freq_data_raw >> 14) & 0x3FFF);    // 高14位

    if (freq_reg_num == 0)
    {
        freq_cmd = AD9833_CMD_FREQ0REG;
    }
    else if (freq_reg_num == 1)
    {
        freq_cmd = AD9833_CMD_FREQ1REG;
    }
    else
    {
        return; // 无效的频率寄存器号
    }

    // 确保B28=1已在控制寄存器中设置 (通常在初始化时完成)
    // 写入频率时，AD9833会自动处理B28=1的情况，先收LSB再收MSB
    // 所以这里直接按顺序写入即可

    AD9833_Write(choice, freq_cmd | freq_LSB); // 先写入低14位 (包含指令)
    AD9833_Write(choice, freq_cmd | freq_MSB); // 再写入高14位 (包含指令)
}

/**
 * @brief     	通过修改控制寄存器选择当前工作的频率寄存器
 * @param     	choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 * @param       freq_reg_num: 要选择的频率寄存器编号 (0 或 1)
 *                  @arg 0: 频率寄存器 0
 *                  @arg 1: 频率寄存器 1
 * @retval    	无
 */
void AD9833_SelectFreqReg(chipChose choice, uint8_t freq_reg_num)
{
    uint16_t* pCtrlReg = AD9833_GetShadowCtrlReg(choice);
    if (!pCtrlReg) return;

    if (freq_reg_num == 0)
    {
        *pCtrlReg &= ~AD9833_CTRL_FSELECT; // FSELECT = 0
    }
    else
    {
        *pCtrlReg |= AD9833_CTRL_FSELECT;  // FSELECT = 1
    }
    AD9833_Write(choice, *pCtrlReg);
}

/**
 * @brief     	通过修改控制寄存器选择当前工作的相位寄存器
 * @param     	choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 * @param       phase_reg_num: 要选择的相位寄存器编号 (0 或 1)
 *                  @arg 0: 相位寄存器 0
 *                  @arg 1: 相位寄存器 1
 * @retval    	无
 */
void AD9833_SelectPhaseReg(chipChose choice, uint8_t phase_reg_num)
{
    uint16_t* pCtrlReg = AD9833_GetShadowCtrlReg(choice);
    if (!pCtrlReg) return;

    if (phase_reg_num == 0)
    {
        *pCtrlReg &= ~AD9833_CTRL_PSELECT; // PSELECT = 0
    }
    else
    {
        *pCtrlReg |= AD9833_CTRL_PSELECT;  // PSELECT = 1
    }
    AD9833_Write(choice, *pCtrlReg);
}

/**
 * @brief     	控制AD9833的复位状态
 * @param     	choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 * @param       reset_active:
 *                  @arg 1: 使能复位
 *                  @arg 0: 取消复位
 * @retval    	无
 */
void AD9833_Reset(chipChose choice, uint8_t reset_active)
{
    uint16_t* pCtrlReg = AD9833_GetShadowCtrlReg(choice);
    if (!pCtrlReg) return;

    if (reset_active)
    {
        *pCtrlReg |= AD9833_CTRL_RESET;
    }
    else
    {
        *pCtrlReg &= ~AD9833_CTRL_RESET;
    }
    AD9833_Write(choice, *pCtrlReg);
}

/**
 * @brief     	控制AD9833的睡眠状态
 * @param     	choice: 片选参数
 *                  @arg CS1: 片选1
 *                  @arg CS2: 片选2
 * @param       sleep1_active:
 *                  @arg 1: 使能SLEEP1 (MCLK关闭)
 *                  @arg 0: 取消
 * @param       sleep12_active:
 *                  @arg 1: 使能SLEEP12 (DAC关闭)
 *                  @arg 0: 取消
 * @retval    	无
 */
void AD9833_Sleep(chipChose choice, uint8_t sleep1_active, uint8_t sleep12_active)
{
    uint16_t* pCtrlReg = AD9833_GetShadowCtrlReg(choice);
    if (!pCtrlReg) return;

    if (sleep1_active)
    {
        *pCtrlReg |= AD9833_CTRL_SLEEP1;
    }
    else
    {
        *pCtrlReg &= ~AD9833_CTRL_SLEEP1;
    }

    if (sleep12_active)
    {
        *pCtrlReg |= AD9833_CTRL_SLEEP12;
    }
    else
    {
        *pCtrlReg &= ~AD9833_CTRL_SLEEP12;
    }
    AD9833_Write(choice, *pCtrlReg);
}

/**
 * @brief     	AD9833初始化并开始输出
 * @note      	顶层封装函数。
 * @param     	AD_InitStruct: 输出初始化结构体
 * @retval    	无
 */
void AD9833_Cmd(AD9833_InitTypedef *AD_InitStruct)
{
    // 初始化芯片并根据工作状态设置睡眠位
    AD9833_Init(AD_InitStruct->status);

    // 配置每个活动的通道
    if (AD_InitStruct->status == CS1_SINGLE || AD_InitStruct->status == CS1_CS2_DOUBLE)
    {
        AD9833_SelectFreqReg(CS1, AD_InitStruct->AD_CS1.freqReg);
        AD9833_SelectPhaseReg(CS1, AD_InitStruct->AD_CS1.phaseReg);

        AD9833_FreqSet(CS1, AD_InitStruct->AD_CS1.freqReg, AD_InitStruct->AD_CS1.freq);
        AD9833_PhaseSet(CS1, AD_InitStruct->AD_CS1.phaseReg, AD_InitStruct->AD_CS1.phase);

        AD9833_SetWaveformAndStart(CS1, (waveType)AD_InitStruct->AD_CS1.wave);
    }

    if (AD_InitStruct->status == CS2_SINGLE || AD_InitStruct->status == CS1_CS2_DOUBLE)
    {
        AD9833_SelectFreqReg(CS2, AD_InitStruct->AD_CS2.freqReg);
        AD9833_SelectPhaseReg(CS2, AD_InitStruct->AD_CS2.phaseReg);

        AD9833_FreqSet(CS2, AD_InitStruct->AD_CS2.freqReg, AD_InitStruct->AD_CS2.freq);
        AD9833_PhaseSet(CS2, AD_InitStruct->AD_CS2.phaseReg, AD_InitStruct->AD_CS2.phase);
        AD9833_SetWaveformAndStart(CS2, (waveType)AD_InitStruct->AD_CS2.wave);
    }
}

/**
 * @brief     AD9833初始化并以相位相干模式开始输出（双通道）
 * @note      使用广播模式实现两个通道的同步复位和同步启动，此模式下要求两个通道波形相同，且必须都使用0号寄存器
 * @param     AD_InitStruct: 输出初始化结构体, 必须包含两个通道的参数
 * @retval    无
 */
void AD9833_Cmd_Sync(AD9833_InitTypedef *AD_InitStruct)
{
    /* 同步复位 */
    // 通过广播模式，同时将两个芯片置于B28和RESET状态
    uint16_t reset_cmd = AD9833_CMD_CTRLREG | AD9833_CTRL_B28 | AD9833_CTRL_RESET;
    AD9833_Write(CS_BOTH, reset_cmd);

    /* 配置参数 (芯片仍处于复位状态) */
    // 单独配置每个通道的频率和相位

    // 通道 1
    AD9833_SelectFreqReg(CS1, AD_InitStruct->AD_CS1.freqReg);
    AD9833_FreqSet(CS1, AD_InitStruct->AD_CS1.freqReg, AD_InitStruct->AD_CS1.freq);
    AD9833_SelectPhaseReg(CS1, AD_InitStruct->AD_CS1.phaseReg);
    AD9833_PhaseSet(CS1, AD_InitStruct->AD_CS1.phaseReg, AD_InitStruct->AD_CS1.phase);
    // 通道 2
    AD9833_SelectFreqReg(CS2, AD_InitStruct->AD_CS2.freqReg);
    AD9833_FreqSet(CS2, AD_InitStruct->AD_CS2.freqReg, AD_InitStruct->AD_CS2.freq);
    AD9833_SelectPhaseReg(CS2, AD_InitStruct->AD_CS2.phaseReg);
    AD9833_PhaseSet(CS2, AD_InitStruct->AD_CS2.phaseReg, AD_InitStruct->AD_CS2.phase);

    /* 同步启动 */
    // 准备一个不带RESET位的启动命令
    // 注意：此模式下，假定两个通道使用相同的波形，以CS1的波形为准。
    uint16_t start_cmd = AD9833_CMD_CTRLREG | AD9833_CTRL_B28;
    switch(AD_InitStruct->AD_CS1.wave)
    {
        case TRIANGLE_WAVE:
            start_cmd |= AD9833_CTRL_MODE;
            break;
        case SQUARE_WAVE:
            start_cmd |= (AD9833_CTRL_OPBITEN | AD9833_CTRL_DIV2);
            break;
        case SINE_WAVE:
        default:
            // 正弦波，无需额外设置
            break;
    }

    // 通过广播模式，同时清除RESET位，让两个通道一起开始输出
    AD9833_Write(CS_BOTH, start_cmd);
}
