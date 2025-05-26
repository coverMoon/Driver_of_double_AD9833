#include "AD9833_HAL.h"
#include <math.h>

// 频率换算因子: FREQ_REG_MAX / MCLK_Frequency
// MCLK 为 25MHz
const double freqScale = (double)FREQ_REG_MAX / 25000000.0;

// 影子控制寄存器，用于保存每个通道的控制寄存器状态
// 初始状态: B28=1 (28位频率写入), RESET=1 (芯片处于复位状态)
static uint16_t s_control_reg_cs1 = AD9833_CMD_CTRLREG | AD9833_CTRL_B28 | AD9833_CTRL_RESET;
static uint16_t s_control_reg_cs2 = AD9833_CMD_CTRLREG | AD9833_CTRL_B28 | AD9833_CTRL_RESET;

/**
 * @brief       向 AD9833 写入一个 16bit 的数据
 * @note        底层SPI发送函数
 * @param       hspi: 指向SPI外设句柄的指针
 * @param       choice: 片选参数 (CS1 或 CS2)
 * @param       TxData: 要发送的16位数据
 * @retval      无
 */
void AD9833_Write(SPI_HandleTypeDef* hspi, chipChose choice, uint16_t TxData)
{
    uint8_t data[2];
    // STM32 HAL SPI通常是MSB First发送，AD9833也是MSB First接收
    // 所以高字节在前，低字节在后
    data[0] = (uint8_t)((TxData >> 8) & 0xFF); // 高字节
    data[1] = (uint8_t)(TxData & 0xFF);        // 低字节

    if (choice == CS1)
    {
        AD9833_CS1_L();
        HAL_SPI_Transmit(hspi, data, 2, AD9833_SPI_TIMEOUT); // 使用传入的hspi句柄
        AD9833_CS1_H();
    }
    else if (choice == CS2)
    {
        AD9833_CS2_L();
        HAL_SPI_Transmit(hspi, data, 2, AD9833_SPI_TIMEOUT); // 使用传入的hspi句柄
        AD9833_CS2_H();
    }
}

/**
 * @brief     	获取指定通道的影子控制寄存器的指针
 * @param     	choice: 片选参数
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
 * @param       hspi: 指向SPI外设句柄的指针
 * @param     	status: 工作状态选择 (决定哪个通道的DAC关闭)
 * @retval    	无
 */
void AD9833_Init(SPI_HandleTypeDef* hspi, workStatus status)
{
    AD9833_CS1_H(); // 初始化时片选拉高
    AD9833_CS2_H();

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
    AD9833_Write(hspi, CS1, s_control_reg_cs1);
    AD9833_Write(hspi, CS2, s_control_reg_cs2);
}

/**
 * @brief     	设置输出波形类型并使芯片退出复位开始输出
 * @note      	此函数会修改影子控制寄存器并写入。
 * @param       hspi: 指向SPI外设句柄的指针
 * @param     	choice: 片选参数
 * @param       wave: 波形选择
 * @retval    	无
 */
void AD9833_SetWaveformAndStart(SPI_HandleTypeDef* hspi, chipChose choice, waveType wave)
{
    uint16_t* pCtrlReg = AD9833_GetShadowCtrlReg(choice);
    if (!pCtrlReg) return;

    // 1. 清除当前波形相关的控制位 (MODE, OPBITEN, DIV2)
    *pCtrlReg &= ~(AD9833_CTRL_MODE | AD9833_CTRL_OPBITEN | AD9833_CTRL_DIV2);

    // 2. 根据选择设置新的波形位
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

    // 3. 确保芯片退出复位状态以开始输出
    *pCtrlReg &= ~AD9833_CTRL_RESET; // RESET = 0

    // 4. 写入更新后的控制寄存器
    AD9833_Write(hspi, choice, *pCtrlReg);
}


/**
 * @brief     	向 AD9833 的指定相位寄存器写入一个12位的值
 * @note      	可直接在芯片工作过程中写入，实现相位可控
 * @param       hspi: 指向SPI外设句柄的指针
 * @param     	choice: 片选参数
 * @param       phase_reg_num: 相位寄存器编号 (0 或 1)
 * @param       phase: 要写入的相位值 (弧度, 0 到 2*PI)
 * @retval    	无
 */
void AD9833_PhaseSet(SPI_HandleTypeDef* hspi, chipChose choice, uint8_t phase_reg_num, double phase)
{
    uint16_t phase_data_raw;
    uint16_t phase_cmd;

    // 相位换算 (0 to 2*PI -> 0 to 4095)
    phase_data_raw = (uint16_t)(fmod(phase, 2.0 * PI) / (2.0 * PI) * 4096.0); // 使用fmod确保在0-2PI内
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
    AD9833_Write(hspi, choice, phase_cmd | phase_data_raw);
}

/**
 * @brief     	向 AD9833 的指定频率寄存器写入一个28位的值
 * @note      	可直接在芯片工作过程中写入，实现频率可控。
 *              需要两次16位的写操作。确保控制寄存器中的B28位已设为1。
 * @param       hspi: 指向SPI外设句柄的指针
 * @param     	choice: 片选参数
 * @param       freq_reg_num: 频率寄存器编号 (0 或 1)
 * @param       freq: 要写入的频率值 (Hz)
 * @retval    	无
 */
void AD9833_FreqSet(SPI_HandleTypeDef* hspi, chipChose choice, uint8_t freq_reg_num, double freq)
{
    uint32_t freq_data_raw;
    uint16_t freq_LSB, freq_MSB;
    uint16_t freq_cmd;

    // 频率换算
    if (freq < 0) 
		freq = 0;           // 频率不能为负
    if (freq > 12500000.0) 
		freq = 12500000.0;  // 最大频率限制

    freq_data_raw = (uint32_t)(freq * freqScale);
    freq_data_raw &= 0x0FFFFFFF; // 取28位

    freq_LSB = (uint16_t)(freq_data_raw & 0x3FFF);      // 低14位
    freq_MSB = (uint16_t)((freq_data_raw >> 14) & 0x3FFF); // 高14位

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

    AD9833_Write(hspi, choice, freq_cmd | freq_LSB); // 先写入低14位 (包含指令)
    AD9833_Write(hspi, choice, freq_cmd | freq_MSB); // 再写入高14位 (包含指令)
}

/**
 * @brief     	通过修改控制寄存器选择当前工作的频率寄存器
 * @param       hspi: 指向SPI外设句柄的指针
 * @param     	choice: 片选参数
 * @param       freq_reg_num: 要选择的频率寄存器编号 (0 或 1)
 * @retval    	无
 */
void AD9833_SelectFreqReg(SPI_HandleTypeDef* hspi, chipChose choice, uint8_t freq_reg_num)
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
    AD9833_Write(hspi, choice, *pCtrlReg);
}

/**
 * @brief     	通过修改控制寄存器选择当前工作的相位寄存器
 * @param       hspi: 指向SPI外设句柄的指针
 * @param     	choice: 片选参数
 * @param       phase_reg_num: 要选择的相位寄存器编号 (0 或 1)
 * @retval    	无
 */
void AD9833_SelectPhaseReg(SPI_HandleTypeDef* hspi, chipChose choice, uint8_t phase_reg_num)
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
    AD9833_Write(hspi, choice, *pCtrlReg);
}

/**
 * @brief     	控制AD9833的复位状态
 * @param       hspi: 指向SPI外设句柄的指针
 * @param     	choice: 片选参数
 * @param       reset_active: 1 使能复位, 0 取消复位
 * @retval    	无
 */
void AD9833_Reset(SPI_HandleTypeDef* hspi, chipChose choice, uint8_t reset_active)
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
    AD9833_Write(hspi, choice, *pCtrlReg);
}

/**
 * @brief     	控制AD9833的睡眠状态
 * @param       hspi: 指向SPI外设句柄的指针
 * @param     	choice: 片选参数
 * @param       sleep1_active: 1 使能SLEEP1 (MCLK关闭), 0 取消
 * @param       sleep12_active: 1 使能SLEEP12 (DAC关闭), 0 取消
 * @retval    	无
 */
void AD9833_Sleep(SPI_HandleTypeDef* hspi, chipChose choice, uint8_t sleep1_active, uint8_t sleep12_active)
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
    AD9833_Write(hspi, choice, *pCtrlReg);
}


/**
 * @brief     	AD9833初始化并开始输出
 * @note      	顶层封装函数。
 * @param     	AD_InitStruct: 输出初始化结构体, 其中包含了SPI句柄
 * @retval    	无
 */
void AD9833_Cmd(AD9833_InitTypedef *AD_InitStruct)
{
    // hspi空指针检查
	if (!AD_InitStruct || !AD_InitStruct->hspi) return;

    SPI_HandleTypeDef* hspi = AD_InitStruct->hspi;

    // 1. 初始化芯片并根据工作状态设置睡眠位
    AD9833_Init(hspi, AD_InitStruct->status);

    // 2. 配置每个活动的通道
    if (AD_InitStruct->status == CS1_SINGLE || AD_InitStruct->status == CS1_CS2_DOUBLE)
    {
        AD9833_SelectFreqReg(hspi, CS1, AD_InitStruct->AD_CS1.freqReg);
        AD9833_SelectPhaseReg(hspi, CS1, AD_InitStruct->AD_CS1.phaseReg);

        AD9833_FreqSet(hspi, CS1, AD_InitStruct->AD_CS1.freqReg, AD_InitStruct->AD_CS1.freq);
        AD9833_PhaseSet(hspi, CS1, AD_InitStruct->AD_CS1.phaseReg, AD_InitStruct->AD_CS1.phase);

        AD9833_SetWaveformAndStart(hspi, CS1, (waveType)AD_InitStruct->AD_CS1.wave);
    }

    if (AD_InitStruct->status == CS2_SINGLE || AD_InitStruct->status == CS1_CS2_DOUBLE)
    {
        AD9833_SelectFreqReg(hspi, CS2, AD_InitStruct->AD_CS2.freqReg);
        AD9833_SelectPhaseReg(hspi, CS2, AD_InitStruct->AD_CS2.phaseReg);

        AD9833_FreqSet(hspi, CS2, AD_InitStruct->AD_CS2.freqReg, AD_InitStruct->AD_CS2.freq);
        AD9833_PhaseSet(hspi, CS2, AD_InitStruct->AD_CS2.phaseReg, AD_InitStruct->AD_CS2.phase);
        AD9833_SetWaveformAndStart(hspi, CS2, (waveType)AD_InitStruct->AD_CS2.wave);
    }
}
