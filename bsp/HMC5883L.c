#include "HMC5883L.h"
#include "i2c.h"

volatile int16_t hmc_x, hmc_y, hmc_z;
volatile float yaw_hmc;

// 我重写了HMC5883L的模块，把它从 软件I2C+私有库 的形式转变成了 硬件I2C+HAL库 的形式
// 函数返回值重写为HAL_StatusTypeDef 使函数可以返回执行状态
// 硬件地址宏定义在.h文件里，可右键跳转查看，硬件地址在寄存器手册里可以查看

/*
	参数一：硬件内 目标写入地址
	参数二：要写入的字节
*/

HAL_StatusTypeDef HMC5883L_Write(uint8_t addr, uint8_t dat)
{
	// Mem就是把 找硬件地址、找硬件内地址、写/读 三位一体的硬件I2C函数。
	
	return HAL_I2C_Mem_Write(&hi2c2,
                             HMC5883L_ADDR,
                             addr,						// 内部要操作的地址
                             I2C_MEMADD_SIZE_8BIT,		// 寄存器有多宽
                             &dat,						
                             1,							// 要操作几字节
                             10);
}

// 原来读取函数是直接返回读取值，更改成了返回执行状态，增加了一个指针参数，用于储存返回值

/*
	参数一：硬件内 目标读取地址
	参数二：返回值将要写入的变量的指针
*/

HAL_StatusTypeDef HMC5883L_Read(uint8_t addr, uint8_t *dat)
{
	    return HAL_I2C_Mem_Read(&hi2c2,
                            HMC5883L_ADDR,
                            addr,
                            I2C_MEMADD_SIZE_8BIT,
                            dat,
                            1,
                            10);
}

HAL_StatusTypeDef HMC5883L_ReadBytes(uint8_t addr, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            HMC5883L_ADDR,
                            addr,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            len,
                            10);
}

HAL_StatusTypeDef HMC5883L_Init()
{
	HAL_Delay(100);
	
	// 最大输出速率（75HZ）
	if (HMC5883L_Write(HMC5883L_CRA, 0x78) != HAL_OK)
        return HAL_ERROR;
	
	// 默认增益
	if (HMC5883L_Write(HMC5883L_CRB, 0x20) != HAL_OK)
        return HAL_ERROR;
	
	// 连续测量
	if (HMC5883L_Write(HMC5883L_MR, 0x00) != HAL_OK)
        return HAL_ERROR;

	HAL_Delay(20); 
	
	return HAL_OK;
}

HAL_StatusTypeDef HMC5883L_GetData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if (HMC5883L_ReadBytes(HMC5883L_DOXMR, buf, 6) != HAL_OK)
        return HAL_ERROR;

    // HMC5883L 连续寄存器顺序是:
    // 0x03 X_MSB
    // 0x04 X_LSB
    // 0x05 Z_MSB
    // 0x06 Z_LSB
    // 0x07 Y_MSB
    // 0x08 Y_LSB

    *x = (int16_t)((buf[0] << 8) | buf[1]);
    *z = (int16_t)((buf[2] << 8) | buf[3]);
    *y = (int16_t)((buf[4] << 8) | buf[5]);

    return HAL_OK;
}


//HAL_StatusTypeDef HMC5883L_GetData(int16_t *x, int16_t *y, int16_t *z)
//{
//	// 高位数据和低位数据，获取后拼起来
//	// 这里采用读六次单字节再拼起来的方法
//	// 更合理的是使用ReadByte函数一次性读取六个字节
//	// 可以参考mpu6050.c的ReadByte写法
//	
//	// 我重写了这个函数，把原来 直接在函数里修改指定变量 变成了 传入指针、修改指针指向的变量
//	// 还增加了状态返回值
//	uint8_t data_h, data_l;
//	if(HMC5883L_Read(HMC5883L_DOXMR, &data_h) != HAL_OK)
//		return HAL_ERROR;
//	if(HMC5883L_Read(HMC5883L_DOXLR, &data_l) != HAL_OK)
//		return HAL_ERROR;
//	*x = data_l | (data_h << 8);

//	if(HMC5883L_Read(HMC5883L_DOYMR, &data_h) != HAL_OK)
//		return HAL_ERROR;
//	if(HMC5883L_Read(HMC5883L_DOYLR, &data_l) != HAL_OK)
//		return HAL_ERROR;
//	*y = data_l | (data_h << 8);

//	if(HMC5883L_Read(HMC5883L_DOZMR, &data_h) != HAL_OK)
//		return HAL_ERROR;
//	if(HMC5883L_Read(HMC5883L_DOZLR, &data_l) != HAL_OK)
//		return HAL_ERROR;
//	*z = data_l | (data_h << 8);
//	
//	return HAL_OK;
//}


/*
 * 校准时请缓慢旋转模块/小车，尽量把各个姿态都扫到。
 * 你当前 HMC5883L_Init() 配的是 75Hz，所以 sample_delay_ms 建议 >= 15ms。
 * 常用参数：sample_count = 1000，sample_delay_ms = 20。
 */
HAL_StatusTypeDef HMC5883L_Calibrate(HMC5883L_Calib_t *calib,
                                     uint16_t sample_count,
                                     uint32_t sample_delay_ms)
{
    uint16_t collected;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
    int16_t min_z;
    int16_t max_z;

    float half_x;
    float half_y;
    float half_z;
    float ref_half;

    if ((calib == 0) || (sample_count == 0U))
    {
        return HAL_ERROR;
    }

    if (HMC5883L_GetData(&raw_x, &raw_y, &raw_z) != HAL_OK)
    {
        return HAL_ERROR;
    }

    min_x = raw_x;
    max_x = raw_x;
    min_y = raw_y;
    max_y = raw_y;
    min_z = raw_z;
    max_z = raw_z;

    for (collected = 1U; collected < sample_count; collected++)
    {
        HAL_Delay(sample_delay_ms);

        if (HMC5883L_GetData(&raw_x, &raw_y, &raw_z) != HAL_OK)
        {
            return HAL_ERROR;
        }

        if (raw_x < min_x) min_x = raw_x;
        if (raw_x > max_x) max_x = raw_x;

        if (raw_y < min_y) min_y = raw_y;
        if (raw_y > max_y) max_y = raw_y;

        if (raw_z < min_z) min_z = raw_z;
        if (raw_z > max_z) max_z = raw_z;
    }

    calib->offset_x = ((float)max_x + (float)min_x) * 0.5f;
    calib->offset_y = ((float)max_y + (float)min_y) * 0.5f;
    calib->offset_z = ((float)max_z + (float)min_z) * 0.5f;

    half_x = ((float)max_x - (float)min_x) * 0.5f;
    half_y = ((float)max_y - (float)min_y) * 0.5f;
    half_z = ((float)max_z - (float)min_z) * 0.5f;

    if ((half_x < 1.0f) || (half_y < 1.0f) || (half_z < 1.0f))
    {
        return HAL_ERROR;
    }

    ref_half = half_x;
    if (half_y > ref_half) ref_half = half_y;
    if (half_z > ref_half) ref_half = half_z;

    /*
     * 这里必须是“参考半径 / 当前半径”。
     * 这样量程偏小的轴会被放大到和最大轴一致。
     */
    calib->scale_x = ref_half / half_x;
    calib->scale_y = ref_half / half_y;
    calib->scale_z = ref_half / half_z;

    return HAL_OK;
}

/* 校准参数使用方式：
float cal_x = ((float)raw_x - calib.offset_x) * calib.scale_x;
float cal_y = ((float)raw_y - calib.offset_y) * calib.scale_y;
float cal_z = ((float)raw_z - calib.offset_z) * calib.scale_z;
float yaw_hmc = atan2f(cal_y, cal_x) * 57.2957795f;
*/




	


