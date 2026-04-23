#ifndef __HMC5883L_H__
#define __HMC5883L_H__
#include "main.h"
#include "math.h"

#define HMC5883L_ADDR 0x3c
#define HMC5883L_CRA 0x00
#define HMC5883L_CRB 0x01
#define HMC5883L_MR 0x02
#define HMC5883L_DOXMR 0x03
#define HMC5883L_DOXLR 0x04
#define HMC5883L_DOZMR 0x05
#define HMC5883L_DOZLR 0x06
#define HMC5883L_DOYMR 0x07
#define HMC5883L_DOYLR 0x08
#define HMC5883L_SR 0x09
#define HMC5883L_IRA 0x0A
#define HMC5883L_IRB 0x0B
#define HMC5883L_IRC 0x0C

typedef struct
{
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;
} HMC5883L_Calib_t;

HAL_StatusTypeDef HMC5883L_Calibrate(HMC5883L_Calib_t *calib,
                                     uint16_t sample_count,
                                     uint32_t sample_delay_ms);


HAL_StatusTypeDef HMC5883L_Write(uint8_t addr, uint8_t dat);
HAL_StatusTypeDef HMC5883L_Read(uint8_t addr, uint8_t *dat);
HAL_StatusTypeDef HMC5883L_Init(void);
HAL_StatusTypeDef HMC5883L_GetData(int16_t *x, int16_t *y, int16_t *z);

extern volatile int16_t hmc_x, hmc_y, hmc_z;
extern volatile float yaw_hmc;



#endif
