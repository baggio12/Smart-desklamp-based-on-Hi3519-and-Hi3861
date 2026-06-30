#ifndef __YUANYI_BH1750FVI_H__
#define __YUANYI_BH1750FVI_H__

#include <stdint.h>

#define BH1750FVI_ADDR_H          0x5C
#define BH1750FVI_ADDR_L          0x23

#define BH1750FVI_POWER_DOWN      0x00
#define BH1750FVI_POWER_ON        0x01
#define BH1750FVI_RESET           0x07

#define BH1750FVI_CONT_H_RES_MODE 0x10
#define BH1750FVI_CONT_H_RES_MODE2 0x11
#define BH1750FVI_CONT_L_RES_MODE 0x13
#define BH1750FVI_ONE_TIME_H_RES_MODE 0x20
#define BH1750FVI_ONE_TIME_H_RES_MODE2 0x21
#define BH1750FVI_ONE_TIME_L_RES_MODE 0x23

unsigned int YuanyiBh1750Init(void);
unsigned int YuanyiBh1750ReadLight(float *light);
unsigned int YuanyiBh1750SetMode(unsigned int mode);
unsigned int YuanyiBh1750PowerDown(void);
unsigned int YuanyiBh1750PowerOn(void);

#endif
