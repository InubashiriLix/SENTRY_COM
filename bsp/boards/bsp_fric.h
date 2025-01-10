#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"
#define THIN_FRIC 1    //0 is thick   fric, 1 is thin firc

	#if THIN_FRIC
	#define FRIC_15 4700 //4800
	#define FRIC_18 5200 //5500
	//#define FRIC_30 7600 //7300 7800 7200
    #define FRIC_30 7700 //7300 7800 7200
		
	#else
	#define FRIC_15 3700
	#define FRIC_18 4250
	#define FRIC_30 6800
	#endif
#define FRIC_OFF 0
#define TAR_FRIC_15 14.4
#define TAR_FRIC_18 17.4
#define TAR_FRIC_30 28.5

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
