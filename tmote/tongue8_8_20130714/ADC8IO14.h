// $Id: CountMsg.h,v 1.1.1.1 2007/11/05 19:08:58 jpolastre Exp $

/*
 * Copyright (c) 2006 Moteiv Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached MOTEIV-LICENSE     
 * file. If you do not find these files, copies can be found at
 * http://www.moteiv.com/MOTEIV-LICENSE.txt and by emailing info@moteiv.com.
 */

/*
 * @author Cory Sharp <info@moteiv.com>
 */

#ifndef ADC8IO14_H
#define ADC8IO14_H
#include "stdio.h"

#define CHAN_NUM	8

typedef struct ADCMsg {
	uint8_t data[CHAN_NUM*CHAN_NUM];
	uint16_t count;
} ADCMsg_t;


#define	AM_ID_PRESSUREMATRIX  	31

/*replace TOSH_DATA_LENGTH in ./tinyos/moteiv/tos/lib/CC2420Radio/AM.h with size of ADCMsg*/

#ifdef DEFAULT_BAUDRATE
#undef DEFAULT_BAUDRATE
#endif

#define DEFAULT_BAUDRATE 115200



#endif
