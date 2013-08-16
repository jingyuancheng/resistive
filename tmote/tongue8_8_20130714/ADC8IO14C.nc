// $Id: CountUartC.nc,v 1.1.1.1 2007/11/05 19:08:58 jpolastre Exp $

/*
 * Copyright (c) 2006 Moteiv Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached MOTEIV-LICENSE     
 * file. If you do not find these files, copies can be found at
 * http://www.moteiv.com/MOTEIV-LICENSE.txt and by emailing info@moteiv.com.
 */

/**
 * CountUart: Count to the leds and send it over the radio.
 *
 * @author Cory Sharp <info@moteiv.com>
 */

#include "./ADC8IO14.h"

configuration ADC8IO14C {
}
implementation {
	components Main;
	components ADC8IO14P;
	components new TimerMilliC();
	components GenericComm;
	components MSP430ADC12C;
	components MSP430GeneralIOC;

	Main.StdControl -> ADC8IO14P;
	Main.StdControl -> MSP430ADC12C;

	ADC8IO14P.Timer -> TimerMilliC;
	ADC8IO14P.SendMsg -> GenericComm.SendMsg[AM_ID_PRESSUREMATRIX];
	ADC8IO14P.ADC0->MSP430ADC12C.MSP430ADC12Single[unique("MSP430ADC12")];
	ADC8IO14P.ADC1->MSP430ADC12C.MSP430ADC12Single[unique("MSP430ADC12")];
	ADC8IO14P.ADC2->MSP430ADC12C.MSP430ADC12Single[unique("MSP430ADC12")];
	ADC8IO14P.ADC3->MSP430ADC12C.MSP430ADC12Single[unique("MSP430ADC12")];
	ADC8IO14P.ADC4->MSP430ADC12C.MSP430ADC12Single[unique("MSP430ADC12")];
	ADC8IO14P.ADC5->MSP430ADC12C.MSP430ADC12Single[unique("MSP430ADC12")];
	ADC8IO14P.ADC6->MSP430ADC12C.MSP430ADC12Single[unique("MSP430ADC12")];
	ADC8IO14P.ADC7->MSP430ADC12C.MSP430ADC12Single[unique("MSP430ADC12")];

	
	ADC8IO14P.Ctrl0->MSP430GeneralIOC.Port23;
	ADC8IO14P.Ctrl1->MSP430GeneralIOC.Port16;
	ADC8IO14P.Ctrl2->MSP430GeneralIOC.Port21;
	ADC8IO14P.Ctrl3->MSP430GeneralIOC.Port55;
	ADC8IO14P.Ctrl4->MSP430GeneralIOC.Port54;
	ADC8IO14P.Ctrl5->MSP430GeneralIOC.Port20;
	ADC8IO14P.Ctrl6->MSP430GeneralIOC.Port15;
	ADC8IO14P.Ctrl7->MSP430GeneralIOC.Port26;

	ADC8IO14P.LedUsr->MSP430GeneralIOC.Port56;
	
}
