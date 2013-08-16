// $Id: CountUartP.nc,v 1.1.1.1 2007/11/05 19:08:58 jpolastre Exp $

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

module ADC8IO14P {
  provides interface StdControl;
  uses interface Timer2<TMilli> as Timer;
  uses interface SendMsg;
  uses interface MSP430ADC12Single as ADC0; //pin3 10-header
  uses interface MSP430ADC12Single as ADC1; //pin3 10-header
  uses interface MSP430ADC12Single as ADC2; //pin3 10-header
  uses interface MSP430ADC12Single as ADC3; //pin3 10-header
  uses interface MSP430ADC12Single as ADC4; //pin3 10-header
  uses interface MSP430ADC12Single as ADC5; //pin3 10-header
  uses interface MSP430ADC12Single as ADC6; //pin3 10-header
  uses interface MSP430ADC12Single as ADC7; //pin3 10-header  
  uses interface MSP430GeneralIO as Ctrl0;
  uses interface MSP430GeneralIO as Ctrl1;
  uses interface MSP430GeneralIO as Ctrl2;
  uses interface MSP430GeneralIO as Ctrl3;	//substractAverage flag
  uses interface MSP430GeneralIO as Ctrl4;	//autobias flag
  uses interface MSP430GeneralIO as Ctrl5;  //D3
  uses interface MSP430GeneralIO as Ctrl6;  //D2
  uses interface MSP430GeneralIO as Ctrl7;  //D1
  uses interface MSP430GeneralIO as LedUsr;  //D1
  
}
implementation
{
  TOS_Msg m_msg;  
  uint16_t m_count;
  bool m_sending;
  uint8_t countCtrl;

  command result_t StdControl.init() 
  {
  
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
    m_count = 0;
    m_sending = FALSE;
	
	body->data[0]=0x00;
	body->data[1]=0x11;
	body->data[2]=0x22;
	body->data[3]=0x33;
	body->data[4]=0x44;
	body->data[5]=0x55;
	body->data[6]=0x66;	
	body->data[7]=0x77;
	body->count = m_count;
	
    return SUCCESS;
  }

  command result_t StdControl.start() {
    
	call ADC0.bind(ADC12_SETTINGS(INPUT_CHANNEL_A2,
		REFERENCE_AVcc_AVss,
		SAMPLE_HOLD_256_CYCLES,
		SHT_SOURCE_ADC12OSC,
		SHT_CLOCK_DIV_8,
		SAMPCON_SOURCE_SMCLK,
		SAMPCON_CLOCK_DIV_4,
		REFVOLT_LEVEL_2_5));
	call ADC1.bind(ADC12_SETTINGS(INPUT_CHANNEL_A0,
		REFERENCE_AVcc_AVss,
		SAMPLE_HOLD_256_CYCLES,
		SHT_SOURCE_ADC12OSC,
		SHT_CLOCK_DIV_8,
		SAMPCON_SOURCE_SMCLK,
		SAMPCON_CLOCK_DIV_4,
		REFVOLT_LEVEL_2_5));
	call ADC2.bind(ADC12_SETTINGS(INPUT_CHANNEL_A6,
		REFERENCE_AVcc_AVss,
		SAMPLE_HOLD_256_CYCLES,
		SHT_SOURCE_ADC12OSC,
		SHT_CLOCK_DIV_8,
		SAMPCON_SOURCE_SMCLK,
		SAMPCON_CLOCK_DIV_4,
		REFVOLT_LEVEL_2_5));
	call ADC3.bind(ADC12_SETTINGS(INPUT_CHANNEL_A4,
		REFERENCE_AVcc_AVss,
		SAMPLE_HOLD_256_CYCLES,
		SHT_SOURCE_ADC12OSC,
		SHT_CLOCK_DIV_8,
		SAMPCON_SOURCE_SMCLK,
		SAMPCON_CLOCK_DIV_4,
		REFVOLT_LEVEL_2_5));
	call ADC4.bind(ADC12_SETTINGS(INPUT_CHANNEL_A5,
		REFERENCE_AVcc_AVss,
		SAMPLE_HOLD_256_CYCLES,
		SHT_SOURCE_ADC12OSC,
		SHT_CLOCK_DIV_8,
		SAMPCON_SOURCE_SMCLK,
		SAMPCON_CLOCK_DIV_4,
		REFVOLT_LEVEL_2_5));
	call ADC5.bind(ADC12_SETTINGS(INPUT_CHANNEL_A7,
		REFERENCE_AVcc_AVss,
		SAMPLE_HOLD_256_CYCLES,
		SHT_SOURCE_ADC12OSC,
		SHT_CLOCK_DIV_8,
		SAMPCON_SOURCE_SMCLK,
		SAMPCON_CLOCK_DIV_4,
		REFVOLT_LEVEL_2_5));
	call ADC6.bind(ADC12_SETTINGS(INPUT_CHANNEL_A1,
		REFERENCE_AVcc_AVss,
		SAMPLE_HOLD_256_CYCLES,
		SHT_SOURCE_ADC12OSC,
		SHT_CLOCK_DIV_8,
		SAMPCON_SOURCE_SMCLK,
		SAMPCON_CLOCK_DIV_4,
		REFVOLT_LEVEL_2_5));
	call ADC7.bind(ADC12_SETTINGS(INPUT_CHANNEL_A3,
		REFERENCE_AVcc_AVss,
		SAMPLE_HOLD_256_CYCLES,
		SHT_SOURCE_ADC12OSC,
		SHT_CLOCK_DIV_8,
		SAMPCON_SOURCE_SMCLK,
		SAMPCON_CLOCK_DIV_4,
		REFVOLT_LEVEL_2_5));
	
	call Ctrl0.makeOutput();
	call Ctrl1.makeOutput();
	call Ctrl2.makeOutput();
	call Ctrl3.makeOutput();
	call Ctrl4.makeOutput();
	call Ctrl5.makeOutput();
	call Ctrl6.makeOutput();
	call Ctrl7.makeOutput();
	call LedUsr.makeOutput(); 
	
atomic
{
	countCtrl = 0;
}

	call Timer.startPeriodic( 50 ); 
	return SUCCESS;
  }

  command result_t StdControl.stop() {
    return SUCCESS;
  }


void switchCtrl()
{
	//set output
atomic
{	
	if(countCtrl==0)
		call Ctrl0.setHigh();
	else
		call Ctrl0.setLow();

	if(countCtrl==1)
		call Ctrl1.setHigh();
	else
		call Ctrl1.setLow();
			
	if(countCtrl==2)
		call Ctrl2.setHigh();
	else
		call Ctrl2.setLow();

	if(countCtrl==3)
		call Ctrl3.setHigh();
	else
		call Ctrl3.setLow();

	if(countCtrl==4)
		call Ctrl4.setHigh();
	else
		call Ctrl4.setLow();

	if(countCtrl==5)
		call Ctrl5.setHigh();
	else
		call Ctrl5.setLow();

	if(countCtrl==6)
		call Ctrl6.setHigh();
	else
		call Ctrl6.setLow();

	if(countCtrl==7)
		call Ctrl7.setHigh();
	else
		call Ctrl7.setLow();
		
}
}

  event void Timer.fired() 
  {
  
//  call LedUsr.setLow();

	switchCtrl();
	call ADC0.getData();
  }


task void taskSendData()
{
	
atomic
{	
	if( call SendMsg.send( TOS_UART_ADDR, sizeof(ADCMsg_t), &m_msg ) == SUCCESS )	
	{
		m_sending = TRUE;
	}
}
}

async event result_t ADC0.dataReady(uint16_t data)
{
atomic
{
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
	body->data[countCtrl*CHAN_NUM]=(data>>4);
	call ADC1.getData();  
	return SUCCESS;
}
}


async event result_t ADC1.dataReady(uint16_t data)
{
atomic
{
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
	body->data[countCtrl*CHAN_NUM+1]=(data>>4);
	call ADC2.getData();  
	return SUCCESS;
}
}

async event result_t ADC2.dataReady(uint16_t data)
{
atomic
{
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
	body->data[countCtrl*CHAN_NUM+2]=(data>>4);
	call ADC3.getData();  
	return SUCCESS;
}
}
async event result_t ADC3.dataReady(uint16_t data)
{
atomic
{
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
	body->data[countCtrl*CHAN_NUM+3]=(data>>4);
	call ADC4.getData();  
	return SUCCESS;
}
}
async event result_t ADC4.dataReady(uint16_t data)
{
atomic
{
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
	body->data[countCtrl*CHAN_NUM+4]=(data>>4);
	call ADC5.getData();  
	return SUCCESS;
}
}
async event result_t ADC5.dataReady(uint16_t data)
{
atomic
{
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
	body->data[countCtrl*CHAN_NUM+5]=(data>>4);
	call ADC6.getData();  
	return SUCCESS;
}
}
async event result_t ADC6.dataReady(uint16_t data)
{
atomic
{
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
	body->data[countCtrl*CHAN_NUM+6]=(data>>4);
	call ADC7.getData();  
	return SUCCESS;
}
}

async event result_t ADC7.dataReady(uint16_t data)
{
atomic
{
	ADCMsg_t* body = (ADCMsg_t*)m_msg.data;
	body->data[countCtrl*CHAN_NUM+7]=(data>>4);
	countCtrl=(countCtrl+1)%CHAN_NUM;
	switchCtrl();
	if( countCtrl==0 && m_sending == FALSE )  
	{
		body->count=m_count++;
		body->count=0xFFFF;
	
		call LedUsr.setLow();
		post taskSendData();
	}
	else
	{
		call ADC0.getData();
	}
}	
	return SUCCESS;
}  
  
event result_t SendMsg.sendDone( TOS_MsgPtr msg, result_t success ) 
{
atomic
{
    m_sending = FALSE;
}
	call LedUsr.setHigh();
    return SUCCESS;
}

}
