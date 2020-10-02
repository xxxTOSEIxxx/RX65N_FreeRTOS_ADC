/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2019 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include "task_function.h"
#include "platform.h"
#include "r_s12ad_rx_if.h"
#include "semphr.h"


#define USE_LED
#define ADC_CH				( 0 )


SemaphoreHandle_t			g_pCallbackSemaphorHandle;					// コールバック同期用セマフォ


void ACM1602_Abort(void);
void AdcCallback(void *pArgs);
void init_mtu0(void);


//********************************************************************
// ADCタスク
//********************************************************************
void Task_ADC(void * pvParameters)
{
	adc_err_t				eAdcRet = ADC_SUCCESS;
	adc_cfg_t				tAdcCfg;
	adc_ch_cfg_t			tChCfg;
	uint16_t				Data = 0x00;


#ifdef USE_LED
	PORTA.PDR.BIT.B1 = 1;
	PORTA.PODR.BIT.B1 = 0;
#endif	// #ifdef USE_LED

	// コールバック同期用セマフォ生成
	g_pCallbackSemaphorHandle = xSemaphoreCreateBinary();
	if (g_pCallbackSemaphorHandle == NULL)
	{
		printf("xSemaphoreCreateBinary Error.\n");
		vTaskDelete(NULL);
		return;
	}

	// A/D コンバーターの初期化
	tAdcCfg.resolution = ADC_RESOLUTION_12_BIT;				// A/D変換精度
	tAdcCfg.alignment = ADC_ALIGN_RIGHT;					// フォーマット
	tAdcCfg.add_cnt = ADC_ADD_OFF;							// 加算/平均モード
	tAdcCfg.clearing = ADC_CLEAR_AFTER_READ_OFF;			// A/D データレジスタ自動クリア有効 /無効
	tAdcCfg.trigger = ADC_TRIG_SYNC_TRG0AN;					// A/D変換開始トリガ
	tAdcCfg.trigger_groupb = ADC_TRIG_NONE;					// グループBのA/D変換開始トリガ
	tAdcCfg.trigger_groupc = ADC_TRIG_NONE;					// グループCのA/D変換開始トリガ
	tAdcCfg.priority = 3;									// S12AD 割込み優先順位
	tAdcCfg.priority_groupb = 0;							// S12GB 割込み優先順位
	tAdcCfg.priority_groupc = 0;							// S12GC 割込み優先順位
	tAdcCfg.temp_sensor = ADC_TEMP_SENSOR_NOT_AD_CONVERTED;	// 温度センサの使用有無
	tAdcCfg.add_temp_sensor = ADC_TEMP_SENSOR_ADD_OFF;		// 温度センサーの加算/平均モード
	eAdcRet = R_ADC_Open(ADC_CH, ADC_MODE_SS_ONE_CH, &tAdcCfg, AdcCallback);
	if (eAdcRet != ADC_SUCCESS)
	{
		printf("R_ADC_Open Error. [eAdcRet:%d]\n", eAdcRet);
		ACM1602_Abort();
	}

	R_ADC_PinSet_S12AD0();

	// A/D 変換を行うチャネルを設定
	tChCfg.chan_mask = ADC_MASK_CH0;						// 使用するチャンネル
	tChCfg.chan_mask_groupb = ADC_MASK_GROUPB_OFF;			// グループBで使用するチャンネル
	tChCfg.chan_mask_groupc = ADC_MASK_GROUPC_OFF;			// グループCで使用するチャンネル
	tChCfg.priority_groupa = ADC_GRPA_PRIORITY_OFF;			// グループ優先制御動作
	tChCfg.add_mask = ADC_MASK_ADD_OFF;						// 加算モードを使用するチャンネル
	tChCfg.diag_method = ADC_DIAG_OFF;						// 自己診断モード
	tChCfg.anex_enable = false;								// 拡張アナログ入力(ANEX1)の使用有無
	tChCfg.sample_hold_mask = ADC_MASK_SAMPLE_HOLD_OFF;		// サンプル&ホールド回路を使用するチャンネル
	tChCfg.sample_hold_states = ADC_SST_SH_CNT_DEFAULT;		// サンプル時間
	eAdcRet = R_ADC_Control( ADC_CH, ADC_CMD_ENABLE_CHANS, &tChCfg);
	if (eAdcRet != ADC_SUCCESS)
	{
		printf("R_ADC_Control(ADC_CMD_ENABLE_CHANS) Error. [eAdcRet:%d]\n", eAdcRet);
		ACM1602_Abort();
	}

	init_mtu0();


	eAdcRet = R_ADC_Control(ADC_CH, ADC_CMD_ENABLE_TRIG, FIT_NO_PTR);
	if (eAdcRet != ADC_SUCCESS)
	{
		printf("R_ADC_Control(ADC_CMD_ENABLE_TRIG) Error. [eAdcRet:%d]\n", eAdcRet);
		ACM1602_Abort();
	}

	while(1)
	{
		// コールバック応答待ち
		xSemaphoreTake(g_pCallbackSemaphorHandle, portMAX_DELAY);

		eAdcRet = R_ADC_Read(ADC_CH, ADC_REG_CH0, &Data);
		if (eAdcRet != ADC_SUCCESS)
		{
			printf("R_ADC_Read Error. [eAdcRet:%d]\n", eAdcRet);
		}
		else
		{
			printf("Data:%d\n", Data);
		}
	}
}


//*******************************************************
// 異常終了
//*******************************************************
void ACM1602_Abort(void)
{
	printf("*** ACM1602_Abort! ***\n");

	while(1)
	{
#ifdef USE_LED
		PORTA.PODR.BIT.B1 = 1;
		vTaskDelay(100);
		PORTA.PODR.BIT.B1 = 0;
		vTaskDelay(100);
		PORTA.PODR.BIT.B1 = 1;
		vTaskDelay(100);
		PORTA.PODR.BIT.B1 = 0;
		vTaskDelay(100);
		PORTA.PODR.BIT.B1 = 1;
		vTaskDelay(100);
		PORTA.PODR.BIT.B1 = 0;
		vTaskDelay(500);
#endif	// #ifdef USE_LED
	}
}





//********************************************************************
// ADCコールバック
//********************************************************************
void AdcCallback(void *pArgs)
{
	BaseType_t 							bHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(g_pCallbackSemaphorHandle,&bHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(bHigherPriorityTaskWoken);
}





//********************************************************************
// MTU0設定
//********************************************************************
void init_mtu0(void)
{
    SYSTEM.PRCR.WORD = 0xA502;
    MSTP(MTU0) = 0;             // Release from module stop state.
    SYSTEM.PRCR.WORD = 0xA500;

    MTU.TSTRA.BYTE = 0x00;      // Stop MTU counters

    /* Timer Control Reg (TCR)
     * CCLR[2:0] = 001: TCNT cleared by TGRA compare match/input capture
     * CKEG[1:0] = 00 : Count at rising edge
     * TPSC[2:0] = 011: Time Prescaler Select; Internal clock: counts on PCLKA/64
     */
    MTU0.TCR.BYTE = 0x23;

    MTU0.TIORH.BYTE = 0x00;     // IO control A/B output prohibited
    MTU0.TIORL.BYTE = 0x00;     // IO control C/D output prohibited

    MTU0.TMDR1.BYTE = 0x00;     // Normal mode

    MTU0.TCNT = 0;              // Clear timer counter
    MTU0.TGRA = 60000;          // Set the compare match count value

    MTU0.TIER.BIT.TTGE = 1;     // Trigger ADC on TRGA compare match (TRGA0N sync trigger)

    /* Enable the TGIA0 (TGRA input capture/compare match) interrupt which will
     * cause the mtu0_tgia0_isr() ISR below to be called each time a TGRA compare
     * match occurs. Note that this is not required to periodically start the ADC
     * conversion (that occurs because MTU0.TIER.BIT.TTGE is set above whenever a
     * compare match occurs).
     */
    IR(MTU0,TGIA0) = 0;         // Clear the interrupt status flag
    IPR(MTU0,TGIA0) = 9;        // Set the interrupt source priority
    R_BSP_InterruptRequestEnable(VECT(MTU0,TGIA0));        // Enable the interrupt request
    MTU0.TIER.BIT.TGIEA = 1;    // Enable TGIA interrupts

    MTU.TSTRA.BYTE = 0x01;      // Start counting on MTU0
} /* End of function init_mtu0() */


//********************************************************************
// MTU0割込み処理
//********************************************************************
R_BSP_PRAGMA_STATIC_INTERRUPT (mtu0_tgia0_isr, VECT(MTU0,TGIA0))
R_BSP_ATTRIB_STATIC_INTERRUPT void mtu0_tgia0_isr(void)
{
#ifdef USE_LED
    static uint16_t cnt = 0;


    if (((cnt++) % 10) == 0)
    {
		PORTA.PODR.BIT.B1 ^= 1;
    }
#endif	// #ifdef USE_LED
}



