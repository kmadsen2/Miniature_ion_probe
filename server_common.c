#include <stdio.h>
#include "btstack.h"
#include "hardware/adc.h"
#include "ad5940.h"
#include "temp_sensor.h"
#include "server_common.h"
#define APP_AD_FLAGS 0x06

//BT Functions
static uint8_t adv_data[] = {
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x17, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0', ':', '0', '0',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};
static const uint8_t adv_data_len = sizeof(adv_data);

int le_notification_enabled;
hci_con_handle_t con_handle;
uint32_t current_temp[Buff_Size];

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;
    if (packet_type != HCI_EVENT_PACKET) return;
    //if (read_count < 30) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch(event_type){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // setup advertisements
            uint16_t adv_int_min = 800;
            uint16_t adv_int_max = 800;
            uint8_t adv_type = 0;
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            assert(adv_data_len <= 31); // ble limitation
            gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
            gap_advertisements_enable(1);

            poll_temp();

            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            le_notification_enabled = 0;
            break;
        case ATT_EVENT_CAN_SEND_NOW:
            printf("notify");
            att_server_notify(con_handle, ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE, (uint8_t*)&TX_Buffer, sizeof(TX_Buffer));
            break;
        default:
            break;
    }
}

uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE){
        return att_read_callback_handle_blob((const uint8_t *)&TX_Buffer, sizeof(TX_Buffer), offset, buffer, buffer_size);
    }
    return 0;
}

int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);
    
    if (att_handle != ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE) return 0;
    le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    con_handle = connection_handle;
    if (le_notification_enabled) {
        att_server_request_can_send_now_event(con_handle);
    }
    return 0;
}

void poll_temp(void) {
    
    return;
    printf("test");
    adc_select_input(ADC_CHANNEL_TEMPSENSOR);
    uint32_t raw32 = adc_read();
    const uint32_t bits = 12;

    // Scale raw reading to 16 bit value using a Taylor expansion (for 8 <= bits <= 16)
    uint16_t raw16 = raw32 << (16 - bits) | raw32 >> (2 * bits - 16);

    // ref https://github.com/raspberrypi/pico-micropython-examples/blob/master/adc/temperature.py
    const float conversion_factor = 3.3 / (65535);
    float reading = raw16 * conversion_factor;
    
    // The temperature sensor measures the Vbe voltage of a biased bipolar diode, connected to the fifth ADC channel
    // Typically, Vbe = 0.706V at 27 degrees C, with a slope of -1.721mV (0.001721) per degree. 
    float deg_c = 27 - (reading - 0.706) / 0.001721;
    //current_temp = deg_c * 100;
    
    
    for (int i=0; i<Buff_Size; i++){
        current_temp[i] = 123456;
    }
    

    read_count++;
    printf("Write temp %.2f degc\n", deg_c);
    
 }


//AD5941 Functions
 AppAMPCfg_Type AppAMPCfg = 
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,
  .FifoThresh = 5,              /* Number of points for FIFO */
  
  .SysClkFreq = 16000000.0,
  .WuptClkFreq = 32000.0,
  .AdcClkFreq = 16000000.0,
  .AmpODR = 1.0,                /* Sample time in seconds. I.e. every 5 seconds make a measurement */
  .NumOfData = -1,
  .RcalVal = 10000.0,           /* RCAL = 10kOhm */
  .PwrMod = AFEPWR_LP,
  .AMPInited = bFALSE,
  .StopRequired = bFALSE,
  
  /* LPTIA Configure */
  .ExtRtia = bFALSE,            /* Set to true if using external RTIA */
  .LptiaRtiaSel = LPTIARTIA_1K, /* COnfigure RTIA */
  .LpTiaRf = LPTIARF_1M,        /* Configure LPF resistor */
  .LpTiaRl = LPTIARLOAD_100R,
  .ReDoRtiaCal = bTRUE,
  .RtiaCalValue = 0,
	.ExtRtiaVal = 0,
  
/*LPDAC Configure */
  .Vzero = 1100,                /* Sets voltage on SE0 and LPTIA */
  .SensorBias = 500,            /* Sets voltage between RE0 and SE0 */
  
/* ADC Configure*/
  .ADCPgaGain = ADCPGA_1P5,
  .ADCSinc3Osr = ADCSINC3OSR_4,
  .ADCSinc2Osr = ADCSINC2OSR_22,
  .DataFifoSrc = FIFOSRC_SINC2NOTCH,
  .ADCRefVolt = 1.8162,			/* Measure voltage on ADCRefVolt pin and enter here*/
};

/**
   This function is provided for upper controllers that want to change 
   application parameters specially for user defined parameters.
*/
AD5940Err AppAMPGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppAMPCfg_Type**)pCfg = &AppAMPCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

AD5940Err AppAMPCtrl(int32_t AmpCtrl, void *pPara)
{
  switch (AmpCtrl)
  {
    case AMPCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;

      AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
      if(AppAMPCfg.AMPInited == bFALSE)
        return AD5940ERR_APPERROR;
      /* Start it */
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.SeqxSleepTime[SEQID_0] = 4-1;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(AppAMPCfg.WuptClkFreq*AppAMPCfg.AmpODR)-4-1; 
      AD5940_WUPTCfg(&wupt_cfg);
      
      AppAMPCfg.FifoDataCount = 0;  /* restart */
      break;
    }
    case AMPCTRL_STOPNOW:
    {
      AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
      /* Start Wupt right now */
      AD5940_WUPTCtrl(bFALSE);
      /* There is chance this operation will fail because sequencer could put AFE back 
        to hibernate mode just after waking up. Use STOPSYNC is better. */
      AD5940_WUPTCtrl(bFALSE);
      break;
    }
    case AMPCTRL_STOPSYNC:
    {
      AppAMPCfg.StopRequired = bTRUE;
      break;
    }
    case AMPCTRL_SHUTDOWN:
    {
      AppAMPCtrl(AMPCTRL_STOPNOW, 0);  /* Stop the measurement if it's running. */
      /* Turn off LPloop related blocks which are not controlled automatically by sleep operation */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* Enter Hibernate */
    }
    break;
    default:
    break;
  }
  return AD5940ERR_OK;
}

/* Generate init sequence */
static AD5940Err AppAMPSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lp_loop;
  DSPCfg_Type dsp_cfg;
  SWMatrixCfg_Type sw_cfg;
  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  //AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */

  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bTRUE;
  aferef_cfg.Lp1V8BuffEn = bTRUE;
  /* LP reference control - turn off them to save power*/
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	

	lp_loop.LpDacCfg.LpdacSel = LPDAC0;
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN;
  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_loop.LpDacCfg.DataRst = bFALSE;
  lp_loop.LpDacCfg.PowerEn = bTRUE;
  lp_loop.LpDacCfg.DacData6Bit = (uint32_t)((AppAMPCfg.Vzero-200)/DAC6BITVOLT_1LSB);
	lp_loop.LpDacCfg.DacData12Bit =(int32_t)((AppAMPCfg.SensorBias)/DAC12BITVOLT_1LSB) + lp_loop.LpDacCfg.DacData6Bit*64;
	if(lp_loop.LpDacCfg.DacData12Bit>lp_loop.LpDacCfg.DacData6Bit*64)
		lp_loop.LpDacCfg.DacData12Bit--;
	lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaRf = AppAMPCfg.LpTiaRf;
  lp_loop.LpAmpCfg.LpTiaRload = AppAMPCfg.LpTiaRl;
  if(AppAMPCfg.ExtRtia == bTRUE)
  {
    lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(9)|LPTIASW(2)|LPTIASW(4)|LPTIASW(5)|LPTIASW(12)|LPTIASW(13); 
  }else
  {
    lp_loop.LpAmpCfg.LpTiaRtia = AppAMPCfg.LptiaRtiaSel;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(5)|LPTIASW(2)|LPTIASW(4)|LPTIASW(12)|LPTIASW(13); 
  }
  AD5940_LPLoopCfgS(&lp_loop);

  
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_VZERO0;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_AIN4;
  dsp_cfg.ADCBaseCfg.ADCPga = AppAMPCfg.ADCPgaGain;
  
  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));
  memset(&dsp_cfg.DftCfg, 0, sizeof(dsp_cfg.DftCfg));
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care because it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;	/* Tell filter block clock rate of ADC*/
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppAMPCfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppAMPCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bFALSE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
  AD5940_DSPCfgS(&dsp_cfg);
  
  sw_cfg.Dswitch = 0;
  sw_cfg.Pswitch = 0;
  sw_cfg.Nswitch = 0;
  sw_cfg.Tswitch = 0; 
  AD5940_SWMatrixCfgS(&sw_cfg);
    
  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_AFECtrlS(AFECTRL_SINC2NOTCH, bFALSE);
  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->endSeq, GP5 -> AD8233=OFF, GP1->RLD=OFF .
  
  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    AppAMPCfg.InitSeqInfo.SeqId = SEQID_1;
    AppAMPCfg.InitSeqInfo.SeqRamAddr = AppAMPCfg.SeqStartAddr;
    AppAMPCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppAMPCfg.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppAMPCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

static AD5940Err AppAMPSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  
  clks_cal.DataType = DATATYPE_SINC2;
  clks_cal.DataCount = 1;
  clks_cal.ADCSinc2Osr = AppAMPCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppAMPCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppAMPCfg.SysClkFreq/AppAMPCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
	WaitClks += 15;
  //printf("WaitClks = %d \n", WaitClks);
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));    /* wait 250us */
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);   /* Start ADC convert*/
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bFALSE);  /* Stop ADC */
  AD5940_SEQGpioCtrlS(0);
  AD5940_EnterSleepS();/* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  if(error == AD5940ERR_OK)
  {
    AppAMPCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppAMPCfg.MeasureSeqInfo.SeqRamAddr = AppAMPCfg.InitSeqInfo.SeqRamAddr + AppAMPCfg.InitSeqInfo.SeqLen ;
    AppAMPCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppAMPCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppAMPCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}
static AD5940Err AppAMPRtiaCal(void)
{
fImpPol_Type RtiaCalValue;  /* Calibration result */
  LPRTIACal_Type lprtia_cal;
  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

  lprtia_cal.bPolarResult = bTRUE;                /* Magnitude + Phase */
  lprtia_cal.AdcClkFreq = AppAMPCfg.AdcClkFreq;
  lprtia_cal.SysClkFreq = AppAMPCfg.SysClkFreq;
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;        /* Use SINC2 data as DFT data source */
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;         /* Maximum DFT number */
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;   /* For frequency under 12Hz, need to optimize DFT source. Use SINC3 data as DFT source */
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = AppAMPCfg.AdcClkFreq/4/22/2048*3;  /* Sample 3 period of signal, 13.317Hz here. Do not use DC method, because it needs ADC/PGA calibrated firstly(but it's faster) */
  lprtia_cal.fRcal = AppAMPCfg.RcalVal;
  lprtia_cal.LpTiaRtia = AppAMPCfg.LptiaRtiaSel;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;
  AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
  AppAMPCfg.RtiaCalValue = RtiaCalValue;
  //printf("RTIA Value = %f \n", RtiaCalValue.Magnitude);
 
  return AD5940ERR_OK;
}
/* This function provide application initialize.   */
AD5940Err AppAMPInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;

  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* Do RTIA calibration */
  if(((AppAMPCfg.ReDoRtiaCal == bTRUE) || \
      AppAMPCfg.AMPInited == bFALSE) && AppAMPCfg.ExtRtia == bFALSE)  /* Do calibration on the first initializaion */
  {
    AppAMPRtiaCal();
    AppAMPCfg.ReDoRtiaCal = bFALSE;
  }else
		AppAMPCfg.RtiaCalValue.Magnitude = AppAMPCfg.ExtRtiaVal;
  
	/* Reconfigure FIFO */
  AD5940_FIFOCtrlS(DFTSRC_SINC3, bFALSE);									/* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = AppAMPCfg.DataFifoSrc;
  fifo_cfg.FIFOThresh = AppAMPCfg.FifoThresh;              
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  
  /* Start sequence generator */
  /* Initialize sequencer generator */
  if((AppAMPCfg.AMPInited == bFALSE)||\
       (AppAMPCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;   
    AD5940_SEQGenInit(pBuffer, BufferSize);

    /* Generate initialize sequence */
    error = AppAMPSeqCfgGen(); /* Application initialization sequence using either MCU or sequencer */
    if(error != AD5940ERR_OK) return error;

    /* Generate measurement sequence */
    error = AppAMPSeqMeasureGen();
    if(error != AD5940ERR_OK) return error;

    AppAMPCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
  }
  /* Initialization sequencer  */
  AppAMPCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppAMPCfg.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer */
  AD5940_SEQMmrTrig(AppAMPCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  
  /* Measurement sequence  */
  AppAMPCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppAMPCfg.MeasureSeqInfo);

//  seq_cfg.SeqEnable = bTRUE;
//  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
	AD5940_SEQCtrlS(bTRUE);  /* Enable sequencer, and wait for trigger. It's disabled in initialization sequence */
  AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */

  AD5940_AFEPwrBW(AppAMPCfg.PwrMod, AFEBW_250KHZ);
  AppAMPCfg.AMPInited = bTRUE;  /* AMP application has been initialized. */
  return AD5940ERR_OK;
}

/* Modify registers when AFE wakeup */
static AD5940Err AppAMPRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppAMPCfg.NumOfData > 0)
  {
    AppAMPCfg.FifoDataCount += *pDataCount/4;
    if(AppAMPCfg.FifoDataCount >= AppAMPCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppAMPCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  
  return AD5940ERR_OK;
}

/* Depending on the data type, do appropriate data pre-process before return back to controller */
static AD5940Err AppAMPDataProcess(int32_t * const pData, uint32_t *pDataCount, float *IData)
{
  uint32_t i, datacount;
  datacount = *pDataCount;
  float *pOut = (float *)pData;
  for(i=0;i<datacount;i++)
  {
    pData[i] &= 0xffff;
    pOut[i] = AppAMPCalcCurrent(pData[i]);
    IData[i] = pOut[i];
  }

  return AD5940ERR_OK;
}

AD5940Err AppAMPISR(void *pBuff, uint32_t *pCount, uint32_t APPBUFF_SIZE, float *IData)
{
  uint32_t FifoCnt;
  if(AD5940_WakeUp(10) > 10){  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */
    printf("Wakeup Failed \n");
  }
  
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
	
  //*pCount = 0;  
  //if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE){    return 0;}


    FifoCnt = AD5940_FIFOGetCnt();
    //printf("FIFO count: %d \n", FifoCnt);
    if (FifoCnt >= APPBUFF_SIZE){
      AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
      AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
      AppAMPRegModify(pBuff, &FifoCnt);   /* If there is need to do AFE re-configure, do it here when AFE is in active state */
      AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); 
      //AD5940_EnterSleepS();  /* Manually put AFE back to hibernate mode. This operation only takes effect when register value is ACTIVE previously */

      /* Process data */
      AppAMPDataProcess((int32_t*)pBuff,&FifoCnt, IData); 
    }
    
    *pCount = FifoCnt;

  
  return 0;
} 

/* Calculate voltage */
float AppAMPCalcVoltage(uint32_t ADCcode)
{
  float kFactor = 1.835/1.82;
  float fVolt = 0.0;
  int32_t tmp = 0;
  tmp = ADCcode - 32768;
  switch(AppAMPCfg.ADCPgaGain)
  {
    case ADCPGA_1:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/1)*kFactor;
      break;
    case ADCPGA_1P5:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/1.5f)*kFactor;
      break;
    case ADCPGA_2:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/2)*kFactor;
      break;
    case ADCPGA_4:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/4)*kFactor;
      break;
    case ADCPGA_9:
      fVolt = ((float)(tmp)/32768)*(AppAMPCfg.ADCRefVolt/9)*kFactor;
      break;
  } 
  return fVolt;
}
/* Calculate current in uA */
float AppAMPCalcCurrent(uint32_t ADCcode)
{
  float fCurrent, fVoltage = 0.0;

  fVoltage = AppAMPCalcVoltage(ADCcode);
  fCurrent = fVoltage/1000.0; // AppAMPCfg.RtiaCalValue.Magnitude;
  
  return -fCurrent*1000000;
}

// Cal function

void AD5940_PGA_Calibration(void){
  AD5940Err err;
  ADCPGACal_Type pgacal;
  pgacal.AdcClkFreq = 16e6;
  pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
  pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
  pgacal.SysClkFreq = 16e6;
  pgacal.TimeOut10us = 1000;
  pgacal.VRef1p11 = 1.11f;
  pgacal.VRef1p82 = 1.82f;
  pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
  pgacal.ADCPga = ADCPGA_GAIN_SEL;
  err = AD5940_ADCPGACal(&pgacal);
  if(err != AD5940ERR_OK){
    printf("AD5940 PGA calibration failed.");
  }
}

void AD5940_PGA_V_Calibration(void){
    AD5940Err err;
    ADCPGACal_Type pgacal;
    pgacal.AdcClkFreq = 16e6;
    pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
    pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
    pgacal.SysClkFreq = 16e6;
    pgacal.TimeOut10us = 1000;
    pgacal.VRef1p11 = 1.11f;
    pgacal.VRef1p82 = 1.82f;
    pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
    pgacal.ADCPga = ADCPGA_GAIN_SEL;
    err = AD5940_ADCPGACal(&pgacal);
    if(err != AD5940ERR_OK){
        printf("AD5940 PGA calibration failed.");
    }
}

void TestAmp()
{
  printf("Amp_Test OK\n");
}

ADCBaseCfg_Type ADC_Setup(ADCBaseCfg_Type adc_base, ADCFilterCfg_Type adc_filter)
{
    AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE); //We are going to measure DAC 1.82V reference.
    adc_base.ADCMuxP = ADCMUXP_VREF1P8DAC;
    adc_base.ADCMuxN = ADCMUXN_VSET1P1;
    adc_base.ADCPga = ADCPGA_GAIN_SEL;
    AD5940_ADCBaseCfgS(&adc_base);

    /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
    adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
    adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
    adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
    adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
    adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
    adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */   
    adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */ 
    AD5940_ADCFilterCfgS(&adc_filter);

    //printf("ADC Setup Complete\n");
    return adc_base;
}



