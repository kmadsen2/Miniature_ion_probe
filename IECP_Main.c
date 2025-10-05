#include <stdio.h>
#include <math.h>
#include "ad5940.h"
#include "btstack.h"
#include <tusb.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "GAP_Advertisement/gap_config.h"
#include "GATT_Service/service_implementation.h"
#include "server_common.h"

//Define SPI pins
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   20
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_RESET 21
#define Chg_Mon 7

#define HEARTBEAT_PERIOD_MS 100
#define ADCPGA_GAIN_SEL   ADCPGA_1P5
float LFOSCFreq;
float exec_start_time;
float current_time;
uint16_t Main_Loop_Frequency = 2;
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

float P_Time = 300;
float A_Time = 30;
#define APPBUFF_SIZE 60
uint32_t AppBuff[APPBUFF_SIZE];
float IData[APPBUFF_SIZE];

static float characteristic_a_val = 0.00;
static int characteristic_b_val = 0;
static int characteristic_c_val = 0;
static float characteristic_d_val = 0.00;
static float characteristic_e_val = 0.00;
static float characteristic_f_val = 0.00;
static int characteristic_g_val = 0;

static char characteristic_a_tx[255] ;
static char characteristic_b_tx[255] ;
static char characteristic_c_rx[255] ;
static char characteristic_d_tx[255] ;
static char characteristic_e_tx[255] ;
static char characteristic_f_tx[255] ;
static char characteristic_g_tx[255] ;

//SPI Functions
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,unsigned char *pRecvBuff,unsigned long length)
{
    spi_write_read_blocking(spi0, pSendBuffer, pRecvBuff, length);
}

void AD5940_CsClr(void)
{
    gpio_put(PIN_CS, 0);
}

void AD5940_CsSet(void)
{
   gpio_put(PIN_CS, 1);
}

void AD5940_RstSet(void)
{
   gpio_put(PIN_RESET, 1);
}

void AD5940_RstClr(void)
{
   gpio_put(PIN_RESET, 0);
}

void AD5940_Delay10us(uint32_t time)
{
    if (time < 1638){
        sleep_us (time*10);
    }
    else if (time>=1638){
        uint32_t timedelaymicro = time % 1000;
        uint32_t timedelay = time - timedelaymicro;
        sleep_ms(timedelay/100);
        sleep_us(timedelaymicro*10);
    }
}

uint32_t MCUPlatformInit(void *pCfg)
{
    printf("MCU initialilzation\n");
    return 1;
}

uint32_t AD5940_MCUResourceInit(void *pCfg)
{
    printf("Resource initialilzation\n");
    return 1;
}

//Standin functions for interrupt handling
uint32_t AD5940_GetMCUIntFlag(void)
{
   return 1;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
   return 1;
}

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
	SEQCfg_Type seq_cfg;
  AGPIOCfg_Type gpio_cfg;
	LFOSCMeasure_Type LfoscMeasure;
  /* Use hardware reset */
  AD5940_HWReset();

  /* Platform configuration */
  AD5940_Initialize();

  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;                      /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;      
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
	fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
	
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP4_SYNC|GP2_SYNC|GP1_SLEEP|GP0_INT;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
	
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  /* Measure LFOSC frequency */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  //printf("Freq:%f\n", LFOSCFreq); 
  return 0;
}

/*Potentiometry parameters*/
void Initialize_Potentiometry(void){
    ADCBaseCfg_Type adc_base;
    ADCFilterCfg_Type adc_filter;
    
    AD5940_HWReset();
    AD5940_Initialize();
    AD5940_PGA_Calibration();

    LPLoopCfg_Type lp_loop;
    lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
    lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
    lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
    lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lp_loop.LpAmpCfg.LpTiaRf = LPTIARF_1M;
    lp_loop.LpAmpCfg.LpTiaRload = LPTIARLOAD_100R;
    lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_4K;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(15)|LPTIASW(8)|LPTIASW(12);
    AD5940_LPAMPCfgS(&lp_loop.LpAmpCfg);

    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
    AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE); //We are going to measure DAC 1.82V reference.
    adc_base.ADCMuxP = ADCMUXP_AIN0;
    adc_base.ADCMuxN = ADCMUXN_VBIAS0;
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

    /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 

    AD5940_ADCPowerCtrlS(bTRUE);
    AD5940_ADCConvtCtrlS(bTRUE);
}

void AD5940AMPStructInit(void)
{
  AppAMPCfg_Type *pAMPCfg;
  
  AppAMPGetCfg(&pAMPCfg);
	pAMPCfg->WuptClkFreq = LFOSCFreq;
    /* Configure general parameters */
    pAMPCfg->SeqStartAddr = 0;
    pAMPCfg->MaxSeqLen = 512;     /* @todo add checker in function */  
    pAMPCfg->RcalVal = 10000.0;
    pAMPCfg->NumOfData = -1;      /* Never stop until you stop it manually by AppAMPCtrl() function */	
	
	
	/* Configure measurement parameters */
    pAMPCfg->AmpODR = 0.5;          	/* Time between samples in seconds */
    pAMPCfg->FifoThresh = 4;      		/* Number of measurements before alerting host microcontroller */
	
    pAMPCfg->SensorBias = -0.2;   			/* Sensor bias voltage between reference and sense electrodes*/
    pAMPCfg->LptiaRtiaSel = LPTIARTIA_1K;
	pAMPCfg->LpTiaRl = LPTIARLOAD_10R;
	pAMPCfg->Vzero = 1100;        		/* Vzero voltage. Voltage on Sense electrode. Unit is mV*/
	
	pAMPCfg->ADCRefVolt = 1.82;		/* Measure voltage on Vref_1V8 pin */
}

void Initialize_Functions(int Current_State){
    switch(Current_State){
        case 0:
            AD5940PlatformCfg();
            break;
        case 1:
            AD5940PlatformCfg();
            Initialize_Potentiometry();
            break;
        case 2:
            AD5940PlatformCfg();
            AD5940AMPStructInit();
            AppAMPInit(AppBuff, APPBUFF_SIZE); 
            break;
        case 3:
            AD5940PlatformCfg();
            Initialize_Potentiometry();
            break;
        case 4:
            AD5940PlatformCfg();
            Initialize_Potentiometry();
            break;
        case 5:
            AD5940PlatformCfg();
        break;
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(3000);
    //getchar();

    spi_init(spi_default, 2000 * 1000);

    //Initialize SPI pins
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    //gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);

    //Initialize reset pin
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);

    //Set SPI polarity and phase
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    //void AD5940_Main(void);
    MCUPlatformInit(0);
    printf("AD5940-Build Time:%s\n",__TIME__);

    AD5940PlatformCfg();
    
    //Configure GPIO0 as output
    AD5940_AGPIOOen(AGPIO_Pin0);
    AD5940_AGPIOOen(AGPIO_Pin1);
    
    //Set GPIO values for potentiometric profiling
    AD5940_AGPIOSet(AGPIO_Pin0);
    AD5940_AGPIOClr(AGPIO_Pin1);

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    l2cap_init();
    sm_init();

    att_server_init(profile_data, NULL, NULL);
    
    // Instantiate our custom service handler
    custom_service_server_init( characteristic_a_tx, characteristic_b_tx,
                                characteristic_c_rx, characteristic_d_tx,
                                characteristic_e_tx, characteristic_f_tx,
                                characteristic_g_tx) ;

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(packet_handler);

    // turn on bluetooth!
    hci_power_control(HCI_POWER_ON);

    set_characteristic_a_value(characteristic_a_val) ;
    set_characteristic_b_value(characteristic_b_val) ;
    set_characteristic_c_value(characteristic_c_val) ;
    set_characteristic_d_value(characteristic_d_val) ;
    set_characteristic_e_value(characteristic_e_val) ;
    set_characteristic_f_value(characteristic_f_val) ;
    set_characteristic_g_value(characteristic_g_val) ;

    static char countval_a[20] ;
    int Current_State = 0;
    int Requested_State = 0;
    float Cycle_Time = 0;

    set_characteristic_c_value(Requested_State) ;
    exec_start_time = to_ms_since_boot(get_absolute_time());
    int Cycle_Index = 0;
    int Previous_State = 0;
    uint32_t rd;
    float Voltage_Value = 0;

    while (true) {
        current_time = (to_ms_since_boot(get_absolute_time()) - exec_start_time)/1000;
        sscanf(characteristic_c_rx, "%d", &Requested_State);
        
        if(Current_State!=Requested_State){
            printf("Requsting state change from %d to %d\n", Current_State, Requested_State);
            Current_State = Requested_State;
            Initialize_Functions(Current_State);
            set_characteristic_b_value(Current_State);

            if (Current_State==1||Current_State==3){
                Cycle_Time = current_time;
                AD5940_AGPIOSet(AGPIO_Pin0);
                AD5940_AGPIOClr(AGPIO_Pin1);
            }

            if (Current_State==2){
                Cycle_Time = current_time;
                AD5940_AGPIOSet(AGPIO_Pin1);
                AD5940_AGPIOClr(AGPIO_Pin0);
            }

            if (Current_State==4){
                AD5940_ADCMuxCfgS(ADCMUXP_AIN2, ADCMUXN_VBIAS0);
                Cycle_Time = current_time;
                AD5940_AGPIOSet(AGPIO_Pin0);
                AD5940_AGPIOClr(AGPIO_Pin1);
            }
        }

        switch (Current_State){
            case 0:
            set_characteristic_a_value(current_time) ;
                break;

            case 1:
            set_characteristic_a_value(current_time) ;
                if ((current_time-Cycle_Time) < P_Time){
                    Cycle_Index = 0;
                } else if(((current_time-Cycle_Time) > P_Time)&&((current_time-Cycle_Time) < P_Time+A_Time)){
                    Cycle_Index = 1;
                }else if(((current_time-Cycle_Time) > P_Time+A_Time)&&((current_time-Cycle_Time) < P_Time+2*A_Time)){
                    Cycle_Index = 2;
                }else if(((current_time-Cycle_Time) > P_Time+2*A_Time)&&((current_time-Cycle_Time) < P_Time+3*A_Time)){
                    Cycle_Index = 3;
                }else{
                    AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                    Cycle_Index = 0; Previous_State = 0;
                    Cycle_Time = current_time;
                }
                set_characteristic_g_value(Cycle_Index) ;

                if (Cycle_Index!=Previous_State&&Cycle_Index==1){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==2){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==2){
                    AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                    Initialize_Functions(1);
                    AD5940_AGPIOSet(AGPIO_Pin0);
                    AD5940_AGPIOClr(AGPIO_Pin1);
                    AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==3){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }
                
                if(Cycle_Index==0||Cycle_Index==2){
                    //Read K electrode
                    AD5940_ADCMuxCfgS(ADCMUXP_AIN0, ADCMUXN_VBIAS0);
                    rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
                    Voltage_Value = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
                    set_characteristic_d_value(Voltage_Value) ;
                    
                    //Read pH electrode
                    AD5940_ADCMuxCfgS(ADCMUXP_AIN1, ADCMUXN_VBIAS0);
                    rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
                    Voltage_Value = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
                    set_characteristic_e_value(Voltage_Value) ;
                } else if (Cycle_Index==1||Cycle_Index==3){
                    uint32_t temp = APPBUFF_SIZE;
                    AppAMPISR(AppBuff, &temp, APPBUFF_SIZE, IData);
                    characteristic_f_val = 0;

                    for (int i = 0; i < APPBUFF_SIZE; i++) {
                        characteristic_f_val = IData[i]/APPBUFF_SIZE;
                    }

                    set_characteristic_f_value(characteristic_f_val) ;
                }

                break;

            case 2:
            set_characteristic_a_value(current_time);
                if ((current_time-Cycle_Time) < A_Time){
                    Cycle_Index = 1;
                }else if(((current_time-Cycle_Time) > A_Time)&&((current_time-Cycle_Time) < 2*A_Time)){
                    Cycle_Index = 2;
                }else if(((current_time-Cycle_Time) > 2*A_Time)&&((current_time-Cycle_Time) < 3*A_Time)){
                    Cycle_Index = 3;
                }else{
                    AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                    Cycle_Index = 0; Previous_State = 0;
                    Cycle_Time = current_time;
                    Requested_State = 1;
                    set_characteristic_c_value(Requested_State); 
                }
                set_characteristic_g_value(Cycle_Index);

                if (Cycle_Index!=Previous_State&&Cycle_Index==1){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==2){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==2){
                    Initialize_Functions(1);
                    AD5940_AGPIOSet(AGPIO_Pin0);
                    AD5940_AGPIOClr(AGPIO_Pin1);
                    AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==3){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }
                
                if(Cycle_Index==0||Cycle_Index==2){
                    //Read K electrode
                    AD5940_ADCMuxCfgS(ADCMUXP_AIN0, ADCMUXN_VBIAS0);
                    rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
                    Voltage_Value = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
                    set_characteristic_d_value(Voltage_Value) ;
                    
                    //Read pH electrode
                    AD5940_ADCMuxCfgS(ADCMUXP_AIN1, ADCMUXN_VBIAS0);
                    rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
                    Voltage_Value = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
                    set_characteristic_e_value(Voltage_Value) ;
                } else if (Cycle_Index==1||Cycle_Index==3){
                    uint32_t temp = APPBUFF_SIZE;
                    AppAMPISR(AppBuff, &temp, APPBUFF_SIZE, IData);
                    characteristic_f_val = 0;

                    for (int i = 0; i < APPBUFF_SIZE; i++) {
                        characteristic_f_val = IData[i]/APPBUFF_SIZE;
                    }

                    set_characteristic_f_value(characteristic_f_val) ;
                }            
                break;

            case 3:
            set_characteristic_a_value(current_time) ;
                if ((current_time-Cycle_Time) < P_Time){
                    Cycle_Index = 0;
                } else if(((current_time-Cycle_Time) > P_Time)&&((current_time-Cycle_Time) < P_Time+A_Time)){
                    Cycle_Index = 1;
                }else if(((current_time-Cycle_Time) > P_Time+A_Time)&&((current_time-Cycle_Time) < P_Time+2*A_Time)){
                    Cycle_Index = 2;
                }else if(((current_time-Cycle_Time) > P_Time+2*A_Time)&&((current_time-Cycle_Time) < P_Time+3*A_Time)){
                    Cycle_Index = 3;
                }else{
                    AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                    Cycle_Index = 0; Previous_State = 0;
                    Cycle_Time = current_time;
                    Requested_State = 0;
                    set_characteristic_c_value(Requested_State); 
                }
                set_characteristic_g_value(Cycle_Index);

                if (Cycle_Index!=Previous_State&&Cycle_Index==1){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==2){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==2){
                    Initialize_Functions(1);
                    AD5940_AGPIOSet(AGPIO_Pin0);
                    AD5940_AGPIOClr(AGPIO_Pin1);
                    AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                    Previous_State = Cycle_Index;
                }else if (Cycle_Index!=Previous_State&&Cycle_Index==3){
                    Initialize_Functions(2);
                    AD5940_AGPIOSet(AGPIO_Pin1);
                    AD5940_AGPIOClr(AGPIO_Pin0);
                    AppAMPCtrl(AMPCTRL_START, 0);
                    Previous_State = Cycle_Index;
                }
                
                if(Cycle_Index==0||Cycle_Index==2){
                    //Read K electrode
                    AD5940_ADCMuxCfgS(ADCMUXP_AIN0, ADCMUXN_VBIAS0);
                    rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
                    Voltage_Value = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
                    set_characteristic_d_value(Voltage_Value) ;
                    
                    //Read pH electrode
                    AD5940_ADCMuxCfgS(ADCMUXP_AIN1, ADCMUXN_VBIAS0);
                    rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
                    Voltage_Value = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
                    set_characteristic_e_value(Voltage_Value) ;
                } else if (Cycle_Index==1||Cycle_Index==3){
                    uint32_t temp = APPBUFF_SIZE;
                    AppAMPISR(AppBuff, &temp, APPBUFF_SIZE, IData);
                    characteristic_f_val = 0;

                    for (int i = 0; i < APPBUFF_SIZE; i++) {
                        characteristic_f_val = IData[i]/APPBUFF_SIZE;
                    }

                    set_characteristic_f_value(characteristic_f_val);
                }
                break;

            case 4:
            set_characteristic_a_value(current_time) ;
                if ((current_time-Cycle_Time)<10){
                rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
                Voltage_Value = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
                set_characteristic_d_value(Voltage_Value) ;
                }else{
                Requested_State = 1;
                set_characteristic_c_value(Requested_State); 
                }
                break;

            case 5:
                Requested_State = 0;
                set_characteristic_c_value(Requested_State); 
                break;

        }
        sleep_ms(1000/Main_Loop_Frequency);
    }
    
    return 0;
}
