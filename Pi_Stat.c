#include <stdio.h>

#include "string.h"
#include <tusb.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"

//AD5940 libraries and SPI
#include "hardware/spi.h"
#include "ad5940.h"

//Bluetooth libraries
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "server_common.h"

//Define SPI pins
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   20
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_RESET 21
#define Chg_Mon 7

#define ADCPGA_GAIN_SEL   ADCPGA_1P5
#define APPBUFF_SIZE 54
#define DBUFF_SIZE 60

uint32_t AppBuff[APPBUFF_SIZE];
float IData[APPBUFF_SIZE];
uint32_t data_count = 0;
float LFOSCFreq;

//BT Heartbeat parameters
#define HEARTBEAT_PERIOD_MS 1000
static int counter = 0;
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

bool POTENTIOMETRIC_MODE = true;
bool POTASSIUM = true;
bool Run_Initialization = true;

uint32_t Voltage_Value; 
uint32_t Current_Value;
uint32_t TX_Buffer[Buff_Size];
uint32_t V_Buffer[Buff_Size-2];
uint32_t I_Buffer[Buff_Size-2];
uint32_t Time_Buffer[Buff_Size-2];

uint32_t exec_start_time;
uint32_t current_time;
uint16_t read_count = 0;
uint16_t Main_Loop_Frequency = 2;

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

//Display result of amperometry
int32_t AMPShowResult(uint32_t *pData, uint32_t DataCount, uint32_t t)
{
  for (int i = 0; i < DataCount; i++){
    printf("time: %f, Current:%fuA\n", i*0.5, pData[i]);
    printf("time: %f, Current:%fuA\n", i*0.5, pData[i]);
  }
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

/* Initialize AD5940 basic blocks like clock */
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

/*Amperometry parameters*/
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


/*Potentiometry parameters*/
void Initialize_Potentiometry(void){
    ADCBaseCfg_Type adc_base;
    ADCFilterCfg_Type adc_filter;
    
    AD5940_HWReset();
    AD5940_Initialize();
    AD5940_PGA_V_Calibration();

    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
    AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE); //We are going to measure DAC 1.82V reference.
    adc_base.ADCMuxP = ADCMUXP_AIN0;
    adc_base.ADCMuxN = ADCMUXP_VRE0;
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

void Initialize_Pulsed_Amperometry(void){
    AD5940_HWReset();
    AD5940_Initialize();
    AD5940PlatformCfg();
    AD5940AMPStructInit(); /* Configure your parameters in this function */ 
    AppAMPInit(AppBuff, APPBUFF_SIZE); 
}

static void heartbeat_handler(struct btstack_timer_source *ts) {
    static uint32_t counter = 0;
    counter++;

    // Update the temp every 10s
    if (counter % 2 == 0) {
        if (le_notification_enabled) {
            att_server_request_can_send_now_event(con_handle);
        }
    }

    // Invert the led
    static int led_on = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    // Restart timer
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

int main()
{
    stdio_init_all();
    //while ((getchar()) != '\n'){}
    
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

    att_server_init(profile_data, att_read_callback, att_write_callback);    

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(packet_handler);

    // set one-shot btstack timer
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);

    // turn on bluetooth!
    hci_power_control(HCI_POWER_ON);

    uint32_t P_Time = 300000;
    uint32_t A_Time = 30000;
    uint32_t T_Time = P_Time + 4*A_Time;

    //Variables used for sequencing
    exec_start_time = to_ms_since_boot(get_absolute_time());
    current_time = exec_start_time;
    uint32_t time_remainder = 0;
    uint32_t temp;

    uint16_t Sequencer = 0;
    uint16_t Sequencer_t = 0;
    uint16_t read_count = 0;
    bool Initialize_Sequence = true;
    bool Event_Trigger = true;
    
    exec_start_time = to_ms_since_boot(get_absolute_time());
    while(true){
        current_time = to_ms_since_boot(get_absolute_time()) - exec_start_time;
        Time_Buffer[read_count] = current_time;
        time_remainder = current_time%T_Time;
        
        Sequencer_t = Sequencer;
        if (time_remainder<=P_Time){Sequencer=0;}
        else if (time_remainder>P_Time && time_remainder<=P_Time+A_Time){Sequencer=1;}
        else if (time_remainder>P_Time+A_Time && time_remainder<=P_Time+2*A_Time){Sequencer=2;}	
        else if (time_remainder>P_Time+2*A_Time && time_remainder<=P_Time+3*A_Time){Sequencer=3;}	
        else if (time_remainder>P_Time+3*A_Time){Sequencer=4;}	
    
        if (Sequencer_t != Sequencer){Event_Trigger=true;}
        
        
        switch (Sequencer){
        case 0:
            if (Event_Trigger){
                printf("starting case %u \n", Sequencer);
                for (int i=2; i<Buff_Size; i++){TX_Buffer[i]=0;}

                TX_Buffer[0] = Sequencer;		
                TX_Buffer[1] = current_time;
                read_count = 0;
                POTASSIUM=true;
            
                AD5940_AGPIOSet(AGPIO_Pin0);
                AD5940_AGPIOClr(AGPIO_Pin1);
                
                Initialize_Potentiometry();
                Event_Trigger = false;
            }   
            
            
            if (read_count > 60){ 
                for (int i=2; i<Buff_Size; i++){TX_Buffer[i]=0;}
                TX_Buffer[1]=current_time;
                read_count=0;
                POTASSIUM=true;
            }

            //Trade off betwen measureing K+ and pH
            if (POTASSIUM){
                AD5940_ADCMuxCfgS(ADCMUXP_AIN0, ADCMUXP_VRE0);
                POTASSIUM=false;
            }else{
                AD5940_ADCMuxCfgS(ADCMUXP_AIN1, ADCMUXP_VRE0);
                POTASSIUM=true;
            }
            
            //Read voltage and push value to transmission buffer
            Voltage_Value = AD5940_ReadAfeResult(AFERESULT_SINC2);
            TX_Buffer[read_count+2] = Voltage_Value;
            
            read_count++;
            break;
    
        case 1:
            if (Event_Trigger){
                printf("starting case %u \n", Sequencer);

                AD5940_AGPIOSet(AGPIO_Pin1);
                AD5940_AGPIOClr(AGPIO_Pin0);
                
                Initialize_Pulsed_Amperometry();
                //for (int i=0; i<BUFF_SIZE; i++){TX_Buffer[i]=0;}
                
                TX_Buffer[0] = Sequencer;		
                TX_Buffer[1] = current_time;
                
                AppAMPCtrl(AMPCTRL_START, 0);
                Event_Trigger = false;
            }
            
            //Waiting for data to populate FIFO
            break;
    
        case 2:
            if (Event_Trigger){
                printf("starting case %u \n", Sequencer);
                for (int i=2; i<Buff_Size; i++){TX_Buffer[i]=0;}
                TX_Buffer[0] = Sequencer;		
                //TX_Buffer[1] = current_time;
            
                temp = APPBUFF_SIZE;
                AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                
                //AMPShowResult(AppBuff, temp, current_time); /* Show the results to UART */
                AppAMPISR(AppBuff, &temp, APPBUFF_SIZE, IData);
                
                //for (int i=0; i<APPBUFF_SIZE; i++){TX_Buffer[i+2]=AppBuff[i];}
                for (int i=0; i<APPBUFF_SIZE; i++){TX_Buffer[i+2]=(uint32_t)round(IData[i]*1000);}
                Event_Trigger = false;
            }	
            //Waiting for predetermined relaxation period
    
            break;
    
        case 3:
            if (Event_Trigger){
                printf("starting case %u \n", Sequencer);
                //for (int i=0; i<BUFF_SIZE; i++){TX_Buffer[i]=0;}
                
                TX_Buffer[0] = Sequencer;		
                TX_Buffer[1] = current_time;
                
                AppAMPCtrl(AMPCTRL_START, 0);
                Event_Trigger = false;
            }	
        
            break;
    
        case 4:
            if (Event_Trigger){
                printf("starting case %u \n", Sequencer);
                for (int i=2; i<Buff_Size; i++){TX_Buffer[i]=0;}
                TX_Buffer[0] = Sequencer;		
                //TX_Buffer[1] = current_time;
            
                temp = APPBUFF_SIZE;
                AppAMPCtrl(AMPCTRL_STOPSYNC, 0);
                
                //AMPShowResult(AppBuff, temp, current_time); /* Show the results to UART */
                AppAMPISR(AppBuff, &temp, APPBUFF_SIZE, IData);
                
                //for (int i=0; i<APPBUFF_SIZE; i++){TX_Buffer[i+2]=AppBuff[i];}
                for (int i=0; i<APPBUFF_SIZE; i++){TX_Buffer[i+2]=(uint32_t)round(IData[i]*1000);}
                Event_Trigger = false;
            }
            break;
        }

        //for (int i=0; i<Buff_Size; i++){printf("%u \n", TX_Buffer[i]);}
        sleep_ms(1000/Main_Loop_Frequency);
    }
}