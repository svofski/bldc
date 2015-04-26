#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "adns-9800.h"
#include "xprintf.h"
#include "delay.h"

#include "adns9800_srom_A6.h"

#define adns_firmware_data SROMA6

#define ADNS_CS_GPIO    GPIOA
#define ADNS_CS_PIN     GPIO_Pin_4
#define ADNS_CS_BR      GPIO_BSRR_BR4
#define ADNS_CS_BS      GPIO_BSRR_BS4

#define ADNS_MOT_GPIO   GPIOB
#define ADNS_MOT_PIN    GPIO_Pin_0

// SPI1: SCK = Pin_5, MISO = Pin_6, MOSI = Pin_7

#define SPI_ADNS                    SPI1
#define SPI_ADNS_CLK                RCC_APB2Periph_SPI1
#define SPI_ADNS_GPIO               GPIOA
#define SPI_ADNS_GPIO_CLK           RCC_APB2Periph_GPIOA  
#define SPI_ADNS_PIN_SCK            GPIO_Pin_5
#define SPI_ADNS_PIN_MISO           GPIO_Pin_6
#define SPI_ADNS_PIN_MOSI           GPIO_Pin_7
#define SPI_ADNS_PIN_NSS            GPIO_Pin_4
#define SPI_ADNS_IRQn               SPI1_IRQn

#define SPI_DMA                     DMA1
#define SPI_DMA_CLK                 RCC_AHBPeriph_DMA1


#define SPI_Rx_DMA_Channel          DMA1_Channel2
#define SPI_Rx_DMA_FLAG             DMA1_FLAG_TC2
#define SPI_Rx_DMA_IRQn             DMA1_Channel2_IRQn
#define SPI_Rx_DMA_IRQHandler       DMA1_Channel2_IRQHandler

#define SPI_Tx_DMA_Channel          DMA1_Channel3
#define SPI_Tx_DMA_FLAG             DMA1_FLAG_TC3  

#define adns_cs_begin               ADNS_CS_GPIO->BSRR=ADNS_CS_BR
#define adns_cs_end                 ADNS_CS_GPIO->BSRR=ADNS_CS_BS
#define adns_cs(X)                  adns_cs_begin;X;adns_cs_end

#define SROM_BYTE_FREQ              50000

volatile uint32_t adns_firmware_data_i;

DMA_InitTypeDef         DMA_InitStruct;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
NVIC_InitTypeDef        NVIC_InitStructure;

static const char *TAG = "ADNS-9800";

// Dummy byte for DMA SPI
static const uint32_t zero = 0;

static void uploadFirmware(void);
static void configureDMA(void);
static uint8_t modifyReg(const char* name, uint8_t addr, uint8_t newvalue);
static uint16_t modifyReg16(const char* name, uint8_t addr_upper, uint16_t newvalue);

static volatile int DataReadyFlag;

int rel_x, rel_y;
int abs_x, abs_y;
int frameRate;
int squal;

ADNSMotionStruct motionStruct;

// Feed SROM bytes to the sensor at timer's pace (the pace should be 15uS between bytes)
void TIM1_UP_IRQHandler(void)
{
    TIM1->SR &= ~TIM_SR_UIF;
    SPI_ADNS->DR = adns_firmware_data[adns_firmware_data_i++];
    if (adns_firmware_data_i == sizeof(adns_firmware_data)/sizeof(adns_firmware_data[0])) {
        TIM1->CR1 &= ~1;
    }
}

// Complete Motion Burst DMA transfer. Raise ADNS_CS.
void SPI_Rx_DMA_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC2)) {
        DMA_ClearITPendingBit(DMA1_IT_GL2);

        DMA_Cmd(SPI_Tx_DMA_Channel, DISABLE);
        DMA_Cmd(SPI_Rx_DMA_Channel, DISABLE);

        SPI_I2S_DMACmd(SPI_ADNS, SPI_I2S_DMAReq_Rx, DISABLE);
        adns_cs_end;

        DataReadyFlag = 1;
    }
}

void ADNS9800_Config(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable SPI1, TIM1 Periph clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1, ENABLE);
    // Enable DMA1 Periph clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // GPIO pins for SPI
    // SCK and MOSI PP outputs
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin =  SPI_ADNS_PIN_SCK | SPI_ADNS_PIN_MISO | SPI_ADNS_PIN_MOSI;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_ADNS_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = ADNS_CS_PIN;
    GPIO_Init(ADNS_CS_GPIO, &GPIO_InitStructure);

    // Motion sensing pin
    GPIO_InitStructure.GPIO_Pin = ADNS_MOT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ADNS_MOT_GPIO, &GPIO_InitStructure);  

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;             // clock is high when idle
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;            // sample on rising edge
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;               // use software NSS
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI_ADNS, &SPI_InitStructure);

    SPI_Cmd(SPI_ADNS, ENABLE);
}

uint8_t spiTxRx(uint8_t data) 
{
    while (SPI_I2S_GetFlagStatus(SPI_ADNS, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI_ADNS, data);
    while (SPI_I2S_GetFlagStatus(SPI_ADNS, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI_ADNS);
}

uint8_t ADNSReadReg(uint8_t addr)
{
    adns_cs(
        spiTxRx(addr);
        Delay_us100(1); // SPI address to read delay (tSRAD) = 100uS
        uint8_t result = spiTxRx(0);
    );
    return result;
}

void ADNSWriteReg(uint8_t addr, uint8_t value)
{
    adns_cs(
        spiTxRx(addr | 0x80);
        spiTxRx(value);
    );  
    Delay_us100(1);
}

void ADNSLoadFirmware() 
{
    // set the configuration_IV register in 3k firmware mode
    ADNSWriteReg(ADNS_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 

    // write 0x1d in SROM_enable reg for initializing
    ADNSWriteReg(ADNS_SROM_Enable, 0x1d);   
    // wait for more than one frame period
    Delay_us100(4);

    // write 0x18 to SROM_enable to start SROM download
    ADNSWriteReg(ADNS_SROM_Enable, 0x18); 

    // write the SROM file (=firmware data) 
    adns_cs(
        spiTxRx(ADNS_SROM_Load_Burst | 0x80); // write burst destination adress
        //delay(2000); // 15uS
        Delay_us100(2);

        uploadFirmware();

        Delay_us100(40);
    );

    Delay_us100(40);
}

uint8_t ADNSInitialize()
{
    adns_cs();
    ADNSWriteReg(ADNS_Power_Up_Reset, 0x5a); // force reset
    Delay_us100(10);

    uint8_t product_id = ADNSReadReg(ADNS_Product_ID);
    uint8_t product_id_n = ADNSReadReg(ADNS_Inverse_Product_ID);
    xprintf("%s: Product ID=%x, Inverse ID=%x\n", TAG, product_id, product_id_n);
    if (product_id != 0x33 || (product_id ^ product_id_n) != 0xff) {
        xprintf("%s: Product ID mismatch. Abort.\n", TAG);
        return -1;
    }
    xprintf("%s: Uploading firmware", TAG);
    //Delay_us100(10);
    //xflush();
    ADNSLoadFirmware();

    if (ADNSCheckFirmware() != 0) {
        xprintf("%s: Firmware upload error\n", TAG);
        return -1;
    }

    Delay_us100(20);
    ADNSConfigureResolution();

    abs_x = abs_y = rel_x = rel_y = 0;

    ADNSLaserOn();

    configureDMA();

    return 0;
}

// Calculate firmware CRC, return 0 if it's correct
int ADNSCheckFirmware(void)
{
    uint8_t SROM_id = ADNSReadReg(ADNS_SROM_ID);
    xprintf("%s: SROM ID=%X\n", TAG, SROM_id);
    if (SROM_id != 0xA6) {
        xprintf("%s: SROM ID mismatch. Abort.\n", TAG);
        return -1;
    }

    ADNSWriteReg(ADNS_SROM_Enable, 0x15);
    Delay_us100(100);
    uint16_t crc = (uint16_t) ADNSReadReg(ADNS_Data_Out_Lower) | ADNSReadReg(ADNS_Data_Out_Upper) << 8;
    xprintf("%s: Firmware CRC: 0x%X\n", TAG, crc);
    
    return crc != 0xBEEF;
}

int ADNSConfigureResolution(void)
{
    // Set resolution: 0x01 = 50cpi, minimum, 0x44 = 3400 default, 0x8e = 7100, 0xA4 = 8200 (maximum)
    modifyReg("Configuration_I", ADNS_Configuration_I, 0xA4);
    // Disable rest mode, 0x08 = fixed frame rate, disable AGC
    modifyReg("Configuration_II", ADNS_Configuration_II, 0x08 + 0x10);
        
    // Default value for Shutter_Max_Bound is 0x4e20, this allows long exposure but limits maximum frame rate.
    // Frame rate, defined by Frame_Period_Max_Bound register, which must be written last in this sequence,
    // is constrained by this formula:
    // Frame_Period_Max_Bound >= Frame_Period_Min_Bound + Shutter_Max_Bound    
    uint16_t shutterMaxBound = 0x100;  // default value = 0x4e20, 0x100 allows 11748 fps tracking but requires better surface quality
    modifyReg16("Shutter_Max_Bound", ADNS_Shutter_Max_Bound_Upper, shutterMaxBound);
    modifyReg16("Frame_Period_Min_Bound", ADNS_Frame_Period_Min_Bound_Upper, 0x0fa0); // 0x0fa0 is the minimal allowed value
    // Set upper frame bound (default 0x5dc0 = 0x4e20 + 0x0fa0)
    // This register must be written last. This write also activates Shutter_Max_Bound and Frame_Period_Min_Bound settings.
    modifyReg16("Frame_Period_Max_Bound", ADNS_Frame_Period_Max_Bound_Upper, 0x0fa0 + shutterMaxBound);
    // Must seriously wait after setting this register
    Delay_us100(2);

    return 0;
}


// If DataReadyFlag is reset, initiate a poll using DMA.
// This function exits immediately. ADNSCheckMotion should be called to process received data
// and clear DataReadyFlag.
int ADNSDMAPoll(void)
{
    if (DataReadyFlag) {
        return 1;
    }
    DataReadyFlag = 0;

    adns_cs_begin;

    spiTxRx(ADNS_Motion_Burst);
    //Delay_us100(2);

    // Reset DMA Rx address
    SPI_Rx_DMA_Channel->CMAR = (uint32_t) &motionStruct;
    SPI_Rx_DMA_Channel->CNDTR = sizeof(ADNSMotionStruct);

    // Enable DMA RX request
    SPI_I2S_DMACmd(SPI_ADNS, SPI_I2S_DMAReq_Rx, ENABLE);
    // Enable DMA TX request
    SPI_I2S_DMACmd(SPI_ADNS, SPI_I2S_DMAReq_Tx, ENABLE);

    // // DMA Rx Transfer Complete interrupt
    // DMA_ITConfig(SPI_Rx_DMA_Channel, DMA_IT_TC, ENABLE);
    // NVIC_EnableIRQ(SPI_Rx_DMA_IRQn);

    SPI_ADNS->DR; // spit caca

    DMA_Cmd(SPI_Rx_DMA_Channel, ENABLE);
    DMA_Cmd(SPI_Tx_DMA_Channel, ENABLE);

    return 0;
}

int ADNSCheckMotion(void) 
{
    if (DataReadyFlag) {
        abs_x += motionStruct.Fields.Dx;
        abs_y += motionStruct.Fields.Dy;
        DataReadyFlag = 0;
        return motionStruct.Fields.Motion & 0x80;
    }
    return 0;
}

int ADNSPoll(void) 
{
    uint8_t motion, dx_l = 0, dx_h = 0, dy_h = 0, dy_l = 0;
    adns_cs(
      spiTxRx(ADNS_Motion_Burst);
      Delay_us100(2);
      do {
          motion = spiTxRx(0);
          if ((motion & 0x80) == 0)
            break;
          /* Observation*/ spiTxRx(0);
          dx_l   = spiTxRx(0);
          dx_h   = spiTxRx(0);
          dy_l   = spiTxRx(0);
          dy_h   = spiTxRx(0);
          squal  = spiTxRx(0) << 2;
          /* Pixel_Sum */     spiTxRx(0);
          /* Maximum_Pixel */ spiTxRx(0);
          /* Minimum_Pixel */ spiTxRx(0);
          /* Shutter_Upper */ spiTxRx(0);
          /* Shutter_Lower */ spiTxRx(0);
          frameRate = 50000000L / ((spiTxRx(0) << 8) + spiTxRx(0));
      } while(0);
    );
    
    rel_x = (int16_t) ((dx_h << 8) | dx_l);
    rel_y = (int16_t) ((dy_h << 8) | dy_l);
    
    abs_x += rel_x;
    abs_y += rel_y;
    
    return motion & 0x80;   
}

int ADNSGetX() {
    return abs_x;
}

int ADNSGetY() {
    return abs_y;
}

int ADNSGetSQUAL() {
    return motionStruct.Fields.SQUAL;
}

void ADNSLaserOn(void)
{
    //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
    // reading the actual value of the register is important because the real
    // default value is different from what is said in the datasheet, and if you
    // change the reserved bytes (like by writing 0x00...) it would not work.
    uint8_t laser_ctrl0 = ADNSReadReg(ADNS_LASER_CTRL0);
    modifyReg("LASER_CTRL0", ADNS_LASER_CTRL0, (laser_ctrl0 & 0xf0) | 0x04); // CW mode, laser enabled    
    Delay_us100(1);
}

// SROM is uploaded in burst mode, but there's a required pace of 15uS between bytes.
// So bytes should be fed one by one. This is done using TIM1.
static void uploadFirmware(void) 
{
    adns_firmware_data_i = 0;

    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct); 
    TIM_TimeBaseInitStruct.TIM_Period = (SystemCoreClock / SROM_BYTE_FREQ) - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0x0;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0x0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    NVIC_EnableIRQ(TIM1_UP_IRQn);

    while(TIM1->CR1 & TIM_CR1_CEN) {
        Delay_us100(100);
        xputchar('.');
    }
    xputchar('\n');
    TIM_Cmd(TIM1, DISABLE);

    // wait for the last transaction to clear up
    while (SPI_I2S_GetFlagStatus(SPI_ADNS, SPI_I2S_FLAG_TXE) == RESET);
    (void) SPI1->DR;
}

static uint8_t modifyReg(const char* name, uint8_t addr, uint8_t newvalue)
{
    xprintf("%s: %s %02X -> %02X", TAG, name, ADNSReadReg(addr), newvalue);
    Delay_us100(2);
    ADNSWriteReg(addr, newvalue);
    uint8_t readback = ADNSReadReg(addr);
    xprintf(": %02X %s\n", readback, readback == newvalue ? "OK" : "FAIL");
    Delay_us100(1);
    
    return readback;
}

static uint16_t modifyReg16(const char* name, uint8_t addr_upper, uint16_t newvalue)
{
    xprintf("%s: %s %04X -> %04X", TAG, name, (ADNSReadReg(addr_upper) << 8) + ADNSReadReg(addr_upper - 1), newvalue);
    Delay_us100(4);
    ADNSWriteReg(addr_upper - 1, newvalue & 0xff);
    ADNSWriteReg(addr_upper, newvalue >> 8);
    Delay_us100(4);
    uint16_t readback = (ADNSReadReg(addr_upper) << 8) + ADNSReadReg(addr_upper - 1);
    xprintf(": %04X %s\n", readback, readback == newvalue ? "OK" : "FAIL");
    Delay_us100(1);
    
    return readback;
}

// Set up DMA channels for Motion Burst poll. This is done only once.
//
// SPI_Tx_DMA_Channel transmits zero in circular mode to provide SPI clock
// SPI_Rx_DMA_Channel receives actual data to motionStruct
static void configureDMA(void)
{
    // SPI_Rx_DMA_Channel configuration
    DMA_DeInit(SPI_Rx_DMA_Channel);
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &(SPI1->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) &motionStruct;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize = sizeof(ADNSMotionStruct);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(SPI_Rx_DMA_Channel, &DMA_InitStruct);

    // SPI_Tx_DMA_Channel configuration 
    DMA_DeInit(SPI_Tx_DMA_Channel);
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &(SPI1->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) &zero;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize = 1;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(SPI_Tx_DMA_Channel, &DMA_InitStruct);

    // Enable DMA Rx Transfer Complete interrupt
    DMA_ITConfig(SPI_Rx_DMA_Channel, DMA_IT_TC, ENABLE);
    NVIC_EnableIRQ(SPI_Rx_DMA_IRQn);    
}

