#include "stm8.h"
#include "API.h"
#include <string.h>
#define SET(x, y)   (x) |= (y)
#define UNSET(x, y) (x) &= ~(y)
#define READ(x, y)  ((x) & (y))
#define BMP180_ADDR	0x77
#define BH1750_ADDR	0x23
#define MCP4725_ADDR	0x62
#define SI7021_ADDR	0x40
#define HMC5883L_ADDR	0x1e
#define I2C_READ        1
#define I2C_WRITE       0
#define OSS		3
#define PC3		3
#define PC4		4
#define CSN		3
#define CE		4
#define BH1750_HRES2_ONETIME	0x23
#define BH1750_POWERON		0x01
#define MCP4725_CMD_WRITEDAC	0x40
#define SI7021_READ_HUMIDITY	0xe5
#define HMC5883L_CRA		0x00
#define HMC5883L_CRB		0x01
#define HMC5883L_MODE		0x02
typedef unsigned char UCHAR;
void delayTenMicro (void) {
   char a;
   for (a = 0; a < 50; ++a)
      __asm__("nop");
}
UCHAR write_spi (UCHAR value) {
   UCHAR ret;
   delayTenMicro ();
   SPI_DR = value;
   delayTenMicro ();
   while ((SPI_SR & TXE) == 0);
   delayTenMicro ();
   while ((SPI_SR & RXNE) == 0);
   delayTenMicro ();
   ret = SPI_DR;
   return (ret);
}
UCHAR write_spi_reg (UCHAR reg, UCHAR value) {
   UCHAR ret;
   PC_ODR &= ~(1 << CSN);
   ret = write_spi (reg);
   if (reg != NOP && reg != FLUSH_RX && reg != FLUSH_TX)
      write_spi (value);
   else
      delayTenMicro ();
   PC_ODR |= (1 << CSN);
   return (ret);
}
UCHAR read_spi_reg (UCHAR reg) {
   UCHAR ret;
   PC_ODR &= ~(1 << CSN);
   ret = write_spi (reg);
   if (reg != NOP && reg != FLUSH_RX && reg != FLUSH_TX)
      ret = write_spi (NOP);
   else
      delayTenMicro ();
   PC_ODR |= (1 << CSN);
   return (ret);
}
UCHAR write_spi_buf (UCHAR reg, UCHAR *array, UCHAR len) {
   UCHAR ret, n;
   PC_ODR &= ~(1 << CSN);
   ret = write_spi (reg);
   for (n = 0; n < len; ++n)
      write_spi (array[n]);
   PC_ODR |= (1 << CSN);
   return (ret);
}
UCHAR read_spi_buf (UCHAR reg, UCHAR *array, UCHAR len) {
   UCHAR ret, n;
   PC_ODR &= ~(1 << CSN);
   ret = write_spi (reg);
   for (n = 0; n < len; ++n)
      array[n] = write_spi (NOP);
   PC_ODR |= (1 << CSN);
   return (ret);
}
void InitializeSPI () {
   SPI_CR1 = MSBFIRST | SPI_ENABLE | BR_DIV256 | MASTER | CPOL0 | CPHA0;
   SPI_CR2 = BDM_2LINE | CRCEN_OFF | CRCNEXT_TXBUF | FULL_DUPLEX | SSM_DISABLE;
   SPI_ICR = TXIE_MASKED | RXIE_MASKED | ERRIE_MASKED | WKIE_MASKED;
   PC_DDR = (1 << PC3) | (1 << PC4); // output mode
   PC_CR1 = (1 << PC3) | (1 << PC4); // push-pull
   PC_CR2 = (1 << PC3) | (1 << PC4); // up to 10MHz speed
   PC_ODR != (1 << CSN);
   PC_ODR &= ~(1 << CE);
}
void InitializeSystemClock() {
   CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
   CLK_ICKR = CLK_HSIEN;               //  Enable the HSI.
   CLK_ECKR = 0;                       //  Disable the external clock.
   while ((CLK_ICKR & CLK_HSIRDY) == 0);       //  Wait for the HSI to be ready for use.
   CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
   CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
   CLK_PCKENR2 = 0xff;                 //  Ditto.
   CLK_CCOR = 0;                       //  Turn off CCO.
   CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
   CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
   CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
   CLK_SWCR = 0;                       //  Reset the clock switch control register.
   CLK_SWCR = CLK_SWEN;                //  Enable switching.
   while ((CLK_SWCR & CLK_SWBSY) != 0);        //  Pause while the clock switch is busy.
}
void delay (int time_ms) {
   volatile long int x;
   for (x = 0; x < 1036*time_ms; ++x)
      __asm__("nop");
}
void i2c_read (unsigned char *x) {
   while ((I2C_SR1 & I2C_RXNE) == 0);
   *x = I2C_DR;
}
void i2c_set_nak (void) {
   I2C_CR2 &= ~I2C_ACK;
}
void i2c_set_stop (void) {
   I2C_CR2 |= I2C_STOP;
}
void i2c_send_reg (UCHAR addr) {
   volatile int reg;
   reg = I2C_SR1;
   reg = I2C_SR3;
   I2C_DR = addr;
   while ((I2C_SR1 & I2C_TXE) == 0);
}
void i2c_send_address (UCHAR addr, UCHAR mode) {
   volatile int reg;
   reg = I2C_SR1;
   I2C_DR = (addr << 1) | mode;
   if (mode == I2C_READ) {
      I2C_OARL = 0;
      I2C_OARH = 0;
   }
   while ((I2C_SR1 & I2C_ADDR) == 0);
   if (mode == I2C_READ)
      UNSET (I2C_SR1, I2C_ADDR);
}
void i2c_set_start_ack (void) {
   I2C_CR2 = I2C_ACK | I2C_START;
   while ((I2C_SR1 & I2C_SB) == 0);
}
//
//  Send a message to the debug port (UART1).
//
void UARTPrintF (char *message) {
   char *ch = message;
   while (*ch) {
      UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
      while ((UART1_SR & SR_TXE) == 0);   //  Wait for transmission to complete.
      ch++;                               //  Grab the next character.
   }
}
void print_UCHAR_hex (unsigned char buffer) {
   unsigned char message[8];
   int a, b;
   a = (buffer >> 4);
   if (a > 9)
      a = a + 'a' - 10;
   else
      a += '0'; 
   b = buffer & 0x0f;
   if (b > 9)
      b = b + 'a' - 10;
   else
      b += '0'; 
   message[0] = a;
   message[1] = b;
   message[2] = 0;
   UARTPrintF (message);
}
unsigned char i2c_read_register (UCHAR addr, UCHAR rg) {
   volatile UCHAR reg;
   UCHAR x;
   i2c_set_start_ack ();
   i2c_send_address (addr, I2C_WRITE);
   i2c_send_reg (rg);
   i2c_set_start_ack ();
   i2c_send_address (addr, I2C_READ);
   reg = I2C_SR1;
   reg = I2C_SR3;
   i2c_set_nak ();
   i2c_set_stop ();
   i2c_read (&x);
   return (x);
}
   
void InitializeI2C (void) {
   I2C_CR1 = 0;   //  Disable I2C before configuration starts. PE bit is bit 0
   //
   //  Setup the clock information.
   //
   I2C_FREQR = 16;                     //  Set the internal clock frequency (MHz).
   UNSET (I2C_CCRH, I2C_FS);           //  I2C running is standard mode.
   I2C_CCRL = 0x10;                    //  SCL clock speed is 500 kHz.
   I2C_CCRH &= 0xf0;	// Clears lower 4 bits "CCR"
   //
   //  Set the address of this device.
   //
   UNSET (I2C_OARH, I2C_ADDMODE);      //  7 bit address mode.
   SET (I2C_OARH, I2C_ADDCONF);        //  Docs say this must always be 1.
   //
   //  Setup the bus characteristics.
   //
   I2C_TRISER = 17;
   //
   //  Turn on the interrupts.
   //
   //I2C_ITR = I2C_ITBUFEN | I2C_ITEVTEN | I2C_ITERREN; //  Buffer, event and error interrupts enabled
   //
   //  Configuration complete so turn the peripheral on.
   //
   I2C_CR1 = I2C_PE;	// Enables port
   //
   //  Enter master mode.
   //
}
   
void InitializeUART() {
    //
    //  Clear the Idle Line Detected bit in the status register by a read
    //  to the UART1_SR register followed by a Read to the UART1_DR register.
    //
    unsigned char tmp = UART1_SR;
    tmp = UART1_DR;
    //
    //  Reset the UART registers to the reset values.
    //
    UART1_CR1 = 0;
    UART1_CR2 = 0;
    UART1_CR4 = 0;
    UART1_CR3 = 0;
    UART1_CR5 = 0;
    UART1_GTR = 0;
    UART1_PSCR = 0;
    //
    //  Now setup the port to 115200,n,8,1.
    //
    UNSET (UART1_CR1, CR1_M);        //  8 Data bits.
    UNSET (UART1_CR1, CR1_PCEN);     //  Disable parity.
    UNSET (UART1_CR3, CR3_STOPH);    //  1 stop bit.
    UNSET (UART1_CR3, CR3_STOPL);    //  1 stop bit.
    UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
    UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
    //
    //  Disable the transmitter and receiver.
    //
    UNSET (UART1_CR2, CR2_TEN);      //  Disable transmit.
    UNSET (UART1_CR2, CR2_REN);      //  Disable receive.
    //
    //  Set the clock polarity, lock phase and last bit clock pulse.
    //
    SET (UART1_CR3, CR3_CPOL);
    SET (UART1_CR3, CR3_CPHA);
    SET (UART1_CR3, CR3_LBCL);
    //
    //  Turn on the UART transmit, receive and the UART clock.
    //
    SET (UART1_CR2, CR2_TEN);
    SET (UART1_CR2, CR2_REN);
    UART1_CR3 = CR3_CLKEN;
}




short SE8R01_DR_2M=0;  //choose 1 of these to set the speed
short SE8R01_DR_1M=0;
short SE8R01_DR_500K=1;


#define ADR_WIDTH    4   // 4 unsigned chars TX(RX) address width
#define PLOAD_WIDTH  32  // 32 unsigned chars TX payload

short pload_width_now=0;
short newdata=0;
UCHAR gtemp[5];
char signal_lv=0;
short pip=0;
unsigned char status =0;



unsigned char ADDRESS2[1]= {0xb1};	
unsigned char ADDRESS3[1]= {0xb2};	
unsigned char ADDRESS4[1]= {0xb3};		
unsigned char ADDRESS5[1]= {0xb4};	


unsigned char ADDRESS1[ADR_WIDTH]  = 
{
  0xb0,0x43,0x10,0x10
}; // Define a static TX address

//***************************************************


unsigned char ADDRESS0[ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10
}; // Define a static TX address

unsigned char rx_buf[PLOAD_WIDTH]= {0};
unsigned char tx_buf[PLOAD_WIDTH]= {0};
//***************************************************
 
struct dataStruct{
  unsigned long counter;
  UCHAR rt;
}myData_pip5;

struct dataStruct1{
  unsigned long counter;
  UCHAR rt;
}myData_pip4;







//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void)
{ //PD3 is interrupt
PD_DDR &= ~(1 << 3); // input mode
PD_CR1 |= (1 << 3); // input with pull up 
PD_CR2 |= (1 << 3); // interrupt enabled 
PD_ODR &= ~(1 << 3);
//  digitalWrite(IRQq, 0);
   //PC_ODR |= (1 << CE);
PC_ODR &= ~(1 << CE);
//  digitalWrite(CEq, 0);			// chip enable
PC_ODR |= (1 << CSN);
//  digitalWrite(CSNq, 1);                 // Spi disable	
}



void rf_switch_bank(unsigned char bankindex)
{
    unsigned char temp0,temp1;
    temp1 = bankindex;

    temp0 = write_spi(iRF_BANK0_STATUS);

    if((temp0&0x80)!=temp1)
    {
        write_spi_reg(iRF_CMD_ACTIVATE,0x53);
    }
}




void SE8R01_Calibration()
{
        unsigned char temp[5];
        rf_switch_bank(iBANK0);
        temp[0]=0x03;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG,temp, 1);

        temp[0]=0x32;

        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_CH, temp,1);



if (SE8R01_DR_2M==1)
  {temp[0]=0x48;}
else if (SE8R01_DR_1M==1)
  {temp[0]=0x40;}
else  
  {temp[0]=0x68;}   
  
  write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,temp,1);
  temp[0]=0x77;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD, temp,1);
  
        rf_switch_bank(iBANK1);
        temp[0]=0x40;
        temp[1]=0x00;
        temp[2]=0x10;
if (SE8R01_DR_2M==1)
          {temp[3]=0xE6;}
else
    {temp[3]=0xE4;}

        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, temp, 4);

        temp[0]=0x20;
        temp[1]=0x08;
        temp[2]=0x50;
        temp[3]=0x40;
        temp[4]=0x50;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, temp, 5);

        temp[0]=0x00;
        temp[1]=0x00;
if (SE8R01_DR_2M==1)
       { temp[2]=0x1E;}
else
   { temp[2]=0x1F;}

        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, temp, 3);
       
if (SE8R01_DR_2M==1)
       { temp[0]=0x29;}
else
 { temp[0]=0x14;}

        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, temp, 1);

        temp[0]=0x00;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW,temp,1);

        temp[0]=0x7F;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI,temp,1);

        temp[0]=0x02;
        temp[1]=0xC1;
        temp[2]=0xEB;            
        temp[3]=0x1C;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, temp,4);
//--
        temp[0]=0x97;
        temp[1]=0x64;
        temp[2]=0x00;
        temp[3]=0x81;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, temp, 4);
        rf_switch_bank(iBANK0);
        
//        digitalWrite(CEq, 1); 
        //delayMicroseconds(30);
//        digitalWrite(CEq, 0);  
 delayTenMicro();
   PC_ODR |= (1 << CE);
   delayTenMicro();
   PC_ODR &= ~(1 << CE);
        delay(50);                            // delay 50ms waitting for calibaration.

 //       digitalWrite(CEq, 1); 
 //       delayMicroseconds(30);
 //       digitalWrite(CEq, 0); 
 delayTenMicro();
   PC_ODR |= (1 << CE);
   delayTenMicro();
   PC_ODR &= ~(1 << CE);
        delay(50);                            // delay 50ms waitting for calibaration.
}


void SE8R01_Analog_Init()           //SE8R01 初始化
{    

  unsigned char temp[5];   

        gtemp[0]=0x28;
        gtemp[1]=0x32;
        gtemp[2]=0x80;
        gtemp[3]=0x90;
        gtemp[4]=0x00;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);
        delay(2);
  
 
  rf_switch_bank(iBANK1);
   
        temp[0]=0x40;
        temp[1]=0x01;               
        temp[2]=0x30;               
if (SE8R01_DR_2M==1)
       { temp[3]=0xE2; }              
else
 { temp[3]=0xE0;}
   
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, temp,4);

        temp[0]=0x29;
        temp[1]=0x89;
        temp[2]=0x55;                     
        temp[3]=0x40;
        temp[4]=0x50;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, temp,5);

if (SE8R01_DR_2M==1)
       { temp[0]=0x29;}
else
 { temp[0]=0x14;}
         
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, temp,1);

    temp[0]=0x55;
    temp[1]=0xC2;
    temp[2]=0x09;
    temp[3]=0xAC;  
    write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RX_CTRL,temp,4);

    temp[0]=0x00;
    temp[1]=0x14;
    temp[2]=0x08;   
    temp[3]=0x29;
    write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FAGC_CTRL_1, temp,4);

    temp[0]=0x02;
    temp[1]=0xC1;
    temp[2]=0xCB;  
    temp[3]=0x1C;
    write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, temp,4);

    temp[0]=0x97;
    temp[1]=0x64;
    temp[2]=0x00;
    temp[3]=0x01;
    write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, temp,4);
    
        gtemp[0]=0x2A;
        gtemp[1]=0x04;
        gtemp[2]=0x00;
        gtemp[3]=0x7D;
        write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_TEST_PKDET, gtemp, 4);

         rf_switch_bank(iBANK0);
}

void SE8R01_Init()  
{
  unsigned char temp[5];
        SE8R01_Calibration();   
        SE8R01_Analog_Init();   
         

  
if (SE8R01_DR_2M==1)
{  temp[0]=0b01001111; }     //2MHz,+5dbm
else if  (SE8R01_DR_1M==1)
{  temp[0]=0b01000111;  }     //1MHz,+5dbm
else
  {temp[0]=0b01101111;  }     //500K,+5dbm

write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,temp,1);



        write_spi_reg(WRITE_REG|iRF_BANK0_EN_AA, 0b00111111);          //enable auto acc on pip 1
        write_spi_reg(WRITE_REG|iRF_BANK0_EN_RXADDR, 0b00111111);      //enable pip 1
        write_spi_reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);  
        write_spi_reg(WRITE_REG|iRF_BANK0_RF_CH, 40);

  write_spi_buf(WRITE_REG + TX_ADDR, ADDRESS0, ADR_WIDTH);    	
  write_spi_buf(WRITE_REG + RX_ADDR_P0, ADDRESS0, ADR_WIDTH); 
  write_spi_buf(WRITE_REG + RX_ADDR_P1, ADDRESS1, ADR_WIDTH); 
  write_spi_buf(WRITE_REG + RX_ADDR_P2, ADDRESS2, 1); 
  write_spi_buf(WRITE_REG + RX_ADDR_P3, ADDRESS3, 1); 
  write_spi_buf(WRITE_REG + RX_ADDR_P4, ADDRESS4, 1); 
  write_spi_buf(WRITE_REG + RX_ADDR_P5, ADDRESS5, 1); 
  write_spi_reg(WRITE_REG + RX_PW_P0, PLOAD_WIDTH); 
  write_spi_reg(WRITE_REG|iRF_BANK0_CONFIG, 0x3f); 
  write_spi_reg(WRITE_REG|iRF_BANK0_DYNPD, 0b00111111);          // enable dynamic payload length data
  write_spi_reg(WRITE_REG|iRF_BANK0_FEATURE, 0x07);        // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack

//  digitalWrite(CEq, 1); 
// delayTenMicro();
   PC_ODR |= (1 << CE);
 //  delayTenMicro();
 //  PC_ODR &= ~(1 << CE);

}

/*

  init_io();                        // Initialize IO port
  write_spi_reg(FLUSH_RX,0); 
  unsigned char status=SPI_Read(STATUS);
  Serial.print("status = ");    
  Serial.println(status,HEX);     
  Serial.println("*******************Radio starting*****************");

 SE8R01_Init();

  if(digitalRead(IRQq)==LOW)
  {
//todo    delayMicroseconds(240);
delayTenMicro();
delayTenMicro();
delayTenMicro();
delayTenMicro();

    signal_lv=SPI_Read(iRF_BANK0_RPD);
    status = SPI_Read(STATUS);
    
    if(status&STA_MARK_RX)                                                 // if receive data ready (TX_DS) interrupt
    {   
      
      pip= (status&0b00001110)>>1;
      pload_width_now=SPI_Read(iRF_CMD_R_RX_PL_WID);
      SPI_Read_Buf(RD_RX_PLOAD, rx_buf,pload_width_now);             // read playload to rx_buf
      write_spi_reg(FLUSH_RX,0); 
      print_pip();
      newdata=1;
      }
   
    write_spi_reg(WRITE_REG+STATUS,status);       // clear RX_DR or TX_DS or MAX_RT interrupt flag
  
  }
   
    
  
  if(newdata==1)
  {
  newdata=0;
  if(pip==0)
  {
     unsigned long T_counter=0;
T_counter = (unsigned long)rx_buf[3] << 24;
T_counter += (unsigned long)rx_buf[2] << 16;
T_counter += (unsigned long)rx_buf[1] << 8;
T_counter += (unsigned long)rx_buf[0];

 Serial.print(" transmission counter: ");
 Serial.print(T_counter);
 
     Serial.print(" rt packets: ");
   Serial.print(rx_buf[4]&B00001111);
  }
  
  
  if(pip==1)
  {
  
  
  }
  
  
  else if(pip==2)
  {
  
  }
  
  else if(pip==3)
  {
   
  } 
   
   
  
 else if(pip==4)
  {
 memcpy(&myData_pip4, rx_buf, sizeof(myData_pip4));
 Serial.print(" transmission counter: ");
 Serial.print(myData_pip4.counter);
 Serial.print(" rt packets: ");
 Serial.print(myData_pip4.rt&B00001111);

    }
    
  
    else if(pip==5)
  {

 memcpy(&myData_pip5, rx_buf, sizeof(myData_pip5));
 Serial.print(" transmission counter: ");
 Serial.print(myData_pip5.counter);
 Serial.print(" rt packets: ");
 Serial.print(myData_pip5.rt&B00001111);

  }
  
  
  Serial.println("");
  
  }
  }
  
}


 void print_pip()
 {
  if(pip>5)
   {
   Serial.println("NO Data"); 
   }
   else{
   Serial.print(" SS= ");
   Serial.print(signal_lv,DEC);
   Serial.print("  pip: ");    
   Serial.print(pip);
   Serial.print(" ");
   Serial.print("rx buff with= ");
   Serial.print(pload_width_now);
   Serial.print(" data= ");
      for(UCHAR i=0; i<pload_width_now; i++)
      {
          Serial.print(" ");
          Serial.print(rx_buf[i]);                              // print rx_buf
      }
     Serial.print("  ");
 }}
 
*/


int main () {
int i;
   short voltage = 1900;
   UCHAR x, a;
   UCHAR rx_addr_p1[]  = { 0xd2, 0xf0, 0xf0, 0xf0, 0xf0 };
   UCHAR tx_addr[]     = { 0xe1, 0xf0, 0xf0, 0xf0, 0xf0 };
   UCHAR tx_payload[33];
  UCHAR readstatus;
   volatile int reg, x1, y1, z1;
   InitializeSystemClock();
   InitializeUART();
   InitializeI2C();
   InitializeSPI ();
/*
// Get the NRF24L01 ready
   reg = write_spi_reg (W_REGISTER + SETUP_AW, AW5);
   reg = write_spi_buf (W_REGISTER + TX_ADDR, tx_addr, 5);
   reg = write_spi_buf (W_REGISTER + RX_ADDR_P0, tx_addr, 5);
   reg = write_spi_buf (W_REGISTER + RX_ADDR_P1, rx_addr_p1, 5);
   reg = write_spi_reg (W_REGISTER + EN_AA, ENAA_P5 | ENAA_P4 | ENAA_P3 | ENAA_P2 | ENAA_P1 | ENAA_P0);
   reg = write_spi_reg (W_REGISTER + EN_RXADDR, ERX_P5 | ERX_P4 | ERX_P3 | ERX_P2 | ERX_P1 | ERX_P0);
   reg = write_spi_reg (W_REGISTER + SETUP_RETR, ARD4000 | ARC15);
   reg = write_spi_reg (W_REGISTER + RF_CH, 92);
   reg = write_spi_reg (W_REGISTER + RF_SETUP, RF_DR_LOW | RF_PWR_MED | 1);
   reg = write_spi_reg (W_REGISTER + CONFIG, EN_CRC | CRCO | PWR_UP | PTX);
   delay(1); // KEF confirmed needed!

 //  x = i2c_read_register (BMP180_ADDR, 0xd0);
 //  memset (tx_payload, 0, sizeof(tx_payload));
*/
  init_io();                        // Initialize IO port
  SE8R01_Init();
  write_spi_reg(FLUSH_RX,0);
  readstatus = read_spi_reg(CONFIG);
  UARTPrintF("config = \n\r");
  print_UCHAR_hex(readstatus);
  readstatus = read_spi_reg(STATUS);
  UARTPrintF("status = \n\r");
 print_UCHAR_hex(readstatus);




while (1) {
// Load the calibration data from the BMP180
//   for (a = 0xaa; a <= 0xbf; ++a) {
//      x = i2c_read_register (BMP180_ADDR, a);
//      tx_payload[a - 0xaa] = x;
//   }

   tx_payload[0] = 0xf0;
   tx_payload[1] = 0x01;

   

//   write_spi_reg (W_REGISTER + STATUS, RX_DR | TX_DS | MAX_RT);
   // reg = write_spi_reg (FLUSH_TX, 0);
 //  delayTenMicro();
//   reg = write_spi_buf (W_TX_PAYLOAD, tx_payload, 32);
   delayTenMicro();
   PC_ODR |= (1 << CE);
   delayTenMicro();
   PC_ODR &= ~(1 << CE);
//todo 



PD_DDR |= ~(1 << 5); // input mode
PD_CR1 |= (1 << 5); // input with pull up
PD_CR2 &= ~(1 << 5); // interrupt disabled


//if(digitalRead(IRQq)==LOW) PD_ODR = 0
if ((PD_IDR & 0b00001000) == 0b00001000)
{
  UARTPrintF("interrupt low = \n\r");

if(     ( readstatus & (RX_DR | TX_DS | MAX_RT) ) != 0  ){
read_spi_buf(RD_RX_PLOAD, tx_payload, 1);
for(i=0;i<32;i++) print_UCHAR_hex(tx_payload[i]); 
  UARTPrintF("data \n\r");
}
}
else
{
  UARTPrintF("interrupt high = \n\r");
} 
// Delay before looping back
   for (x1 = 0; x1 < 50; ++x1)
      for (y1 = 0; y1 < 50; ++y1)
         for (z1 = 0; z1 < 50; ++z1)
            __asm__("nop");

 readstatus = read_spi_reg(CONFIG);
  UARTPrintF("config = \n\r");
  print_UCHAR_hex(readstatus);
  readstatus = read_spi_reg(STATUS);
  UARTPrintF("status = \n\r");
 print_UCHAR_hex(readstatus);


}
}
