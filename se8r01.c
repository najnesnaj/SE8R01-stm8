/*
   connection se8r01 -- stm8s   (see also jpg pictures)


   1 VCC
   2 GND
   3 CE      PC4
   4 CSN     PC3
   5 SCK     PC5
   6 MOSI    PC6
   7 MISO    PC7
   8 IRQ     PD3
 */






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
unsigned char TX_ADDRESS[ADR_WIDTH]  =
{
	0xb3,0x43,0x10,0x10
}; // Define a static TX address




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



//SE8R01 transmitter part



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
	delayTenMicro();
	delayTenMicro();
	PC_ODR &= ~(1 << CE);
	delay(50);                            // delay 50ms waitting for calibaration.

	//       digitalWrite(CEq, 1); 
	//       delayMicroseconds(30);
	//       digitalWrite(CEq, 0); 
	delayTenMicro();
	PC_ODR |= (1 << CE);
	delayTenMicro();
	delayTenMicro();
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
	//
	write_spi_reg(WRITE_REG|iRF_BANK0_EN_AA, 0x01);          //enable auto acc on pip 1
	write_spi_reg(WRITE_REG|iRF_BANK0_EN_RXADDR, 0x01);      //enable pip 1
	write_spi_reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);        //4 byte adress
	write_spi_reg(WRITE_REG|iRF_BANK0_SETUP_RETR, 0x08);        //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit Delay
	write_spi_reg(WRITE_REG|iRF_BANK0_RF_CH, 40);
	write_spi_reg(WRITE_REG|iRF_BANK0_DYNPD, 0x01);          //pipe0 pipe1 enable dynamic payload length data
	write_spi_reg(WRITE_REG|iRF_BANK0_FEATURE, 0x07);        // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
	write_spi_reg(WRITE_REG + CONFIG, 0x3E);
	write_spi_buf(WRITE_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH);  //from tx

	write_spi_buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, ADR_WIDTH); // Use the same address on the RX device as the TX device write_spi_reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width


	/*
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
	 */
	PC_ODR |= (1 << CE);

}



int main () {
	short voltage = 1900;
	UCHAR rx_addr_p1[]  = { 0xd2, 0xf0, 0xf0, 0xf0, 0xf0 };
	UCHAR tx_addr[]     = { 0xe1, 0xf0, 0xf0, 0xf0, 0xf0 };
	UCHAR tx_payload[33];
	UCHAR readstatus;
	volatile int x1, y1, z1;
	InitializeSystemClock();
	InitializeUART();
	//   InitializeI2C();
	InitializeSPI ();


	memset (tx_payload, 0, sizeof(tx_payload));


	init_io();                        // Initialize IO port
	write_spi_reg(FLUSH_TX,0); // transmit -- send data 
	readstatus = read_spi_reg(CONFIG);
	UARTPrintF("config = \n\r");
	print_UCHAR_hex(readstatus);
	readstatus = read_spi_reg(STATUS);
	UARTPrintF("status = \n\r");
	print_UCHAR_hex(readstatus);

	SE8R01_Init();



	while (1) {
		//some testdata
		tx_payload[0] = 0xff;
		tx_payload[1] = 0xff;
		tx_payload[2] = 0x01;
		tx_payload[3] = 0x02;
		write_spi_buf(iRF_CMD_WR_TX_PLOAD, tx_payload, 4);
		write_spi_reg(WRITE_REG+STATUS, 0xff);
		// readstatus = read_spi_reg(STATUS);
		//  UARTPrintF("status = \n\r");
		//  print_UCHAR_hex(readstatus);
		//	readstatus=read_spi_reg(OBSERVE_TX); 
		//	print_UCHAR_hex(readstatus);
		// Delay before looping back
		for (x1 = 0; x1 < 50; ++x1)
			for (y1 = 0; y1 < 50; ++y1)
				for (z1 = 0; z1 < 50; ++z1)
					__asm__("nop");


	}
}
