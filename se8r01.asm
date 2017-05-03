;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jul 11 2014) (Linux)
; This file was generated Wed May  3 10:49:15 2017
;--------------------------------------------------------
	.module se8r01
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _SE8R01_Init
	.globl _SE8R01_Analog_Init
	.globl _SE8R01_Calibration
	.globl _rf_switch_bank
	.globl _init_io
	.globl _InitializeUART
	.globl _InitializeI2C
	.globl _i2c_read_register
	.globl _print_UCHAR_hex
	.globl _UARTPrintF
	.globl _i2c_set_start_ack
	.globl _i2c_send_address
	.globl _i2c_send_reg
	.globl _i2c_set_stop
	.globl _i2c_set_nak
	.globl _i2c_read
	.globl _delay
	.globl _InitializeSystemClock
	.globl _InitializeSPI
	.globl _read_spi_buf
	.globl _write_spi_buf
	.globl _read_spi_reg
	.globl _write_spi_reg
	.globl _write_spi
	.globl _delayTenMicro
	.globl _memset
	.globl _tx_buf
	.globl _rx_buf
	.globl _ADDRESS0
	.globl _ADDRESS1
	.globl _ADDRESS5
	.globl _ADDRESS4
	.globl _ADDRESS3
	.globl _ADDRESS2
	.globl _status
	.globl _pip
	.globl _signal_lv
	.globl _newdata
	.globl _pload_width_now
	.globl _SE8R01_DR_500K
	.globl _SE8R01_DR_1M
	.globl _SE8R01_DR_2M
	.globl _myData_pip4
	.globl _myData_pip5
	.globl _gtemp
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_gtemp::
	.ds 5
_myData_pip5::
	.ds 5
_myData_pip4::
	.ds 5
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
_SE8R01_DR_2M::
	.ds 2
_SE8R01_DR_1M::
	.ds 2
_SE8R01_DR_500K::
	.ds 2
_pload_width_now::
	.ds 2
_newdata::
	.ds 2
_signal_lv::
	.ds 1
_pip::
	.ds 2
_status::
	.ds 1
_ADDRESS2::
	.ds 1
_ADDRESS3::
	.ds 1
_ADDRESS4::
	.ds 1
_ADDRESS5::
	.ds 1
_ADDRESS1::
	.ds 4
_ADDRESS0::
	.ds 4
_rx_buf::
	.ds 32
_tx_buf::
	.ds 32
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ;reset
	int 0x0000 ;trap
	int 0x0000 ;int0
	int 0x0000 ;int1
	int 0x0000 ;int2
	int 0x0000 ;int3
	int 0x0000 ;int4
	int 0x0000 ;int5
	int 0x0000 ;int6
	int 0x0000 ;int7
	int 0x0000 ;int8
	int 0x0000 ;int9
	int 0x0000 ;int10
	int 0x0000 ;int11
	int 0x0000 ;int12
	int 0x0000 ;int13
	int 0x0000 ;int14
	int 0x0000 ;int15
	int 0x0000 ;int16
	int 0x0000 ;int17
	int 0x0000 ;int18
	int 0x0000 ;int19
	int 0x0000 ;int20
	int 0x0000 ;int21
	int 0x0000 ;int22
	int 0x0000 ;int23
	int 0x0000 ;int24
	int 0x0000 ;int25
	int 0x0000 ;int26
	int 0x0000 ;int27
	int 0x0000 ;int28
	int 0x0000 ;int29
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	se8r01.c: 46: void delayTenMicro (void) {
;	-----------------------------------------
;	 function delayTenMicro
;	-----------------------------------------
_delayTenMicro:
;	se8r01.c: 48: for (a = 0; a < 50; ++a)
	ld	a, #0x32
00104$:
;	se8r01.c: 49: __asm__("nop");
	nop
	dec	a
;	se8r01.c: 48: for (a = 0; a < 50; ++a)
	tnz	a
	jrne	00104$
	ret
;	se8r01.c: 51: UCHAR write_spi (UCHAR value) {
;	-----------------------------------------
;	 function write_spi
;	-----------------------------------------
_write_spi:
;	se8r01.c: 53: delayTenMicro ();
	call	_delayTenMicro
;	se8r01.c: 54: SPI_DR = value;
	ldw	x, #0x5204
	ld	a, (0x03, sp)
	ld	(x), a
;	se8r01.c: 55: delayTenMicro ();
	call	_delayTenMicro
;	se8r01.c: 56: while ((SPI_SR & TXE) == 0);
00101$:
	ldw	x, #0x5203
	ld	a, (x)
	bcp	a, #0x02
	jreq	00101$
;	se8r01.c: 57: delayTenMicro ();
	call	_delayTenMicro
;	se8r01.c: 58: while ((SPI_SR & RXNE) == 0);
00104$:
	ldw	x, #0x5203
	ld	a, (x)
	srl	a
	jrnc	00104$
;	se8r01.c: 59: delayTenMicro ();
	call	_delayTenMicro
;	se8r01.c: 60: ret = SPI_DR;
	ldw	x, #0x5204
	ld	a, (x)
;	se8r01.c: 61: return (ret);
	ret
;	se8r01.c: 63: UCHAR write_spi_reg (UCHAR reg, UCHAR value) {
;	-----------------------------------------
;	 function write_spi_reg
;	-----------------------------------------
_write_spi_reg:
	push	a
;	se8r01.c: 65: PC_ODR &= ~(1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	se8r01.c: 66: ret = write_spi (reg);
	ld	a, (0x04, sp)
	push	a
	call	_write_spi
	addw	sp, #1
	ld	(0x01, sp), a
;	se8r01.c: 67: if (reg != NOP && reg != FLUSH_RX && reg != FLUSH_TX)
	ld	a, (0x04, sp)
	cp	a, #0xff
	jreq	00102$
	ld	a, (0x04, sp)
	cp	a, #0xe2
	jreq	00102$
	ld	a, (0x04, sp)
	cp	a, #0xe1
	jreq	00102$
;	se8r01.c: 68: write_spi (value);
	ld	a, (0x05, sp)
	push	a
	call	_write_spi
	pop	a
	jra	00103$
00102$:
;	se8r01.c: 70: delayTenMicro ();
	call	_delayTenMicro
00103$:
;	se8r01.c: 71: PC_ODR |= (1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	se8r01.c: 72: return (ret);
	ld	a, (0x01, sp)
	addw	sp, #1
	ret
;	se8r01.c: 74: UCHAR read_spi_reg (UCHAR reg) {
;	-----------------------------------------
;	 function read_spi_reg
;	-----------------------------------------
_read_spi_reg:
	push	a
;	se8r01.c: 76: PC_ODR &= ~(1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	se8r01.c: 77: ret = write_spi (reg);
	ld	a, (0x04, sp)
	push	a
	call	_write_spi
	addw	sp, #1
	ld	(0x01, sp), a
;	se8r01.c: 78: if (reg != NOP && reg != FLUSH_RX && reg != FLUSH_TX)
	ld	a, (0x04, sp)
	cp	a, #0xff
	jreq	00102$
	ld	a, (0x04, sp)
	cp	a, #0xe2
	jreq	00102$
	ld	a, (0x04, sp)
	cp	a, #0xe1
	jreq	00102$
;	se8r01.c: 79: ret = write_spi (NOP);
	push	#0xff
	call	_write_spi
	addw	sp, #1
	ld	(0x01, sp), a
	jra	00103$
00102$:
;	se8r01.c: 81: delayTenMicro ();
	call	_delayTenMicro
00103$:
;	se8r01.c: 82: PC_ODR |= (1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	se8r01.c: 83: return (ret);
	ld	a, (0x01, sp)
	addw	sp, #1
	ret
;	se8r01.c: 85: UCHAR write_spi_buf (UCHAR reg, UCHAR *array, UCHAR len) {
;	-----------------------------------------
;	 function write_spi_buf
;	-----------------------------------------
_write_spi_buf:
	sub	sp, #2
;	se8r01.c: 87: PC_ODR &= ~(1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	se8r01.c: 88: ret = write_spi (reg);
	ld	a, (0x05, sp)
	push	a
	call	_write_spi
	addw	sp, #1
	ld	(0x02, sp), a
;	se8r01.c: 89: for (n = 0; n < len; ++n)
	clr	(0x01, sp)
00103$:
	ld	a, (0x01, sp)
	cp	a, (0x08, sp)
	jrnc	00101$
;	se8r01.c: 90: write_spi (array[n]);
	clrw	x
	ld	a, (0x01, sp)
	ld	xl, a
	addw	x, (0x06, sp)
	ld	a, (x)
	push	a
	call	_write_spi
	pop	a
;	se8r01.c: 89: for (n = 0; n < len; ++n)
	inc	(0x01, sp)
	jra	00103$
00101$:
;	se8r01.c: 91: PC_ODR |= (1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	se8r01.c: 92: return (ret);
	ld	a, (0x02, sp)
	addw	sp, #2
	ret
;	se8r01.c: 94: UCHAR read_spi_buf (UCHAR reg, UCHAR *array, UCHAR len) {
;	-----------------------------------------
;	 function read_spi_buf
;	-----------------------------------------
_read_spi_buf:
	sub	sp, #2
;	se8r01.c: 96: PC_ODR &= ~(1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	se8r01.c: 97: ret = write_spi (reg);
	ld	a, (0x05, sp)
	push	a
	call	_write_spi
	addw	sp, #1
	ld	(0x01, sp), a
;	se8r01.c: 98: for (n = 0; n < len; ++n)
	clr	(0x02, sp)
00103$:
	ld	a, (0x02, sp)
	cp	a, (0x08, sp)
	jrnc	00101$
;	se8r01.c: 99: array[n] = write_spi (NOP);
	clrw	x
	ld	a, (0x02, sp)
	ld	xl, a
	addw	x, (0x06, sp)
	pushw	x
	push	#0xff
	call	_write_spi
	addw	sp, #1
	popw	x
	ld	(x), a
;	se8r01.c: 98: for (n = 0; n < len; ++n)
	inc	(0x02, sp)
	jra	00103$
00101$:
;	se8r01.c: 100: PC_ODR |= (1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	se8r01.c: 101: return (ret);
	ld	a, (0x01, sp)
	addw	sp, #2
	ret
;	se8r01.c: 103: void InitializeSPI () {
;	-----------------------------------------
;	 function InitializeSPI
;	-----------------------------------------
_InitializeSPI:
;	se8r01.c: 104: SPI_CR1 = MSBFIRST | SPI_ENABLE | BR_DIV256 | MASTER | CPOL0 | CPHA0;
	ldw	x, #0x5200
	ld	a, #0x7c
	ld	(x), a
;	se8r01.c: 105: SPI_CR2 = BDM_2LINE | CRCEN_OFF | CRCNEXT_TXBUF | FULL_DUPLEX | SSM_DISABLE;
	ldw	x, #0x5201
	clr	(x)
;	se8r01.c: 106: SPI_ICR = TXIE_MASKED | RXIE_MASKED | ERRIE_MASKED | WKIE_MASKED;
	ldw	x, #0x5202
	clr	(x)
;	se8r01.c: 107: PC_DDR = (1 << PC3) | (1 << PC4); // output mode
	ldw	x, #0x500c
	ld	a, #0x18
	ld	(x), a
;	se8r01.c: 108: PC_CR1 = (1 << PC3) | (1 << PC4); // push-pull
	ldw	x, #0x500d
	ld	a, #0x18
	ld	(x), a
;	se8r01.c: 109: PC_CR2 = (1 << PC3) | (1 << PC4); // up to 10MHz speed
	ldw	x, #0x500e
	ld	a, #0x18
	ld	(x), a
;	se8r01.c: 111: PC_ODR &= ~(1 << CE);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
	ret
;	se8r01.c: 113: void InitializeSystemClock() {
;	-----------------------------------------
;	 function InitializeSystemClock
;	-----------------------------------------
_InitializeSystemClock:
;	se8r01.c: 114: CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
	ldw	x, #0x50c0
	clr	(x)
;	se8r01.c: 115: CLK_ICKR = CLK_HSIEN;               //  Enable the HSI.
	ldw	x, #0x50c0
	ld	a, #0x01
	ld	(x), a
;	se8r01.c: 116: CLK_ECKR = 0;                       //  Disable the external clock.
	ldw	x, #0x50c1
	clr	(x)
;	se8r01.c: 117: while ((CLK_ICKR & CLK_HSIRDY) == 0);       //  Wait for the HSI to be ready for use.
00101$:
	ldw	x, #0x50c0
	ld	a, (x)
	bcp	a, #0x02
	jreq	00101$
;	se8r01.c: 118: CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
	ldw	x, #0x50c6
	clr	(x)
;	se8r01.c: 119: CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
	ldw	x, #0x50c7
	ld	a, #0xff
	ld	(x), a
;	se8r01.c: 120: CLK_PCKENR2 = 0xff;                 //  Ditto.
	ldw	x, #0x50ca
	ld	a, #0xff
	ld	(x), a
;	se8r01.c: 121: CLK_CCOR = 0;                       //  Turn off CCO.
	ldw	x, #0x50c9
	clr	(x)
;	se8r01.c: 122: CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
	ldw	x, #0x50cc
	clr	(x)
;	se8r01.c: 123: CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
	ldw	x, #0x50cd
	clr	(x)
;	se8r01.c: 124: CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
	ldw	x, #0x50c4
	ld	a, #0xe1
	ld	(x), a
;	se8r01.c: 125: CLK_SWCR = 0;                       //  Reset the clock switch control register.
	ldw	x, #0x50c5
	clr	(x)
;	se8r01.c: 126: CLK_SWCR = CLK_SWEN;                //  Enable switching.
	ldw	x, #0x50c5
	ld	a, #0x02
	ld	(x), a
;	se8r01.c: 127: while ((CLK_SWCR & CLK_SWBSY) != 0);        //  Pause while the clock switch is busy.
00104$:
	ldw	x, #0x50c5
	ld	a, (x)
	srl	a
	jrc	00104$
	ret
;	se8r01.c: 129: void delay (int time_ms) {
;	-----------------------------------------
;	 function delay
;	-----------------------------------------
_delay:
	sub	sp, #10
;	se8r01.c: 131: for (x = 0; x < 1036*time_ms; ++x)
	clrw	x
	ldw	(0x03, sp), x
	ldw	(0x01, sp), x
	ldw	x, (0x0d, sp)
	pushw	x
	push	#0x0c
	push	#0x04
	call	__mulint
	addw	sp, #4
	ldw	(0x05, sp), x
00103$:
	ldw	y, (0x05, sp)
	ldw	(0x09, sp), y
	ld	a, (0x09, sp)
	rlc	a
	clr	a
	sbc	a, #0x00
	ld	(0x08, sp), a
	ld	(0x07, sp), a
	ldw	x, (0x03, sp)
	cpw	x, (0x09, sp)
	ld	a, (0x02, sp)
	sbc	a, (0x08, sp)
	ld	a, (0x01, sp)
	sbc	a, (0x07, sp)
	jrsge	00105$
;	se8r01.c: 132: __asm__("nop");
	nop
;	se8r01.c: 131: for (x = 0; x < 1036*time_ms; ++x)
	ldw	y, (0x03, sp)
	addw	y, #0x0001
	ld	a, (0x02, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x01, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x03, sp), y
	ldw	(0x01, sp), x
	jra	00103$
00105$:
	addw	sp, #10
	ret
;	se8r01.c: 134: void i2c_read (unsigned char *x) {
;	-----------------------------------------
;	 function i2c_read
;	-----------------------------------------
_i2c_read:
;	se8r01.c: 135: while ((I2C_SR1 & I2C_RXNE) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
;	se8r01.c: 136: *x = I2C_DR;
	ldw	y, (0x03, sp)
	ldw	x, #0x5216
	ld	a, (x)
	ld	(y), a
	ret
;	se8r01.c: 138: void i2c_set_nak (void) {
;	-----------------------------------------
;	 function i2c_set_nak
;	-----------------------------------------
_i2c_set_nak:
;	se8r01.c: 139: I2C_CR2 &= ~I2C_ACK;
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	se8r01.c: 141: void i2c_set_stop (void) {
;	-----------------------------------------
;	 function i2c_set_stop
;	-----------------------------------------
_i2c_set_stop:
;	se8r01.c: 142: I2C_CR2 |= I2C_STOP;
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
	ret
;	se8r01.c: 144: void i2c_send_reg (UCHAR addr) {
;	-----------------------------------------
;	 function i2c_send_reg
;	-----------------------------------------
_i2c_send_reg:
	sub	sp, #2
;	se8r01.c: 146: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	se8r01.c: 147: reg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	se8r01.c: 148: I2C_DR = addr;
	ldw	x, #0x5216
	ld	a, (0x05, sp)
	ld	(x), a
;	se8r01.c: 149: while ((I2C_SR1 & I2C_TXE) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	sll	a
	jrnc	00101$
	addw	sp, #2
	ret
;	se8r01.c: 151: void i2c_send_address (UCHAR addr, UCHAR mode) {
;	-----------------------------------------
;	 function i2c_send_address
;	-----------------------------------------
_i2c_send_address:
	sub	sp, #3
;	se8r01.c: 153: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	clrw	x
	ld	xl, a
	ldw	(0x01, sp), x
;	se8r01.c: 154: I2C_DR = (addr << 1) | mode;
	ld	a, (0x06, sp)
	sll	a
	or	a, (0x07, sp)
	ldw	x, #0x5216
	ld	(x), a
;	se8r01.c: 155: if (mode == I2C_READ) {
	ld	a, (0x07, sp)
	cp	a, #0x01
	jrne	00127$
	ld	a, #0x01
	ld	(0x03, sp), a
	jra	00128$
00127$:
	clr	(0x03, sp)
00128$:
	tnz	(0x03, sp)
	jreq	00103$
;	se8r01.c: 156: I2C_OARL = 0;
	ldw	x, #0x5213
	clr	(x)
;	se8r01.c: 157: I2C_OARH = 0;
	ldw	x, #0x5214
	clr	(x)
;	se8r01.c: 159: while ((I2C_SR1 & I2C_ADDR) == 0);
00103$:
;	se8r01.c: 153: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	se8r01.c: 159: while ((I2C_SR1 & I2C_ADDR) == 0);
	bcp	a, #0x02
	jreq	00103$
;	se8r01.c: 160: if (mode == I2C_READ)
	tnz	(0x03, sp)
	jreq	00108$
;	se8r01.c: 161: UNSET (I2C_SR1, I2C_ADDR);
	and	a, #0xfd
	ldw	x, #0x5217
	ld	(x), a
00108$:
	addw	sp, #3
	ret
;	se8r01.c: 163: void i2c_set_start_ack (void) {
;	-----------------------------------------
;	 function i2c_set_start_ack
;	-----------------------------------------
_i2c_set_start_ack:
;	se8r01.c: 164: I2C_CR2 = I2C_ACK | I2C_START;
	ldw	x, #0x5211
	ld	a, #0x05
	ld	(x), a
;	se8r01.c: 165: while ((I2C_SR1 & I2C_SB) == 0);
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	srl	a
	jrnc	00101$
	ret
;	se8r01.c: 170: void UARTPrintF (char *message) {
;	-----------------------------------------
;	 function UARTPrintF
;	-----------------------------------------
_UARTPrintF:
;	se8r01.c: 171: char *ch = message;
	ldw	y, (0x03, sp)
;	se8r01.c: 172: while (*ch) {
00104$:
	ld	a, (y)
	tnz	a
	jreq	00107$
;	se8r01.c: 173: UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
	ldw	x, #0x5231
	ld	(x), a
;	se8r01.c: 174: while ((UART1_SR & SR_TXE) == 0);   //  Wait for transmission to complete.
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	sll	a
	jrnc	00101$
;	se8r01.c: 175: ch++;                               //  Grab the next character.
	incw	y
	jra	00104$
00107$:
	ret
;	se8r01.c: 178: void print_UCHAR_hex (unsigned char buffer) {
;	-----------------------------------------
;	 function print_UCHAR_hex
;	-----------------------------------------
_print_UCHAR_hex:
	sub	sp, #12
;	se8r01.c: 181: a = (buffer >> 4);
	ld	a, (0x0f, sp)
	swap	a
	and	a, #0x0f
	clrw	x
	ld	xl, a
;	se8r01.c: 182: if (a > 9)
	cpw	x, #0x0009
	jrsle	00102$
;	se8r01.c: 183: a = a + 'a' - 10;
	addw	x, #0x0057
	ldw	(0x0b, sp), x
	jra	00103$
00102$:
;	se8r01.c: 185: a += '0'; 
	addw	x, #0x0030
	ldw	(0x0b, sp), x
00103$:
;	se8r01.c: 186: b = buffer & 0x0f;
	ld	a, (0x0f, sp)
	and	a, #0x0f
	clrw	x
	ld	xl, a
;	se8r01.c: 187: if (b > 9)
	cpw	x, #0x0009
	jrsle	00105$
;	se8r01.c: 188: b = b + 'a' - 10;
	addw	x, #0x0057
	ldw	(0x09, sp), x
	jra	00106$
00105$:
;	se8r01.c: 190: b += '0'; 
	addw	x, #0x0030
	ldw	(0x09, sp), x
00106$:
;	se8r01.c: 191: message[0] = a;
	ldw	y, sp
	incw	y
	ld	a, (0x0c, sp)
	ld	(y), a
;	se8r01.c: 192: message[1] = b;
	ldw	x, y
	incw	x
	ld	a, (0x0a, sp)
	ld	(x), a
;	se8r01.c: 193: message[2] = 0;
	ldw	x, y
	incw	x
	incw	x
	clr	(x)
;	se8r01.c: 194: UARTPrintF (message);
	pushw	y
	call	_UARTPrintF
	addw	sp, #2
	addw	sp, #12
	ret
;	se8r01.c: 196: unsigned char i2c_read_register (UCHAR addr, UCHAR rg) {
;	-----------------------------------------
;	 function i2c_read_register
;	-----------------------------------------
_i2c_read_register:
	sub	sp, #2
;	se8r01.c: 199: i2c_set_start_ack ();
	call	_i2c_set_start_ack
;	se8r01.c: 200: i2c_send_address (addr, I2C_WRITE);
	push	#0x00
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	se8r01.c: 201: i2c_send_reg (rg);
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_reg
	pop	a
;	se8r01.c: 202: i2c_set_start_ack ();
	call	_i2c_set_start_ack
;	se8r01.c: 203: i2c_send_address (addr, I2C_READ);
	push	#0x01
	ld	a, (0x06, sp)
	push	a
	call	_i2c_send_address
	addw	sp, #2
;	se8r01.c: 204: reg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	ld	(0x02, sp), a
;	se8r01.c: 205: reg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	ld	(0x02, sp), a
;	se8r01.c: 206: i2c_set_nak ();
	call	_i2c_set_nak
;	se8r01.c: 207: i2c_set_stop ();
	call	_i2c_set_stop
;	se8r01.c: 208: i2c_read (&x);
	ldw	x, sp
	incw	x
	pushw	x
	call	_i2c_read
	addw	sp, #2
;	se8r01.c: 209: return (x);
	ld	a, (0x01, sp)
	addw	sp, #2
	ret
;	se8r01.c: 212: void InitializeI2C (void) {
;	-----------------------------------------
;	 function InitializeI2C
;	-----------------------------------------
_InitializeI2C:
;	se8r01.c: 213: I2C_CR1 = 0;   //  Disable I2C before configuration starts. PE bit is bit 0
	ldw	x, #0x5210
	clr	(x)
;	se8r01.c: 217: I2C_FREQR = 16;                     //  Set the internal clock frequency (MHz).
	ldw	x, #0x5212
	ld	a, #0x10
	ld	(x), a
;	se8r01.c: 218: UNSET (I2C_CCRH, I2C_FS);           //  I2C running is standard mode.
	bres	0x521c, #7
;	se8r01.c: 219: I2C_CCRL = 0x10;                    //  SCL clock speed is 500 kHz.
	ldw	x, #0x521b
	ld	a, #0x10
	ld	(x), a
;	se8r01.c: 220: I2C_CCRH &= 0xf0;	// Clears lower 4 bits "CCR"
	ldw	x, #0x521c
	ld	a, (x)
	and	a, #0xf0
	ld	(x), a
;	se8r01.c: 224: UNSET (I2C_OARH, I2C_ADDMODE);      //  7 bit address mode.
	bres	0x5214, #7
;	se8r01.c: 225: SET (I2C_OARH, I2C_ADDCONF);        //  Docs say this must always be 1.
	ldw	x, #0x5214
	ld	a, (x)
	or	a, #0x40
	ld	(x), a
;	se8r01.c: 229: I2C_TRISER = 17;
	ldw	x, #0x521d
	ld	a, #0x11
	ld	(x), a
;	se8r01.c: 237: I2C_CR1 = I2C_PE;	// Enables port
	ldw	x, #0x5210
	ld	a, #0x01
	ld	(x), a
	ret
;	se8r01.c: 243: void InitializeUART() {
;	-----------------------------------------
;	 function InitializeUART
;	-----------------------------------------
_InitializeUART:
;	se8r01.c: 253: UART1_CR1 = 0;
	ldw	x, #0x5234
	clr	(x)
;	se8r01.c: 254: UART1_CR2 = 0;
	ldw	x, #0x5235
	clr	(x)
;	se8r01.c: 255: UART1_CR4 = 0;
	ldw	x, #0x5237
	clr	(x)
;	se8r01.c: 256: UART1_CR3 = 0;
	ldw	x, #0x5236
	clr	(x)
;	se8r01.c: 257: UART1_CR5 = 0;
	ldw	x, #0x5238
	clr	(x)
;	se8r01.c: 258: UART1_GTR = 0;
	ldw	x, #0x5239
	clr	(x)
;	se8r01.c: 259: UART1_PSCR = 0;
	ldw	x, #0x523a
	clr	(x)
;	se8r01.c: 263: UNSET (UART1_CR1, CR1_M);        //  8 Data bits.
	ldw	x, #0x5234
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	se8r01.c: 264: UNSET (UART1_CR1, CR1_PCEN);     //  Disable parity.
	ldw	x, #0x5234
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	se8r01.c: 265: UNSET (UART1_CR3, CR3_STOPH);    //  1 stop bit.
	ldw	x, #0x5236
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
;	se8r01.c: 266: UNSET (UART1_CR3, CR3_STOPL);    //  1 stop bit.
	ldw	x, #0x5236
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	se8r01.c: 267: UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
	ldw	x, #0x5233
	ld	a, #0x0a
	ld	(x), a
;	se8r01.c: 268: UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
	ldw	x, #0x5232
	ld	a, #0x08
	ld	(x), a
;	se8r01.c: 272: UNSET (UART1_CR2, CR2_TEN);      //  Disable transmit.
	ldw	x, #0x5235
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	se8r01.c: 273: UNSET (UART1_CR2, CR2_REN);      //  Disable receive.
	ldw	x, #0x5235
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	se8r01.c: 277: SET (UART1_CR3, CR3_CPOL);
	ldw	x, #0x5236
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	se8r01.c: 278: SET (UART1_CR3, CR3_CPHA);
	ldw	x, #0x5236
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
;	se8r01.c: 279: SET (UART1_CR3, CR3_LBCL);
	bset	0x5236, #0
;	se8r01.c: 283: SET (UART1_CR2, CR2_TEN);
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	se8r01.c: 284: SET (UART1_CR2, CR2_REN);
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	se8r01.c: 285: UART1_CR3 = CR3_CLKEN;
	ldw	x, #0x5236
	ld	a, #0x08
	ld	(x), a
	ret
;	se8r01.c: 353: void init_io(void)
;	-----------------------------------------
;	 function init_io
;	-----------------------------------------
_init_io:
;	se8r01.c: 355: PD_DDR &= ~(1 << 3); // input mode
	ldw	x, #0x5011
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	se8r01.c: 356: PD_CR1 |= (1 << 3); // input with pull up 
	ldw	x, #0x5012
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	se8r01.c: 357: PD_CR2 |= (1 << 3); // interrupt enabled 
	ldw	x, #0x5013
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	se8r01.c: 358: PD_ODR &= ~(1 << 3);
	ldw	x, #0x500f
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
;	se8r01.c: 361: PC_ODR &= ~(1 << CE);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	se8r01.c: 363: PC_ODR |= (1 << CSN);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	ret
;	se8r01.c: 369: void rf_switch_bank(unsigned char bankindex)
;	-----------------------------------------
;	 function rf_switch_bank
;	-----------------------------------------
_rf_switch_bank:
	push	a
;	se8r01.c: 372: temp1 = bankindex;
	ld	a, (0x04, sp)
	ld	(0x01, sp), a
;	se8r01.c: 374: temp0 = write_spi(iRF_BANK0_STATUS);
	push	#0x07
	call	_write_spi
	addw	sp, #1
;	se8r01.c: 376: if((temp0&0x80)!=temp1)
	and	a, #0x80
	cp	a, (0x01, sp)
	jreq	00103$
;	se8r01.c: 378: write_spi_reg(iRF_CMD_ACTIVATE,0x53);
	push	#0x53
	push	#0x50
	call	_write_spi_reg
	addw	sp, #2
00103$:
	pop	a
	ret
;	se8r01.c: 385: void SE8R01_Calibration()
;	-----------------------------------------
;	 function SE8R01_Calibration
;	-----------------------------------------
_SE8R01_Calibration:
	sub	sp, #13
;	se8r01.c: 388: rf_switch_bank(iBANK0);
	push	#0x00
	call	_rf_switch_bank
	pop	a
;	se8r01.c: 389: temp[0]=0x03;
	ldw	x, sp
	incw	x
	ldw	(0x06, sp), x
	ldw	x, (0x06, sp)
	ld	a, #0x03
	ld	(x), a
;	se8r01.c: 390: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG,temp, 1);
	ldw	x, (0x06, sp)
	push	#0x01
	pushw	x
	push	#0x20
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 392: temp[0]=0x32;
	ldw	x, (0x06, sp)
	ld	a, #0x32
	ld	(x), a
;	se8r01.c: 394: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_CH, temp,1);
	ldw	x, (0x06, sp)
	push	#0x01
	pushw	x
	push	#0x25
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 398: if (SE8R01_DR_2M==1)
	ldw	x, _SE8R01_DR_2M+0
	cpw	x, #0x0001
	jrne	00105$
;	se8r01.c: 399: {temp[0]=0x48;}
	ldw	x, (0x06, sp)
	ld	a, #0x48
	ld	(x), a
	jra	00106$
00105$:
;	se8r01.c: 400: else if (SE8R01_DR_1M==1)
	ldw	x, _SE8R01_DR_1M+0
	cpw	x, #0x0001
	jrne	00102$
;	se8r01.c: 401: {temp[0]=0x40;}
	ldw	x, (0x06, sp)
	ld	a, #0x40
	ld	(x), a
	jra	00106$
00102$:
;	se8r01.c: 403: {temp[0]=0x68;}   
	ldw	x, (0x06, sp)
	ld	a, #0x68
	ld	(x), a
00106$:
;	se8r01.c: 405: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,temp,1);
	ldw	x, (0x06, sp)
	push	#0x01
	pushw	x
	push	#0x26
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 406: temp[0]=0x77;
	ldw	x, (0x06, sp)
	ld	a, #0x77
	ld	(x), a
;	se8r01.c: 407: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD, temp,1);
	ldw	x, (0x06, sp)
	push	#0x01
	pushw	x
	push	#0x3f
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 409: rf_switch_bank(iBANK1);
	push	#0x80
	call	_rf_switch_bank
	pop	a
;	se8r01.c: 410: temp[0]=0x40;
	ldw	x, (0x06, sp)
	ld	a, #0x40
	ld	(x), a
;	se8r01.c: 411: temp[1]=0x00;
	ldw	x, (0x06, sp)
	incw	x
	ldw	(0x0a, sp), x
	ldw	x, (0x0a, sp)
	clr	(x)
;	se8r01.c: 412: temp[2]=0x10;
	ldw	x, (0x06, sp)
	incw	x
	incw	x
	ldw	(0x08, sp), x
	ldw	x, (0x08, sp)
	ld	a, #0x10
	ld	(x), a
;	se8r01.c: 414: {temp[3]=0xE6;}
	ldw	x, (0x06, sp)
	addw	x, #0x0003
	ldw	(0x0c, sp), x
;	se8r01.c: 413: if (SE8R01_DR_2M==1)
	ldw	x, _SE8R01_DR_2M+0
	cpw	x, #0x0001
	jrne	00108$
;	se8r01.c: 414: {temp[3]=0xE6;}
	ldw	x, (0x0c, sp)
	ld	a, #0xe6
	ld	(x), a
	jra	00109$
00108$:
;	se8r01.c: 416: {temp[3]=0xE4;}
	ldw	x, (0x0c, sp)
	ld	a, #0xe4
	ld	(x), a
00109$:
;	se8r01.c: 418: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, temp, 4);
	ldw	x, (0x06, sp)
	push	#0x04
	pushw	x
	push	#0x21
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 420: temp[0]=0x20;
	ldw	x, (0x06, sp)
	ld	a, #0x20
	ld	(x), a
;	se8r01.c: 421: temp[1]=0x08;
	ldw	x, (0x0a, sp)
	ld	a, #0x08
	ld	(x), a
;	se8r01.c: 422: temp[2]=0x50;
	ldw	x, (0x08, sp)
	ld	a, #0x50
	ld	(x), a
;	se8r01.c: 423: temp[3]=0x40;
	ldw	x, (0x0c, sp)
	ld	a, #0x40
	ld	(x), a
;	se8r01.c: 424: temp[4]=0x50;
	ldw	x, (0x06, sp)
	ld	a, #0x50
	ld	(0x0004, x), a
;	se8r01.c: 425: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, temp, 5);
	ldw	x, (0x06, sp)
	push	#0x05
	pushw	x
	push	#0x23
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 427: temp[0]=0x00;
	ldw	x, (0x06, sp)
	clr	(x)
;	se8r01.c: 428: temp[1]=0x00;
	ldw	x, (0x0a, sp)
	clr	(x)
;	se8r01.c: 429: if (SE8R01_DR_2M==1)
	ldw	x, _SE8R01_DR_2M+0
	cpw	x, #0x0001
	jrne	00111$
;	se8r01.c: 430: { temp[2]=0x1E;}
	ldw	x, (0x08, sp)
	ld	a, #0x1e
	ld	(x), a
	jra	00112$
00111$:
;	se8r01.c: 432: { temp[2]=0x1F;}
	ldw	x, (0x08, sp)
	ld	a, #0x1f
	ld	(x), a
00112$:
;	se8r01.c: 434: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, temp, 3);
	ldw	x, (0x06, sp)
	push	#0x03
	pushw	x
	push	#0x2a
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 436: if (SE8R01_DR_2M==1)
	ldw	x, _SE8R01_DR_2M+0
	cpw	x, #0x0001
	jrne	00114$
;	se8r01.c: 437: { temp[0]=0x29;}
	ldw	x, (0x06, sp)
	ld	a, #0x29
	ld	(x), a
	jra	00115$
00114$:
;	se8r01.c: 439: { temp[0]=0x14;}
	ldw	x, (0x06, sp)
	ld	a, #0x14
	ld	(x), a
00115$:
;	se8r01.c: 441: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, temp, 1);
	ldw	x, (0x06, sp)
	push	#0x01
	pushw	x
	push	#0x2c
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 443: temp[0]=0x00;
	ldw	x, (0x06, sp)
	clr	(x)
;	se8r01.c: 444: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW,temp,1);
	ldw	x, (0x06, sp)
	push	#0x01
	pushw	x
	push	#0x37
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 446: temp[0]=0x7F;
	ldw	x, (0x06, sp)
	ld	a, #0x7f
	ld	(x), a
;	se8r01.c: 447: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI,temp,1);
	ldw	x, (0x06, sp)
	push	#0x01
	pushw	x
	push	#0x38
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 449: temp[0]=0x02;
	ldw	x, (0x06, sp)
	ld	a, #0x02
	ld	(x), a
;	se8r01.c: 450: temp[1]=0xC1;
	ldw	x, (0x0a, sp)
	ld	a, #0xc1
	ld	(x), a
;	se8r01.c: 451: temp[2]=0xEB;            
	ldw	x, (0x08, sp)
	ld	a, #0xeb
	ld	(x), a
;	se8r01.c: 452: temp[3]=0x1C;
	ldw	x, (0x0c, sp)
	ld	a, #0x1c
	ld	(x), a
;	se8r01.c: 453: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, temp,4);
	ldw	x, (0x06, sp)
	push	#0x04
	pushw	x
	push	#0x3d
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 455: temp[0]=0x97;
	ldw	x, (0x06, sp)
	ld	a, #0x97
	ld	(x), a
;	se8r01.c: 456: temp[1]=0x64;
	ldw	x, (0x0a, sp)
	ld	a, #0x64
	ld	(x), a
;	se8r01.c: 457: temp[2]=0x00;
	ldw	x, (0x08, sp)
	clr	(x)
;	se8r01.c: 458: temp[3]=0x81;
	ldw	x, (0x0c, sp)
	ld	a, #0x81
	ld	(x), a
;	se8r01.c: 459: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, temp, 4);
	ldw	x, (0x06, sp)
	push	#0x04
	pushw	x
	push	#0x3e
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 460: rf_switch_bank(iBANK0);
	push	#0x00
	call	_rf_switch_bank
	pop	a
;	se8r01.c: 465: delayTenMicro();
	call	_delayTenMicro
;	se8r01.c: 466: PC_ODR |= (1 << CE);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	se8r01.c: 467: delayTenMicro();
	call	_delayTenMicro
;	se8r01.c: 468: PC_ODR &= ~(1 << CE);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	se8r01.c: 469: delay(50);                            // delay 50ms waitting for calibaration.
	push	#0x32
	push	#0x00
	call	_delay
	addw	sp, #2
;	se8r01.c: 474: delayTenMicro();
	call	_delayTenMicro
;	se8r01.c: 475: PC_ODR |= (1 << CE);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
;	se8r01.c: 476: delayTenMicro();
	call	_delayTenMicro
;	se8r01.c: 477: PC_ODR &= ~(1 << CE);
	ldw	x, #0x500a
	ld	a, (x)
	and	a, #0xef
	ld	(x), a
;	se8r01.c: 478: delay(50);                            // delay 50ms waitting for calibaration.
	push	#0x32
	push	#0x00
	call	_delay
	addw	sp, #2
	addw	sp, #13
	ret
;	se8r01.c: 482: void SE8R01_Analog_Init()           //SE8R01 初始化
;	-----------------------------------------
;	 function SE8R01_Analog_Init
;	-----------------------------------------
_SE8R01_Analog_Init:
	sub	sp, #21
;	se8r01.c: 487: gtemp[0]=0x28;
	ldw	x, #_gtemp+0
	ldw	(0x0e, sp), x
	ldw	x, (0x0e, sp)
	ld	a, #0x28
	ld	(x), a
;	se8r01.c: 488: gtemp[1]=0x32;
	ldw	x, (0x0e, sp)
	incw	x
	ldw	(0x0c, sp), x
	ldw	x, (0x0c, sp)
	ld	a, #0x32
	ld	(x), a
;	se8r01.c: 489: gtemp[2]=0x80;
	ldw	x, (0x0e, sp)
	incw	x
	incw	x
	ldw	(0x0a, sp), x
	ldw	x, (0x0a, sp)
	ld	a, #0x80
	ld	(x), a
;	se8r01.c: 490: gtemp[3]=0x90;
	ldw	x, (0x0e, sp)
	addw	x, #0x0003
	ldw	(0x08, sp), x
	ldw	x, (0x08, sp)
	ld	a, #0x90
	ld	(x), a
;	se8r01.c: 491: gtemp[4]=0x00;
	ldw	x, (0x0e, sp)
	addw	x, #0x0004
	clr	(x)
;	se8r01.c: 492: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);
	ldw	x, (0x0e, sp)
	push	#0x05
	pushw	x
	push	#0x3e
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 493: delay(2);
	push	#0x02
	push	#0x00
	call	_delay
	addw	sp, #2
;	se8r01.c: 496: rf_switch_bank(iBANK1);
	push	#0x80
	call	_rf_switch_bank
	pop	a
;	se8r01.c: 498: temp[0]=0x40;
	ldw	x, sp
	incw	x
	ldw	(0x14, sp), x
	ldw	x, (0x14, sp)
	ld	a, #0x40
	ld	(x), a
;	se8r01.c: 499: temp[1]=0x01;               
	ldw	x, (0x14, sp)
	incw	x
	ldw	(0x12, sp), x
	ldw	x, (0x12, sp)
	ld	a, #0x01
	ld	(x), a
;	se8r01.c: 500: temp[2]=0x30;               
	ldw	x, (0x14, sp)
	incw	x
	incw	x
	ldw	(0x10, sp), x
	ldw	x, (0x10, sp)
	ld	a, #0x30
	ld	(x), a
;	se8r01.c: 502: { temp[3]=0xE2; }              
	ldw	x, (0x14, sp)
	addw	x, #0x0003
	ldw	(0x06, sp), x
;	se8r01.c: 501: if (SE8R01_DR_2M==1)
	ldw	x, _SE8R01_DR_2M+0
	cpw	x, #0x0001
	jrne	00102$
;	se8r01.c: 502: { temp[3]=0xE2; }              
	ldw	x, (0x06, sp)
	ld	a, #0xe2
	ld	(x), a
	jra	00103$
00102$:
;	se8r01.c: 504: { temp[3]=0xE0;}
	ldw	x, (0x06, sp)
	ld	a, #0xe0
	ld	(x), a
00103$:
;	se8r01.c: 506: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, temp,4);
	ldw	x, (0x14, sp)
	push	#0x04
	pushw	x
	push	#0x21
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 508: temp[0]=0x29;
	ldw	x, (0x14, sp)
	ld	a, #0x29
	ld	(x), a
;	se8r01.c: 509: temp[1]=0x89;
	ldw	x, (0x12, sp)
	ld	a, #0x89
	ld	(x), a
;	se8r01.c: 510: temp[2]=0x55;                     
	ldw	x, (0x10, sp)
	ld	a, #0x55
	ld	(x), a
;	se8r01.c: 511: temp[3]=0x40;
	ldw	x, (0x06, sp)
	ld	a, #0x40
	ld	(x), a
;	se8r01.c: 512: temp[4]=0x50;
	ldw	x, (0x14, sp)
	ld	a, #0x50
	ld	(0x0004, x), a
;	se8r01.c: 513: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, temp,5);
	ldw	x, (0x14, sp)
	push	#0x05
	pushw	x
	push	#0x23
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 515: if (SE8R01_DR_2M==1)
	ldw	x, _SE8R01_DR_2M+0
	cpw	x, #0x0001
	jrne	00105$
;	se8r01.c: 516: { temp[0]=0x29;}
	ldw	x, (0x14, sp)
	ld	a, #0x29
	ld	(x), a
	jra	00106$
00105$:
;	se8r01.c: 518: { temp[0]=0x14;}
	ldw	x, (0x14, sp)
	ld	a, #0x14
	ld	(x), a
00106$:
;	se8r01.c: 520: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, temp,1);
	ldw	x, (0x14, sp)
	push	#0x01
	pushw	x
	push	#0x2c
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 522: temp[0]=0x55;
	ldw	x, (0x14, sp)
	ld	a, #0x55
	ld	(x), a
;	se8r01.c: 523: temp[1]=0xC2;
	ldw	x, (0x12, sp)
	ld	a, #0xc2
	ld	(x), a
;	se8r01.c: 524: temp[2]=0x09;
	ldw	x, (0x10, sp)
	ld	a, #0x09
	ld	(x), a
;	se8r01.c: 525: temp[3]=0xAC;  
	ldw	x, (0x06, sp)
	ld	a, #0xac
	ld	(x), a
;	se8r01.c: 526: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RX_CTRL,temp,4);
	ldw	x, (0x14, sp)
	push	#0x04
	pushw	x
	push	#0x31
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 528: temp[0]=0x00;
	ldw	x, (0x14, sp)
	clr	(x)
;	se8r01.c: 529: temp[1]=0x14;
	ldw	x, (0x12, sp)
	ld	a, #0x14
	ld	(x), a
;	se8r01.c: 530: temp[2]=0x08;   
	ldw	x, (0x10, sp)
	ld	a, #0x08
	ld	(x), a
;	se8r01.c: 531: temp[3]=0x29;
	ldw	x, (0x06, sp)
	ld	a, #0x29
	ld	(x), a
;	se8r01.c: 532: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FAGC_CTRL_1, temp,4);
	ldw	x, (0x14, sp)
	push	#0x04
	pushw	x
	push	#0x33
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 534: temp[0]=0x02;
	ldw	x, (0x14, sp)
	ld	a, #0x02
	ld	(x), a
;	se8r01.c: 535: temp[1]=0xC1;
	ldw	x, (0x12, sp)
	ld	a, #0xc1
	ld	(x), a
;	se8r01.c: 536: temp[2]=0xCB;  
	ldw	x, (0x10, sp)
	ld	a, #0xcb
	ld	(x), a
;	se8r01.c: 537: temp[3]=0x1C;
	ldw	x, (0x06, sp)
	ld	a, #0x1c
	ld	(x), a
;	se8r01.c: 538: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, temp,4);
	ldw	x, (0x14, sp)
	push	#0x04
	pushw	x
	push	#0x3d
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 540: temp[0]=0x97;
	ldw	x, (0x14, sp)
	ld	a, #0x97
	ld	(x), a
;	se8r01.c: 541: temp[1]=0x64;
	ldw	x, (0x12, sp)
	ld	a, #0x64
	ld	(x), a
;	se8r01.c: 542: temp[2]=0x00;
	ldw	x, (0x10, sp)
	clr	(x)
;	se8r01.c: 543: temp[3]=0x01;
	ldw	x, (0x06, sp)
	ld	a, #0x01
	ld	(x), a
;	se8r01.c: 544: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, temp,4);
	ldw	x, (0x14, sp)
	push	#0x04
	pushw	x
	push	#0x3e
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 546: gtemp[0]=0x2A;
	ldw	x, (0x0e, sp)
	ld	a, #0x2a
	ld	(x), a
;	se8r01.c: 547: gtemp[1]=0x04;
	ldw	x, (0x0c, sp)
	ld	a, #0x04
	ld	(x), a
;	se8r01.c: 548: gtemp[2]=0x00;
	ldw	x, (0x0a, sp)
	clr	(x)
;	se8r01.c: 549: gtemp[3]=0x7D;
	ldw	x, (0x08, sp)
	ld	a, #0x7d
	ld	(x), a
;	se8r01.c: 550: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK1_TEST_PKDET, gtemp, 4);
	ldw	x, (0x0e, sp)
	push	#0x04
	pushw	x
	push	#0x3f
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 552: rf_switch_bank(iBANK0);
	push	#0x00
	call	_rf_switch_bank
	pop	a
	addw	sp, #21
	ret
;	se8r01.c: 555: void SE8R01_Init()  
;	-----------------------------------------
;	 function SE8R01_Init
;	-----------------------------------------
_SE8R01_Init:
	sub	sp, #5
;	se8r01.c: 558: SE8R01_Calibration();   
	call	_SE8R01_Calibration
;	se8r01.c: 559: SE8R01_Analog_Init();   
	call	_SE8R01_Analog_Init
;	se8r01.c: 563: if (SE8R01_DR_2M==1)
	ldw	x, _SE8R01_DR_2M+0
	cpw	x, #0x0001
	jrne	00105$
;	se8r01.c: 564: {  temp[0]=0b01001111; }     //2MHz,+5dbm
	ldw	x, sp
	incw	x
	ld	a, #0x4f
	ld	(x), a
	jra	00106$
00105$:
;	se8r01.c: 565: else if  (SE8R01_DR_1M==1)
	ldw	x, _SE8R01_DR_1M+0
	cpw	x, #0x0001
	jrne	00102$
;	se8r01.c: 566: {  temp[0]=0b01000111;  }     //1MHz,+5dbm
	ldw	x, sp
	incw	x
	ld	a, #0x47
	ld	(x), a
	jra	00106$
00102$:
;	se8r01.c: 568: {temp[0]=0b01101111;  }     //500K,+5dbm
	ldw	x, sp
	incw	x
	ld	a, #0x6f
	ld	(x), a
00106$:
;	se8r01.c: 570: write_spi_buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,temp,1);
	ldw	x, sp
	incw	x
	push	#0x01
	pushw	x
	push	#0x26
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 574: write_spi_reg(WRITE_REG|iRF_BANK0_EN_AA, 0b00111111);          //enable auto acc on pip 1
	push	#0x3f
	push	#0x21
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 575: write_spi_reg(WRITE_REG|iRF_BANK0_EN_RXADDR, 0b00111111);      //enable pip 1
	push	#0x3f
	push	#0x22
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 576: write_spi_reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);  
	push	#0x02
	push	#0x23
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 577: write_spi_reg(WRITE_REG|iRF_BANK0_RF_CH, 40);
	push	#0x28
	push	#0x25
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 579: write_spi_buf(WRITE_REG + TX_ADDR, ADDRESS0, ADR_WIDTH);    	
	ldw	x, #_ADDRESS0+0
	ldw	y, x
	pushw	x
	push	#0x04
	pushw	y
	push	#0x30
	call	_write_spi_buf
	addw	sp, #4
	popw	x
;	se8r01.c: 580: write_spi_buf(WRITE_REG + RX_ADDR_P0, ADDRESS0, ADR_WIDTH); 
	push	#0x04
	pushw	x
	push	#0x2a
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 581: write_spi_buf(WRITE_REG + RX_ADDR_P1, ADDRESS1, ADR_WIDTH); 
	ldw	x, #_ADDRESS1+0
	push	#0x04
	pushw	x
	push	#0x2b
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 582: write_spi_buf(WRITE_REG + RX_ADDR_P2, ADDRESS2, 1); 
	ldw	x, #_ADDRESS2+0
	push	#0x01
	pushw	x
	push	#0x2c
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 583: write_spi_buf(WRITE_REG + RX_ADDR_P3, ADDRESS3, 1); 
	ldw	x, #_ADDRESS3+0
	push	#0x01
	pushw	x
	push	#0x2d
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 584: write_spi_buf(WRITE_REG + RX_ADDR_P4, ADDRESS4, 1); 
	ldw	x, #_ADDRESS4+0
	push	#0x01
	pushw	x
	push	#0x2e
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 585: write_spi_buf(WRITE_REG + RX_ADDR_P5, ADDRESS5, 1); 
	ldw	x, #_ADDRESS5+0
	push	#0x01
	pushw	x
	push	#0x2f
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 586: write_spi_reg(WRITE_REG + RX_PW_P0, PLOAD_WIDTH); 
	push	#0x20
	push	#0x31
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 587: write_spi_reg(WRITE_REG|iRF_BANK0_CONFIG, 0x3f); 
	push	#0x3f
	push	#0x20
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 588: write_spi_reg(WRITE_REG|iRF_BANK0_DYNPD, 0b00111111);          // enable dynamic payload length data
	push	#0x3f
	push	#0x3c
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 589: write_spi_reg(WRITE_REG|iRF_BANK0_FEATURE, 0x07);        // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
	push	#0x07
	push	#0x3d
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 593: PC_ODR |= (1 << CE);
	ldw	x, #0x500a
	ld	a, (x)
	or	a, #0x10
	ld	(x), a
	addw	sp, #5
	ret
;	se8r01.c: 601: int main () {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #51
;	se8r01.c: 603: UCHAR rx_addr_p1[]  = { 0xd2, 0xf0, 0xf0, 0xf0, 0xf0 };
	ldw	y, sp
	addw	y, #34
	ld	a, #0xd2
	ld	(y), a
	ldw	x, y
	incw	x
	ld	a, #0xf0
	ld	(x), a
	ldw	x, y
	incw	x
	incw	x
	ld	a, #0xf0
	ld	(x), a
	ldw	x, y
	ld	a, #0xf0
	ld	(0x0003, x), a
	ldw	x, y
	ld	a, #0xf0
	ld	(0x0004, x), a
;	se8r01.c: 604: UCHAR tx_addr[]     = { 0xe1, 0xf0, 0xf0, 0xf0, 0xf0 };
	ldw	y, sp
	addw	y, #39
	ld	a, #0xe1
	ld	(y), a
	ldw	x, y
	incw	x
	ld	a, #0xf0
	ld	(x), a
	ldw	x, y
	incw	x
	incw	x
	ld	a, #0xf0
	ld	(x), a
	ldw	x, y
	ld	a, #0xf0
	ld	(0x0003, x), a
	ldw	x, y
	addw	x, #0x0004
	ld	a, #0xf0
	ld	(x), a
;	se8r01.c: 608: InitializeSystemClock();
	call	_InitializeSystemClock
;	se8r01.c: 609: InitializeUART();
	call	_InitializeUART
;	se8r01.c: 611: InitializeSPI ();
	call	_InitializeSPI
;	se8r01.c: 614: memset (tx_payload, 0, sizeof(tx_payload));
	ldw	x, sp
	incw	x
	ldw	(0x32, sp), x
	ldw	y, (0x32, sp)
	push	#0x21
	push	#0x00
	clrw	x
	pushw	x
	pushw	y
	call	_memset
	addw	sp, #6
;	se8r01.c: 617: init_io();                        // Initialize IO port
	call	_init_io
;	se8r01.c: 618: write_spi_reg(FLUSH_TX,0); // transmit -- send data 
	push	#0x00
	push	#0xe1
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 619: readstatus = read_spi_reg(CONFIG);
	push	#0x00
	call	_read_spi_reg
	addw	sp, #1
;	se8r01.c: 620: UARTPrintF("config = \n\r");
	ldw	x, #___str_0+0
	push	a
	pushw	x
	call	_UARTPrintF
	addw	sp, #2
	pop	a
;	se8r01.c: 621: print_UCHAR_hex(readstatus);
	push	a
	call	_print_UCHAR_hex
	pop	a
;	se8r01.c: 622: readstatus = read_spi_reg(STATUS);
	push	#0x07
	call	_read_spi_reg
	addw	sp, #1
;	se8r01.c: 623: UARTPrintF("status = \n\r");
	ldw	x, #___str_1+0
	push	a
	pushw	x
	call	_UARTPrintF
	addw	sp, #2
	pop	a
;	se8r01.c: 624: print_UCHAR_hex(readstatus);
	push	a
	call	_print_UCHAR_hex
	pop	a
;	se8r01.c: 626: SE8R01_Init();
	call	_SE8R01_Init
;	se8r01.c: 630: while (1) {
00105$:
;	se8r01.c: 632: tx_payload[0] = 0xf0;
	ldw	x, (0x32, sp)
	ld	a, #0xf0
	ld	(x), a
;	se8r01.c: 633: tx_payload[1] = 0x01;
	ldw	x, (0x32, sp)
	incw	x
	ld	a, #0x01
	ld	(x), a
;	se8r01.c: 634: tx_payload[2] = 0x33;
	ldw	x, (0x32, sp)
	incw	x
	incw	x
	ld	a, #0x33
	ld	(x), a
;	se8r01.c: 635: write_spi_buf(iRF_CMD_WR_TX_PLOAD, tx_payload, 32);
	ldw	x, (0x32, sp)
	push	#0x20
	pushw	x
	push	#0xa0
	call	_write_spi_buf
	addw	sp, #4
;	se8r01.c: 636: write_spi_reg(WRITE_REG+STATUS, 0xff);
	push	#0xff
	push	#0x27
	call	_write_spi_reg
	addw	sp, #2
;	se8r01.c: 638: for (x1 = 0; x1 < 50; ++x1)
	clrw	x
	ldw	(0x30, sp), x
00114$:
	ldw	x, (0x30, sp)
	cpw	x, #0x0032
	jrsge	00105$
;	se8r01.c: 639: for (y1 = 0; y1 < 50; ++y1)
	clrw	x
	ldw	(0x2e, sp), x
00111$:
	ldw	x, (0x2e, sp)
	cpw	x, #0x0032
	jrsge	00115$
;	se8r01.c: 640: for (z1 = 0; z1 < 50; ++z1)
	clrw	x
	ldw	(0x2c, sp), x
00108$:
	ldw	x, (0x2c, sp)
	cpw	x, #0x0032
	jrsge	00112$
;	se8r01.c: 641: __asm__("nop");
	nop
;	se8r01.c: 640: for (z1 = 0; z1 < 50; ++z1)
	ldw	x, (0x2c, sp)
	incw	x
	ldw	(0x2c, sp), x
	jra	00108$
00112$:
;	se8r01.c: 639: for (y1 = 0; y1 < 50; ++y1)
	ldw	x, (0x2e, sp)
	incw	x
	ldw	(0x2e, sp), x
	jra	00111$
00115$:
;	se8r01.c: 638: for (x1 = 0; x1 < 50; ++x1)
	ldw	x, (0x30, sp)
	incw	x
	ldw	(0x30, sp), x
	jra	00114$
	addw	sp, #51
	ret
	.area CODE
___str_0:
	.ascii "config = "
	.db 0x0A
	.db 0x0D
	.db 0x00
___str_1:
	.ascii "status = "
	.db 0x0A
	.db 0x0D
	.db 0x00
	.area INITIALIZER
__xinit__SE8R01_DR_2M:
	.dw #0x0000
__xinit__SE8R01_DR_1M:
	.dw #0x0000
__xinit__SE8R01_DR_500K:
	.dw #0x0001
__xinit__pload_width_now:
	.dw #0x0000
__xinit__newdata:
	.dw #0x0000
__xinit__signal_lv:
	.db #0x00	;  0
__xinit__pip:
	.dw #0x0000
__xinit__status:
	.db #0x00	; 0
__xinit__ADDRESS2:
	.db #0xB1	; 177
__xinit__ADDRESS3:
	.db #0xB2	; 178
__xinit__ADDRESS4:
	.db #0xB3	; 179
__xinit__ADDRESS5:
	.db #0xB4	; 180
__xinit__ADDRESS1:
	.db #0xB0	; 176
	.db #0x43	; 67	'C'
	.db #0x10	; 16
	.db #0x10	; 16
__xinit__ADDRESS0:
	.db #0x34	; 52	'4'
	.db #0x43	; 67	'C'
	.db #0x10	; 16
	.db #0x10	; 16
__xinit__rx_buf:
	.db #0x00	; 0
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
__xinit__tx_buf:
	.db #0x00	; 0
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.db 0x00
	.area CABS (ABS)
