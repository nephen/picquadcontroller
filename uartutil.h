#ifndef UARTUTIL_H
#define UARTUTIL_H
//---------------------------------------------------------------------
// UART
//---------------------------------------------------------------------

//specify that we want printf() to redirect to UART1
//int __C30_UART = 1;

#define UART_BUFFER_SIZE 256
unsigned char uart_buffer[UART_BUFFER_SIZE] __attribute__((space(dma)));
unsigned char uart_buffering  = 0;
unsigned int uart_buffering_sent = 0;	//number of chars sent while in buffering mode (for debug purposes)
volatile unsigned char uart_buffering_busy  = 0;
volatile unsigned int uart_buffer_pos = 0;

void uart_init(){
	//dsPIC33FJ* does not have dedicated ports for UART, we need to assign the pin's we're going to use



	//setup remapable pins for UART function
	_TRIS(pinTX) = 1;


	__builtin_write_OSCCONL(OSCCON & ~(1<<6));		//unlock registers
	RPOR12bits.RP24R = 0b00011;	//U1TX = RP24	0b00011 means RPn tied to UART1 Transmit
	RPINR18bits.U1RXR = 25;   	//U1RX = RP25 	25 specifies RPn pin used for UART1 Receive
	__builtin_write_OSCCONL(OSCCON | (1<<6));		//lock registers


	OpenUART1(
		UART_EN &  UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 &  UART_DIS_WAKE &  
			UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_NO_PAR_8BIT & UART_BRGH_SIXTEEN & UART_1STOPBIT , 
		UART_INT_TX & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED &  UART_TX_ENABLE &  UART_INT_RX_CHAR &  UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR, 
		129		// ~=  40Mhz / (16 * 19200bps) - 1 
		//21		// ~=  40Mhz / (16 * 115200bps) - 1   with  1.4% error

	);

	//Setup UART with DMA Channel 1 see: http://ww1.microchip.com/downloads/en/DeviceDoc/70188D.pdf page 17-28

	DMA1CON = 0; 							// One-Shot, Post-Increment, RAM-to-Peripheral, Byte size 
	DMA1CONbits.SIZE = 1;					// Byte size
	DMA1CONbits.DIR = 1;					// Read from DPSRAM address, write to peripheral address
	DMA1CONbits.MODE = 0b01;				// One-Shot, Ping-Pong modes disabled


	DMA1REQ = 0b0001100;						// Select UART1 Transmitter
	DMA1PAD = (volatile unsigned int) &U1TXREG;
	DMA1STA = __builtin_dmaoffset(uart_buffer);
	IFS0bits.DMA1IF = 0; 						// Clear DMA Interrupt Flag
	IEC0bits.DMA1IE = 1; 						// Enable DMA Interrupt
	


	__delay_ms(10); //UART requires some delay (at least one bit time) before sending data
}

//start buffering
void uart_buffering_start(){
	uart_buffering_sent = 0;
	uart_buffering = 1;
}

//start sending current buffer
void uart_buffering_send(){
	if(uart_buffer_pos > 0){
		uart_buffering_busy = 1;
		DMA1CNT = uart_buffer_pos-1;	// number of bytes to send 
		DMA1CONbits.CHEN = 1;			// Enable DMA1 Channel
		DMA1REQbits.FORCE = 1;			// Manual mode: Kick-start the 1st transfer
	}
}

void __attribute__((__interrupt__,no_auto_psv)) _DMA1Interrupt(void){
	uart_buffer_pos = 0;	
	uart_buffering_busy = 0;		// send complete	
	IFS0bits.DMA1IF = 0;	// Clear the DMA1 Interrupt Flag;
}

//send existing buffer and stop buffering
int uart_buffering_end(){
	uart_buffering = 0;
	uart_buffering_send();
	return uart_buffering_sent;
}


void uart_send_char(char c){
	WriteUART1(c);
	while(BusyUART1());
}



//override default write() used by printf
//see http://support2.microchip.com/KBSearch/KB_StdProb.aspx?ID=SQ6UJ9A009MTU
int write(int handle, void *buffer, unsigned int len) {
	unsigned int i = 0;
	unsigned char* c = (unsigned char*)buffer;
	while(i++<len){
		while(uart_buffering_busy);
		if(uart_buffering){
			if(uart_buffer_pos>=UART_BUFFER_SIZE){
				uart_buffering_send();
				while(uart_buffering_busy);
			}
			uart_buffer[uart_buffer_pos] = *c;
			uart_buffer_pos++;
			uart_buffering_sent++;
		}else{
			uart_send_char(*c);
		}
		c++;
	}
	return len;
}





void print_float_list(int n,float *a){
	int i;
	for(i=0;i<n;i++){
		printf("%f",(double)a[i]);
		if(i<n-1) printf(",");
	}
}




//HDLC encoding http://www.tinyos.net/tinyos-2.x/doc/html/tep113.html#hdlc
#define HDLC_ESC	0x7D	//escape char	
#define HDLC_SEP	0x7E	//packet separator char
#define HDLC_XOR	0x20	//xor char


void hdlc_send_sep(){
	uart_send_char(HDLC_SEP);
}

void hdlc_send_byte(unsigned char c ){
	if(HDLC_SEP == c || HDLC_ESC == c){
		uart_send_char(HDLC_ESC);
		uart_send_char(c ^ HDLC_XOR);
	}else{
		uart_send_char(c);
	}
}

void hdlc_send_word(unsigned int w){
	//send Most Significat Byte First
	hdlc_send_byte(w >> 8 );
	hdlc_send_byte(w & 0xFF);
}


void hdlc_send_float(float f){
	unsigned char* p = (unsigned char*)(&f);
	hdlc_send_byte(*p);p++;
	hdlc_send_byte(*p);p++;
	hdlc_send_byte(*p);p++;
	hdlc_send_byte(*p);
}

#endif
