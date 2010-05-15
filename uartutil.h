#ifndef UARTUTIL_H
#define UARTUTIL_H
//---------------------------------------------------------------------
// UART
//---------------------------------------------------------------------

void uart_init(){
	//dsPIC33FJ* does not have dedicated ports for UART, we need to assign the pin's we're going to use



	//setup remapable pins for UART function
	_TRIS(pinTX) = 1;


	__builtin_write_OSCCONL(OSCCON & ~(1<<6));		//unlock registers
	RPOR2bits.RP4R = 0b00011;	//U1TX = RP4
	RPINR18bits.U1RXR = 7;   	//U1RX = RP7 
	__builtin_write_OSCCONL(OSCCON | (1<<6));		//lock registers


	OpenUART1(
		UART_EN &  UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 &  UART_DIS_WAKE &  
			UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_NO_PAR_8BIT & UART_BRGH_SIXTEEN & UART_1STOPBIT , 
		UART_INT_TX & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED &  UART_TX_ENABLE &  UART_INT_RX_CHAR &  UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR, 
		//129	// ~=  40Mhz / (16 * 19200bps) - 1 
		21		// ~=  40Mhz / (16 * 115200bps) - 1   with  1.4% error

	);

	__delay_ms(10); //UART requires some delay (at least one bit time) before sending data
}


void uart_send_char(char c){
	WriteUART1(c);
	while(BusyUART1());
}


#define PACKET_LEN 7 
unsigned char receiveBuffer[255];
unsigned char receiveBufferPos=0;




int uart_read_packet(){
	// must clear the overrun error to keep uart receiving 
	if(U1STAbits.OERR == 1)
	{
		U1STAbits.OERR = 0;
		receiveBufferPos = 0;
		return -1;
	}
	// check for receive errors 
	if(U1STAbits.FERR == 1)
	{
		U1STAbits.FERR = 0;
		receiveBufferPos = 0;
		return -1;
	}
	
	while(DataRdyUART1()){
		unsigned char receiveByte = U1RXREG;
		if(255 == receiveByte){
			unsigned char pos = receiveBufferPos;
			receiveBufferPos = 0;
			if(pos >= PACKET_LEN) return pos-PACKET_LEN;
		}else{
			receiveBuffer[receiveBufferPos] = receiveByte;
			receiveBufferPos++;
		}
	}

	return -1;
}


#endif
