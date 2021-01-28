//////////////////////////////////////////////////////////////////////////////////
//
// Name:				(Master) test RS-485 for MPC
// Author:				Pankov D
// Date of completion:	01.2020
// Version:				1.0
//
// ��������� ������� ��
#define F_CPU 8000000UL
// ����������� ���������
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

char c_DEVICE_UID[8] = {'C', 'O', 'N', 'T', 'R', 'O', 'L', 'L'};	// CONTROLL
char c_DEVICE_MAC[4] = {0x01, 0x01, 0x01, 0x01};
#define DEVICE_UID					c_DEVICE_UID
#define DEVICE_MAC					c_DEVICE_MAC

#define RS485_SETBIT(x,y)			( x |= ( 1 << y ) )
#define RS485_CLRBIT(x,y)			( x &= ~( 1 << y ) )
#define RS485_PORT					PORTD
#define RS485_CONTROL				PD3          // ���������� �������/��������� RS485
#define RS485_TR					RS485_SETBIT(RS485_PORT,RS485_CONTROL) // ���������� RE DE � 1. ��������
#define RS485_RS					RS485_CLRBIT(RS485_PORT,RS485_CONTROL) // ���������� RE DE � 0. �����

// Usart makros of speed
#define USART_BAUD					115200
#define UBRR_VAL					F_CPU / 16 / USART_BAUD - 1
// USART Receiver buffer
#define DATA_REGISTER_EMPTY			( 1 << UDRE )
#define RX_COMPLETE					( 1 << RXC )
#define FRAMING_ERROR				( 1 << FE )
//#define PARITY_ERROR				( 1 << UPE )
#define DATA_OVERRUN				( 1 << DOR )
#define RX_ADDRESS_SIZE				4
//#define RX_DATA_SIZE				16
#define RX_BUFFER_SIZE				/*1 + RX_ADDRESS_SIZE + RX_DATA_SIZE + 1*/ 8
// ��������� ���������
#define USART_STARTPACKET 0x00	// ��������� ����
char ADRESS_DBK[2] = {0x00, 0x00};	// ����� ���
char DIEN_DBK[2] = {0x00, 0x00};	// ��������� ���� ���
char DIAG_DBK = 0x00;	// ���� �����������
char INFO_DBK[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	// ���� ������
char DATA_DBK[4] = {0x00, 0x00, 0x00, 0x00};	// ����� �������
char CONTROL_DBK[2]	= {0x00, 0x00};	// ����������� ����
#define LEN_RS 18	// ����� ������ ������
#define USART_STOPPACKET 0xBC
// This flag is set on USART Receiver buffer overflow
unsigned char ex_rx_index;
char ex_rx_buffer_adr[RX_ADDRESS_SIZE];
char ex_rx_buffer_cmd;
//char ex_rx_buffer_dat[RX_DATA_SIZE];
char rx_buffer_dat;
char ex_rx_buffer_dat1;
bool ex_rx_buffer_overflow = false;
bool ex_rx_enable = false;
bool ex_rx_data_complite = false;

// LED ON
void LEDLAMP_ON(unsigned int n_pin)
{
	PORTC |= ( 1 << n_pin );
}

// LED OFF
void LEDLAMP_OFF(unsigned int n_pin)
{
	PORTC &= ~( 1 << n_pin );
}

void UART_Init (unsigned int speed)
{
	// ������������� �������� Baud Rate
	UBRRH = (unsigned char)( speed >> 8 );
	UBRRL = (unsigned char) speed;
	UCSRA |= (1<<U2X); // �������� �������
	
	// ���������� ������ �����������
	UCSRB = ( 1 << TXEN ) | ( 1 << RXEN ) | (1 << RXCIE ) | (1 << TXCIE );
	
	/* Set frame format: 8data, 2stop bit */
	UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
	
}

// Send to UART
void UART_Send_Char (char data_tx) ////
{
	while ( !( UCSRA & (1<<5)) ) {}
	RS485_TR;
	UDR = data_tx;
}

// Send to UART
void UART_SendString (char data_tx[])
{
	int i;
	//len = strlen( data_tx );
	for (i=0; i < RX_BUFFER_SIZE; i++) {
		//LEDLAMP_ON(1);
		UART_Send_Char(data_tx[i]);
		//_delay_ms(50);
		//LEDLAMP_OFF(1);
	}
}

ISR(USART_TXC_vect)
{
	RS485_RS; // ���������� RE DE � 0. �����
}

void USART_SendPacket(char rx_device_mac[4], char rx_buffer_cmd, char rx_buffer_dat /*char rx_buffer_dat[RX_DATA_SIZE]*/)
{
	char tmp_tx_data[RX_BUFFER_SIZE];
	rx_buffer_dat = 0xCC;
	sprintf(tmp_tx_data, "%c%c%c%c%c%c%c%c"/*"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"*/,	USART_STARTPACKET,
																	/*
																	ADRESS_DBK[0],
																	ADRESS_DBK[1],
																	DIEN_DBK[0],
																	DIEN_DBK[1],
																	DIAG_DBK,
																	INFO_DBK[0],
																	INFO_DBK[1],
																	INFO_DBK[2],
																	INFO_DBK[3],
																	INFO_DBK[4],
																	INFO_DBK[5],
																	DATA_DBK[0],
																	DATA_DBK[1],
																	DATA_DBK[2],
																	DATA_DBK[3],
																	CONTROL_DBK[0],
																	CONTROL_DBK[1]
																	*/
																	///*
																	rx_device_mac[0],
																	rx_device_mac[1],
																	rx_device_mac[2],
																	rx_device_mac[3],
																	rx_buffer_cmd,
																	rx_buffer_dat,	
																	USART_STOPPACKET
																	//*/
																	);
	//LEDLAMP_ON(1);											
	UART_SendString(tmp_tx_data);
	//_delay_ms(50);
	//LEDLAMP_OFF(1);
}

ISR(USART_RXC_vect)
{
	char status, data; ////
	//unsigned char sub_rx_index;
	status = UCSRA;
	data = UDR;
	if ((status & (FRAMING_ERROR /*| PARITY_ERROR */| DATA_OVERRUN))==0)
	{

		if (data == USART_STARTPACKET)	// ��������� ���������� ������
		{
			memset(ex_rx_buffer_adr, 0, sizeof(ex_rx_buffer_adr));// �������� ���� ����������
			ex_rx_buffer_cmd = 0;
			//memset(ex_rx_buffer_dat, 0, sizeof(ex_rx_buffer_dat));
			ex_rx_index=0;
			ex_rx_enable = true;
			ex_rx_data_complite = false;
			
		}
		
		if ( (data == USART_STOPPACKET) && (ex_rx_enable == 1) )
		{
			ex_rx_enable = 0;
			ex_rx_data_complite = true;
		}
		
		if (ex_rx_enable == 1)
		{
			if ( (ex_rx_index >= 0) && (ex_rx_index <= 4) ) // ������ ������ 4� ������
			{
				ex_rx_buffer_adr[ex_rx_index] = data;
				if (ex_rx_index == 4)
				{
					//if ( !strcat(ex_rx_buffer_adr, DEVICE_MAC) )
					//{
					//	ex_rx_enable = 0;
					//	ex_rx_data_complite = false;
					//}
				}
				++ex_rx_index;
			} else if (ex_rx_index == 5) // ���������� ����� ���
			{
				ex_rx_buffer_cmd = data;
				++ex_rx_index;
			} else if (ex_rx_index > 5)
			{
				//sub_rx_index = ex_rx_index - 6;
				//ex_rx_buffer_dat[sub_rx_index] = data;
				ex_rx_buffer_dat1 = data;
				++ex_rx_index;
			}

			if (ex_rx_index >= RX_BUFFER_SIZE)
			{
				ex_rx_enable = false;
				ex_rx_buffer_overflow=1;
			}
			
		}
		
	}
}

void ExchangeUART(char rx_buffer_cmd, char rx_buffer_dat /*char rx_buffer_dat[RX_DATA_SIZE]*/)
{
	//char tmp_rx_buffer[RX_DATA_SIZE];
	if (rx_buffer_cmd == 0x02) // ��������� ����������
	{
		//memset(tmp_rx_buffer, 0, sizeof(tmp_rx_buffer));
		//sprintf(tmp_rx_buffer, "%s", "GOOD!!!");
		LEDLAMP_ON(1);
		//USART_SendPacket(DEVICE_MAC, 0xA2, tmp_rx_buffer);
		USART_SendPacket(DEVICE_MAC, 0x01, rx_buffer_dat);
		_delay_ms(50);
		LEDLAMP_OFF(1);
	}
}

int main(void)
{
	UART_Init(8); //115200
	char rx_device_mac[4] = {0xF2, 0xF2, 0xF2, 0xF2};
		
	
	DDRD |= ( 1 << PD3);	// ���� �� �����
	RS485_RS; // ���������� RE DE � 0. �����
	//PORTD = ( 1 << PD3);	// ���� �� 1
	
	DDRC |= ( 1 << PC0) | ( 1 << PC1);	// ���� �� �����
	
	sei();
	//ex_rx_data_complite = true;
	//_delay_ms(1500);
	//USART_SendPacket(rx_device_mac, 0xA1, ex_rx_buffer_dat1);
	_delay_ms(15);
	//memset(ex_rx_buffer_dat, 0, sizeof(ex_rx_buffer_dat));
	USART_SendPacket(rx_device_mac, 0x01, ex_rx_buffer_dat1);
	while(1)
	{
		LEDLAMP_ON(0);
		_delay_ms(50);
		LEDLAMP_OFF(0);
		//PORTD = ~( 1 << PD3);
		//RS485_RS; 
		//_delay_ms(1);
		if (ex_rx_data_complite)
		{
			ex_rx_data_complite = false;
			ExchangeUART(ex_rx_buffer_cmd, ex_rx_buffer_dat1);
		}
		_delay_ms(15);
	}
}