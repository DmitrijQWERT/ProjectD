//////////////////////////////////////////////////////////////////////////////////
//
// Name:				(Slave) test RS-485 for MPC, ����-��
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
#define RX_BUFFER_SIZE				18 // ����� ������ ������
// ��������� ���������
#define USART_STARTPACKET 0x00	// ��������� ����
char ADR_DBK[2] = {0x00, 0x00};	// ����� ���
char DIEN_DBK[2] = {0x00, 0x00};	// ��������� ���� ��� - ����������� ������ ���
char DIAG_DBK = 0x00;	// ���� ����������� - ����������� ������ ���
char DAN_DBK[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	// ���� ������
char DATA_DBK[4] = {0x00, 0x00, 0x00, 0x00};	// ����� �������
char CONTROL_DBK[2]	= {0x00, 0x00};	// ����������� ����
// ���������� ������ �� �����
char ex_rx_buffer_ADRESS_DBK[2];
char ex_rx_buffer_DIEN_DBK[2];
char ex_rx_buffer_DIAG_DBK;
char ex_rx_buffer_DAN_DBK[6];
char ex_rx_buffer_DATA_DBK[4];
char ex_rx_buffer_CONTROL_DBK[2];
// This flag is set on USART Receiver buffer overflow
unsigned char ex_rx_index;
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
void UART_Send_Char (char data_tx)////
{
	while ( !( UCSRA & (1<<5)) ) {}
	RS485_TR;
	UDR = data_tx;
}

// Send to UART
void UART_SendString (char data_tx[])
{
	int i;
	for (i=0; i < RX_BUFFER_SIZE; i++) {
		UART_Send_Char(data_tx[i]);
	}
}

ISR(USART_TXC_vect)
{
	RS485_RS; // ���������� RE DE � 0. �����
}

void USART_SendPacket(char ADR_DBK[2], char DAN_DBK[6], char DATA_DBK[4], char CONTROL_DBK[2])
{
	char tmp_tx_data[RX_BUFFER_SIZE];
	sprintf(tmp_tx_data, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",	USART_STARTPACKET,
																	ADR_DBK[0],
																	ADR_DBK[1],
																	DIEN_DBK[0],
																	DIEN_DBK[1],
																	DIAG_DBK,
																	DAN_DBK[0],
																	DAN_DBK[1],
																	DAN_DBK[2],
																	DAN_DBK[3],
																	DAN_DBK[4],
																	DAN_DBK[5],
																	DATA_DBK[0],
																	DATA_DBK[1],
																	DATA_DBK[2],
																	DATA_DBK[3],
																	CONTROL_DBK[0],
																	CONTROL_DBK[1]
	);
	UART_SendString(tmp_tx_data);
}

ISR(USART_RXC_vect)
{
	char status, data; //
	status = UCSRA;
	data = UDR;
	if ((status & (FRAMING_ERROR | DATA_OVERRUN))==0)
	{
		if ((ex_rx_enable == false) && (data == USART_STARTPACKET))	// ��������� ���������� ������
		{
			// ������� ���������� ������ ��� ������ ������ ������
			memset(ex_rx_buffer_ADRESS_DBK, 0, sizeof(ex_rx_buffer_ADRESS_DBK));
			memset(ex_rx_buffer_DIEN_DBK, 0, sizeof(ex_rx_buffer_DIEN_DBK));
			ex_rx_buffer_DIAG_DBK = 0;
			memset(ex_rx_buffer_DAN_DBK, 0, sizeof(ex_rx_buffer_DAN_DBK));
			memset(ex_rx_buffer_DATA_DBK, 0, sizeof(ex_rx_buffer_DATA_DBK));
			memset(ex_rx_buffer_CONTROL_DBK, 0, sizeof(ex_rx_buffer_CONTROL_DBK));
			//
			ex_rx_index=0;
			ex_rx_enable = true;
			ex_rx_data_complite = false;
		}
		if (ex_rx_enable == 1)	// ��������� ����������
		{
			switch (ex_rx_index)
			{
				case 1:
				ex_rx_buffer_ADRESS_DBK[ex_rx_index - 1] = data;
				break;
				case 2:
				ex_rx_buffer_ADRESS_DBK[ex_rx_index - 1] = data;
				break;
				case 3:
				ex_rx_buffer_DIEN_DBK[ex_rx_index - 3] = data;
				break;
				case 4:
				ex_rx_buffer_DIEN_DBK[ex_rx_index - 3] = data;
				break;
				case 5:
				ex_rx_buffer_DIAG_DBK = data;
				break;
				case 6:
				ex_rx_buffer_DAN_DBK[ex_rx_index - 6] = data;
				break;
				case 7:
				ex_rx_buffer_DAN_DBK[ex_rx_index - 6] = data;
				break;
				case 8:
				ex_rx_buffer_DAN_DBK[ex_rx_index - 6] = data;
				break;
				case 9:
				ex_rx_buffer_DAN_DBK[ex_rx_index - 6] = data;
				break;
				case 10:
				ex_rx_buffer_DAN_DBK[ex_rx_index - 6] = data;
				break;
				case 11:
				ex_rx_buffer_DAN_DBK[ex_rx_index - 6] = data;
				break;
				case 12:
				ex_rx_buffer_DATA_DBK[ex_rx_index - 12] = data;
				break;
				case 13:
				ex_rx_buffer_DATA_DBK[ex_rx_index - 12] = data;
				break;
				case 14:
				ex_rx_buffer_DATA_DBK[ex_rx_index - 12] = data;
				break;
				case 15:
				ex_rx_buffer_DATA_DBK[ex_rx_index - 12] = data;
				break;
				case 16:
				ex_rx_buffer_CONTROL_DBK[ex_rx_index - 16] = data;
				break;
				case 17:
				ex_rx_buffer_CONTROL_DBK[ex_rx_index - 17] = data;
				break;
			}
			++ex_rx_index;
		}
		if ( (ex_rx_index >= RX_BUFFER_SIZE) && (ex_rx_enable == 1) )	// ����� �������
		{
			ex_rx_enable = 0;
			ex_rx_data_complite = true;
		}
	}
}
void ExchangeUART(char ADR_DBK[2], char INFO_DBK[6], char DATA_DBK[4], char CONTROL_DBK[2])
{
	if ((ADR_DBK[0] == 0x01) && ADR_DBK[1] == 0x5E) // ��������� ����������
	{
		USART_SendPacket(ex_rx_buffer_ADRESS_DBK, ex_rx_buffer_DAN_DBK, ex_rx_buffer_DATA_DBK, ex_rx_buffer_CONTROL_DBK);
	}
}
int main(void)
{
	UART_Init(8); //115200
	DDRD |= ( 1 << PD3);	// ���� �� �����
	RS485_RS; // ���������� RE DE � 0. �����
	DDRC |= ( 1 << PC0) | ( 1 << PC1); // ���� �� �����
	sei();
	while(1)
    {
		_delay_ms(1);
		if (ex_rx_data_complite)
		{
			ex_rx_data_complite = false;
			ExchangeUART(ex_rx_buffer_ADRESS_DBK, ex_rx_buffer_DAN_DBK, ex_rx_buffer_DATA_DBK, ex_rx_buffer_CONTROL_DBK);
		}
    }
}