/*
 * RTOS.cpp
 *
 * Created: 2016-11-26 오후 2:50:59
 * Author : kimkisu
 */ 

/* Scheduler include files. */

#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>

//#include "ip_arp_udp_tcp.h"
//#include "enc28j60.h"
//#include "timeout.h"
//#include "avr_compat.h"
//#include "net.h"


#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "queue.h"
#include "Channel.h"
#include "Request.h"
#include "UartDriver.h"
#include "Dev_Manager.h"
#include "SerialBuffer.h"
#include "RS485Driver.h"
#include "Modbus_rtu.h"


//////////////////

typedef enum
{
	BIT0 = 0,
	BIT1 = 1,
	BIT2 = 2,
	BIT3 = 3,
	BIT4 = 4,
	BIT5 = 5,
	BIT6 = 6,
	BIT7 = 7,	
}BIT;

typedef enum
{
	FWD,
	REV,
	STOP
}Inverter_States;

typedef enum
{
	LOW,
	HIGH,	
}Signals;
#define sbi(PORTX,bitX) PORTX |= (1 <<bitX)
#define cbi(PORTX,bitX) PORTX &= ~(1 <<bitX)

//#define MYWWWPORT 80
//#define MYUDPPORT 1200
//#define BUFFER_SIZE 450


///////////////


#define  F_CPU   16000000UL
#include <util/delay.h>

#define UPE 2
#define DOR 3
#define FE 4
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)


/////////////////////////////////////////////////////////////////////////////////

static void vTask1(void *pvParam);
static void vTask2(void *pvParam);

static void proc(void* pvParam);
static void proc1(void* pvParam);
static void proc2(void* pvParam);


/*
 * Idle hook is used to scheduler co-routines.
 */
extern "C"
{
	void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName );

}
Dev_Manager *dev;

void *DataStruct[MAX] = {nullptr};
void Init_Dev();
void Uart_ISR(Dev_type Device,uint16_t Arg);
void RS485_ISR(Dev_type Device,uint16_t Arg);


void digital_OUT(volatile unsigned char *PORT,char Pine,Signals _Signal);
void func05_output_ctl(char* mem,int adr);


GetFunctionCode01 func01;
GetFunctionCode05 func05;
GetFunctionCode04 func04;
GetFunctionCode10 func10; 


RspExceptionCode exception;

ResponseFunctionCode10 rsp10;

char mem1[2];
int mem4[10] = {0};
char mem5[10] = {0};

char buffer_flag = 0;

enum
{
	TARGET_TEMP1,  //0
	CURRENT_TEMP1, //1
	CURRENT_SV1,  //2
	INVERTER_DIRECTION, //3
	INVERTER_HERTZ,  //4
	INVERTER_SPEED,  //5
	TARGET_TEMP2,    //6
	CURRENT_TEMP2,   //7
	CURRENT_SV2,     //8
};

int main( void )
{
	cli();
	DDRG = 0xff;                        
	PORTG = 0xff;
	DDRF = 0xff;
	DDRD |= 1;
	DDRE = 0xff;
	PORTE = 0x00;
	
	
	
	DDRB = 0xff;
	PORTB = 0xff;
	
	
	
	Init_Dev();
	dev->Open_Handle(UART0,Uart_ISR);
	
	dev->Open_Handle(RS485,RS485_ISR);
	
	SerialBuffer *sb = new SerialBuffer(dev,UART0);
	SerialBuffer *sb1 = new SerialBuffer(dev,RS485);
	
	DataStruct[UART0] = sb;
	DataStruct[RS485] = sb1;
	
	sei();
	//Serialinit();


	
	xTaskCreate(proc,                //테스크 실행할 함수 포인터
	"Task1",      //테스크 이름
	240,                   //스택의 크기
	sb,       // 테스크 매개 변수
	2,                     //테스크 우선 순위
	NULL                   //태스크 핸들
	);
	
		xTaskCreate(proc1,                //테스크 실행할 함수 포인터
		"Task2",      //테스크 이름
		240,                   //스택의 크기
		sb1,       // 테스크 매개 변수
		2,                     //테스크 우선 순위
		NULL                   //태스크 핸들
		);
		
				xTaskCreate(proc2,                //테스크 실행할 함수 포인터
				"Task3",      //테스크 이름
				240,                   //스택의 크기
				NULL,       // 테스크 매개 변수
				2,                     //테스크 우선 순위
				NULL                   //태스크 핸들
				);

			
		
	
	vTaskStartScheduler();
	return 0;
}
void Init_Dev()
{
	dev = new Dev_Manager();
	dev->Register_Dev(new UartDriver,UART0);
	dev->Register_Dev(new RS485Driver,RS485);
	dev->Device_Init(UART0);
	dev->Device_Init(RS485);
	//dev->Writes(UART0,"Uart Init\r\n");
	//dev->Writes(RS485,"RS485 Init\r\n");
}

void Uart_ISR(Dev_type Device,uint16_t Arg)
{ 
	uint8_t data = Arg;
	SerialBuffer *sb = (SerialBuffer*)DataStruct[UART0];
	sb->Serialstore(data);

}
void RS485_ISR(Dev_type Device,uint16_t Arg)
{
	uint8_t data = Arg;
	SerialBuffer *sb = (SerialBuffer*)DataStruct[RS485];
	sb->Serialstore(data);
}
Inverter_States inverter;
static void proc(void* pvParam)
{
	char read_Flag = 0;
	char function_code;
	char buf1[10];

	SerialBuffer *sb = (SerialBuffer*)pvParam;
	mem1[0] = 0;
	mem4[0] = 500;
	while(1)
	{
		if(read_Flag == 0)
		{
			if(sb->SerialAvailable() >= 2)
			{
				for(int i=0;i<2;i++)
				{
					PORTG = 0xff;
					buf1[i] = sb->SerialRead();
					//sb->SerialWrite(buf1[i]);
					PORTG = 0x00;
				}
				if(buf1[0] != 0x01)
				{
					buffer_flag =1;
					PORTE = 0b00000001;
					mem4[5]++;
					sb->SerialFlush();
					PORTE = 0x00;
					read_Flag = 0;
				}
				if(buf1[1] == 0x01)
				{
					function_code = 0x01;
					read_Flag = 1;
				}
				else if(buf1[1] == 0x04)
				{
					function_code = 0x04;
					read_Flag = 1;
				}
				else if(buf1[1] == 0x05)
				{
					function_code = 0x05;                              
					read_Flag = 1;
				}
				else if(buf1[1] == 0x10)
				{
					function_code = 0x10;
					read_Flag = 1;
				}
				else
				{
					GetExceptionCode(buf1,&exception,0x01,0x01);
					sb->SerialWrite((char*)&exception,sizeof(exception));
					buffer_flag =1;
					PORTE = 0b00000100;                                                                                                                                                                                                                    
					//mem4[5]++;
					sb->SerialFlush();
					PORTE = 0;
					
					read_Flag = 0;	
				}
			}
			
		}
		if(read_Flag == 1)
		{
			if(function_code == 0x01)
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						PORTG = 0xff;
						buf1[i] = sb->SerialRead();
						PORTG = 0x00;
					}
					GetFunc01Data(buf1,&func01,mem1);
					sb->SerialWrite((char*)&func01,sizeof(func01));
					read_Flag = 0;
				}
			}
			else if(function_code == 0x04)
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						PORTG = 0xff;
						buf1[i] = sb->SerialRead();
						//sb->SerialWrite(buf1[i]);
						PORTG = 0x00;
					}
					GetFunc04Data(buf1,&func04,mem4);
					sb->SerialWrite((char*)&func04,sizeof(func04));
					//sb->SerialWrite((char*)&func04,sizeof(func04));
					read_Flag = 0;
				}
			}
			else if(function_code == 0x05)
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						PORTG = 0xff;
						buf1[i] = sb->SerialRead();
						PORTG = 0x00;
					}
					GetFunc05Data(buf1,&func05);
					int adr = func05.OutputAddressHi << 8;
					adr |= func05.OutputAddressLo;
					
					mem5[adr] = func05.OutputValueHi;
					func05_output_ctl(mem5,adr);
					sb->SerialWrite((char*)&func05,sizeof(func05));
					read_Flag = 0;
				}
			}
			else if(function_code == 0x10)
			{
				if(sb->SerialAvailable() >= 9)
				{
					for(int i=2;i<11;i++)
					{
						PORTG = 0xff;
						buf1[i] = sb->SerialRead();
						//sb->SerialWrite(buf1[i]);
						PORTG = 0x00;
					}
					GetFucc10Data(buf1,&func10,mem4); //데이터 파싱
					ResponseFucc10Data(buf1,&rsp10); //리스폰스 데이터를 만듬.
					sb->SerialWrite((char*)&rsp10,sizeof(rsp10)); //리스폰스 데이터 쓰기.
					//sb->SerialWrite((char*)&rsp10,sizeof(rsp10));
					read_Flag = 0;
				}
						
			}
			
			/////////////////////////////////			
		}

		
		
		
		////
	}
}
#define USE_TEMP 1
#define USE_TEMP1 1
#define USE_INVERTER 0

static void proc1(void* pvParam)
{
	char read_flag = 0;
	char function_code;
	char func_adr;
	char init_read = 0;
	int Hertz;
	int SV1 = 0;
	int SV2 = 0;
	int Move_speed;
    Inverter_States cmp;
	InputOutput8Byte byte03;
	GetData gd;
	SerialBuffer *sb = (SerialBuffer*)pvParam;
	SerialBuffer *sb1 = (SerialBuffer*)DataStruct[UART0];
	char buf[10];

	char write_flag = 0;
	mem4[1] = 0;
	char rs485_cnt = 0;
	while(init_read == 0)
	{
		if(rs485_cnt == 0 && write_flag == 0) //설정된 sv값을 읽어옴
		{
			Function03Write(0x01,302,0x01,&byte03);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
			write_flag = 1;
		}
		else if(rs485_cnt == 1 && write_flag == 0)  //인버터 상태
		{
			Function03Write(0x03,0x05,0x01,&byte03);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
			write_flag = 1;
		}
		else if(rs485_cnt == 2 && write_flag == 0) //인버터 속도 
		{
			Function03Write(0x03,0x04,0x01,&byte03);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
			write_flag  =1;
		}
		else if(rs485_cnt == 3 && write_flag == 0) //설정된 sv값을 읽어옴
		{
			Function03Write(0x02,301,0x01,&byte03);
			//vTaskDelay(1000);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
			write_flag = 1;
		}
		if(sb->SerialAvailable() >= 7 && rs485_cnt == 0)
		{
			for(int i=0;i<7;i++)
			{
				buf[i] = sb->SerialRead();
			}
			getFunction3Data(buf,&gd);
			mem4[CURRENT_SV1] = gd.CurrentPv;
			mem4[1] = 0;
			SV1 = mem4[CURRENT_SV1];
			sb->SerialFlush();
			#if USE_INVERTER
				rs485_cnt++;
			#else
				#if USE_TEMP1
					rs485_cnt = 3;	
				#else
					init_read = 1;
				#endif
			#endif
			write_flag = 0;
		}
		if(sb->SerialAvailable() >= 7 && rs485_cnt == 1)
		{
			for(int i=0;i<7;i++)
			{
				buf[i] = sb->SerialRead();
				//sb1->SerialWrite(buf[i]);
			}
			char temp = buf[4];
			temp &= 0b00000111;
			if(temp == 0x01) 
			{
				inverter = STOP;
				cmp = STOP;
				sb->SerialFlush();
				mem4[INVERTER_DIRECTION] = inverter;
				rs485_cnt++;
			}
			else if(temp == 0x02)
			{
				 inverter = FWD;
				 cmp = FWD;
				 sb->SerialFlush();
				 mem4[INVERTER_DIRECTION] = inverter;
				 rs485_cnt++;
			}
			else if(temp == 0x04)
			{
				 inverter = REV;
				 cmp = REV;
				 sb->SerialFlush();
				 mem4[INVERTER_DIRECTION] = inverter;
				 rs485_cnt++;
			}
			write_flag = 0;
		}
		if(sb->SerialAvailable() >= 7 && rs485_cnt == 2)
		{
			for(int i=0;i<7;i++)
			{
				buf[i] = sb->SerialRead();
				//sb1->SerialWrite(buf[i]);
			}
			Hertz = 0xff00 & (buf[3] << 8);
			Hertz |= buf[4];
			mem4[INVERTER_HERTZ] = Hertz;
			Move_speed = (( mem4[INVERTER_HERTZ] / 100 ) * 5); 
			mem4[INVERTER_SPEED] = Move_speed; 
		
			write_flag = 0;
			#if USE_TEMP1
				rs485_cnt++;
			#else
				init_read = 1;
			#endif
		
		}
		if(sb->SerialAvailable() >= 7 && rs485_cnt == 3)
		{
			for(int i=0;i<7;i++)
			{
				buf[i] = sb->SerialRead();
			}
			getFunction3Data(buf,&gd);
			mem4[CURRENT_SV2] = gd.CurrentPv;
			SV2 = mem4[CURRENT_SV2];
			sb->SerialFlush();
			write_flag = 0;
			init_read = 1;
		}
		vTaskDelay(300);
	}
	vTaskDelay(100);
	//////////////////////////start main()////////////////////////////
	
	while(1)
	{
		#if USE_INVERTER
		if(Move_speed != mem4[5])
		{
			write_flag = 3;
		}
		
		if(cmp != inverter)
		{
			write_flag = 2;
		}
		#endif
		#if USE_TEMP1
		if(SV2 != mem4[CURRENT_SV2])
		{
			SV2 = mem4[CURRENT_SV2];
			write_flag = 5;
		}
		#endif
		if(SV1 != mem4[2])
		{
			write_flag = 1;
			SV1 = mem4[2];
		}
		if(write_flag == 0)  //첫번째 기기 한영넉스 PV값 읽기
		{
			Function03Write(0x01,0x01,0x01,&byte03);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
		}
		if(write_flag == 4) //두번째 기기 한영넉스 pV값 읽기
		{
			Function03Write(0x02,0x01,0x01,&byte03);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
			//write_flag = 0;
		}
		if(write_flag == 5) //두번째 기기 한영넉스 SV 기록
		{
			Function06Write(0x02,301,mem4[CURRENT_SV2],&byte03);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
			write_flag = 0;
		}
		if(write_flag == 1) //첫번째 기기 한영넉스 SV 기록
		{
			Function06Write(0x01,302,mem4[2],&byte03);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
			write_flag = 0;
		}
		if(write_flag == 2) //인버터 동작 명령
		{
			if(inverter == REV)
			{
				Function06Write(0x03,0x05,0x04,&byte03);
				sb->SerialWrite((char*)&byte03,sizeof(byte03));
				cmp = REV;
				write_flag = 0;
			}
			else if(inverter == FWD)
			{
				Function06Write(0x03,0x05,0x02,&byte03);
				sb->SerialWrite((char*)&byte03,sizeof(byte03));
				cmp = FWD;
				write_flag = 0;
			}
			else if(inverter == STOP)
			{
				Function06Write(0x03,0x05,0x00,&byte03);
				sb->SerialWrite((char*)&byte03,sizeof(byte03));
				cmp = STOP;
				write_flag = 0;
			}

		}
		if(write_flag == 3)
		{
			Function06Write(0x03,0x04,((mem4[5] / 5) * 100),&byte03);
			sb->SerialWrite((char*)&byte03,sizeof(byte03));
			write_flag = 0;
		}
		
		if(sb->SerialAvailable() >= 2 && read_flag == 0)
		{
			for(int i=0;i<2;i++)
			{
				buf[i] = sb->SerialRead();
			}
			if(buf[0] == 0x01)
			{
				read_flag = 1;
			}
			else if(buf[0] == 0x02)
			{
				read_flag = 3;
			}
			else if(buf[0] == 0x03)
			{
				read_flag = 2;
			}
			if(buf[1] == 0x03)
			{
				function_code = 0x03;
			}
			else if(buf[1] == 0x06)
			{
				function_code = 0x06;
			}
		}
		if(read_flag == 1)
		{
			if(function_code == 0x03)
			{
				if(sb->SerialAvailable() >= 5)
				{
					for(int i=2;i<7;i++)
					{
						buf[i] = sb->SerialRead();
					}
					getFunction3Data(buf,&gd);
					mem4[CURRENT_TEMP1] = gd.CurrentPv;
					read_flag = 0;
					write_flag = 4;
				}
			}
			if(function_code == 0x06)
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						buf[i] = sb->SerialRead();
					}
				}
				read_flag = 0;
				write_flag = 0;
			}
		}
		if(read_flag == 2)
		{
			if(function_code == 0x03)
			{
				if(sb->SerialAvailable() >= 5)
				{
					for(int i=2;i<7;i++)
					{
						buf[i] = sb->SerialRead();
					}
				}
				read_flag = 0;
				write_flag = 0;
			}
			if(function_code == 0x06)
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						buf[i] = sb->SerialRead();
					}
				}
				if(buf[3] == 0x04)
				{
					Hertz = 0xff00 & (buf[4] << 8);
					Hertz |= buf[5];
					mem4[4] = Hertz;
					Move_speed = (( mem4[4] / 100 ) * 5);
					mem4[5] = Move_speed;
				}
				read_flag = 0;
				write_flag = 0;
			}
			
		}
		if(read_flag == 3)
		{
			if(function_code == 0x03)
			{
				if(sb->SerialAvailable() >= 5)
				{
					for(int i=2;i<7;i++)
					{
						buf[i] = sb->SerialRead();
					}
					getFunction3Data(buf,&gd);
					mem4[CURRENT_TEMP2] = gd.CurrentPv;
					read_flag = 0;
					write_flag = 0;
				}
			}
			if(function_code == 0x06)
			{
				if(sb->SerialAvailable() >= 6)
				{
					for(int i=2;i<8;i++)
					{
						buf[i] = sb->SerialRead();
					}
				}
				int temp_sv = 0xff00 &(buf[4] << 8); 
				temp_sv |= buf[5];
				mem4[CURRENT_SV2] = temp_sv;
				read_flag = 0;
				write_flag = 0;
			}
		}
		vTaskDelay(300);
	}
}
void func05_output_ctl(char* mem,int adr)
{
	char out_put = 1;
	if(adr == 2 || adr == 3 || adr == 4)
	{
		out_put = 0;
	}
	char out_data;
	if(mem[adr])
	{
		mem1[0] |= (1 << adr);
		char pin = adr;
		if(out_put)
		{
			digital_OUT(&PORTB,pin,HIGH);
		}
		if(adr == 2)
		{
			inverter = FWD;
			mem4[3] = inverter;
		}
		else if(adr == 3)
		{
			inverter = REV;
			mem4[3] = inverter;
		}
		else if(adr == 4)
		{
			inverter = STOP;
			mem4[3] = inverter;
		}
	}
	else if(mem[adr] == 0x00)
	{
		mem1[0] &= ~(1 << adr);
		char pin = adr;
		if(out_put)
		{
			digital_OUT(&PORTB,pin,LOW);
		}
	}
}

void digital_OUT(volatile unsigned char *PORT,char Pine,Signals _Signal)
{
	if(_Signal == LOW)
	{
		sbi(*PORT,Pine);
	}
	else
	{
		cbi(*PORT,Pine);
	}
}
static void proc2(void* pvParam)
{
	DDRC = 0xe0;   // 111 <--출력 00000 <-- 입력 
	PORTC = 0xff; //풀업 사용 레지스터
	SFIOR = 0x00;  //풀업 Enable,Disable
	while(1)
	{
		if(PINC & 0x01)
		{
			mem1[0] |= (1 << PINC7);
			digital_OUT(&PORTC,BIT5,LOW);
		}
		else
		{
			mem1[0] = (mem1[0] & (0xff & ~(1 << PINC7)));
			digital_OUT(&PORTC,BIT5,HIGH);
			digital_OUT(&PORTC,BIT6,HIGH);
		}                                                                                                                                                                                                                                                                                                                                      
		if(PINC & 0x02)
		{
			mem1[1] |= (1 << PINC0);
		}
		else 
		{
			mem1[1] = (mem1[1] & (0xff & ~(1 << PINC0)));
		}
		if(PINC & 0b00000100)
		{
			mem1[1] |= (1 << PINC1);
		}
		else
		{
			mem1[1] = (mem1[1] &(0xff & ~(1 << PINC1)));
		}
		if(PINC & 0b00001000)
		{
			mem1[1] |= (1 << PINC2);
		}
		else
		{
			mem1[1] = (mem1[1] &(0xff & ~(1 << PINC2)));
		}
	}
}

/*static void proc2(void* pvParam)
{
	static uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x24};
	static uint8_t myip[4] = {192,168,1,108};
	static uint8_t buf[BUFFER_SIZE+1];
	uint16_t plen;
    char str[30];
	
	       
	      enc28j60Init(mymac);
	      enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
	       delay_ms(10);
	       

	       enc28j60PhyWrite(PHLCON,0x476);
	       delay_ms(20);
	       

	       //init the ethernet/ip layer:
	       init_ip_arp_udp_tcp(mymac,myip,MYWWWPORT);

	       while(1){
			   
		       // get the next new packet:
		       plen = enc28j60PacketReceive(BUFFER_SIZE, buf);

		       
		       if(eth_type_is_arp_and_my_ip(buf,plen)){
			       make_arp_answer_from_request(buf);
			       continue;
		       }
		       strcpy(str,"hello RTOS ETH :)");
               //str[0] = mem1[0];
		    //   make_udp_reply_from_request(buf,str,strlen(str),MYUDPPORT);
				int data = mem4[0];
				
		     	make_udp_reply_from_request(buf,(char*)&data,sizeof(int),MYUDPPORT);
		       vTaskDelay(500);
		       
	       }
}*/



void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{

}





