/*
 * RTOS.cpp
 *
 * Created: 2016-11-26 오후 2:50:59
 * Author : kimkisu
 */ 


//mem4 = 0//SV1 목표도달 온도값 1//디지털온도계1 PV값 2// 디지털 온도계1 SV값 OR 변경
//       3 인버터 상태 4인버터 해르즈 5인버터 속도 
//       7디지털온도계2 PV값 8현재 SV값 OR 변경 9 SV2 목표 도달 온도값
//      10 와치독 타이머 클리어 1: 클리어 0: HMI 이상 2_S 안에 클리어 해야됨.         
/* Scheduler include files. */

#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h>

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



#define HEATER1 5
#define HEATER2 6
#define ON 1
#define OFF 0
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
void detect_signal();
//Global Var

GetFunctionCode01 func01;
GetFunctionCode05 func05;
GetFunctionCode04 func04;
GetFunctionCode10 func10; 


RspExceptionCode exception;

ResponseFunctionCode10 rsp10;

char heater1_states = 0;  //히터 1의 상태를 표시 한다 1: On상태 0: OFF상태
char heater2_states = 0;  //히터 2의 상태를 표시 한다 1: On상태 0: OFF상태

char mem1[2]; //bit램프 표시를 위한 배열
int mem4[20] = {0}; //3x_MAX1 의 상태를 가지는 메모리
char mem5[20] = {0}; // 터치 버튼 입력시 값을 저장하는 메모리 

char buffer_flag = 0;


	char buzzer_stop = 0; //3번째 쓰레드 변수
	char warring_flag = 0; //3번째 쓰레드 변수 
enum  //mem4[]배열의 enum 값 
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
	RESERVED, //9
	WATCH_DOG, //10
	CONTROL_FOR_BUTTON, //11
	CONTROL_FOR_STATE_ON_OFF_HEATER, //12
	HEATER_SWITCH_STATE //13
};

//mem4 = 0//SV1 목표도달 온도값 1//디지털온도계1 PV값 2// 디지털 온도계1 SV값 OR 변경
//       3 인버터 상태 4인버터 해르즈 5인버터 속도
//       6 SV2 목표 도달 온도값 PV값 7현재 PV값  8 현재 SV값 OR 변경
//      10 와치독 타이머 클리어 1: 클리어 0: HMI 이상 2_S 안에 클리어 해야됨.
//      11 온도계1 온도 상하한선 12 온도계2 온도 상하한선
int main( void )
{
	cli();  //인터럽트 금지 
	
	DDRG = 0xff;                        
	PORTG = 0xff;
	

	DDRD |= 1;
	
	DDRE = 0xff;
	PORTE = 0x00;
	
	
	
	DDRB = 0xff;
	PORTB = 0xff;
	
	DDRF = 0xE0;   // 111 <--출력 00000 <-- 입력 0:scr1 1:scr2 2:inver1 3:inver2 4:부저 정지 버튼
	PORTF = 0xff; //풀업 사용 레지스터
	
	SFIOR = 0x00;  //풀업 Enable,Disable
	
	Init_Dev(); //dev 매니저 초기화
	
	
	
	dev->Open_Handle(UART0,Uart_ISR);  //드라이버 매니져에 인터럽트 루틴 등록
	
	dev->Open_Handle(RS485,RS485_ISR); //드라이버 매니져에 인터럽트 루틴 등록
	
	SerialBuffer *sb = new SerialBuffer(dev,UART0); //링 버퍼 
	SerialBuffer *sb1 = new SerialBuffer(dev,RS485); //링 버퍼 
	
	DataStruct[UART0] = sb;
	DataStruct[RS485] = sb1;

	sei(); //인터럽트 사용 
	//Serialinit();
	wdt_enable(WDTO_2S); //와치독 2초 
	
	
	//mem1[0] = 0xff;
	///mem1[1] = 0xff;
	
	xTaskCreate(proc,                //테스크 실행할 함수 포인터
	"Task1",      //테스크 이름
	350,                   //스택의 크기
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
		
				/*xTaskCreate(proc2,                //테스크 실행할 함수 포인터
				"Task3",      //테스크 이름
				240,                   //스택의 크기
				NULL,       // 테스크 매개 변수
				2,                     //테스크 우선 순위
				NULL                   //태스크 핸들
				);*/

			
		
	
	vTaskStartScheduler();//스케줄러 실행 
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
static void proc(void* pvParam) //터치패널 HMI RS232 쓰레드
{
	char read_Flag = 0;
	char function_code;
	char buf1[10];

	SerialBuffer *sb = (SerialBuffer*)pvParam;
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
			if(function_code == 0x01)  //비트램프
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
			else if(function_code == 0x04) //Max1W 값 읽기
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
			else if(function_code == 0x05) //터치 버튼 
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
			else if(function_code == 0x10)  //3_MAX1W 입력 
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
		if(mem4[WATCH_DOG] == 0x01) //2_S 안에 HMI 에서 1이라는 신호를 주기적으로 줘야 함..
		{
			wdt_reset(); //와치독 리셋
			mem4[WATCH_DOG] = 0;
		}
		detect_signal();
		
		
		////
	}
}
#define USE_TEMP 1
#define USE_TEMP1 1
#define USE_INVERTER 1

static void proc1(void* pvParam)  //RS485 통신 (인버터,한영넉스1,한영넉스2)쓰레드
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
	//wdt_disable();
	while(1)
	{
		detect_signal();
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
		
		if(sb->SerialAvailable() >= 2 && read_flag == 0)  //무조건 처음 2개 프로토콜 파싱.
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
		if(read_flag == 1) //디지털온도계 1번지 PV 값  + SV값 읽어오는 로직
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
		if(read_flag == 2) //인버터 프로토콜 파싱 로직
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
		if(read_flag == 3) //디지털온도계 3번지 PV 값  + SV값 읽어오는 로직
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
			if(adr == HEATER1)
			{
				heater1_states = ON;
				mem4[HEATER_SWITCH_STATE] = ON;
			}
			else if(adr == HEATER2)
			{
				heater2_states = ON;
				mem4[HEATER_SWITCH_STATE] = ON;
			}
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
			if(adr == HEATER1)  //버튼 주소가 히터이면 히터를 끈다
			{
				heater1_states = OFF;
			}
			else if(adr == HEATER2) //버튼 주소가 히터이면 히터를 끈다
			{
				heater2_states = OFF;
			}
			if(heater1_states == OFF && heater2_states == OFF) //히터상태가 둘다 OFF 일시 히터의 상태를 OFF로 바꾼다 
			{
				mem4[HEATER_SWITCH_STATE] = OFF;
			}
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
void detect_signal()
{
	/*if((mem1[0] & 0x80) && (mem1[1] & 0x1f)) //부저 복귀 조건 경고 램프가 다 ON 일시 부저 복귀 버튼은 리셋 됨.
	{
		buzzer_stop = 0;
		digital_OUT(&PORTF,BIT5,LOW); //경광등 OFF
	}*/
	if(PINF & 0x01) ///PORTF 0번째 핀 검출 
	{
		mem1[0] |= (1 << PINF7);
	}
	else if(~PINF & 0x01)
	{
		mem1[0] = (mem1[0] & (0xff & ~(1 << PINF7)));
	}
	if(PINF & 0x02)
	{
		mem1[1] |= (1 << PINF0);
	}
	else if(~PINF & 0x02)
	{
		mem1[1] = (mem1[1] & (0xff & ~(1 << PINF0)));
	}
	if(PINF & 0x04)
	{
		mem1[1] |= (1 << PINF1);
	}
	else if(~PINF & 0x04)
	{
		mem1[1] = (mem1[1] &(0xff & ~(1 << PINF1)));
	}
	if(PINF & 0x08)
	{
		mem1[1] |= (1 << PINF2);
	}
	else if(~PINF & 0x08)
	{
		mem1[1] = (mem1[1] &(0xff & ~(1 << PINF2)));
	}
	if(PINF & 0x10) //풀업상태 버튼 버튼이 안눌리면 핀 버튼의 값은 1을 유지한다.
	{
			// 아무것도 하는일이 없음..
	}
	else if(~PINF & 0x10)//부저 버튼이 눌리면,
	{
		buzzer_stop = 1;
		digital_OUT(&PORTF,BIT6,LOW); // 부저 OFF
	}
	if(mem4[CONTROL_FOR_BUTTON] == 1)
	{
		digital_OUT(&PORTF,BIT5,HIGH); //경광등 ON
		if(buzzer_stop == 0)
		{
			digital_OUT(&PORTF,BIT6,HIGH); // 부저 ON
		}
	}
	else if(mem4[CONTROL_FOR_BUTTON] == 0)  //상태가 정상이 되면 
	{
		buzzer_stop = 0;
		digital_OUT(&PORTF,BIT6,LOW); // 부저 OFF
		digital_OUT(&PORTF,BIT5,LOW); //경광등 OFF
	}
	if(heater1_states == ON || heater2_states == ON) //히터가 온 일시에 작동
	{
		if(mem4[CONTROL_FOR_STATE_ON_OFF_HEATER] == 1) //1번존 온도 과승 상태
		{
			mem1[1] = (mem1[1] &(0xff & ~(1 << PINF3)));  //1번존 온도 과승
			mem1[1] |= (1 << PINF4);  //온도 저하 램프 OFF
			if(heater1_states == ON)
			{
				digital_OUT(&PORTB,HEATER1,LOW);
			}
		}
		else if(mem4[CONTROL_FOR_STATE_ON_OFF_HEATER] == 2) //1번존 온도 저하상태
		{
			if(heater1_states == ON)
			{
				digital_OUT(&PORTB,HEATER1,HIGH);
			}
			mem1[1] = (mem1[1] &(0xff & ~(1 << PINF4))); //온도 저하 램프 온
			mem1[1] |= (1 << PINF3); // 온도 과승 램프 오프 
		}
		else if(mem4[CONTROL_FOR_STATE_ON_OFF_HEATER] == 0) //정상일시
		{
			//digital_OUT(&PORTF,BIT5,LOW); //경광등 OFF
			//digital_OUT(&PORTB,HEATER1,LOW);
			//digital_OUT(&PORTB,HEATER2,LOW);
			if(heater1_states == ON)
			{
				digital_OUT(&PORTB,HEATER1,HIGH);
			}
			mem1[1] |= (1 << PINF3);  //온도 램프 OFF
			mem1[1] |= (1 << PINF4);  //온도 램프 OFF
		}
		if(mem4[CONTROL_FOR_STATE_ON_OFF_HEATER] == 3) //2번존 온도 과승 상태
		{
			mem1[1] = (mem1[1] &(0xff & ~(1 << PINF5)));  //2번존 온도 과승
			mem1[1] |= (1 << PINF6);  //온도 램프 OFF
			if(heater2_states == ON)
			{
				digital_OUT(&PORTB,HEATER2,LOW);
			}
		}
		else if(mem4[CONTROL_FOR_STATE_ON_OFF_HEATER] == 4) //2번존 온도 저하 상태 
		{
			if(heater2_states == ON)
			{
				digital_OUT(&PORTB,HEATER2,HIGH);
			}
			mem1[1] |= (1 << PINF5);  //온도 램프 OFF
			mem1[1] = (mem1[1] &(0xff & ~(1 << PINF6)));  //2번존 온도 저하
		}
		else if(mem4[CONTROL_FOR_STATE_ON_OFF_HEATER] == 5) //2번존 온도 정상 상태 
		{
			if(heater2_states == ON)
			{
				digital_OUT(&PORTB,HEATER2,HIGH);
			}
			mem1[1] |= (1 << PINF5);  //온도 램프 OFF
			mem1[1] |= (1 << PINF6);  //온도 램프 OFF
		}
	}
	else
	{
		mem1[1] |= (1 << PINF3);  //경고 램프는 항상 ON 변화감지 -> ON -> OFF로 될시 감지
		mem1[1] |= (1 << PINF4);  //경고 램프는 항상 ON
		
		mem1[1] |= (1 << PINF5);  //경고 램프는 항상 ON 변화감지 -> ON -> OFF로 될시 감지
		mem1[1] |= (1 << PINF6);  //경고 램프는 항상 ON
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





