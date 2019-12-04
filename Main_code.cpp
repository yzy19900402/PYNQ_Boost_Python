#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/python.hpp>
#include <pthread.h>
#include <sys/wait.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>

using namespace boost::python;
using namespace std;
namespace bp = boost::python;

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long u64;

#define ServerIP_addr "192.168.0.200"
/******************** Constant Definitions **********************************/
/***********************************Control Values***********************************/
//OPLL Control signals
#define System_reset_OFF 0x00000001
#define System_reset_ON 0x00000000
#define SW_I_on 0x00000000
#define SW_I_off 0x00000001
#define single_type_INT 0x00000001
#define double_type_INT 0x00000000

#define Active_Ctl_on 0x00000001
#define Active_Ctl_off 0x00000000

///*********************************240us coefficients for 12.5MHz***************************/
#define REF_Clk_parameter 0x00000005
#define Trigger_stop_parameter 0x00007530//240us
#define Trigger_start_parameter 0x00000006//
#define DINT_Fall_parameter 0x00003A98//120us
#define DINT_Keep_parameter 0x00003A98//
#define Window_start_parameter 0x00000058//22clks//0x00000001//0clks//0x00000038//57clks//
#define Window_stop_parameter Trigger_stop_parameter + Window_start_parameter - 2//22clks//0x00003A9A//0clks//0x00003ACF//57clks//
#define Count_parameter Window_stop_parameter + 0x00000006//0x00003ADF//120us
#define BC_Interval_addr_paramter 0x00000800//0x00001000 corresponds addr 1 in bram exchanging
#define CMPs_Delay_Para 0x00000002

/************************Main control parameters**********************************/
#define BC_N_Hold_parameter 0x00000032//50//0x000001F4//500//I//0x0000000A//10//
#define INT_coe_parameter 0x00000000//0x2C60238E//2.096//0x11990439//2.1997//0x12660000//2.2998//0x10280000//2.0195//0x14000000//2.5996//0x12990000//2.3247
#define DAC_Offset_parameter 0x000007D1//16bit is the int mode, 1 for single, 0 for double


typedef struct CLK_CTL
{
	volatile u32 Count_para;
	volatile u32 Trigger_stop_para;
	volatile u32 Trigger_start_para;
	volatile u32 Window_start_para;
	volatile u32 Window_stop_para;
	volatile u32 DINT_Fall_para;
	volatile u32 DINT_Keep_para;
	volatile u32 System_reset_n;
}CLK_control;

typedef struct BRAM_CTL
{
	volatile u32 BC_N_Hold;
	volatile u32 SW_I;
	volatile u32 BC_Interval_addr;
}BRAM_control;

typedef struct INT_CTL
{
	volatile u32 INT_coe;
	volatile u32 INT_Type_s;
	volatile u32 DAC_Offset_para;
	volatile u32 Active_Control;
	volatile u32 Change_xor_step;
}INT_control;

/* For debug sends the infor by UART */
char buffer[128];
inline void ttyPrint(char* s)
{
	sprintf(buffer,"echo %s > /dev/ttyPS0",s);
	system(buffer);
}

class Controller{
private:
public:
	unsigned int mem_fd;
    CLK_control *CLK_Changable;
    BRAM_control* TriBRAM_Control;
    u32 *TDC_Control;
    INT_control *DINT_Control;
    u32 *CMP_Delay;
	u32 iterated_times = 0;

    
    Controller(
    	u32 TriBRAM_Control_Addr, 
    	u32 CLK_Control_Addr, 
    	u32 TDC_Control_Addr, 
    	u32 DINT_Control_Addr, 
    	u32 CMP_Delay_Addr
		){

    	mem_fd = open("/dev/mem", O_RDWR);
		CLK_Changable = (CLK_control*)mmap(0x00,sizeof(CLK_control) , PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, CLK_Control_Addr);
		if(CLK_Changable == NULL)
		{
			printf("CLK_Changable Initialized failed!Exit!\n");
			exit(-1);
		}
		TriBRAM_Control = (BRAM_control*)mmap(0x00,sizeof(BRAM_control) , PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, TriBRAM_Control_Addr);
		if(TriBRAM_Control == NULL)
		{
			printf("TriBRAM_Control Initialized failed!Exit!\n");
			exit(-1);
		}
		TDC_Control = (u32*)mmap(0x00,sizeof(u32) , PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, TDC_Control_Addr);
		if(TDC_Control == NULL)
		{
			printf("TDC_Control Initialized failed!Exit!\n");
			exit(-1);
		}
		DINT_Control = (INT_control*)mmap(0x00,sizeof(INT_control) , PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, DINT_Control_Addr);
		if(DINT_Control == NULL)
		{
			printf("DINT_Control Initialized failed!Exit!\n");
			exit(-1);
		}
		CMP_Delay = (u32*)mmap(0x00,sizeof(u32) , PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, CMP_Delay_Addr);
		if(CMP_Delay == NULL)
		{
			printf("CMP_Delay Initialized failed!Exit!\n");
			exit(-1);
		}

		//Initial parameters
	    /*****************reset system***************************/
	    CLK_Changable->System_reset_n = System_reset_ON;

	    /******************CLK Module****************************/
	    CLK_Changable->Count_para = Count_parameter;
	    CLK_Changable->DINT_Fall_para = DINT_Fall_parameter;
	    CLK_Changable->DINT_Keep_para = DINT_Keep_parameter;
	    CLK_Changable->Trigger_start_para = Trigger_start_parameter;
	    CLK_Changable->Trigger_stop_para = Trigger_stop_parameter;
	    CLK_Changable->Window_start_para = Window_start_parameter;
	    CLK_Changable->Window_stop_para = Window_stop_parameter;

	    /******************BRAM Contrl Module********************/
	    TriBRAM_Control->SW_I = SW_I_off;
	    TriBRAM_Control->BC_N_Hold = BC_N_Hold_parameter;
	    TriBRAM_Control->BC_Interval_addr = BC_Interval_addr_paramter;

	    /*******************TDC Module***************************/
	    *TDC_Control = REF_Clk_parameter;

	    /*******************INT Module***************************/
	    DINT_Control->INT_coe = INT_coe_parameter;
	    DINT_Control->INT_Type_s = single_type_INT;
	    DINT_Control->DAC_Offset_para = DAC_Offset_parameter;
	    DINT_Control->Active_Control = Active_Ctl_off;
	    DINT_Control->Change_xor_step = INT_coe_parameter;

	    /********************CMP Delay control*******************/
	    *CMP_Delay = CMPs_Delay_Para;	
    }

    ~Controller()
    {
		munmap(CLK_Changable, sizeof(CLK_control));
		munmap(TriBRAM_Control, sizeof(BRAM_control));
		munmap(TDC_Control, sizeof(u32));
		munmap(DINT_Control, sizeof(INT_control));
		munmap(CMP_Delay, sizeof(u32));
		close(mem_fd);
    }

    void system_rst_on()
	{
		CLK_Changable->System_reset_n = System_reset_ON;
		iterated_times = 0;
	}
	void system_rst_off()
	{
		CLK_Changable->System_reset_n = System_reset_OFF;
		DINT_Control->INT_coe = 0;
	    DINT_Control->DAC_Offset_para = DAC_Offset_parameter;
	    usleep(10000);
	    iterated_times = 0;
	}

	void Set_Trigger(
		u32 Trig_start,
	 	u32 Trig_end,
	 	u32 Wind_start,
	 	u32 Wind_stop,
	 	u32 DINT_fall,
	 	u32 DINT_keep )
	{
		CLK_Changable->Count_para 			= Trig_end + 6;
		CLK_Changable->Trigger_start_para 	= Trig_start;
		CLK_Changable->Trigger_stop_para 	= Trig_end;
		CLK_Changable->Window_start_para 	= Wind_start;
		CLK_Changable->Window_stop_para 	= Wind_stop;
		CLK_Changable->DINT_Fall_para 		= DINT_fall;
		CLK_Changable->DINT_Keep_para 		= DINT_keep;
	}

	void Active_Control_on()
	{
		DINT_Control->Active_Control 		= Active_Ctl_on;
	}

	void Active_Control_off()
	{
		DINT_Control->Active_Control 		= Active_Ctl_off;
	}

	void Set_INT_coe(u32 INT_coe)
	{
		DINT_Control->INT_coe 				= INT_coe;
	}

//Select the integrate mode in falling control, 1(default) for single, 0 for double
	void Iterating(u32 Iteration_mode)
	{
		if(DINT_Control->Active_Control == Active_Ctl_on)
			printf("\nIteration is forbidden in the active control\n\n");
		else
		{
			TriBRAM_Control->SW_I = SW_I_on;
			if(Iteration_mode == single_type_INT)
				DINT_Control->INT_Type_s = single_type_INT;
			else
				DINT_Control->INT_Type_s = double_type_INT;
			usleep(1000);
			TriBRAM_Control->SW_I = SW_I_off;
			iterated_times = iterated_times + 1;
			printf("iterated %d time(s)\n",iterated_times);
		}
	}

	void Set_Ref(u32 Refer)
	{
		iterated_times = 0;
		CLK_Changable->System_reset_n = System_reset_ON;
		*TDC_Control = Refer;
		usleep(1000);
		CLK_Changable->System_reset_n = System_reset_OFF;
	}

	void Set_BM(u32 BC_interval)
	{
		CLK_Changable->System_reset_n = System_reset_ON;
		DINT_Control->INT_coe = INT_coe_parameter;
		TriBRAM_Control->BC_Interval_addr = BC_interval;
		usleep(1000);
		CLK_Changable->System_reset_n = System_reset_OFF;
	}

	void Set_Delay(u32 Delay)
	{
		*CMP_Delay = Delay;	
	}
    
	void Set_Offset(u32 Offset)
    	{
        	CLK_Changable->System_reset_n = System_reset_ON;
		usleep(1000);
        	DINT_Control->DAC_Offset_para = Offset;
	        CLK_Changable->System_reset_n = System_reset_OFF;
    	}
};

BOOST_PYTHON_MODULE(Controller)
{
	class_<Controller>("Controller", init<u32,u32,u32,u32,u32>())
		.def(init<u32,u32,u32,u32,u32>())
		.def("system_rst_on",&Controller::system_rst_on)
		.def("system_rst_off",&Controller::system_rst_off)
		.def("Set_Trigger",&Controller::Set_Trigger)
		.def("Active_Control_on",&Controller::Active_Control_on)
		.def("Active_Control_off",&Controller::Active_Control_off)
		.def("Set_INT_coe",&Controller::Set_INT_coe)
		.def("Iterating",&Controller::Iterating)
		.def("Set_Ref",&Controller::Set_Ref)
		.def("Set_BM",&Controller::Set_BM)
		.def("Set_Delay",&Controller::Set_Delay)
		.def("Set_Offset",&Controller::Set_Offset)
		// .def_readonly("TH_L",&Controller::TH_L)
		// .def_readonly("LENGTH",&Controller::byte_length)
		// .def_readonly("PHYADDR",&Controller::mem_addr)
		;
}
