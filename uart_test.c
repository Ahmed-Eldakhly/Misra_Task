


#include "Basic_Types.h"
#include "BLMGR.h"


#include "PWM.h"
#include "GPT.h"

#define Num_0 (0)
#define Num_1 (1)
#define Num_4 (4)
#define Num_5 (5)
#define Num_10 (10)
#define Num_20 (20)
#define Num_25 (25)
#define Num_100 (100)

void Cyclic30ms(void);
void _delay_ms(s32 Delay);
void sei(void);
u32 main(void);
u32 main(void)
{
    s32 Count= 0u;
    s32 start = 0u;
    s32 Count2 = 0u;
 


GPT_Timer30msInit(&Cyclic30ms);
	
	BLMGR_BluetoothInit();
	BLMGR_SetReceiver((u8)ROLE_MAPP);
    
							
	PWM_Init();
	sei();
	PWM_SetSpeed((u8)Num_25);

	
	while(1)
	{
		Count2 = (Count2 +(s32)Num_1) %Num_20;
		/*count is s32 but i cast it to u8*/
		BLMGR_SetBattLevel(((u8)Count2 /(u8) Num_4));
		
		_delay_ms((s32)Num_100);
		Count ++;
		if(start == Num_0)
		{
					if(Count > Num_5)
					{
						BLMGR_StartDevice();

						start = Num_5;
					}
					
		}

	    
		
		
	}
	
	
}
void Cyclic30ms(void)
{
    static u8 TimeoutCounter = Num_0;
    TimeoutCounter ++;
    if(TimeoutCounter == (u8)Num_10)
    {
        BLMGR_BluetoothStateMachine();
        TimeoutCounter = (u8)Num_0;


    }
}
