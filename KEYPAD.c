/*
 * KEYPAD.c
 *
 * Created: 30/01/2016 06:38:37 م
 *  Author: hossam
 */ 
#include "DIO.h"
#include "KEYPAD.h"

/*Local Symbols*/
#define kpdD_COL_PORT (PD)
#define KPd_ROW_PORT (PD)
#define KpPD_COL_MASK (0x70)
#define kPD_ROW_MASK (0x0f)
#define KPD_COL_PIN_NUM (4u)
#define KPD_ROW_PIN_NUM (0u)
/**************************************************/
 

void KPD_Init(void)
{
    DIO_vidWritePortDirection((u8)kpdD_COL_PORT,(u8)KpPD_COL_MASK,(u8)0x00);
    DIO_vidWritePortDirection((u8)KPd_ROW_PORT,(u8)kPD_ROW_MASK,(u8)0xff);
    DIO_vidWritePortData((u8)KPd_ROW_PORT,(u8)kPD_ROW_MASK,(u8)0x00);
	
}
void KPD_ReadVal(u8* ValuePtr)
{
    static const u8 KeysLut[]= { '1' , '2' , '3' , '4' , '5' , '6' , '7' , '8' , '9' , '*' , '0' , '#' };
	u8 Rowdata;
	u8 Col_Data;
	u8 LoopTermnate = 0u;
	for(Rowdata = 0u ; ((Rowdata < 4u) && (LoopTermnate == 0u)) ; Rowdata ++)
	{
	    u8 j;
	    j=((u8)1<<Rowdata);
	    u8 f=(j<< KPD_ROW_PIN_NUM);
	    DIO_vidWritePortData((u8)KPd_ROW_PORT,(u8)kPD_ROW_MASK,(u8)f);
	    DIO_vidReadPortData((u8)kpdD_COL_PORT,(u8)KpPD_COL_MASK,(&Col_Data));
	    *(&Col_Data) = (*(&Col_Data)) >> KPD_COL_PIN_NUM;
		if(Col_Data != 0u)
		{
		    u16 o=Rowdata*3u;
		    u8 i=Col_Data/2u;
			*ValuePtr = KeysLut[(u8)(o) + i];
			LoopTermnate = 1u;
		}
		else
		{
			*ValuePtr =(u8) 'n';
		}
	}

	
	
}

