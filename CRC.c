#include "Basic_Types.h"
/*we can't ignore this error in misra rules

ERROR :(MISRA-C:2004 19.4/R) C macros shall only expand to a braced initialiser, a constant, a string literal, a parenthesised expression, a type qualifier, a storage class specifier, or a do-while-zero construct

this error because of stdlib.h library for rand() fuction.
*/
#include <stdlib.h>
/*u32 rand_(void);*/
/***************************************************************************************/
static u32 GetPower(u32 Base,u32 Pow);
static u16 gen_crc16(const u8 *data, u16 size, u32 CRC16);
void SECR_CrcPolynomialGenerate(u32* PolynomialPtr,u8 CrcLengthInBits);
void SECR_GnerateCrc(const u8* PayloadPtr,u16 PayloadLength, u16* CrcPtr, u32 CrcPoly);
/***************************************************************************************/
void SECR_CrcPolynomialGenerate(u32* PolynomialPtr,u8 CrcLengthInBits)
{
    u32 DevisorValue;
    DevisorValue = (u32)(GetPower((u32)2,(u32)CrcLengthInBits)) - 1u;
    *PolynomialPtr = ((u32)(rand()) % DevisorValue) +(u32)0x10000 ;
}
/***************************************************************************************/
void SECR_GnerateCrc(const u8* PayloadPtr,u16 PayloadLength, u16* CrcPtr, u32 CrcPoly)
{
    u16 LoopIndex;
    static u8 InternalBuffer[8];
    /*Copying data to internal buffer*/
    for (LoopIndex = 0u; LoopIndex < PayloadLength; LoopIndex ++)
    {
        InternalBuffer[LoopIndex] = *(PayloadPtr += LoopIndex);
        PayloadPtr -= LoopIndex;
    }
    /*perform bit wise invert on the data*/
    for (LoopIndex = 0u; LoopIndex < PayloadLength; LoopIndex ++)
    {
        InternalBuffer[LoopIndex]  ^= 0xffu;
    }
    /*Generate CRC*/
    *CrcPtr = gen_crc16(InternalBuffer,PayloadLength*8u,0x18005u);
}
/***************************************************************************************/
static u32 GetPower(u32 Base,u32 Pow)
{
    u32 result = 1;
    u32 LoopIndex_;
    for (LoopIndex_ = 0u; LoopIndex_ < Pow; LoopIndex_ ++)
    {
        result *= Base;
    }
    return result;
}
/***************************************************************************************/
static u16 gen_crc16(const u8 *data, u16 size, u32 CRC16)
{
    u16 ReturnValue;
    u16 out = 0u;
    u16 bits_read = 0u, bit_flag;
    u16 i;
    u16 crc = 0u;
    u16 j = 0x0001u;
    /* Sanity check: */
    if(data == 0u)
    {
        ReturnValue=0u;
    }
    else
    {
        while(size > 0u)
        {
            bit_flag = out >> 15;

            /* Get next bit: */
            out <<= 1u;
            u8 x=(*data >> bits_read);
            u8 y=x&1u;
            out |= y ;/* item a) work from the least significant bits*/

            /* Increment bit counter: */
            bits_read++;
            if(bits_read > 7u)
            {
                bits_read = 0u;
                data+=1;
                size--;
            }

            /* Cycle check: */
            if(bit_flag)
            {
                out ^= CRC16;
            }
        }

        /* item b) "push out" the last 16 bits*/

        for (i = 0u; i < 16u; ++i) {
            bit_flag = out >> 15;
            out <<= 1;
            if(bit_flag)
            {
                out ^= CRC16;
            }

        }

        /* item c) reverse the bits*/

        i = 0x8000u;

        for (; i != 0u; i >>=1u)
        {
            if (i & out)
            {
                crc |= j;
            }
            j <<= 1u;
        }

        ReturnValue=crc;
    }
    return ReturnValue;


}
/***************************************************************************************/
