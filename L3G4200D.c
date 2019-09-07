/*
 * L3G4200D.c
 *
 * Created: 24/09/2015 09:13:31 م
 *  Author: hossam
 */ 

#include "L3G4200D.h"
#include "L3G4200D_Cfg.h"
#include "L3G4200D_Lcfg.h"
#include "SPI.h"
/*****************************************************************************************************/
/* Local Symbols */
/*****************************************************************************************************/
/*Registers addresses*/
#define WHO_AM_I_REG_ADD (0x0f)
#define CTRL_REG1_ADD (0x20)
#define CTRL_REG2_ADD (0x21)
#define CTRL_REG3_ADD (0x22)
#define CTRL_REG4_ADD (0x23)
#define CTRL_REG5_ADD (0x24)
#define REFERENCE_ADD (0x25)
#define OUT_TEMP_ADD (0x26)
#define STATUS_REG_ADD (0x27)
#define OUT_X_L_ADD (0x28)
#define OUT_X_H_ADD (0x29)
#define OUT_Y_L_ADD (0x2a)
#define OUT_Y_H_ADD (0x2b)
#define OUT_Z_L_ADD (0x2c)
#define OUT_Z_H_ADD (0x2d)
#define FIFO_CTRL_REG_ADD (0x2e)
#define FIFO_SRC_REG_ADD (0x2f)
#define INT1_CFG_ADD (0x30)
#define INT1_SRC_ADD (0x31)
#define INT1_TSH_XH_ADD (0x32)
#define INT1_TSH_XL_ADD (0x33)
#define INT1_TSH_YH_ADD (0x34)
#define INT1_TSH_YL_ADD (0x35)
#define INT1_TSH_ZH_ADD (0x36)
#define INT1_TSH_ZL_ADD (0x37)
#define INT1_DURATION_ADD (0x38)
/*****************************************************************************************************/
/*General Write Read Macros*/

/*#define L3G4200D_REG_WRITE(ADD,DATA) GYHD_ACTIVATE_SLAVE_SELECT();\
        SPI_u8TransfereByte(ADD);\
        SPI_u8TransfereByte(DATA);\
        GYHD_DEACTIVATE_SLAVE_SELECT()

#define L3G4200D_REG_READ(ADD,DATA_PTR) GYHD_ACTIVATE_SLAVE_SELECT();\
        SPI_u8TransfereByte(ADD|0x80);\
        (*DATA_PTR) = SPI_u8TransfereByte(0x00);\
        GYHD_DEACTIVATE_SLAVE_SELECT()*/
/*****************************************************************************************************/
/* MAcros To Axis Registers*/
/*#define L3G4200D_READ_WHO_AM_I(DATA_PTR)           L3G4200D_REG_READ(WHO_AM_I_REG_ADD,DATA_PTR)
#define L3G4200D_READ_CTRL_REG1(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG1_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG1(DATA)             L3G4200D_REG_WRITE(CTRL_REG1_ADD,DATA)
#define L3G4200D_READ_CTRL_REG2(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG2_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG2(DATA)             L3G4200D_REG_WRITE(CTRL_REG2_ADD,DATA)
#define L3G4200D_READ_CTRL_REG3(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG3_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG3(DATA)             L3G4200D_REG_WRITE(CTRL_REG3_ADD,DATA)
#define L3G4200D_READ_CTRL_REG4(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG4_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG4(DATA)             L3G4200D_REG_WRITE(CTRL_REG4_ADD,DATA)
#define L3G4200D_READ_CTRL_REG5(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG5_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG5(DATA)             L3G4200D_REG_WRITE(CTRL_REG5_ADD,DATA)
#define L3G4200D_READ_REFERENCE(DATA_PTR)          L3G4200D_REG_READ(REFERENCE_ADD,DATA_PTR)
#define L3G4200D_WRITE_REFERENCE(DATA)             L3G4200D_REG_WRITE(REFERENCE_ADD,DATA)
#define L3G4200D_READ_OUT_TEMP(DATA_PTR)           L3G4200D_REG_READ(OUT_TEMP_ADD,DATA_PTR)
#define L3G4200D_READ_STATUS_REG(DATA_PTR)         L3G4200D_REG_READ(STATUS_REG_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_X_L(DATA_PTR)            L3G4200D_REG_READ(OUT_X_L_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_X_H(DATA_PTR)            L3G4200D_REG_READ(OUT_X_H_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_Y_L(DATA_PTR)            L3G4200D_REG_READ(OUT_Y_L_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_Y_H(DATA_PTR)            L3G4200D_REG_READ(OUT_Y_H_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_Z_L(DATA_PTR)            L3G4200D_REG_READ(OUT_Z_L_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_Z_H(DATA_PTR)            L3G4200D_REG_READ(OUT_Z_H_ADD,DATA_PTR)
#define L3G4200D_READ_FIFO_CTRL_REG(DATA_PTR)      L3G4200D_REG_READ(FIFO_CTRL_REG_ADD,DATA_PTR)
#define L3G4200D_WRITE_FIFO_CTRL_REG(DATA)         L3G4200D_REG_WRITE(FIFO_CTRL_REG_ADD,DATA)
#define L3G4200D_READ_FIFO_SRC_REG(DATA_PTR)       L3G4200D_REG_READ(FIFO_SRC_REG_ADD,DATA_PTR)
#define L3G4200D_READ_INT1_CFG(DATA_PTR)           L3G4200D_REG_READ(INT1_CFG_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_CFG(DATA)              L3G4200D_REG_WRITE(INT1_CFG_ADD,DATA)
#define L3G4200D_READ_INT1_SRC(DATA_PTR)           L3G4200D_REG_READ(INT1_SRC_ADD,DATA_PTR)
#define L3G4200D_READ_INT1_TSH_XH(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_XH_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_XH(DATA)           L3G4200D_REG_WRITE(INT1_TSH_XH_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_XL(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_XL_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_XL(DATA)           L3G4200D_REG_WRITE(INT1_TSH_XL_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_YH(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_YH_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_YH(DATA)           L3G4200D_REG_WRITE(INT1_TSH_YH_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_YL(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_YL_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_YL(DATA)           L3G4200D_REG_WRITE(INT1_TSH_YL_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_ZH(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_ZH_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_ZH(DATA)           L3G4200D_REG_WRITE(INT1_TSH_ZH_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_ZL(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_ZL_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_ZL(DATA)           L3G4200D_REG_WRITE(INT1_TSH_ZL_ADD,DATA)
#define L3G4200D_READ_INT1_DURATION(DATA_PTR)      L3G4200D_REG_READ(INT1_DURATION_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_DURATION(DATA)         L3G4200D_REG_WRITE(INT1_DURATION_ADD,DATA)*/
/*****************************************************************************************************/
/*Register Default Value*/
#define L3G4200D_WHO_AM_I_DEFAULT 0xD3                                  
/*****************************************************************************************************/
/*Operational Modes Management*/
#define u8PWR_DN_MOD 0x00
#define u8SLEEP_MOD  0x08
#define u8NORMAL_MOD 0x0f
#define u8MOD_CLR_MASK 0xf0
#define u8TO_SLEEP_MODE_DELAY_MS 0x05
/*****************************************************************************************************/
/*CTRL_REG4 Initialization*/
#define u8BLOCK_DATA_UPDATE 0x00u
#define	u8BIG_LITTLE_ENDIAN 0x00u
#define u8SELF_TEST_MODE    0x00u
#define u8SPI_MODE          0x00u
#define u8CTRL_REG4_INIT (u8BLOCK_DATA_UPDATE | u8BIG_LITTLE_ENDIAN | u8SELF_TEST_MODE | u8SPI_MODE)
/*****************************************************************************************************/
/*Reading Operation*/
#define u8NEW_DATA_AVAILABEL_X_MASK 0x01
#define u8NEW_DATA_AVAILABEL_Y_MASK 0x02
#define u8NEW_DATA_AVAILABEL_Z_MASK 0x04
#define u8DATA_OVER_WRITTEN_MASK    0x80
#define u8AXIS_IS_ENABLED           0x00
#define u8AXIS_IS_DISABLED          0x01
/*****************************************************************************************************/
/*Filters Configuration*/
#define u8FILTERS_CLR_MASK 0xE0
/*****************************************************************************************************/
/*Self Axis Movement Config*/
#if (u8SELF_AXIS_MOV == ON)
#define u8INT1_ENABLE 0x80
#endif
/*****************************************************************************************************/
/*Static Variables*/
/*****************************************************************************************************/
static u8 GYHD_u8LastPowerMode = u8PWR_DN_MOD;		
static u8 GYHD_u8TimeOutFlag;
static u8 GYHD_u8StartTimeoutFlag;	
/*		*/
/*****************************************************************************************************/
/*Private functions declarations*/
/*****************************************************************************************************/
static u8 u8CheckTimeOut(u8 u8TimeInMs);
static void vidManageOperationModes(u8 u8NewMode);
static u8 udtSelfTest(void);
static void vidInitCtrlReg4(void);
static void vidCheckForNewData(u8* pu8Status, u8 u8AxisMask);
static u16 u16ReadAxisData(u8 AxisMask);
static u8 u8CheckAxisIsEnabled(u8 u8AxisMask);
static void vidInitFilters(void);
#if (u8SELF_AXIS_MOV == ON)
/*static void vidInitSelectiveAxisMovement(void);*/
void u8START_TIME_OUT_MS(u8 aa,u8* bb);
void GYHD_ACTIVATE_SLAVE_SELECT(void);
void GYHD_DEACTIVATE_SLAVE_SELECT(void);
#endif
/*****************************************************************************************************/
/* Global Services */
/*****************************************************************************************************/
u8 GYHD_Init(void)
{
    u8 RetVal;

    DIO_WritePort((u8)PB,(u8)((u8)1<<4u),(u8)((u8)1<<4u));
    SPI_vidInit();
    GYHD_u8TimeOutFlag = 0u;
    GYHD_u8StartTimeoutFlag = 0u;
    /*To Do: Initialized Module Registers with the Specific values*/
    /*Wait for Entering Power Down Mode*/
    do
    {
        u8START_TIME_OUT_MS((u8) 250,&GYHD_u8TimeOutFlag);
    }while (GYHD_u8TimeOutFlag == 0u);
    GYHD_u8TimeOutFlag = 0u;

    /*Perform Sensor Self Test*/
    RetVal = udtSelfTest();
    if(RetVal == OK)
    {
        /*Init Power Mode*/
        vidManageOperationModes((u8)u8PWR_DN_MOD);
        GYHD_u8LastPowerMode = (u8)u8PWR_DN_MOD;

        /*Init FullScale, Data Order, Spi Mode and block data update*/
        vidInitCtrlReg4();

        /*Init Filters*/
        vidInitFilters();
    }
    else
    {
        /* Self Test Failed, a Problem in Communication Found*/
    }
    return RetVal;
}
/*****************************************************************************************************/
void GYHD_WakeUpModule(u8* u8WakeUpStatePtr)
{
    vidManageOperationModes((u8)u8NORMAL_MOD);
    if((GYHD_u8LastPowerMode == (u8)u8NORMAL_MOD) && (GYHD_u8StartTimeoutFlag == 0u))
    {
        *u8WakeUpStatePtr =(u8) u8WAKEUP_DN;
    }
    else
    {
        *u8WakeUpStatePtr =(u8) u8WAKEUP_IN_PROGRESS;
    }

}
/*****************************************************************************************************/
void GYHD_SleepModule(void)
{
    vidManageOperationModes((u8)u8SLEEP_MOD);
}
/*****************************************************************************************************/
u8 GYHD_ReadXDirection(u16* pu16Data,u8* pu8ReadingStatus)
{
    u8 RetVar = NOK;
    u8 u8IsEnabled;
    u8 u8Axis_Mask = (u8)u8NEW_DATA_AVAILABEL_X_MASK;
    /*Check if Axis was enabled*/
    u8IsEnabled = u8CheckAxisIsEnabled(u8Axis_Mask);
    if(u8IsEnabled == (u8)u8AXIS_IS_ENABLED)
    {
        /*Check for Data Updated*/
        vidCheckForNewData(pu8ReadingStatus,u8Axis_Mask);
        if(*pu8ReadingStatus ==(u8) u8READING_OK)
        {
            /*Read Data*/
            *pu16Data = u16ReadAxisData(u8Axis_Mask);
            RetVar = OK;
        }
        else
        {
            /*Data overwritten and Pointer Initialized*/
        }
    }
    else
    {
        /*Axis Disabled*/
        *pu8ReadingStatus = (u8)u8AXIS_DISABLED;
    }

    return RetVar;
}
/*****************************************************************************************************/
u8 GYHD_ReadYDirection(u16* pu16Data,u8* pu8ReadingStatus)
{
    u8 Ret_Var = NOK;
    u8 u8Is_Enabled;
    u8 u8_AxisMask = u8NEW_DATA_AVAILABEL_Y_MASK;
    /*Check if Axis was enabled*/
    u8Is_Enabled = u8CheckAxisIsEnabled(u8_AxisMask);
    if(u8Is_Enabled == (u8)u8AXIS_IS_ENABLED)
    {
        /*Check for Data Updated*/
        vidCheckForNewData(pu8ReadingStatus,u8_AxisMask);
        if(*pu8ReadingStatus == (u8)u8READING_OK)
        {
            /*Read Data*/
            *pu16Data = u16ReadAxisData(u8_AxisMask);
            Ret_Var = OK;
        }
        else
        {
            /*Data overwritten and Pointer Initialized*/
        }
    }
    else
    {
        /*Axis Disabled*/
        *pu8ReadingStatus = (u8)u8AXIS_DISABLED;
    }

    return Ret_Var;
}
/*****************************************************************************************************/
u8 GYHD_ReadZDirection(u16* pu16Data,u8* pu8ReadingStatus)
{
    u8 RetVar_ = NOK;
    u8 u8_IsEnabled;
    u8 u8AxisMask_ = u8NEW_DATA_AVAILABEL_Z_MASK;
    /*Check if Axis was enabled*/
    u8_IsEnabled = u8CheckAxisIsEnabled(u8AxisMask_);
    if(u8_IsEnabled ==(u8) u8AXIS_IS_ENABLED)
    {
        /*Check for Data Updated*/
        vidCheckForNewData(pu8ReadingStatus,u8AxisMask_);
        if(*pu8ReadingStatus ==(u8) u8READING_OK)
        {
            /*Read Data*/
            *pu16Data = u16ReadAxisData(u8AxisMask_);
            RetVar_ = OK;
        }
        else
        {
            /*Data overwritten and Pointer Initialized*/
        }
    }
    else
    {
        /*Axis Disabled*/
        *pu8ReadingStatus =(u8) u8AXIS_DISABLED;
    }

    return RetVar_;
}
/*****************************************************************************************************/
/*Private functions definitions*/
/*****************************************************************************************************/
static void vidCheckForNewData(u8* pu8Status, u8 u8AxisMask)
{
    u8 u8Status;
    DIO_WritePort((u8)PB,(u8)(~(u8)((u8)1<<4u)),(u8)((u8)1<<4u));
    SPI_u8TransfereByte((u8)((u8)STATUS_REG_ADD|0x80u));
    (*&u8Status) = SPI_u8TransfereByte(0x00u);
    DIO_WritePort((u8)PB,(u8)((u8)1<<4u),(u8)((u8)1<<4u));
    /*Check for Data updated*/
    if((u8Status & u8AxisMask) == u8AxisMask)
    {
        /*Check for Data Overwritten*/
        if (((u8)u8Status & (u8)u8DATA_OVER_WRITTEN_MASK) == (u8)u8DATA_OVER_WRITTEN_MASK)
        {
            *pu8Status =(u8) u8DATA_OVERWRITTEN;
        }
        else
        {
            *pu8Status =(u8) u8READING_OK;
        }
    }
    else
    {
        *pu8Status =(u8) u8NO_DATA_UPDATED;
    }

}
/*****************************************************************************************************/
static u8 u8CheckAxisIsEnabled(u8 u8AxisMask)
{
    u8 _u8IsEnabled;
    u8 _u8RegValue;
    /*Read Control Register*/
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)CTRL_REG1_ADD|0x80u);
    (*&_u8RegValue) = SPI_u8TransfereByte(0x00u);
    GYHD_DEACTIVATE_SLAVE_SELECT();
    /*Check for access Status*/
    if(((u8)(_u8RegValue & (u8)u8AxisMask)) ==(u8) u8AxisMask)
    {
        _u8IsEnabled =(u8) u8AXIS_IS_ENABLED;
    }
    else
    {
        _u8IsEnabled =(u8) u8AXIS_IS_DISABLED;
    }
    return _u8IsEnabled;
}
/*****************************************************************************************************/
static u16 u16ReadAxisData(u8 AxisMask)
{
    u16 u16AxisData;
    u8 u8RegData1;
    u8 u8RegData2;
    switch(AxisMask)
    {
    case (u8)u8NEW_DATA_AVAILABEL_X_MASK:
        {
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)OUT_X_L_ADD|0x80u);
        (*&u8RegData1) = SPI_u8TransfereByte(0x00u);
        GYHD_DEACTIVATE_SLAVE_SELECT();

        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)OUT_X_H_ADD|0x80u);
        (*&u8RegData2) = SPI_u8TransfereByte(0x00u);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        u16AxisData = ((u8RegData1) | (u16)(((u16)u8RegData2)<<8u));
        }
    break;
    case (u8)u8NEW_DATA_AVAILABEL_Y_MASK:
        {
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)OUT_Y_L_ADD|0x80u);
        (*&u8RegData1) = SPI_u8TransfereByte(0x00u);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)OUT_Y_H_ADD|0x80u);
        (*&u8RegData2) = SPI_u8TransfereByte(0x00u);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        u16AxisData = u8RegData1 |(u16) (((u16)u8RegData2<<8));
        }
    break;
    case (u8)u8NEW_DATA_AVAILABEL_Z_MASK:
        {
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)OUT_Z_L_ADD|0x80u);
        (*&u8RegData1) = SPI_u8TransfereByte(0x00u);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)OUT_Z_H_ADD|0x80u);
        (*&u8RegData2) = SPI_u8TransfereByte((u8)0x00);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        u16AxisData = u8RegData1 | ((u16)((u16)u8RegData2<<8));
        }
    break;
    default:
        u16AxisData = 0x00u;
        break;
    }
    return u16AxisData;
}

/*****************************************************************************************************/
static void vidInitCtrlReg4(void)
{
    static u8 GYHD_u8FullScale;
    u8 u8InitParams = 0x00u;
    GYHD_u8FullScale = L3G4200D_ConfigParam.u8FullScaleValue;
    u8InitParams|= (GYHD_u8FullScale |(u8) u8CTRL_REG4_INIT);
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)CTRL_REG4_ADD);
    SPI_u8TransfereByte(u8InitParams);
    GYHD_DEACTIVATE_SLAVE_SELECT();

}	
/*****************************************************************************************************/
static void vidManageOperationModes(u8 u8NewMode)
{
    u8 u8RegData;
    switch(u8NewMode)
    {
    case (u8)u8PWR_DN_MOD:
        {
        /*Clear the Power Mode bits in the register and write the mode values*/
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)CTRL_REG1_ADD|0x80u);
        (*&u8RegData) = SPI_u8TransfereByte(0x00u);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        u8RegData &= (u8)u8MOD_CLR_MASK;
        u8RegData |= u8NewMode;
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)CTRL_REG1_ADD);
        SPI_u8TransfereByte(u8RegData);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        GYHD_u8LastPowerMode =(u8) u8PWR_DN_MOD;
        }
    break;

    case (u8)u8SLEEP_MOD:
        {
        /*Clear the Power Mode bits in the register and write the mode values*/
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)CTRL_REG1_ADD|0x80u);
        (*&u8RegData) = SPI_u8TransfereByte(0x00u);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        u8RegData &= (u8)u8MOD_CLR_MASK;
        u8RegData |= u8NewMode;
        GYHD_ACTIVATE_SLAVE_SELECT();
        SPI_u8TransfereByte((u8)CTRL_REG1_ADD);
        SPI_u8TransfereByte(u8RegData);
        GYHD_DEACTIVATE_SLAVE_SELECT();
        GYHD_u8LastPowerMode = (u8)u8SLEEP_MOD;
        }
    break;

    case (u8)u8NORMAL_MOD:
        {
        /*Wait for 250ms*/
        if(GYHD_u8StartTimeoutFlag == 0u)
        {
            /*Clear axis activation bits*/
            u8NewMode &= 0xF8u;
            /*Write Activation signals*/
            u8NewMode |=(u8) (L3G4200D_ConfigParam.strAxisActivation.u8XAxisActivation | L3G4200D_ConfigParam.strAxisActivation.u8YAxisActivation | L3G4200D_ConfigParam.strAxisActivation.u8ZAxisActivation);
            GYHD_u8TimeOutFlag = 0u;
            /*Clear the Power Mode bits in the register and write the mode values*/
            GYHD_ACTIVATE_SLAVE_SELECT();
            SPI_u8TransfereByte((u8)CTRL_REG1_ADD|0x80u);
            (*&u8RegData) = SPI_u8TransfereByte(0x00u);
            GYHD_DEACTIVATE_SLAVE_SELECT();
            u8RegData &=(u8) u8MOD_CLR_MASK;
            u8RegData |= u8NewMode;
            GYHD_ACTIVATE_SLAVE_SELECT();
            SPI_u8TransfereByte((u8)CTRL_REG1_ADD);
            SPI_u8TransfereByte(u8RegData);
            GYHD_DEACTIVATE_SLAVE_SELECT();
        }
        else
        {
            if(GYHD_u8LastPowerMode == (u8)u8SLEEP_MOD)
            {
                GYHD_u8TimeOutFlag = u8CheckTimeOut((u8)u8TO_SLEEP_MODE_DELAY_MS);
            }
            else
            {
                GYHD_u8TimeOutFlag =(u8) u8CheckTimeOut(250u);
            }

            if(GYHD_u8TimeOutFlag == 1u)
            {
                GYHD_u8StartTimeoutFlag = 0u;
                GYHD_u8LastPowerMode =(u8) u8NORMAL_MOD;
            }
            else
            {
                /* Do nothing: The timeout didn't pass*/
            }
        }
        }
    break;
    default:
        break;
    }
}
/*****************************************************************************************************/
static u8 u8CheckTimeOut(u8 u8TimeInMs)
{
    u8 u8TimeOutFlag=0;
    u8START_TIME_OUT_MS((u8)u8TO_SLEEP_MODE_DELAY_MS,&u8TimeOutFlag);
    GYHD_u8StartTimeoutFlag = 1u;
    return u8TimeOutFlag;
}
/*****************************************************************************************************/
static u8 udtSelfTest(void)
{
    u8 RetVal_;
    u8 RegData;
    /*Read the Who AM I? Register*/
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)WHO_AM_I_REG_ADD|0x80u);
    (*&RegData) = SPI_u8TransfereByte(0x00u);
    GYHD_DEACTIVATE_SLAVE_SELECT();
    if(RegData == (u8)L3G4200D_WHO_AM_I_DEFAULT)
    {
        RetVal_ = OK;
    }
    else
    {
        RetVal_ = NOK;
    }
    return RetVal_;
}
/*****************************************************************************************************/
static void vidInitFilters(void)
{
    u8 u8RegData_;
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)CTRL_REG5_ADD|0x80u);
    (*&u8RegData_) = SPI_u8TransfereByte(0x00u);
    GYHD_DEACTIVATE_SLAVE_SELECT();
    u8RegData_ &=(u8) u8FILTERS_CLR_MASK;
    u8RegData_ |=(u8) (L3G4200D_ConfigParam.strFilterCfg.u8FilterOnData | L3G4200D_ConfigParam.strFilterCfg.u8FilterOnInterrupt);
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte(u8RegData_);
    GYHD_DEACTIVATE_SLAVE_SELECT();
}
/*****************************************************************************************************/
#if(u8SELF_AXIS_MOV == ON)
/*static void vidInitSelectiveAxisMovement(void)
{
    u8 u8RegData__;
    u8 u8Threshold;
    Enable Interrupt 1

    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)CTRL_REG5_ADD);
    SPI_u8TransfereByte((u8)u8INT1_ENABLE);
    GYHD_DEACTIVATE_SLAVE_SELECT();

    Write threshold values for each axis
    u8Threshold = (u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdX;
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)INT1_TSH_XL_ADD);
    SPI_u8TransfereByte((u8)u8Threshold);
    GYHD_DEACTIVATE_SLAVE_SELECT();
    u8Threshold = (u8)((u16)(L3G4200D_ConfigParam.strIntCfg.u16ThresholdX>>8U));
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)INT1_TSH_XH_ADD);
    SPI_u8TransfereByte((u8)u8Threshold);
    GYHD_DEACTIVATE_SLAVE_SELECT();

    u8Threshold = (u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdY;
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)INT1_TSH_YL_ADD);
    SPI_u8TransfereByte((u8)u8Threshold);
    GYHD_DEACTIVATE_SLAVE_SELECT();
    u8Threshold = (u8)((u16)(L3G4200D_ConfigParam.strIntCfg.u16ThresholdY>>8U));
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)INT1_TSH_YH_ADD);
    SPI_u8TransfereByte((u8)u8Threshold);
    GYHD_DEACTIVATE_SLAVE_SELECT();

    u8Threshold = (u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdZ;
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)INT1_TSH_ZL_ADD);
    SPI_u8TransfereByte((u8)u8Threshold);
    GYHD_DEACTIVATE_SLAVE_SELECT();
    u8Threshold = (u8)((u16)(L3G4200D_ConfigParam.strIntCfg.u16ThresholdZ>>8U));
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)INT1_TSH_ZH_ADD);
    SPI_u8TransfereByte((u8)u8Threshold);
    GYHD_DEACTIVATE_SLAVE_SELECT();

    Write Interrupt Duration
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)INT1_DURATION_ADD);
    SPI_u8TransfereByte(L3G4200D_ConfigParam.strIntCfg.u8IntDuration);
    GYHD_DEACTIVATE_SLAVE_SELECT();
    Write Interrupt Activation
    u8RegData__ = (u8)(L3G4200D_ConfigParam.strIntCfg.u8XAxisInterrupt | (u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdY |(u8) L3G4200D_ConfigParam.strIntCfg.u16ThresholdZ);
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte((u8)INT1_CFG_ADD);
    SPI_u8TransfereByte((u8)u8RegData__);
    GYHD_DEACTIVATE_SLAVE_SELECT();
}*/
#endif
