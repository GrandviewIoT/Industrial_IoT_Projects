/**
  ******************************************************************************
  * @file    compass.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    16-September-2015   
  * @brief   System information functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "k_config.h"
#include "k_module.h"
#include "k_menu.h"
#include "math.h"

/* Private function prototypes -----------------------------------------------*/
KMODULE_RETURN CompassInit(void);
KMODULE_RETURN CompassExec(void);
KMODULE_RETURN CompassDeInit(void);
static void Compass_Calib(void);
static void Compass_Run(void);

/* Private Define ------------------------------------------------------------*/
#define COMPASS_CALIBRATED 0xAABBCCDD
#define COMPASS_OFFSET     280
/* Private Variable ----------------------------------------------------------*/
CompassBackupData_TypeDef CompassBackup;

const tMenuItem CompassMenuItems[] =
{
    {"RUN"          , 0, 0,   SEL_EXEC, MODULE_NONE, Compass_Run, NULL, NULL, NULL},
    {"CALIB"        , 0, 0,   SEL_EXEC, MODULE_NONE, Compass_Calib, NULL, NULL, NULL},
    {""             , 0, 0,   SEL_EXIT, 0, NULL, NULL, NULL, NULL}
};

const tMenu CompassMenu = {
  "", CompassMenuItems, countof(CompassMenuItems), TYPE_TEXT, 2, 3
  };

int16_t MagBuffer[3];
int16_t AccBuffer[3];
int32_t XmMax = -10000;
int32_t XmMin = 10000;
int32_t YmMax = -10000;
int32_t YmMin = 10000;
int32_t ZmMax = -10000;
int32_t ZmMin = 10000;
float heading;
float Magx;
float Magy;
float Magz;
float AccXnorm;
float AccYnorm;
float Xh;
float Yh;
float Pitch;
float Roll;

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define max(a,b) (a>=b?a:b)
#define min(a,b) (a<=b?a:b)
/* External variables --------------------------------------------------------*/
const K_ModuleItem_Typedef ModuleCompass =
{
  MODULE_COMPASS,
  CompassInit,
  CompassExec,
  CompassDeInit,
  NULL
};  

/**
  * @brief  Initialize the Compass application 
  * @param  None.
  * @note   None.
  * @retval None.
  */
KMODULE_RETURN CompassInit(void)
{
  if( PowerSupplyMode == SUPPLY_MODE_BATTERY)
  { 
    SystemClock_BatterySupply_ClockIncrease();
  }

  /* Init Magneto Mems */
  if(BSP_COMPASS_Init() != COMPASS_OK)
  {
    /* Initialization Error */
    /* Error_Handler(); */
  }
  
  return KMODULE_OK;
}

/**
  * @brief  Run the Compass Measurement application 
  * @param  None.
  * @note   run and display Compass Menu.  
  * @retval None.
  */
KMODULE_RETURN CompassExec(void)
{
  kMenu_Execute(CompassMenu);
  return KMODULE_OK;
}

/**
  * @brief  DeInitialize the Compass application
  * @param  None.
  * @note   None.
  * @retval None.
  */
KMODULE_RETURN CompassDeInit(void)
{
  /* put device in low power mode */
  BSP_COMPASS_LowPower();

  /* Un-initialize the compass */
  BSP_COMPASS_DeInit();

  if( PowerSupplyMode == SUPPLY_MODE_BATTERY)
  { 
    SystemClock_BatterySupply_ClockDecrease();
  }
  
  return KMODULE_OK;
}

/**
  * @brief  Compass calibration mode 
  * @param  None.
  * @note   None.  
  * @retval None.
  */
void Compass_Calib(void)
{
  CompassBackupData_TypeDef backup;
  JOYState_TypeDef key = JOY_NONE;
  
  /* Display Info on LCD */
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_ScrollSentence("      ROTATE BOARD ON ALL AXIS", 1, SCROLL_SPEED_HIGH);
  BSP_LCD_GLASS_Clear();
  BSP_LCD_GLASS_DisplayString((uint8_t *)"STOP");
  
  /* Wait first measure */
  HAL_Delay(25);
  
  /* Loop to perform compass calibration */
  do {
    /* Get magnetometer values */
    BSP_COMPASS_MagGetXYZ(MagBuffer);
    /* Store maximum and minimum values */
    XmMax = max((int32_t) MagBuffer[0],XmMax);
    XmMin = min((int32_t) MagBuffer[0],XmMin);
    YmMax = max((int32_t) MagBuffer[1],YmMax);
    YmMin = min((int32_t) MagBuffer[1],YmMin);
    ZmMax = max((int32_t) MagBuffer[2],ZmMax);
    ZmMin = min((int32_t) MagBuffer[2],ZmMin);
    
    key = kMenu_GetEvent(10);
  } while((key != JOY_SEL) && (key != JOY_RIGHT) && (key != JOY_LEFT));
  
  /* Store calibration values */
  backup.xmax = XmMax;
  backup.xmin = XmMin;
  backup.ymax = YmMax;
  backup.ymin = YmMin;
  backup.zmax = ZmMax;
  backup.zmin = ZmMin;
  backup.calibration = COMPASS_CALIBRATED;
  
  SystemBackupWrite(BACKUP_COMPASS, (void *)&backup);
}

/**
  * @brief  Compass RUN mode 
  * @param  None.
  * @note   None.  
  * @retval None.
  */
void Compass_Run(void)
{
  /* Gyroscope variable */
  char  string_display[7];
  float Temp;
  
  /* read Compass backup data */
  SystemBackupRead(BACKUP_COMPASS, (void *)&CompassBackup);

  /* Check if calibration has been performed */
  if(COMPASS_CALIBRATED != CompassBackup.calibration)
  {
    /* Inform user that calibration has to be performed */
    BSP_LCD_GLASS_Clear();
    BSP_LCD_GLASS_ScrollSentence("      PLEASE PERFORM CALIBRATION", 1, SCROLL_SPEED_HIGH);    
  }
  else
  {
    /* Retrieve calibration values */
    XmMax = CompassBackup.xmax;
    XmMin = CompassBackup.xmin;
    YmMax = CompassBackup.ymax;
    YmMin = CompassBackup.ymin;
    ZmMax = CompassBackup.zmax;
    ZmMin = CompassBackup.zmin;

    /* Wait first measure */
    HAL_Delay(25);
    
    /* Loop to display compass information */
    do {
      /* Get acceleremoter and magnetometer values */
      BSP_COMPASS_AccGetXYZ(AccBuffer);
      BSP_COMPASS_MagGetXYZ(MagBuffer);
      
      /* use calibration values to shift and scale magnetometer measurements */
      Magx = ((float) MagBuffer[0]-XmMin)/(XmMax-XmMin)*2-1;
      Magy = ((float) MagBuffer[1]-YmMin)/(YmMax-YmMin)*2-1;
      Magz = ((float) MagBuffer[2]-ZmMin)/(ZmMax-ZmMin)*2-1;

      /* Normalize acceleration measurements so they range from 0 to 1 */
      Temp = AccBuffer[0]*AccBuffer[0]+AccBuffer[1]*AccBuffer[1]+AccBuffer[2]*AccBuffer[2];
      AccXnorm =  AccBuffer[0]/sqrt(Temp);
      AccYnorm =  AccBuffer[1]/sqrt(Temp);
      
      
      /* Calculate Pitch and Roll values in radian */
      Pitch = asin(-AccXnorm);
      Roll  = asin(AccYnorm/cos(Pitch));
      
      /* Calculate tilted position */
      Xh = Magx*cos(Pitch) + Magz*sin(Pitch);
      Yh = Magx*sin(Roll)*sin(Pitch) + Magy*cos(Roll) - Magz*sin(Roll)*cos(Pitch);
      
      heading = 180*atan2(Yh,Xh)/(3.14);
      if (heading <0)
        heading += 360;
      /* Revert angle and apply offset*/
      heading = 360 - heading - COMPASS_OFFSET;
      if (heading <0)
        heading += 360;
      
      sprintf(string_display," %3d",(int16_t) heading);
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t*)string_display);
      
    } while(kMenu_GetEvent(400) != JOY_LEFT);  /* Min screen update is 400ms */
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

