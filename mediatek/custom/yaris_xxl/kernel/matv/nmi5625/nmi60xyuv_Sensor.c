

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "nmi60xyuv_Sensor.h"


#define NMI60XYUV_DEBUG
#ifdef NMI60XYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif


static kal_uint32 NMI60X_GetSensorID(kal_uint32 *sensorID)

{
	SENSORDB("NMI60XYUV NMI60X_GetSensorID");

	*sensorID = 0x601b0;//NMI60X_SENSOR_ID;
	 return ERROR_NONE;
 
}   /* NMI60XOpen  */

static void NMI60X_Sensor_Init(void)
{

	SENSORDB("NMI60XYUV NMI60X_Sensor_Init");
}
  

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
UINT32 NMI60XOpen(void)
{

	SENSORDB("NMI60XYUV NMI60XOpen\r\n");
	return ERROR_NONE;
}	/* NMI60XOpen() */

UINT32 NMI60XClose(void)
{
//	CISModulePowerOn(FALSE);
	SENSORDB("NMI60XYUV NMI60XClose\r\n");
	return ERROR_NONE;
}	/* NMI60XClose() */

UINT32 NMI60XPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

		SENSORDB("NMI60XYUV NMI60XPreview\n");
		
		image_window->GrabStartX= IMAGE_SENSOR_START_GRAB_X;
		image_window->GrabStartY= IMAGE_SENSOR_START_GRAB_Y;
		image_window->ExposureWindowWidth = NMI60X_IMAGE_SENSOR_FULL_WIDTH; //IMAGE_SENSOR_FULL_WIDTH; 
		image_window->ExposureWindowHeight =NMI60X_IMAGE_SENSOR_FULL_HEIGHT; //IMAGE_SENSOR_FULL_HEIGHT;

		return ERROR_NONE;
}	/* NMI60XPreview() */

UINT32 NMI60XCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    	SENSORDB("NMI60XYUV NMI60XCapture\n");		
		return ERROR_NONE;
}	/* NMI60XCapture() */




UINT32 NMI60XGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{


		pSensorResolution->SensorPreviewWidth=NMI60X_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
		pSensorResolution->SensorPreviewHeight=NMI60X_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	

		pSensorResolution->SensorFullWidth=NMI60X_IMAGE_SENSOR_FULL_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
		pSensorResolution->SensorFullHeight=NMI60X_IMAGE_SENSOR_FULL_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	
		return ERROR_NONE;
}	/* NMI60XGetResolution() */

UINT32 NMI60XGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

		pSensorInfo->SensorPreviewResolutionX=NMI60X_IMAGE_SENSOR_PV_WIDTH;
		pSensorInfo->SensorPreviewResolutionY=NMI60X_IMAGE_SENSOR_PV_HEIGHT;

		pSensorInfo->SensorFullResolutionX=NMI60X_IMAGE_SENSOR_FULL_WIDTH;
		pSensorInfo->SensorFullResolutionY=NMI60X_IMAGE_SENSOR_FULL_HEIGHT;

		pSensorInfo->SensorCameraPreviewFrameRate=30;
    	pSensorInfo->SensorVideoFrameRate=30;
    	pSensorInfo->SensorStillCaptureFrameRate=10;
    	pSensorInfo->SensorWebCamCaptureFrameRate=15;
    	pSensorInfo->SensorResetActiveHigh=FALSE;
    	pSensorInfo->SensorResetDelayCount=1;
    	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
    	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    	pSensorInfo->SensorInterruptDelayLines = 1;
    	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

	    /*pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=FALSE;

	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
	    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=FALSE;*/
	    pSensorInfo->CaptureDelayFrame = 1;
	    pSensorInfo->PreviewDelayFrame = 0;
	    pSensorInfo->VideoDelayFrame = 4;
	    pSensorInfo->SensorMasterClockSwitch = 0;
	    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA;

	    switch (ScenarioId)
	    {
	    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
	        pSensorInfo->SensorClockFreq=24;
	        pSensorInfo->SensorClockDividCount=	3;
	        pSensorInfo->SensorClockRisingCount= 0;
	        pSensorInfo->SensorClockFallingCount= 2;
	        pSensorInfo->SensorPixelClockCount= 3;
	        pSensorInfo->SensorDataLatchCount= 2;
	        pSensorInfo->SensorGrabStartX = 0;
	        pSensorInfo->SensorGrabStartY = 1;

	        break;
	    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
	        pSensorInfo->SensorClockFreq=24;
	        pSensorInfo->SensorClockDividCount= 3;
	        pSensorInfo->SensorClockRisingCount=0;
	        pSensorInfo->SensorClockFallingCount=2;
	        pSensorInfo->SensorPixelClockCount=3;
	        pSensorInfo->SensorDataLatchCount=2;
	        pSensorInfo->SensorGrabStartX = 0;
	        pSensorInfo->SensorGrabStartY = 1;
	        break;
	    default:
	        pSensorInfo->SensorClockFreq=24;
	        pSensorInfo->SensorClockDividCount= 3;
	        pSensorInfo->SensorClockRisingCount=0;
	        pSensorInfo->SensorClockFallingCount=2;
	        pSensorInfo->SensorPixelClockCount=3;
	        pSensorInfo->SensorDataLatchCount=2;
	        pSensorInfo->SensorGrabStartX = 0;
	        pSensorInfo->SensorGrabStartY = 1;
	        break;
	    }
    return ERROR_NONE;
}	/* NMI60XGetInfo() */


UINT32 NMI60XControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			NMI60XPreview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			NMI60XCapture(pImageWindow, pSensorConfigData);
		break;
		default:
		    break; 
	}
	return TRUE;
}	/* NMI60XControl() */

UINT32 NMI60XFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=NMI60X_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=NMI60X_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = 24; //power.sheng
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		case SENSOR_FEATURE_SET_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER:
			
		case SENSOR_FEATURE_GET_CONFIG_PARA:
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
              *pFeatureReturnPara32++=0;
              *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			 NMI60X_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		     break; 
		default:
			break;			
	}
	return ERROR_NONE;
}	/* NMI60XFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncNMI60X=
{
	NMI60XOpen,
	NMI60XGetInfo,
	NMI60XGetResolution,
	NMI60XFeatureControl,
	NMI60XControl,
	NMI60XClose
};


UINT32 NMI60X_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncNMI60X;

	return ERROR_NONE;
}	/* SensorInit() */
