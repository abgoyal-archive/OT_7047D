
#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>
/*zuoyong*/
#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_s5k3h7ymipiraw.h"
#include "camera_info_s5k3h7ymipiraw.h"
#include "camera_custom_AEPlinetable.h"
#include "camera_custom_tsf_tbl.h"


const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,

    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    	}
    },
    ISPPca: {
        #include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
        #include INCLUDE_FILENAME_ISP_REGS_PARAM
    },
    ISPMfbMixer:{{
        {//00: MFB mixer for ISO 100
            0x00000000, 0x00000000
        },
        {//01: MFB mixer for ISO 200
            0x00000000, 0x00000000
        },
        {//02: MFB mixer for ISO 400
            0x00000000, 0x00000000
        },
        {//03: MFB mixer for ISO 800
            0x00000000, 0x00000000
        },
        {//04: MFB mixer for ISO 1600
            0x00000000, 0x00000000
        },
        {//05: MFB mixer for ISO 2400
            0x00000000, 0x00000000
        },
        {//06: MFB mixer for ISO 3200
            0x00000000, 0x00000000
        }
    }},
    ISPCcmPoly22:{
        68400,    // i4R_AVG
        14607,    // i4R_STD
        106675,    // i4B_AVG
        29955,    // i4B_STD
        {  // i4P00[9]
            5275000, -2135000, -580000, -867500, 3472500, -47500, 45000, -1882500, 4395000
        },
        {  // i4P10[9]
            1010729, -936795, -73934, -114186, 172658, -65453, 25214, 204166, -236360
        },
        {  // i4P01[9]
            606288, -488385, -117903, -249985, 130852, 112137, -23264, -338790, 355058
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    }
}};

const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1136,    // u4MinGain, 1024 base = 1x
            10240,    // u4MaxGain, 16x
            86,    // u4MiniISOGain, ISOxx  
            256,    // u4GainStepUnit, 1x/8 
            13172,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            13172,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            13172,    // u4CapExpUnit 
            30,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            22,    // u4LensFno, Fno = 2.8
            350    // u4FocusLength_100x
        },
        // rHistConfig
        {
            4,    // u4HistHighThres
            30,    // u4HistLowThres
            2,    // u4MostBrightRatio
            1,    // u4MostDarkRatio
            160,    // u4CentralHighBound
            20,    // u4CentralLowBound
            {240, 230, 220, 210, 200},    // u4OverExpThres[AE_CCT_STRENGTH_NUM] 
            {62, 70, 88, 108, 141},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
            {18, 22, 30, 30, 40}    // u4BlackLightThres[AE_CCT_STRENGTH_NUM] 
        },
        // rCCTConfig
        {
            TRUE,    // bEnableBlackLight
            TRUE,    // bEnableHistStretch
            FALSE,    // bEnableAntiOverExposure
            TRUE,    // bEnableTimeLPF
            FALSE,    // bEnableCaptureThres
            FALSE,    // bEnableVideoThres
            FALSE,    // bEnableStrobeThres
            47,    // u4AETarget
            0,    // u4StrobeAETarget
            50,    // u4InitIndex
            8,    // u4BackLightWeight
            12,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            4,    // u4BlackLightStrengthIndex
            2,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            6,    // i4BVOffset delta BV = value/10 
            96,    // u4PreviewFlareOffset
            96,    // u4CaptureFlareOffset
            3,    // u4CaptureFlareThres
            96,    // u4VideoFlareOffset
            3,    // u4VideoFlareThres
            64,    // u4StrobeFlareOffset
            3,    // u4StrobeFlareThres
            160,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            160,    // u4VideoMaxFlareThres
            0,    // u4VideoMinFlareThres
            18,    // u4FlatnessThres    // 10 base for flatness condition.
            75    // u4FlatnessStrength
        }
    },

    // AWB NVRAM
    {
	// AWB calibration data
	{
		// rCalGain (calibration gain: 1.0 = 512)
		{
			0,	// u4R
			0,	// u4G
			0	// u4B
		},
		// rDefGain (Default calibration gain: 1.0 = 512)
		{
			0,	// u4R
			0,	// u4G
			0	// u4B
		},
		// rDefGain (Default calibration gain: 1.0 = 512)
		{
			0,	// u4R
			0,	// u4G
			0	// u4B
            },
            // rD65Gain (D65 WB gain: 1.0 = 512)
            {
                886,    // i4R
                512,    // i4G
                638    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                18,    // i4X
                -423    // i4Y
            },
            // Horizon
            {
                -451,    // i4X
                -382    // i4Y
            },
            // A
            {
                -325,    // i4X
                -374    // i4Y
            },
            // TL84
            {
                -193,    // i4X
                -349    // i4Y
            },
            // CWF
            {
                -164,    // i4X
                -509    // i4Y
            },
            // DNP
            {
                121,    // i4X
                -284    // i4Y
            },
            // D65
            {
                121,    // i4X
                -284    // i4Y
            },
		// DF
		{
			0, 	// i4X
			0	// i4Y
		}
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                -63,    // i4X
                -418    // i4Y
            },
            // Horizon
            {
                -515,    // i4X
                -288    // i4Y
            },
            // A
            {
                -390,    // i4X
                -304    // i4Y
            },
            // TL84
            {
                -256,    // i4X
                -305    // i4Y
            },
            // CWF
            {
                -258,    // i4X
                -468    // i4Y
            },
            // DNP
            {
                64,    // i4X
                -302    // i4Y
            },
            // D65
            {
                64,    // i4X
                -302    // i4Y
            },
            // DF
            {
                0,    // i4X
			0	// i4Y
		}
	},
	// AWB gain of AWB light source
        {
            // Strobe 
            {
                930,    // i4R
                512,    // i4G
                886    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                563,    // i4G
                1738    // i4B
            },
            // A 
            {
                548,    // i4R
                512,    // i4G
                1319    // i4B
            },
            // TL84 
            {
                633,    // i4R
                512,    // i4G
                1067    // i4B
            },
            // CWF 
            {
                817,    // i4R
                512,    // i4G
                1273    // i4B
            },
            // DNP 
            {
                886,    // i4R
                512,    // i4G
                638    // i4B
            },
            // D65 
            {
                886,    // i4R
                512,    // i4G
                638    // i4B
            },
            // DF 
            {
                512,    // i4R
                512,    // i4G
                512    // i4B
            }
        },
        // Rotation matrix parameter
        {
            11,    // i4RotationAngle
            251,    // i4Cos
            49    // i4Sin
        },
        // Daylight locus parameter
        {
            -187,    // i4SlopeNumerator
            128    // i4SlopeDenominator
        },
	// AWB light area
	{
		// Strobe
            {
            100,    // i4RightBound
            -150,    // i4LeftBound
            -222,    // i4UpperBound
            -382    // i4LowerBound
            },
            // Tungsten
            {
            -306,    // i4RightBound
            -956,    // i4LeftBound
            -246,    // i4UpperBound
            -346    // i4LowerBound
            },
            // Warm fluorescent
            {
            -306,    // i4RightBound
            -956,    // i4LeftBound
            -346,    // i4UpperBound
            -466    // i4LowerBound
            },
            // Fluorescent
            {
            -150,    // i4RightBound
            -306,    // i4LeftBound
            -234,    // i4UpperBound
            -386    // i4LowerBound
            },
            // CWF
            {
            -150,    // i4RightBound
            -306,    // i4LeftBound
            -386,    // i4UpperBound
            -518    // i4LowerBound
            },
            // Daylight
            {
            100,    // i4RightBound
            -150,    // i4LeftBound
            -222,    // i4UpperBound
            -382    // i4LowerBound
            },
            // Shade
            {
            449,    // i4RightBound
            100,    // i4LeftBound
            -222,    // i4UpperBound
            -382    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            110,    // i4RightBound
            -150,    // i4LeftBound
            -382,    // i4UpperBound
            -450    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            449,    // i4RightBound
            -956,    // i4LeftBound
            -222,    // i4UpperBound
            -518    // i4LowerBound
            },
            // Daylight
            {
            125,    // i4RightBound
            -150,    // i4LeftBound
            -222,    // i4UpperBound
            -382    // i4LowerBound
            },
            // Cloudy daylight
            {
            225,    // i4RightBound
            50,    // i4LeftBound
            -222,    // i4UpperBound
            -382    // i4LowerBound
            },
            // Shade
            {
            325,    // i4RightBound
            50,    // i4LeftBound
            -222,    // i4UpperBound
            -382    // i4LowerBound
            },
            // Twilight
            {
            -150,    // i4RightBound
            -310,    // i4LeftBound
            -222,    // i4UpperBound
            -382    // i4LowerBound
            },
            // Fluorescent
            {
            114,    // i4RightBound
            -358,    // i4LeftBound
            -252,    // i4UpperBound
            -518    // i4LowerBound
            },
            // Warm fluorescent
            {
            -290,    // i4RightBound
            -490,    // i4LeftBound
            -252,    // i4UpperBound
            -518    // i4LowerBound
            },
            // Incandescent
            {
            -290,    // i4RightBound
            -490,    // i4LeftBound
            -222,    // i4UpperBound
            -382    // i4LowerBound
            },
            // Gray World
            {
            5000,    // i4RightBound
            -5000,    // i4LeftBound
            5000,    // i4UpperBound
            -5000    // i4LowerBound
            }
        },
        // PWB default gain	
        {
            // Daylight
            {
            817,    // i4R
            512,    // i4G
            722    // i4B
            },
            // Cloudy daylight
            {
            959,    // i4R
            512,    // i4G
            569    // i4B
            },
            // Shade
            {
            1011,    // i4R
            512,    // i4G
            525    // i4B
            },
            // Twilight
            {
            647,    // i4R
            512,    // i4G
            1020    // i4B
            },
            // Fluorescent
            {
            829,    // i4R
            512,    // i4G
            939    // i4B
            },
            // Warm fluorescent
            {
            622,    // i4R
            512,    // i4G
            1437    // i4B
            },
            // Incandescent
            {
            545,    // i4R
            512,    // i4G
            1315    // i4B
            },
            // Gray World
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        // AWB preference color	
        {
            // Tungsten
            {
            0,    // i4SliderValue
            7068    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            6128    // i4OffsetThr
            },
            // Shade
            {
            50,    // i4SliderValue
            499    // i4OffsetThr
            },
            // Daylight WB gain
            {
            886,    // i4R
            512,    // i4G
            639    // i4B
            },
            // Preference gain: strobe
            {
            480, // i4R512, // i4R//yange test
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            480,    // i4R
            512,    // i4G
            520    // i4B
            },
            // Preference gain: warm fluorescent
            {
            470,    // i4R
            512,    // i4G
            530    // i4B
            },
            // Preference gain: fluorescent
            {
            504,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: CWF
            {
            502,    // i4R
            506,    // i4G
            512    // i4B
            },
            // Preference gain: daylight
            {
            504,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: shade
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        {// CCT estimation
            {// CCT
                2300,    // i4CCT[0]
                2850,    // i4CCT[1]
                4100,    // i4CCT[2]
                5100,    // i4CCT[3]
                6500    // i4CCT[4]
            },
            {// Rotated X coordinate
                -579,    // i4RotatedXCoordinate[0]
                -454,    // i4RotatedXCoordinate[1]
                -320,    // i4RotatedXCoordinate[2]
                0,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }
    },
    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace

const CAMERA_TSF_TBL_STRUCT CAMERA_TSF_DEFAULT_VALUE =
{
    #include INCLUDE_FILENAME_TSF_PARA
    #include INCLUDE_FILENAME_TSF_DATA
};


typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T),
                                             0,
                                             sizeof(CAMERA_TSF_TBL_STRUCT)};

    if (CameraDataType > CAMERA_DATA_TSF_TABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        case CAMERA_DATA_TSF_TABLE:
            memcpy(pDataBuf,&CAMERA_TSF_DEFAULT_VALUE,sizeof(CAMERA_TSF_TBL_STRUCT));
            break;
        default:
            break;
    }
    return 0;
}};  //  NSFeature


