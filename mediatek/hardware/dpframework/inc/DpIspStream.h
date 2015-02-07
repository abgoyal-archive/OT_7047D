#ifndef __DP_ISP_STREAM_H__
#define __DP_ISP_STREAM_H__

#include "DpDataType.h"
#include "tpipe_config.h"

#define ISP_MAX_OUTPUT_PORT_NUM     3

class DpStream;
class DpChannel;
class DpBasicBufferPool;

class DpIspStream
{
public:
    enum ISPStreamType
    {
        ISP_IC_STREAM,
        ISP_VR_STREAM,
        ISP_ZSD_STREAM,
        ISP_IP_STREAM,
        ISP_VSS_STREAM
    };

    DpIspStream(ISPStreamType type);

    ~DpIspStream();
   
    DP_STATUS_ENUM queueSrcBuffer(void    *pVA,
                                 uint32_t size);
   
    DP_STATUS_ENUM queueSrcBuffer(void    *pVA,
                                 uint32_t MVA,
                                 uint32_t size);

    DP_STATUS_ENUM queueSrcBuffer(void     **pVAList,
                                  uint32_t *pSizeList,
                                  int32_t  planeNum);

    DP_STATUS_ENUM queueSrcBuffer(void     **pVAList,
                                  uint32_t *pMVAList,
                                  uint32_t *pSizeList,
                                  int32_t  planeNum);

    DP_STATUS_ENUM dequeueSrcBuffer();

    /**
     * Description:
     *     Set source buffer configuration information
     *
     * Parameter:
     *     srcFormat: Source buffer format
     *     srcWidth: Source buffer width
     *     srcHeight: Source buffer height
     *     srcPitch: Source buffer pitch
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM setSrcConfig(DpColorFormat srcFormat,
                                int32_t       srcWidth,
                                int32_t       srcHeight,
                                int32_t       srcPitch);


    DP_STATUS_ENUM setSrcConfig(int32_t           width,
                                int32_t           height,
                                int32_t           YPitch,
                                int32_t           UVPitch,
                                DpColorFormat     format,
                                DP_PROFILE_ENUM   profile = DP_PROFILE_BT601,
                                DpInterlaceFormat field   = eInterlace_None,
                                DpRect            *pROI   = 0);

    /**
     * Description:
     *     Set source buffer crop window information
     *
     * Parameter:
     *     XStart: Source crop X start coordinate
     *     XSubpixel: Source crop X subpixel coordinate
     *     YStart: Source crop Y start coordinate
     *     YSubpixel: Source crop Y subpixel coordinate
     *     cropWidth: Source crop window width
     *     cropHeight: Source crop window height
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM setSrcCrop(int32_t XStart,
                              int32_t XSubpixel,
                              int32_t YStart,
                              int32_t YSubpixel,
                              int32_t cropWidth,
                              int32_t cropHeight);


    DP_STATUS_ENUM queueDstBuffer(int32_t  portIndex,
                                  void     **pVAList,
                                  uint32_t *pSizeList,
                                  int32_t  planeNum);

    DP_STATUS_ENUM queueDstBuffer(int32_t  portIndex,
                                  void     **pVAList,
                                  uint32_t *pMVAList,
                                  uint32_t *pSizeList,
                                  int32_t  planeNum);

    /**
     * Description:
     *     Acquire a destination buffer for HW processing
     *
     * Parameter:
     *     port: Output port index
     *     base: buffer virtual base address
     *     waitBuf: true for the buffer is ready;
     *              else return immediately
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM dequeueDstBuffer(int32_t  portIndex,
                                    void     **pVABase,
                                    bool     waitBuf = true);

    /**
     * Description:
     *     Set destination buffer configuration information
     *
     * Parameter:
     *     format: Destination buffer format
     *     width: Destination buffer width
     *     height: Destination buffer height
     *     pitch: Destination buffer pitch
     *     port: Destination port number
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM setDstConfig(int32_t       portIndex,
                                DpColorFormat dstFormat,
                                int32_t       dstWidth,
                                int32_t       dstHeight,
                                int32_t       dstPitch);


    DP_STATUS_ENUM setDstConfig(int32_t           portIndex,
                                int32_t           width,
                                int32_t           height,
                                int32_t           YPitch,
                                int32_t           UVPitch,
                                DpColorFormat     format,
                                DP_PROFILE_ENUM   profile = DP_PROFILE_BT601,
                                DpInterlaceFormat field   = eInterlace_None,
                                DpRect            *pROI   = 0);


    /**
     * Description:
     *     Set port desired rotation angles
     *
     * Parameter:
     *     portIndex: Port index number
     *     rotation: Desired rotation angle
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM setRotation(int32_t portIndex,
                               int32_t rotation);

    /**
     * Description:
     *     Set port desired flip status
     *
     * Parameter:
     *    portIndex: Port index number
     *    flipStatus: Desired flip status
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM setFlipStatus(int32_t  portIndex,
                                 bool     flipStatus);


    /**
     * Description:
     *     Set extra parameter for ISP
     *
     * Parameter:
     *     extraPara: ISP extra parameters
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM setParameter(ISP_TPIPE_CONFIG_STRUCT &extraPara);


    DP_STATUS_ENUM setSharpness(int32_t portIndex,
                                int32_t gain) 
    { 
        m_sharpness[portIndex] = gain; 
        return DP_STATUS_RETURN_SUCCESS;
    }


    DP_STATUS_ENUM setDither(int32_t portIndex,
                             bool    enDither) 
    {
        m_ditherStatus[portIndex] = enDither;
        return DP_STATUS_RETURN_SUCCESS;
    }

    /**
     * Description:
     *     Start ISP stream processing (non-blocking)
     *
     * Parameter:
     *     None
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM startStream();

    /**
     * Description:
     *     Stop ISP stream processing
     *
     * Parameter:
     *     None
     *
     * Return Value:
     *     Return DP_STATUS_RETURN_SUCCESS if the API succeeded,
     *     else the API will return the error code.
     */
    DP_STATUS_ENUM stopStream();

private:
    ISPStreamType           m_streamType;
    int32_t                 m_currentMode;
    bool                    m_frameChange;
    DpStream                *m_pStream;
    DpChannel               *m_pChannel;
    int32_t                 m_channelID;

    // Source information
    DpBasicBufferPool       *m_pSrcPool;
    int32_t                 m_srcBuffer;
    DpColorFormat           m_srcFormat;
    int32_t                 m_srcWidth;
    int32_t                 m_srcHeight;
    int32_t                 m_srcYPitch;
    int32_t                 m_srcUVPitch;
    bool                    m_cropChange;

    // Destination information
    DpBasicBufferPool       *m_pDstPool[ISP_MAX_OUTPUT_PORT_NUM];
    int32_t                 m_dstBuffer[ISP_MAX_OUTPUT_PORT_NUM];
    DpColorFormat           m_dstFormat[ISP_MAX_OUTPUT_PORT_NUM];
    int32_t                 m_dstPlane[ISP_MAX_OUTPUT_PORT_NUM];
    int32_t                 m_dstWidth[ISP_MAX_OUTPUT_PORT_NUM];
    int32_t                 m_dstHeight[ISP_MAX_OUTPUT_PORT_NUM];
    int32_t                 m_dstYPitch[ISP_MAX_OUTPUT_PORT_NUM];
    int32_t                 m_dstUVPitch[ISP_MAX_OUTPUT_PORT_NUM];
    int32_t                 m_rotation[ISP_MAX_OUTPUT_PORT_NUM];
    bool                    m_flipStatus[ISP_MAX_OUTPUT_PORT_NUM];
    bool                    m_dstEnable[ISP_MAX_OUTPUT_PORT_NUM];
    int32_t                 m_sharpness[ISP_MAX_OUTPUT_PORT_NUM];
    bool                    m_ditherStatus[ISP_MAX_OUTPUT_PORT_NUM];

    // Crop information
    int32_t                 m_srcXStart;
    int32_t                 m_srcXSubpixel;
    int32_t                 m_srcYStart;
    int32_t                 m_srcYSubpixel;
    int32_t                 m_cropWidth;
    int32_t                 m_cropHeight;

    bool                    m_parameterSet;
    ISP_TPIPE_CONFIG_STRUCT *m_pParameter;
   
};

#endif  // __DP_ISP_STREAM_H__
