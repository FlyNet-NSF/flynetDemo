#pragma once
/*///////////////////////////////////////////////////////////////////////*/
//  	Hadi AliAkbarpour
//
//  For more information, contact:
//      Dr. Hadi AliAkbarpour, hd.akbarpour@gmail.com, aliakbarpourh@missouri.edu ,
//		Dr. Filiz Bunyak,
//      Prof. K. Palaniappan palaniappank@missouri.edu,
//      329 Engineering Building West
//      University of Missouri-Columbia
//      Columbia, MO 65211
//
//	Restructured into breakout file by Steve Suddarth, 10 Aug 2016
//
/*///////////////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////////////////////////*/


#include <cv.hpp>
#include <highgui.h>
#include <cuda_runtime.h>
#include <npp.h>
#include "npp/ImagesNPP.h"
#include "npp/Exceptions.h"
#include <helper_functions.h>
#include <helper_string.h>
#include <helper_cuda.h>

#include "flux_util.h"
#include "FluxCudaKernels.h"

#if 1
struct BG_model_pars {
    int learning_start_frame = 30 ; // frame to start learning the BG
    int learning_end_frame  = 150 ; //  frame to stop learning background
    int detection_start_frame  = 151 ; // frame to start considering as foreground
    int detection_time_buffer_size_ = 30 ; // number of frames between tail and head in detection
};
#else
struct BG_model_pars {
    int learning_start_frame = 100 ; // frame to start learning the BG
    int learning_end_frame  = 300 ; //  frame to stop learning background
    int detection_start_frame  = 301 ; // frame to start considering as foreground
    int detection_time_buffer_size_ = 100 ; // number of frames between tail and head in detection
};
#endif


void OtsuThresholding(int nHistBin , Npp32s *histHost , int histBinCount) ;
void computeHistRange_32f(Image32f *oDeviceSrc , Npp32s *histHost , int histBinCount, Npp32f *levelsHost) ;
void StretchHist_32f( Image32f *oDeviceSrc, Image32f *oDeviceDst ,
                      Npp32f min_percent , Npp32f max_percent, int histBinCount ) ;



////      ts: [2.3000e-04 0.0094 0.0774 0.2405 0.3448 0.2405 0.0774 0.0094 2.3000e-04]
////     td1: [0.0012 0.0257 0.1214 0.1753 0 -0.1753 -0.1214 -0.0257 -0.0012]
////     td2: [0.0050 0.0563 0.1170 -0.0554 -0.2459 -0.0554 0.1170 0.0563 0.0050]
////      xs: [2.3000e-04 0.0094 0.0774 0.2405 0.3448 0.2405 0.0774 0.0094 2.3000e-04]
////     xd1: [0.0012 0.0257 0.1214 0.1753 0 -0.1753 -0.1214 -0.0257 -0.0012]
////    xavg: [1 1 1 1 1 1 1 1 1]
////    tavg: [1 1 1 1 1 1 1 1 1]
///*
// * Filters for flux/structure tensor spatial and temporal derivative and averaging and smoothing
// *
// */
//struct filters {
//	Npp32f ts[9] = {0.0002, 0.0094, 0.0774, 0.2405, 0.3448, 0.2405, 0.0774, 0.0094, 0.0002} ;
//	NppiSize ts_size = {9, 1};
//	NppiSize ts_transpose_size = {1, 9};
//	NppiPoint ts_anchor={4,0} ;
//	NppiPoint ts_transpose_anchor={1,5} ;
//
//	Npp32f xs[9] = {2.3000e-04,  0.0094, 0.0774,  0.2405 , 0.3448 , 0.2405 , 0.0774 , 0.0094  , 2.3000e-04} ;
//	NppiSize xs_size = {9, 1};
//	NppiSize xs_transpose_size = {1, 9};
//	NppiPoint xs_anchor={4,0} ;
//	NppiPoint xs_transpose_anchor={1,5} ;
//
//	//Npp32f xd1[9]= {0.0012, 0.0257, 0.1214, 0.1753, 0, -0.1753, -0.1214, -0.0257, -0.0012} ;
//	Npp32f xd1[9]= {-0.0012, -0.0257, -0.1214, -0.1753, 0, 0.1753, 0.1214, +0.0257, +0.0012} ;
//	NppiSize xd1_size = {9, 1};
//	NppiSize xd1_transpose_size = {1, 9};
//	NppiPoint xd1_anchor={4,0} ;
//	NppiPoint xd1_transpose_anchor={0,4} ;
//
//	Npp32f td1[9]= {0.0012, 0.0257, 0.1214, 0.1753, 0, -0.1753, -0.1214, -0.0257, -0.0012} ;
//	NppiSize td1_size = {9, 1};
//	NppiSize td1_transpose_size = {1, 9};
//	NppiPoint td1_anchor={4,0} ;
//	NppiPoint td1_transpose_anchor={1,5} ;
//
//	Npp32f td2[9]= {0.0050 ,0.0563, 0.1170 ,-0.0554, -0.2459 ,-0.0554, 0.1170, 0.0563, 0.0050} ;
//	NppiSize td2_size = {9, 1};
//	NppiSize td2_transpose_size = {1, 9};
//	NppiPoint td2_anchor={4,0} ;
//	NppiPoint td2_transpose_anchor={1,5} ;
//
//	Npp32f xavg[9]= {1 , 1, 1, 1, 1, 1, 1, 1, 1} ;
//	NppiSize xavg_size = {9, 1};
//	NppiSize xavg_transpose_size = {1, 9};
//	NppiPoint xavg_anchor={4,0} ;
//	NppiPoint xavg_transpose_anchor={1,5} ;
//
//	Npp32f tavg[9]= {1 , 1, 1, 1, 1, 1, 1, 1, 1} ;
//	NppiSize tavg_size = {9, 1};
//	NppiSize tavg_transpose_size = {1, 9};
//	NppiPoint tavg_anchor={4,0} ;
//	NppiPoint tavg_transpose_anchor={1,5} ;
//};

extern fluxFilters fluxFilter;

/*
 * Computes derivatives in x and y directions combined with smoothing
 * oDeviceSrc: Pointer to input image
 * oDeviceDst_Ix, oDeviceDst_Iy and oDeviceDst_Is: outputs
 *
 * d: derivative filter
 * s: smoothing filter
 *
 *    t                   t                 t
 *   /                  /                  /
 *  d                 d                  d
 * /                 /                  /
 * -----d---->x     ----s---->x        ----s---->x
 * |                |                  |
 * s    Ixt         d   Iyt            s    Itt
 * |                |                  |
 * v                v                  v
 * y                y                  y
 *
 */
void computeIxIyIs(Image32f *oDeviceSrc, Image32f *oDeviceDst_Ix ,
                   Image32f *oDeviceDst_Iy ,
                   Image32f *oDeviceDst_Is ,
                   fluxFilters *filter1);


/*
 * A class which contains flux tensor operations and needed data
 */

class FluxTensor
{
    bool debug_ ;
    std::string debug_outpath_ ;
    bool run_detection_ ;
    int detection_channels_ ;

    ImgList32f Ix_List_ ;
    ImgList32f Iy_List_ ;
    ImgList32f Is_List_ ;
    ImgList32f trace_sumxy_List_ ;
    ImgList32f traceST_sumxy_List_ ;
    int *frames_ ;
    //	int num_elements_ ;
    //	int nextPos_ ;
    //	std::deque<Image32f *> Ix_ ;
    //	std::deque<Image32f *> Iy_ ;
    //	std::deque<Image32f *> Is_ ;
    //	std::deque<Image32f *> trace_sumxy_ ;
    //	std::deque<Image32f *> traceST_sumxy_ ;

    Image32f * flux_trace_ ;
    Image32f * flux_trace_norm_ ;
    Image32f * st_trace_ ;
    Image32f * st_trace_norm_ ;
    Image32f * static_ ;
    Image32f * flux_trace_mask_ ;
    Image32f * capture_buffer_grey_ ;
    Image32f * capture_buffer_color_ ;

    Image32f *Mxt ;
    Image32f *Myt ;
    Image32f *Mtt ;
    Image32f *Mt ;
    Image32f *Mtrace ;
    Image32f *MtraceST ;

    Image32f * util_4temporalDerivativeTrace ;
    Image32f * util_4Tensor2Static_ ;

    //std::deque<int> frames_ ;
    int capacity_  , img_width_ , img_height_;
    int frame_counter_ ;
    bool initialized_ ;
    bool first_detection_entry_ ;
    int nt_deriv_ , half_nt_deriv_ , nt_avg_ , half_nt_avg_ , start_frame_ ;
    fluxFilters* filter_ ;

    //---------------------------------------------------------
    // -------------- detection params ------------------------
    //---------------------------------------------------------
    //#define SINGLE_CHANEL_BG_MODEL
    //#ifdef SINGLE_CHANEL_BG_MODEL
    //#define    IMG  Image32f
    //#define    IMGL ImgList32f
    //#else
    //#define    IMG Image32f
    //#define    IMGL ImgList32f
    //#endif
    int start_frame1_, end_frame1_, start_frame2_ ,end_frame2_, detection_time_buffer_size_ ;
    Image32f * BG_ ; // background, will keep sum of ftraces
    Image32f * BG80percent_ ; //
    Image32f * BGmask_ ;

    Image32f * I_ ; // original grey images, will keep sum of them
    ImgList32f lstD_ ; // to store grey difference or change
    ImgList32f lstEi_ ; // to store Ei (fresh statics) history
    ImgList32f lstIm_ ; // to store greyscale I (input image) history

    //Image32f * util_4detection_delta_ ;
    Image32f * E_sum_head_ ;
    Image32f * E_sum_tail_ ;
    Image32f * I_sum_head_ ;
    Image32f * I_sum_tail_ ;
    Image32f * I_mask_ ;
    Image32f * E_mask_ ;
    Image32f * E_pm_ ; // persistent
    Image32f * I_pm_ ; // persistent
    Image32f * E_ ; // foreground, will keep sum of statics
    Image32f * E2d_ ;
    Image32f * D_ ;
    Image32f * D2_ ;
    int E_count_head_ ;
    int E_count_tail_ ;
    int I_count_head_ ;
    int I_count_tail_ ;

    //-------------------

protected:
    FluxHelper *helper_ ;

    //std::string output_path_ ;
    static const int histBinCount_=255;
    static const int fcoef_=5 ;

    bool savedState_ ;

    bool temporalDerivativeTrace(int) ;
    bool temporalDerivativeTrace_V2(int) ;
    int seekFrameID(int) ;
    int getIxIyIs(Image32f ** , Image32f ** , Image32f **,int) ;
    void insert(int) ;
    inline int getBufferCapacity() {return capacity_ ;}
    inline FluxHelper * getHelper() {return helper_;}
    //void TemporalAvg( Image32f * flux_trace_norm , Image32f * mask , Image32f * st_trace , int fr) ;
    void TemporalAvg(int fr) ;
    void TemporalAvg_V2(int fr) ;
    void Tensor2Static() ;
    void NormalizeTrace(Image32f *oDeviceSrc , Image32f *oDeviceDst , Npp32f nx, Npp32f nt ) ;

public:
    FluxTensor() ;
    ~FluxTensor() ;
    void Init(int img_width, int img_height, int capacity, fluxFilters* fluxFilter, BG_model_pars bg_pars, bool run_detection=true, bool colorDetection=false, bool debug=false, std::string debug_outpath="" ) ;
    NppiSize getImgSize() ;

    //	inline int getNumOfElements() {return num_elements_ ;}
    //inline int getNextAvailablePos() {return nextPos_ ;}
    void addFrame(Image32f *, Image32f *imgColor=NULL) ;
    void addFrame_Mat(cv::Mat  imgGrey) ;
    void addFrame_Mat(cv::Mat  imgGrey,cv::Mat imgColor) ;

    void Reset() ;
    void storeState(std::string) ;
    void restoreState(std::string) ;

    void setTraceSumXY( Image32f *   , int) ;
    Image32f *  getTraceSumXY(int) ;

    void setTraceSTSumXY( Image32f *   , int) ;
    Image32f * getTraceSTSumXY(int) ;
    Image32f * getFluxTrace() ;

    Image32f * getFluxTraceNorm() ;
    cv::Mat getFluxTraceNorm_Mat() ;

    Image32f * getSTTrace() ;

    Image32f * getSTTraceNorm() ;
    cv::Mat getSTTraceNorm_Mat() ;

    inline Image32f  * getFluxTraceMask() {return flux_trace_mask_ ;}
    cv::Mat getFluxTraceMask_Mat() ;

    float getFluxTraceAvg() ; //returns avergae of flux traces
    float getStaticTraceAvg() ; //returns avergae of static traces

    inline Image32f  * getStatic() { return static_ ; }
    cv::Mat getStatic_Mat() ;

    cv::Mat getStaticModel_Mat() ;

    //    inline Image32f  * getChange() { int loc = lstD_.getLastFilledPos(); return lstD_.getImage(loc);}
    //    cv::Mat getChange_Mat() ;

    inline Image32f  * getPersistent() { return E_pm_ ; }
    cv::Mat getPersistent_Mat() ;

    // persistency computed from color version
    inline Image32f  * getPersistentColor() { return I_pm_ ; }
    cv::Mat getPersistentColor_Mat() ;

    inline Image32f  * getPersistentColorMask() { return I_mask_ ; }
    cv::Mat getPersistentColorMask_Mat() ;


    inline Image32f  * getPersistentMask() { return E_mask_ ; }
    cv::Mat getPersistentMask_Mat() ;

    int writeResults(std::string orgFilePath) ;
    //void computeDifference(Image32f * sum_head, int * count, Image32f * Ei, Image32f * E, Image32f * BG, int type) ;
};

