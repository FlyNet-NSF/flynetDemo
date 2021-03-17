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
//
/*///////////////////////////////////////////////////////////////////////////*/

/*
 * flux_util.h
 *
 *  Created on: Aug 5, 2016
 *      Author: ubuntu
 */

#ifndef FLUX_UTIL_H_
#define FLUX_UTIL_H_

#include <string.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unordered_map>
#ifdef _WIN32
#else
  #include <sys/time.h>
#endif
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <cv.hpp>
#include <highgui.h>
#include <cuda_runtime.h>
#include <npp.h>
#include "npp/ImagesNPP.h"
#include "npp/Exceptions.h"
//#include <helper_functions.h>
//#include <helper_string.h>
#include <helper_cuda.h>

#include "nvtx.h"

#define DP(pointer) pointer->data(),pointer->pitch()
#define DP2(object) object.data(),object.pitch()

#if 0
    #define NVTX_PUSHSync(name,cid)  cudaDeviceSynchronize();NVTX_PUSH(name,cid);
    #define NVTX_POPSync  cudaDeviceSynchronize();NVTX_POP
#else
    #define NVTX_PUSHSync(name,cid)
    #define NVTX_POPSync
#endif

//#define USE_UNIFIEDMEM


#define cudaMemErrChk(ans) { cudaMemAssert((ans) , __FILE__ , __LINE__) ; }
inline void cudaMemAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
    if(code != cudaSuccess)
    {
        //fprintf(stderr,"Fatal error: memory allocation failed!\n") ;
        fprintf(stderr,"cudaMemAssert: %s %s %d\n" , cudaGetErrorString(code) , file , line) ;
        if(abort)
            exit(code) ;
    }
}

// Macro to aligned up to the memory size in question
#define MEMORY_ALIGNMENT  4096
#define ALIGN_UP(x,size) ( ((size_t)x+(size-1))&(~(size-1)) )

#ifdef USE_UNIFIEDMEM
    #define cudaMemCpyType_D2D cudaMemcpyDefault
    #define cudaMemCpyType_D2H cudaMemcpyDefault
    #define cudaMemCpyType_H2D cudaMemcpyDefault
    #define cudaMallocV2 cudaMallocManaged
#else
    #define cudaMemCpyType_D2D cudaMemcpyDeviceToDevice
    #define cudaMemCpyType_D2H cudaMemcpyDeviceToHost
    #define cudaMemCpyType_H2D cudaMemcpyHostToDevice
    #define cudaMallocV2 cudaMalloc
#endif


template<typename T>
class ImageBase
{
protected:
    T* R_ ;
    T* R_UA_ ;
    T* G_ ;
    T* B_ ;
    int planes ; // number of channels in the image
    int width_ ;
    int height_ ;
    int pitch_ ;
    NppiSize ROI_ ;
public:
    ImageBase(unsigned int w, unsigned int h,int planes): width_(w) , height_(h) , planes(planes)
    {
        assert(planes==1 || planes==3) ;

#ifdef USE_UNIFIEDMEM
        bool badMemAlign = false ;
        cudaMemErrChk( cudaMallocManaged(&R_, width_ * height_ * sizeof(T)) )
        badMemAlign |= (uintptr_t) R_ % sizeof(T)!=0 ;
        if(planes==3) {
            cudaMemErrChk( cudaMallocManaged(&G_, width_ * height_ * sizeof(T)) )
            badMemAlign |= (uintptr_t) G_ % sizeof(T)!=0 ;

            cudaMemErrChk( cudaMallocManaged(&B_, width_ * height_ * sizeof(T)) )
            badMemAlign |= (uintptr_t) B_ % sizeof(T)!=0 ;
        }
        if(badMemAlign)
            fprintf(stderr,"warning: memory allocation is not aligned!\n") ;
        pitch_ = width_ *  sizeof(T) ;
#else
        if(0) {
            cudaMemErrChk( cudaMallocManaged(&R_UA_, width_ * height_ * sizeof(T) + MEMORY_ALIGNMENT) ) ;
            R_ = (T *) ALIGN_UP(R_UA_, MEMORY_ALIGNMENT);
            pitch_ =  width_ * sizeof(T) +   (size_t)R_-(size_t)R_UA_ ;
        }
        cudaMemErrChk ( cudaMallocPitch((void **)&R_ , (size_t *) &pitch_ , (size_t) width_ *  sizeof(T) ,  (size_t)height_ ) ) ;
        //printf("+++ Pitch is %d\n",pitch_) ;  //hadl
#endif
        ROI_= {(int)width_, (int)height_} ;
    }
     inline T* data() {return R_ ; }
     inline T* R() {return R_ ; }
     inline T* G() {return G_ ; }
     inline T* B() {return B_ ; }
     inline int height() {	return height_ ;	}
     inline int width() {	return width_ ;		}
     inline int pitch(){ return pitch_ ; }
     inline void copyTo(T *dst, unsigned int dstPitch) {
         cudaMemcpy2D(dst, dstPitch , this->R_ , pitch_ , width_* sizeof(T) , height_ , cudaMemCpyType_D2D ) ;
     }
     inline void copyTo(ImageBase *dst) {
         cudaMemcpy2D(dst->R_, dst->pitch() , this->R_ , pitch_ , width_* sizeof(T) , height_ , cudaMemCpyType_D2D ) ;
     }
     inline void copyFrom(T *src,int srcPitch) {
         cudaMemcpy2D( this->R_ , pitch_  , src , srcPitch , width_* sizeof(T) , height_ , cudaMemCpyType_D2D ) ;
     }
     inline void copyFrom(ImageBase *src) {
         cudaMemcpy2D( this->R_ , pitch_  , src->R_ , src->pitch() , width_* sizeof(T) , height_ , cudaMemCpyType_D2D ) ;
         if(planes==3) {
            cudaMemcpy2D( this->G_ , pitch_  , src->G_ ,  src->pitch()  , width_* sizeof(T) , height_ , cudaMemCpyType_D2D ) ;
            cudaMemcpy2D( this->B_ , pitch_  , src->B_ ,  src->pitch()  , width_* sizeof(T) , height_ , cudaMemCpyType_D2D ) ;
         }
     }
     inline NppiSize getROI() {return ROI_;}
     virtual void set(T value)=0 ;
     inline void Free()
     {
          nppiFree(R_);

          if(planes==3)
          {
               nppiFree(G_);
               nppiFree(B_);
          }
     }
};
// -----
class Image32f : public ImageBase<Npp32f> {
public:
    Image32f(int w, int h, int N_channels=1): ImageBase(w,h,N_channels) {} ;
    void set(Npp32f value) {
        nppiSet_32f_C1R( value , R_ , pitch_ , ROI_) ;
        if(planes==3)
        {
            nppiSet_32f_C1R( value , G_ , pitch_ , ROI_) ;
            nppiSet_32f_C1R( value , B_ , pitch_ , ROI_) ;
        }
    }

};
class Image8u : public ImageBase<Npp8u> {
public:
    Image8u(int w, int h, int N_channels=1): ImageBase(w,h,N_channels) {} ;
    void set(Npp8u value) {
        nppiSet_8u_C1R( value , R_ , pitch_ , ROI_) ;
        if(planes==3)
        {
            nppiSet_8u_C1R( value , G_ , pitch_ , ROI_) ;
            nppiSet_8u_C1R( value , B_ , pitch_ , ROI_) ;
        }
    }

};


#if 0
// =====================================================================
// base class for 1 Channel (grey) image
// =====================================================================
template<typename T>
class ImageC1Base
{
private:
    T*  data_ ;
    unsigned int width_ ;
    unsigned int height_ ;
    unsigned int pitch_ ;
    NppiSize ROI_ ;
public:
    ImageC1Base(unsigned int w, unsigned int h): width_(w) , height_(h)
    {
        cudaMallocManaged(&data_, width_ * height_ * sizeof(T));
        if((uintptr_t) data_ % sizeof(T)!=0)
            fprintf(stderr,"warning: memory allocation is not aligned!\n") ;

        pitch_ = width_ *  sizeof(T) ;
        ROI_= {(int)width_, (int)height_} ;  //ss Steve added type cast
    }
    inline T* data() {return data_ ; }
    inline unsigned int height() {	return height_ ;	}
    inline unsigned int width() {	return width_ ;		}
    inline unsigned int pitch(){ return pitch_ ; }
    inline void copyTo(T *dst, unsigned int dstPitch) {
        cudaMemcpy2D(dst, dstPitch , this->data_ , pitch_ , width_* sizeof(T) , height_ , cudaMemcpyDefault ) ;
    }
    inline void copyFrom(T *src, unsigned int srcPitch) {
        cudaMemcpy2D( this->data_ , pitch_  , src , srcPitch , width_* sizeof(T) , height_ , cudaMemcpyDefault ) ;
    }
    inline NppiSize getROI() {return ROI_;}
    inline void set(T value) { nppiSet_32f_C1R( value , data_ , pitch_ , ROI_) ; }
    inline void Free() { nppiFree(data_); }
};

// =====================================================================
// float- 1 Channel  image
// =====================================================================
class Image32f : public ImageC1Base<Npp32f> {
public:
    Image32f(unsigned int w, unsigned int h): ImageC1Base(w,h) {} ;
};

// =====================================================================
// 8u integer- 1 Channel image
// =====================================================================
class Image8u : public ImageC1Base<Npp8u> {
public:
    Image8u(unsigned int w, unsigned int h): ImageC1Base(w,h) {} ;
};

// =====================================================================
// base class for 3 Channel (RGB) image
// =====================================================================
template<typename T,typename T2>
class ImageC3Base
{
private:
        unsigned int width_ ;
        unsigned int height_ ;
        unsigned int pitch_ ;
        T *R_,*G_,*B_ ;
public:
    ImageC3Base(unsigned int w, unsigned int h):  width_(w) , height_(h)
    {
        R_ = new T(w,h) ;
        G_ = new T(w,h) ;
        B_ = new T(w,h) ;
        pitch_ = R_->pitch() ;
    }
     inline void copyTo(ImageC3Base dst)
     {
        R_->copyTo(dst.R_->data,pitch_) ;
        G_->copyTo(dst.G_->data,pitch_) ;
        B_->copyTo(dst.B_->data,pitch_) ;
     }
     inline void copyFrom(ImageC3Base src)
     {
        R_->copyFrom(src.R_->data,pitch_) ;
        G_->copyFrom(src.G_->data,pitch_) ;
        B_->copyFrom(src.B_->data,pitch_) ;
     }
    inline void set(T2 value)
    {
        R_->set(value) ;
        G_->set(value) ;
        B_->set(value) ;
    }
    inline unsigned int height() {	return height_ ;	}
    inline unsigned int width() {	return width_ ;		}
    inline unsigned int pitch(){ return pitch_ ; }
    inline T* R() { return R_; }
    inline T* G() { return G_; }
    inline T* B() { return B_; }
    inline void Free()
    {
        nppiFree(R_->data()) ;
        nppiFree(G_->data()) ;
        nppiFree(B_->data()) ;
    }
};

// =====================================================================
// float- 3 Channel (RGB) image
// =====================================================================
class Image32fC3 : public ImageC3Base<Image32f,Npp32f> {
public:
    Image32fC3(unsigned int w, unsigned int h): ImageC3Base(w,h) {} ;
};

// =====================================================================
// 8u integer- 3 Channel (RGB) image
// =====================================================================
class Image8uC3 : public ImageC3Base<Image8u,Npp8u> {
public:
    Image8uC3(unsigned int w, unsigned int h): ImageC3Base(w,h) {} ;
};
#endif


#if 0
struct Image32f
{
private:
    Npp32f * data_ ;
    unsigned int width_ ;
    unsigned int height_ ;
    unsigned int pitch_ ;
    NppiSize ROI_ ;
public:
    Image32f(unsigned int w, unsigned int h): width_(w) , height_(h)
    {
        cudaMallocManaged(&data_, width_ * height_ * sizeof(Npp32f));
        if((uintptr_t) data_ % sizeof(Npp32f)!=0)
            fprintf(stderr,"warning: memory allocation is not aligned!\n") ;

        pitch_ = width_ *  sizeof(Npp32f) ;
        ROI_= {(int)width_, (int)height_} ;  //ss Steve added type cast
    }
    Npp32f * data() {return data_ ; }
    unsigned int height() {	return height_ ;	}
    unsigned int width() {	return width_ ;		}
    unsigned int pitch(){ return pitch_ ; }
    void copyTo(Npp32f *dst, unsigned int dstPitch) {
        cudaMemcpy2D(dst, dstPitch , this->data_ , pitch_ , width_* sizeof(Npp32f) , height_ , cudaMemcpyDefault ) ;
    }
    void copyFrom(Npp32f *src, unsigned int srcPitch) {
        cudaMemcpy2D( this->data_ , pitch_  , src , srcPitch , width_* sizeof(Npp32f) , height_ , cudaMemcpyDefault ) ;
    }
    inline NppiSize getROI() {return ROI_;}
    inline void set(Npp32f value) { nppiSet_32f_C1R( value , data_ , pitch_ , ROI_) ; }
};

struct Image8u
{
private:
    Npp8u * data_ ;
    unsigned int width_ ;
    unsigned int height_ ;
    unsigned int pitch_ ;
public:
    Image8u(unsigned int w, unsigned int h): width_(w) , height_(h)
    {
        cudaMallocManaged(&data_, width_ * height_ * sizeof(Npp8u));
        pitch_ = width_ *  sizeof(Npp8u) ;
    }
    Npp8u * data() {return data_ ; }
    unsigned int height() {	return height_ ;	}
    unsigned int width() {	return width_ ;		}
    unsigned int pitch(){ return pitch_ ; }
    void copyTo(Npp8u *dst, unsigned int dstPitch) {
        cudaMemcpy2D(dst, dstPitch , this->data_ , pitch_ , width_* sizeof(Npp8u) , height_ , cudaMemcpyDefault ) ;
    }
    void copyFrom(Npp8u *src, unsigned int srcPitch) {
        cudaMemcpy2D( this->data_ , pitch_  , src , srcPitch , width_* sizeof(Npp8u) , height_ , cudaMemcpyDefault ) ;
    }
};
#endif


/*
 * loads an image from file and stores it on device memory
 * sFileName: file name
 * oDeviceImg: pointer to store the loaded image
 */
void loadImageCV(std::string sFileName, Image32f **oDeviceImg);

/*
 * saves an image from device memory to a file
 * sFileName: output file name
 * oDeviceImg: point to the image to be saved
 */
void writeImageCV(std::string sFileName, Image32f *oDeviceImg);

//      ts: [2.3000e-04 0.0094 0.0774 0.2405 0.3448 0.2405 0.0774 0.0094 2.3000e-04]
//     td1: [0.0012 0.0257 0.1214 0.1753 0 -0.1753 -0.1214 -0.0257 -0.0012]
//     td2: [0.0050 0.0563 0.1170 -0.0554 -0.2459 -0.0554 0.1170 0.0563 0.0050]
//      xs: [2.3000e-04 0.0094 0.0774 0.2405 0.3448 0.2405 0.0774 0.0094 2.3000e-04]
//     xd1: [0.0012 0.0257 0.1214 0.1753 0 -0.1753 -0.1214 -0.0257 -0.0012]
//    xavg: [1 1 1 1 1 1 1 1 1]
//    tavg: [1 1 1 1 1 1 1 1 1]
/*
 * Filters for flux/structure tensor spatial and temporal derivative and averaging and smoothing
 *
 */
struct fluxFilters {
#if 1
	Npp32f ts[5] = { 0.01504f , 0.23301f, 0.50390f , 0.23301f, 0.01504f };
	int ts_size = 5;
	int ts_anchor = 2;
	Npp32f* ts_devKernel; // pointer to device memory to keep its values

	Npp32f xs[5] = { 0.01554f, 0.23204f, 0.50484f, 0.23204f ,0.01554f };
	int xs_size = 5;
	int xs_anchor = 2;
	Npp32f* xs_devKernel; // pointer to device memory to keep its values

						  //Npp32f xd1[9]= {0.06368 0.37263 0 -0.37263 -0.06368} ;
	Npp32f xd1[5] = { -0.06368f, -0.37263f , 0.0f , 0.37263f, 0.06368f };
	int xd1_size = 5;
	int xd1_anchor = 2;
	Npp32f* xd1_devKernel; // pointer to device memory to keep its values

	Npp32f td1[5] = { -0.06368f, -0.37263f , 0.0f , 0.37263f, 0.06368f };
	int td1_size = 5;
	int td1_anchor = 2;
	Npp32f* td1_devKernel; // pointer to device memory to keep its values

	Npp32f td2[5] = { 0.20786f, 0.16854f ,-0.75282f, 0.16854f ,0.20786f };
	int  td2_size = 5;
	int td2_anchor = 2;
	Npp32f* td2_devKernel; // pointer to device memory to keep its values

	Npp32f xavg[5] = { 1 , 1, 1, 1, 1 };
	int xavg_size = 5;
	int xavg_anchor = 2;
	Npp32f* xavg_devKernel; // pointer to device memory to keep its values

	Npp32f tavg[5] = { 1 , 1, 1, 1, 1 };
	int tavg_size = 5;
	int tavg_anchor = 2;
	Npp32f* tavg_devKernel; // pointer to device memory to keep its values
#else
        Npp32f ts[9] = {0.0002, 0.0094, 0.0774, 0.2405, 0.3448, 0.2405, 0.0774, 0.0094, 0.0002} ;
        int ts_size = 9 ;
        int ts_anchor= 4 ;
        Npp32f* ts_devKernel ; // pointer to device memory to keep its values

        Npp32f xs[9] = {2.3000e-04,  0.0094, 0.0774,  0.2405 , 0.3448 , 0.2405 , 0.0774 , 0.0094  , 2.3000e-04} ;
        int xs_size = 9 ;
        int xs_anchor= 4 ;
        Npp32f* xs_devKernel ; // pointer to device memory to keep its values

        //Npp32f xd1[9]= {0.0012, 0.0257, 0.1214, 0.1753, 0, -0.1753, -0.1214, -0.0257, -0.0012} ;
        Npp32f xd1[9]= {-0.0012, -0.0257, -0.1214, -0.1753, 0, 0.1753, 0.1214, +0.0257, +0.0012} ;
        int xd1_size = 9 ;
        int xd1_anchor= 4 ;
        Npp32f* xd1_devKernel ; // pointer to device memory to keep its values

        Npp32f td1[9]= {0.0012, 0.0257, 0.1214, 0.1753, 0, -0.1753, -0.1214, -0.0257, -0.0012} ;
        int td1_size = 9 ;
        int td1_anchor= 4 ;
        Npp32f* td1_devKernel ; // pointer to device memory to keep its values

        Npp32f td2[9]= {0.0050 ,0.0563, 0.1170 ,-0.0554, -0.2459 ,-0.0554, 0.1170, 0.0563, 0.0050} ;
        int  td2_size = 9 ;
        int td2_anchor= 4 ;
        Npp32f* td2_devKernel ; // pointer to device memory to keep its values

        Npp32f xavg[9]= {1 , 1, 1, 1, 1, 1, 1, 1, 1} ;
        int xavg_size = 9 ;
        int xavg_anchor= 4 ;
        Npp32f* xavg_devKernel ; // pointer to device memory to keep its values

        Npp32f tavg[9]= {1 , 1, 1, 1, 1, 1, 1, 1, 1} ;
        int tavg_size = 9 ;
        int tavg_anchor= 4 ;
        Npp32f* tavg_devKernel ; // pointer to device memory to keep its values
#endif
};


struct filterKernel {
    NppiSize size ;
    Npp32f* hostKernel ;
    Npp8u* devKernel8u; // pointer to device memory
    Npp32f* devKernel32f;
    NppiPoint anchor ;
};

#if 0
struct filter_bank {
    filterKernel ts ;
    filterKernel xs ;
    filterKernel td1 ;
    filter_bank() {
        ts.size = {9, 1};
        ts.anchor={4,0} ;
        ts.hostKernel = new Npp32f[ts.size.height*ts.size.width ] ;
        Npp32f ts_values = {0.0002, 0.0094, 0.0774, 0.2405, 0.3448, 0.2405, 0.0774, 0.0094, 0.0002} ;
        fillHostKernelWithValue(ts.hostKernel,values)
        NPP_CHECK_CUDA(cudaMalloc((void**)&(ts.devKernel), ts.size.height * ts.size.width ));
    }
    void fillHostKernelWithValue(Npp32f* hostKernel, Npp32f* values , int nElements)
    {
        for(int i=0 ; i<nElements ; i++)
            *hostKernel[i]= *values[i] ;
    }
};
#endif

class FluxHelper
{
    bool debug_ ;
    std::string debug_outpath_ ;

    //Image32f *util_imgBuffer_1 ;
    //Image32f *util_imgBuffer_2 ;
    Image32f *util_4resetImagePad ;
    Image32f *util_4conv_32f ;
    Image32f *util_4computeIxIyIs ;
    Image32f *util32f_4OpenCV_mat_returns ;

    Image8u  *util8u_4OpenCV_mat_returns ;

    Npp32f * devScalar_32fBuffer_1 ;
    Npp32f * devScalar_32fBuffer_2 ;

    Npp8u * devBuffer_4nppiMax_ ;
    Npp8u * devBuffer_4dilation_ ;

    Npp32s *histDeviceBuffer_ ;
    Npp32f *levelsDeviceBuffer_ ;
    Npp8u *histDeviceBuffer_4nppInternalUse_;

    filterKernel devKernel5x5_ ;

    int  width_ , height_;
public:
    FluxHelper(int width,int height, bool debug=false, std::string debug_outpath="" ) ;
    ~FluxHelper() ;
    NppiSize getImgSize() ;
    void resetImagePad(Image32f *oDeviceSrc, const NppiSize& ROI_current, const NppiSize& ROI_desired) ;
    void applyFilter_Column_32f(Image32f *oDeviceSrc, Image32f *oDeviceDst, Npp32f *devKernel, int kernelSize, int kernelAnchor) ;
    void applyFilter_Row_32f(Image32f *oDeviceSrc, Image32f *oDeviceDst, Npp32f *deviceKernel, int kernelSize, int kernelAnchor) ;
    void conv_32f(Image32f *oDeviceSrc, Image32f *oDeviceDst, Npp32f *devKernel1, int kernelSize1, int kernelAnchor1,  Npp32f *devKernel2, int kernelSize2, int kernelAnchor2) ;
    void setSrcDstSafeBorders_32f( Image32f * oDeviceSrc,  Image32f * oDeviceDst, Npp32f **imgSrcOffset ,  Npp32f **imgDstOffset  , NppiSize *newROI) ;
    void setROI4Filtering_32f( Image32f * oDeviceSrc, Npp32f **imgOffset , NppiSize *ROI) ;
    void computeIxIyIs(Image32f *oDeviceSrc, Image32f *oDeviceDst_Ix ,
            Image32f *oDeviceDst_Iy ,
            Image32f *oDeviceDst_Is ,
            fluxFilters *filter1) ;
    Npp32f getMaxInImage_32f( Image32f *oDeviceSrc) ;
    Npp32f getMinInImage_32f( Image32f *oDeviceSrc) ;
    cv::Mat getMat8u_from_Npp32f( Image32f *oSrc , Npp32f truncate_val=1.0 , int channel=1, bool scale=false) ;
    void getNpp32f_from_Mat8u(cv::Mat *oSrc, Image32f * oDeviceDst,int channel=1) ;
    void computeHistRange_32f(Image32f *oDeviceSrc , Npp32s *histHost , int histBinCount, Npp32f *levelsHost) ;
    int getOtsuThreshold(Npp32s *histHost , int histBinCount, int TotalNumOfPixels) ;
    void ThresholdUsingOtsu_0toMax(Image32f *img, Npp32f newMin , Npp32f newMax) ;
    void StretchHist_32f( Image32f *oDeviceSrc, Image32f *oDeviceDst ,
            Npp32f min_percent , Npp32f max_percent, int histBinCount ) ;
    void dilate(Image32f *oDeviceSrc, Image32f *oDeviceDst) ;
    void dilate_v2(Image32f *oDeviceSrc, Image32f *oDeviceDst) ;
    void imWrite(std::string sFileName, Image32f *oDeviceImg, int N_channels=1, int channel=0) ;
    void imWriteBinary(std::string sFileName, Image32f *oDeviceImg) ;
    void imReadBinary(std::string sFileName, Image32f *oDeviceImg) ;
    void reScaleFrom0and1_to_0and255(Image32f *img) ;

};

template<typename T>
class ImgListBase {
protected:
    T **images_ ;
    int nextPos_ ;
    int capacity_ ;
    int nc_ ; // number of image channels
    bool initialized_ ;
public:
    void Init(int capacity,int img_width,int img_height, int N_channels=1) ;
    ImgListBase() ;
    ~ImgListBase() ;
    inline int getNextAvailablePos() {return nextPos_ ;}
    inline T *  getNextAvailableImg() {return images_[nextPos_] ;}
    int getLastFilledPos() ;
    void insert() ;
    void getImage(T **, int loc) ;
    T * getImage(int loc) ;
    T * getOldestImage() ;
    inline bool initialized() {return initialized_ ;}
    inline int getCapacity() { return capacity_ ; }
};
template<typename T>
void ImgListBase<T>::Init(int capacity,int img_width,int img_height, int N_channels)
{
    initialized_ = true ;
    capacity_ = capacity ;
    nc_ = N_channels ;
    nextPos_ = 0 ;
    images_ = new T *[capacity_] ;
    for(int i=0 ; i<capacity_ ; i++)
        images_[i] = new T(img_width,img_height,nc_) ;
}
template<typename T>
ImgListBase<T>::ImgListBase()
{
    capacity_ = 0 ;
    initialized_ = false ;
}
template<typename T>
ImgListBase<T>::~ImgListBase()
{
    if(!initialized_)
        return ;
    for(int i=0 ; i<capacity_ ; i++)
        images_[i]->Free() ;
    delete images_ ;
}
template<typename T>
void ImgListBase<T>::insert()
{
    assert(capacity_>0 && initialized_) ;
    //images_[nextPos_] = img ;
    if(++nextPos_ == capacity_)
        nextPos_ = 0 ;
}
template<typename T>
void ImgListBase<T>::getImage(T **img, int loc)
{
    assert(capacity_>0 && loc<capacity_ && loc>=0 && initialized_) ;
    *img = images_[loc] ;
}
template<typename T>
T * ImgListBase<T>::getImage(int loc)
{
    assert(capacity_>0 && loc<capacity_ && loc>=0 && initialized_) ;
    return images_[loc] ;
}
template<typename T>
T * ImgListBase<T>::getOldestImage()
{
    assert(initialized_) ;
    return images_[nextPos_] ;
}
template<typename T>
int ImgListBase<T>::getLastFilledPos()
{
    return nextPos_>0 ? nextPos_-1 : capacity_-1 ;
}

// =====================================================================
// float- 1 Channel  imageList
// =====================================================================
class ImgList32f : public ImgListBase<Image32f> {
public:
    ImgList32f(): ImgListBase() {} ;
};

#if 0
template<typename T>
class ImgListBase {
private:
    T **images_ ;
    int nextPos_ ;
    int capacity_ ;
    bool initialized_ ;
public:
    void Init(int capacity,int img_width,int img_height) ;
    ImgListBase() ;
    ~ImgListBase() ;
    inline int getNextAvailablePos() {return nextPos_ ;}
    inline T *  getNextAvailableImg() {return images_[nextPos_] ;}
    int getLastFilledPos() ;
    void insert() ;
    void getImage(T **, int loc) ;
    T * getImage(int loc) ;
    T * getOldestImage() ;
};
template<typename T>
void ImgListBase<T>::Init(int capacity,int img_width,int img_height)
{
    initialized_ = true ;
    capacity_ = capacity ;
    nextPos_ = 0 ;
    images_ = new T *[capacity_] ;
    for(int i=0 ; i<capacity_ ; i++)
        images_[i] = new T(img_width,img_height) ;
}
template<typename T>
ImgListBase<T>::ImgListBase()
{
    capacity_ = 0 ;
    initialized_ = false ;
}
template<typename T>
ImgListBase<T>::~ImgListBase()
{
    if(!initialized_)
        return ;
    for(int i=0 ; i<capacity_ ; i++)
        nppiFree( images_[i] ) ;
    delete images_ ;
}
template<typename T>
void ImgListBase<T>::insert()
{
    assert(capacity_>0) ;
    //images_[nextPos_] = img ;
    if(++nextPos_ == capacity_)
        nextPos_ = 0 ;
}
template<typename T>
void ImgListBase<T>::getImage(T **img, int loc)
{
    assert(capacity_>0 && loc<capacity_ && loc>=0) ;
    *img = images_[loc] ;
}
template<typename T>
T * ImgListBase<T>::getImage(int loc)
{
    assert(capacity_>0 && loc<capacity_ && loc>=0) ;
    return images_[loc] ;
}
template<typename T>
T * ImgListBase<T>::getOldestImage()
{
    return images_[nextPos_] ;
}
template<typename T>
int ImgListBase<T>::getLastFilledPos()
{
    return nextPos_>0 ? nextPos_-1 : capacity_-1 ;
}
// =====================================================================
// float- 1 Channel  imageList
// =====================================================================
class ImgList32fC1 : public ImgListBase<Image32f> {
public:
    ImgList32fC1(): ImgListBase() {} ;
};
// =====================================================================
// float- 3 Channels  imageList
// =====================================================================
class ImgList32fC3 : public ImgListBase<Image32fC3> {
public:
    ImgList32fC3(): ImgListBase() {} ;
};
#endif

#if 0
class ImgList32fC1 {
private:
    Image32f **images_ ;
    int nextPos_ ;
    int capacity_ ;
    bool initialized_ ;
public:
    void Init(int capacity,int img_width,int img_height) ;
    ImgList32fC1() ;
    ~ImgList32fC1() ;
    inline int getNextAvailablePos() {return nextPos_ ;}
    inline Image32f *  getNextAvailableImg() {return images_[nextPos_] ;}
    int getLastFilledPos() ;
    void insert() ;
    void getImage(Image32f **, int loc) ;
    Image32f * getImage(int loc) ;
    Image32f * getOldestImage() ;
};
#endif


class myClock
{
private:
	long tv_start, tv_end;
	long clock_exclude_start, clock_exclude_end, clock_exclude_total;
	long elapsed_time;
	double calculateElapsedTime_getDouble(clock_t &start, clock_t &end);
	clock_t calculateElapsedTime_getClockT(clock_t &start, clock_t &end);
	bool active;
	bool paused;

public:
	myClock();
	clock_t getTime();
	void tic();
	double toc();
	//void reset();
	double getElapsedTime();
	void pause();
	double resume(); // resume the pause and return the elapsed time while paused
	double getTotalPausedDuration();
	void print(std::string message);
};


int cudaDeviceInit(int argc, const char **argv) ;
bool printfNPPinfo(int argc, char *argv[]) ;
std::string getFileFullNameFromFullPath(const std::string& s) ;
bool splitFileNameAndExtension(const std::string& s, std::string& just_name,std::string& ext) ;
std::string getPathFromFullPath(const std::string& s);
void append2FileName(std::string& s, const std::string& postfix) ;
bool dirExists(const std::string dirName) ;
bool createFolderOrDie(const std::string dirName) ;
#endif /* FLUX_UTIL_H_ */
