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

#include "flux_util.h"

/*
 * loads an image from file and stores it on device memory
 * sFileName: file name
 * oDeviceImg: pointer to store the loaded image
 */
void loadImageCV(std::string sFileName, Image32f **oDeviceImg)
{

	IplImage* img=cvLoadImage(sFileName.c_str()) ;
	NppiSize oSize = {(int)img->width,(int)img->height} ;
    (*oDeviceImg)= new Image32f ((int)oSize.width, (int)oSize.height,3);

	IplImage* imgbw = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1) ;
	cvCvtColor(img, imgbw, CV_RGB2GRAY);
	npp::ImageNPP_8u_C1  * tmp = new npp::ImageNPP_8u_C1 (oSize.width, oSize.height);
	tmp->copyFrom((Npp8u *)imgbw->imageData,imgbw->widthStep ) ;


	nppiSet_32f_C1R (0, (*oDeviceImg)->data(), (*oDeviceImg)->pitch(), oSize) ;
	nppiAddWeighted_8u32f_C1IR( tmp->data(), tmp->pitch() , (*oDeviceImg)->data() , (*oDeviceImg)->pitch() , oSize , 1 ) ;

	nppiFree(tmp->data()) ;

	cvReleaseImage(&img);
	cvReleaseImage(&imgbw);
}
#if 0
/*
 * saves an image from device memory to a file
 * sFileName: output file name
 * oDeviceImg: point to the image to be saved
 */
void writeImageCV(std::string sFileName, Image32f *oDeviceImg)
{
	CvSize size1={(int)oDeviceImg->width(),(int)oDeviceImg->height()} ;
	IplImage* img32f= cvCreateImage(size1, IPL_DEPTH_32F,1) ;
	IplImage* img8u= cvCreateImage(size1, IPL_DEPTH_8U,1) ;

	oDeviceImg->copyTo((Npp32f *) img32f->imageData , img32f->widthStep) ;

//	//normalization
//	NppiSize ROI = {(int)oDeviceImg->width(),(int)oDeviceImg->height()} ;
//	nppiThreshold_32f_C1IR (oDeviceImg->data(),oDeviceImg->pitch(), ROI , 0.0f , NPP_CMP_LESS ) ;
//	nppiThreshold_32f_C1IR (oDeviceImg->data(),oDeviceImg->pitch(), ROI , 1.0f , NPP_CMP_GREATER ) ;
//	//nppiDivC_32f_C1IR( max_host , oDeviceDst->data(),oDeviceDst->pitch() , getImgSize() ) ;

	cvConvert(img32f,img8u) ;
	cvSaveImage(sFileName.c_str(),img32f) ; // img8u

	cvReleaseImage(&img8u);
	cvReleaseImage(&img32f);
}
#endif
FluxHelper::FluxHelper(int width,int height, bool debug, std::string debug_outpath)
{
    debug_ = debug ;
    debug_outpath_=debug_outpath ;
	width_=width ;
	height_=height ;
    //util_imgBuffer_1  = new Image32f( width_, height_) ;
    //util_imgBuffer_2  = new Image32f( width_, height_) ;
    util_4resetImagePad = new Image32f( width_, height_) ;
    util_4conv_32f = new Image32f( width_, height_) ;
    util_4computeIxIyIs = new Image32f( width_, height_) ;
    util8u_4OpenCV_mat_returns = new Image8u( width_, height_) ;
    util32f_4OpenCV_mat_returns = new Image32f( width_, height_) ;



    cudaMemErrChk(cudaMallocV2(&devScalar_32fBuffer_1, sizeof(Npp32f)));
    cudaMemErrChk(cudaMallocV2(&devScalar_32fBuffer_2, sizeof(Npp32f)));

	// Npp hist vars -----
	int histBinCount_max = 255 ;
	int levelCount_max=histBinCount_max+1 ;
    cudaMemErrChk(cudaMallocV2((void **)&histDeviceBuffer_,   histBinCount_max   * sizeof(Npp32s)));
    cudaMemErrChk(cudaMallocV2((void **)&levelsDeviceBuffer_, levelCount_max * sizeof(Npp32f)));
	//
	int nDeviceBufferSize;
	nppiHistogramRangeGetBufferSize_32f_C1R(getImgSize(), levelCount_max ,&nDeviceBufferSize);
    cudaMemErrChk(cudaMallocV2((void **)&histDeviceBuffer_4nppInternalUse_ , nDeviceBufferSize));
	// for max and min
	int bufferSize ;
	nppiMaxGetBufferHostSize_32f_C1R( getImgSize() , &bufferSize ) ;
    cudaMemErrChk(cudaMallocV2((void **)&devBuffer_4nppiMax_, bufferSize));

	devKernel5x5_.size = {5,5} ;
	devKernel5x5_.anchor = {2,2} ;
    cudaMemErrChk(cudaMallocV2((void**)&(devKernel5x5_.devKernel8u), devKernel5x5_.size.height * devKernel5x5_.size.width ));
    cudaMemset (devKernel5x5_.devKernel8u , 1 ,  devKernel5x5_.size.height * devKernel5x5_.size.width ) ;
    cudaMemErrChk(cudaMallocV2((void**)&(devKernel5x5_.devKernel32f), devKernel5x5_.size.height * devKernel5x5_.size.width * sizeof(Npp32f) ));
    float tmp1[]={1,1,1,1,1} ;
    NPP_CHECK_CUDA(cudaMemcpy(devKernel5x5_.devKernel32f, tmp1, devKernel5x5_.size.height * devKernel5x5_.size.width * sizeof(Npp32f) , cudaMemCpyType_H2D )) ;

    cudaDeviceSynchronize() ;
}
FluxHelper::~FluxHelper()
{
    //nppiFree(util_imgBuffer_1->data()) ;
    //nppiFree(util_imgBuffer_2->data()) ;
	nppiFree(util_4resetImagePad->data()) ;
	nppiFree(util_4conv_32f->data()) ;
	nppiFree(util_4computeIxIyIs->data()) ;
	nppiFree(util8u_4OpenCV_mat_returns->data()) ;
	nppiFree(util32f_4OpenCV_mat_returns->data()) ;

	cudaFree(devScalar_32fBuffer_1);
	cudaFree(devScalar_32fBuffer_2);

	cudaFree(histDeviceBuffer_) ;
	cudaFree(levelsDeviceBuffer_) ;
	cudaFree(histDeviceBuffer_4nppInternalUse_) ;

	cudaFree(devBuffer_4nppiMax_);

    cudaFree(devKernel5x5_.devKernel8u) ;
}

/*
 * returns the set image dimension
 */
NppiSize FluxHelper::getImgSize()
{
	NppiSize oSize = { width_, height_};
	return oSize ;
}

/*
 * To translate back the processed image into its original coordinates.
 * This is to compensate the Neighboring NPP operation which removes borders
 * Param:
 * oDeviceSrc: pointer to the input image. Result will be overwritten to the input image
 * ROI_current:Current ROI
 * ROI_desired: Desired ROI
 */
void FluxHelper::resetImagePad(Image32f *oDeviceSrc, const NppiSize& ROI_current, const NppiSize& ROI_desired)
{
	assert(oDeviceSrc->width()==width_ && oDeviceSrc->height()==height_) ;
	//NppiSize ROI_src={ width_, height() } ;
	int topBorderHeight = (ROI_desired.height - ROI_current.height)/2 ;
	int leftBorderWidth = (ROI_desired.width - ROI_current.width)/2 ;
	Npp32f pixValue = 0 ;
	nppiCopyConstBorder_32f_C1R(oDeviceSrc->data(), oDeviceSrc->pitch(), ROI_current ,
			util_4resetImagePad->data() , util_4resetImagePad->pitch() ,
			ROI_desired ,  topBorderHeight , leftBorderWidth , pixValue ) ;
	util_4resetImagePad->copyTo(oDeviceSrc->data(),oDeviceSrc->pitch()) ;
}

void FluxHelper::setSrcDstSafeBorders_32f( Image32f * oDeviceSrc, Image32f * oDeviceDst,  Npp32f **imgSrcOffset ,  Npp32f **imgDstOffset  , NppiSize *newROI)
{
    int pixelSize = sizeof(Npp32f) ;
    int xROI,yROI ;
    xROI = 10 ;
    yROI = 10  ;
    *imgSrcOffset = oDeviceSrc->data() + yROI * oDeviceSrc->pitch() + xROI * pixelSize ;
    newROI->width  = oDeviceSrc->width() - 50 ; //50
    newROI->height = oDeviceSrc->height() - 90 ; //90
    *imgDstOffset = oDeviceDst->data() + yROI * oDeviceDst->pitch() + xROI * pixelSize ;
}

/*
 * To set a ROIin which the image borders are cropped which is needed for Neighboring NPP operations.
 * img: Pointer to the image.
 * imgOffset: Pointer to the new offset in the image which will be computed and assigned. This is an output.
 * ROI: new computed ROI
 */
void FluxHelper::setROI4Filtering_32f( Image32f * oDeviceSrc, Npp32f **imgOffset , NppiSize *ROI)
{
	assert(oDeviceSrc->width()==width_ && oDeviceSrc->height()==height_) ;
	int pixelSize = sizeof(Npp32f) ;
	int xROI,yROI ;
	xROI = 10 ;
	yROI = 10  ;
	*imgOffset = oDeviceSrc->data() + yROI * oDeviceSrc->pitch() + xROI * pixelSize ;
	if(0) {
		std::cout << "address before: " << (unsigned long long int)oDeviceSrc->data() << std::endl;
		std::cout << "pitch size is: " << (unsigned long long int)oDeviceSrc->pitch() << std::endl;
		std::cout << "address after : " << (unsigned long long int)(*imgOffset) << std::endl;
		std::cout << "difference is : " << (unsigned long long int)((*imgOffset)-oDeviceSrc->data()) << std::endl;
	}
	ROI->width  = oDeviceSrc->width() - 50 ; //50
	ROI->height = oDeviceSrc->height() - 90 ; //90
}
void FluxHelper::dilate_v2(Image32f *oDeviceSrc, Image32f *oDeviceDst)
{
    conv_32f( oDeviceSrc , oDeviceDst ,
           devKernel5x5_.devKernel32f , 5 , 2 ,
           devKernel5x5_.devKernel32f , 5 , 2 ) ;
    //nppiThreshold_LTValGTVal_32f_C1IR(DP(oDeviceDst),getImgSize(), 0,0 , getMaxInImage_32f(oDeviceSrc),getMaxInImage_32f(oDeviceSrc)) ;
}

/*
 * Morphological dilation on an image
 * oDeviceSrc: pointer to input image
 * oDeviceDst: pointer to output image
 */
void FluxHelper::dilate(Image32f *oDeviceSrc, Image32f *oDeviceDst)
{
	NppiSize ROI2 ;
	Npp32f* pSrcOffset ;
	Npp32f* pDstOffset ;
	setSrcDstSafeBorders_32f (oDeviceSrc, oDeviceDst , &pSrcOffset, &pDstOffset , &ROI2) ;
	NPP_CHECK_NPP (
			nppiDilate_32f_C1R (pSrcOffset, oDeviceSrc->pitch(),
					pDstOffset, oDeviceDst->pitch(),
                    ROI2, devKernel5x5_.devKernel8u, devKernel5x5_.size, devKernel5x5_.anchor)) ;
#if 0
	assert(oDeviceSrc->width()==width_ && oDeviceSrc->height()==height_) ;
	assert(oDeviceDst->width()==width_ && oDeviceDst->height()==height_) ;

	NppiSize ROI2 ;
	Npp32f* pSrcOffset ;
	setROI4Filtering_32f (oDeviceSrc , &pSrcOffset , &ROI2) ;

	NPP_CHECK_NPP (
			nppiDilate_32f_C1R (pSrcOffset, oDeviceSrc->pitch(),
					oDeviceDst->data(), oDeviceDst->pitch(),
					ROI2, devKernel5x5_.devKernel, devKernel5x5_.size, devKernel5x5_.anchor)) ;

	//	cudaFree(deviceKernel) ;

	NppiSize ROI_src={ (int) width_, (int) height_ } ;
	resetImagePad(oDeviceDst, ROI2, ROI_src) ;
#endif
}

/*
 * Applies a 1D Column kernel on an image
 * oDeviceSrc: Pointer to the source image
 * oDeviceDst: Pointer to the result image
 * oDeviceDst, kernelSize and kernelAnchor: Kernel (filter) parameters
 * deviceKernel, must be device memory pointer
 */
void FluxHelper::applyFilter_Column_32f(Image32f *oDeviceSrc, Image32f *oDeviceDst, Npp32f *deviceKernel, Npp32s kernelSize, Npp32s kernelAnchor)
{
#if 0
    NppiPoint offset = { 0, 0};
    NPP_CHECK_NPP (
                nppiFilterColumnBorder_32f_C1R(
                    DP(oDeviceSrc) ,
                    getImgSize() , offset ,
                    DP(oDeviceDst) ,
                    getImgSize() ,
                    deviceKernel, kernelSize, kernelAnchor ,
                    NPP_BORDER_REPLICATE
                    )
                ) ;
#else
	NppiSize ROI2 ;
	Npp32f* pSrcOffset ;
	Npp32f* pDstOffset ;
	setSrcDstSafeBorders_32f (oDeviceSrc, oDeviceDst , &pSrcOffset, &pDstOffset , &ROI2) ;
	NPP_CHECK_NPP (
			nppiFilterColumn_32f_C1R (pSrcOffset, oDeviceSrc->pitch(),
					pDstOffset , oDeviceDst->pitch(),
					ROI2, deviceKernel, kernelSize, kernelAnchor)) ;
#endif
#if 0
	NppiSize ROI2 ;
	Npp32f* pSrcOffset ;
	this->setROI4Filtering_32f (oDeviceSrc , &pSrcOffset, &ROI2) ;
	NPP_CHECK_NPP (
			nppiFilterColumn_32f_C1R (pSrcOffset, oDeviceSrc->pitch(),
					oDeviceDst->data(), oDeviceDst->pitch(),
					ROI2, deviceKernel, kernelSize, kernelAnchor)) ;
	NppiSize ROI_src={ (int) oDeviceSrc->width(), (int) oDeviceSrc->height() } ;
	this->resetImagePad(oDeviceDst, ROI2, ROI_src) ;
#endif
}

/*
 * Applies a 1D Row kernel on an image
 * oDeviceSrc: Pointer to the source image
 * oDeviceDst: Pointer to the result image
 * oDeviceDst, kernelSize and kernelAnchor: Kernel (filter) parameters
 * deviceKernel, must be device memory pointer
 */
void FluxHelper::applyFilter_Row_32f(Image32f *oDeviceSrc, Image32f *oDeviceDst,
		Npp32f *deviceKernel, int kernelSize, int kernelAnchor)
{
#if 0
    NppiPoint offset = { 0, 0};
    NPP_CHECK_NPP (
                nppiFilterRowBorder_32f_C1R(
                    DP(oDeviceSrc) ,
                    getImgSize() , offset ,
                    DP(oDeviceDst) ,
                    getImgSize() ,
                    deviceKernel, kernelSize, kernelAnchor ,
                    NPP_BORDER_REPLICATE
                    )
                ) ;
#else
    NppiSize ROI2 ;
	Npp32f* pSrcOffset ;
	Npp32f* pDstOffset ;
	setSrcDstSafeBorders_32f (oDeviceSrc, oDeviceDst , &pSrcOffset, &pDstOffset , &ROI2) ;
	NPP_CHECK_NPP (
			nppiFilterRow_32f_C1R (pSrcOffset, oDeviceSrc->pitch(),
					pDstOffset , oDeviceDst->pitch(),
					ROI2, deviceKernel, kernelSize, kernelAnchor)) ;
#endif
#if 0
	NppiSize ROI2 ;
	Npp32f* pSrcOffset ;
	setROI4Filtering_32f (oDeviceSrc , &pSrcOffset,&ROI2) ;

	NPP_CHECK_NPP (
			nppiFilterRow_32f_C1R (pSrcOffset, oDeviceSrc->pitch(),
					oDeviceDst->data(), oDeviceDst->pitch(),
					ROI2, deviceKernel, kernelSize, kernelAnchor)) ;
	NppiSize ROI_src={ (int) oDeviceSrc->width(), (int) oDeviceSrc->height() } ;
	resetImagePad(oDeviceDst, ROI2, ROI_src) ;
#endif
}

void FluxHelper::conv_32f(Image32f *oDeviceSrc, Image32f *oDeviceDst, Npp32f *devKernel1, int kernelSize1, int kernelAnchor1,  Npp32f *devKernel2, int kernelSize2, int kernelAnchor2)
{
    this->applyFilter_Row_32f(oDeviceSrc, util_4conv_32f , devKernel1 , kernelSize1, kernelAnchor1) ;
    this->applyFilter_Column_32f(util_4conv_32f, oDeviceDst , devKernel2 , kernelSize2, kernelAnchor2) ;
}

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
void FluxHelper::computeIxIyIs(Image32f *oDeviceSrc, Image32f *oDeviceDst_Ix ,
        Image32f *oDeviceDst_Iy ,
        Image32f *oDeviceDst_Is ,
		fluxFilters *filter1)
{
	// Ix
    //imWrite("/home/ubuntu/Downloads/oDeviceSrc.png",oDeviceSrc) ;
	applyFilter_Row_32f(oDeviceSrc, util_4computeIxIyIs , filter1->xd1_devKernel , filter1->xd1_size, filter1->xd1_anchor) ;
    //imWrite("/home/ubuntu/Downloads/util_4computeIxIyIs-1.png",util_4computeIxIyIs) ;
	applyFilter_Column_32f( util_4computeIxIyIs, oDeviceDst_Ix , filter1->xs_devKernel  , filter1->xs_size, filter1->xs_anchor) ;
    //imWrite("/home/ubuntu/Downloads/oDeviceDst_Ix.png",oDeviceDst_Ix) ;

	// Iy
	applyFilter_Row_32f (oDeviceSrc, util_4computeIxIyIs , filter1->xs_devKernel  , filter1->xs_size, filter1->xs_anchor) ;
	applyFilter_Column_32f(util_4computeIxIyIs, oDeviceDst_Iy , filter1->xd1_devKernel , filter1->xd1_size, filter1->xd1_anchor) ;
    //imWrite("/home/ubuntu/Downloads/oDeviceDst_Iy.png",oDeviceDst_Iy) ;

	// Is
	applyFilter_Row_32f(oDeviceSrc, util_4computeIxIyIs , filter1->xs_devKernel  , filter1->xs_size, filter1->xs_anchor) ;
	applyFilter_Column_32f (util_4computeIxIyIs, oDeviceDst_Is , filter1->xs_devKernel , filter1->xs_size, filter1->xs_anchor) ;
    //imWrite("/home/ubuntu/Downloads/oDeviceDst_Is.png",oDeviceDst_Is) ;
}


/*
 * Returns max in an image
 * oDeviceSrc: pointer to the image
 * ROI: Image ROI (input)
 */
Npp32f FluxHelper::getMaxInImage_32f( Image32f *oDeviceSrc)
{
	Npp32f max_host ;
	Npp32f* max_dev;
	max_dev = devScalar_32fBuffer_1 ;
	NPP_CHECK_NPP (
			nppiMax_32f_C1R ( oDeviceSrc->data() , oDeviceSrc->pitch() , getImgSize() , devBuffer_4nppiMax_ ,  max_dev) ;
	) ;
    cudaMemcpy(&max_host, max_dev, sizeof(Npp32f), cudaMemCpyType_D2H);
	return max_host ;
}

/*
 * Returns min in an image
 * oDeviceSrc: pointer to the image
 * ROI: Image ROI (input)
 */
Npp32f FluxHelper::getMinInImage_32f( Image32f *oDeviceSrc)
{
	Npp32f min_host ;
	Npp32f* min_dev;
	min_dev = devScalar_32fBuffer_1 ;
	NPP_CHECK_NPP (
			nppiMin_32f_C1R ( oDeviceSrc->data() , oDeviceSrc->pitch() , getImgSize() , devBuffer_4nppiMax_ ,  min_dev) ;
	) ;
    cudaMemcpy(&min_host, min_dev, sizeof(Npp32f), cudaMemCpyType_D2H);
	return min_host ;
}

// rescale values between 0-1 to 0-255. (To be compatible with Matlab's version)
void FluxHelper::reScaleFrom0and1_to_0and255(Image32f *img)
{
	// truncate values sma;;er than 0 and larger than 1
	nppiThreshold_LTValGTVal_32f_C1IR(DP(img),getImgSize(), 0,0 , 1,1) ;
	// mulitply by 255 (scale up)
	nppiMulC_32f_C1IR(255,DP(img),getImgSize()) ;
}

//withNormalization_0_to_255 without truncating
cv::Mat FluxHelper::getMat8u_from_Npp32f( Image32f *oSrc, Npp32f truncate_val , int channel, bool scale)
{
#ifdef USE_UNIFIEDMEM
    Npp32f *src_data ;
     if(channel==1)
         src_data = oSrc->R() ;
     else if(channel==2)
         src_data = oSrc->G() ;
     else
         src_data = oSrc->B() ;
     NVTX_PUSHSync("npThresh",0);
     nppiThreshold_LTValGTVal_32f_C1R( src_data , oSrc->pitch() ,  DP(util32f_4OpenCV_mat_returns) , getImgSize() ,
             0 , 0 , 255 ,255 ) ;
     NVTX_POPSync;

     NVTX_PUSHSync("npCvt",0);
     nppiConvert_32f8u_C1R( DP(util32f_4OpenCV_mat_returns) ,
                 DP(util8u_4OpenCV_mat_returns) , getImgSize() ,
                 NPP_RND_NEAR ) ;
     NVTX_POPSync;

     cv::Mat dstMat8u(getImgSize().height , getImgSize().width , CV_8UC1);
     util8u_4OpenCV_mat_returns->copyTo((Npp8u *) dstMat8u.data , dstMat8u.step) ;
     return dstMat8u ;
#else

    Npp32f *src_data ;
    if(channel==1)
        src_data = oSrc->R() ;
    else if(channel==2)
        src_data = oSrc->G() ;
    else
        src_data = oSrc->B() ;
    nppiThreshold_LTValGTVal_32f_C1R( src_data , oSrc->pitch() ,  DP(util32f_4OpenCV_mat_returns) , getImgSize() ,
            0 , 0 , 255 ,255 ) ;

    nppiConvert_32f8u_C1R( DP(util32f_4OpenCV_mat_returns) ,
                DP(util8u_4OpenCV_mat_returns) , getImgSize() ,
                NPP_RND_NEAR ) ;

    cv::Mat dstMat8u(getImgSize().height , getImgSize().width , CV_8UC1);
    //util8u_4OpenCV_mat_returns->copyTo((Npp8u *) dstMat8u.data , dstMat8u.step) ;
    cudaMemcpy2D((Npp8u *) dstMat8u.data , dstMat8u.step ,
                 util8u_4OpenCV_mat_returns->data(), util8u_4OpenCV_mat_returns->pitch()  ,
                 util8u_4OpenCV_mat_returns->width() , util8u_4OpenCV_mat_returns->height() ,
                 cudaMemcpyDeviceToHost) ;
    //cv::imwrite("/home/ubuntu/Downloads/util8u_4OpenCV_mat_returns.jpg",dstMat8u) ;

    return dstMat8u ;
#endif

#if 0 // until 12 Oct 2016
	//copy to avoid value changes
	oSrc->copyTo(util32f_4OpenCV_mat_returns->data(), util32f_4OpenCV_mat_returns->pitch()) ;
	nppiThreshold_32f_C1IR (util32f_4OpenCV_mat_returns->data(),util32f_4OpenCV_mat_returns->pitch(), getImgSize() , truncate_val , NPP_CMP_GREATER ) ;
	nppiThreshold_32f_C1IR (util32f_4OpenCV_mat_returns->data(),util32f_4OpenCV_mat_returns->pitch(), getImgSize() , 0.0f , NPP_CMP_LESS ) ;
if(scale)
	nppiConvert_32f8u_C1RSfs ( DP(util32f_4OpenCV_mat_returns) ,
				DP(util8u_4OpenCV_mat_returns) , getImgSize() ,
				NPP_RND_NEAR , -8 ) ;
else
	nppiConvert_32f8u_C1R( DP(util32f_4OpenCV_mat_returns) ,
				DP(util8u_4OpenCV_mat_returns) , getImgSize() ,
				NPP_RND_NEAR ) ;

	cv::Mat dstMat8u(getImgSize().height , getImgSize().width , CV_8UC1);
	util8u_4OpenCV_mat_returns->copyTo((Npp8u *) dstMat8u.data , dstMat8u.step) ;
	return dstMat8u ;
#endif
}

void FluxHelper::getNpp32f_from_Mat8u(cv::Mat *oSrc, Image32f * oDeviceDst, int channel)
{
#ifdef USE_UNIFIEDMEM
    util8u_4OpenCV_mat_returns->copyFrom(oSrc->data , oSrc->step) ;
       Npp32f * dst ;
       if(channel==1)
           dst=oDeviceDst->R() ;
       else if(channel==2)
           dst=oDeviceDst->G() ;
       else
           dst=oDeviceDst->B() ;
       nppiConvert_8u32f_C1R( util8u_4OpenCV_mat_returns->data() , util8u_4OpenCV_mat_returns->pitch() , dst , oDeviceDst->pitch() , getImgSize() ) ;
#else

    //    cv::imwrite("/home/ubuntu/Downloads/oSrc.jpg",*oSrc) ;
        cudaMemcpy2D(util8u_4OpenCV_mat_returns->data(), util8u_4OpenCV_mat_returns->pitch() ,
                     oSrc->data , oSrc->step ,
                     util8u_4OpenCV_mat_returns->width() , util8u_4OpenCV_mat_returns->height() ,
                     cudaMemcpyHostToDevice ) ;
    //    util8u_4OpenCV_mat_returns->copyFrom(oSrc->data , oSrc->step) ;
        Npp32f * dst ;
        if(channel==1)
            dst=oDeviceDst->R() ;
        else if(channel==2)
            dst=oDeviceDst->G() ;
        else
            dst=oDeviceDst->B() ;
        nppiConvert_8u32f_C1R( util8u_4OpenCV_mat_returns->data() , util8u_4OpenCV_mat_returns->pitch() , dst , oDeviceDst->pitch() , getImgSize() ) ;
    //    imWrite( "/home/ubuntu/Downloads/oDeviceDst.jpg" , oDeviceDst) ;
    //
    //    cv::Mat dstMat8u(getImgSize().height , getImgSize().width , CV_8UC1);
    //    cudaMemcpy2D((Npp8u *) dstMat8u.data , dstMat8u.step ,
    //                 util8u_4OpenCV_mat_returns->data(), util8u_4OpenCV_mat_returns->pitch()  ,
    //                 util8u_4OpenCV_mat_returns->width() , util8u_4OpenCV_mat_returns->height() ,
    //                 cudaMemcpyDeviceToHost) ;
    //    cv::imwrite("/home/ubuntu/Downloads/util8u_4OpenCV_mat_returns.jpg",dstMat8u) ;

            //util8u_4OpenCV_mat_returns->copyTo((Npp8u *) dstMat8u.data , dstMat8u.step) ;
#endif
}


/*
 * Computes histogram of the image oDeviceSrc
 * Inputs: oDeviceSrc pointer to the image
 * histBinCount: number of bins
 * histHost: a pre-allocated array of Npp32s  with size of histBinCount
 * levelsHost: a pre-allocated array of Npp32s  with size of histBinCount+1
 */
void FluxHelper::computeHistRange_32f(Image32f *oDeviceSrc , Npp32s *histHost , int histBinCount, Npp32f *levelsHost)
{
	Npp32s *histDevice = histDeviceBuffer_ ;
	Npp32f *levelsDevice = levelsDeviceBuffer_ ;

	const int levelCount = histBinCount + 1; // levels array has one more element
	NppiSize oSizeROI = {(int)oDeviceSrc->width() , (int)oDeviceSrc->height() };


	//	NPP_CHECK_CUDA(cudaMalloc((void **)&histDevice,   histBinCount   * sizeof(Npp32s)));
	//	NPP_CHECK_CUDA(cudaMalloc((void **)&levelsDevice, levelCount * sizeof(Npp32f)));
	//
	//	// create device scratch buffer for nppiHistogram
	//	int nDeviceBufferSize;
	//	nppiHistogramRangeGetBufferSize_32f_C1R(oSizeROI, levelCount ,&nDeviceBufferSize);
	//	NPP_CHECK_CUDA(cudaMalloc((void **)&pDeviceBuffer, nDeviceBufferSize));

	Npp8u *pDeviceBuffer=histDeviceBuffer_4nppInternalUse_ ;


	Npp32f step,xStart , localMaxVal ;
	localMaxVal =  this->getMaxInImage_32f (oDeviceSrc) ;
	step = localMaxVal/histBinCount ;
	xStart = step/2.0f ;
	for  (int k=0; k < levelCount ; k++)
		levelsHost[k] = xStart + k * step ;

    cudaMemcpy(levelsDevice,levelsHost, levelCount * sizeof(Npp32f), cudaMemCpyType_H2D);

	nppiHistogramRange_32f_C1R( oDeviceSrc->data() ,  oDeviceSrc->pitch(), oSizeROI ,
			histDevice	, levelsDevice , levelCount , pDeviceBuffer) ;

    cudaMemcpy(histHost, histDevice, histBinCount * sizeof(Npp32s), cudaMemCpyType_H2D);

	//	cudaFree(histDevice);
	//	cudaFree(levelsDevice);
	//	cudaFree(pDeviceBuffer);
}

/*
 * Returns the Otsu threshold from image histogram
 * histHost: histogram as input
 * histBinCount: number of bins
 * TotalNumOfPixels: total number of pixels in image
 */
int FluxHelper::getOtsuThreshold(Npp32s *histHost , int histBinCount, int TotalNumOfPixels)
{
	const int levelCount = histBinCount + 1;

	Npp32s sum = 0;
	Npp32s sumB = 0;
	Npp32s q1 = 0;
	Npp32s q2 = 0;
	float varMax = 0;
	int threshold = 0;
	int i ;

	for (i = 0; i < histBinCount; i++)
		sum += i * ((int)histHost[i]);
	for (i = 0 ; i < histBinCount ; i++) {
		// Update q1
		q1 += histHost[i];
		if (q1 == 0)
			continue;
		// Update q2
		q2 = TotalNumOfPixels - q1;

		if (q2 == 0)
			break;
		// Update m1 and m2
		sumB += (Npp32s) (i * ((int)histHost[i]));
		Npp32f m1 = sumB / q1;
		Npp32f m2 = (sum - sumB) / q2;

		// Update the between class variance
		Npp32f varBetween = (Npp32f) q1 * (Npp32f) q2 * (m1 - m2) * (m1 - m2);

		// Update the threshold if necessary
		if (varBetween > varMax) {
			varMax = varBetween;
			threshold = i;
		}
	}
	return threshold ;
}
/*
 * Threshold an image using Otsu method
 * img: image to be thresholded (in place). The result for each pixel will be 0 or 255
 * reverse: set to true if you want to apply a reverse threshold
 */
void FluxHelper::ThresholdUsingOtsu_0toMax(Image32f *img, Npp32f newMin , Npp32f newMax)
{
	const int histBinCount_ = 255 ;
	const int levelCount = histBinCount_ + 1;
	Npp32s histHost[histBinCount_] ;
	Npp32f levelsHost[levelCount] ;
	computeHistRange_32f(img , histHost , histBinCount_ , levelsHost) ;
	int TotalNumOfPixels = img->width() * img->height()  ;
	int thresh_pos = getOtsuThreshold( histHost , histBinCount_ ,TotalNumOfPixels) ;
	Npp32f threshValue = levelsHost[thresh_pos] ;
	const Npp32f EPSILON = 1.19209e-07 ;
	//const Npp32f newMin = 0 ;
	//const Npp32f newMax = 255 ;
		NPP_CHECK_NPP (
				nppiThreshold_LTValGTVal_32f_C1IR(img->data(),img->pitch(), getImgSize() ,
						threshValue , newMin , threshValue - EPSILON , newMax ) ;
		) ;
}
/*
 * To strech an image using desired percentage from begining and end
 * oDeviceSrc: pointer to the input image
 * oDeviceDst: Pointer to where the streched image will be stored.
 * min_percent and max_percent: Minumum and maximum of percentages for the begining and end.
 * Accepted values are between 0 and 1
 */
void FluxHelper::StretchHist_32f( Image32f *oDeviceSrc, Image32f *oDeviceDst ,
		Npp32f min_percent , Npp32f max_percent, int histBinCount )
{
	int k ;
	const int levelCount = histBinCount + 1;
	Npp32s *histHost = new Npp32s[histBinCount] ;
	Npp32f *levelsHost = new Npp32f[levelCount];
	computeHistRange_32f(oDeviceSrc , histHost , histBinCount , levelsHost) ;

	//2116 , 186
	//	NppiSize oSizeROI = {(int)oDeviceSrc->width() , (int)oDeviceSrc->height() };

	//-------normalized cumulative histogram-------
	int sum ;
	Npp32f *accumHist = new Npp32f[histBinCount] ;
	for(k=0 , sum=0 ; k<histBinCount ; k++) {
		sum += histHost[k] ;
		accumHist[k] = sum ;
	}
	for(k=0  ; k<histBinCount ; k++)
		accumHist[k] = (float)accumHist[k]/(float)sum    ;

	//-------histogram cropping--------------------
	int i_lower=-1, i_upper=-1 ;
	Npp32f lower_center , upper_center ;
	for(k=0  ; k<levelCount && (i_upper==-1 || i_lower==-1)  ; k++) {
		if(i_lower==-1 && accumHist[k] >= min_percent)
			i_lower = k ;
		if(i_upper==-1 && accumHist[k]>=max_percent)
			i_upper = k ;
	}
	lower_center = levelsHost[i_lower] ;
	upper_center = levelsHost[i_upper] ;

	oDeviceSrc->copyTo(oDeviceDst->data(),oDeviceDst->pitch()) ;
	//nppiThreshold_LTValGTVal_32f_C1IR
	NPP_CHECK_NPP (
			nppiThreshold_32f_C1IR (oDeviceDst->data(),oDeviceDst->pitch(), getImgSize() , lower_center , NPP_CMP_LESS ) ;
	) ;
	NPP_CHECK_NPP (
			nppiThreshold_32f_C1IR (oDeviceDst->data(),oDeviceDst->pitch(), getImgSize()  , upper_center , NPP_CMP_GREATER ) ;
	) ;
	delete accumHist;
	delete levelsHost;
	delete histHost;
}
void FluxHelper::imWrite(std::string sFileName, Image32f *oDeviceImg, int planes, int channel)
{
    printf("Saving image to %s ...",sFileName.c_str());
    cv::Mat mat ;
    if(planes==1) // greyscale
    {
        mat = getMat8u_from_Npp32f(oDeviceImg,255.0) ;
        cv::imwrite(sFileName.c_str(),mat) ;
    }
    else // color
    {
        if(channel==0)  // all thee channels as one color
        {
            std::vector<cv::Mat> channels ;
            cv::Mat mat_res ;
            mat = getMat8u_from_Npp32f(oDeviceImg,255.0,3) ;
            channels.push_back(mat);
            mat = getMat8u_from_Npp32f(oDeviceImg,255.0,2) ;
            channels.push_back(mat);
            mat = getMat8u_from_Npp32f(oDeviceImg,255.0,1) ;
            channels.push_back(mat);
            cv::merge(channels,mat_res) ;
            cv::imwrite(sFileName.c_str(),mat_res) ;
        }
        else { // just a particular channel between the three
            mat = getMat8u_from_Npp32f(oDeviceImg,255.0,channel) ;
            cv::imwrite(sFileName.c_str(),mat) ;
        }
    }
    printf(" Done.\n") ;
}
void FluxHelper::imWriteBinary(std::string sFileName, Image32f *oDeviceImg)
{
    std::ofstream out(sFileName, std::ios::out | std::ios::binary) ;
    if(!out) {
        std::cerr << "Error in opening binary files for storing.\n" ;
        return ;
    }
    out.write((char *) oDeviceImg->data() , oDeviceImg->pitch()*oDeviceImg->height()) ;
    out.close();
}
void FluxHelper::imReadBinary(std::string sFileName, Image32f *oDeviceImg)
{
    std::ifstream in(sFileName, std::ios::in | std::ios::binary) ;
    in.read((char *) oDeviceImg->data() , oDeviceImg->pitch()*oDeviceImg->height()) ;
    if(in.gcount()<=0)
    {
        std::cerr << "Error in reading binary files for restoring.\n" ;
        return ;
    }
}

///*
// * loads an image from file and stores it on device memory
// * sFileName: file name
// * oDeviceImg: pointer to store the loaded image
// */
//void FluxHelper::loadImageCV(std::string sFileName, Image32f *oDeviceImg)
//{
//	cv::Mat colorMat;
//	colorMat = cv::imread(sFilename, CV_LOAD_IMAGE_COLOR);
//	cv::Mat greyMat ;
//	cv::cvtColor(colorMat, greyMat, CV_BGR2GRAY);
//
//	cv::Mat img32f;
//	img8u->convertTo(img32f, CV_32F);
//	//cv::imwrite("/media/ubuntu/SataHDD1/data/16-02-26-1422-pos3_D_small/output/cvOut_test.jpg",img32f) ;
//
//	oDeviceImg->copyFrom( (Npp32f *)img32f.data , img32f.step) ;
//}



//--------------------------------------------------------------------------------
//uint32_t elapsed(timeval &a, timeval &b)
//{
//	return (  ((b.tv_sec-a.tv_sec)*1000)   +   ((b.tv_usec-a.tv_usec)/1000)  );
//}
//
//uint myClock::calculateElapsedTime(timeval &a,timeval &b)
//{
//	return ((b.tv_sec-a.tv_sec)*1000)   +   ((b.tv_usec-a.tv_usec)/1000) ;
//}
//void myClock::tic()
//{
//	gettimeofday(&tv_start, NULL);
//}
//uint myClock::toc()
//{
//	gettimeofday(&tv_end, NULL);
//	elapsed_time = calculateElapsedTime(tv_start, tv_end)/1000 ;
//	return elapsed_time ;
//}
//uint myClock::getElapsedTime()
//{
//	return elapsed_time ;
//}
///////////////////////

int cudaDeviceInit(int argc, const char **argv)
{
	int deviceCount;
	checkCudaErrors(cudaGetDeviceCount(&deviceCount));

	if (deviceCount == 0)
	{
		std::cerr << "CUDA error: no devices supporting CUDA." << std::endl;
		exit(EXIT_FAILURE);
	}

	int dev = findCudaDevice(argc, argv);

	cudaDeviceProp deviceProp;
	cudaGetDeviceProperties(&deviceProp, dev);
	std::cerr << "cudaSetDevice GPU" << dev << " = " << deviceProp.name << std::endl;

	checkCudaErrors(cudaSetDevice(dev));

	return dev;
}
bool printfNPPinfo(int argc, char *argv[])
{
	const NppLibraryVersion *libVer   = nppGetLibVersion();

	printf("NPP Library Version %d.%d.%d\n", libVer->major, libVer->minor, libVer->build);

	int driverVersion, runtimeVersion;
	cudaDriverGetVersion(&driverVersion);
	cudaRuntimeGetVersion(&runtimeVersion);

	printf("  CUDA Driver  Version: %d.%d\n", driverVersion/1000, (driverVersion%100)/10);
	printf("  CUDA Runtime Version: %d.%d\n", runtimeVersion/1000, (runtimeVersion%100)/10);

	// Min spec is SM 1.0 devices
	bool bVal = checkCudaCapabilities(1, 0);
	return bVal;
}
//////////
std::string getFileFullNameFromFullPath(const std::string& s)
{
	char sep='/' ;
	size_t l=s.rfind(sep,s.length()) ;
	if (l!=std::string::npos)
		return (s.substr(l+1,s.length()-l)) ;
	else
		return "" ;
}
bool splitFileNameAndExtension(const std::string& s, std::string& just_name,std::string& ext)
{
	char sep='.' ;
	size_t l=s.rfind(sep,s.length()) ;
	if (l==std::string::npos)
		return false ;
	just_name = s.substr(0,l) ;
	ext = s.substr(l,s.length()-l) ;
	return true ;
}
std::string getPathFromFullPath(const std::string& s)
{
	char sep='/' ;
	size_t l=s.rfind(sep,s.length()) ;
	if (l!=std::string::npos)
		return (s.substr(0,l-1)) ;
	else
		return "" ;
}
void append2FileName(std::string& s, const std::string& postfix)
{
	char sep='.' ;
	size_t l=s.rfind(sep,s.length()) ;
	if (l!=std::string::npos)
		s.insert(l,postfix) ;
	else
		s="" ; // error
}

bool dirExists(const std::string dirName)
{
	struct stat info;
	if(stat( dirName.c_str(), &info ) != 0)
		return 0;
	else if(info.st_mode & S_IFDIR)
		return 1;
	else
		return 0;
}
bool createFolderOrDie(const std::string dirName)
{
#ifndef _WIN32
	if(dirExists(dirName))
		return 1 ;
	int err = mkdir(dirName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (err == -1 )
	{
		printf("Error creating directory!n");
		return 0 ;
	}
	else
		return 1 ;
#else
	return 0;
#endif
}


long myClock::getTime()
{
#ifdef _WIN32
	return clock();
#else
	timeval tmp;
	gettimeofday(&tmp, NULL);
	return tmp.tv_sec * 1000 + tmp.tv_usec / 1000;
#endif
}

void myClock::tic()
{
	active = true;
	tv_start = getTime();
}
double myClock::toc()
{
	if (active) // && !paused
	{
		active = false;
		tv_end = getTime();
		elapsed_time = (tv_end - tv_start) - clock_exclude_total;
		return elapsed_time;
	}
	else {
		printf("\nWarning: calling toc() in myClock is violated!\n");
		return -1;
	}
}
//void myClock::reset()
//{
//	tic();
//}
double myClock::getElapsedTime()
{

	if (active) {
		if (paused) {
			resume();
			toc();
		}
		else
		{
			printf("\nWarning: Calling getElapsedTime() in myClock is violated!\n");
			return -1;
		}
	}
	return elapsed_time;
}
myClock::myClock()
{
	active = false;
	paused = false;
	clock_exclude_total = 0;
}
void myClock::pause()
{
	if (!active || paused)
	{
		printf("\nWarning: Calling pause() in myClock is violated!\n");
		return;
	}
	else
	{
		paused = true;
		clock_exclude_start = getTime();
	}
}
// resume the pause and return the total elapsed time while paused
double myClock::resume()
{
	if (!active || !paused)
	{
		printf("\nWarning: Calling resume() in myClock is violated!\n");
		return -1;
	}
	else
	{
		paused = false;
		clock_exclude_end = getTime();
		clock_exclude_total += (clock_exclude_end - clock_exclude_start);
		return clock_exclude_total;
	}
}
double myClock::getTotalPausedDuration()
{
	if (!active || paused)
	{
		printf("\nWarning: Calling getTotalPausedDuration() in myClock is violated!\n");
		return -1;
	}
	else
	{
		return clock_exclude_total;
	}
}

void myClock::print(std::string message)
{
	printf("--- Elapsed time (%s) in ms: %f\n", message, getElapsedTime());
}