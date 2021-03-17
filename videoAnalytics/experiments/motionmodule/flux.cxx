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

#include "flux.h"
#include "filelisting_RP.h"

//#include "npp/ImagesCPU.h"
//#include "npp/ImageIO.h"
#include "nvtx.h"

fluxFilters fluxFilter;
Image32f  * imgNpp_32f_util1 ;


/*
 * FluxTensor is a class which contains flux tensor operations and needed data
 */

FluxTensor::FluxTensor()
{
}
FluxTensor::~FluxTensor()
{
    if (!initialized_)
        return ;
    flux_trace_->Free();
    flux_trace_norm_->Free();
    st_trace_->Free();
    st_trace_norm_->Free();
    static_->Free();
    flux_trace_mask_->Free();
    capture_buffer_grey_->Free();
    capture_buffer_color_->Free();

    delete frames_ ;

    util_4temporalDerivativeTrace->Free();
    util_4Tensor2Static_->Free();

    Mxt->Free();
    Myt->Free();
    Mtt->Free();
    Mt->Free();
    Mtrace->Free();
    MtraceST->Free();

    // detection vars
    BG_->Free();
    BG80percent_->Free();
    BGmask_->Free();
    E_sum_head_->Free();
    E_sum_tail_->Free();
    I_sum_head_->Free();
    I_sum_tail_->Free();
    I_mask_->Free();
    E_mask_->Free();
    E_->Free();
    I_->Free();
    E_pm_->Free();
    I_pm_->Free();
    E2d_->Free();
    D_->Free();
    D2_->Free();

    delete helper_ ;

    cudaFree(filter_->ts_devKernel) ;
    cudaFree(filter_->xs_devKernel) ;
    cudaFree(filter_->xd1_devKernel) ;
    cudaFree(filter_->td1_devKernel) ;
    cudaFree(filter_->td2_devKernel) ;
    cudaFree(filter_->xavg_devKernel) ;
    cudaFree(filter_->tavg_devKernel) ;
}
void FluxTensor::Reset()
{
    BG_->set(0);
    E_sum_head_->set(0);
    E_sum_tail_->set(0);
    I_sum_head_->set(0);
    I_sum_tail_->set(0);
    I_mask_->set(0);
    E_mask_->set(0);
    E_->set(0);
    I_->set(0);
    E_pm_->set(0);
    I_pm_->set(0);
    frame_counter_ = 0 ;
    start_frame_ = 0 ;
    first_detection_entry_ = true ;
}


/*
 * To initialize the flux tensor class.
 * Inputs: img_width and img_height: image dimensions
 * Temporal buffer size of flux tensor: capacity
 * Pointer to a filter bank
 */
void FluxTensor::Init(int img_width, int img_height, int capacity, fluxFilters* fluxFilter, BG_model_pars bg_pars,bool run_detection, bool colorDetection, bool debug, std::string debug_outpath)
{
    initialized_ = true ;
    debug_ = debug ;
    debug_outpath_=debug_outpath ;
    run_detection_=run_detection ;
    detection_channels_ = colorDetection ? 3 : 1 ;
    savedState_ = false ;

    capacity_ = capacity ;
    img_width_ = img_width  ;
    img_height_ = img_height ;
    flux_trace_ = new Image32f(img_width_,img_height_) ;
    flux_trace_norm_ = new Image32f(img_width_,img_height_) ;
    st_trace_ = new Image32f(img_width_,img_height_) ;
    st_trace_norm_ = new Image32f(img_width_,img_height_) ;
    static_ = new Image32f(img_width_,img_height_) ;
    flux_trace_mask_ = new Image32f(img_width_,img_height_) ;
    capture_buffer_grey_ = new Image32f(img_width_,img_height_) ;
    capture_buffer_color_ = new Image32f(img_width_,img_height_,detection_channels_) ;

    // ----  detection params -----------------------

    //    start_frame1_ = 60 ; // frame to start considering as background
    //    end_frame1_ = 100 ; //  frame to stop considering as background
    //    start_frame2_ = 101 ; // frame to start considering as foreground
    //    //end_frame2_ =  ;  // frame to stop considering as foreground.
    //    detection_time_buffer_size_ = 100 ; // number of frames between tail and head

    start_frame1_ = bg_pars.learning_start_frame ; // frame to start considering as background
    end_frame1_ = bg_pars.learning_end_frame ; //  frame to stop considering as background
    start_frame2_ = bg_pars.detection_start_frame ; // frame to start considering as foreground
    detection_time_buffer_size_ = bg_pars.detection_time_buffer_size_ ; // number of frames between tail and head

    BG_ = new Image32f(img_width_,img_height_) ;
    BG80percent_ = new Image32f(img_width_,img_height_) ;
    BGmask_ = new Image32f(img_width_,img_height_) ;
    E_sum_head_ = new Image32f(img_width_,img_height_) ;
    E_sum_tail_ = new Image32f(img_width_,img_height_) ;
    I_sum_head_ = new Image32f(img_width_,img_height_,detection_channels_) ;
    I_sum_tail_ = new Image32f(img_width_,img_height_,detection_channels_) ;
    I_mask_ = new Image32f(img_width_,img_height_) ;
    E_mask_ = new Image32f(img_width_,img_height_) ;
    E_ = new Image32f(img_width_,img_height_) ;
    I_ = new Image32f(img_width_,img_height_,detection_channels_) ;
    E_pm_ = new Image32f(img_width_,img_height_) ;
    I_pm_ = new Image32f(img_width_,img_height_) ;

    //lstD_.Init(detection_time_buffer_size_,img_width_,img_height_,detection_channels_) ;
    lstEi_.Init(detection_time_buffer_size_,img_width_,img_height_,detection_channels_) ;
    if(detection_channels_==3)
        lstIm_.Init(detection_time_buffer_size_,img_width_,img_height_,detection_channels_) ;
    E2d_ = new Image32f(img_width_,img_height_) ;
    D_ = new Image32f(img_width_,img_height_) ;
    D2_ = new Image32f(img_width_,img_height_,detection_channels_) ;
    //-------------------------------------------------------

    //Ix_ = new Image32f *[capacity_] ;
    Ix_List_.Init(capacity_,img_width_,img_height_) ;
    Iy_List_.Init(capacity_,img_width_,img_height_) ;
    Is_List_.Init(capacity_,img_width_,img_height_) ;
    trace_sumxy_List_.Init(capacity_,img_width_,img_height_) ;
    traceST_sumxy_List_.Init(capacity_,img_width_,img_height_) ;

    frames_ = new int[capacity_] ;

    Mxt = new Image32f( img_width_, img_height_) ;
    Myt = new Image32f( img_width_, img_height_) ;
    Mtt = new Image32f( img_width_, img_height_) ;
    Mt=  new Image32f( img_width_, img_height_) ;
    Mtrace= new Image32f( img_width_, img_height_) ;
    MtraceST =new Image32f( img_width_, img_height_) ;
    util_4temporalDerivativeTrace =new Image32f( img_width_, img_height_) ;
    util_4Tensor2Static_ =new Image32f( img_width_, img_height_) ;

    filter_ = fluxFilter ;

//#ifdef USE_UNIFIEDMEM
//#define cudaMemCpyType2 cudaMemcpyDefault
//#else
//#define cudaMemCpyType2 cudaMemcpyHostToDevice
//#endif
    //ts
    cudaMemErrChk(cudaMallocV2((void**)&(filter_->ts_devKernel), filter_->ts_size * sizeof(Npp32f))) ;
    NPP_CHECK_CUDA(cudaMemcpy(filter_->ts_devKernel, filter_->ts , filter_->ts_size  * sizeof(Npp32f), cudaMemCpyType_H2D));
    //xs
    cudaMemErrChk(cudaMallocV2((void**)&(filter_->xs_devKernel), filter_->xs_size * sizeof(Npp32f)));
    NPP_CHECK_CUDA(cudaMemcpy(filter_->xs_devKernel, filter_->xs , filter_->xs_size  * sizeof(Npp32f), cudaMemCpyType_H2D));
    //xd1
    cudaMemErrChk(cudaMallocV2((void**)&(filter_->xd1_devKernel), filter_->xd1_size * sizeof(Npp32f)));
    NPP_CHECK_CUDA(cudaMemcpy(filter_->xd1_devKernel, filter_->xd1 , filter_->xd1_size  * sizeof(Npp32f), cudaMemCpyType_H2D));
    //td1
    cudaMemErrChk(cudaMallocV2((void**)&(filter_->td1_devKernel), filter_->td1_size * sizeof(Npp32f)));
    NPP_CHECK_CUDA(cudaMemcpy(filter_->td1_devKernel, filter_->td1 , filter_->td1_size  * sizeof(Npp32f), cudaMemCpyType_H2D));
    //td2
    cudaMemErrChk(cudaMallocV2((void**)&(filter_->td2_devKernel), filter_->td2_size * sizeof(Npp32f)));
    NPP_CHECK_CUDA(cudaMemcpy(filter_->td2_devKernel, filter_->td2 , filter_->td2_size  * sizeof(Npp32f), cudaMemCpyType_H2D));
    //xavg
    cudaMemErrChk(cudaMallocV2((void**)&(filter_->xavg_devKernel), filter_->xavg_size * sizeof(Npp32f)));
    NPP_CHECK_CUDA(cudaMemcpy(filter_->xavg_devKernel, filter_->xavg , filter_->xavg_size  * sizeof(Npp32f), cudaMemCpyType_H2D));
    // tavg
    cudaMemErrChk(cudaMallocV2((void**)&(filter_->tavg_devKernel), filter_->tavg_size * sizeof(Npp32f)));
    NPP_CHECK_CUDA(cudaMemcpy(filter_->tavg_devKernel, filter_->tavg , filter_->tavg_size  * sizeof(Npp32f), cudaMemCpyType_H2D));

    //printf("Before...\n") ; //hadl
    helper_ = new FluxHelper(img_width_,img_height_,debug_,debug_outpath_) ;
    //printf("After...\n") ; //hadl

    nt_deriv_ = fluxFilter->xd1_size ;
    half_nt_deriv_ = nt_deriv_/2 ;
    nt_avg_ = fluxFilter->tavg_size ;
    half_nt_avg_ = nt_avg_/2 ;

    // to set variables with some initial values
    Reset();

    ImageProp ip ;
    ip.h=img_height_ ;
    ip.w=img_width_ ;
    ip.stride=flux_trace_->pitch() ;
    FluxCudaKernelsInit(&ip , filter_->td1 , filter_->td2 , filter_->td1_size , filter_->tavg , filter_->tavg_size ) ;
    cudaDeviceSynchronize() ;
}

/*
 * returns the set image dimension
 */
NppiSize FluxTensor::getImgSize()
{
    NppiSize oSize = {img_width_,img_height_};
    return oSize ;
}
/*
 * Inserts computed spatial derivatives Ix, Iy and smoothed Is related to a frame into the class temporal buffer
 * Inputs: Ix, Ix and Is. frame: frame number
 * No output
 */
void FluxTensor::insert(int frame)
{
    int nextPos = Ix_List_.getNextAvailablePos() ;
    Ix_List_.insert() ;
    Iy_List_.insert() ;
    Is_List_.insert() ;
    frames_[nextPos] = frame ;
}
/*
 * returns pointers of Ix, Iy and Is (spatial derivatives) corresponding to a location from the buffer
 * loc: location as input
 * Ix, Iy and Is: outputs
 */
int FluxTensor::getIxIyIs(Image32f **Ix, Image32f **Iy, Image32f **Is, int loc)
{
    assert(loc<capacity_ && loc>=0) ;
    //*Ix = Ix_[loc] ;
    Ix_List_.getImage(Ix,loc) ;
    Iy_List_.getImage(Iy,loc) ;
    Is_List_.getImage(Is,loc) ;
	return 0;
}
// seeks for a frame in the frames_ and return position
int FluxTensor::seekFrameID(int fr)
{
    int loc=-1 ;
    for(int i=0 ; i<capacity_ ; i++)
        if(frames_[i] == fr) {
            loc=i ;
            break ;
        }
    //    if( loc==-1)
    //    {
    //         fprintf(stderr,"error: 'loc' is invalid!\n") ;
    ////         exit(-1) ;
    //    }

    return loc ;
}

bool FluxTensor::temporalDerivativeTrace_V2(int fr)
{
    int nt_deriv = filter_->xd1_size ;
    int half_nt_deriv = nt_deriv/2 ;
    int loc , i , fr_index , masterFrameInList;
    Image32f *Ix, *Iy, *Is ;

    //Image32f * tmp = util_4temporalDerivativeTrace ;
    //NppiSize oSize = {img_width_,img_height_};

    fr_index = seekFrameID(fr) ;

    float **IxData ;
    IxData= new float*[nt_deriv] ;

    float **IyData ;
    IyData= new float*[nt_deriv] ;

    float **IsData ;
    IsData= new float*[nt_deriv] ;

    //-----------------------------------------------------------------------------------
    masterFrameInList=-1 ;
    for(i=0 ; i<nt_deriv ; i++) {
        int t = fr - half_nt_deriv + i  ; //-1
        loc = seekFrameID(t) ;
        if(loc<0)
            return false ;
        if(loc==fr_index)
            masterFrameInList = i ;
        getIxIyIs(&Ix,&Iy,&Is, loc) ;
        IxData[i] = Ix->data() ;
        IyData[i] = Iy->data() ;
        IsData[i] = Is->data() ;
    }
    if(masterFrameInList<0) {
        fprintf(stderr,"warning! Master frame not found in temporal AVG. Skipping ...\n") ;
        return false;
    }

    //cudaDeviceSynchronize() ;


    FluxDerivByKernel(Ix->width() , Ix->height() , Ix->pitch() , masterFrameInList ,
                      IxData , IyData , IsData ,
                      Mxt->data() , Myt->data(), Mt->data(), Mtt->data() , Mtrace->data() ,  MtraceST->data() ) ;
    //cudaDeviceSynchronize() ;

    if(0) {
        float *aa=IxData[0] ;
        float val=0 ;
        cudaMemcpy(&val,aa,sizeof(float),cudaMemcpyDeviceToHost) ;
        fprintf(stderr,"IxData[0](0,0) is: %f\n",val) ;
    }

    if(0) {
        getHelper()->imWrite("/home/ubuntu/Downloads/Ix.png",Ix) ;
        getHelper()->imWrite("/home/ubuntu/Downloads/Mxt.png",Mxt) ;
        getHelper()->imWrite("/home/ubuntu/Downloads/Myt.png",Myt) ;
        getHelper()->imWrite("/home/ubuntu/Downloads/Mt.png",Mt) ;
        getHelper()->imWrite("/home/ubuntu/Downloads/Mtt.png",Mtt) ;
        getHelper()->imWrite("/home/ubuntu/Downloads/Mtrace.png",Mtrace) ;
        getHelper()->imWrite("/home/ubuntu/Downloads/MtraceST.png",MtraceST) ;
    }

    if(0) {
        double min1 = getHelper()->getMinInImage_32f(Mt) ;
        double max1 = getHelper()->getMaxInImage_32f(Mt) ;
        fprintf(stderr,"min and max of Mt are: %f, %f\n",min1, max1) ;

        double max2 = getHelper()->getMaxInImage_32f(Is) ;
        fprintf(stderr,"max of Is is: %f\n",max2) ;
    }

    //std::cout << "test\n" ;
    Image32f * Mtrace_avg = getTraceSumXY(fr_index) ;
    //nppiSet_32f_C1R (0, Mtrace_avg->data(), Mtrace_avg->pitch(), oSize) ; 	// set to zero
    Mtrace_avg->set(0); // set to zero
    getHelper()->conv_32f( Mtrace , Mtrace_avg ,
                           filter_->xavg_devKernel , filter_->xavg_size , filter_->xavg_anchor ,
                           filter_->xavg_devKernel , filter_->xavg_size , filter_->xavg_anchor ) ;

    Image32f * MtraceST_avg = getTraceSTSumXY(fr_index) ;
    getHelper()->conv_32f( MtraceST , MtraceST_avg ,
                           filter_->xavg_devKernel , filter_->xavg_size , filter_->xavg_anchor ,
                           filter_->xavg_devKernel , filter_->xavg_size , filter_->xavg_anchor ) ;

    delete[] IxData ;
    delete[] IyData ;
    delete[] IsData ;
}

/*
 * Trace of temporal derivatives in the buffer
 * Input: fr as frame number
 * Outputs: will be stored internally
 */
bool FluxTensor::temporalDerivativeTrace(int fr)
{
    int nt_deriv = filter_->xd1_size ;
    int half_nt_deriv = nt_deriv/2 ;
    int loc , i , fr_index ;
    Image32f *Ix, *Iy, *Is ;
    Image32f * tmp = util_4temporalDerivativeTrace ;
    NppiSize oSize = {img_width_,img_height_};

    NVTX_PUSHSync("fl_TD_Set0",0);
    // set to zero
    Mxt->set(0);
    Myt->set(0);
    Mtt->set(0);
    Mt->set(0);
    NVTX_POPSync ;

    fr_index = seekFrameID(fr) ;

    NVTX_PUSHSync("fl_TD_M",0);
    for(i=0 ; i<nt_deriv ; i++) {
        int t = fr - half_nt_deriv + i  ; //-1
        loc = seekFrameID(t) ;
        if(loc<0)
            return false ;
        //Npp32f alpha= filter_->td1[i] ;
        getIxIyIs(&Ix,&Iy,&Is, loc) ;

        //		Mxt=Mxt+filter.td1(i)*W.Ix(:,:,t_index);
        nppiMulC_32f_C1R (  Ix->data(), Ix->pitch() , filter_->td1[i] ,
                            tmp->data() , tmp->pitch(), oSize) ;
        nppiAdd_32f_C1IR( tmp->data() , tmp->pitch(),  Mxt->data() , Mxt->pitch() , oSize) ;

        //		Myt=Myt+filter.td1(i)*W.Iy(:,:,t_index);
        nppiMulC_32f_C1R (  Iy->data(), Iy->pitch() , filter_->td1[i] ,
                            tmp->data() , tmp->pitch(), oSize) ;
        nppiAdd_32f_C1IR( tmp->data() , tmp->pitch(),  Myt->data() , Myt->pitch() , oSize) ;

        //		Mtt=Mtt+filter.td2(i)*W.Is(:,:,t_index);
        nppiMulC_32f_C1R (  Is->data(), Is->pitch() , filter_->td2[i] ,
                            tmp->data() , tmp->pitch(), oSize) ;
        nppiAdd_32f_C1IR( tmp->data() , tmp->pitch(),  Mtt->data() , Mtt->pitch() , oSize) ;

        //		Mt=Mt+filter.td1(i)*W.Is(:,:,t_index);
        nppiMulC_32f_C1R (  Is->data(), Is->pitch() , filter_->td1[i] ,
                            tmp->data() , tmp->pitch(), oSize) ;
        nppiAdd_32f_C1IR( tmp->data() , tmp->pitch(),  Mt->data() , Mt->pitch() , oSize) ;
    }
    NVTX_POPSync ;

    //if(flag_debug) {
    if(0) {
        double min1 = getHelper()->getMinInImage_32f(Mt) ;
        double max1 = getHelper()->getMaxInImage_32f(Mt) ;
        fprintf(stderr,"min and max of Mt are: %f, %f\n",min1, max1) ;
        getHelper()->imWrite("/media/ubuntu/SataHDD1/data/16-02-26-1422-pos3_D_small/output/Mxt_raw.png", Mxt) ;
        getHelper()->imWrite("/media/ubuntu/SataHDD1/data/16-02-26-1422-pos3_D_small/output/Myt_raw.png", Myt) ;
        getHelper()->imWrite("/media/ubuntu/SataHDD1/data/16-02-26-1422-pos3_D_small/output/Mtt_raw.png", Mtt) ;
        getHelper()->imWrite("/media/ubuntu/SataHDD1/data/16-02-26-1422-pos3_D_small/output/Mt_raw.png", Mt) ;
    }

    nppiSet_32f_C1R (0, Mtrace->data(), Mtrace->pitch(), oSize) ; 	// set to zero


    NVTX_PUSHSync("fl_TD_FT",0);
    //----- Flux --------
    //                 Mtrace=sqrt(Mxt->*Mxt+Myt->*Myt+Mtt->*Mtt);
    nppiAddSquare_32f_C1IR ( Mxt->data() , Mxt->pitch() , Mtrace->data(), Mtrace->pitch() , oSize ) ;
    nppiAddSquare_32f_C1IR ( Myt->data() , Myt->pitch() , Mtrace->data(), Mtrace->pitch() , oSize ) ;
    nppiAddSquare_32f_C1IR ( Mtt->data() , Mtt->pitch() , Mtrace->data(), Mtrace->pitch() , oSize ) ;
    nppiSqrt_32f_C1IR(Mtrace->data(), Mtrace->pitch() , oSize ) ;

    // 					Mtrace_avg=conv2(filter.xavg',filter.xavg,Mtrace,'same');

    Image32f * Mtrace_avg = getTraceSumXY(fr_index) ;
    nppiSet_32f_C1R (0, Mtrace_avg->data(), Mtrace_avg->pitch(), oSize) ; 	// set to zero

    getHelper()->conv_32f( Mtrace , Mtrace_avg ,
                           filter_->xavg_devKernel , filter_->xavg_size , filter_->xavg_anchor ,
                           filter_->xavg_devKernel , filter_->xavg_size , filter_->xavg_anchor ) ;
    NVTX_POPSync

            //	//----- 3D structure tensor --------
            NVTX_PUSHSync("fl_TD_ST",0);
    nppiSet_32f_C1R (0, MtraceST->data(), MtraceST->pitch(), oSize) ; 	// set to zero

    getIxIyIs(&Ix,&Iy,&Is, fr_index) ;

    nppiAddSquare_32f_C1IR ( Ix->data() , Ix->pitch() , MtraceST->data(), MtraceST->pitch() , oSize ) ;
    nppiAddSquare_32f_C1IR ( Iy->data() , Iy->pitch() , MtraceST->data(), MtraceST->pitch() , oSize ) ;
    nppiAddSquare_32f_C1IR ( Mt->data() , Mt->pitch() , MtraceST->data(), MtraceST->pitch() , oSize ) ;
    nppiSqrt_32f_C1IR(MtraceST->data(), MtraceST->pitch() , oSize ) ;

    //loc = getNextAvailablePos() ;
    //loc = traceST_sumxy_List_.getNextAvailablePos() ;
    //Image32f * MtraceST_avg = new Image32f(img_width_,img_height_) ;
    Image32f * MtraceST_avg = getTraceSTSumXY(fr_index) ;
    getHelper()->conv_32f( MtraceST , MtraceST_avg ,
                           filter_->xavg_devKernel , filter_->xavg_size , filter_->xavg_anchor ,
                           filter_->xavg_devKernel , filter_->xavg_size , filter_->xavg_anchor ) ;

    NVTX_POPSync

            //setTraceSTSumXY(MtraceST_avg,fr_index) ;

            //////////////////////
            //	nppiFree(Mxt.data());
            //	nppiFree(Myt.data());
            //	nppiFree(Mtt.data());
            //	nppiFree(Mt.data());
            //	nppiFree(Mtrace.data());
            //	nppiFree(MtraceST.data());
            //////////////////////
            return true ;
}
/*
 * Assign an already computed trace temporal summation of Flux Tensor in a proper location
 * current_trace_sumxy: input, pointer to the computed trace temporal summation
 * fr_index: the place in which 'current_trace_sumxy' will be assigned
 */
//void FluxTensor::setTraceSumXY(Image32f * current_trace_sumxy, int fr_index)
//{
//	trace_sumxy_[fr_index] = current_trace_sumxy ;
//}
/*
 * returns an already computed trace temporal summation of Flux Tensor corresponding to frame 'fr_index'
 */
Image32f *  FluxTensor::getTraceSumXY(int fr_index)
{
    //return trace_sumxy_[fr_index]  ;
    return trace_sumxy_List_.getImage(fr_index) ;
}
/*
 * Assign an already computed trace temporal summation of Structure Tensor in a proper location
 * current_trace_sumxy: input, pointer to the computed trace temporal summation
 * fr_index: the place in which 'current_trace_sumxy' will be assigned
 */
//void FluxTensor::setTraceSTSumXY(Image32f * current_traceST_sumxy, int fr_index)
//{
//	traceST_sumxy_[fr_index] = current_traceST_sumxy ;
//}
/*
 * returns an already computed trace temporal summation of Structure Tensor corresponding to frame 'fr_index'
 */
Image32f *  FluxTensor::getTraceSTSumXY(int fr_index)
{
    //return traceST_sumxy_[fr_index]  ;
    return traceST_sumxy_List_.getImage(fr_index) ;
}
/*
 * returns flux tensor trace
 */
Image32f *  FluxTensor::getFluxTrace()
{
    return flux_trace_ ;
}
/*
 * returns normalized flux tensor trace
 */
Image32f *  FluxTensor::getFluxTraceNorm()
{
    return flux_trace_norm_ ;
}
/*
 * returns flux trace normalized image
 * output: img_Mat8u as a 8bit unsigned opencv Mat
 */
cv::Mat FluxTensor::getFluxTraceNorm_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getFluxTraceNorm(),255) ;
    return img_Mat8u ;
}
/*
 * returns flux trace mask image
 * output: img_Mat8u as a 8bit unsigned opencv Mat
 */
cv::Mat FluxTensor::getFluxTraceMask_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getFluxTraceMask(),255) ;
    return img_Mat8u ;
}



float FluxTensor::getFluxTraceAvg()
{
    // allocate device buffer for cuda sum
    int bufferSize ;
    Npp8u* buffer ;
    nppiSumGetBufferHostSize_32f_C1R(getImgSize(),&bufferSize) ;
    cudaMemErrChk(cudaMalloc((void **)&buffer,bufferSize)) ;

    // allocate sum on device
    Npp64f* sum ;
    cudaMemErrChk(cudaMallocV2(&sum, sizeof(Npp64f)));

    // get result into host mem from cuda
    nppiSum_32f_C1R(DP(getFluxTraceNorm()),getImgSize(), buffer ,sum) ;
    Npp64f sum_ret ;
    cudaMemcpy(&sum_ret, sum, sizeof(Npp64f), cudaMemCpyType_D2H);


    cudaFree(sum) ;
    cudaFree(buffer) ;

    return (Npp32f)sum_ret/(getImgSize().height * getImgSize().width ) ;
}
float FluxTensor::getStaticTraceAvg()
{
    // allocate device buffer for cuda sum
    int bufferSize ;
    Npp8u* buffer ;
    nppiSumGetBufferHostSize_32f_C1R(getImgSize(),&bufferSize) ;
    cudaMemErrChk(cudaMalloc((void **)&buffer,bufferSize)) ;

    // allocate sum on device
    Npp64f* sum ;
    cudaMemErrChk(cudaMallocV2(&sum, sizeof(Npp64f)));

    // get result into host mem from cuda
    nppiSum_32f_C1R(DP(getSTTraceNorm()),getImgSize(), buffer ,sum) ;
    Npp64f sum_ret ;
    cudaMemcpy(&sum_ret, sum, sizeof(Npp64f), cudaMemCpyType_D2H);


    cudaFree(sum) ;
    cudaFree(buffer) ;

    return (Npp32f)sum_ret/(getImgSize().height * getImgSize().width ) ;
}

/*
 * returns structure tensor trace normalized image
 * output: img_Mat8u as a 8bit unsigned opencv Mat
 */
cv::Mat FluxTensor::getSTTraceNorm_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getSTTraceNorm(),255) ;
    return img_Mat8u ;
}
/*
 * returns the static image
 * output: img_Mat8u as a 8bit unsigned opencv Mat
 */
cv::Mat FluxTensor::getStatic_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getStatic(),255) ;
    return img_Mat8u ;
}
/*
 * returns the staticModel image
 * output: img_Mat8u as a 8bit unsigned opencv Mat
 */
cv::Mat FluxTensor::getStaticModel_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(E2d_,255) ;
    return img_Mat8u ;
}

/*
 * returns structure tensor trace
 */
Image32f *  FluxTensor::getSTTrace()
{
    return st_trace_ ;
}
/*
 * returns normalized structure tensor trace
 */
Image32f *  FluxTensor::getSTTraceNorm()
{
    return st_trace_norm_ ;
}
/*
 * returns the change (used in detection)
 * output: img_Mat8u as a 8bit unsigned opencv Mat
 */
//cv::Mat FluxTensor::getChange_Mat()
//{
//    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getChange(),255) ;
//    return img_Mat8u ;
//}
/*
 * returns the persistent image (used in detection)
 * output: img_Mat8u as a 8bit unsigned opencv Mat
 */
cv::Mat FluxTensor::getPersistent_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getPersistent(),255) ;
    return img_Mat8u ;
}
/*
 * returns the persistent image computed from color version (used in detection)
 * output: img_Mat8u as a 8bit unsigned opencv Mat
 */
cv::Mat FluxTensor::getPersistentColor_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getPersistentColor(),255) ;
    //DDD
    //cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(this->D_,255) ;
    return img_Mat8u ;
}
cv::Mat FluxTensor::getPersistentMask_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getPersistentMask(),255) ;
    return img_Mat8u ;
}
cv::Mat FluxTensor::getPersistentColorMask_Mat()
{
    cv::Mat img_Mat8u = getHelper()->getMat8u_from_Npp32f(getPersistentColorMask(),255) ;
    return img_Mat8u ;
}


void FluxTensor::TemporalAvg_V2( int fr)
{
    int loc ,i ;
    static int nt, nx, half_nt ;
    static bool first=true ;
    if(first)
    {
        first = false ;
        for(i=0 , nt=0 ; i<filter_->tavg_size ; i++)
            nt += filter_->tavg[i] ;
        for(i=0 , nx=0 ; i<filter_->xavg_size ; i++)
            nx += filter_->xavg[i] ;
        half_nt = nt/2 ;
    }

    loc = seekFrameID(fr) ;
    int& fr_index = loc ;

    float **TraceSumXY_Data ;
    TraceSumXY_Data= new float*[filter_->tavg_size] ;

    float **TraceSTSumXY_Data ;
    TraceSTSumXY_Data= new float*[filter_->tavg_size] ;

    //----add trace values within the temporal window centered at frame=fr
    for(i=0 ; i<filter_->tavg_size ; i++)
    {
        //---locate the frame of interest in W
#if 0
        int t = fr - half_nt + i  ;
#else
        int t = fr + i  ;
#endif
        loc = seekFrameID(t) ;
        if(loc<0)
            return  ;
        int& t_index = loc ;

        TraceSumXY_Data[i] = getTraceSumXY(t_index)->data() ;
        TraceSTSumXY_Data[i] = getTraceSTSumXY(t_index)->data() ;
    }

    float denominator = nx * nx * nt  ;

    FluxTempAVGByKernel(getSTTrace()->width() , getSTTrace()->height() , getSTTrace()->pitch() ,
                        TraceSumXY_Data , TraceSTSumXY_Data ,
                        getFluxTraceNorm()->data() , getSTTraceNorm()->data() , denominator) ;

    if(0)
    {
        fprintf(stderr,"values range of 'FluxTracNorm' is between %f and %f\n", getHelper()->getMinInImage_32f(getFluxTraceNorm()),getHelper()->getMaxInImage_32f(getFluxTraceNorm())) ;
        fprintf(stderr,"values range of 'getSTTraceNorm' is between %f and %f\n", getHelper()->getMinInImage_32f(getSTTraceNorm()),getHelper()->getMaxInImage_32f(getSTTraceNorm())) ;
    }
}



//------------------------------------------------
/*
 * gievn a middle frame 'fr', it performs temporal averaging.
 * The results are stored in appropriate internal variables.
 * They include flux and structure traces and their normalized versions
 */
void FluxTensor::TemporalAvg( int fr)
{
    int loc ,i ;
    static Npp32f nt, nx, half_nt ;
    static bool first=true ;
    if(first)
    {
        first = false ;
        for(i=0 , nt=0 ; i<filter_->tavg_size ; i++)
            nt += filter_->tavg[i] ;
        for(i=0 , nx=0 ; i<filter_->xavg_size ; i++)
            nx += filter_->xavg[i] ;
        half_nt = nt/2 ;
    }
    loc = seekFrameID(fr) ;
    int& fr_index = loc ;

    //NppiSize oSize = {img_width_,img_height_};

    Image32f * flux_trace_iter ;
    Image32f * st_trace_iter ;

    nppiSet_32f_C1R (0, getFluxTrace()->data(), getFluxTrace()->pitch(), getImgSize()) ; // set to zero
    nppiSet_32f_C1R (0, getSTTrace()->data(), getSTTrace()->pitch(), getImgSize() ) ; // set to zero

    //----add trace values within the temporal window centered at frame=fr
    for(i=0 ; i<nt ; i++)
    {
        //---locate the frame of interest in W
#if 0
        int t = fr - half_nt + i  ;
#else
        int t = fr + i  ;
#endif
        loc = seekFrameID(t) ;
        int& t_index = loc ;

        if(debug_)
        {
            fprintf(stderr,"----------------- values range of 'getTraceSumXY (before norm, before temporal avg) for frame# %d is between %f and %f\n",t, getHelper()->getMinInImage_32f(getTraceSumXY(t_index)),getHelper()->getMaxInImage_32f(getTraceSumXY(t_index))) ;
            fprintf(stderr,"----------------- values range of 'getTraceSTSumXY  (before norm, before temporal avg) for frame# %d is between %f and %f\n",t, getHelper()->getMinInImage_32f(getTraceSTSumXY(t_index)),getHelper()->getMaxInImage_32f(getTraceSTSumXY(t_index))) ;
        }


        flux_trace_iter = getTraceSumXY(t_index) ;
        st_trace_iter = getTraceSTSumXY(t_index) ;

        //		if(flag_debug)
        //			{
        //				fprintf(stderr,"values range of 'flux_trace_iter' in t_index(%d) is between %f and %f\n",t_index, getNppConv()->getMinInImage_32f(flux_trace_iter),getNppConv()->getMaxInImage_32f(flux_trace_iter)) ;
        //				fprintf(stderr,"values range of 'st_trace_iter ' in t_index(%d) is between %f and %f\n",t_index, getNppConv()->getMinInImage_32f(st_trace_iter),getNppConv()->getMaxInImage_32f(st_trace_iter)) ;
        //			}

#if 0
        //---add its trace to flux_trace
        NPP_CHECK_NPP (
                    nppiAddWeighted_32f_C1IR( flux_trace_iter->data(), flux_trace_iter->pitch() ,
                                              getFluxTrace()->data() , getFluxTrace()->pitch() ,
                                              getImgSize() , filter_->tavg[i] )
                    ) ;
        //---add its trace to structure_trace
        NPP_CHECK_NPP (
                    nppiAddWeighted_32f_C1IR( st_trace_iter->data(), st_trace_iter->pitch() ,
                                              getSTTrace()->data() , getSTTrace()->pitch() ,
                                              getImgSize() , filter_->tavg[i] )
                    ) ;
#endif

        Image32f * tmp = util_4temporalDerivativeTrace ;

        //---add its trace to flux_trace
        nppiMulC_32f_C1R (  DP(flux_trace_iter) , filter_->tavg[i] ,
                            DP(tmp) , getImgSize()) ;
        nppiAdd_32f_C1IR( DP(tmp), DP(getFluxTrace()) , getImgSize()) ;

        //---add its trace to flux_trace
        nppiMulC_32f_C1R (  DP(st_trace_iter) , filter_->tavg[i] ,
                            DP(tmp) , getImgSize()) ;
        nppiAdd_32f_C1IR( DP(tmp), DP(getSTTrace()) , getImgSize()) ;
    }

    if(debug_)
    {
        fprintf(stderr,"values range of 'FluxTrac (before norm, after temporal avg)' is between %f and %f\n", getHelper()->getMinInImage_32f(getFluxTrace()),getHelper()->getMaxInImage_32f(getFluxTrace())) ;
        fprintf(stderr,"values range of 'getSTTrace  (before norm, after temporal avg)' is between %f and %f\n", getHelper()->getMinInImage_32f(getSTTrace()),getHelper()->getMaxInImage_32f(getSTTrace())) ;
    }

    Npp32f denominator_host = nx * nx * nt ;
    NPP_CHECK_NPP (
                //nppiDivC_32f_C1IR( denominator_host , getFluxTrace()->data() , getFluxTrace()->pitch() , getImgSize() )
                nppiDivC_32f_C1R( getFluxTrace()->data() , getFluxTrace()->pitch(), denominator_host , getFluxTraceNorm()->data() , getFluxTraceNorm()->pitch() , getImgSize() )
                ) ;
    NPP_CHECK_NPP (
                //nppiDivC_32f_C1IR( denominator_host , getSTTrace()->data() , getSTTrace()->pitch() , getImgSize() )
                nppiDivC_32f_C1R( getSTTrace()->data() , getSTTrace()->pitch() , denominator_host ,getSTTraceNorm()->data() ,getSTTraceNorm()->pitch()  , getImgSize() )
                ) ;


    if(debug_)
    {
        fprintf(stderr,"values range of 'FluxTracNorm (after normalization)' is between %f and %f\n", getHelper()->getMinInImage_32f(getFluxTraceNorm()),getHelper()->getMaxInImage_32f(getFluxTraceNorm())) ;
        fprintf(stderr,"values range of 'getSTTraceNorm  (after normalization)' is between %f and %f\n", getHelper()->getMinInImage_32f(getSTTraceNorm()),getHelper()->getMaxInImage_32f(getSTTraceNorm())) ;
    }

    if(0) {
        getHelper()->reScaleFrom0and1_to_0and255 (getFluxTraceNorm()) ;
        getHelper()->reScaleFrom0and1_to_0and255 (getSTTraceNorm()) ;
    }


    if(debug_)
    {
        fprintf(stderr,"values range of 'FluxTrac (after re-scaling)' is between %f and %f\n", getHelper()->getMinInImage_32f(getFluxTraceNorm()),getHelper()->getMaxInImage_32f(getFluxTraceNorm())) ;
        fprintf(stderr,"values range of 'getSTTrace  (after re-scaling)' is between %f and %f\n", getHelper()->getMinInImage_32f(getSTTraceNorm()),getHelper()->getMaxInImage_32f(getSTTraceNorm())) ;
    }


    //	if(flag_debug)
    //		getNppConv()->imWrite("/home/ubuntu/data/test/output/output_NormalizeTraceSrc_afterDevision.png", oDeviceSrc) ;
    //
    //		if(flag_debug)
    //			getNppConv()->imWrite("/home/ubuntu/data/test/output/output_getFluxTrace().png", getFluxTrace()) ;
    //////

#if 0 // we are not using Flux Mask anymore, removed on Aug 24, 2016
    Npp32s histHost_FluxTraceNorm[histBinCount_] ;
    NormalizeTrace(getFluxTrace() , getFluxTraceNorm(), nx , nt ) ;

    Npp32s histHost_STTraceNorm[histBinCount_] ;
    NormalizeTrace(getSTTrace() , getSTTraceNorm(), nx , nt ) ;

    if(flag_debug) {
        getNppConv()->imWrite("/home/ubuntu/data/test/output/output_FluxTraceNorm.png", getFluxTraceNorm()) ;
        getNppConv()->imWrite("/home/ubuntu/data/test/output/output_STTraceNorm.png", getSTTraceNorm()) ;
    }

    // ---- Otsu thresholding and computing mask ----
    //		L=graythresh(flux_trace_norm);
    //		mask=zeros(rows,cols);
    //		mask(flux_trace_norm>0.5*L)=1;
    const int levelCount = histBinCount_ + 1;
    Npp32s histHost[histBinCount_] ;
    Npp32f levelsHost[levelCount] ;
    getNppConv()->computeHistRange_32f(getFluxTraceNorm() , histHost , histBinCount_ , levelsHost) ;
    int TotalNumOfPixels = getFluxTraceNorm()->width() * getFluxTraceNorm()->height()  ;
    int thresh_pos = getNppConv()->OtsuThresholding( histHost , histBinCount_ ,TotalNumOfPixels) ;
    Npp32f threshValue = levelsHost[thresh_pos] ;
    const Npp32f EPSILON = 1.19209e-07 ;
    const Npp32f newMin = 0 ;
    const Npp32f newMax = 255 ;
    getFluxTraceMask()->copyFrom(getFluxTraceNorm()->data(),getFluxTraceNorm()->pitch()) ;
    NPP_CHECK_NPP (
                nppiThreshold_LTValGTVal_32f_C1IR(getFluxTraceMask()->data(),getFluxTraceMask()->pitch(), getImgSize() ,
                                                  threshValue , newMin , threshValue - EPSILON , newMax ) ;
            ) ;
#endif
}
/*
 * Obtains static by subtraction of traces of flux tensor and strcuture tensor
 * Result is internally stored
 */
void FluxTensor::Tensor2Static()
{
    Image32f *flux_dilated = util_4Tensor2Static_ ;
    //Image32f *subtr_result = new Image32f(img_width_,img_height_) ;
    Image32f  *flux_trace = getFluxTraceNorm() ;
    Image32f  *st_trace = getSTTraceNorm() ;
    Image32f  *static1 = getStatic() ;

    if(debug_) {
        fprintf(stderr,"values range of 'flux_trace' before dilation is between %f and %f\n", getHelper()->getMinInImage_32f(flux_trace),getHelper()->getMaxInImage_32f(flux_trace)) ;
    }

    getHelper()->dilate( flux_trace , flux_dilated ) ;
    //flux_trace->copyTo(DP(flux_dilated)) ;
    if(debug_) {
        fprintf(stderr,"values range of 'flux_dilated' after dilation is between %f and %f\n", getHelper()->getMinInImage_32f(flux_dilated),getHelper()->getMaxInImage_32f(flux_dilated)) ;
    }

    //	NppiSize ROI = {img_width_ , img_height_} ;
    NPP_CHECK_NPP (
                nppiSub_32f_C1R( DP(flux_dilated) , DP(st_trace) , DP(static1) , getImgSize() ) ;
            ) ;

    if(debug_)
        fprintf(stderr,"values range of 'static1' before norm is between %f and %f\n", getHelper()->getMinInImage_32f(static1),getHelper()->getMaxInImage_32f(static1)) ;

    NPP_CHECK_NPP (
                nppiThreshold_32f_C1IR (static1->data(),static1->pitch(), getImgSize() , 0.0f , NPP_CMP_LESS ) ;
            ) ;
    //	NPP_CHECK_NPP (
    //			nppiThreshold_32f_C1IR (static1->data(),static1->pitch(), getImgSize() , 1.0f , NPP_CMP_GREATER ) ;
    //	) ;
}
/*
 * It normalizes a structure or flux trace by histogram streching followed by dividing by max
 *  oDeviceSrc: pointer to the input trace image
 *  oDeviceDst: pointer to store the output
 *
 */
void FluxTensor::NormalizeTrace(Image32f *oDeviceSrc , Image32f *oDeviceDst , Npp32f nx, Npp32f nt)
{
    Npp32f denominator_host = nx * nx * nt ;
    NPP_CHECK_NPP (
                nppiDivC_32f_C1IR( denominator_host , oDeviceSrc->data(), oDeviceSrc->pitch() , getImgSize() )
                ) ;

    getHelper()->StretchHist_32f( oDeviceSrc , oDeviceDst,  0.0 , 0.995 , histBinCount_) ;

    Npp32f max_host = getHelper()->getMaxInImage_32f( oDeviceDst )  ;

    if (max_host > 1) {
        NPP_CHECK_NPP (
                    nppiDivC_32f_C1IR( max_host , oDeviceDst->data(),oDeviceDst->pitch() , getImgSize() )
                    ) ;
        NPP_CHECK_NPP (
                    nppiThreshold_32f_C1IR (oDeviceDst->data(),oDeviceDst->pitch(), getImgSize() , 1.0f , NPP_CMP_GREATER ) ;
                ) ;
        //flux_trace_norm=flux_trace_norm/max_flux;
        //flux_trace_norm(flux_trace_norm>1)=1;
    }
}
/*
 * Receives a new input frame and processes it
 * Input: img8u as pointer to an opencv  8u single channel Mat image.
 */
void FluxTensor::addFrame_Mat(cv::Mat img8u)
{
    NVTX_PUSHSync("fl_copyIn",0);
    getHelper()->getNpp32f_from_Mat8u(&img8u , capture_buffer_grey_) ;
    NVTX_POPSync;
    //cv::imwrite("/home/ubuntu/Downloads/In-addFrame_Mat.png",img8u) ;
    addFrame(capture_buffer_grey_) ;
}
void FluxTensor::addFrame_Mat(cv::Mat  img8uGrey,cv::Mat img8uColor)
{
    //cv::imwrite("/home/ubuntu/Downloads/tmpCol.png",img8uColor) ;
    //cudaDeviceSynchronize() ;
    NVTX_PUSHSync("fl_copyIn",0);
    getHelper()->getNpp32f_from_Mat8u(&img8uGrey , capture_buffer_grey_) ;
    NVTX_POPSync;

    NVTX_PUSHSync("fl_split",0);
    std::vector<cv::Mat> channels(3);
    cv::split(img8uColor, channels); // split to BGR

    if(debug_) {
        cv::imwrite("/home/ubuntu/Downloads/inputR1.png",channels[0]) ;
        cv::imwrite("/home/ubuntu/Downloads/inputG1.png",channels[1]) ;
        cv::imwrite("/home/ubuntu/Downloads/inputB1.png",channels[2]) ;
    }

    getHelper()->getNpp32f_from_Mat8u(&(channels[0]) , capture_buffer_color_,3) ;
    getHelper()->getNpp32f_from_Mat8u(&(channels[1]) , capture_buffer_color_,2) ;
    getHelper()->getNpp32f_from_Mat8u(&(channels[2]) , capture_buffer_color_,1) ;

    if(debug_) {
        getHelper()->imWrite("/home/ubuntu/Downloads/inputR2.png",capture_buffer_color_,3,1);
        getHelper()->imWrite("/home/ubuntu/Downloads/inputG2.png",capture_buffer_color_,3,2);
        getHelper()->imWrite("/home/ubuntu/Downloads/inputB2.png",capture_buffer_color_,3,3);
    }


    NVTX_POPSync;
    addFrame(capture_buffer_grey_,capture_buffer_color_) ;
}

/*
 * Receives a new input frame and processes it
 * Input: oDeviceSrc pointer to a Npp image
 */
void FluxTensor::addFrame(Image32f * inImageGrey,Image32f * inImageColor)
{
    NVTX_PUSHSync("fl_addFrame",0) ;
    int frame_id  ;
    int fr_mid_deriv ;
    int fr_mid_avg ;

    frame_counter_ ++ ;
    frame_id = frame_counter_ - 1 ;

    //NppiSize ROI = getImgSize() ;
    Image32f * Ix ;
    Image32f * Iy ;
    Image32f * Is ;

    int loc = Ix_List_.getNextAvailablePos() ;
    getIxIyIs(&Ix,&Iy,&Is,loc) ;

    // compute spatial derivatives
    //computeIxIyIs(oDeviceSrc, oDeviceDst_Ix , oDeviceDst_Iy , oDeviceDst_Is , filter_ ) ;
    NVTX_PUSHSync("ComIx",0) ;
    getHelper()->computeIxIyIs(inImageGrey, Ix , Iy , Is , filter_ ) ;
    NVTX_POPSync ; //ComIx
    insert(frame_id) ;
    // ----
    NVTX_PUSHSync("fl_temDer", 0);
    if (frame_counter_ >= nt_deriv_) {
        fr_mid_deriv = frame_id - half_nt_deriv_ ;
        //[W]=TemporalDerivativeTrace(W,filter,fr_mid_deriv);
        //temporalDerivativeTrace(fr_mid_deriv) ;
        temporalDerivativeTrace_V2(fr_mid_deriv) ;
    }
    NVTX_POPSync ; // fl_temDer
    // -----
    NVTX_PUSHSync("fl_temAvg", 0);
    if (frame_id >= nt_avg_ + half_nt_deriv_) {
        fr_mid_avg = frame_id - half_nt_avg_ - half_nt_deriv_ ;
        //[flux_trace,mask,st_trace]=TemporalAvg(W,FTparam,filter,fr_mid_avg);
        //TemporalAvg(fr_mid_avg) ;


        TemporalAvg_V2(fr_mid_avg) ;

        // added by Filiz in her new version in Oct 2016
        //nppiMulC_32f_C1IR(fcoef_,DP(getFluxTraceNorm()),getImgSize()) ;
        //nppiMulC_32f_C1IR(fcoef_,DP(getSTTraceNorm()),getImgSize()) ;
        //-------------------

        Tensor2Static() ;
    }
    NVTX_POPSync ; //fl_temAvg
    NVTX_POPSync ; //fl_addFrame

    // =====================================================================================================================
    // =====================================================================================================================
    // =====================================================================================================================
    // -- detection part ----
    // =====================================================================================================================
    // =====================================================================================================================
    // =====================================================================================================================

    if(!run_detection_ || frame_id<start_frame1_)
        return ;
    NVTX_PUSHSync("fl_detec", 0);
    if(first_detection_entry_)
    {
        fprintf(stderr,"Detection module is activated ...\n") ;
        E_count_head_ = 0 ;
        E_count_tail_ = 0 ;
        I_count_head_ = 0 ;
        I_count_tail_ = 0 ;
        first_detection_entry_=0 ;
    }

    if(detection_channels_==3)  // color processing
    {
        if(frame_id>= start_frame1_ && frame_id<= end_frame1_) // building models
        {
            if(debug_) {
                fprintf(stderr,"Detection: building model...\n") ;
            }

            //debug
            if(0)
            {
                getHelper()->imWrite( "/home/ubuntu/Downloads/inImageColor.png" , inImageColor,3) ;
                getHelper()->imWrite( "/home/ubuntu/Downloads/inImageColor_R.png" , inImageColor,3,1) ;
                getHelper()->imWrite( "/home/ubuntu/Downloads/inImageColor_G.png" , inImageColor,3,2) ;
                getHelper()->imWrite( "/home/ubuntu/Downloads/inImageColor_B.png" , inImageColor,3,3) ;
            }


            accumFlux_Color(getImgSize().width,getImgSize().height,
                            getFluxTraceNorm()->data() , getStatic()->data() ,
                            inImageColor->R(), inImageColor->G() , inImageColor->B() ,
                            end_frame1_-start_frame1_+1 , BG_->data(),E_->data(),
                            I_->R() , I_->G() , I_->B()
                            ) ;

            accumFlux(getImgSize().width,getImgSize().height, E_->pitch() ,
                      getFluxTraceNorm()->data() , getStatic()->data() , inImageGrey->data(),
                      end_frame1_-start_frame1_+1 , BG_->data(),E_->data(),I_->data() ) ;


            if(frame_id == end_frame1_) {
                BGmask_->copyFrom(BG_->data() , BG_->pitch()) ;
                getHelper()->ThresholdUsingOtsu_0toMax(BG_ , 0 , 255) ;
                nppiDivC_32f_C1R( DP(BGmask_) , 0.8 , DP( BG80percent_ ) , getImgSize()) ;
            }
            if(debug_) {
                fprintf(stderr,"----------------------------------------------------------\n") ;
                fprintf(stderr,"values range of 'getFluxTraceNorm()' is between %f and %f\n", getHelper()->getMinInImage_32f(getFluxTraceNorm()),getHelper()->getMaxInImage_32f(getFluxTraceNorm())) ;
                fprintf(stderr,"values range of 'BG_' is between %f and %f\n", getHelper()->getMinInImage_32f(BG_),getHelper()->getMaxInImage_32f(BG_)) ;
                fprintf(stderr,"values range of 'E_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_),getHelper()->getMaxInImage_32f(E_)) ;
                fprintf(stderr,"values range of 'Ei_' is between %f and %f\n", getHelper()->getMinInImage_32f(getStatic()),getHelper()->getMaxInImage_32f(getStatic())) ;
                fprintf(stderr,"----------------------------------------------------------\n") ;
                getHelper()->imWrite( debug_outpath_+std::string("BG_.png") , BG_) ;
            }
        }
        else
        {
            if(debug_)
                fprintf(stderr,"Detection: running ...\n") ;
            Image32f * Ei = lstEi_.getNextAvailableImg() ;
            Ei->copyFrom(DP(getStatic()));
            lstEi_.insert();

            Image32f * im = lstIm_.getNextAvailableImg() ;
            im->copyFrom(inImageColor);
            lstIm_.insert();


            if(debug_)
            {
                getHelper()->imWrite( "/home/ubuntu/Downloads/im.png" , im,3) ;
                getHelper()->imWrite( "/home/ubuntu/Downloads/im_R.png" , im,3,1) ;
                getHelper()->imWrite( "/home/ubuntu/Downloads/im_G.png" , im,3,2) ;
                getHelper()->imWrite( "/home/ubuntu/Downloads/im_B.png" , im,3,3) ;
            }

            //DDD
            //D_->copyFrom(im);

            if(debug_)
                fprintf(stderr,"values range of 'im(1)' is between %f and %f\n", getHelper()->getMinInImage_32f(im),getHelper()->getMaxInImage_32f(im)) ;

            computeDifference_Color(getImgSize().width , getImgSize().height,
                                    I_sum_head_->R() , I_sum_head_->G(), I_sum_head_->B(),
                                    &I_count_head_ ,
                                    im->R(), im->G() , im->B(),
                                    I_->R(), I_->G() , I_->B(),
                                    BG_->data(),D_->data()) ;

            //Image32f * D = lstD_.getNextAvailableImg() ;
            Image32f * D = D2_ ;

            getHelper()->dilate(E_,E2d_);
            computeDifference(getImgSize().width , getImgSize().height, E_->pitch() ,
                              E_sum_head_->data(),
                              &E_count_head_ ,
                              Ei->data(),
                              E_->data(),
                              BG_->data(), E2d_->data(), D->data(), 0) ;

            //lstD_.insert() ;
            if(frame_id >= start_frame2_ + detection_time_buffer_size_)
            {
                //Image32f * Df_before = D_List_.getOldestImage() ;
                Image32f * Ei_before = lstEi_.getOldestImage() ;
                //Image32f * Df_before = lstD_.getOldestImage() ;

                computeDifference_Color(getImgSize().width , getImgSize().height,
                                        I_sum_tail_->R() , I_sum_tail_->G(), I_sum_tail_->B(),
                                        &I_count_tail_ ,
                                        im->R(), im->G() , im->B(),
                                        I_->R(), I_->G() , I_->B(),
                                        BG_->data(),D_->data()) ;

                getHelper()->dilate(E_,E2d_);
                computeDifference(getImgSize().width , getImgSize().height, E_->pitch() ,
                                  E_sum_tail_->data(), &E_count_tail_ ,
                                  Ei_before->data(), E_->data(), BG_->data(), E2d_->data(), D->data(), 0) ;

                //getNppConv()->imWrite("/home/ubuntu/Downloads/computeDifference.jpg",);

                checkPersistency_Color(getImgSize().width , getImgSize().height,
                                       I_->R(),I_->G(),I_->B(),
                                       I_sum_head_->R() , I_sum_head_->G() , I_sum_head_->B() ,
                                       I_sum_tail_->R() , I_sum_tail_->G() , I_sum_tail_->B() ,
                                       I_mask_->data(), I_pm_->data() , I_count_head_ , I_count_tail_ , D_->data()) ;

                checkPersistency(getImgSize().width , getImgSize().height, E_->pitch() ,
                                 E_->data(),
                                 E_sum_head_->data() ,
                                 E_sum_tail_->data() ,
                                 E_mask_->data(), E_pm_->data() , E_count_head_ , E_count_tail_ , 0) ;

                if(debug_) {
                    fprintf(stderr,"values range of 'im(2)' is between %f and %f\n", getHelper()->getMinInImage_32f(im),getHelper()->getMaxInImage_32f(im)) ;
                    fprintf(stderr,"values range of 'inImageColor' is between %f and %f\n", getHelper()->getMinInImage_32f(inImageColor),getHelper()->getMaxInImage_32f(inImageColor)) ;
                    fprintf(stderr,"values range of 'inImageGrey' is between %f and %f\n", getHelper()->getMinInImage_32f(inImageGrey),getHelper()->getMaxInImage_32f(inImageGrey)) ;

                    fprintf(stderr,"values range of 'E_sum_head_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_sum_head_),getHelper()->getMaxInImage_32f(E_sum_head_)) ;
                    fprintf(stderr,"values range of 'E_sum_tail_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_sum_tail_),getHelper()->getMaxInImage_32f(E_sum_tail_)) ;
                    fprintf(stderr,"values range of 'E_pm_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_pm_),getHelper()->getMaxInImage_32f(E_pm_)) ;
                    fprintf(stderr,"values range of 'E_mask_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_mask_),getHelper()->getMaxInImage_32f(E_mask_)) ;

                    fprintf(stderr,"values range of 'I_' is between %f and %f\n", getHelper()->getMinInImage_32f(I_),getHelper()->getMaxInImage_32f(I_)) ;
                    fprintf(stderr,"values range of 'I_sum_head_' is between %f and %f\n", getHelper()->getMinInImage_32f(I_sum_head_),getHelper()->getMaxInImage_32f(I_sum_head_)) ;
                    fprintf(stderr,"values range of 'I_sum_tail_' is between %f and %f\n", getHelper()->getMinInImage_32f(I_sum_tail_),getHelper()->getMaxInImage_32f(I_sum_tail_)) ;
                    fprintf(stderr,"values range of 'I_pm_' is between %f and %f\n", getHelper()->getMinInImage_32f(I_pm_),getHelper()->getMaxInImage_32f(I_pm_)) ;
                    fprintf(stderr,"values range of 'I_mask_' is between %f and %f\n", getHelper()->getMinInImage_32f(I_mask_),getHelper()->getMaxInImage_32f(I_mask_)) ;

                    fprintf(stderr,"values range of 'D_' is between %f and %f\n", getHelper()->getMinInImage_32f(D_),getHelper()->getMaxInImage_32f(D_)) ;
                }
            }
        }
    }
    else  // grey processing
    {
        if(frame_id>= start_frame1_ && frame_id<= end_frame1_) // building models
        {
            if(frame_id==start_frame1_)
                fprintf(stderr,"Building the model ...\n") ;
            accumFlux(getImgSize().width,getImgSize().height, E_->pitch() ,
                      getFluxTraceNorm()->data() , getStatic()->data() , inImageGrey->data(),
                      end_frame1_-start_frame1_+1 , BG_->data(),E_->data(),I_->data() ) ;
            if(debug_) {
                fprintf(stderr,"values range of 'BG_ before threshing' is between %f and %f\n", getHelper()->getMinInImage_32f(BG_),getHelper()->getMaxInImage_32f(BG_)) ;
                getHelper()->imWrite( debug_outpath_+std::string("BG(beforeThreshing)_.png") , BG_) ;
            }
            if(frame_id == end_frame1_) {
                BGmask_->copyFrom(BG_->data() , BG_->pitch()) ;
                //getHelper()->ThresholdUsingOtsu_0toMax(BG_ , 0 , 255) ;
                //nppiDivC_32f_C1R( DP(BGmask_) , 0.8 , DP( BG80percent_ ) , getImgSize()) ;
                fprintf(stderr,"Model built.\n") ;
            }
            NVTX_PUSHSync("dilate",0);
            getHelper()->dilate(E_,E2d_);
            NVTX_POPSync;
        }
        else
        {
            if(frame_id == end_frame1_ +1 )
                fprintf(stderr,"Detection activated ...\n") ;
            Image32f * Ei = lstEi_.getNextAvailableImg() ;
            NVTX_PUSHSync("copyFrom",0);
            Ei->copyFrom(DP(getStatic()));
            NVTX_POPSync;
            lstEi_.insert();

            //Image32f * I = lstIm_.getNextAvailableImg() ;

            //I->copyFrom(DP(inImage));
            //I_List_.insert();

            //  D=BGmask.*(double(Ei)-double(E)-double(BG)/0.8);

            //Image32f * D = lstD_.getNextAvailableImg() ;
            Image32f * D = D2_ ;


            //NVTX_PUSHSync("fl_DcompDif1", 0);
            computeDifference(getImgSize().width , getImgSize().height, E_->pitch() ,
                              E_sum_head_->data(), &E_count_head_ ,
                              Ei->data(), E_->data(), BG_->data(), E2d_->data(), D->data(), 0) ;
             //NVTX_POPSync ; // fl_DcompDif1

            //lstD_.insert() ;
            if(frame_id >= start_frame2_ + detection_time_buffer_size_)
            {
                //Image32f * Df_before = D_List_.getOldestImage() ;
                Image32f * Ei_before = lstEi_.getOldestImage() ;
                //Image32f * Df_before = lstD_.getOldestImage() ;
                //getHelper()->dilate(E_,E2d_); //HAW
                 //NVTX_PUSHSync("fl_DcompDif2", 0);
                computeDifference(getImgSize().width , getImgSize().height, E_->pitch()
                                  , E_sum_tail_->data(), &E_count_tail_ ,
                                  Ei_before->data(), E_->data(), BG_->data(), E2d_->data(), D->data(), 0) ;
                 //NVTX_POPSync ; // fl_DcompDif2
                 NVTX_PUSHSync("fl_DPers", 0);
                checkPersistency(getImgSize().width , getImgSize().height, E_->pitch() ,
                                 E_->data(), E_sum_head_->data() ,
                                 E_sum_tail_->data() , E_mask_->data(), E_pm_->data() , E_count_head_ , E_count_tail_ , 0) ;
                NVTX_POPSync ; // fl_DPers
                if(debug_) {
                    fprintf(stderr,"values range of 'E_sum_head_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_sum_head_),getHelper()->getMaxInImage_32f(E_sum_head_)) ;
                    fprintf(stderr,"values range of 'E_sum_tail_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_sum_tail_),getHelper()->getMaxInImage_32f(E_sum_tail_)) ;
                    fprintf(stderr,"values range of 'E_pm_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_pm_),getHelper()->getMaxInImage_32f(E_pm_)) ;
                    fprintf(stderr,"values range of 'I_mask_' is between %f and %f\n", getHelper()->getMinInImage_32f(I_mask_),getHelper()->getMaxInImage_32f(I_mask_)) ;
                }
            }
        }
        if(debug_) {
            fprintf(stderr,"----------------------------------------------------------\n") ;
            fprintf(stderr,"values range of 'getFluxTraceNorm()' is between %f and %f\n", getHelper()->getMinInImage_32f(getFluxTraceNorm()),getHelper()->getMaxInImage_32f(getFluxTraceNorm())) ;
            fprintf(stderr,"values range of 'BG_ after threshing' is between %f and %f\n", getHelper()->getMinInImage_32f(BG_),getHelper()->getMaxInImage_32f(BG_)) ;
            fprintf(stderr,"values range of 'E_' is between %f and %f\n", getHelper()->getMinInImage_32f(E_),getHelper()->getMaxInImage_32f(E_)) ;
            fprintf(stderr,"values range of 'Ei_' is between %f and %f\n", getHelper()->getMinInImage_32f(getStatic()),getHelper()->getMaxInImage_32f(getStatic())) ;
            fprintf(stderr,"----------------------------------------------------------\n") ;
            getHelper()->imWrite( debug_outpath_+std::string("getFluxTraceNorm.png") , getFluxTraceNorm()) ;
            getHelper()->imWrite( debug_outpath_+std::string("getStatic.png") , getStatic()) ;
            getHelper()->imWrite( debug_outpath_+std::string("BG(afterThreshing)_.png") , BG_) ;
        }

    }
    NVTX_POPSync ; // fl_detec"


    //  ----------------------------------------------------------------------------------
    //                              OLD DETECTION- Before Nov-16
    //  ----------------------------------------------------------------------------------

    //	if(frame_id>= start_frame1_ && frame_id<= end_frame1_) // building models
    //	{
    //		nppiAdd_32f_C1IR( DP(getFluxTrace()),  DP(BG_) , getImgSize()) ;
    //		nppiAdd_32f_C1IR( DP(getStatic()),  DP(E_) , getImgSize()) ;
    //		if(frame_id == end_frame1_) {
    //			BGmask_->copyFrom(BG_->data() , BG_->pitch()) ;
    //			getNppConv()->ThresholdUsingOtsu_0toMax(BG_ , 1 , 0) ;
    //			nppiDivC_32f_C1R( DP(BGmask_) , 0.8 , DP( BG80percent_ ) , getImgSize()) ;
    //		}
    //	}
    //	else
    //	{
    //		//  D=BGmask.*(double(Ei)-double(E)-double(BG)/0.8);
    //		Image32f * D = D_List_.getNextAvailableImg() ;

    //		nppiSub_32f_C1R( DP(getStatic()) , DP(E_) , DP(D) , getImgSize()) ; // delta = Ei - E
    //		nppiSub_32f_C1IR( DP( BG80percent_) , DP(D) , getImgSize()) ; // delta - BG80
    //		nppiMul_32f_C1IR(DP(BGmask_) , DP(D) , getImgSize()) ; // D = BGMask*D
    //		nppiThreshold_32f_C1IR( DP(D) , getImgSize() , 0.0 , NPP_CMP_LESS ) ; //D(D<0)=0;
    //		D_List_.insert() ;
    //		nppiAdd_32f_C1IR( DP(D),DP(sum_head_), getImgSize()) ;
    //		if(frame_id >= start_frame2_ + detection_time_buffer_size_)
    //		{
    //			Image32f * Df_before = D_List_.getOldestImage() ;
    //			nppiAdd_32f_C1IR( DP(Df_before),DP(sum_tail_), getImgSize()) ;
    //			nppiSub_32f_C1R( DP(sum_head_), DP(sum_tail_)  , DP(P_) ,  getImgSize()) ;
    //			nppiDivC_32f_C1IR( detection_time_buffer_size_ , DP(P_) , getImgSize()) ;
    //			nppiThreshold_32f_C1IR( DP(P_) , getImgSize() , 0.0 , NPP_CMP_LESS ) ; //P_(P_<0)=0;
    //		}
    //	}

}
/*
 * writes out the processed data into files.
 * Input: orgFilePath a string containing the output path
 * Notice: there must be a subfolder "output" already in orgFilePath
 */
int FluxTensor::writeResults(std::string orgFilePath)
{
    Image32f *Ix, *Iy, *Is, *fluxTrace ;
    int lastFrameID ;
    std::string relPath ;

    //if( getNumOfElements()==0 )
    //	return -1 ;
    //lastFrameID = getNumOfElements() - 1 ;
    lastFrameID = Ix_List_.getLastFilledPos() ;

    std::string sFilename  = getFileFullNameFromFullPath(orgFilePath) ;
    std::string sFilename_justName ;
    std::string strExt ;
    splitFileNameAndExtension (sFilename,sFilename_justName,strExt) ;
    std::string sOutputPath = getPathFromFullPath(orgFilePath) + std::string("/output/") ;
    createFolderOrDie(sOutputPath) ;
    //----------------------
    getIxIyIs( &Ix , &Iy ,  &Is , lastFrameID ) ;
    relPath = "IxIyIs/" ;
    createFolderOrDie(sOutputPath + relPath ) ;
    //std::string tmp= sOutputPath + relPath + sFilename_justName + "_Ix" + strExt ;
    getHelper()->imWrite( sOutputPath + relPath + sFilename_justName + "_Ix" + strExt, Ix) ;
    getHelper()->imWrite( sOutputPath + relPath + sFilename_justName + "_Iy"+ strExt, Iy) ;
    getHelper()->imWrite( sOutputPath + relPath + sFilename_justName + "_Is"+ strExt, Is) ;
    //----------------------
    relPath = "FTrace_Normalized/" ;
    createFolderOrDie(sOutputPath + relPath ) ;
    getHelper()->imWrite(sOutputPath + relPath + sFilename_justName  + "_ftraceNorm" + strExt, getFluxTraceNorm() ) ;
    //----------------------
    relPath = "STrace_Normalized/" ;
    createFolderOrDie(sOutputPath + relPath ) ;
    getHelper()->imWrite(sOutputPath + relPath + sFilename_justName + "_straceNorm"+ strExt, getSTTraceNorm() ) ;
    //----------------------
    relPath = "static/" ;
    createFolderOrDie(sOutputPath + relPath ) ;
    getHelper()->imWrite(sOutputPath + relPath + sFilename_justName + "_static"+ strExt, getStatic() ) ;
    //----------------------
    relPath = "Mask_FluxTrace/" ;
    createFolderOrDie(sOutputPath + relPath ) ;
    getHelper()->imWrite(sOutputPath + relPath + sFilename_justName + "_fluxTrace_mask"+ strExt, getFluxTraceMask() ) ;
    //----------------------
    //    relPath = "change_static2/" ;
    //    createFolderOrDie(sOutputPath + relPath ) ;
    //    getHelper()->imWrite(sOutputPath + relPath + sFilename_justName + "_diff_image"+ strExt, getChange()) ;
    //----------------------
    relPath = "change_persistent/" ;
    createFolderOrDie(sOutputPath + relPath ) ;
    getHelper()->imWrite(sOutputPath + relPath + sFilename_justName + "_persistent_image"+ strExt, getPersistent()) ;
    //----------------------
    relPath = "change_persistent/" ;
    createFolderOrDie(sOutputPath + relPath ) ;
    getHelper()->imWrite(sOutputPath + relPath + sFilename_justName + "_persistent_mask"+ strExt, getPersistentMask()) ;

    //----------------------
    if(debug_)
    {
        fprintf(stderr,"values range of 'Ix' is between %f and %f\n", getHelper()->getMinInImage_32f(Ix),getHelper()->getMaxInImage_32f(Ix)) ;
        fprintf(stderr,"values range of 'getFluxTraceNorm()' is between %f and %f\n", getHelper()->getMinInImage_32f(getFluxTraceNorm()),getHelper()->getMaxInImage_32f(getFluxTraceNorm())) ;
        fprintf(stderr,"values range of 'getSTTraceNorm()' is between %f and %f\n", getHelper()->getMinInImage_32f(getSTTraceNorm()),getHelper()->getMaxInImage_32f(getSTTraceNorm())) ;
        fprintf(stderr,"values range of 'getStatic()' is between %f and %f\n", getHelper()->getMinInImage_32f(getStatic()),getHelper()->getMaxInImage_32f(getStatic())) ;
    }
    //----------------------
    return 1 ;
}

/*
 * to store the current model internal state
 */
//#define GET_VAR_NAME(var) (#var)
void FluxTensor::storeState(std::string sOutputPath)
{
    sOutputPath += "/SYS_" ;
    //std::string sOutputPath = "/media/ubuntu/SataHDD/TX1_state_cache/SYS__" ;
    // ----------------------------------
    fprintf(stderr,"saving current internal state...\n") ;
    // getHelper()->imWriteBinary(sOutputPath + ".dat" , ) ;
    getHelper()->imWriteBinary(sOutputPath + "static_.dat" , static_) ;
    getHelper()->imWriteBinary(sOutputPath + "flux_trace_.dat" , flux_trace_) ;
    getHelper()->imWriteBinary(sOutputPath + "flux_trace_norm_.dat" ,flux_trace_norm_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "st_trace_.dat" ,st_trace_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "st_trace_norm_.dat" , st_trace_norm_) ;
    getHelper()->imWriteBinary(sOutputPath + "static_.dat" , static_) ;
    getHelper()->imWriteBinary(sOutputPath + "flux_trace_mask_.dat" , flux_trace_mask_) ;
    getHelper()->imWriteBinary(sOutputPath + "BG_.dat" , BG_) ;
    getHelper()->imWriteBinary(sOutputPath + "BGmask_.dat" , BGmask_) ;
    getHelper()->imWriteBinary(sOutputPath + "E_sum_head_.dat" , E_sum_head_) ;
    getHelper()->imWriteBinary(sOutputPath + "E_sum_tail_.dat" ,E_sum_tail_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "I_sum_head_.dat" , I_sum_head_) ;
    getHelper()->imWriteBinary(sOutputPath + "I_sum_tail_.dat" ,I_sum_tail_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "I_mask_.dat" ,I_mask_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "E_mask_.dat" ,E_mask_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "E_.dat" , E_) ;
    getHelper()->imWriteBinary(sOutputPath + "I_.dat" ,I_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "E_pm_.dat" ,E_pm_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "I_pm_.dat" , I_pm_) ;
    getHelper()->imWriteBinary(sOutputPath + "E2d_.dat" , E2d_) ;
    getHelper()->imWriteBinary(sOutputPath + "D_.dat" ,D_ ) ;
    getHelper()->imWriteBinary(sOutputPath + "D2_.dat" ,D2_ ) ;

    for(int i=0 ; i<lstEi_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imWriteBinary(sOutputPath + "lstEi_" +num.str()+ ".dat" , lstEi_.getImage(i) ) ;
    }

    for(int i=0 ; i<Ix_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imWriteBinary(sOutputPath + "Ix_List_" +num.str()+ ".dat" , Ix_List_.getImage(i) ) ;
    }

    for(int i=0 ; i<Iy_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imWriteBinary(sOutputPath + "Iy_List_" +num.str()+ ".dat" , Iy_List_.getImage(i) ) ;
    }

    for(int i=0 ; i<Is_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imWriteBinary(sOutputPath + "Is_List_" +num.str()+ ".dat" , Is_List_.getImage(i) ) ;
    }

    for(int i=0 ; i<trace_sumxy_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imWriteBinary(sOutputPath + "trace_sumxy_List_" +num.str()+ ".dat" , trace_sumxy_List_.getImage(i) ) ;
    }
    for(int i=0 ; i<traceST_sumxy_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imWriteBinary(sOutputPath + "traceST_sumxy_List_" +num.str()+ ".dat" , traceST_sumxy_List_.getImage(i) ) ;
    }

    fprintf(stderr,"Internal state saved.\n") ;
    savedState_ = true ;
}
void FluxTensor::restoreState(std::string sOutputPath)
{
    if(!savedState_)
    {
        fprintf(stderr,"There is no state to restore!\n") ;
        return ;
    }
    sOutputPath += "/SYS_" ;
    //std::string sOutputPath = "/media/ubuntu/SataHDD/TX1_state_cache/SYS__" ;
    // ----------------------------------
    fprintf(stderr,"restoring previous internal state...\n") ;
    getHelper()->imReadBinary(sOutputPath + "static_.dat" , static_) ;
    getHelper()->imReadBinary(sOutputPath + "flux_trace_.dat" , flux_trace_) ;
    getHelper()->imReadBinary(sOutputPath + "flux_trace_norm_.dat" ,flux_trace_norm_ ) ;
    getHelper()->imReadBinary(sOutputPath + "st_trace_.dat" ,st_trace_ ) ;
    getHelper()->imReadBinary(sOutputPath + "st_trace_norm_.dat" , st_trace_norm_) ;
    getHelper()->imReadBinary(sOutputPath + "static_.dat" , static_) ;
    getHelper()->imReadBinary(sOutputPath + "flux_trace_mask_.dat" , flux_trace_mask_) ;
    getHelper()->imReadBinary(sOutputPath + "BG_.dat" , BG_) ;
    getHelper()->imReadBinary(sOutputPath + "BGmask_.dat" , BGmask_) ;
    getHelper()->imReadBinary(sOutputPath + "E_sum_head_.dat" , E_sum_head_) ;
    getHelper()->imReadBinary(sOutputPath + "E_sum_tail_.dat" ,E_sum_tail_ ) ;
    getHelper()->imReadBinary(sOutputPath + "I_sum_head_.dat" , I_sum_head_) ;
    getHelper()->imReadBinary(sOutputPath + "I_sum_tail_.dat" ,I_sum_tail_ ) ;
    getHelper()->imReadBinary(sOutputPath + "I_mask_.dat" ,I_mask_ ) ;
    getHelper()->imReadBinary(sOutputPath + "E_mask_.dat" ,E_mask_ ) ;
    getHelper()->imReadBinary(sOutputPath + "E_.dat" , E_) ;
    getHelper()->imReadBinary(sOutputPath + "I_.dat" ,I_ ) ;
    getHelper()->imReadBinary(sOutputPath + "E_pm_.dat" ,E_pm_ ) ;
    getHelper()->imReadBinary(sOutputPath + "I_pm_.dat" , I_pm_) ;
    getHelper()->imReadBinary(sOutputPath + "E2d_.dat" , E2d_) ;
    getHelper()->imReadBinary(sOutputPath + "D_.dat" ,D_ ) ;
    getHelper()->imReadBinary(sOutputPath + "D2_.dat" ,D2_ ) ;

    for(int i=0 ; i<lstEi_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imReadBinary(sOutputPath + "lstEi_" +num.str()+ ".dat" , lstEi_.getImage(i) ) ;
    }

    for(int i=0 ; i<Ix_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imReadBinary(sOutputPath + "Ix_List_" +num.str()+ ".dat" , Ix_List_.getImage(i) ) ;
    }

    for(int i=0 ; i<Iy_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imReadBinary(sOutputPath + "Iy_List_" +num.str()+ ".dat" , Iy_List_.getImage(i) ) ;
    }

    for(int i=0 ; i<Is_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imReadBinary(sOutputPath + "Is_List_" +num.str()+ ".dat" , Is_List_.getImage(i) ) ;
    }

    for(int i=0 ; i<trace_sumxy_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imReadBinary(sOutputPath + "trace_sumxy_List_" +num.str()+ ".dat" , trace_sumxy_List_.getImage(i) ) ;
    }
    for(int i=0 ; i<traceST_sumxy_List_.getCapacity() ; i++) {
        std::ostringstream num ;
        num << i ;
        getHelper()->imReadBinary(sOutputPath + "traceST_sumxy_List_" +num.str()+ ".dat" , traceST_sumxy_List_.getImage(i) ) ;
    }

    fprintf(stderr,"Internal state restored.\n") ;
}
