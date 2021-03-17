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

#pragma once

#include <stdio.h>
#include <cuda_runtime.h>
#include <stdlib.h>

typedef struct
{
	int w, h;				// Image width and height
	int stride;
} ImageProp ;

#define cudaMemErrChk2(ans) { cudaMemAssert2((ans) , __FILE__ , __LINE__) ; }
inline void cudaMemAssert2(cudaError_t code, const char *file, int line, bool abort=true)
{
    if(code != cudaSuccess)
    {
        //fprintf(stderr,"Fatal error: memory allocation failed!\n") ;
        fprintf(stderr,"cudaMemAssert: %s %s %d\n" , cudaGetErrorString(code) , file , line) ;
        if(abort)
            exit(code) ;
    }
}

//__global__ void hello(char *a, int *b) ;
void testHello(char* a,int *b, int csize, int isize) ;

void FluxCudaKernelsInit(ImageProp *imgProp , float *td1_ , float *td2_, int td1_size_ , float *tavg_, int tavg_size_) ;
void addImages(float * a, float * b, float * c, int , int) ;
void FluxDerivByKernel(int w, int h, int pitch, int masterFrameIndex , float *Ix[] , float *Iy[], float *Is[] , float *Mxt, float *Myt, float *Mt, float *Mtt, float *Mtrace, float *MtraceST) ;
void FluxTempAVGByKernel(int w, int h,int pitch, float *TraceSumXY_Data[] , float *TraceSTSumXY_Data[], float *FluxTrace, float * STTrace, float dominator) ;

//-------------------------
// -----  detection -------
//-------------------------
void computeDifference(int w, int h, int pitch, float *sum_head, int *count, float *Ei, float *E, float *BG, float *E2d, float *result,int type) ;
void computeDifference_Color(int w, int h,
                             float *sum_head_r, float *sum_head_g, float *sum_head_b,
                             int *count,
                             float *Ei_r,float *Ei_g,float *Ei_b,
                             float *E_r,float *E_g,float *E_b,
                             float *BG,float *D) ;
void checkPersistency(int w, int h, int pitch, float *E,float *sum_head,float *sum_tail, float *mask, float *pm,int count_head, int count_tail,int type) ;
void checkPersistency_Color(int w, int h,
                            float *E_r,float *E_g,float *E_b,
                            float *sum_head_r,float *sum_head_g,float *sum_head_b,
                            float *sum_tail_r,float *sum_tail_g,float *sum_tail_b,
                            float *mask, float *pm,int count_head, int count_tail,float *D) ;
void accumFlux(int w, int h, int pitch, float *flux,float *E,float *I, int n_frames , float *out_flux_mean, float *out_E_mean, float *out_I_mean) ;
void accumFlux_Color(int w, int h,float *flux,float *E,float *Ir,float *Ig,float *Ib, int n_frames , float *out_flux_mean, float *out_E_mean, float *out_Ir_mean,float *out_Ig_mean,float *out_Ib_mean) ;
