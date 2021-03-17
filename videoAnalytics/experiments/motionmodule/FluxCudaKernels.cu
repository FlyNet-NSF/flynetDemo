
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

#include "FluxCudaKernels.h"

const int BLOCK_SIZE1 = 16;
const int BLOCK_SIZE2 = 16;

const int border = 30 ;

__constant__ ImageProp ip ;

__constant__ float td1[20] ;
__constant__ float td2[20] ;
__constant__ int td1_size ;
__constant__ float tavg[20] ;
__constant__ int tavg_size ;

__global__
void hello(char *a, int *b)
{
    a[threadIdx.x] += b[threadIdx.x];
}

void testHello(char* a,int *b, int csize, int isize)
{
    char *ad;
    int *bd;
    cudaMalloc( (void**)&ad, csize );
    cudaMalloc( (void**)&bd, isize );
    cudaMemcpy( ad, a, csize, cudaMemcpyHostToDevice );
    cudaMemcpy( bd, b, isize, cudaMemcpyHostToDevice );

    dim3 dimBlock( BLOCK_SIZE1, 1 );
    dim3 dimGrid( 1, 1 );
    hello<<<dimGrid, dimBlock>>>(ad, bd);
    cudaMemcpy( a, ad, csize, cudaMemcpyDeviceToHost );
    cudaFree( ad );
    cudaFree( bd );
}

__global__ void addImages_kernel(float * a, float * b, float * c)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
#if 1
    int w= ip.w ;
    int h= ip.h ;
    int stride = ip.stride ;
#endif
#if 0
    int w= 1000;
    int h= 1000 ;
    int pitch = 1000 ;
#endif
    if ((i >= 0) && (j >= 0) && (i < (w-1)) && (j < (h-1)))
    {
        float *p_a = a + j*stride + i ;
        float *p_b = b + j*stride + i ;
        float *p_c = c + j*stride + i ;
        *p_c = *p_a + *p_b ;
    }
}
void addImages(float * a, float * b, float * c,int w, int h)
{
#if 0
    dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    dim3 numBlocks(1000/threadsPerBlock.x,  /* for instance 512/8 = 64*/
                   1000/threadsPerBlock.y);
#endif
#if 1
    dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    dim3 numBlocks(w/threadsPerBlock.x,  /* for instance 512/8 = 64*/
                   h/threadsPerBlock.y);
#endif
    addImages_kernel<<<numBlocks, threadsPerBlock>>>(a, b , c);
}

void FluxCudaKernelsInit(ImageProp *imgProp , float *td1_ , float *td2_, int td1_size_ , float *tavg_, int tavg_size_)
{
    cudaMemcpyToSymbol(ip, imgProp, sizeof(ImageProp));
    cudaMemcpyToSymbol(td1, td1_, sizeof(float)*td1_size_);
    cudaMemcpyToSymbol(td2, td2_, sizeof(float)*td1_size_);
    cudaMemcpyToSymbol(td1_size, &td1_size_, sizeof(int));
    cudaMemcpyToSymbol(tavg, tavg_, sizeof(float)*tavg_size_);
    cudaMemcpyToSymbol(tavg_size, &tavg_size_, sizeof(int));
}

////////////////////////////////////////////////////////
__global__ void TEST(int width,int height, int pitch, float *Ix[] , float *Iy[], float *Is[] , float *Mxt, float *Myt, float *Mt, float *Mtt, float *Mtrace, float *MtraceST, int masterFrameIndex)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    pitch /= sizeof(float) ; // making it proper for later pointer usage
    int idx = j*pitch + i;

    float v_Mxt = 0.0f;
    float v_Myt = 0.0f;
    float v_Mt = 0.0f;
    float v_Mtt = 0.0f;

    if ((i >= 0) && (j >= 0) && (i < 100) && (j < 100))
        //if ((i >= border) && (j >= border) && (i < (width-border-1)) && (j < (height-border-1)))
    {
        //float *p_a = a + j*stride + i ;
        float *p_Mxt = Mxt + idx;
        float *p_Myt = Myt + idx;
        float *p_Mt  = Mt + idx;
        float *p_Mtt = Mtt + idx;
        float *p_Mtrace = Mtrace + idx;
        float *p_MtraceST = MtraceST + idx;
        float *p_Ix ;
        float *p_Iy ;
        float *p_Is ;

        *p_Mxt = 0 ;
        *p_Myt = 0 ;
        *p_Mt = 0 ;
        *p_Mtt = 0 ;

        //for(int k=0 ; k<td1_size ; k++ ) {
        for(int k=0 ; k<5 ; k++ ) {
            p_Ix = Ix[k] + idx;
            p_Iy = Iy[k] + idx;
            p_Is = Is[k] + idx;
            (*p_Mxt) +=  (*p_Ix)*td1[k] ;
            (*p_Myt) +=  (*p_Iy)*td1[k] ;
            (*p_Mt) +=  (*p_Is)*td1[k] ;
            (*p_Mtt) +=  (*p_Is)*td2[k] ;

            v_Mxt +=  (*p_Ix)*td1[k] ;
            v_Myt +=  (*p_Iy)*td1[k] ;
            v_Mt +=  (*p_Is)*td1[k] ;
            v_Mtt +=  (*p_Is)*td2[k] ;

            (*p_Ix) = (float)td1_size ;
            //(*p_Ix) = 99.0 ;
        }
        //*p_Mtrace = v_Mxt ;
        // for flux tensor
        //*p_Mtrace = sqrt((*p_Mxt)*(*p_Mxt) + (*p_Myt)*(*p_Myt) + (*p_Mtt)*(*p_Mtt) );
        *p_Mtrace = sqrt((v_Mxt)*(v_Mxt) + (v_Myt)*(v_Myt) + (v_Mtt)*(v_Mtt) );

        // for 3D structure tensor
        p_Ix = Ix[masterFrameIndex] + idx;
        p_Iy = Iy[masterFrameIndex] + idx;
        //*p_MtraceST = sqrt((*p_Ix)*(*p_Ix) + (*p_Iy)*(*p_Iy) + (*p_Mt)*(*p_Mt) );
        *p_MtraceST = sqrt((*p_Ix)*(*p_Ix) + (*p_Iy)*(*p_Iy) + (v_Mt)*(v_Mt) );
    }
}

////////////////////////////////////////////////////////
__global__ void FluxDerivKernel(int width,int height, int pitch, float *Ix[] , float *Iy[], float *Is[] , float *Mxt, float *Myt, float *Mt, float *Mtt, float *Mtrace, float *MtraceST, int masterFrameIndex)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    pitch /= sizeof(float) ; // making it proper for later pointer usage
    int idx = j*pitch + i;

    float v_Mxt = 0.0f;
    float v_Myt = 0.0f;
    float v_Mt = 0.0f;
    float v_Mtt = 0.0f;

    //if ((i >= 50) && (j >= 50) && (i < 100) && (j < 100))
    if ((i >= border) && (j >= border) && (i < (width-border-1)) && (j < (height-border-1)))
    {
        //float *p_a = a + j*stride + i ;
        float v_Ix, v_Iy, v_Is ;

        for(int k=0 ; k<td1_size ; k++ ) {
            v_Ix = (Ix[k])[idx];
            v_Iy = (Iy[k])[idx];
            v_Is = (Is[k])[idx];

            v_Mxt +=  v_Ix*td1[k] ;
            v_Myt +=  v_Iy*td1[k] ;
            v_Mt +=  v_Is*td1[k] ;
            v_Mtt +=  v_Is*td2[k] ;
        }

        Mxt[idx] = v_Mxt;
        Myt[idx] = v_Myt;
        Mt[idx] = v_Mt;
        Mtt[idx] = v_Mtt;

        // for flux tensor
        Mtrace[idx] = sqrt((v_Mxt)*(v_Mxt) + (v_Myt)*(v_Myt) + (v_Mtt)*(v_Mtt) );

        // for 3D structure tensor
        v_Ix = (Ix[masterFrameIndex])[idx];
        v_Iy = (Iy[masterFrameIndex])[idx];
        MtraceST[idx] = sqrt(v_Ix*v_Ix + v_Iy*v_Iy + v_Mt*v_Mt );
    }
}

void FluxDerivByKernel(int width, int height, int pitch, int masterFrameIndex, float *Ix[] , float *Iy[], float *Is[] , float *Mxt, float *Myt, float *Mt, float *Mtt, float *Mtrace, float *MtraceST)
{
    static bool first=true ;
    static dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    static dim3 numBlocks(width/threadsPerBlock.x,  height/threadsPerBlock.y);
    static int nt_deriv ;
    if(first)
        cudaMemcpyFromSymbol(&nt_deriv,td1_size, sizeof(int));

    //printf("td1_size is %d\n",nt_deriv) ;

    static float **d_IxData ;
    static float **d_IyData ;
    static float **d_IsData ;
    if(first) {
        cudaMemErrChk2(cudaMalloc((void**)&d_IxData, nt_deriv * sizeof(float *)));
        cudaMemErrChk2(cudaMalloc((void**)&d_IyData, nt_deriv * sizeof(float *)));
        cudaMemErrChk2(cudaMalloc((void**)&d_IsData, nt_deriv * sizeof(float *)));
    }

    //	for(int i=0 ; i<nt_deriv ; i++) {
    //		cudaMemcpy(&(d_IxData[i]), &(Ix[i]), sizeof(float *), cudaMemcpyHostToDevice);//copy child pointer to device
    //		cudaMemcpy(&(d_IyData[i]), &(Iy[i]), sizeof(float *), cudaMemcpyHostToDevice);//copy child pointer to device
    //		cudaMemcpy(&(d_IsData[i]), &(Is[i]), sizeof(float *), cudaMemcpyHostToDevice);//copy child pointer to device
    //	}

    cudaMemcpy(d_IxData, Ix, nt_deriv*sizeof(float *), cudaMemcpyHostToDevice);//copy child pointer to device
    cudaMemcpy(d_IyData, Iy, nt_deriv*sizeof(float *), cudaMemcpyHostToDevice);//copy child pointer to device
    cudaMemcpy(d_IsData, Is, nt_deriv*sizeof(float *), cudaMemcpyHostToDevice);//copy child pointer to device

    FluxDerivKernel<<<numBlocks, threadsPerBlock>>>(width, height, pitch, d_IxData , d_IyData , d_IsData , Mxt , Myt, Mt , Mtt, Mtrace, MtraceST, masterFrameIndex);
    //TEST<<<numBlocks, threadsPerBlock>>>(width, height, pitch, d_IxData , d_IyData , d_IsData , Mxt , Myt, Mt , Mtt, Mtrace, MtraceST, masterFrameIndex);

    first = false ;
    return ;
    //ssd These lines are unreachable	cudaFree(d_IxData) ;
    //ssd These lines are unreachable	cudaFree(d_IyData) ;
    //ssd These lines are unreachable	cudaFree(d_IsData) ;
}
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
__global__ void FluxTempAVGKernel(int width,int height, int pitch, float *TraceSumXY[] , float *TraceSTSumXY[], float *FluxTrace, float * STTrace, float d_denominator)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    pitch /= sizeof(float) ; // making it proper for later pointer usage
    int idx = j*pitch + i;
    //if ((i >= 0) && (j >= 0) && (i < (width-1)) && (j < (height-1)))
    if ((i >= border) && (j >= border) && (i < (width-border-1)) && (j < (height-border-1)))
    {
/*        float *p_FluxTrace = FluxTrace + idx ;
        float *p_STTrace = STTrace + idx ;
        float *p_TraceSumXY ;
        float *p_TraceSTSumXY ;      
        *p_FluxTrace = 0 ;
        *p_STTrace = 0 ;
*/
        float v_FluxTrace = 0.0f;
        float v_STTrace = 0.0;

        for(int k=0 ; k<tavg_size ; k++ ) {
 /*           p_TraceSumXY = TraceSumXY[k] + idx ;
            (*p_FluxTrace) += (*p_TraceSumXY)*tavg[k] ;

            p_TraceSTSumXY = TraceSTSumXY[k] + idx ;
            (*p_STTrace) += (*p_TraceSTSumXY)*tavg[k] ;
 */
             v_FluxTrace += (TraceSumXY[k])[idx] * tavg[k] ;

             v_STTrace += (TraceSTSumXY[k])[idx] * tavg[k] ;
        }
/*        (*p_FluxTrace) /= d_denominator ;
        (*p_STTrace) /= d_denominator ;
        */
        FluxTrace[idx] = v_FluxTrace / d_denominator;
        STTrace[idx] = v_STTrace / d_denominator;
    }
}
void FluxTempAVGByKernel(int width,int height, int pitch, float *TraceSumXY[] , float *TraceSTSumXY[], float *FluxTrace, float * STTrace, float denominator)
{
    static bool first=true ;
    static dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    static dim3 numBlocks(width/threadsPerBlock.x,  height/threadsPerBlock.y);
    static int nt_avg ;

    static float **d_TraceSumXY ;
    static float **d_TraceSTSumXY;
    //static float *d_denominator ;
    if(first) {
        cudaMemcpyFromSymbol(&nt_avg,tavg_size, sizeof(int));
        cudaMemErrChk2(cudaMalloc((void**)&d_TraceSumXY, nt_avg * sizeof(float *)));
        cudaMemErrChk2(cudaMalloc((void**)&d_TraceSTSumXY, nt_avg * sizeof(float *)));

        //cudaMemErrChk2(cudaMalloc((void**)&d_denominator,  sizeof(float)));
        //cudaMemcpy(d_denominator , &denominator, sizeof(float), cudaMemcpyHostToDevice);
    }
    cudaMemcpy(d_TraceSumXY , TraceSumXY, nt_avg*sizeof(float *), cudaMemcpyHostToDevice);
    cudaMemcpy(d_TraceSTSumXY , TraceSTSumXY, nt_avg*sizeof(float *), cudaMemcpyHostToDevice);

    FluxTempAVGKernel<<<numBlocks, threadsPerBlock>>>(width, height, pitch,d_TraceSumXY,d_TraceSTSumXY,FluxTrace,STTrace, denominator) ;

    first = false ;
    return ;
}

//--------------------------------------------
// -----  detection--computeDifference -------
//--------------------------------------------
__global__ void computeDifferenceKernel(int width,int height, int pitch, float *sum_head , float *Ei, float *E, float *BG, float *E2d, float *D)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    int offset ;
    pitch /= sizeof(float) ; // making it proper for later pointer usage
    //static float ww=0 ; //sa 1,25
    //if ((i >= 0) && (j >= 0) && (i <= (width-1)) && (j <= (height-1)))
    if ((i >= border) && (j >= border) && (i < (width-border-1)) && (j < (height-border-1)))
    {
        offset =  j*pitch + i ;
        float *p_sum_head = sum_head + offset ;
        float *p_Ei = Ei  + offset  ;
        //ssd        float *p_E = E + offset  ;
        float *p_BG = BG + offset  ;
        float *p_D = D + offset  ;
        float *p_E2d = E2d + offset  ;
        //*p_D = (*p_Ei) - (*p_E2d) - ww * (*p_BG) ; //sa
        *p_D = (*p_Ei) - (*p_E2d)  ;
        if((*p_D)<0)
            *p_D = 0 ;
        (*p_sum_head) += *p_D ;
    }
}
void computeDifference(int width, int height, int pitch, float *sum_head, int *count, float *Ei, float *E, float *BG, float *E2d, float *D, int type)
{
    if(type!=0) //not supported
        return ;
    static dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    static dim3 numBlocks(width/threadsPerBlock.x,  height/threadsPerBlock.y);
    computeDifferenceKernel<<<numBlocks, threadsPerBlock>>>(width, height, pitch,sum_head,Ei,E,BG,E2d,D) ;
    (*count)++ ;
}
//----------------------------------------------------------------------------------------------
__global__ void computeDifferenceKernel_color( float *sum_head_r, float *sum_head_g, float *sum_head_b,
                                               float *Ei_r,float *Ei_g,float *Ei_b, float *E_r,float *E_g,float *E_b, float *BG, float *D)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    int offset ;
    int w= ip.w ;
    int h= ip.h ;
    int stride = ip.stride ;
    static float ww=1.25 ;
    const float EPS=0.0001 ;
    if ((i >= 0) && (j >= 0) && (i <= (w-1)) && (j <= (h-1)))
    {
        offset =  j*stride + i ;
        float *p_sum_head_r = sum_head_r + offset ;
        float *p_sum_head_g = sum_head_g + offset ;
        //ssd        float *p_sum_head_b = sum_head_b + offset ;

        float *p_Ei_r = Ei_r  + offset  ;
        float *p_Ei_g = Ei_g  + offset  ;
        float *p_Ei_b = Ei_b  + offset  ;

        float *p_E_r = E_r + offset  ;
        float *p_E_g = E_g + offset  ;
        float *p_E_b = E_b + offset  ;
        //ssd        float *p_D = D + offset  ;

        float *p_BG = BG + offset  ;

        float si = (*p_Ei_r) +  (*p_Ei_g) + (*p_Ei_b) + EPS ;
        float s = (*p_E_r) +  (*p_E_g) + (*p_E_b) + EPS  ;

        //DDD
        //*p_D = *p_Ei_g ;


#if 1

        float D_r = 150*abs((*p_Ei_r)/si-(*p_E_r)/s) - ww*(*p_BG) ;
        float D_g = 150*abs((*p_Ei_g)/si-(*p_E_g)/s) - ww*(*p_BG) ;


        //        double D_r = 150*abs((*p_Ei_r)/si-(*p_E_r)/s) - ww*(*p_BG) ;
        //        double D_g = 150*abs((*p_Ei_g)/si-(*p_E_g)/s) - ww*(*p_BG) ;

#else
        (*p_Ei_r) /= si ;
        (*p_Ei_g) /= si ;
        (*p_Ei_b) = 0 ;

        (*p_E_r) /= s ;
        (*p_E_g) /= s ;
        (*p_E_b) = 0 ;

        double D_r = 150*abs((*p_Ei_r)-(*p_E_r)) - ww*(*p_BG) ;
        double D_g = 150*abs((*p_Ei_g)-(*p_E_g)) - ww*(*p_BG) ;
#endif

        if( D_r < 0 )
            D_r = 0 ;
        if( D_g < 0 )
            D_g = 0 ;



        (*p_sum_head_r) += D_r ;
        (*p_sum_head_g) += D_g ;


    }
}
void computeDifference_Color(int w, int h,
                             float *sum_head_r, float *sum_head_g, float *sum_head_b,
                             int *count,
                             float *Ei_r, float *Ei_g, float *Ei_b,
                             float *E_r, float *E_g, float *E_b,
                             float *BG,float *D)
{
    static dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    static dim3 numBlocks(w/threadsPerBlock.x,  h/threadsPerBlock.y);
    computeDifferenceKernel_color<<<numBlocks, threadsPerBlock>>>(sum_head_r,sum_head_g,sum_head_b,Ei_r,Ei_g,Ei_b,E_r,E_g,E_b,BG,D) ;
    (*count)++ ;
}

//--------------------------------------------
// -----  detection--checkPersistency -------
//--------------------------------------------
__global__ void checkPersistencyKernel(int width, int height, int pitch, float *E,float *sum_head,float *sum_tail, float *mask, float *pm, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    int offset ;
    pitch /= sizeof(float) ; // making it proper for later pointer usage
    //if ((i >= 0) && (j >= 0) && (i <= (width-1)) && (j <= (height-1)))
    if ((i >= border) && (j >= border) && (i < (width-border-1)) && (j < (height-border-1)))
    {
        offset =  j*pitch + i ;
        //ssd        float *p_E = E + offset  ;
        float *p_sum_head = sum_head + offset ;
        float *p_sum_tail = sum_tail + offset ;
        float *p_pm = pm + offset  ;
        float *p_mask = mask + offset  ;
        *p_pm = ((*p_sum_head) - (*p_sum_tail)) / n ;
        *p_mask = *p_pm > 10 ? 255 : 0 ;
    }
}
void checkPersistency(int width, int height, int pitch, float *E, float *sum_head,float *sum_tail, float *mask, float *pm,int count_head, int count_tail,int type)
{
    if(type!=0) //not supported
        return ;
    int n=count_head-count_tail+1 ;
    static dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    static dim3 numBlocks(width/threadsPerBlock.x,  height/threadsPerBlock.y);
    checkPersistencyKernel<<<numBlocks, threadsPerBlock>>>(width, height, pitch, E, sum_head, sum_tail, mask, pm, n);
}
//------------------------------------------------------------------------------
__global__ void checkPersistencyKernel_Color(float *E_r,float *E_g,float *E_b,float *sum_head_r,float *sum_head_g,float *sum_head_b,float *sum_tail_r,float *sum_tail_g,float *sum_tail_b, float *mask, float *pm, int n , float *D)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    int offset ;
    int w= ip.w ;
    int h= ip.h ;
    int stride = ip.stride ;
    if ((i >= 0) && (j >= 0) && (i <= (w-1)) && (j <= (h-1)))
    {
        offset =  j*stride + i ;

        //ssd        float *p_D = D + offset  ;

        float *p_E_r = E_r + offset  ;
        float *p_E_g = E_g + offset  ;
        float *p_E_b = E_b + offset  ;

        float *p_sum_head_r = sum_head_r + offset ;
        float *p_sum_head_g = sum_head_g + offset ;
        float *p_sum_head_b = sum_head_b + offset ;

        float *p_sum_tail_r = sum_tail_r + offset ;
        float *p_sum_tail_g = sum_tail_g + offset ;
        float *p_sum_tail_b = sum_tail_b + offset ;

        float *p_pm = pm + offset  ;
        float *p_mask = mask + offset  ;

        // picking the max in r,g,b
        float tmp_r = ((*p_sum_head_r) - (*p_sum_tail_r)) / n ;
        float tmp_g = ((*p_sum_head_g) - (*p_sum_tail_g)) / n ;
        float tmp_b = ((*p_sum_head_b) - (*p_sum_tail_b)) / n ;
        float tmp_final = tmp_r >= tmp_g ? tmp_r : tmp_g ;
        if(tmp_final<tmp_b)
            tmp_final = tmp_b ;

        //DDD
        //*p_D = tmp_r ;

        (*p_pm) = tmp_final ;

        float s = (*p_E_r) + (*p_E_g) + (*p_E_b) ;
        float B = s/(3*255) ;
        (*p_pm) *= B ;

        //debug
        //*p_pm = tmp_final ;
        //*p_pm = ((*p_sum_head_r) - (*p_sum_tail_r)) / n ;

        *p_mask = *p_pm > 10 ? 255 : 0 ;
    }
}
void checkPersistency_Color(int w, int h,float *E_r,float *E_g,float *E_b,float *sum_head_r,float *sum_head_g,float *sum_head_b,float *sum_tail_r,float *sum_tail_g,float *sum_tail_b, float *mask, float *pm,int count_head, int count_tail,float *D)
{
    int n=count_head-count_tail+1 ;
    static dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    static dim3 numBlocks(w/threadsPerBlock.x,  h/threadsPerBlock.y);
    checkPersistencyKernel_Color<<<numBlocks, threadsPerBlock>>>(E_r,E_g,E_b,sum_head_r,sum_head_g,sum_head_b,sum_tail_r,sum_tail_g,sum_tail_b,mask,pm,n,D);
}
//--------------------------------------------
// -----  detection--accumFlux -------
//--------------------------------------------
__global__ void accumFluxKernel_Grey(int width,int height, int pitch, float *flux,float *E,float *I, int n_frames , float *out_flux_mean, float *out_E_mean, float *out_I_mean)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    int offset ;
    pitch /= sizeof(float) ; // making it proper for later pointer usage
    //if ((i >= 0) && (j >= 0) && (i <= (width-1)) && (j <= (height-1)))
    if ((i >= border) && (j >= border) && (i < (width-border-1)) && (j < (height-border-1)))
    {
        offset =  j*pitch + i ;
        float *p_flux = flux + offset  ;
        float *p_E = E + offset  ;
        //float *p_I = I + offset  ;
        float *p_out_flux_mean = out_flux_mean + offset  ;
        float *p_out_E_mean = out_E_mean + offset  ;
        //ssd        float *p_out_I_mean = out_I_mean + offset  ;
        (*p_out_flux_mean) += (*p_flux)/n_frames ;
        (*p_out_E_mean) += (*p_E)/n_frames ;
        //*p_out_I_mean += (*p_I)/n_frames ;
    }
}
void accumFlux(int width,int height, int pitch, float *flux,float *E,float *I, int n_frames , float *out_flux_mean, float *out_E_mean, float *out_I_mean)
{
    static dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    static dim3 numBlocks(width/threadsPerBlock.x,  height/threadsPerBlock.y);
    accumFluxKernel_Grey<<<numBlocks, threadsPerBlock>>>(width, height, pitch,flux,E,I,n_frames,out_flux_mean,out_E_mean,out_I_mean);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
__global__ void accumFluxKernel_Color(float *flux,float *E,float *Ir,float *Ig,float *Ib, int n_frames , float *out_flux_mean, float *out_E_mean, float *out_Ir_mean,float *out_Ig_mean,float *out_Ib_mean)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;
    int offset ;
    int w= ip.w ;
    int h= ip.h ;
    int stride = ip.stride ;
    if ((i >= 0) && (j >= 0) && (i <= (w-1)) && (j <= (h-1)))
    {
        offset =  j*stride + i ;
        float *p_flux = flux + offset  ;
        float *p_E = E + offset  ;
        float *p_Ir = Ir + offset  ;
        float *p_Ig = Ig + offset  ;
        float *p_Ib = Ib + offset  ;
        float *p_out_flux_mean = out_flux_mean + offset  ;
        float *p_out_E_mean = out_E_mean + offset  ;
        float *p_out_Ir_mean = out_Ir_mean + offset  ;
        float *p_out_Ig_mean = out_Ig_mean + offset  ;
        float *p_out_Ib_mean = out_Ib_mean + offset  ;
        (*p_out_flux_mean) += (*p_flux)/n_frames ;
        (*p_out_E_mean) += (*p_E)/n_frames ;
        (*p_out_Ir_mean) += (*p_Ir)/n_frames ;
        (*p_out_Ig_mean) += (*p_Ig)/n_frames ;
        (*p_out_Ib_mean) += (*p_Ib)/n_frames ;
    }
}

void accumFlux_Color(int w, int h,float *flux,float *E,
                     float *Ir,float *Ig,float *Ib,
                     int n_frames , float *out_flux_mean, float *out_E_mean,
                     float *out_Ir_mean,float *out_Ig_mean,float *out_Ib_mean)
{
    static dim3 threadsPerBlock( BLOCK_SIZE1, BLOCK_SIZE2 );
    static dim3 numBlocks(w/threadsPerBlock.x,  h/threadsPerBlock.y);
    accumFluxKernel_Color<<<numBlocks, threadsPerBlock>>>(flux,E,Ir,Ig,Ib,n_frames,out_flux_mean,out_E_mean,out_Ir_mean,out_Ig_mean,out_Ib_mean);
}
