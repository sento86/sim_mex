/*
**	AUTHOR:
**		Vicent Girbes Juan
**
**  EMAIL: vgirbes@idf.upv.es
**  URL: www.upv.es
*/

#include "sim_mex.h"

#include "mex.h"

//static sim_mex p;
sim_mex *p;

void InitializeFcn( ){
    p = new sim_mex();
}

void FinalizeFcn( ){
    delete p;
    p = NULL;
}

void MainFcn( float steer, float accel, float brake, float handbrake, float time, float rate ){
    
    //sim_mex *p = new sim_mex();
    //p->Run(steer, accel, brake, handbrake, time, rate);
    //delete p;
    //p.Run(steer, accel, brake, handbrake, time, rate);
    p->Run(steer, accel, brake, handbrake, time, rate);
}

/* The gateway function. */ 
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]) {

    int nrhs_max = 7;
    
    /* Check for proper number of arguments */
    if(nrhs != nrhs_max) {
        mexErrMsgIdAndTxt("MATLAB:mexcpp:nargin",
                          "MEXCPP requires 7 input arguments.");
    }
    if(nlhs != 0) {
        mexErrMsgIdAndTxt("MATLAB:mexcpp:nargout",
                          "MEXCPP requires 0 output argument.");
    }

    /* Check if the input is of proper type */
    for (int i = 0; i < nrhs; i++) {
        if(!mxIsDouble(prhs[i])){    // or not scalar
            mexErrMsgIdAndTxt("MATLAB:mexcpp:typeargin",
                              "First argument has to be double scalar.");
        }
    }
    
    /* Acquire pointers to the input data */
    double* steer = mxGetPr(prhs[0]);
    double* accel = mxGetPr(prhs[1]);
    double* brake = mxGetPr(prhs[2]);
    double* handbrake = mxGetPr(prhs[3]);
    double* time  = mxGetPr(prhs[4]);
    double* rate  = mxGetPr(prhs[5]);
    double* mode  = mxGetPr(prhs[6]);

    if(*mode==1.0)
        InitializeFcn();
    else if(*mode==0.0)
        MainFcn(*steer, *accel, *brake, *handbrake, *time, *rate);
    else if(*mode==-1.0)
        FinalizeFcn();
    else
        std::cout << "Wrong mode!" << std::endl;

}

