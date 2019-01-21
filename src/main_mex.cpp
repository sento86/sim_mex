/*
**	AUTHOR:
**		Vicent Girbes Juan
**
**  EMAIL: vgirbes@idf.upv.es
**  URL: www.upv.es
*/

#include "sim_mex.h"

#include "mex.h"

void MainFcn( float steer, float accel, float brake, float handbrake, float time, float rate ){
    
    //sim_mex *p = new sim_mex();
    //p->Run(steer, accel, brake, handbrake, time, rate);
    //delete p;
    sim_mex p;
    p.Run(steer, accel, brake, handbrake, time, rate);
}

/* The gateway function. */ 
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]) {

    int nrhs_max = 6;
    
    /* Check for proper number of arguments */
    if(nrhs != nrhs_max) {
        mexErrMsgIdAndTxt("MATLAB:mexcpp:nargin",
                          "MEXCPP requires 6 input arguments.");
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

    MainFcn(*steer, *accel, *brake, *handbrake, *time, *rate);

}

