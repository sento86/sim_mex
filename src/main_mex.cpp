/*
**	AUTHOR:
**		Vicent Girbes Juan
**
**  EMAIL: vgirbes@idf.upv.es
**  URL: www.upv.es
*/
#include <vector>
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

std::vector <std::vector<double> >  MainFcn( std::vector <std::vector<double> > inputs ){
    std::vector <std::vector<double> > outputs;
    //sim_mex *p = new sim_mex();
    //p->Run(steer, accel, brake, handbrake, time, rate);
    //delete p;
    //p.Run(steer, accel, brake, handbrake, time, rate);
    outputs=p->Run(inputs);
	return outputs;
}

mxArray * getMexArray(const std::vector<double>& v){
    mxArray * mx = mxCreateDoubleMatrix(1,v.size(), mxREAL);
    std::copy(v.begin(), v.end(), mxGetPr(mx));
    return mx;
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
    
	std::vector< std::vector<double> > inputs;
	for( int i = 0; i < nrhs; i++ ) {
		double *ptr = mxGetPr(prhs[i]);
		mwSize n = mxGetNumberOfElements(prhs[i]);
		std::vector<double> t( ptr, ptr+n );
		inputs.push_back( t );
	}
	
	mwSize n = mxGetNumberOfElements(prhs[0]);
	std::vector <std::vector<double> > outputs;
	plhs[0] = mxCreateDoubleMatrix((mwSize)1,(mwSize)n,mxREAL);
	std::vector<double> speed;
    double* mode  = mxGetPr(prhs[6]);
	
    if(*mode==1.0){
        InitializeFcn();
    }else if(*mode==0.0){
        outputs=MainFcn(inputs);
		speed = outputs[0];
		plhs[0] = getMexArray(speed);
	}else if(*mode==-1.0){
        FinalizeFcn();
    }else{
        std::cout << "Wrong mode!" << std::endl;
	}
}

