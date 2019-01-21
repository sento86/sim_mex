#include "mex.h"

/* nlhs	N�mero de argumentos de salida (lado izquierdo), o el tama�o de la matriz plhs .
   plhs	Matriz de argumentos de salida.
   nrhs	N�mero de argumentos de entrada (lado derecho), o el tama�o del array prhs .
   prhs	Array de argumentos de entrada. */

/* Input Arguments */

#define	NUM_1	prhs[0]
#define	NUM_2	prhs[1]


/* Output Arguments */

#define	RES	plhs[0]


/* function returning the max between two numbers */
void maxCfunction(double num1[], double num2[], double result[]) {

   /* local variable declaration */
   /*int result;
   int num2 = 5;*/
  if(num1[0]>=num2[0]){
      result[0]=num1[0];
  }else{
      result[0]=num2[0];
  }
}
/* The gateway function */ 
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {
    /* variable declarations here */
    double *result;
    double *num1,*num2; 
    
        if (nrhs != 2) { 
	    mexErrMsgIdAndTxt( "MATLAB:yprime:invalidNumInputs",
                "Two input arguments required."); 
    } else if (nlhs > 1) {
	    mexErrMsgIdAndTxt( "MATLAB:yprime:maxlhs",
                "Too many output arguments."); 
    } 
    
    RES = mxCreateDoubleMatrix((mwSize)1,(mwSize)1,mxREAL);
    result = mxGetPr(RES);
    num1 = mxGetPr(NUM_1); 
    num2 = mxGetPr(NUM_2);
    /*mxGetData*/

    
    /* code here */ 
    maxCfunction(num1,num2,result); 
    return;
} 