/*
**	AUTHOR:
**		Vicent Girbes Juan
**
**  EMAIL: vgirbes@idf.upv.es
**  URL: www.upv.es
*/

#ifndef SIM_MEX_H
#define SIM_MEX_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "sim.hpp"
//#include "vis.hpp"

#include "math.hpp"

#include <iostream>
 
//static const float BUS_ENVIRONMENT_RADIUS = 32.0f;
static const float RATE = 100.0f;

class sim_mex
{
public:
    sim_mex();
    ~sim_mex();

    void Initialize( void );
    void Finalize( void );
    void Sleep( float );
    void Loop( float steer, float accel, float brake, float handbrake, float time = 5.0f, float rate = 100.0f );
    void Run( float steer, float accel, float brake, float handbrake, float time = 5.0f, float rate = 100.0f );
    bool GetOutputs( void );
    bool SetInputs( float steer_value, float accel_value, float brake_value, float handbrake );
    
private:
	float3 pose_pos;
	float4 pose_ori;
	float3 twist_linear;
	float3 twist_angular;
	float3 accel_linear;
	float engine_speed;
	float wheels_speed[8]; //Only vehicles with at most 8 wheels are allowed
    int wheels_num;
	unsigned int gear_current, gear_target;
	float gear_ratio;
};

#endif // SIM_MEX_H
