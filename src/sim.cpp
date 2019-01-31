/*
**	AUTHORS:
**		Leopoldo Armesto
**		Juan Dols
**		Jaime Molina
**
**	MAGV Simulator by Leopoldo Armesto is licensed under a Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
**	To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/deed.en_GB.
**
**  EMAIL: larmesto@idf.upv.es
**  URL: www.upv.es
*/


/// \cond PRIVATE

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "main.hpp"
#include "sim.hpp"

#include <iostream>

//#include <OgreQuaternion.h>


// CGAL Library for geometric computation
//#include <CGAL/basic.h>
//#include <CGAL/minkowski_sum_2.h>
//#include "bops_linear.h"

//#include <ros/ros.h>
//#include <ros/subscriber.h>
//#include <ros/publisher.h>
//#include <safetrans_msgs/event.h>



//static const nav::veh::Graph *nvg;
//static const nav::ped::Graph *npg;

static sim::Bus			 bus;


float angleDiff( float a, float b) { // angular distance between a and b
	float c=a-b;
	while(c>M_PI) c-=2*M_PI;
	while(c<-M_PI) c+=2*M_PI;
	return c;
}



inline static float Random( float a, float b, int seed0, int seed1=0 ) { // random number between a and b
	const int   r = seed0*3941169319u + seed1*2902958803u;
	const float f = (  ( r ^ (r>>16) ) & 0xFFFFF ) / (float) 0xFFFFF;
	return a + f * ( b - a );
}

const sim::Bus * sim::GetBus( void )	//####BUS
{
	return &bus;
}


#include "db_veh.hpp"	//####BUS

void sim::Initialize( const sim::Config &config )
{
	/* Initialize random seed */
	//srand(1);
	srand(time(NULL)); //Use current time as seed for random generator
	
	phys::Initialize(4);
	world::Initialize();
	//nav::Initialize();
	
	phys::LoadGroundMeshBig( config.collision_mesh );

//	nvg = nav::veh::Load( config.nav_veh_graph );
//	if( !nvg ) ERROR( "sim::Initialize: Can not load vehicle graph '%s'", config.nav_veh_graph );
//
//	npg = nav::ped::Load( config.nav_ped_graph );
//	if( !npg ) ERROR( "sim::Initialize: Can not load pedestrian graph '%s'", config.nav_ped_graph );
//
//	bool ok = nav::sem::Load( config.nav_sem_times );
//	if( !ok ) ERROR( "sim::Initialize: Can not load semaphore times '%s'", config.nav_sem_times );
//
//	vehicles = new sim::Vehicle[ config.num_vehicles ];
//	if( !vehicles ) ERROR( "sim::Initialize: Can not allocate vehicles array" );
//	
//	pedestrians = new sim::Pedestrian [ config.num_pedestrians ];
//	if( !pedestrians ) ERROR( "sim::Initialize: Can not allocate pedestrians array" );
//	
//	objects = new sim::Object [ config.num_objects ];
//	if( !objects ) ERROR( "sim::Initialize: Can not allocate objects array" );
	
	
	//####BUS
	db::veh::Initialize();
	//db::veh::LoadFile( "/home/idf/ros/magv_simulator/vehicles/resources/spreadsheet.xlsx" );
	db::veh::LoadFile( "/home/idf/ros/magv_simulator/magv/sim_mex/src/shared/resources/spreadsheet.xlsx" );
		
	//####BUS START @@@@
//	bus.px = 20.0f;
//	bus.py =  -400.0f;
//	bus.pz =  4.0f;
/*
	// Rotonda
	bus.px =  -5.0f;
	bus.py = -30.0f;
	bus.px = -50.0f;
	bus.py = -15.0f;
	bus.pz =   1.0f;
	bus.dx = +1.0f;
	bus.dy = -1.0f;
*/
/*
	// Final Av. Francia
	bus.px =  755.2f;
	bus.py = -135.3f;
	bus.pz =   0.15f;
	bus.dx = -13.0f;
	bus.dy =  3.65f;
*/
/*
	// Inicio Av. Francia
	bus.px =  60.0f;
	bus.py =  65.0f;
	bus.pz =  0.15f;
	bus.dx = -13.0f;
	bus.dy =  3.65f;
*/
/*
	// Inicio Av. Francia (direccion puerto)
	bus.px = 20.0f;
	bus.py = 45.0f;
	bus.pz = 0.15f;
	bus.dx =  13.0f;
	bus.dy = -3.65f;
*/
/*
	// Semáforo Av. Francia
	bus.px = 374.0f;
	bus.py = -55.5f;
	bus.pz =  0.15f;
	bus.dx =  13.0f;
	bus.dy = -3.65f;
*/
/*
	// Origen
	bus.px = 5.0f;
	bus.py = 0.0f;
	bus.pz = 0.15f;
	bus.dx = 0.0f;
	bus.dy = 0.0f;
*/

	// Origen
	bus.px = 0.0f;
	//bus.px = -10000.0f;
	bus.py = 0.0f;
	bus.pz = 0.15f;
	bus.dx = 0.0f;
	bus.dy = 0.0f;

/*
	// Inicio Paseo Alameda
	bus.px =  0.0f;
	bus.py =-40.0f;
	bus.pz = 0.15f;
	bus.dx =  6.0f;
	bus.dy = -6.0f;
*/
	bus.speed = 0.0f;
	bus.orientation[0] = bus.orientation[1] = bus.orientation[2] = 0.0f;  bus.orientation[3] = 1.0f;
/*	bus.orientation[0] = 0.991445f; bus.orientation[1] = 0.130525f; bus.orientation[2] = 0.0f;  bus.orientation[3] = 0.0f;*/
	bus.length = 8.0f;
	bus.width  = 2.6f;
	bus.height = 2.2f; // 2.0f + wheels
/*	(phys::UserVehicleID&)bus = phys::userveh::Create( "Vehicle Big" );*/
/*	(phys::UserVehicleID&)bus = phys::userveh::Create( "Vehicle Tempus" );*/
	(phys::UserVehicleID&)bus = phys::userveh::Create( "Vehicle EMT" );
	bus.WorldUpdate( bus.px, bus.py );
	phys::userveh::SetPositionDirection( bus, (float3&)bus.px, (float2&)bus.dx );
	
//	// Save the coordinates of both paths
//	double x, y;
//	std::ifstream inFile, inFile2;
//	
//	inFile.open("/home/idf/ros/magv_simulator/magv/simulator/resources/path.txt");
//	while(!inFile.eof()){
//		inFile >> x >> y; // extracts 2 floating point values seperated by whitespace
//		event::pathX.push_back(x);
//		event::pathY.push_back(y);
//		//std::cout << "x: " << x << " y: " << y << std::endl;
//	}
//	inFile2.open("/home/idf/ros/magv_simulator/magv/simulator/resources/path2.txt");
//	while(!inFile2.eof()){
//		inFile2 >> x >> y; // extracts 2 floating point values seperated by whitespace
//		event::path2X.push_back(x);
//		event::path2Y.push_back(y);
//		//std::cout << "x: " << x << " y: " << y << std::endl;
//	}
//	inFile.close();
//	inFile2.close();
	
}


void sim::Finalize( void )
{
	phys::Finalize();
	world::Finalize();
	//nav::Finalize();

}


//bool sim::Update( const float dt )
bool sim::Update( const float dt)//, const safetrans_msgs::event msg )
{	
	phys::Update( dt );
	//nav::Update( dt );
	
	//####BUS
	phys::userveh::GetPositionDirectionOrientationSpeed( bus, (float3&)bus.px, (float2&)bus.dx, (short4&)bus.orientation[0], bus.speed );
	bus.WorldUpdate( bus.px, bus.py );	//####BUS2
	
	return true;
}



