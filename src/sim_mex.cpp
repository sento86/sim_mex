/*
**	AUTHOR:
**		Vicent Girbes Juan
**
**  EMAIL: vgirbes@idf.upv.es
**  URL: www.upv.es
*/

#include "sim_mex.h"

sim_mex::sim_mex()
{
    Initialize();
}

sim_mex::~sim_mex()
{    
 	Finalize();
}

void sim_mex::Initialize( void )  //####FIXME: hardcoded stuff
{
    std::cout << "Initialize" << std::endl;
  
	//#define BASE "/home/idf/ros/magv_simulator/magv/simulator/data/"
	#define BASE "/home/idf/ros/magv_simulator/magv/sim_mex/data/"
	
	sim::Config sim_config;
	//sim_config.collision_mesh   = BASE"valencia_collision.obj";
 	sim_config.collision_mesh   = BASE"empty.obj";
// 	sim_config.nav_veh_graph	= BASE"nav_veh_graph.dat";
// 	sim_config.nav_ped_graph	= BASE"nav_ped_graph.dat";
// 	sim_config.nav_sem_times	= BASE"nav_sem_times.txt";
// 	sim_config.num_vehicles		= VEHICLES;
// 	sim_config.num_pedestrians	= PEDESTRIANS + PEDESTRIANS_CROSSWALK + PEDESTRIANS_BUSSTOP + PEDESTRIANS_CORNER;
// 	sim_config.num_objects		= OBJECTS;
// 	sim_config.num_events		= OBJECTS + VEHICLES_RANDOM + PEDESTRIANS_RANDOM + 1;
 	sim::Initialize( sim_config );
}
 
void sim_mex::Finalize( void )
{
    std::cout << "Finalize" << std::endl;
	
	sim::Finalize();  
}

void sim_mex::Sleep( float dt )
{
    usleep(dt*1000000);
}

std::vector <std::vector<double> > sim_mex::Loop( std::vector <std::vector<double> > inputs)
{
	std::vector <std::vector<double> > outputs;
	std::vector<double> speed;
    std::cout << "Loop" << std::endl;
   
 	double dt, time_old, time_a, time_b, time_c, time_d, time_e;
 	bool ok;
 	
 	time_old = GetTime();
    
    int i_max = ceil(inputs[4][0]*inputs[5][0]-1);
 	
 	for(int i = 0; i <= i_max; i++)
    {
 		time_a = GetTime();
 
 		//####FIXME: choose fixed or variable frametime
 		dt = 1/inputs[5][0];		            // fixed frame time
 		//dt = ( time_a - time_old );	// variable frame time
 		if( dt < 0.0 ) dt = 0.0;
 		time_old = time_a;

 		ok = sim::Update( dt );
 		if( !ok ) break;
 		
 		time_b = GetTime();
 
 		ok = GetOutputs();
 		if( !ok ) break;
 		
 		time_c = GetTime();
 
 		ok = SetInputs( inputs[0][i], inputs[1][i], inputs[2][i], inputs[3][i] );
 		if( !ok ) break;
		speed.push_back(twist_linear.x);
 		std::cout << "v=" << twist_linear.x << "m/s, " << std::endl;
		std::cout << "accel=" << inputs[1][i] << std::endl;
		std::cout << "brake=" << inputs[2][i] << std::endl;
		std::cout << "i=" << i << std::endl;
 		
 		time_d = GetTime();
 
 		//Sleep( dt );
 		
 		time_e = GetTime();
 
 		if( 1 ) { //####PROFILING
 			static double t_sim, t_get, t_set, t_sleep;
 			static int    t_count, t_count2;
 			t_sim   += time_b - time_a;
 			t_get   += time_c - time_b;
 			t_set   += time_d - time_c;
 			t_sleep += time_e - time_d;
 			if( t_count++ >= 100 ) {
 				//printf("TIME: sim=%.3fms, read=%.3fms, sleep=%.3fms\n", t_sim*1000/t_count, t_read*1000/t_count, t_sleep*1000/t_count );
 				std::cout << "***TIME: " ;
 				std::cout << "sim=" << t_sim*1000/t_count << "ms, ";
 				std::cout << "get=" << t_get*1000/t_count << "ms, ";
 				std::cout << "set=" << t_set*1000/t_count << "ms, ";
 				std::cout << "sleep=" << t_sleep*1000/t_count << "ms" << std::endl;
 				t_sim = t_get = t_set = t_sleep = 0.0;
 				t_count = 0;
 			}
 			if( t_count2++ >= 10 ) {
 				std::cout << "***OUTPUT: " ;
 				std::cout << "v=" << twist_linear.x << "m/s, ";
 				std::cout << "w=" << twist_angular.z << "rad/s" << std::endl;  
 				t_count2 = 0;              
            }
 		}
 	}
	outputs.push_back(speed);
	return outputs;
 }

bool sim_mex::SetInputs( float steer_value, float accel_value, float brake_value, float handbrake_value )
{
 	enum { AXIS_STEER=0, AXIS_ACCEL, AXIS_BRAKE, AXES_SIZE };							//####TODO: use shared header file
 	enum { BUTTON_GEAR_NEUTRAL=0, BUTTON_GEAR_REAR, BUTTON_GEAR_DIRECT, BUTTONS_SIZE };	//####TODO: use shared header file

 	if( steer_value < -1.0f ) steer_value = -1.0f;
 	if( steer_value > +1.0f ) steer_value = +1.0f;
 	
 	if( accel_value < 0.0f ) accel_value = 0.0f;
 	if( accel_value > 1.0f ) accel_value = 1.0f;

 	if( brake_value < 0.01f ) brake_value = 0.0f;
 	if( brake_value > 1.0f ) brake_value = 1.0f;

 	if( handbrake_value < 0.01f ) handbrake_value = 0.0f;
 	if( handbrake_value > 1.0f ) handbrake_value = 1.0f;
 	
 	const sim::Bus &bus = *sim::GetBus();
 	phys::userveh::ActionMode( bus, true, false );
 	phys::userveh::ActionAutobox( bus, true );
 	//phys::userveh::ActionAutobox( bus, false );
 	phys::userveh::ActionSteer( bus, steer_value );
 	phys::userveh::ActionAccel( bus, accel_value );
 	phys::userveh::ActionBrake( bus, brake_value );
 	phys::userveh::ActionHandbrake( bus, handbrake_value );
 	phys::userveh::ActionGear( bus, 2, true );
 	//if( msg->buttons[BUTTON_GEAR_REAR   ] ) phys::userveh::ActionGear( bus, 0, true );
 	//if( msg->buttons[BUTTON_GEAR_NEUTRAL] ) phys::userveh::ActionGear( bus, 1, true );
 	//if( msg->buttons[BUTTON_GEAR_DIRECT ] ) phys::userveh::ActionGear( bus, 2, true );
  
    return true;
}

bool sim_mex::GetOutputs( void )
{
//    const sim::Bus *bus = sim::GetBus();
//
//	phys::userveh::GetPoseTwist( *bus, pose_pos, pose_ori, twist_linear, twist_angular, true ); //####BUS odometry
//	phys::userveh::GetAcceleration( *bus, accel_linear, true );                                 //####BUS acceleration
//	phys::userveh::GetEngineRotationSpeed( *bus, engine_speed );                                //####BUS engine speed
//	wheels_num = phys::userveh::GetWheelRotationSpeed( *bus, wheels_speed, 8 );                 //####BUS wheels speed
//	phys::userveh::GetTransmission( *bus, gear_current, gear_target, gear_ratio );              //####BUS transmission

 	const sim::Bus &bus = *sim::GetBus();

	phys::userveh::GetPoseTwist( bus, pose_pos, pose_ori, twist_linear, twist_angular, true ); //####BUS odometry
	phys::userveh::GetAcceleration( bus, accel_linear, true );                                 //####BUS acceleration
	phys::userveh::GetTransmission( bus, gear_current, gear_target, gear_ratio );              //####BUS transmission
	phys::userveh::GetEngineRotationSpeed( bus, engine_speed );                                //####BUS engine speed
	wheels_num = phys::userveh::GetWheelRotationSpeed( bus, wheels_speed, 8 );                 //####BUS wheels speed
    
	//printf(";%f", bus.speed);
    //std::cout << ";" << bus.speed << std::endl;
	
	return true;
}

std::vector <std::vector<double> > sim_mex::Run( std::vector <std::vector<double> > inputs  )
{
	std::vector <std::vector<double> > outputs;
 	Initialize();
    
 	outputs=Loop(inputs);
    
 	Finalize();
	return outputs;
}

int main( int argc, char **argv )
{
    float steer = 0.0f;
    float accel = 1.0f;
    float brake = 0.0f;
    float handbrake = 0.0f;
    float time = 10.0f;
    float rate = 100.0f;

	std::vector<double> time(1,10);
	//time.push_back(6);
	std::vector<double> rate(1,100);
	//rate.push_back(1);
	double total = time[0]*rate[0];
	//$(IntermediateDirectory)/lib$(ProjectName)
	
	std::vector<double> halfzeros(total/2,0);
	std::vector<double> halfones(total/2,1);
	
	std::vector<double> steer(total,0);
	//steer.push_back(0); steer.push_back(0); steer.push_back(0);steer.push_back(0);steer.push_back(0);steer.push_back(0);
	//std::vector<double> accel(total,1);
	std::vector<double> accel(halfones);
	accel.insert(accel.end(), halfzeros.begin(), halfzeros.end());
	//accel.push_back(1.0); accel.push_back(1.0); accel.push_back(1.0);accel.push_back(0);accel.push_back(0);accel.push_back(0);
	std::vector<double> brake(total,0);
	//brake.push_back(0); brake.push_back(0); brake.push_back(1.0);brake.push_back(1.0);brake.push_back(1.0);brake.push_back(1.0);
	std::vector<double> handbrake(total,0);
	//handbrake.push_back(0); handbrake.push_back(0); handbrake.push_back(0);handbrake.push_back(0);handbrake.push_back(0);handbrake.push_back(0);

	std::vector< std::vector<double> > inputs;
	inputs.push_back(steer);inputs.push_back(accel);inputs.push_back(brake);inputs.push_back(handbrake);inputs.push_back(time);inputs.push_back(rate);

    sim_mex p;
    p.Run(inputs);
    
	return 0;
}