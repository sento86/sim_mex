
#ifndef __DB_VEH_HPP__
#define __DB_VEH_HPP__


#include <vector>


namespace db
{

namespace veh
{
	void Initialize( void );
	void Finalize( void );
	
	void LoadFile( const char *filename_xlsx );
	
	
	struct Surface {
		enum { TARMAC=0, MUD, GRASS, SNOW, ICE, NUM_SURFACES };
		float 		friction_static;
		float 		friction_dynamic;
		float 		restitution;
	};

	struct Suspension {
		const char *name;
		float		spring_strength;
		float		spring_damper_rate;
		float		max_compression;
		float		max_droop;
		float		camber_at_rest;
		float		camber_at_max_compression;
		float		camber_at_max_droop;
	};
	
	struct Wheel {
		const char *name;
		const char *file_visual;
		float		radius;
		float		mass;
		float		width;
		float		moment_of_inertia;
		float		max_dampling_rate;
		float		toe_angle;
		float		max_brake_torque;
		float		max_handbrake_torque;
	};

	struct Tire {
		const char *name;
		float		lateral_stiffness_x;
		float		lateral_stiffness_y;
		float		longitudinal_stiffness;
		float		camber_stiffness;
		float		friction[ Surface::NUM_SURFACES ];
	};

	struct Wheelset {
		const char *name;
		const char *name_suspension[2];
		const char *name_wheel[2];
		const char *name_tire[2];
		float		max_steer;
		float		ackermann_accuracy;
	};

	struct Chassis {
		const char *name;
		const char *file_collision;
		const char *file_visual;
		float		mass;
	};

	struct Engine {
		const char *name;
		float		moment_of_inertia;
		float		peack_torque;
		float		max_omega;
		float		damping_rate_full;
		float		damping_rate_zero_engaged;
		float		damping_rate_zero_disengaged;
	};

	struct Differential {
		const char *name;
		float		split;
		float		split_front;
		float		split_rear;
		float		bias_center;
		float		bias_front;
		float		bias_rear;
	};

	struct Clutch {
		const char *name;
		float		strength;
	};

	struct Gearbox {
		static const int MAX_GEARS = 10;
		const char *name;
		float		switch_time;
		float		final_ratio;
		float		autobox_latency;			// autobox
		int			num_gears;		
		float		ratios[MAX_GEARS];
		float		up_ratios[MAX_GEARS];		// autobox
		float		down_ratios[MAX_GEARS];		// autobox
	};


	struct Vehicle {
		static const int MAX_WHEELSETS = 4;
		const char *name;
		const char *name_chassis;
		const char *name_engine;
		const char *name_differential;
		const char *name_clutch;
		const char *name_gearbox;
		const char *name_wheelset[MAX_WHEELSETS];
	};


	extern const std::vector< Surface      > &surfaces;
	extern const std::vector< Suspension   > &suspensions;
	extern const std::vector< Wheel        > &wheels;
	extern const std::vector< Tire         > &tires;
	extern const std::vector< Wheelset     > &wheelsets;
	extern const std::vector< Chassis      > &chassis;
	extern const std::vector< Engine       > &engines;
	extern const std::vector< Differential > &differentials;
	extern const std::vector< Clutch       > &clutches;
	extern const std::vector< Gearbox      > &gearboxes;
	extern const std::vector< Vehicle      > &vehicles;

}  // namespace veh

}  // namespace db


#endif  // __DB_VEH_HPP__
