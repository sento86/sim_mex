
#ifndef __PHYS_HPP__
#define __PHYS_HPP__


#include "main.hpp"


namespace phys
{
	
	struct GeomsCallbacks {
		enum Type { NONE=0, WHEEL, PLANE, BOX, SPHERE, CYLINDER, CAPSULE, HFIELD, CONVEX, MESH, _SIZE };
		virtual void OnCounts ( int total, int counts[_SIZE] ) { };
		virtual void OnHField ( int m ) { }; //####TODO
		virtual void OnMesh   ( int m, bool convex, int num_points, const float3 points[], const char4 normals[], int index_size, int num_indexes, int num_polygons, const void *polygons ) { };
		virtual void OnGeom   ( int t, int type, int n, int m=-1 ) { };
	};


	void Initialize( const int threads=1 );
	void Finalize( void );

	void Update( const float dt );


	void LoadGroundMeshBig( const char *filename );
	void LoadGroundMeshes( const char *filename, GeomsCallbacks *callbacks=NULL );
	int GetGroundMatrices( matrix44 &mat, int num, matrix44 mats[] );


	struct UserVehicleID {
		UserVehicleID( unsigned int index=-1 ) : index(index) { }
		unsigned int index;
	};

/*	struct PedestrianID {
		PedestrianID( unsigned int index=-1 ) : index(index) { }
		unsigned int index;
	};*/

/*	struct VehicleID {
		VehicleID( unsigned int index=-1 ) : index(index) { }
		unsigned int index;
	};*/
	
	
	namespace userveh {

		UserVehicleID Create( const char *vehicle_name, GeomsCallbacks *callbacks=NULL );
		UserVehicleID Delete( const UserVehicleID id );

		void SetPositionDirection( const UserVehicleID id, const float3 &pos, const float2 &dir );
		void GetPositionDirectionOrientationSpeed( const UserVehicleID id, float3 &pos, float2 &dir, short4 &ori, float &speed );
		void GetPoseTwist( const UserVehicleID id, float3 &pose_pos, float4 &pose_ori, float3 &twist_linear, float3 &twist_angular, bool local_twist=false );

		void GetAcceleration( const UserVehicleID id, float3 &linear_accel, bool local_accel );
		int GetWheelRotationSpeed( const UserVehicleID id, float *wheels_speed, uint wheels_max );
		void GetEngineRotationSpeed( const UserVehicleID id, float &engine_speed );
		void GetTransmission( const UserVehicleID id, uint &gear_current, uint &gear_target, float &gear_ratio );

		int GetMatrices( const UserVehicleID id, matrix44 &mat, int num, matrix44 mats[] );
		
		void ActionMode     ( const UserVehicleID id, const bool analog, const bool smoothing );
		void ActionAccel    ( const UserVehicleID id, const float input_accel );
		void ActionSteer    ( const UserVehicleID id, const float input_steer );
		void ActionBrake    ( const UserVehicleID id, const float input_brake );
		void ActionHandbrake( const UserVehicleID id, const float input_handbrake );
		void ActionGear     ( const UserVehicleID id, const int   input_gear, const bool target=false );
		void ActionAutobox  ( const UserVehicleID id, const bool  enable );

	}

/*	namespace ped {

		void SetCollisionCallback( void (*cb) ( const phys::PedestrianID id, const phys::PedestrianID  idother ) );
		void SetCollisionCallback( void (*cb) ( const phys::PedestrianID id, const phys::VehicleID     idother ) );
		void SetCollisionCallback( void (*cb) ( const phys::PedestrianID id, const phys::UserVehicleID idother ) );

		void Configure( PedestrianID &id, float height, float radius );

		void SetActive( const PedestrianID id, bool active=true );

		void SetPosition( const PedestrianID id, const float3 &pos );
		void GetPosition( const PedestrianID id, float3 &pos );
		
		void Move( const PedestrianID id, float dt, const float3 &displacement );
		
	}
	
	namespace veh {
		
		void Configure( VehicleID &id, const float3 &size );

		void SetActive( const VehicleID id, bool active=true );
		
		void SetPositionDirection( const VehicleID id, const float3 &pos, const float2 &dir );
		void GetPositionDirectionOrientationSpeed( const VehicleID id, float3 &pos, float2 &dir, short4 &ori, float &speed );

		void Move( const VehicleID id, float dt, float accel, float steer );
		
	}*/

}


#endif  // __PHYS_HPP__
