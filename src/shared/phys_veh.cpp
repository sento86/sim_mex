
#include <PxPhysicsAPI.h>
#include <vehicle/PxVehicleSDK.h>
#include <vehicle/PxVehicleUtil.h>

#include "main.hpp"
#include "util.hpp"
#include "phys.hxx"


using namespace physx;




template < int SIZE, int MAX_RAYCASTS=4*SIZE >
class VehicleBatchRaycaster
{
	public:
	
		VehicleBatchRaycaster() : batch_query(0)
		{
			DBG_ASSERT( MAX_RAYCASTS >= 4*SIZE );
		}

		virtual ~VehicleBatchRaycaster()
		{
			this->Finalize();
		}

		void Initialize( void )
		{
			DBG_ASSERT( phys::scene && !this->batch_query );
			
			PxBatchQueryDesc batch_query_desc( MAX_RAYCASTS, 0, 0 );
			batch_query_desc.queryMemory.userRaycastResultBuffer = this->raycast_results;
			batch_query_desc.queryMemory.userRaycastTouchBuffer  = this->raycast_hits;
			batch_query_desc.queryMemory.raycastTouchBufferSize  = MAX_RAYCASTS;
			batch_query_desc.preFilterShader = phys::filter::GetWheelRaycastPreFilter();
			DBG_ASSERT( batch_query_desc.isValid() );
			
			this->batch_query = phys::scene->createBatchQuery( batch_query_desc );

			for( int i = 0; i < SIZE; i++ ) {
				this->vehicle_wheel_query[i].nbWheelQueryResults = 4;
				this->vehicle_wheel_query[i].wheelQueryResults = &this->wheel_query_results[i*4];
			}
		}
		
		void Finalize( void )
		{
			if( this->batch_query )
				this->batch_query->release();
			this->batch_query = NULL;
		}

		void UpdateVehicles( const float dt, const int num, PxVehicleNoDrive **vehicles )
		{
			DBG_ASSERT( phys::scene && this->batch_query );
			
			const auto &friction_pairs = phys::material::GetFrictionPairs();
			
			int size, total = 0;
			while( total < num )
			{
				for( size = 0; size < SIZE && total < num; total++ )
					if( vehicles[total] && vehicles[total]->getRigidDynamicActor()->getScene() )
						this->vehicles[size++] = vehicles[total];
				
				for( int i = 0; i < size; i++ ) DBG_ASSERT( this->vehicles[i]->mWheelsSimData.getNbWheels() == 4 );
				
				PxVehicleSuspensionRaycasts( this->batch_query, size, this->vehicles, 4*size, this->raycast_results );
				
				PxVehicleUpdates( dt, phys::scene->getGravity(), friction_pairs, size, this->vehicles, this->vehicle_wheel_query );
			}
		}
		
	private:
	
		PxBatchQuery				*batch_query;
		PxVehicleWheels				*vehicles[SIZE];
		PxVehicleWheelQueryResult	vehicle_wheel_query[SIZE];
		PxWheelQueryResult			wheel_query_results[4*SIZE];
		PxRaycastQueryResult		raycast_results[MAX_RAYCASTS];
		PxRaycastHit				raycast_hits[MAX_RAYCASTS];		
};





//####TODO: move this constant to proper place
static const float VEH_CHASIS_DENSITY = 250.0f;	///< Densidad de la carroceria de los vehículos.


static VehicleBatchRaycaster< 32 > vehicle_batch_raycaster;

static std::vector< PxVehicleNoDrive* > vehicles;



void phys::veh::Configure( VehicleID &id, const float3 &size )
{
	static PxMaterial *material_vehicle = phys::physics->createMaterial( 0.5f, 0.5f, 0.1f );	//####TODO: material_vehicle->release();

	if( (int)id.index >= 0 )
	{
		DBG_ASSERT( id.index < vehicles.size() );

		PxRigidDynamic *actor = vehicles[id.index]->getRigidDynamicActor();

		PxBoxGeometry box( size.x/2, size.y/2, size.z/2 );
		PxShape *shape = NULL;
		actor->getShapes( &shape, 1 );
		shape->setGeometry( box );
	}
	else
	{
		DBG_ASSERT( (int)id.index == -1 );
		id.index = vehicles.size();
		
		PxTransform   pos( PxVec3( 0.0f, 0.0f, 0.0f ) );
		PxBoxGeometry box( size.x/2, size.y/2, size.z/2 );
		PxRigidDynamic *actor = PxCreateDynamic( *physics, pos, box, *material_vehicle, VEH_CHASIS_DENSITY );

		phys::filter::Set( phys::filter::CHASSIS, actor );

		actor->userData = phys::ped::GetUserDataFromID( id );

		PxVehicleWheelsSimData *wheels_data = PxVehicleWheelsSimData::allocate( 4 );
		for( int i = 0; i < 4; i++ )
		{
			bool is_front = i < 2;
			bool is_right = i % 2;

			PxVehicleWheelData wheel;
			wheel.mRadius = 0.414192;
			wheel.mMass   = 20.00000;
			wheel.mWidth  = 0.344028;
			wheel.mMOI    = 0.5f * wheel.mMass * wheel.mRadius * wheel.mRadius;
			wheel.mMaxBrakeTorque = 1e6f;
			wheel.mMaxSteer       = ( is_front ? 0.49f*PxPi : 0.0f );

			PxVehicleTireData tire;
			tire.mType = 1;	//####TODO: TIRE_TYPE_SLICKS = 1

			PxVehicleSuspensionData susp;
			susp.mMaxCompression   = 0.3f;
			susp.mMaxDroop         = 0.1f;
			susp.mSpringStrength   = 35000.0f;
			susp.mSpringDamperRate = 4500.0f;
			susp.mSprungMass = actor->getMass() * 0.5f * ( is_front ? 0.5f : 0.5f );

			float xoffset = ( is_front ? +1 : -1 ) * ( size.x/2 - 2*wheel.mRadius );			// x axis -> length
			float yoffset = ( is_right ? +1 : -1 ) * ( size.y/2 - wheel.mWidth/2 - 0.01f );		// y axis -> width

			wheels_data->setWheelShapeMapping( i, -1 );
			wheels_data->setSuspensionData( i, susp );
			wheels_data->setWheelData( i, wheel );
			wheels_data->setTireData( i, tire );
			wheels_data->setWheelCentreOffset      ( i, PxVec3( xoffset, yoffset, -size.z/2 ) );
			wheels_data->setTireForceAppPointOffset( i, PxVec3( xoffset, yoffset,     -0.3f ) );
			wheels_data->setSuspForceAppPointOffset( i, PxVec3( xoffset, yoffset,     -0.3f ) );
			wheels_data->setSuspTravelDirection( i, PxVec3(0,0,-1) );
		}
		
		PxVehicleNoDrive *vehicle = PxVehicleNoDrive::create( physics, actor, *wheels_data );
		
		wheels_data->free();

		//scene->addActor( *actor );
		
		vehicles.push_back( vehicle );
	}
}


void phys::veh::SetActive( const VehicleID id, bool active )
{
	DBG_ASSERT( id.index < vehicles.size() );
	PxRigidDynamic *actor = vehicles[id.index]->getRigidDynamicActor();
	if( active  ) scene->addActor( *actor );
	if( !active ) scene->removeActor( *actor );
}


void phys::veh::SetPositionDirection( const VehicleID id, const float3 &pos, const float2 &dir )
{
	PxTransform xform( PxVec3(pos.x,pos.y,pos.z), PxQuat( atan2(dir.y,dir.x) , PxVec3(0,0,1) ) );
	DBG_ASSERT( id.index < vehicles.size() );
	vehicles[id.index]->getRigidDynamicActor()->setGlobalPose( xform );
	vehicles[id.index]->getRigidDynamicActor()->setLinearVelocity( PxVec3(0,0,0) );
	vehicles[id.index]->getRigidDynamicActor()->setAngularVelocity( PxVec3(0,0,0) );	
}


void phys::veh::GetPositionDirectionOrientationSpeed( const VehicleID id, float3 &pos, float2 &dir, short4 &ori, float &speed )
{
	DBG_ASSERT( id.index < vehicles.size() );
	const PxTransform &xform = vehicles[id.index]->getRigidDynamicActor()->getGlobalPose();
	PxVec3 forward = xform.q.getBasisVector0();
	pos   = float3( xform.p.x, xform.p.y, xform.p.z );
	dir   = float2( forward.x, forward.y );
	ori   = short4( xform.q.x, xform.q.y, xform.q.z, xform.q.w );
	speed = vehicles[id.index]->computeForwardSpeed();
}


void phys::veh::Move( const VehicleID id, const float dt, const float accel, const float steer )
{
	DBG_ASSERT( id.index < vehicles.size() );
	
	// torque  =  wheel_radius * force  =  wheel_radius * vehicle_mass * accel    //####FIXME: verify formula
	// Since vehicles have different wheel_radius and vehicle_mass, using constant wheel_radius and
	// constant vehicle_mass in the formula is like having variable vehicle accelerations.	
	const float torque = (0.4f*10.0f*VEH_CHASIS_DENSITY) * accel;

	PxVehicleNoDrive *veh = vehicles[id.index];
	for( int i = 0; i < 2; i++ ) {
		//veh->setBrakeTorque( i, ( torque < 0.0f ? -torque : 0.0f ) );
		veh->setBrakeTorque( i, ( torque < 0.0f ? -torque : 0.0f ) );
		veh->setDriveTorque( i, ( torque > 0.0f ? +torque : 0.0f ) );
		//veh->setDriveTorque( i, torque );
		veh->setSteerAngle ( i, steer );
	}

//printf( "accel = %9.3f  torque= %9.3f  speed = %9.3f\n", accel, torque, veh->computeForwardSpeed() );
}



void phys::veh::Initialize( void )
{
	phys::veh::Finalize();
	
	vehicle_batch_raycaster.Initialize();
}


void phys::veh::Finalize( void )
{
	vehicle_batch_raycaster.Finalize();

	for( uint i = 0; i < vehicles.size(); i++ )
		vehicles[i]->free();
	
	vehicles.clear();
}


void phys::veh::Update( const float dt )
{
	vehicle_batch_raycaster.UpdateVehicles( dt, vehicles.size(), &vehicles[0] );
}

