
#include <PxPhysicsAPI.h>

#include "main.hpp"
#include "util.hpp"
#include "phys.hxx"


using namespace physx;



//####TODO: move this constant to proper place
static const float PED_BODY_DENSITY = 980.0f;	///< Densidad de los peatones. Densidad = Masa/Volumen. Densidad media en humanos = 980 kg/m^3.


static std::vector< PxController* > pedestrians;



static void (*pedestrian_collision_callback__ped_ped)  ( const phys::PedestrianID id, const phys::PedestrianID  idother );
static void (*pedestrian_collision_callback__ped_veh)  ( const phys::PedestrianID id, const phys::VehicleID     idother );
static void (*pedestrian_collision_callback__ped_uveh) ( const phys::PedestrianID id, const phys::UserVehicleID idother );

class PedestrianControllerHitReport : public PxUserControllerHitReport
{

	public:
	
		static PedestrianControllerHitReport & Singleton( void ) {
			static PedestrianControllerHitReport hit_report;
			return hit_report;
		}

		static const uint COUNT     = 1000000;
		static const uint BASE_PED  = COUNT * 1;
		static const uint BASE_VEH  = COUNT * 2;
		static const uint BASE_UVEH = COUNT * 3;
		
		inline static void * GetUserDataFromID( const phys::PedestrianID  idped ) {  return (char*)BASE_PED  + idped.index;  }
		inline static void * GetUserDataFromID( const phys::VehicleID     idveh ) {  return (char*)BASE_VEH  + idveh.index;  }
		inline static void * GetUserDataFromID( const phys::UserVehicleID idveh ) {  return (char*)BASE_UVEH + idveh.index;  }

	public:

		virtual void onShapeHit( const PxControllerShapeHit &hit ) {
			const void *user_data = hit.shape->getActor()->userData;
			if( !user_data ) return;
			const phys::PedestrianID  idped(  (char*)hit.controller->getUserData() - (char*)BASE_PED  );
			const phys::VehicleID     idveh(  (char*)user_data                     - (char*)BASE_VEH  );
			if( idveh.index < COUNT ) {
				DBG_ASSERT( pedestrian_collision_callback__ped_veh );
				pedestrian_collision_callback__ped_veh( idped, idveh );
			} else {
				const phys::UserVehicleID iduveh( (char*)user_data - (char*)BASE_UVEH );
				if( iduveh.index < COUNT ) {
					DBG_ASSERT( pedestrian_collision_callback__ped_uveh );
					pedestrian_collision_callback__ped_uveh( idped, iduveh );
				}
			}
		}
		
		virtual void onControllerHit( const PxControllersHit &hit ) {
			const phys::PedestrianID idped( (char*)hit.controller->getUserData() - (char*)BASE_PED );
			const phys::PedestrianID other( (char*)hit.other->getUserData()      - (char*)BASE_PED );
			DBG_ASSERT( pedestrian_collision_callback__ped_ped );
			if( other.index < COUNT )
				pedestrian_collision_callback__ped_ped( idped, other );
		}

		virtual void onObstacleHit( const PxControllerObstacleHit& hit ) {
			print::Error( "PedestrianControllerHitReport::onObstacleHit: Obstacles!!!" );
		}

	private:
	
		PedestrianControllerHitReport() { }
};


void phys::ped::SetCollisionCallback( void (*cb) ( const phys::PedestrianID, const phys::PedestrianID ) )
{
	pedestrian_collision_callback__ped_ped = cb;
}

void phys::ped::SetCollisionCallback( void (*cb) ( const phys::PedestrianID, const phys::VehicleID ) )
{
	pedestrian_collision_callback__ped_veh = cb;
}

void phys::ped::SetCollisionCallback( void (*cb) ( const phys::PedestrianID, const phys::UserVehicleID ) )
{
	pedestrian_collision_callback__ped_uveh = cb;
}


void phys::ped::Configure( PedestrianID &id, float height, float radius )
{
	static PxMaterial *material_pedestrian = phys::physics->createMaterial( 0.5f, 0.5f, 0.1f );	//####TODO: material_pedestrian->release();
	
	if( (int)id.index >= 0 )
	{
		DBG_ASSERT( id.index < pedestrians.size() );
		PxController *ctrl = pedestrians[id.index];
		
		switch( ctrl->getType() )
		{
			case PxControllerShapeType::eBOX:
				( (PxBoxController*) ctrl )->setHalfHeight( height/2 );
				( (PxBoxController*) ctrl )->setHalfForwardExtent( radius );
				( (PxBoxController*) ctrl )->setHalfSideExtent( radius );
			break;

			case PxControllerShapeType::eCAPSULE:
				( (PxCapsuleController*) ctrl )->setHeight( height );
				( (PxCapsuleController*) ctrl )->setRadius( radius );
			break;

			default:
				print::Error( "phys::ped::Configure: Invalid phys_data" );
		}
	}
	else
	{
		DBG_ASSERT( (int)id.index == -1 );
		id.index = pedestrians.size();
		
		//PxBoxControllerDesc desc;
		//desc.halfHeight        = height / 2;
		//desc.halfSideExtent    = radius;
		//desc.halfForwardExtent = radius;

		PxCapsuleControllerDesc desc;
		desc.height = height;
		desc.radius = radius;		

		desc.stepOffset     =  0.3f;
		desc.density        = PED_BODY_DENSITY;
		desc.material       = material_pedestrian;
		desc.position       = PxExtendedVec3( 0, 0, height/2+radius );
		desc.upDirection    = PxVec3( 0, 0, 1 );
		//desc.volumeGrowth = 1.5f;
		desc.userData       = phys::ped::GetUserDataFromID( id );
		desc.reportCallback = &PedestrianControllerHitReport::Singleton();
		DBG_ASSERT( desc.isValid() );
		
		PxController *ctrl = manager->createController( desc );
		if( !ctrl ) print::Error( "phys::ped::Configure: manager->createController() NULL" );

		scene->removeActor( *ctrl->getActor() );

		phys::filter::Set( phys::filter::OBSTACLE, ctrl->getActor() );
				
		pedestrians.push_back( ctrl );
	}
}


void phys::ped::SetActive( const PedestrianID id, bool active )
{
	DBG_ASSERT( id.index < pedestrians.size() );
	PxRigidDynamic *actor = pedestrians[id.index]->getActor();
	if( active  ) scene->addActor( *actor );
	if( !active ) scene->removeActor( *actor );
}


void phys::ped::SetPosition( const PedestrianID id, const float3 &pos )
{
	DBG_ASSERT( id.index < pedestrians.size() );
	//pedestrians[id.index]->invalidateCache();
	pedestrians[id.index]->setFootPosition( PxExtendedVec3( pos.x, pos.y, pos.z ) );
}


void phys::ped::GetPosition( const PedestrianID id, float3 &pos )
{
	DBG_ASSERT( id.index < pedestrians.size() );
	PxExtendedVec3 xyz = pedestrians[id.index]->getFootPosition();
	pos = float3( xyz.x, xyz.y, xyz.z );
}


void phys::ped::Move( const PedestrianID id, float dt, const float3 &displacement )
{
	static PxControllerFilters filters;
	DBG_ASSERT( id.index < pedestrians.size() );
	const PxVec3 r( displacement.x, displacement.y, displacement.z );
	pedestrians[id.index]->move( r, 0.001f, dt, filters, NULL );
}



void * phys::ped::GetUserDataFromID( const phys::PedestrianID idped ) {
	return PedestrianControllerHitReport::GetUserDataFromID( idped );
}

void * phys::ped::GetUserDataFromID( const phys::VehicleID idveh ) {
	return PedestrianControllerHitReport::GetUserDataFromID( idveh );
}

void * phys::ped::GetUserDataFromID( const phys::UserVehicleID iduveh ) {
	return PedestrianControllerHitReport::GetUserDataFromID( iduveh );
}


void phys::ped::Initialize( void )
{
	phys::ped::Finalize();
}


void phys::ped::Finalize( void )
{
	for( uint i = 0; i < pedestrians.size(); i++ )
		pedestrians[i]->release();
	
	pedestrians.clear();
}


void phys::ped::Update( const float dt )
{
	// nothing to do
}

