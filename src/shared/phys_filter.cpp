
#include "main.hpp"
#include "phys.hxx"


using namespace physx;



enum {  // collision types and flags describing collision interactions of each collision type
	COLLISION_FLAG_GROUND				=	1 << 0,
	COLLISION_FLAG_OBSTACLE			=	1 << 1,
	COLLISION_FLAG_CHASSIS			=	1 << 2,
	COLLISION_FLAG_WHEEL				=	1 << 3,
	COLLISION_FLAG_DRIVABLE_OBSTACLE=	1 << 4,
//	COLLISION_FLAG_GROUND_AGAINST	=								  0 |						0 | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
//	COLLISION_FLAG_WHEEL_AGAINST		=								  0 | COLLISION_FLAG_WHEEL | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | 0,
//	COLLISION_FLAG_CHASSIS_AGAINST	=			COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | 0,
//	COLLISION_FLAG_OBSTACLE_AGAINST	=			COLLISION_FLAG_GROUND | COLLISION_FLAG_WHEEL | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
//	COLLISION_FLAG_DRIVABLE_OBSTACLE_AGAINST=	COLLISION_FLAG_GROUND |							 COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
	COLLISION_FLAG_GROUND_AGAINST	=								  0 |						0 | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
	COLLISION_FLAG_WHEEL_AGAINST		=								  0 |                    0 |                      0 |                       0 | 0,
	COLLISION_FLAG_CHASSIS_AGAINST	=			COLLISION_FLAG_GROUND |                    0 | COLLISION_FLAG_CHASSIS |                       0 | 0,
	COLLISION_FLAG_OBSTACLE_AGAINST	=			COLLISION_FLAG_GROUND |                    0 | COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
	COLLISION_FLAG_DRIVABLE_OBSTACLE_AGAINST=	COLLISION_FLAG_GROUND |							 COLLISION_FLAG_CHASSIS | COLLISION_FLAG_OBSTACLE | COLLISION_FLAG_DRIVABLE_OBSTACLE,
};

enum {
	MASK_DRIVABLE_SURFACE   = 0xffff0000,
	MASK_UNDRIVABLE_SURFACE = 0x0000ffff
};


static const PxFilterData sim_filters[] = {
	{ 0,						0,									0, 0 },	// filter::NONE
	{ COLLISION_FLAG_GROUND,	COLLISION_FLAG_GROUND_AGAINST,		0, 0 },	// filter::GROUND
	{ COLLISION_FLAG_OBSTACLE,	COLLISION_FLAG_OBSTACLE_AGAINST,	0, 0 },	// filter::OBSTACLE
	{ COLLISION_FLAG_CHASSIS,	COLLISION_FLAG_CHASSIS_AGAINST,		0, 0 },	// filter::CHASSIS
	{ COLLISION_FLAG_WHEEL,		COLLISION_FLAG_WHEEL_AGAINST,		0, 0 }	// filter::WHEEL
};

static const PxFilterData qry_filters[] = {
	{ 0, 0, 0, 0						},	// filter::NONE
	{ 0, 0, 0, MASK_DRIVABLE_SURFACE	},	// filter::GROUND
	{ 0, 0, 0, MASK_UNDRIVABLE_SURFACE	},	// filter::OBSTACLE
	{ 0, 0, 0, MASK_UNDRIVABLE_SURFACE	},	// filter::CHASSIS
	{ 0, 0, 0, MASK_UNDRIVABLE_SURFACE	}	// filter::WHEEL
};


void phys::filter::Set( filter::Filter f, PxShape *shape )
{
	DBG_ASSERT( f >= filter::NONE && f <= filter::WHEEL );
	
	shape->setSimulationFilterData( sim_filters[f] );
	shape->setQueryFilterData( qry_filters[f] );
}


void phys::filter::Set( filter::Filter f, PxRigidActor *actor )
{
	const int num = actor->getNbShapes();
	PxShape *shapes[num];
	actor->getShapes( shapes, num );
	for( auto shape : shapes )
		 phys::filter::Set( f, shape );
}


static PxSceneQueryHitType::Enum WheelRaycastPreFilter(	
	PxFilterData fdata0, PxFilterData fdata1,
	const void *block_data, PxU32 block_size,
	PxHitFlags &filter_flags )
{
	//filterData0 is the vehicle suspension raycast.
	// filter1 is the shape potentially hit by the raycast.
	PX_UNUSED( filter_flags );
	PX_UNUSED( block_data );
	PX_UNUSED( block_size );
	PX_UNUSED( fdata0 );
	return ( ( fdata1.word3 & MASK_DRIVABLE_SURFACE ) == 0 ? PxSceneQueryHitType::eNONE : PxSceneQueryHitType::eBLOCK );
}

PxBatchQueryPreFilterShader phys::filter::GetWheelRaycastPreFilter( void )
{
	return WheelRaycastPreFilter;
}


void phys::filter::Initialize( void )
{
	// nothing to do
}


void phys::filter::Finalize( void )
{
	// nothing to do
}

