
#include "main.hpp"
#include "phys.hxx"


using namespace physx;



enum { // drivable surface types
	SURFACE_TYPE_TARMAC=0,
	SURFACE_TYPE_MUD,
	SURFACE_TYPE_GRASS,
	SURFACE_TYPE_SNOW,
	SURFACE_TYPE_ICE,
	MAX_NUM_SURFACE_TYPES
};

enum {  // tire types
	TIRE_TYPE_WETS=0,
	TIRE_TYPE_SLICKS,
	TIRE_TYPE_ICE,
	TIRE_TYPE_MUD,
	MAX_NUM_TIRE_TYPES
};

static float materials[MAX_NUM_SURFACE_TYPES][3] = {
	// static friction, dynamic friction, restitution
	{ 0.9f, 0.9f, 0.1f },  // SURF TARMAC
	{ 0.8f, 0.8f, 0.1f },  // SURF MUD
	{ 0.7f, 0.7f, 0.1f },  // SURF GRASS
	{ 0.5f, 0.5f, 0.1f },  // SURF SNOW
	{ 0.4f, 0.4f, 0.1f }   // SURF ICE
};

//Tire model friction for each combination of drivable surface type and tire type.
static const PxF32 friction_multipliers[MAX_NUM_SURFACE_TYPES][MAX_NUM_TIRE_TYPES] = {
	// WETS  SLICKS  ICE     MUD
	{ 1.10f, 1.15f,  1.10f,  1.10f },  // SURF TARMAC
	{ 0.95f, 0.95f,  0.95f,  0.95f },  // SURF MUD
	{ 0.80f, 0.80f,  0.80f,  0.80f },  // SURF GRASS
	{ 0.70f, 0.70f,  0.70f,  0.70f },  // SURF SNOW
	{ 0.70f, 0.70f,  0.70f,  0.70f }   // SURF ICE
};
/*static const PxF32 friction_multipliers[MAX_NUM_SURFACE_TYPES][MAX_NUM_TIRE_TYPES] = {
	// WETS  SLICKS  ICE     MUD
	{ 1.0f, 1.0f, 1.0f, 1.0f },  // SURF TARMAC
	{ 1.0f, 1.0f, 1.0f, 1.0f },  // SURF MUD
	{ 1.0f, 1.0f, 1.0f, 1.0f },  // SURF GRASS
	{ 1.0f, 1.0f, 1.0f, 1.0f },  // SURF SNOW
	{ 1.0f, 1.0f, 1.0f, 1.0f }   // SURF ICE
};*/



static PxVehicleDrivableSurfaceToTireFrictionPairs *pairs;



const PxVehicleDrivableSurfaceToTireFrictionPairs & phys::material::GetFrictionPairs( void )
{
	return *pairs;
}


void phys::material::Initialize( void )
{
	DBG_ASSERT( physics && scene );
	
	const PxMaterial *surface_materials[MAX_NUM_SURFACE_TYPES];
	PxVehicleDrivableSurfaceType surface_types[MAX_NUM_SURFACE_TYPES];
	for( int i = 0; i < MAX_NUM_SURFACE_TYPES; i++ ) {
		surface_materials[i] = physics->createMaterial( materials[i][0], materials[i][1], materials[i][2] );
		surface_types[i].mType = i;
	}

	pairs = PxVehicleDrivableSurfaceToTireFrictionPairs::allocate( MAX_NUM_TIRE_TYPES, MAX_NUM_SURFACE_TYPES );
	pairs->setup( MAX_NUM_TIRE_TYPES, MAX_NUM_SURFACE_TYPES, surface_materials, surface_types );
	for( int i = 0; i < MAX_NUM_SURFACE_TYPES; i++ ) {
		for( int j = 0; j < MAX_NUM_TIRE_TYPES; j++ )
			pairs->setTypePairFriction( i, j, friction_multipliers[i][j] );
	}
}


void phys::material::Finalize( void )
{
	if( pairs ) pairs->release();
}

