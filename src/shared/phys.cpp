
#include <PxPhysicsAPI.h>

#include "main.hpp"
#include "util.hpp"
#include "math.hpp"
#include "phys.hxx"


using namespace physx;


namespace phys {
	PxFoundation		*foundation;
	PxPhysics			*physics;
	PxCooking			*cooking;
	PxScene				*scene;
	PxControllerManager	*manager;
}



// physx main callbacks ////////////////////////////////////////////////////////////////////////


class AllocatorCallback : public PxAllocatorCallback
{
	public:
	
		static AllocatorCallback & Singleton( void ) {
			static AllocatorCallback allocator_callback;
			return allocator_callback;
		}

		void * allocate( size_t size, const char *tag, const char *file, int line )
		{
			return ::memalign( 16, size );
		}

		void deallocate( void* ptr )
		{
			::free( ptr );
		}
};



class ErrorCallback : public PxErrorCallback
{
	public:
	
		static ErrorCallback & Singleton( void ) {
			static ErrorCallback error_callback;
			return error_callback;
		}

		void reportError( PxErrorCode::Enum code, const char *message, const char *file, int line )
		{
			char label[64];
			label[0] = '\0';
			if( code == PxErrorCode::eNO_ERROR          ) strcat( label, "[NO_ERROR]"          );
			if( code &  PxErrorCode::eDEBUG_INFO        ) strcat( label, "[DEBUG_INFO]"        );
			if( code &  PxErrorCode::eDEBUG_WARNING     ) strcat( label, "[DEBUG_WARNING]"     );
			if( code &  PxErrorCode::eINVALID_PARAMETER ) strcat( label, "[INVALID_PARAMETER]" );
			if( code &  PxErrorCode::eINVALID_OPERATION ) strcat( label, "[INVALID_OPERATION]" );
			if( code &  PxErrorCode::eOUT_OF_MEMORY     ) strcat( label, "[OUT_OF_MEMORY]"     );
			if( code &  PxErrorCode::eINTERNAL_ERROR    ) strcat( label, "[INTERNAL_ERROR]"    );
			if( code &  PxErrorCode::eABORT             ) strcat( label, "[ABORT]"             );
			if( code &  PxErrorCode::ePERF_WARNING      ) strcat( label, "[PERF_WARNING]"      );

//			print::Debug( "%s  line=%d, file='%s'\n", label, line, file );			
//			print::Debug( "  %s\n", message );

			if( code < 0 || code > PxErrorCode::eDEBUG_WARNING )
				print::Error( "ErrorCallback: PysX error." );
		}
};



// phys module /////////////////////////////////////////////////////////////////////////////////////////////



PxFilterFlags SimulationFilterShader(
	PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags &pairFlags, const void *constantBlock, PxU32 constantBlockSize )
{
	// let triggers through
	if( PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1) ) {
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}

	// generate contacts for all that were not filtered above
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where the filtermask of A contains the ID of B and vice versa.
	if( ( filterData0.word0 & filterData1.word1 ) && ( filterData1.word0 & filterData0.word1 ) ) {
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
		return PxFilterFlag::eDEFAULT;
	}
//	const uint a_collides_b = ( filterData0.word0 & filterData1.word1 );
//	const uint b_collides_a = ( filterData1.word0 & filterData0.word1 );
//	if( a_collides_b || b_collides_a ) pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
//	if( a_collides_b && b_collides_a ) return PxFilterFlag::eDEFAULT;

	return PxFilterFlag::eSUPPRESS;	
}


void phys::Initialize( const int threads )
{
	phys::Finalize();

	foundation = PxCreateFoundation( PX_PHYSICS_VERSION, AllocatorCallback::Singleton(), ErrorCallback::Singleton() );
	if( !foundation ) print::Error( "phys::Initialize: PxCreateFoundation NULL" );
	
	physics = PxCreatePhysics( PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), false, NULL );
	if( !physics ) print::Error( "phys::Initialize: PxCreatePhysics NULL" );

	cooking = PxCreateCooking( PX_PHYSICS_VERSION, *foundation, PxCookingParams(PxTolerancesScale()) );
	if( !cooking ) print::Error( "phys::Initialize: PxCreateCooking NULL" );

	bool ok = PxInitExtensions( *physics );
	if( !ok ) print::Error( "phys::Initialize: PxInitExtensions FALSE" );

	ok = PxInitVehicleSDK( *physics );
	if( !ok ) print::Error( "phys::Initialize: PxInitVehicleSDK FALSE" );
	PxVehicleSetBasisVectors( PxVec3( 0, 0, 1 ), PxVec3( 1, 0, 0 ) );
	PxVehicleSetUpdateMode( PxVehicleUpdateMode::eVELOCITY_CHANGE );  // PxVehicleUpdateMode::eACCELERATION

	PxSceneDesc scene_desc( physics->getTolerancesScale() );
	scene_desc.gravity       = PxVec3( 0.0f, 0.0f, -9.81f );
	scene_desc.filterShader  = SimulationFilterShader;	//PxDefaultSimulationFilterShader;
	scene_desc.cpuDispatcher = PxDefaultCpuDispatcherCreate( threads );
	DBG_ASSERT( scene_desc.isValid() );
	scene = physics->createScene( scene_desc );
	if( !scene ) print::Error( "phys::Initialize: physics->createScene NULL" );

	manager = PxCreateControllerManager( *scene );
	if( !manager ) print::Error( "phys::Initialize: PxCreateControllerManager NULL" );
	
	phys::filter::Initialize();
	phys::material::Initialize();
	phys::mesh::Initialize();
	phys::geom::Initialize();
	phys::userveh::Initialize();
//	phys::veh::Initialize();
//	phys::ped::Initialize();
}


void phys::Finalize( void )
{
//	phys::ped::Finalize();
//	phys::veh::Finalize();
	phys::userveh::Finalize();
	phys::geom::Finalize();
	phys::mesh::Finalize();
	phys::material::Finalize();
	phys::filter::Finalize();

	if( physics ) PxCloseVehicleSDK();
	if( physics ) PxCloseExtensions();

	if( manager    ) manager->release();
	if( scene      ) scene->release();
	if( cooking    ) cooking->release();
	if( physics    ) physics->release();
	if( foundation ) foundation->release();
	
	foundation = NULL;
	physics    = NULL;
	cooking    = NULL;
	scene      = NULL;
	manager    = NULL;
}


void phys::Update( const float dt )
{
	double t0 = GetTime();

	phys::userveh::Update( dt );
//	phys::veh::Update( dt );
//	phys::ped::Update( dt );

	double t1 = GetTime();

	scene->simulate( dt );

	double t2 = GetTime();
	
	scene->fetchResults( true );

	double t3 = GetTime();
	
	//printf( "phys::Update:  total=%.6f  update=%.6f  simulate=%.6fms  results=%.6fms\n", 1000*(t3-t0), 1000*(t1-t0), 1000*(t2-t1), 1000*(t3-t2) );
}


static PxRigidStatic * GetOrCreateWorld( void )
{
	PxActor *actors[2] = { NULL, NULL };
	
	int count = phys::scene->getActors( PxActorTypeFlag::eRIGID_STATIC, actors, 2 );

	if( count ) {
		DBG_ASSERT( count == 1 && actors[0] );
		DBG_ASSERT( !util::str::CmpI( actors[0]->getName(), "world" ) );
		return (PxRigidStatic*) actors[0];
	}

	PxRigidStatic *world = phys::physics->createRigidStatic( PxTransform::createIdentity() );
	if( !world ) print::Error( "phys::LoadGroundMeshes: PxCreateStatic NULL" );
	
	world->setName( "world" );	
		
	phys::scene->addActor( *world );
	
	return world;
}


void phys::LoadGroundMeshes( const char *filename, GeomsCallbacks *callbacks )
{
	static PxMaterial *material_world = physics->createMaterial( 0.0f, 0.0f, 0.0f );	//####TODO:  material_world->release();

	DBG_ASSERT( physics && cooking && scene );

	PxRigidStatic *world = GetOrCreateWorld();

	const auto *geoms = phys::geom::LoadGeoms( filename, callbacks );
	for( int i = 0; i < geoms->array_geoms_size; i++ )
	{
		const auto &geom = geoms->array_geoms[i];
		
		PxShape *shape = world->createShape( geom.geom.any(), *material_world );
		if( !shape ) print::Error( "phys::LoadGroundMeshes: world->createShape() NULL'" );
		
		shape->setLocalPose( geom.trans );

		filter::Set( filter::GROUND, shape );
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



typedef std::vector< PxU32  > VectorIndexes;
typedef std::vector< PxVec3 > VectorVertexes;


static void LoadMeshOBJ( const char *filename, VectorIndexes &indxs, VectorVertexes &verts )
{
	char buffer[256], *p;

	FILE *f = fopen( filename, "rb" );
	if( !f ) print::Error( "LoadMeshOBJ: Can not open file '%s'.", filename );

	int line = 0;
	while( true )
	{
		p = fgets( buffer, sizeof(buffer), f );
		if( !p ) break;
		line++;

		while( *p && *p <= ' ' ) p++;
		
		if( p[0] == 'v' && p[1] <= ' ' )
		{
			float a, b, c, n;
			n = sscanf( p+2, "%f%f%f", &a, &b, &c );
			DBG_ASSERT( n == 3 );
			verts.push_back( PxVec3( a, b, c ) );
			//verts.push_back( PxVec3( a, -c, b ) );
		}

		if( p[0] == 'f' && p[1] <= ' ' )
		{
			int a, b, c, n;
			n = sscanf( p+2, "%d%d%d", &a, &b, &c );
			if( n != 3 ) print::Debug( "LoadMeshOBJ: line:%d. Face is not a triangle.", line );
			if( n == 3 ) {
				indxs.push_back( a-1 );
				indxs.push_back( b-1 );
				indxs.push_back( c-1 );
			}
		}
	}

	fclose( f );

	DBG_ASSERT( indxs.size() % 3 == 0 );
	for( unsigned int i = 0; i < indxs.size(); i++ ) {
		DBG_ASSERT( indxs[i] >= 0 && indxs[i] < verts.size() );
	}
}


void phys::LoadGroundMeshBig( const char *filename )
{
	static PxMaterial *material_world = physics->createMaterial( 0.0f, 0.0f, 0.0f );	//####TODO:  material_world->release();
	
	VectorIndexes  indxs;
	VectorVertexes verts;

	DBG_ASSERT( physics && cooking && scene );

	if( strstr( filename, ".obj" ) ) LoadMeshOBJ( filename, indxs, verts );
	if( !indxs.size() && !verts.size() ) print::Error( "phys::LoadGroundMeshBig: Unsupported file foramat (file extension)." );
//	printf( "phys::LoadGroundMeshBig: file '%s'\n", filename );
//	printf( "phys::LoadGroundMeshBig: num vertexes : %d\n", (int)verts.size()/1 );
//	printf( "phys::LoadGroundMeshBig: num triangles: %d\n", (int)indxs.size()/3 );

	PxTriangleMeshDesc mesh_desc;
	mesh_desc.points.count		= verts.size() / 1;
	mesh_desc.points.stride		= sizeof(PxVec3);
	mesh_desc.points.data		= (void*) &verts[0];
	mesh_desc.triangles.count	= indxs.size() / 3;
	mesh_desc.triangles.stride	= 3*sizeof(PxU32);
	mesh_desc.triangles.data	= (void*) &indxs[0];
	DBG_ASSERT( sizeof(PxU32 ) == sizeof(indxs[0]) );
	DBG_ASSERT( sizeof(PxVec3) == sizeof(verts[0]) );
	DBG_ASSERT( mesh_desc.isValid() );

	PxDefaultMemoryOutputStream wbuffer;
	bool status = cooking->cookTriangleMesh( mesh_desc, wbuffer );
	if( !status ) print::Error( "phys::LoadGroundMeshBig: cooking->cookTriangleMesh() FALSE'" );

	PxDefaultMemoryInputData rbuffer( wbuffer.getData(), wbuffer.getSize() );
	PxTriangleMesh *world_mesh = physics->createTriangleMesh( rbuffer );
	if( !world_mesh ) print::Error( "phys::LoadGroundMeshBig: physics->createTriangleMesh() NULL'" );

	PxRigidStatic *world = GetOrCreateWorld();
	PxTriangleMeshGeometry world_geometry( world_mesh );
	PxShape *shape = world->createShape( world_geometry, *material_world );
	if( !shape ) print::Error( "phys::LoadGroundMeshBig: world->createShape() NULL'" );
	filter::Set( filter::GROUND, shape );
}
