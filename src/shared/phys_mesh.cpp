
#include <extensions/PxDefaultStreams.h>

#include "main.hpp"
#include "phys.hxx"



using namespace physx;




template < typename T > // T = byte, ushort, int
PxTriangleMesh * phys::mesh::CreateTriangleMesh( int num_points, int num_indexes, float3 points[], T triangles[] )
{
	DBG_ASSERT( sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 );
	DBG_ASSERT( sizeof(float3) == sizeof(PxVec3) );

	ASSERT( num_indexes % 3 == 0, "CreateTriangleMesh: Wrong num_indexes, is not multiple of 3" );

	if( sizeof(T) == 1 && num_points >= (1<<8) )
		print::Error( "CreateTriangleMesh: Wrong num_indexes, data type can not allow access to all vertices" );

	if( sizeof(T) == 2 && num_points >= (1<<16) )
		print::Error( "CreateTriangleMesh: Wrong num_indexes, data type can not allow access to all vertices" );

	ushort *array = ( sizeof(T) == 2 ? triangles : NULL );
	if( sizeof(T) != 2 && num_points < (1<<16) ) {
		array = (ushort*) std::malloc( sizeof(ushort) * num_indexes );
		for( int i = 0; i < num_indexes; i++ ) {
			DBG_ASSERT( triangles[i] < (1<<16) );
			array[i] = triangles[i];
		}		
	}
	
	PxTriangleMeshDesc mesh_desc;
	mesh_desc.points.count		= num_points;
	mesh_desc.points.stride		= sizeof(PxVec3);
	mesh_desc.points.data		= (void*) points;
	mesh_desc.triangles.count	= num_indexes / 3;
	mesh_desc.triangles.stride	= 3*sizeof(T);
	mesh_desc.triangles.data	= ( array ? (void*)array : (void*)triangles );
	mesh_desc.flags				= ( array ? PxMeshFlag::e16_BIT_INDICES : (PxMeshFlag::Enum)0 );
	ASSERT( mesh_desc.isValid(), "CreateTriangleMesh: Invalid triangle mesh descriptor" );

	PxDefaultMemoryOutputStream wbuffer;
	bool status = cooking->cookTriangleMesh( mesh_desc, wbuffer );
	if( !status ) print::Error( "CreateTriangleMesh: cooking->cookTriangleMesh() FALSE'" );

	PxDefaultMemoryInputData rbuffer( wbuffer.getData(), wbuffer.getSize() );
	PxTriangleMesh *mesh = physics->createTriangleMesh( rbuffer );
	if( !mesh ) print::Error( "CreateTriangleMesh: physics->createTriangleMesh() NULL'" );
	
	if( array && array != (ushort*)triangles )
		std::free( array );
	
	return mesh;
}


template < typename T > // T = byte, ushort, int
PxTriangleMesh * phys::mesh::CreatePolygonalMesh( int num_points, int num_indexes, float3 points[], T polygons[], int num_polygons_check )
{
	ASSERT( num_points < (1<<16), "CreatePolygonalMesh: Max number of points = 64K" );
	
	int num_polygons = 0;
	int num_tris     = 0;
	
	for( int i = 0; i < num_indexes; i += 1+polygons[i] ) {
		ASSERT( polygons[i] >= 3, "CreatePolygonalMesh: Found polygon with %d points", polygons[i] );
		num_tris += polygons[i] - 2;
		num_polygons++;
	}
	
	if( num_polygons_check >= 0 && num_polygons != num_polygons_check )
		print::Error( "CreatePolygonalMesh: Num polygons check failed, found %d, exptected %d", num_polygons, num_polygons_check );

	ushort *array = (ushort*) std::malloc( sizeof(ushort) * 3*num_tris );
	
	int count_indexes = 0;
	for( int i = 0; i < num_indexes; i += 1+polygons[i] ) {
		for( int j = 2; j < polygons[i]; j++ ) {
			array[count_indexes++] = polygons[i+1+0-0];
			array[count_indexes++] = polygons[i+1+j-1];
			array[count_indexes++] = polygons[i+1+j-0];
		}
	}
	
	DBG_ASSERT( count_indexes == 3*num_tris );
	
	PxTriangleMesh *mesh = CreateTriangleMesh( num_points, 3*num_tris, points, array );
	
	std::free( array );

	return mesh;
}


PxHeightField * phys::mesh::CreateHeightField( int num_points_x, int num_points_y, float zscale, short *heights )
{
	ASSERT( false, "phys::mesh::CreateHeightField: TODO" );
}


PxConvexMesh * phys::mesh::CreateConvexMesh( int num, float3 points[] )
{
	DBG_ASSERT( sizeof(float3) == sizeof(PxVec3) );
	
	PxConvexMeshDesc convex_desc;
	convex_desc.points.count    = num;
	convex_desc.points.stride   = sizeof(PxVec3);
	convex_desc.points.data     = (void*) points;
	convex_desc.flags           = PxConvexFlag::eCOMPUTE_CONVEX;
	convex_desc.vertexLimit     = num + 1;	// why +1 ?
	ASSERT( convex_desc.isValid(), "CreateConvexMesh: Invalid convex descriptor" );

	PxDefaultMemoryOutputStream wbuffer;
	bool status = cooking->cookConvexMesh( convex_desc, wbuffer );
	if( !status ) print::Error( "CreateConvexMesh: cooking->cookConvexMesh() FALSE'" );

	PxDefaultMemoryInputData rbuffer( wbuffer.getData(), wbuffer.getSize() );
	PxConvexMesh *mesh = physics->createConvexMesh( rbuffer );
	if( !mesh ) print::Error( "CreateConvexMesh: physics->createConvexMesh() NULL'" );
	
	return mesh;
}


PxConvexMesh * phys::mesh::CreateCylinderMesh( int sides, float radius, float heigh )
{
	DBG_ASSERT( sides >= 3 && sides <= 32 );
	
	float3 points[ 2*sides ];
	for( int i = 0; i < sides; i++ ) {
		const float ang = i * (2*M_PI) / sides;
		points[2*i+0].x = points[2*i+1].x = radius * cos( ang );
		points[2*i+0].y = points[2*i+1].y = radius * sin( ang );
		points[2*i+0].z = -0.5f * heigh;
		points[2*i+1].z = +0.5f * heigh;
	}
	
	return CreateConvexMesh( 2*sides, points );
}


PxConvexMesh * phys::mesh::CreateWheelMesh( int sides, float radius, float width )
{
	ASSERT( sides >= 3 && sides <= 32, "GetWheelMesh: Wrong number of wheel sides" );

	float3 points[ 2*sides ];
	for( int i = 0; i < sides; i++ ) {
		const float ang = ( (i-0.5f) / sides ) * (2*M_PI) - (M_PI/2);
		points[2*i+0].x = points[2*i+1].x = radius * cos( ang );
		points[2*i+0].z = points[2*i+1].z = radius * sin( ang );
		points[2*i+0].y = -0.5f * width;
		points[2*i+1].y = +0.5f * width;
	}
	
	return CreateConvexMesh( 2*sides, points );
}

		
void phys::mesh::Initialize( void )
{
	DBG_ASSERT( physics && cooking );
	
	phys::mesh::Finalize();
}


void phys::mesh::Finalize( void )
{
	// nothing to do
}


void __phys_mesh__compile_template( void )
{
	phys::mesh::CreatePolygonalMesh( 0, 0, (float3*)NULL, (byte*)  NULL );
	phys::mesh::CreatePolygonalMesh( 0, 0, (float3*)NULL, (ushort*)NULL );
	phys::mesh::CreatePolygonalMesh( 0, 0, (float3*)NULL, (int*)   NULL );
}

