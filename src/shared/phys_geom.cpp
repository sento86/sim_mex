
#include "main.hpp"
#include "util.hpp"
#include "phys.hxx"



using namespace physx;



static const int INDEX_SIZE = 16;


struct GeomData {
	int    type, mesh_index, _pad0, _pad1;
	float4 pos;
	float4 ori;
	float4 params;
};

struct HFieldData {
	int    type, num_points_x, num_points_y, offset;
	float  zscale, _pad1, _pad2, _pad3;
	short  *heights;
};

struct MeshData {
	int    type, num_points, _pad0, offset_points;
	int    index_size, num_indexes, num_polygons, offset_polygons;
	void   *buffer;
	float3 *points;
	char4  *normals;
	void   *polygons;
};



static PxConvexMesh *the_cylinder_convex_mesh;
static PxConvexMesh *the_wheel_convex_mesh;



static phys::geom::Geoms * AllocGeoms( const int index[INDEX_SIZE][2], phys::GeomsCallbacks *callbacks )
{
	int num_meshes = 0;
	for( int i = 1; i < 4; i++ )
		num_meshes += index[i][0];

	int num_geoms = 0;
	for( int i = 4; i < INDEX_SIZE; i++ )
		num_geoms += index[i][0];

	uint bytes	= 1          * sizeof( phys::geom::Geoms )
				+ num_meshes * sizeof( phys::geom::Mesh  )
				+ num_geoms  * sizeof( phys::geom::Geom  );
				
	auto *geoms = (phys::geom::Geoms*) std::malloc( bytes + 256 );	
	ASSERT( geoms, "AllocGeoms: Out of memory" );
	std::memset( geoms, 0, bytes + 256 );

	geoms->callbacks         = callbacks;
	geoms->array_meshes_size = num_meshes;
	geoms->array_geoms_size  = num_geoms;
	geoms->array_meshes      = (phys::geom::Mesh*) ( ( (intptr_t)(geoms+1) + 0*num_meshes*sizeof(phys::geom::Mesh) + 0*16+15 ) & ~15 );
	geoms->array_geoms       = (phys::geom::Geom*) ( ( (intptr_t)(geoms+1) + 1*num_meshes*sizeof(phys::geom::Mesh) + 1*16+15 ) & ~15 );

	DBG_ASSERT( phys::geom::_SIZE_GEOMS < INDEX_SIZE && phys::geom::WHEEL == 4 && phys::geom::MESH == 12 );  // who modified the enumeration ?

	phys::geom::Geom *g = geoms->array_geoms;
	for( int i = 4; i < INDEX_SIZE; i++ ) {
		geoms->num[i]  = index[i][0];
		geoms->geom[i] = ( index[i][0] ? g : NULL );
		g += index[i][0];
	}	
	DBG_ASSERT( g == geoms->array_geoms + num_geoms );
	
	return geoms;
}


static void ReadHeader( FILE *file, bool &swap_endian, int &version )
{
	int identif;
	
	int read0 = fread( &identif, sizeof(identif), 1, file );
	int read1 = fread( &version, sizeof(version), 1, file );
	ASSERT( read0 == 1 && read1 == 1, "ReadHeader: Can not read Header" );
	
	swap_endian = !util::endian::Test( identif, "VCOL" );
	
	if( swap_endian ) util::endian::Swap( identif );
	if( swap_endian ) util::endian::Swap( version );
	
	if( !util::endian::Test( identif, "VCOL" ) )
		print::Error( "ReadHeader: Invalid file identifier, expected 'VCOL'" );
}


static void ReadIndex( FILE *file, const bool swap_endian, int index[INDEX_SIZE][2] )
{
	int read = fseek( file, 2*4, 0 );
	ASSERT( read == 0, "ReadIndex: Can not seek to index" );

	read = fread( index, sizeof(index[0]), INDEX_SIZE, file );	
	ASSERT( read == INDEX_SIZE, "ReadIndex: Can not read Index" );
	
	if( swap_endian ) util::endian::Swap( 2*INDEX_SIZE, (int*)index );
	ASSERT( index[0][0] == 0, "LoadChassis: Malformed index data" );
	
	for( int i = 1; i < phys::geom::_SIZE_GEOMS; i++ )
		if( index[i-1][1] > index[i][1] )
			print::Error( "LoadChassis: Malformed index data (bad offsets)" );
}


static void ReadGlobals( FILE *file, const bool swap_endian, const int index[INDEX_SIZE][2], phys::geom::Geoms *geoms )
{
	int read = fseek( file, index[0][1], 0 );
	ASSERT( read == 0, "ReadGlobals: Can not seek to globals" );
	
	read = fread( &geoms->bbox_size, sizeof(geoms->bbox_size), 1, file );	
	ASSERT( read == 1, "ReadGlobals: Can not read bbox_size" );

	read = fread( &geoms->bbox_center, sizeof(geoms->bbox_center), 1, file );	
	ASSERT( read == 1, "ReadGlobals: Can not read bbox_center" );

	read = fread( &geoms->mass_center, sizeof(geoms->mass_center), 1, file );	
	ASSERT( read == 1, "ReadGlobals: Can not read mass_center" );	
}


static HFieldData ReadHFieldData( FILE *file, const bool swap_endian, const int type, const char *tag )
{
	HFieldData data;

	int read = fread( &data, 2*4*4, 1, file );
	ASSERT( read == 1, "ReadHFieldData: Can not read %s", tag );
	
	if( swap_endian ) util::endian::Swap( 4, (int*)&data );
	
	ASSERT( data.type == type, "ReadHFieldData: Invalid type %d for %s", data.type, tag );
 
	const int num_points = data.num_points_x * data.num_points_y;
	
	data.heights = (short*) std::malloc( sizeof(short) * num_points );
	
	read = fseek( file, data.offset, 0 );
	ASSERT( read == 0, "ReadHFieldData: Can not seek to heights", tag );

	read = fread( data.heights, sizeof(short), num_points, file );	
	ASSERT( read == num_points, "ReadHFieldData: Can not read heights" );
	
	if( swap_endian ) util::endian::Swap( num_points, data.heights );
	
	return data;
}


static MeshData ReadMeshData( FILE *file, const bool swap_endian, const int type, const char *tag )
{
	MeshData data;

	int read = fread( &data, 2*4*4, 1, file );
	ASSERT( read == 1, "ReadMeshData: Can not read %s", tag );
	
	if( swap_endian ) util::endian::Swap( 4, (int*)&data + 0*4 );
	if( swap_endian ) util::endian::Swap( 4, (int*)&data + 1*4 );

	ASSERT( data.type == type, "ReadMeshData: Invalid type %d for %s", data.type, tag );
	
	ASSERT( data.index_size == 1 || data.index_size == 2 || data.index_size == 4,
		"ReadMeshData: Unsuported index size, found %d bytes, expected 1/2/4 bytes per index", data.index_size
	);

	data.buffer = std::malloc( data.num_points * ( sizeof(float3) + sizeof(char4) ) + data.num_indexes * data.index_size );
	ASSERT( data.buffer, "ReadMeshData: Out of memory" );	
	data.points   = (float3*) ( data.buffer );
	data.normals  = (char4*)  ( data.points  + data.num_points );
	data.polygons = (void*)   ( data.normals + data.num_points );
	
	read = fseek( file, data.offset_points, 0 );
	ASSERT( read == 0, "ReadMeshData: Can not seek to points", tag );

	read = fread( data.points, sizeof(float3), data.num_points, file );	
	ASSERT( read == data.num_points, "ReadMeshData: Can not read points" );

	read = fread( data.normals, sizeof(char4), data.num_points, file );	
	ASSERT( read == data.num_points, "ReadMeshData: Can not read normals" );

	read = fseek( file, data.offset_polygons, 0 );
	ASSERT( read == 0, "ReadMeshData: Can not seek to polygons", tag );

	read = fread( data.polygons, data.index_size, data.num_indexes, file );	
	ASSERT( read == data.num_indexes, "ReadMeshData: Can not read polygons" );
	
	if( swap_endian )
	{
		ASSERT( false, "NOT TESTED" );
		
		util::endian::Swap( data.num_points, (int*)data.normals );
		
		if( data.index_size == 2 )
			util::endian::Swap( data.num_indexes, (short*)data.polygons );

		if( data.index_size == 4 )
			util::endian::Swap( data.num_indexes, (int*)data.polygons );
	}

	return data;
}


static void ReadMeshes( FILE *file, const bool swap_endian, const int index[INDEX_SIZE][2], phys::geom::Geoms *geoms )
{
	static const int bytes = 2 * 4 * 4;

	DBG_ASSERT( geoms->array_meshes_size == index[1][0] + index[2][0] + index[3][0] );
	
	int offset = index[1][1];
	
	for( int i = 0; i < geoms->array_meshes_size; i++ )
	{
		int   type = 0;
		void *mesh = NULL;

		DBG_ASSERT( offset < index[3][1] + index[3][0]*bytes );

		int read = fseek( file, offset, 0 );
		ASSERT( read == 0, "ReadMeshes: Can not seek to mesh %d", i );

		read = fread( &type, sizeof(int), 1, file );
		ASSERT( read == 1, "ReadMeshes: Can not read mesh type" );

		read = fseek( file, offset, 0 );
		ASSERT( read == 0, "ReadMeshes: Can not re-seek to mesh %d", i );
		
		if( swap_endian ) type = util::endian::Swap( type );
		
		switch( type )
		{
			case phys::geom::MESH_HFIELD: {
				HFieldData data = ReadHFieldData( file, swap_endian, type, "HFIELD" );
				mesh = phys::mesh::CreateHeightField( data.num_points_x, data.num_points_y, data.zscale, data.heights );
				std::free( data.heights );
				break;
			}
			
			case phys::geom::MESH_CONVEX: {
				MeshData data = ReadMeshData( file, swap_endian, type, "CONVEX" );
				mesh = phys::mesh::CreateConvexMesh( data.num_points, data.points );
				if( geoms->callbacks )
					geoms->callbacks->OnMesh( i, true, data.num_points, data.points, data.normals, data.index_size, data.num_indexes, data.num_polygons, data.polygons );
				std::free( data.buffer );
				break;
			}
			
			case phys::geom::MESH_MESH: {
				MeshData data = ReadMeshData( file, swap_endian, type, "MESH" );
				if( data.index_size == 1 ) mesh = phys::mesh::CreatePolygonalMesh( data.num_points, data.num_indexes, data.points, (byte*)  data.polygons, data.num_polygons );
				if( data.index_size == 2 ) mesh = phys::mesh::CreatePolygonalMesh( data.num_points, data.num_indexes, data.points, (ushort*)data.polygons, data.num_polygons );
				if( data.index_size == 4 ) mesh = phys::mesh::CreatePolygonalMesh( data.num_points, data.num_indexes, data.points, (int*)   data.polygons, data.num_polygons );
				if( geoms->callbacks )
					geoms->callbacks->OnMesh( i, false, data.num_points, data.points, data.normals, data.index_size, data.num_indexes, data.num_polygons, data.polygons );
				std::free( data.buffer );
				break;
			}
			
			default:
				print::Error( "ReadMeshes: Invalid mesh type" );			
		}
		
		ASSERT( type && mesh, "ReadMeshes: Someting is wrong!!" );
		
		geoms->array_meshes[i] = (phys::geom::Mesh){ type, mesh };
		
		offset += bytes;
	}
}


static GeomData ReadGeomData( FILE *file, const bool swap_endian, const int offset, const int index, const int type, const char *tag )
{
	GeomData data;
	
	const int bytes = sizeof(GeomData);
	DBG_ASSERT( bytes == 4*4*4 );

	int read = fseek( file, offset + index*bytes, 0 );
	ASSERT( read == 0, "ReadGeomData: Can not seek to %s", tag );

	read = fread( &data, bytes, 1, file );
	ASSERT( read == 1, "ReadGeomData: Can not read %s", tag );
	
	if( swap_endian ) util::endian::Swap( 4, (int*)&data );
	
	ASSERT( data.type == type, "ReadGeomData: Invalid type %d for %s", data.type, tag );
	
	return data;
}


static void SetGeomData( phys::geom::Geoms *geoms, int type, int index, const GeomData &data, const PxGeometry &geom, int mesh_index=-1 )
{
	DBG_ASSERT( type >= phys::geom::WHEEL && type <= phys::geom::MESH );
	DBG_ASSERT( index >= 0 && index < geoms->num[type] );	
	DBG_ASSERT( mesh_index < 0 || mesh_index < geoms->array_meshes_size );	

	phys::geom::Geom &g = geoms->geom[type][index];
	g.type    = type;
	g.mesh    = mesh_index;
	g.trans.p = PxVec3( data.pos.x, data.pos.y, data.pos.z );
	g.trans.q = PxQuat( data.ori.x, data.ori.y, data.ori.z, data.ori.w );
	g.geom.storeAny( geom );
}


static void ReadGeomWheels( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::WHEEL;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "WHEEL" );
		
		const PxGeometry &geom = PxConvexMeshGeometry( the_wheel_convex_mesh );

		SetGeomData( geoms, type, i, data, geom );
	}
}


static void ReadGeomPlanes( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::PLANE;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "PLANE" );

		ASSERT( false, "TODO" ); //###TODO: oh no, plane is infinite!!
		const PxGeometry &geom = PxPlaneGeometry(); // PxBoxGeometry( data.params.x, data.params.y );

		SetGeomData( geoms, type, i, data, geom );
	}
}


static void ReadGeomBoxes( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::BOX;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "BOX" );
		
		const PxGeometry &geom = PxBoxGeometry( data.params.x, data.params.y, data.params.z );

		SetGeomData( geoms, type, i, data, geom );
	}
}


static void ReadGeomSpheres( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::SPHERE;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "SPHERE" );
		
		const PxGeometry &geom = PxSphereGeometry( data.params.x );

		SetGeomData( geoms, type, i, data, geom );
	}
}


static void ReadGeomCylinders( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::CYLINDER;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "CYLINDER" );
		
		PxMeshScale scale;
		scale.scale = PxVec3( data.params.x, data.params.x, data.params.y );

		const PxGeometry &geom = PxConvexMeshGeometry( the_cylinder_convex_mesh, scale );

		SetGeomData( geoms, type, i, data, geom );
	}
}


static void ReadGeomCapsules( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::CAPSULE;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "CAPSULE" );
		
		const PxGeometry &geom = PxCapsuleGeometry( data.params.x, data.params.y/2 );

		SetGeomData( geoms, type, i, data, geom );
	}
}


static void ReadGeomHFields( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::HFIELD;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "HFIELD" );

		ASSERT( data.mesh_index >= 0 && data.mesh_index < geoms->array_meshes_size,
			"ReadHFields: Mesh index out of range"
		);
		
		ASSERT( geoms->array_meshes[ data.mesh_index ].type == phys::geom::MESH_HFIELD,
			"ReadHFields: Mesh type mismatch"
		);

		PxHeightField *hf = (PxHeightField*) geoms->array_meshes[ data.mesh_index ].ptr;
		
		const PxGeometry &geom = PxHeightFieldGeometry( hf, (PxMeshGeometryFlag::Enum)0, data.params.z, data.params.y, data.params.x );
		
		SetGeomData( geoms, type, i, data, geom, data.mesh_index );
	}
}


static void ReadGeomConvexes( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::CONVEX;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "CONVEX" );

		ASSERT( data.mesh_index >= 0 && data.mesh_index < geoms->array_meshes_size,
			"ReadConvexes: Mesh index out of range"
		);
		
		ASSERT( geoms->array_meshes[ data.mesh_index ].type == phys::geom::MESH_CONVEX,
			"ReadConvexes: Mesh type mismatch"
		);

		PxConvexMesh *mesh = (PxConvexMesh*) geoms->array_meshes[ data.mesh_index ].ptr;

		PxMeshScale scale;
		scale.scale = PxVec3( data.params.x, data.params.y, data.params.z );

		const PxGeometry &geom = PxConvexMeshGeometry( mesh, scale );
		
		SetGeomData( geoms, type, i, data, geom, data.mesh_index );
	}
}


static void ReadGeomMeshes( FILE *file, const bool swap_endian, const int offset, phys::geom::Geoms *geoms )
{
	const int type = phys::geom::MESH;
	
	for( int i = 0; i < geoms->num[type]; i++ )
	{
		GeomData data = ReadGeomData( file, swap_endian, offset, i, type, "MESH" );

		ASSERT( data.mesh_index >= 0 && data.mesh_index < geoms->array_meshes_size,
			"ReadMeshes: Mesh index out of range"
		);
		
		ASSERT( geoms->array_meshes[ data.mesh_index ].type == phys::geom::MESH_MESH,
			"ReadMeshes: Mesh type mismatch"
		);

		PxTriangleMesh *mesh = (PxTriangleMesh*) geoms->array_meshes[ data.mesh_index ].ptr;

		PxMeshScale scale;
		scale.scale = PxVec3( data.params.x, data.params.y, data.params.z );

		const PxGeometry &geom = PxTriangleMeshGeometry( mesh, scale );
		
		SetGeomData( geoms, type, i, data, geom, data.mesh_index );
	}
}


const phys::geom::Geoms * phys::geom::LoadGeoms( const char *filename, phys::GeomsCallbacks *callbacks )
{
	print::Info( "phys::geom::LoadGeoms: Loading file '%s'\n", filename );
	
	FILE *file = fopen( filename, "rb" );
if( !file ) {	//####TEMP
	//char buff[256] = "/home/idf/ros/magv_simulator/vehicles/resources/";
	char buff[256] = "/home/idf/ros/magv_simulator/magv/sim_mex/src/shared/resources/";
	file = fopen( strcat(buff,filename), "rb" );
}
	if( !file ) print::Error( "phys::geom::LoadGeoms: Can not open file '%s'", filename );
	
	int version = 0;
	bool swap_endian = false;
	ReadHeader( file, swap_endian, version );
	if( version != 1 ) print::Error( "phys::geom::LoadGeoms: Unsupported version %d, expected %d", version, 1 );
	
	int index[INDEX_SIZE][2] = {};
	ReadIndex( file, swap_endian, index );

	phys::geom::Geoms *geoms = AllocGeoms( index, callbacks );

	if( geoms->callbacks )
	{
		DBG_ASSERT( phys::GeomsCallbacks::_SIZE == phys::geom::_SIZE_GEOMS-3 );
		int total=0, counts[phys::GeomsCallbacks::_SIZE] = {};
		for( int i = 1; i < phys::GeomsCallbacks::_SIZE; i++ ) {
			counts[i] = index[i+3][0];
			total += index[i][0];
		}

		geoms->callbacks->OnCounts( total, counts );
	}

	ReadGlobals( file, swap_endian, index, geoms );
	
	ReadMeshes( file, swap_endian, index, geoms );

	ReadGeomWheels    ( file, swap_endian, index[phys::geom::WHEEL   ][1], geoms );
	ReadGeomPlanes    ( file, swap_endian, index[phys::geom::PLANE   ][1], geoms );
	ReadGeomBoxes     ( file, swap_endian, index[phys::geom::BOX     ][1], geoms );
	ReadGeomSpheres   ( file, swap_endian, index[phys::geom::SPHERE  ][1], geoms );
	ReadGeomCylinders ( file, swap_endian, index[phys::geom::CYLINDER][1], geoms );
	ReadGeomCapsules  ( file, swap_endian, index[phys::geom::CAPSULE ][1], geoms );
	ReadGeomHFields   ( file, swap_endian, index[phys::geom::HFIELD  ][1], geoms );
	ReadGeomConvexes  ( file, swap_endian, index[phys::geom::CONVEX  ][1], geoms );
	ReadGeomMeshes    ( file, swap_endian, index[phys::geom::MESH    ][1], geoms );
	
	fclose( file );
	
	if( geoms->callbacks )
	{
		for( int t = 0, i = phys::geom::WHEEL; i <= phys::geom::MESH; i++ ) {
			for( int n = 0; n < geoms->num[i]; n++, t++ ) {
				int type = geoms->geom[i][n].type - 3;
				int mesh = geoms->geom[i][n].mesh;
				geoms->callbacks->OnGeom( t, type, n, mesh );
			}
		}
	}	

	return geoms;
}


const phys::geom::Geoms * phys::geom::FreeGeoms( const phys::geom::Geoms *geoms )
{
	DBG_ASSERT( geoms );
	
	std::free( (void*)geoms );
	
	return NULL;
}


void phys::geom::Initialize( void )
{
	DBG_ASSERT( physics && scene );
	
	phys::geom::Finalize();
	
	the_cylinder_convex_mesh = phys::mesh::CreateCylinderMesh( 16, 1.0f, 2.0f );	
	the_wheel_convex_mesh    = phys::mesh::CreateWheelMesh( 8, 1.0f, 2.0f );	
}


void phys::geom::Finalize( void )
{
	if( the_cylinder_convex_mesh ) {
		the_cylinder_convex_mesh->release();
		the_cylinder_convex_mesh = NULL;
	}
	
	if( the_wheel_convex_mesh ) {
		the_wheel_convex_mesh->release();
		the_wheel_convex_mesh = NULL;	
	}
}

