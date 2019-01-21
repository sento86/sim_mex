
#ifndef __PHYS_HXX__
#define __PHYS_HXX__


#include <vector>

#include <PxPhysicsAPI.h>

#include "phys.hpp"


namespace phys
{
	
	using namespace physx;
	
	
	extern PxFoundation			*foundation;
	extern PxPhysics			*physics;
	extern PxCooking			*cooking;
	extern PxScene				*scene;
	extern PxControllerManager	*manager;


	namespace filter
	{
		void Initialize( void );
		void Finalize( void );
		
		enum Filter { NONE=0, GROUND, OBSTACLE, CHASSIS, WHEEL };
		
		void Set( filter::Filter f, PxShape *shape );		
		void Set( filter::Filter f, PxRigidActor *actor );

		PxBatchQueryPreFilterShader GetWheelRaycastPreFilter( void );
	}
	
	
	namespace material
	{
		void Initialize( void );
		void Finalize( void );
		
		enum Surface {
			SURFACE_TARMAC = 0,
			SURFACE_MUD,
			SURFACE_GRASS,
			SURFACE_SNOW,
			SURFACE_ICE,
		};
		
		const PxVehicleDrivableSurfaceToTireFrictionPairs & GetFrictionPairs( void );
	}
	
	
	namespace mesh
	{
		void Initialize( void );
		void Finalize( void );
		
		template < typename T > // T = byte, ushort, int
			PxTriangleMesh * CreateTriangleMesh( int num_points, int num_indexes, float3 points[], T triangles[] );
		
		template < typename T > // T = byte, ushort, int
			PxTriangleMesh * CreatePolygonalMesh( int num_points, int num_indexes, float3 points[], T polygons[], int num_polygons_check=-1 );

		PxHeightField  * CreateHeightField( int num_points_x, int num_points_y, float zscale, short *heights );
		
		PxConvexMesh   * CreateConvexMesh( int num, float3 points[] );		
		PxConvexMesh   * CreateCylinderMesh( int sides, float radius, float heigh );		
		PxConvexMesh   * CreateWheelMesh( int sides, float radius, float width );
	}
	
	
	namespace geom
	{
		enum {  		// do not change order, match file index
			NONE = 0,
			MESH_HFIELD,
			MESH_CONVEX,
			MESH_MESH,
			WHEEL,
			PLANE,
			BOX,
			SPHERE,
			CYLINDER,
			CAPSULE,
			HFIELD,
			CONVEX,
			MESH,
			_SIZE_GEOMS
		};
		
		struct Mesh {
			int		type;
			void	*ptr;
		};

		struct Geom {
			int				 type;
			int				 mesh;
			PxTransform		 trans;
			PxGeometryHolder geom;
		};
		
		struct Geoms {
			GeomsCallbacks *callbacks;
			PxVec3	 bbox_size;
			PxVec3	 bbox_center;
			PxVec3	 mass_center;
			int		 array_meshes_size;
			int		 array_geoms_size;
			Mesh	*array_meshes;
			Geom	*array_geoms;
			int      num[_SIZE_GEOMS];
			Geom	*geom[_SIZE_GEOMS];
		};
		
		
		void Initialize( void );
		void Finalize( void );
		
		const Geoms * LoadGeoms( const char *filename, phys::GeomsCallbacks *callbacks );
		const Geoms * FreeGeoms( const Geoms *geoms );
	}


	namespace userveh {
		void Initialize( void );
		void Finalize( void );
		void Update( const float dt );
	}

//	namespace veh {
//		void Initialize( void );
//		void Finalize( void );
//		void Update( const float dt );
//	}
//
//	namespace ped {
//		void Initialize( void );
//		void Finalize( void );
//		void Update( const float dt );
//		void * GetUserDataFromID( const phys::PedestrianID  idped  );
//		void * GetUserDataFromID( const phys::VehicleID     idveh  );
//		void * GetUserDataFromID( const phys::UserVehicleID iduveh );
//	}

}  // namespace phys


#endif  // __PHYS_HXX__