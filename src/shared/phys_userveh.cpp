
#include <PxPhysicsAPI.h>
#include <vehicle/PxVehicleSDK.h>
#include <vehicle/PxVehicleUtil.h>
#include <vehicle/PxVehicleWheels.h>
#include <vehicle/PxVehicleDrive.h>

#include "main.hpp"
#include "util.hpp"
#include "phys.hxx"

#include "db_veh.hpp"


using namespace physx;



struct UserInput {
	bool		analog;
	bool		smoothing;
	union {
		PxVehicleDrive4WRawInputData   *input_4w;
		PxVehicleDriveNWRawInputData   *input_nw;
		PxVehicleDriveTankRawInputData *input_tank;
	};
};

struct UserVehicle {
	PxVehicleWheels				*vehicle;
	PxVehicleTelemetryData		*telemetry;
	PxVehicleWheelQueryResult	query;
	UserInput					input;
};



////////////////////////////////////////////////////////////////////////////////////////////////////
// UserVehicle array


static std::vector< UserVehicle > user_vehicles;


static UserVehicle & GetUserVehicle( const phys::UserVehicleID id ) {
	DBG_ASSERT( id.index < user_vehicles.size() );
	return user_vehicles[id.index];
}


static phys::UserVehicleID AllocUserVehicle( PxVehicleWheels *vehicle )
{
	phys::UserVehicleID id;
	
	const int num_wheels = vehicle->mWheelsSimData.getNbWheels();

	for( id.index = 0; id.index < user_vehicles.size(); id.index++ )
		if( !user_vehicles[id.index].vehicle ) break;
	
	if( id.index == user_vehicles.size() )
		user_vehicles.push_back( {} );
	
	UserVehicle &uveh = GetUserVehicle( id );
	uveh.vehicle   = vehicle;
	uveh.telemetry = PxVehicleTelemetryData::allocate( num_wheels );
	uveh.query.nbWheelQueryResults = num_wheels;
	uveh.query.wheelQueryResults   = new PxWheelQueryResult[num_wheels];
	
	uveh.input.analog    = false;
	uveh.input.smoothing = false;
	uveh.input.input_4w  = NULL;
	switch( uveh.vehicle->getVehicleType() ) {
		case PxVehicleTypes::eDRIVE4W:		uveh.input.input_4w   = new PxVehicleDrive4WRawInputData;			break;
		case PxVehicleTypes::eDRIVENW:		uveh.input.input_nw   = new PxVehicleDriveNWRawInputData;			break;
		case PxVehicleTypes::eDRIVETANK: 	uveh.input.input_tank = new PxVehicleDriveTankRawInputData(PxVehicleDriveTankControlModel::eSTANDARD);	break;	//  eSTANDARD eSPECIAL 
		case PxVehicleTypes::eNODRIVE:																	break;
		default:
			print::Error( "AllocUserVehicle: Invalid vehicle type %d", uveh.vehicle->getVehicleType() );
	}

	return id;
}


static phys::UserVehicleID FreeUserVehicle( const phys::UserVehicleID id )
{
	UserVehicle &uveh = GetUserVehicle( id );

	DBG_ASSERT( uveh.vehicle && uveh.telemetry && uveh.query.wheelQueryResults );
	
	switch( uveh.vehicle->getVehicleType() ) {
		case PxVehicleTypes::eDRIVE4W:		delete uveh.input.input_4w;		break;
		case PxVehicleTypes::eDRIVENW:		delete uveh.input.input_nw;		break;
		case PxVehicleTypes::eDRIVETANK: 	delete uveh.input.input_tank;	break;
	}
	
	delete [] uveh.query.wheelQueryResults;
	uveh.telemetry->free();
	uveh.vehicle->release();
	
	uveh = {};

	return phys::UserVehicleID();
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// User input actions


static const PxVehicleKeySmoothingData smoothing_digital_none = {		// no smoothing, instant change from current to target values
	{ 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f },
	{ 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f }
};

static const PxVehiclePadSmoothingData smoothing_analog_none = {		// no smoothing, instant change from current to target values
	{ 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f },
	{ 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f, 1e10f }
};

static const PxVehicleKeySmoothingData smoothing_digital_4w = {	//####FIXME: hardcoded
	{
		3.0f,   // rise rate eANALOG_INPUT_ACCEL
		3.0f,   // rise rate eANALOG_INPUT_BRAKE
		10.0f,  // rise rate eANALOG_INPUT_HANDBRAKE
		2.5f,   // rise rate eANALOG_INPUT_STEER_LEFT
		2.5f,   // rise rate eANALOG_INPUT_STEER_RIGHT
	}, {           
		5.0f,   // fall rate eANALOG_INPUT__ACCEL
		5.0f,   // fall rate eANALOG_INPUT__BRAKE
		10.0f,  // fall rate eANALOG_INPUT__HANDBRAKE
		5.0f,   // fall rate eANALOG_INPUT_STEER_LEFT
		5.0f    // fall rate eANALOG_INPUT_STEER_RIGHT
	}
};

static const PxVehiclePadSmoothingData smoothing_analog_4w = {	//####FIXME: hardcoded
	{
		6.0f,   // rise rate eANALOG_INPUT_ACCEL
		6.0f,   // rise rate eANALOG_INPUT_BRAKE
		12.0f,  // rise rate eANALOG_INPUT_HANDBRAKE
		2.5f,   // rise rate eANALOG_INPUT_STEER_LEFT
		2.5f,   // rise rate eANALOG_INPUT_STEER_RIGHT
	}, {           
		10.0f,  // fall rate eANALOG_INPUT_ACCEL
		10.0f,  // fall rate eANALOG_INPUT_BRAKE
		12.0f,  // fall rate eANALOG_INPUT_HANDBRAKE
		5.0f,   // fall rate eANALOG_INPUT_STEER_LEFT
		5.0f    // fall rate eANALOG_INPUT_STEER_RIGHT
	}
};

static const PxVehicleKeySmoothingData smoothing_digital_tank = {		//####TODO		//####FIXME: hardcoded
	{
		3.0f,   // rise rate eANALOG_INPUT_ACCEL
		3.0f,   // rise rate eANALOG_INPUT_BRAKE_LEFT
		3.0f,   // rise rate eANALOG_INPUT_BRAKE_RIGHT
		3.0f,   // rise rate eANALOG_INPUT_THRUST_LEFT
		3.0f,   // rise rate eANALOG_INPUT_THRUST_RIGHT
	}, {           
		3.0f,   // rise rate eANALOG_INPUT_ACCEL
		3.0f,   // rise rate eANALOG_INPUT_BRAKE_LEFT
		3.0f,   // rise rate eANALOG_INPUT_BRAKE_RIGHT
		3.0f,   // rise rate eANALOG_INPUT_THRUST_LEFT
		3.0f,   // rise rate eANALOG_INPUT_THRUST_RIGHT
	}
};

static const PxVehiclePadSmoothingData smoothing_analog_tank = {		//####TODO		//####FIXME: hardcoded
	{
		6.0f,   // rise rate eANALOG_INPUT_ACCEL
		6.0f,   // rise rate eANALOG_INPUT_BRAKE_LEFT
		6.0f,   // rise rate eANALOG_INPUT_BRAKE_RIGHT
		6.0f,   // rise rate eANALOG_INPUT_THRUST_LEFT
		6.0f,   // rise rate eANALOG_INPUT_THRUST_RIGHT
	}, {           
		6.0f,   // rise rate eANALOG_INPUT_ACCEL
		6.0f,   // rise rate eANALOG_INPUT_BRAKE_LEFT
		6.0f,   // rise rate eANALOG_INPUT_BRAKE_RIGHT
		6.0f,   // rise rate eANALOG_INPUT_THRUST_LEFT
		6.0f,   // rise rate eANALOG_INPUT_THRUST_RIGHT
	}
};

static const PxF32 steer_vs_speed_pairs[2*8] = {		//####FIXME: hardcoded
	0.0f,		0.75f,
	5.0f,		0.75f,
	30.0f,		0.125f,
	120.0f,		0.1f,
	PX_MAX_F32,	PX_MAX_F32,
	PX_MAX_F32,	PX_MAX_F32,
	PX_MAX_F32,	PX_MAX_F32,
	PX_MAX_F32,	PX_MAX_F32
};

static PxFixedSizeLookupTable<8> steer_vs_speed( steer_vs_speed_pairs, 4 );



static void FlushUserVehicleActions( const float dt, UserVehicle &uveh )
{
	switch( uveh.vehicle->getVehicleType() )
	{
		case PxVehicleTypes::eDRIVE4W: {
			PxVehicleDrive4W &vehicle = *(PxVehicleDrive4W*)uveh.vehicle;
			const bool in_air = PxVehicleIsInAir( uveh.query );
			const PxVehiclePadSmoothingData &smoothing_analog  = ( uveh.input.smoothing ? smoothing_analog_4w  : smoothing_analog_none  );
			const PxVehicleKeySmoothingData &smoothing_digital = ( uveh.input.smoothing ? smoothing_digital_4w : smoothing_digital_none );
			if(  uveh.input.analog ) PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs ( smoothing_analog,  steer_vs_speed, *uveh.input.input_4w, dt, in_air, vehicle );			
			if( !uveh.input.analog ) PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs( smoothing_digital, steer_vs_speed, *uveh.input.input_4w, dt, in_air, vehicle );			
			break;
		}
		
		case PxVehicleTypes::eDRIVENW: {
			PxVehicleDriveNW &vehicle = *(PxVehicleDriveNW*)uveh.vehicle;
			const bool in_air = PxVehicleIsInAir( uveh.query );
			const PxVehiclePadSmoothingData &smoothing_analog  = ( uveh.input.smoothing ? smoothing_analog_4w  : smoothing_analog_none  );
			const PxVehicleKeySmoothingData &smoothing_digital = ( uveh.input.smoothing ? smoothing_digital_4w : smoothing_digital_none );
			if(  uveh.input.analog ) PxVehicleDriveNWSmoothAnalogRawInputsAndSetAnalogInputs ( smoothing_analog,  steer_vs_speed, *uveh.input.input_nw, dt, in_air, vehicle );			
			if( !uveh.input.analog ) PxVehicleDriveNWSmoothDigitalRawInputsAndSetAnalogInputs( smoothing_digital, steer_vs_speed, *uveh.input.input_nw, dt, in_air, vehicle );			
			break;
		}
		
		case PxVehicleTypes::eDRIVETANK: {
			PxVehicleDriveTank &vehicle = *(PxVehicleDriveTank*)uveh.vehicle;
			const PxVehiclePadSmoothingData &smoothing_analog  = ( uveh.input.smoothing ? smoothing_analog_tank  : smoothing_analog_none  );
			const PxVehicleKeySmoothingData &smoothing_digital = ( uveh.input.smoothing ? smoothing_digital_tank : smoothing_digital_none );
			if(  uveh.input.analog ) PxVehicleDriveTankSmoothAnalogRawInputsAndSetAnalogInputs ( smoothing_analog,  *uveh.input.input_tank, dt, vehicle );			
			if( !uveh.input.analog ) PxVehicleDriveTankSmoothDigitalRawInputsAndSetAnalogInputs( smoothing_digital, *uveh.input.input_tank, dt, vehicle );			
			break;
		}
		
		case PxVehicleTypes::eNODRIVE: {
			//####TODO
			break;
		}

		default:
			print::Error( "FlushUserInput: Invalid vehicle type %d", uveh.vehicle->getVehicleType() );
	}
}



void phys::userveh::ActionMode( const UserVehicleID id, const bool analog, const bool smoothing )
{
	UserVehicle &uveh = GetUserVehicle( id );
	uveh.input.analog    = analog;
	uveh.input.smoothing = smoothing;
}

void phys::userveh::ActionAccel( const UserVehicleID id, const float input_accel )
{
	UserVehicle &uveh = GetUserVehicle( id );
	switch( uveh.vehicle->getVehicleType() ) {
		case PxVehicleTypes::eDRIVE4W:
			uveh.input.input_4w->setDigitalAccel( input_accel );
			uveh.input.input_4w->setAnalogAccel( input_accel );
			break;
		case PxVehicleTypes::eDRIVENW:
			uveh.input.input_nw->setDigitalAccel( input_accel );
			uveh.input.input_nw->setAnalogAccel( input_accel );
			break;
		case PxVehicleTypes::eDRIVETANK:
			uveh.input.input_tank->setDigitalAccel( input_accel );
			uveh.input.input_tank->setAnalogAccel( input_accel );
			break;
		case PxVehicleTypes::eNODRIVE:
			DBG_ASSERT( false );	//####TODO
			break;
	}
}

void phys::userveh::ActionSteer( const UserVehicleID id, const float input_steer )
{
	UserVehicle &uveh = GetUserVehicle( id );
	switch( uveh.vehicle->getVehicleType() ) {
		case PxVehicleTypes::eDRIVE4W:
			uveh.input.input_4w->setDigitalSteerLeft ( input_steer < 0.0f );
			uveh.input.input_4w->setDigitalSteerRight( input_steer > 0.0f );
			uveh.input.input_4w->setAnalogSteer( input_steer );
			break;
		case PxVehicleTypes::eDRIVENW:
			uveh.input.input_nw->setDigitalSteerLeft ( input_steer < 0.0f );
			uveh.input.input_nw->setDigitalSteerRight( input_steer > 0.0f );
			uveh.input.input_nw->setAnalogSteer( input_steer );
			break;
		case PxVehicleTypes::eDRIVETANK:
			DBG_ASSERT( false );	//####TODO
			break;
		case PxVehicleTypes::eNODRIVE:
			DBG_ASSERT( false );	//####TODO
			break;
	}
}

void phys::userveh::ActionBrake( const UserVehicleID id, const float input_brake )
{
	UserVehicle &uveh = GetUserVehicle( id );
	switch( uveh.vehicle->getVehicleType() ) {
		case PxVehicleTypes::eDRIVE4W:
			uveh.input.input_4w->setDigitalBrake( input_brake );
			uveh.input.input_4w->setAnalogBrake( input_brake );
			break;
		case PxVehicleTypes::eDRIVENW:
			uveh.input.input_nw->setDigitalBrake( input_brake );
			uveh.input.input_nw->setAnalogBrake( input_brake );
			break;
		case PxVehicleTypes::eDRIVETANK:
			uveh.input.input_tank->setDigitalLeftBrake ( input_brake );
			uveh.input.input_tank->setDigitalRightBrake( input_brake );
			uveh.input.input_tank->setAnalogLeftBrake  ( input_brake );
			uveh.input.input_tank->setAnalogRightBrake ( input_brake );
			break;
		case PxVehicleTypes::eNODRIVE:
			DBG_ASSERT( false );	//####TODO
			break;
	}
}

void phys::userveh::ActionHandbrake( const UserVehicleID id, const float input_handbrake )
{
	UserVehicle &uveh = GetUserVehicle( id );
	switch( uveh.vehicle->getVehicleType() ) {
		case PxVehicleTypes::eDRIVE4W:
			uveh.input.input_4w->setDigitalHandbrake( input_handbrake );
			uveh.input.input_4w->setAnalogHandbrake ( input_handbrake );
			break;
		case PxVehicleTypes::eDRIVENW:
			uveh.input.input_nw->setDigitalHandbrake( input_handbrake );
			uveh.input.input_nw->setAnalogHandbrake ( input_handbrake );
			break;
		case PxVehicleTypes::eDRIVETANK:
			uveh.input.input_tank->setDigitalLeftBrake ( input_handbrake );
			uveh.input.input_tank->setDigitalRightBrake( input_handbrake );
			uveh.input.input_tank->setAnalogLeftBrake  ( input_handbrake );
			uveh.input.input_tank->setAnalogRightBrake ( input_handbrake );
			break;
		case PxVehicleTypes::eNODRIVE:
			DBG_ASSERT( false );	//####TODO
			break;
	}
}

void phys::userveh::ActionGear( const UserVehicleID id, const int input_gear, const bool target ) {
	UserVehicle &uveh = GetUserVehicle( id );
	switch( uveh.vehicle->getVehicleType() ) {
		case PxVehicleTypes::eDRIVE4W:
			if( target ) {
				( (PxVehicleDrive*) uveh.vehicle )->mDriveDynData.setTargetGear( input_gear );
			} else {
				uveh.input.input_4w->setGearUp  ( input_gear > 0 );
				uveh.input.input_4w->setGearDown( input_gear < 0 );
			}
			break;
		case PxVehicleTypes::eDRIVENW:
			if( target ) {
				( (PxVehicleDrive*) uveh.vehicle )->mDriveDynData.setTargetGear( input_gear );
			} else {
				uveh.input.input_nw->setGearUp  ( input_gear > 0 );
				uveh.input.input_nw->setGearDown( input_gear < 0 );
			}
			break;
		case PxVehicleTypes::eDRIVETANK:
			if( target ) {
				( (PxVehicleDrive*) uveh.vehicle )->mDriveDynData.setTargetGear( input_gear );
			} else {
				uveh.input.input_tank->setGearUp  ( input_gear > 0 );
				uveh.input.input_tank->setGearDown( input_gear < 0 );
			}
			break;
		case PxVehicleTypes::eNODRIVE:
			DBG_ASSERT( false );	//####TODO
			break;
	}
}

void phys::userveh::ActionAutobox( const UserVehicleID id, const bool enable ) {
	PxVehicleDrive *veh = (PxVehicleDrive*) GetUserVehicle(id).vehicle;
	if( veh->getVehicleType() < PxVehicleTypes::eNODRIVE )
		veh->mDriveDynData.setUseAutoGears( enable );
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// Create UserVehicle from DataBase


template < typename Row >
const Row & FindByName( const std::vector<Row> &table, const char *name, const char *tag=NULL )
{
	for( const Row &row : table ) {
		if( !util::str::CmpI( row.name, name ) ) {
			return row;
		}
	}
	
	if( !tag ) tag = "";
	
	print::Error( "FindByName: Can not find '%s' (%s) in the database", name, tag );

	return table[0];
}

	
static PxVehicleWheels * CreateVehicle( const db::veh::Vehicle &db_vehicle, phys::GeomsCallbacks *callbacks )
{
	static PxMaterial *material_chassis = phys::physics->createMaterial( 0.5f, 0.5f, 0.1f );	//####TODO: material_chassis->release();
	static PxMaterial *material_wheel   = phys::physics->createMaterial( 0.5f, 0.5f, 0.1f );	//####TODO: material_wheel->release();

	const auto &db_chassis = FindByName( db::veh::chassis,       db_vehicle.name_chassis,      "CHASSIS"      );
	const auto &db_engine  = FindByName( db::veh::engines,       db_vehicle.name_engine,       "ENGINE"       );
	const auto &db_diff    = FindByName( db::veh::differentials, db_vehicle.name_differential, "DIFFERENTIAL" );
	const auto &db_clutch  = FindByName( db::veh::clutches,      db_vehicle.name_clutch,       "CLUTCH"       );
	const auto &db_gearbox = FindByName( db::veh::gearboxes,     db_vehicle.name_gearbox,      "GEARBOX"      );

	int num_wheels_total = 0;
	int num_wheels_steer = 0;
	const db::veh::Wheelset *db_wheelsets[ db::veh::Vehicle::MAX_WHEELSETS + 1 ] = { };
	for( int i = 0; i < db::veh::Vehicle::MAX_WHEELSETS; i++ ) {
		if( db_vehicle.name_wheelset[i][0] == '-' ) break;
		db_wheelsets[i] = &FindByName( db::veh::wheelsets,  db_vehicle.name_wheelset[i], "WHEELSET" );
		if( db_wheelsets[i]->max_steer != 0.0f ) num_wheels_steer += 2;
		num_wheels_total += 2;
	}

	const auto *geoms = phys::geom::LoadGeoms( db_chassis.file_collision, callbacks );
	
	if( num_wheels_total < 4 )
		print::Error( "CreateVehicle: Expected at least four wheels in vehicle '%s', found %d", db_vehicle.name, num_wheels_total );

	if( num_wheels_steer != 2 )
		print::Error( "CreateVehicle: Expected only two steering wheels in vehicle '%s', found %d", db_vehicle.name, num_wheels_steer );

	if( geoms->num[phys::geom::WHEEL] != num_wheels_total )
		print::Error( "CreateVehicle: Found %d wheels in chassis '%s' (file '%s'), expected %d wheels",
			geoms->num[phys::geom::WHEEL], db_chassis.name, db_chassis.file_collision, num_wheels_total );

	PxVehicleWheelsSimData *wheels_data = PxVehicleWheelsSimData::allocate( num_wheels_total );
	for( int i = 0; i < num_wheels_total; i++ )
	{
		const auto &db_susp  = FindByName( db::veh::suspensions, db_wheelsets[i/2]->name_suspension[i%2], "SUSPENSION" );
		const auto &db_wheel = FindByName( db::veh::wheels,      db_wheelsets[i/2]->name_wheel[i%2],      "WHEEL"      );
		const auto &db_tire  = FindByName( db::veh::tires,       db_wheelsets[i/2]->name_tire[i%2],       "TIRE"       );

		const auto &geom = geoms->geom[phys::geom::WHEEL][i];
		PxVec3 pos = geom.trans.p - geoms->mass_center;
		const int side = ( pos.y > 0.0f ? +1 : -1 );
		pos.y += side * db_wheel.width/2;

		PxVehicleSuspensionData susp;
		susp.mSpringStrength			= db_susp.spring_strength;
		susp.mSpringDamperRate			= db_susp.spring_damper_rate;
		susp.mMaxCompression			= db_susp.max_compression;
		susp.mMaxDroop					= db_susp.max_droop;
		susp.mCamberAtRest				= side * db_susp.camber_at_rest;
		susp.mCamberAtMaxCompression	= side * db_susp.camber_at_max_compression;
		susp.mCamberAtMaxDroop			= side * db_susp.camber_at_max_droop;
		susp.mSprungMass				= db_chassis.mass / wheels_data->getNbWheels();

		PxVehicleWheelData wheel;
		wheel.mRadius				= db_wheel.radius;
		wheel.mMass					= db_wheel.mass;
		wheel.mWidth				= db_wheel.width;
		wheel.mMOI					= db_wheel.moment_of_inertia;   // 0.5f * mass * radius*radius
		wheel.mDampingRate			= db_wheel.max_dampling_rate;
		wheel.mToeAngle				= db_wheel.toe_angle;
		/*wheel.mMaxBrakeTorque		= db_wheel.max_handbrake_torque;
		wheel.mMaxHandBrakeTorque	= db_wheel.max_brake_torque;*/
		wheel.mMaxBrakeTorque		= db_wheel.max_brake_torque;
		wheel.mMaxHandBrakeTorque	= db_wheel.max_handbrake_torque;
		wheel.mMaxSteer				= db_wheelsets[i/2]->max_steer * (M_PI/180);
		
		PxVehicleTireData tire;
		tire.mType									= 1;	//####TODO: TIRE_TYPE_SLICKS = 1
		tire.mLatStiffX								= db_tire.lateral_stiffness_x;
		tire.mLatStiffY								= db_tire.lateral_stiffness_y;
		tire.mLongitudinalStiffnessPerUnitGravity	= db_tire.longitudinal_stiffness;
		tire.mCamberStiffnessPerUnitGravity			= db_tire.camber_stiffness;
		//tire.mFrictionVsSlipGraph[3][2];					//####TODO

		wheels_data->setWheelShapeMapping( i, i );
		wheels_data->setSuspensionData( i, susp );
		wheels_data->setWheelData( i, wheel );
		wheels_data->setTireData( i, tire );
		wheels_data->setWheelCentreOffset      ( i, PxVec3( pos.x, pos.y, pos.z ) );
		wheels_data->setTireForceAppPointOffset( i, PxVec3( pos.x, pos.y, pos.z ) );	//####TODO
		wheels_data->setSuspForceAppPointOffset( i, PxVec3( pos.x, pos.y, pos.z ) );	//####TODO
		wheels_data->setSuspTravelDirection( i, PxVec3(0,0,-1) );
	}
	
	PxRigidDynamic *actor = phys::physics->createRigidDynamic( PxTransform::createIdentity() );

	for( int i = 0; i < geoms->array_geoms_size; i++ )
	{
		const auto &geom = geoms->array_geoms[i];
		
		DBG_ASSERT( num_wheels_total == geoms->num[phys::geom::WHEEL] );
		if( i < num_wheels_total )
		{
			DBG_ASSERT( geom.type == phys::geom::WHEEL );

			DBG_ASSERT( i < (int)wheels_data->getNbWheels() );			
			const float radius = wheels_data->getWheelData(i).mRadius;
			const float width  = wheels_data->getWheelData(i).mWidth;
			
			DBG_ASSERT( geom.geom.getType() == PxGeometryType::eCONVEXMESH );
			PxConvexMeshGeometry convex = geom.geom.convexMesh();
			convex.scale.scale = PxVec3( radius, width/2, radius );
			
			PxShape *shape = actor->createShape( convex, *material_wheel );
			shape->setFlags( shape->getFlags() & ~PxShapeFlag::eSIMULATION_SHAPE ); // eSCENE_QUERY_SHAPE
			phys::filter::Set( phys::filter::WHEEL, shape );
		}
		else
		{
			PxShape *shape = actor->createShape( geom.geom.any(), *material_chassis );
			shape->setLocalPose( geom.trans );
			phys::filter::Set( phys::filter::CHASSIS, shape );
		}
	}

	bool ok = PxRigidBodyExt::setMassAndUpdateInertia( *actor, db_chassis.mass, &geoms->mass_center );
	if( !ok ) print::Error( "CreateVehicle: PxRigidBodyExt::setMassAndUpdateInertia() -> false" );
	actor->setCMassLocalPose( PxTransform( geoms->mass_center, PxQuat(1) ) );	// fix orientation

	//actor->userData = ControllerHitReport::ObjRefToUserData( ref );

	PxVehicleWheels       *vehicle    = NULL;
	PxVehicleDriveSimData *drive_data = NULL;
	
	if( false )	// non-driven vehicle
	{
		vehicle = PxVehicleNoDrive::create( phys::physics, actor, *wheels_data );
		ASSERT( vehicle, "phys::CreateCar: PxVehicleNoDrive::create() NULL" );
	}
	else if( true )		// 4-drive + n-undrive vehicle
	{
		PxVehicleDifferential4WData differential;
		differential.mFrontRearSplit					= db_diff.split;
		differential.mFrontLeftRightSplit				= db_diff.split_front;
		differential.mRearLeftRightSplit				= db_diff.split_rear;
		differential.mCentreBias						= db_diff.bias_center;
		differential.mFrontBias							= db_diff.bias_front;
		differential.mRearBias							= db_diff.bias_rear;
		differential.mType								= PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD;		//eDIFF_TYPE_OPEN_4WD
		if( db_diff.split > 0.99f ) differential.mType	= PxVehicleDifferential4WData::eDIFF_TYPE_LS_FRONTWD;	//eDIFF_TYPE_OPEN_FRONTWD
		if( db_diff.split < 0.01f ) differential.mType	= PxVehicleDifferential4WData::eDIFF_TYPE_LS_REARWD;	//eDIFF_TYPE_OPEN_REARWD

		PxVehicleAckermannGeometryData ackermann;
		ackermann.mAccuracy	= std::min( db_wheelsets[0]->ackermann_accuracy, db_wheelsets[1]->ackermann_accuracy );	//####: ... my best guess
		float front = -1e10f;
		float rear  = +1e10f;
		for( uint i = 0; i < wheels_data->getNbWheels()/2; i++ ) {
			const PxVec3 &pos0 = wheels_data->getWheelCentreOffset( 2*i+0 );
			const PxVec3 &pos1 = wheels_data->getWheelCentreOffset( 2*i+1 );
			const float x = ( pos0.x + pos1.x ) / 2.0f;
			const float w = fabs( pos0.y - pos1.y );
			if( x > front ) ackermann.mFrontWidth = w;
			if( x < rear  ) ackermann.mRearWidth  = w;
			if( x > front ) front = x;
			if( x < rear  ) rear  = x;
			ackermann.mAxleSeparation = front - rear;
		}

		PxVehicleDriveSimData4W drive_data_4w;		
		drive_data_4w.setDiffData( differential );		
		drive_data_4w.setAckermannGeometryData( ackermann ); 	

		PxVehicleDrive4W *veh4 = PxVehicleDrive4W::create( phys::physics, actor, *wheels_data, drive_data_4w, wheels_data->getNbWheels()-4 );
		ASSERT( veh4, "phys::CreateCar: PxVehicleDrive4W::create() NULL" );

		vehicle    = veh4;
		drive_data = &veh4->mDriveSimData;
	}
	/*else if( false )	// tank vehicle	//####TODO
	{
		PxVehicleDriveSimData drive_data; //####TODO
		
		PxVehicleDriveTank *tank = PxVehicleDriveTank::create( phys::physics, actor, *wheels_data, drive_data_tank, nbDrivenWheels );
		ASSERT( tank, "phys::CreateCar: PxVehicleDriveTank::create() NULL" );

		vehicle    = tank;
		drive_data = &tank->mDriveSimData;
	}*/
	else // n-wheeled vehicle
	{
		PxVehicleDifferentialNWData differential;
		for( uint i = 0; i < wheels_data->getNbWheels(); i++ ) {
			differential.setDrivenWheel( i, true );	//####TODO
		}
		
		PxVehicleDriveSimDataNW drive_data_nw;
		drive_data_nw.setDiffData( differential );
		
		PxVehicleDriveNW *vehn = PxVehicleDriveNW::create( phys::physics, actor, *wheels_data, drive_data_nw, wheels_data->getNbWheels() );
		ASSERT( vehn, "phys::CreateCar: PxVehicleDriveNW::create() NULL" );
		
		vehicle    = vehn;
		drive_data = &vehn->mDriveSimData;
	}
	
	if( drive_data )
	{
		PxVehicleEngineData engine = drive_data->getEngineData();
		engine.mMOI										= db_engine.moment_of_inertia;
		engine.mPeakTorque								= db_engine.peack_torque;
		engine.mMaxOmega								= db_engine.max_omega;
		engine.mDampingRateFullThrottle					= db_engine.damping_rate_full;
		engine.mDampingRateZeroThrottleClutchEngaged	= db_engine.damping_rate_zero_engaged;
		engine.mDampingRateZeroThrottleClutchDisengaged	= db_engine.damping_rate_zero_disengaged;
		//engine.mTorqueCurve	= //####TODO

		PxVehicleClutchData clutch = drive_data->getClutchData();
		clutch.mStrength			= db_clutch.strength;
		clutch.mAccuracyMode		= PxVehicleClutchAccuracyMode::eBEST_POSSIBLE;		// eESTIMATE
		clutch.mEstimateIterations	= 4;		// if mAccuracyMode == eESTIMATE
		
		PxVehicleGearsData gears = drive_data->getGearsData();
		DBG_ASSERT( db::veh::Gearbox::MAX_GEARS <= PxVehicleGearsData::eGEARSRATIO_COUNT );
		gears.mSwitchTime	= db_gearbox.switch_time;
		gears.mNbRatios		= db_gearbox.num_gears;
		gears.mFinalRatio	= db_gearbox.final_ratio;
		for( int i = 0; i < PxVehicleGearsData::eGEARSRATIO_COUNT; i++ )
			gears.mRatios[i] = ( i < db_gearbox.num_gears ? db_gearbox.ratios[i] : 0.0f );
		
		PxVehicleAutoBoxData autobox = drive_data->getAutoBoxData();
		DBG_ASSERT( db::veh::Gearbox::MAX_GEARS <= PxVehicleGearsData::eGEARSRATIO_COUNT-1 );
		autobox.setLatency( db_gearbox.autobox_latency );
		for( int i = 0; i < PxVehicleGearsData::eGEARSRATIO_COUNT-1; i++ ) {
			autobox.mUpRatios[i]     = ( i < db_gearbox.num_gears ? db_gearbox.up_ratios[i]   : 0.0f );
			autobox.mDownRatios[i+1] = ( i < db_gearbox.num_gears ? db_gearbox.down_ratios[i] : 0.0f );
		}
		
		drive_data->setEngineData( engine );
		drive_data->setGearsData( gears );
		drive_data->setClutchData( clutch );
		drive_data->setAutoBoxData( autobox );
	}

	wheels_data->free();
	
	phys::geom::FreeGeoms( geoms );

	return vehicle;	
}


static PxVehicleWheels * CreateVehicle( const char *name, phys::GeomsCallbacks *callbacks )
{
	for( auto &db_vehicle : db::veh::vehicles ) {
		if( !util::str::CmpI( name, db_vehicle.name ) )
			return CreateVehicle( db_vehicle, callbacks );
	}
	
	print::Error( "CreateVehicle: Can not find vehicle name '%s'", name );
	
	return NULL;
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// Update UserVehicles


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
/*
		void UpdateVehicles( const float dt, const int num, PxVehicleNoDrive **vehicles )
		{
			DBG_ASSERT( phys::scene && this->batch_query );
			
			const auto &friction_pairs = phys::material::GetFrictionPairs();
			
			int size, total = 0;
			while( total < num )
			{
				for( size = 0; size < SIZE && total < num; total++ )
					if( vehicles[total] ) this->vehicles[size++] = vehicles[total];
				
				for( int i = 0; i < size; i++ ) DBG_ASSERT( this->vehicles[i]->mWheelsSimData.getNbWheels() == 4 );
				
				PxVehicleSuspensionRaycasts( this->batch_query, size, this->vehicles, 4*size, this->raycast_results );
				
				PxVehicleUpdates( dt, this->scene->getGravity(), friction_pairs, size, this->vehicles, this->vehicle_wheel_query );
			}
		}
*/
/*
		template < typename ID >
		void UpdateVehicles( const float dt, ent::Array< ID, physx::PxVehicleNoDrive* > &array )
		{
			DBG_ASSERT( scene && this->batch_query );
			
			const auto &friction_pairs = phys::material::GetFrictionPairs();
			
			int size = 0;
			ent::IteratorID<ID> id;
			while( id )
			{
				for( size = 0; size < SIZE && id; ++size, ++id ) {
					DBG_ASSERT( array[id] );
					this->vehicles[size] = array[id];
				}
				
				for( int i = 0; i < size; i++ ) DBG_ASSERT( this->vehicles[i]->mWheelsSimData.getNbWheels() == 4 );

				PxVehicleSuspensionRaycasts( this->batch_query, size, this->vehicles, 4*size, this->raycast_results );

				PxVehicleUpdates( dt, this->scene->getGravity(), friction_pairs, size, this->vehicles, this->vehicle_wheel_query );
			}
		}
*/
		void UpdateVehicles( const float dt, const int num, UserVehicle *vehicles )
		{
			DBG_ASSERT( phys::scene && this->batch_query );
			
			const auto &friction_pairs = phys::material::GetFrictionPairs();
			
			int size, total = 0;
			while( total < num )
			{
				const int old_total = total;
				
				int num_wheels = 0;
				for( size = 0; size < SIZE && total < num; total++ ) {
					UserVehicle &veh = vehicles[total];
					if( !veh.vehicle ) continue;
					this->vehicles[size++] = veh.vehicle;
					num_wheels += veh.vehicle->mWheelsSimData.getNbWheels();
				}
				
				DBG_ASSERT( num_wheels <= MAX_RAYCASTS );
				
				PxVehicleSuspensionRaycasts( this->batch_query, size, this->vehicles, num_wheels, this->raycast_results );
				
				for( int i = old_total; i < total; i++ ) {
					UserVehicle &veh = vehicles[i];
					if( !veh.vehicle ) continue;
					PxVehicleUpdateSingleVehicleAndStoreTelemetryData(
						dt, phys::scene->getGravity(), friction_pairs,
						veh.vehicle, &veh.query, *veh.telemetry
					);
				}
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


static VehicleBatchRaycaster< 32, 32*8 > vehicle_batch_raycaster;



/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////



phys::UserVehicleID phys::userveh::Create( const char *vehicle_name, phys::GeomsCallbacks *callbacks )
{
	PxVehicleWheels *vehicle = CreateVehicle( vehicle_name, callbacks );

  //####TEMP:
  phys::scene->addActor( *vehicle->getRigidDynamicActor() );
  //vehicle->setToRestState();
  //vehicle->getRigidDynamicActor()->setGlobalPose( PxTransform( 0, 0, 10 ) );
  static int seed = 0;
  //vehicle->getRigidDynamicActor()->setGlobalPose( PxTransform( RandF(-50,+50,seed,3), RandF(-50,+50,seed,11), 10 ) );
  vehicle->getRigidDynamicActor()->setGlobalPose( PxTransform( (seed%8)*6, -(seed/8)*4, 5 ) );
  seed++;

//	//####BUS2
//	phys::UserVehicleID id = AllocUserVehicle( vehicle );
//	vehicle->getRigidDynamicActor()->userData = phys::ped::GetUserDataFromID( id );
//	return id;
	
	return AllocUserVehicle( vehicle );
}


phys::UserVehicleID phys::userveh::Delete( const phys::UserVehicleID id )
{
	return FreeUserVehicle( id );
}



void phys::userveh::Update( const float dt )
{
	for( UserVehicle &uveh : user_vehicles )
		FlushUserVehicleActions( dt, uveh );

	/*
	for( UserVehicle &uveh : user_vehicles )	//####TEMP
	{ 
		switch( uveh.vehicle->getVehicleType() )
		{
			case PxVehicleTypes::eDRIVE4W:
			case PxVehicleTypes::eDRIVENW:
			case PxVehicleTypes::eDRIVETANK: {
				PxVehicleDrive *v = (PxVehicleDrive*) uveh.vehicle;
				//v->mDriveDynData.forceGearChange( 2 );
				v->mDriveDynData.setUseAutoGears( true );
				v->mDriveDynData.setAnalogInput( PxVehicleDrive4WControl::eANALOG_INPUT_STEER_LEFT, -0.05 );		// -1 to +1
				v->mDriveDynData.setAnalogInput( PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL,       1.00 );		//  0 to +1
				v->mDriveDynData.setAnalogInput( PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE,       0.00 );		//  0 to +1
				v->mDriveDynData.setAnalogInput( PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE,   0.00 );		//  0 to +1
				break;
			}
			
			case PxVehicleTypes::eNODRIVE: {
				PxVehicleNoDrive *v = (PxVehicleNoDrive*) uveh.vehicle;
				v->setSteerAngle( 0, 0*M_PI/180 );
				v->setSteerAngle( 1, 0*M_PI/180 );
				v->setDriveTorque( 2, 300.0f );
				v->setDriveTorque( 3, 300.0f );
				break;
			}

			default:
				print::Error( "phys::userveh::Update: Invalid vehicle type %d", uveh.vehicle->getVehicleType() );
		}
	}
	*/
	
	vehicle_batch_raycaster.UpdateVehicles( dt, user_vehicles.size(), &user_vehicles[0] );
}


void phys::userveh::Initialize( void )
{
	vehicle_batch_raycaster.Initialize();
}


void phys::userveh::Finalize( void )
{
	vehicle_batch_raycaster.Finalize();
}



/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////



#include "math.hpp"


static float3 GetShapeScale( const PxShape *shape )
{
	const PxGeometryHolder &geom = shape->getGeometry();
	switch( geom.getType() )
	{
		case PxGeometryType::eSPHERE: {
			const float radius = geom.sphere().radius;
			return float3( radius, radius, radius );
		}
		
		case PxGeometryType::eCAPSULE: {
			const float radius = geom.capsule().radius;
			const float height = geom.capsule().halfHeight;
			return float3( radius, radius, height );
		}
		
		case PxGeometryType::eBOX: {
			const PxVec3 &size = geom.box().halfExtents;
			return float3( size.x, size.y, size.z );
		}
		
		case PxGeometryType::eCONVEXMESH: {
			const PxMeshScale &trans = geom.convexMesh().scale;
			const PxVec3 scale = trans.rotation.rotate( trans.scale );
			return float3( scale.x, scale.y, scale.z );
		}
		
		case PxGeometryType::eTRIANGLEMESH: {
			const PxMeshScale &trans = geom.triangleMesh().scale;
			const PxVec3 scale = trans.rotation.rotate( trans.scale );
			return float3( scale.x, scale.y, scale.z );
		}

		default:
			print::Error( "GetShapeScale: Invalid geometry type" );
	}
	
	return float3( 0, 0, 0 );
}


static int GetActorMatrices( const PxRigidActor *actor, matrix44 &mat, int num, matrix44 mats[] )
{
	const int num_shapes = actor->getNbShapes();
	
	if( num_shapes < num ) num = num_shapes;
	
	const PxTransform &global = actor->getGlobalPose();
	const float4 ori( global.q.x, global.q.y, global.q.z, global.q.w );
	const float3 pos( global.p.x, global.p.y, global.p.z );
	math::BuildMatrix( mat, ori, pos );
	
	if( num > 0 )
	{
		PxShape *shapes[ num ];
		actor->getShapes( shapes, num );
		
		for( int i = 0; i < num; i++ ) {
			const PxTransform &trans = global.transform( shapes[i]->getLocalPose() );
			//const PxTransform &trans = shapes[i]->getLocalPose();
			const float4 ori( trans.q.x, trans.q.y, trans.q.z, trans.q.w );
			const float3 pos( trans.p.x, trans.p.y, trans.p.z );
			const float3 scl = GetShapeScale( shapes[i] );
			math::BuildMatrix( mats[i], ori, pos, scl );
		}
	}
	
	return num;
}


int phys::GetGroundMatrices( matrix44 &mat, int num, matrix44 mats[] )
{
	PxActor *actors[1] = { NULL };
	int count = phys::scene->getActors( PxActorTypeFlag::eRIGID_STATIC, actors, 1 );
	DBG_ASSERT( count && actors[0] && !util::str::CmpI( actors[0]->getName(), "world" ) );

	return GetActorMatrices( (PxRigidActor*)actors[0], mat, num, mats );
}


int phys::userveh::GetMatrices( const UserVehicleID id, matrix44 &mat, int num, matrix44 mats[] )
{
	const UserVehicle  &uveh = GetUserVehicle( id );
	ASSERT( uveh.vehicle, "phys::userveh::GetUserVehicleMatrices: NULL vehicle" );

	return GetActorMatrices( uveh.vehicle->getRigidDynamicActor(), mat, num, mats );
}


void phys::userveh::SetPositionDirection( const UserVehicleID id, const float3 &pos, const float2 &dir )
{
	const UserVehicle  &uveh = GetUserVehicle( id );
	ASSERT( uveh.vehicle, "phys::userveh::SetPositionDirection: NULL vehicle" );

	PxTransform xform( PxVec3(pos.x,pos.y,pos.z), PxQuat( atan2(dir.y,dir.x) , PxVec3(0,0,1) ) );
	uveh.vehicle->getRigidDynamicActor()->setGlobalPose( xform );
	uveh.vehicle->getRigidDynamicActor()->setLinearVelocity( PxVec3(0,0,0) );
	uveh.vehicle->getRigidDynamicActor()->setAngularVelocity( PxVec3(0,0,0) );	
}


void phys::userveh::GetPositionDirectionOrientationSpeed( const UserVehicleID id, float3 &pos, float2 &dir, short4 &ori, float &speed )
{
	const UserVehicle  &uveh = GetUserVehicle( id );
	ASSERT( uveh.vehicle, "phys::userveh::GetPositionDirectionOrientationSpeed: NULL vehicle" );

	const PxTransform &xform = uveh.vehicle->getRigidDynamicActor()->getGlobalPose();
	const PxVec3 &forward = xform.q.getBasisVector0();
	pos   = float3( xform.p.x, xform.p.y, xform.p.z );
	dir   = float2( forward.x, forward.y );
	ori   = short4( xform.q.x, xform.q.y, xform.q.z, xform.q.w );
	speed = uveh.vehicle->computeForwardSpeed();
}


void phys::userveh::GetPoseTwist( const UserVehicleID id, float3 &pose_pos, float4 &pose_ori, float3 &twist_linear, float3 &twist_angular, bool local_twist )
{
	const UserVehicle  &uveh = GetUserVehicle( id );
	ASSERT( uveh.vehicle, "phys::userveh::GetPoseTwist: NULL vehicle" );

	const PxTransform &xform = uveh.vehicle->getRigidDynamicActor()->getGlobalPose();
	/*PxVec3 linear  = uveh.vehicle->getRigidDynamicActor()->getAngularVelocity(); 
	PxVec3 angular = uveh.vehicle->getRigidDynamicActor()->getLinearVelocity();*/
	PxVec3 linear  = uveh.vehicle->getRigidDynamicActor()->getLinearVelocity(); 
	PxVec3 angular = uveh.vehicle->getRigidDynamicActor()->getAngularVelocity();

	if( local_twist ) {
		linear  = xform.q.rotateInv( linear );		//####FIXME: not tested
		angular = xform.q.rotateInv( angular );	//####FIXME: not tested
	}

	pose_pos      = float3( xform.p.x, xform.p.y, xform.p.z );
	pose_ori      = float4( xform.q.x, xform.q.y, xform.q.z, xform.q.w );
	twist_linear  = float3( linear.x, linear.y, linear.z );
	twist_angular = float3( angular.x, angular.y, angular.z );
}

static PxVec3 linear_vel, linear_vel_prev;
static double time_now, time_prev;

void phys::userveh::GetAcceleration( const UserVehicleID id, float3 &linear_accel, bool local_accel )
{
	const UserVehicle  &uveh = GetUserVehicle( id );
	ASSERT( uveh.vehicle, "phys::userveh::GetAcceleration: NULL vehicle" );
	ASSERT( (uveh.vehicle->getVehicleType()!=PxVehicleTypes::eNODRIVE), "phys::userveh::GetAcceleration: wrong vehicle" );

	const PxTransform &xform = uveh.vehicle->getRigidDynamicActor()->getGlobalPose();
	
	time_now = GetTime();
	linear_vel = uveh.vehicle->getRigidDynamicActor()->getLinearVelocity();
	PxVec3 linear_acc = (linear_vel-linear_vel_prev)/(time_now-time_prev);
	linear_vel_prev = linear_vel;
	time_prev = time_now;

	if( local_accel )
		linear_acc  = xform.q.rotateInv( linear_acc );		//####FIXME: not tested

	linear_accel  = float3( linear_acc.x, linear_acc.y, linear_acc.z );	
}

int phys::userveh::GetWheelRotationSpeed( const UserVehicleID id, float *wheels_speed, uint wheels_max )
{
	const UserVehicle  &uveh = GetUserVehicle( id );
	ASSERT( uveh.vehicle, "phys::userveh::GetWheelRotationSpeed: NULL vehicle" );
	ASSERT( uveh.vehicle->mWheelsSimData.getNbWheels()<=wheels_max, "phys::userveh::GetWheelRotationSpeed: too many wheels" );

	int nWheels = uveh.vehicle->mWheelsSimData.getNbWheels();
	
	for(int i=0; i<nWheels; i++)
		wheels_speed[i] = uveh.vehicle->mWheelsDynData.getWheelRotationSpeed(i);
	
	return nWheels;
}

void phys::userveh::GetEngineRotationSpeed( const UserVehicleID id, float &engine_speed )
{
	const UserVehicle  &uveh = GetUserVehicle( id );
	ASSERT( uveh.vehicle, "phys::userveh::GetGearSpeed: NULL vehicle" );
	ASSERT( (uveh.vehicle->getVehicleType()!=PxVehicleTypes::eNODRIVE), "phys::userveh::GetGearSpeed: wrong vehicle" );

	engine_speed  = ((PxVehicleDrive*)uveh.vehicle)->mDriveDynData.getEngineRotationSpeed();
}


void phys::userveh::GetTransmission( const UserVehicleID id, uint &gear_current, uint &gear_target, float &gear_ratio )
{
	const UserVehicle  &uveh = GetUserVehicle( id );
	ASSERT( uveh.vehicle, "phys::userveh::GetAcceleration: NULL vehicle" );
	ASSERT( (uveh.vehicle->getVehicleType()!=PxVehicleTypes::eNODRIVE), "phys::userveh::GetAcceleration: wrong vehicle" );	
	
	gear_current  = ((PxVehicleDrive*)uveh.vehicle)->mDriveDynData.getCurrentGear();
	gear_target  = ((PxVehicleDrive*)uveh.vehicle)->mDriveDynData.getTargetGear();
	gear_ratio = 4.0f;
}