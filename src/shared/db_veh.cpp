
#include "main.hpp"
#include "xlsx.hpp"
#include "db_veh.hpp"



static xlsx::Strings *strings;



static bool DataToBool( const char *data )
{
	const char *s = strings->Get( data );
	
	while( *s && *s <= ' ' ) s++;	
	
	if( s[0] == '0' || s[0] == '-' ) return false;
	if( s[0] == 'N' || s[0] == 'n' ) return false;
	if( s[0] == 'F' || s[0] == 'f' ) return false;
	if( s[0] == 'O' && s[1] == 'F' ) return false;
	if( s[0] == 'O' && s[1] == 'f' ) return false;
	if( s[0] == 'o' && s[1] == 'f' ) return false;
	
	return true;
}

static int DataToInteger( const char *data )
{
	return std::atoi( data );
}

static double DataToFloat( const char *data )
{
	return std::atof( data );
}

static const char * DataToString( const char *data )
{
	return strings->Get( data );
}

template < int SIZE >
static int DataToFloatArray( const char *data, float (&array)[SIZE] )
{
	const char *s = strings->Get( data );
	
	int n = 0;
	
	while( *s ) {
		if( n >= SIZE ) print::Error( "DataToFloats: Max floats size reached" );		
		while( *s && *s <= ' ' ) s++;
		if( !*s ) break;
		array[n++] = std::atof( s );
		while( *s && *s != ';' ) s++;
		if( *s ) s += 1;
	}	
	
	return n;
}



template < typename ROW >
struct Table : xlsx::TableReaderListener {
	typedef ROW Row;
	std::vector< Row > table;
};


struct Surfaces : Table< db::veh::Surface > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().friction_static  = DataToFloat( data );		break;
			case 1:  table.back().friction_dynamic = DataToFloat( data );		break;
			case 2:  table.back().restitution      = DataToFloat( data );		break;
		}
	}
};


struct Suspensions : Table< db::veh::Suspension > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name						= DataToString( data ); 	break;
			case 1:  table.back().spring_strength			= DataToFloat( data );		break;
			case 2:  table.back().spring_damper_rate		= DataToFloat( data );		break;
			case 3:  table.back().max_compression			= DataToFloat( data );		break;
			case 4:  table.back().max_droop					= DataToFloat( data );		break;
			case 5:  table.back().camber_at_rest			= DataToFloat( data );		break;
			case 6:  table.back().camber_at_max_compression	= DataToFloat( data );		break;
			case 7:  table.back().camber_at_max_droop		= DataToFloat( data );		break;
		}
	}
};


struct Wheels : Table< db::veh::Wheel > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name					= DataToString( data ); 	break;
			case 1:  table.back().file_visual			= DataToString( data ); 	break;
			case 2:  table.back().radius				= DataToFloat( data );		break;
			case 3:  table.back().width					= DataToFloat( data );		break;
			case 4:  table.back().mass					= DataToFloat( data );		break;
			case 5:  table.back().moment_of_inertia		= DataToFloat( data );		break;
			case 6:  table.back().max_dampling_rate		= DataToFloat( data );		break;
			case 7:  table.back().toe_angle				= DataToFloat( data );		break;
			case 8:  table.back().max_brake_torque		= DataToFloat( data );		break;
			case 9:  table.back().max_handbrake_torque	= DataToFloat( data );		break;
		}
	}
};


struct Tires : Table< db::veh::Tire > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name					 = DataToString( data );	break;
			case 1:  table.back().lateral_stiffness_x	 = DataToFloat( data );		break;
			case 2:  table.back().lateral_stiffness_y	 = DataToFloat( data );		break;
			case 3:  table.back().longitudinal_stiffness = DataToFloat( data );		break;
			case 4:  table.back().camber_stiffness		 = DataToFloat( data );		break;
			default:
				DBG_ASSERT( col-5 >= 0 && col-5 < db::veh::Surface::NUM_SURFACES );
				table.back().friction[col-5] = DataToFloat( data );
		}
	}
};


struct Wheelsets : Table< db::veh::Wheelset > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name					= DataToString( data ); 			break;
			case 1:  table.back().name_suspension[0]	= DataToString( data ); 			break;
			case 2:  table.back().name_suspension[1]	= DataToString( data ); 			break;
			case 3:  table.back().name_wheel[0]			= DataToString( data ); 			break;
			case 4:  table.back().name_wheel[1]			= DataToString( data ); 			break;
			case 5:  table.back().name_tire[0]			= DataToString( data );				break;
			case 6:  table.back().name_tire[1]			= DataToString( data );				break;
			case 7:  table.back().max_steer				= DataToFloat( data );				break;
			case 8:  table.back().ackermann_accuracy	= DataToFloat( data );				break;
		}
	}
};


struct Chassiss : Table< db::veh::Chassis > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name				= DataToString( data ); 	break;
			case 1:  table.back().file_collision	= DataToString( data ); 	break;
			case 2:  table.back().file_visual		= DataToString( data );		break;
			case 3:  table.back().mass				= DataToFloat( data );		break;
		}
	}
};


struct Engines : Table< db::veh::Engine > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name							= DataToString( data ); 	break;
			case 1:  table.back().moment_of_inertia				= DataToFloat( data );		break;
			case 2:  table.back().peack_torque					= DataToFloat( data );		break;
			case 3:  table.back().max_omega						= DataToFloat( data );		break;
			case 4:  table.back().damping_rate_full				= DataToFloat( data );		break;
			case 5:  table.back().damping_rate_zero_engaged		= DataToFloat( data );		break;
			case 6:  table.back().damping_rate_zero_disengaged	= DataToFloat( data );		break;
		}
	}
};


struct Differentials : Table< db::veh::Differential > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name			= DataToString( data ); 	break;
			case 1:  table.back().split			= DataToFloat( data );		break;
			case 2:  table.back().split_front	= DataToFloat( data );		break;
			case 3:  table.back().split_rear	= DataToFloat( data );		break;
			case 4:  table.back().bias_center	= DataToFloat( data );		break;
			case 5:  table.back().bias_front	= DataToFloat( data );		break;
			case 6:  table.back().bias_rear		= DataToFloat( data );		break;
		}
	}
};


struct Clutches : Table< db::veh::Clutch > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name			= DataToString( data ); 	break;
			case 1:  table.back().strength		= DataToFloat( data );		break;
		}
	}
};


struct Gearboxes : Table< db::veh::Gearbox > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name				= DataToString( data ); 	break;
			case 1:  table.back().switch_time		= DataToFloat( data );		break;
			case 2:  table.back().final_ratio		= DataToFloat( data );		break;
			case 4:  table.back().autobox_latency	= DataToFloat( data );		break;
			case 3:
				row = DataToFloatArray( data, table.back().ratios );
				table.back().num_gears = row;
				break;
			case 5:
				row = DataToFloatArray( data, table.back().up_ratios );
				if( row != table.back().num_gears-1 )
					print::Error( "VehicleDatabase: Clutch '%s', found %d up-ratios, expected %d",  table.back().name, row, table.back().num_gears-1 );
				break;
			case 6:
				row = DataToFloatArray( data, table.back().down_ratios );
				if( row != table.back().num_gears-1 )
					print::Error( "VehicleDatabase: Clutch '%s', found %d down-ratios, expected %d",  table.back().name, row, table.back().num_gears-1 );
				break;
		}
	}
};


struct Vehicles : Table< db::veh::Vehicle > {
	void OnValue( int row, int col, int size, const char *data ) {
		if( row <= 1 || !data ) return;
		if( col == 0 ) table.emplace_back();
		switch( col ) {
			case 0:  table.back().name					= DataToString( data );		break;
			case 1:  table.back().name_chassis			= DataToString( data );		break;
			case 2:  table.back().name_engine			= DataToString( data );		break;
			case 3:  table.back().name_differential		= DataToString( data );		break;
			case 4:  table.back().name_clutch			= DataToString( data );		break;
			case 5:  table.back().name_gearbox			= DataToString( data );		break;
			case 6:  table.back().name_wheelset[0]		= DataToString( data );		break;
			case 7:  table.back().name_wheelset[1]		= DataToString( data );		break;
			case 8:  table.back().name_wheelset[2]		= DataToString( data );		break;
			case 9:  table.back().name_wheelset[3]		= DataToString( data );		break;
		}
	}
};


	
#define DEFINE_TABLE( Table, name )  	\
	static Table _##name;				\
	const std::vector< Table::Row > &name = _##name.table;  // extern

namespace db {
	namespace veh {
		DEFINE_TABLE( Surfaces,      surfaces      );
		DEFINE_TABLE( Suspensions,   suspensions   );
		DEFINE_TABLE( Wheels,        wheels        );
		DEFINE_TABLE( Tires,         tires         );
		DEFINE_TABLE( Wheelsets,     wheelsets     );
		DEFINE_TABLE( Chassiss,      chassis       );
		DEFINE_TABLE( Engines,       engines       );
		DEFINE_TABLE( Differentials, differentials );
		DEFINE_TABLE( Clutches,      clutches      );
		DEFINE_TABLE( Gearboxes,     gearboxes     );
		DEFINE_TABLE( Vehicles,      vehicles      );
	}
}



void db::veh::LoadFile( const char *filename_xlsx )
{
	db::veh::Finalize();
	
	xlsx::File file( filename_xlsx );
	
	strings = file.AllocStrings();
	
	file.ReadTable(  1, _surfaces      );
	file.ReadTable(  2, _suspensions   );
	file.ReadTable(  3, _wheels        );
	file.ReadTable(  4, _tires         );
	file.ReadTable(  5, _wheelsets     );
	file.ReadTable(  6, _chassis       );
	file.ReadTable(  7, _engines       );
	file.ReadTable(  8, _differentials );
	file.ReadTable(  9, _clutches      );
	file.ReadTable( 10, _gearboxes     );
	file.ReadTable( 11, _vehicles      );
}


void db::veh::Initialize( void )
{
	db::veh::Finalize();
}


void db::veh::Finalize( void )
{
	if( strings ) std::free( strings );
	strings = NULL;
}



