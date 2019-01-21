/*
**	AUTHORS:
**		Leopoldo Armesto
**		Juan Dols
**		Jaime Molina
**
**	MAGV Simulator by Leopoldo Armesto is licensed under a Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
**	To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/deed.en_GB.
**
**  EMAIL: larmesto@idf.upv.es
**  URL: www.upv.es
*/


/// \cond PRIVATE


#include "nav.hxx"



void nav::Initialize( void )
{
	nav::veh::Initialize();
	nav::ped::Initialize();
	nav::sem::Initialize();
}


void nav::Finalize( void )
{
	nav::veh::Finalize();
	nav::ped::Finalize();
	nav::sem::Finalize();
}


void nav::Update( const float dt )
{
	nav::veh::Update( dt );
	nav::ped::Update( dt );
	nav::sem::Update( dt );
}

