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


#include <stdio.h>
#include <assert.h>

#include "nav.hxx"



namespace nav
{
	namespace sem
	{
		/// Información sobre el estado de los tipos de semáforo.
		struct Semaphore {
			unsigned char secs_total;	///< Tiempo de ciclo o periodo.
			unsigned char secs_green;	///< Tiempo en estado verde.
			unsigned char secs_phase;	///< Tiempo de fase.
			unsigned char _padding;
		};
		
		/// Todos los tipos de semáforos permitidos.
		static Semaphore semaphores[nav::sem::MAX];
		
		/// Tiempo global para sincronizar todos los semáforos (segundos).
		/// Este tiempo se actualiza en nav::sem::Update(), llamado desde nav::Update(). \n
		static unsigned int time_secs;
		
		/// Fración de segundo sobrante de sem::time_secs a tener en cuenta en la próxima actualización.
		static float time_remaining;
	}
}	


bool nav::sem::Load( const char *file )
{
	FILE *f = fopen( file, "rt" );
	if( !f ) return false;
	
	int idx, total, green, phase;
	char buff[256];
	
	while( !feof( f ) )
	{
		fgets( buff, sizeof(buff), f );
		
		int num = sscanf( buff, "%d%d%d%d", &idx, &total, &green, &phase );
		if( num != 4 ) continue;

		if( idx < 1 || idx >= nav::sem::MAX ) return false;
		if( green < 1 || total < 1 || green > total ) return false;

		semaphores[idx].secs_total = total;
		semaphores[idx].secs_green = green;
		semaphores[idx].secs_phase = phase;
	}
	
	fclose( f );
	
	return true;
}


void nav::sem::SetTimes( const int idx, int secs_total, int secs_green, int secs_phase )
{
	assert( idx >= 0 && idx < nav::sem::MAX );
	nav::sem::Semaphore &s = nav::sem::semaphores[idx];
	s.secs_total = secs_total;
	s.secs_green = secs_green;
	s.secs_phase = secs_phase;
}


bool nav::sem::IsGreen( const int idx )
{
	assert( idx >= 0 && idx < nav::sem::MAX );
	const nav::sem::Semaphore &s = nav::sem::semaphores[idx];
	return ( ( time_secs + s.secs_phase ) % s.secs_total < s.secs_green );
}


void nav::sem::Initialize( void )
{
	nav::sem::Finalize();
}


void nav::sem::Finalize( void )
{
	nav::sem::time_remaining = 0.0f;
	nav::sem::time_secs = 0;
	
	nav::sem::Semaphore def = { 30, 30, 0, 0 };
	for( int i = 0; i < nav::sem::MAX; i++ )
		nav::sem::semaphores[i] = def;
}


void nav::sem::Update( const float dt )
{
	const float t = time_remaining + dt;
	nav::sem::time_remaining = t - (int)t;
	nav::sem::time_secs += (int)t;
}

