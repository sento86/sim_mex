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


#ifndef __NAV_HXX__
#define __NAV_HXX__



/// \cond PRIVATE



#include "nav.hpp"


namespace nav // navigation
{

	
	namespace veh // vehicle
	{		
		void Initialize( void );		///< Inicializa recursos para gestionar los vehículos.
		void Finalize( void );			///< Libera recursos.	
		void Update( const float dt );
	}
	

	namespace ped // pedestrian
	{		
		void Initialize( void );		///< Inicializa recursos para gestionar los peatones.
		void Finalize( void );			///< Libera recursos.
		void Update( const float dt );		
	}


	namespace sem // semaphore
	{
		void Initialize( void );		///< Inicializa recursos para gestionar los semáforos.
		void Finalize( void );			///< Libera recursos.	
		void Update( const float dt );	///< Actualiza el estado de los tipos de semáforos.
	}
	

} // namespace nav
	

#endif // __NAV_HXX__
