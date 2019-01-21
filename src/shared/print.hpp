
#ifndef __PRINT_HPP__
#define __PRINT_HPP__


#include "main.hpp"


namespace print
{
	
	void Initialize( const char *filename );
	void Finalize( void );
	
	NO_INLINE  void Print( const char *msg, ... );
	NO_INLINE  void Info ( const char *msg, ... );
	NO_INLINE  void Debug( const char *msg, ... );
	NO_INLINE  void Error( const char *msg, ... ) NO_RETURN;
	
}


#endif  // __PRINT_HPP__
