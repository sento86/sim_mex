
#ifndef __UTIL_HPP__
#define __UTIL_HPP__


#include "main.hpp"


namespace util
{
	
	namespace str
	{
		
		int Cmp ( const char *str0, const char *str1 );
		int CmpI( const char *str0, const char *str1 );
		
		int Copy( const int size, char *out, const char *str );
				
		int Join( int size, char *out, const char *str0=NULL, const char *str1=NULL, const char *str2=NULL );
		
		template < int SIZE >
		int Copy( char (&out)[SIZE], const char *str ) {
			return Copy( SIZE, out, str );
		}

		template < int SIZE >
		int Join( char (&out)[SIZE], const char *str0=NULL, const char *str1=NULL, const char *str2=NULL ) {
			return Join( SIZE, out, str0, str1, str2 );
		}
	}


	namespace endian
	{
		static const bool BIG    = ( *(int*)"\xA\xB\xC\xD" == 0xABCD );
		static const bool LITTLE = !BIG;

		inline bool Test( int four_int_bytes, const char *four_char_bytes ) {
			return ( *(int*)four_char_bytes == four_int_bytes );
		}

		inline unsigned short Swap( unsigned short x ) {  return ( x << 8 ) | ( x >> 8 );  }
		inline unsigned int   Swap( unsigned int   x ) {  return ( x << 24 ) | ( (x<<8) & 0x00FF0000 ) | ( (x>>8) & 0x0000FF00 ) | ( x >> 24 );  }

		inline short Swap( short x ) {  return (short) Swap( (unsigned short) x );  }
		inline int   Swap( int   x ) {  return (int  ) Swap( (unsigned int  ) x );  }
	
		template < typename T >
		inline void Swap( int num, T *data ) {
			if( sizeof(T) == 4 ) {
				auto *x = (unsigned int*) data;
				for( int i = 0; i < num; i++ )
					x[i] = Swap( x[i] );
			} else if( sizeof(T) == 2 ) {
				auto *x = (unsigned short*) data;
				for( int i = 0; i < num; i++ )
					x[i] = Swap( x[i] );
			}
		}
	}
}



#endif  // __UTIL_HPP__
