
#include "main.hpp"
#include "util.hpp"




int util::str::Cmp( const char *str0, const char *str1 )
{
	while( true )
	{
		int a = *str0++;
		int b = *str1++;
		
		if( a != b || !a )
			return a - b;
	}
	
	return 0;
}


inline static int NormalizeChar( int x )
{
	if( x >= 'a' && x <= 'z' ) return x + ( 'A' - 'a' );
	
	if( x <= ' ' ) return ( x ? '_' : 0 );
	
	if( x == '\\' ) return '/';
	
	if( x >= 127 ) return '?';
	
	return x;
}


int util::str::CmpI( const char *str0, const char *str1 )
{
	while( true )
	{
		int a = NormalizeChar( *str0++ );
		int b = NormalizeChar( *str1++ );
		
		if( a != b || !a )
			return a - b;
	}
	
	return 0;
}


int util::str::Copy( const int size, char *out, const char *str )
{
	int len = 0;
	
	while( *str && len < size-1 )
		out[len++] = *str++;
	
	out[len] = '\0';
	
	return len;
}


int util::str::Join( int size, char *out, const char *str0, const char *str1, const char *str2 )
{
	int len = 0;
	
	if( str0 ) while( *str0 && len < size-1 ) out[len++] = *str0++;
	if( str1 ) while( *str1 && len < size-1 ) out[len++] = *str1++;
	if( str2 ) while( *str2 && len < size-1 ) out[len++] = *str2++;
	
	out[len] = '\0';
	
	return len;
}


