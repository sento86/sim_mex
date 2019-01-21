
#include <cstdio>
#include <cstdlib>
#include <cstdarg>

#include "print.hpp"



static FILE *out;



static void PrintOut( const char *tag, const char *format, va_list &args )
{
	char buff[512];
	
	vsnprintf ( buff, sizeof(buff), format, args );
	buff[ sizeof(buff) - 1 ] = '\0';
	
	if( tag ) fputs( tag, stderr );
	fputs( buff, stderr );
	
	if( out ) {
		if( tag ) fputs( tag, out );
		fputs( buff, out );
	}
}



void print::Print( const char *msg, ... )
{
	va_list args;
	va_start( args, msg );
	PrintOut( NULL, msg, args );
	va_end( args );
}


void print::Info( const char *msg, ... )
{
	va_list args;
	va_start( args, msg );
	PrintOut( "INFO: ", msg, args );
	va_end( args );
}


void print::Debug( const char *msg, ... )
{
	va_list args;
	va_start( args, msg );
	PrintOut( "DEBUG: ", msg, args );
	va_end( args );
}


void print::Error( const char *msg, ... )
{
	va_list args;
	va_start( args, msg );
	PrintOut( "ERROR: ", msg, args );
	va_end( args );
	
	*(volatile int*)0 = 0;	// hardcoded breakpoint
	
	exit( -1 );
}


void print::Initialize( const char *filename )
{
	print::Finalize();
	
	if( filename )
	{
		out = fopen( filename, "wt" );
		if( !out ) print::Error( "log::Initialize: Can not open file '%s'", filename );
	}	
}


void print::Finalize( void )
{
	if( out ) fclose( out );
	out = NULL;
}

