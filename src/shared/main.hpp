
#ifndef __MAIN_HPP__
#define __MAIN_HPP__



#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <vector>



#define NO_INLINE  __attribute__ ((noinline))
#define NO_RETURN  __attribute__ ((noreturn))



#include "print.hpp"

#define ASSERT( x, msg, ... )  if( !(x) ) print::Error( msg, ##__VA_ARGS__ )

#define DBG_ASSERT( x )  if( !(x) ) print::Error( "ASSERT( %s )  line=%d  file='%s'", #x, __LINE__, __FILE__ )



inline static double GetTime( void ) {
	struct timespec now;
	clock_gettime( CLOCK_MONOTONIC, &now );
	return now.tv_sec + now.tv_nsec * 1e-9;
}



inline static float RandF( float a, float b, int seed0, int seed1=0 ) { // random float number between a and b
	const int   r = seed0*3941169319u + seed1*2902958803u;
	const float f = (  ( r ^ (r>>16) ) & 0xFFFFF ) / (float) 0xFFFFF;
	return a + f * ( b - a );
}

inline static float RandI( int a, int b, int seed0, int seed1=0 ) { // random int number between a and b
	const int r = seed0*3941169319u + seed1*2902958803u;
	return a + ( r ^ (r>>16) ) % ( b - a + 1 );
}



typedef  unsigned char  byte;
typedef  unsigned short ushort;
typedef  unsigned int   uint;


struct float2 {
	float2() { }
	float2( float x, float y ) : x(x), y(y) { }
	float x, y;
};

struct float3 {
	float3() { }
	float3( float x, float y, float z ) : x(x), y(y), z(z) { }
	float x, y, z;
};

struct float4 {
	float4() { }
	float4( float x, float y, float z, float w ) : x(x), y(y), z(z), w(w) { }
	float x, y, z, w;
};

struct short3 {
	short3() { }
	short3( int x, int y, int z ) : x(x), y(y), z(z) { }
	short3( float x, float y, float z ) : x(x*0x7FFF), y(y*0x7FFF), z(z*0x7FFF) { }
	short3( const float3 &xyz ) : x(xyz.x*0x7FFF), y(xyz.y*0x7FFF), z(xyz.z*0x7FFF) { }
	short x, y, z;
};

struct short4 {
	short4() { }
	short4( int x, int y, int z, int w ) : x(x), y(y), z(z), w(w) { }
	short4( float x, float y, float z, float w ) : x(x*0x7FFF), y(y*0x7FFF), z(z*0x7FFF), w(w*0x7FFF) { }
	short4( const float4 &xyzw ) : x(xyzw.x*0x7FFF), y(xyzw.y*0x7FFF), z(xyzw.z*0x7FFF), w(xyzw.w*0x7FFF) { }
	short x, y, z, w;
};

struct char4 {
	char4() { }
	char4( int x, int y, int z, int w ) : x(x), y(y), z(z), w(w) { }
	char4( float x, float y, float z, float w ) : x(x*127.99f), y(y*127.99f), z(z*127.99f), w(w*127.99f) { }
	char x, y, z, w;
};

struct byte4 {
	byte4() { }
	byte4( int x, int y, int z, int w ) : x(x), y(y), z(z), w(w) { }
	byte4( float x, float y, float z, float w ) : x(x*255.99f), y(y*255.99f), z(z*255.99f), w(w*255.99f) { }
	byte x, y, z, w;
};

struct matrix44 {
	matrix44() { }
	matrix44( const float4 &col0, const float4 &col1, const float4 &col2, const float4 &col3 ) : col0(col0), col1(col1), col2(col2), col3(col3) {}
	operator const float*() const { return &col0.x; }
	float4 col0, col1, col2, col3;
};

template < int SIZE >
struct Chars {
	static const int size = SIZE;	
	char buff[ (SIZE+3)&~3 ];
	operator       char* ( void )       { return buff; }
	operator const char* ( void ) const { return buff; }
};

#endif  // __MAIN_HPP__
