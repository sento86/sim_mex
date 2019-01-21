
#ifndef __MATH_HPP__
#define __MATH_HPP__

#include <cmath>

#include "main.hpp"



inline float3 operator + ( const float3 &a, const float3 &b ) {
	return float3( a.x+b.x, a.y+b.y, a.z+b.z );
}

inline float3 operator - ( const float3 &a, const float3 &b ) {
	return float3( a.x-b.x, a.y-b.y, a.z-b.z );
}

inline float3 operator * ( const float3 &xyz, const float m ) {
	return float3( m*xyz.x, m*xyz.y, m*xyz.z );
}

inline float3 operator * ( const float m, const float3 &xyz ) {
	return float3( m*xyz.x, m*xyz.y, m*xyz.z );
}

inline float Dot( const float3 &a, const float3 &b ) {
	return a.x*b.x + a.y*b.y + a.z*b.z;
}


inline float4 operator * ( const float4 &xyzw, const float m ) {
	return float4( m*xyzw.x, m*xyzw.y, m*xyzw.z, m*xyzw.w );
}

inline float4 operator * ( const float m, const float4 &xyzw ) {
	return float4( m*xyzw.x, m*xyzw.y, m*xyzw.z, m*xyzw.w );
}

inline float Dot( const float4 &a, const float4 &b ) {
	return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

inline float4 Cross( const float4 &a, const float4 &b ) {
	return float4(
		a.y*b.z - a.z*b.y,
		a.z*b.x - a.x*b.z,
		a.x*b.y - a.y*b.x,
		0.0f
	);
}



namespace math
{

	static const matrix44 MATRIX_IDENTITY = { { 1,0,0,0 }, { 0,1,0,0 }, { 0,0,1,0 }, { 0,0,0,1 } };
	
	
	inline void BuildMatrix( matrix44 &mat, const float2 &dir, const float3 &pos ) {
		mat.col0 = float4( +dir.x, +dir.y, 0.000f, 0.0f );
		mat.col1 = float4( -dir.y, +dir.x, 0.000f, 0.0f );
		mat.col2 = float4( 0.000f, 0.000f, 1.000f, 0.0f );
		mat.col3 = float4( pos.x, pos.y, pos.z, 1.0f );
	}

	inline void BuildMatrix( matrix44 &mat, const float2 &dir, const float3 &pos, const float3 &scale ) {
		mat.col0 = float4( +dir.x, +dir.y, 0.000f, 0.0f ) * scale.x;
		mat.col1 = float4( -dir.y, +dir.x, 0.000f, 0.0f ) * scale.y;
		mat.col2 = float4( 0.000f, 0.000f, 1.000f, 0.0f ) * scale.z;
		mat.col3 = float4( pos.x, pos.y, pos.z, 1.0f );
	}
	
	inline void BuildMatrix( matrix44 &mat, const float4 &ori, const float3 &pos ) {
		mat.col0 = float4( 1.0f - 2.0f*ori.y*ori.y - 2.0f*ori.z*ori.z, 2.0f*ori.x*ori.y + 2.0f*ori.z*ori.w, 2.0f*ori.x*ori.z - 2.0f*ori.y*ori.w, 0.0f );
		mat.col1 = float4( 2.0f*ori.x*ori.y - 2.0f*ori.z*ori.w, 1.0f - 2.0f*ori.x*ori.x - 2.0f*ori.z*ori.z, 2.0f*ori.y*ori.z + 2.0f*ori.x*ori.w, 0.0f );
		mat.col2 = float4( 2.0f*ori.x*ori.z + 2.0f*ori.y*ori.w, 2.0f*ori.y*ori.z - 2.0f*ori.x*ori.w, 1.0f - 2.0f*ori.x*ori.x - 2.0f*ori.y*ori.y, 0.0f );
		mat.col3 = float4( pos.x, pos.y, pos.z, 1.0f );
	}

	inline void BuildMatrix( matrix44 &mat, const float4 &ori, const float3 &pos, const float3 &scale ) {
		mat.col0 = float4( 1.0f - 2.0f*ori.y*ori.y - 2.0f*ori.z*ori.z, 2.0f*ori.x*ori.y + 2.0f*ori.z*ori.w, 2.0f*ori.x*ori.z - 2.0f*ori.y*ori.w, 0.0f ) * scale.x;
		mat.col1 = float4( 2.0f*ori.x*ori.y - 2.0f*ori.z*ori.w, 1.0f - 2.0f*ori.x*ori.x - 2.0f*ori.z*ori.z, 2.0f*ori.y*ori.z + 2.0f*ori.x*ori.w, 0.0f ) * scale.y;
		mat.col2 = float4( 2.0f*ori.x*ori.z + 2.0f*ori.y*ori.w, 2.0f*ori.y*ori.z - 2.0f*ori.x*ori.w, 1.0f - 2.0f*ori.x*ori.x - 2.0f*ori.y*ori.y, 0.0f ) * scale.z;
		mat.col3 = float4( pos.x, pos.y, pos.z, 1.0f );
	}
	
	/// Crea una matriz a partir de los ángulos y una posición.
	/// \param [out] mat    Matriz de vista generada.
	/// \param  [in] ang    Ángulos de la cámara en radianes: ( x=pitch, y=yaw, z=roll ).
	/// \param  [in] pos    Posición de la cámara.
	inline void BuildMatrix( matrix44 &mat, const float3 &ang, const float3 &pos ) {
		const float sx = sin(-ang.x), cx = cos(-ang.x);
		const float sy = sin(-ang.y), cy = cos(-ang.y);
		const float sz = sin(-ang.z), cz = cos(-ang.z);
		mat.col0 = float4( cy*cz+sy*sx*sz, sy*sx*cz-cy*sz, sy*cx, 0.0f );
		mat.col1 = float4(          cx*sz,          cx*cz,   -sx, 0.0f );
		mat.col2 = float4( cy*sx*sz-sy*cz, cy*sx*cz+sy*sz, cy*cx, 0.0f );
		mat.col3 = float4(          pos.x,          pos.y, pos.z, 1.0f );		
		//mat.col3 = float4(          pos.y,          pos.x, pos.z, 1.0f );		
	}

	inline void BuildMatrix( matrix44 &mat, const float3 &from, const float3 &to, const float3 &up ) {
		mat.col2 = float4( from.x-to.x, from.y-to.y, from.z-to.z, 0.0f );
		const float m = mat.col2.x*mat.col2.x + mat.col2.y*mat.col2.y + mat.col2.z*mat.col2.z;
		if( m > 0.0001f ) mat.col2 = mat.col2 * ( 1.0f / sqrt( m ) );
		mat.col0 = Cross( float4(up.x,up.y,up.z,0.0f), mat.col2 );
		mat.col1 = Cross( mat.col2, mat.col0 );
		mat.col3 = float4( from.x, from.y, from.z, 1.0f );
	}

	/// Crea una matriz de proyección en perspectiva.
	/// \param [out] mat    Matriz de proyección generada.
	/// \param  [in] fovy   Field of view o ángulo de visión en el eje Y (grados).
	/// \param  [in] aspect Relación de aspecto = ancho/alto --> 4:3=1.3333, 16:9=1.77777 ...
	/// \param  [in] near   Plano de corte cercano.
	/// \param  [in] far    Plano de corte lejano.
	inline void BuildMatrix( matrix44 &mat, const float fovy, const float aspect, const float near, const float far ) {
		const float n = near;
		const float f = far;
		const float fy = 1.0f / tan( fovy / 2 * (M_PI/180) );
        const float fx = fy / aspect;
		mat.col0 = float4( fx,  0,           0,  0 );
		mat.col1 = float4(  0, fy,           0,  0 );
		mat.col2 = float4(  0,  0, (f+n)/(n-f), -1 );
		mat.col3 = float4(  0,  0, 2*f*n/(n-f),  0 );
	}

	inline void FastInvertMatrix( matrix44 &mat, const matrix44 &orig ) {	// orthonormal 3x3 rotation matrix
		DBG_ASSERT( &mat != &orig );
		mat.col0 = float4( orig.col0.x, orig.col1.x, orig.col2.x, 0.0f );
		mat.col1 = float4( orig.col0.y, orig.col1.y, orig.col2.y, 0.0f );
		mat.col2 = float4( orig.col0.z, orig.col1.z, orig.col2.z, 0.0f );
		mat.col3 = float4(
			-Dot( orig.col0, orig.col3 ),
			-Dot( orig.col1, orig.col3 ),
			-Dot( orig.col2, orig.col3 ),
			1.0f
		);
	}

	inline void TranslateGlobal( matrix44 &mat, const float3 &displacement ) {
		mat.col3 = float4( mat.col3.x + displacement.x, mat.col3.y + displacement.y, mat.col3.z + displacement.z, 1.0f );
	}

	inline void TranslateLocal( matrix44 &mat, const float3 &displacement ) {
		const float local_x = displacement.x * mat.col0.x + displacement.y * mat.col1.x + displacement.z * mat.col2.x;
		const float local_y = displacement.x * mat.col0.y + displacement.y * mat.col1.y + displacement.z * mat.col2.y;
		const float local_z = displacement.x * mat.col0.z + displacement.y * mat.col1.z + displacement.z * mat.col2.z;
		mat.col3 = float4( mat.col3.x + local_x, mat.col3.y + local_y, mat.col3.z + local_z, 1.0f );
	}

	inline void Mult( matrix44 &mat, const matrix44 &a, const matrix44 &b ) {
		DBG_ASSERT( ( &mat != &a ) && ( &mat != &b ) );
		mat.col0.x = a.col0.x*b.col0.x + a.col1.x*b.col0.y + a.col2.x*b.col0.z + a.col3.x*b.col0.w;
		mat.col0.y = a.col0.y*b.col0.x + a.col1.y*b.col0.y + a.col2.y*b.col0.z + a.col3.y*b.col0.w;
		mat.col0.z = a.col0.z*b.col0.x + a.col1.z*b.col0.y + a.col2.z*b.col0.z + a.col3.z*b.col0.w;
		mat.col0.w = a.col0.w*b.col0.x + a.col1.w*b.col0.y + a.col2.w*b.col0.z + a.col3.w*b.col0.w;
		mat.col1.x = a.col0.x*b.col1.x + a.col1.x*b.col1.y + a.col2.x*b.col1.z + a.col3.x*b.col1.w;
		mat.col1.y = a.col0.y*b.col1.x + a.col1.y*b.col1.y + a.col2.y*b.col1.z + a.col3.y*b.col1.w;
		mat.col1.z = a.col0.z*b.col1.x + a.col1.z*b.col1.y + a.col2.z*b.col1.z + a.col3.z*b.col1.w;
		mat.col1.w = a.col0.w*b.col1.x + a.col1.w*b.col1.y + a.col2.w*b.col1.z + a.col3.w*b.col1.w;
		mat.col2.x = a.col0.x*b.col2.x + a.col1.x*b.col2.y + a.col2.x*b.col2.z + a.col3.x*b.col2.w;
		mat.col2.y = a.col0.y*b.col2.x + a.col1.y*b.col2.y + a.col2.y*b.col2.z + a.col3.y*b.col2.w;
		mat.col2.z = a.col0.z*b.col2.x + a.col1.z*b.col2.y + a.col2.z*b.col2.z + a.col3.z*b.col2.w;
		mat.col2.w = a.col0.w*b.col2.x + a.col1.w*b.col2.y + a.col2.w*b.col2.z + a.col3.w*b.col2.w;
		mat.col3.x = a.col0.x*b.col3.x + a.col1.x*b.col3.y + a.col2.x*b.col3.z + a.col3.x*b.col3.w;
		mat.col3.y = a.col0.y*b.col3.x + a.col1.y*b.col3.y + a.col2.y*b.col3.z + a.col3.y*b.col3.w;
		mat.col3.z = a.col0.z*b.col3.x + a.col1.z*b.col3.y + a.col2.z*b.col3.z + a.col3.z*b.col3.w;
		mat.col3.w = a.col0.w*b.col3.x + a.col1.w*b.col3.y + a.col2.w*b.col3.z + a.col3.w*b.col3.w;
	}

	inline void Mult( float4 &r, const matrix44 &m, const float4 &v ) {
		DBG_ASSERT( &r != &v );
		r.x = m.col0.x*v.x + m.col1.x*v.y + m.col2.x*v.z + m.col3.x*v.w;
		r.y = m.col0.y*v.x + m.col1.y*v.y + m.col2.y*v.z + m.col3.y*v.w;
		r.z = m.col0.z*v.x + m.col1.z*v.y + m.col2.z*v.z + m.col3.z*v.w;
		r.w = m.col0.w*v.x + m.col1.w*v.y + m.col2.w*v.z + m.col3.w*v.w;
	}

	inline float4 Mult( const matrix44 &m, const float4 &v ) {
		return float4(
			m.col0.x*v.x + m.col1.x*v.y + m.col2.x*v.z + m.col3.x*v.w,
			m.col0.y*v.x + m.col1.y*v.y + m.col2.y*v.z + m.col3.y*v.w,
			m.col0.z*v.x + m.col1.z*v.y + m.col2.z*v.z + m.col3.z*v.w,
			m.col0.w*v.x + m.col1.w*v.y + m.col2.w*v.z + m.col3.w*v.w
		);
	}
	
	inline void Mult( float3 &r, const matrix44 &m, const float3 &v, const float w=1.0f ) {
		DBG_ASSERT( &r != &v );
		r.x = m.col0.x*v.x + m.col1.x*v.y + m.col2.x*v.z + m.col3.x*w;
		r.y = m.col0.y*v.x + m.col1.y*v.y + m.col2.y*v.z + m.col3.y*w;
		r.z = m.col0.z*v.x + m.col1.z*v.y + m.col2.z*v.z + m.col3.z*w;
	}

	inline float3 Mult( const matrix44 &m, const float3 &v, const float w=1.0f ) {
		return float3(
			m.col0.x*v.x + m.col1.x*v.y + m.col2.x*v.z + m.col3.x*w,
			m.col0.y*v.x + m.col1.y*v.y + m.col2.y*v.z + m.col3.y*w,
			m.col0.z*v.x + m.col1.z*v.y + m.col2.z*v.z + m.col3.z*w
		);
	}
	
	inline void Orbit( matrix44 &mat, const matrix44 &target, const float3 &angles, const float distance ) {
		matrix44 rot;
		BuildMatrix( rot, angles, float3(0,0,0) );
		Mult( mat, target, rot );
		TranslateLocal( mat, float3( 0, 0, distance ) );
	}
}


#endif  // __MATH_HPP__
