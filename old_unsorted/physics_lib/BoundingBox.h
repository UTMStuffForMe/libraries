#pragma once
#include "Point3D.h"

class BoundingBox {
  public:
    Point3D min;
    Point3D max;

    bool Intersects(const BoundingBox &other) const;
};

//TODO: TELL CARRIE THAT HER FLOATS ARE CUTTING OFF A LOT OF "TO_RAD" i.e. float * TO_RAD
const double TO_RAD = PI / 180.0;


class CVec3
{
public:
	// Data
	float x, y, z;

	// Ctors
	CVec3( float InX, float InY, float InZ ) : x( InX ), y( InY ), z( InZ )
		{
		}
	CVec3( ) : x(0), y(0), z(0)
		{
		}

	// Operator Overloads
	inline bool operator== (const CVec3& V2) const 
		{
		return (x == V2.x && y == V2.y && z == V2.z);
		}

	inline CVec3 operator+ (const CVec3& V2) const 
		{
		return CVec3( x + V2.x,  y + V2.y,  z + V2.z);
		}
	inline CVec3 operator- (const CVec3& V2) const
		{
		return CVec3( x - V2.x,  y - V2.y,  z - V2.z);
		}
	inline CVec3 operator- ( ) const
		{
		return CVec3(-x, -y, -z);
		}

	inline CVec3 operator/ (float S ) const
		{
		float fInv = 1.0f / S;
		return CVec3 (x * fInv , y * fInv, z * fInv);
		}
	inline CVec3 operator/ (const CVec3& V2) const
		{
		return CVec3 (x / V2.x,  y / V2.y,  z / V2.z);
		}
	inline CVec3 operator* (const CVec3& V2) const
		{
		return CVec3 (x * V2.x,  y * V2.y,  z * V2.z);
		}
	inline CVec3 operator* (float S) const
		{
		return CVec3 (x * S,  y * S,  z * S);
		}

	inline void operator+= ( const CVec3& V2 )
		{
		x += V2.x;
		y += V2.y;
		z += V2.z;
		}
	inline void operator-= ( const CVec3& V2 )
		{
		x -= V2.x;
		y -= V2.y;
		z -= V2.z;
		}

	inline float operator[] ( int i )
		{
		if ( i == 0 ) return x;
		else if ( i == 1 ) return y;
		else return z;
		}

	// Functions
	inline float Dot( const CVec3 &V1 ) const
		{
		return V1.x*x + V1.y*y + V1.z*z;
		}

	inline CVec3 CrossProduct( const CVec3 &V2 ) const
		{
		return CVec3(
			y * V2.z  -  z * V2.y,
			z * V2.x  -  x * V2.z,
			x * V2.y  -  y * V2.x 	);
		}

	// Return vector rotated by the 3x3 portion of matrix m
	// (provided because it's used by bbox.cpp in article 21)
	CVec3 RotByMatrix( const float m[16] ) const
	{
	return CVec3( 
		x*m[0] + y*m[4] + z*m[8],
		x*m[1] + y*m[5] + z*m[9],
		x*m[2] + y*m[6] + z*m[10] );
 	}

	// These require math.h for the sqrtf function
	float Magnitude( ) const
		{
		return sqrtf( x*x + y*y + z*z );
		}

	float Distance( const CVec3 &V1 ) const
		{
		return ( *this - V1 ).Magnitude();	
		}

	inline void Normalize()
		{
		float fMag = ( x*x + y*y + z*z );
		if (fMag == 0) {return;}

		float fMult = 1.0f/sqrtf(fMag);            
		x *= fMult;
		y *= fMult;
		z *= fMult;
		return;
		}
};

class CMatrix 
{
public:
// Data
	float mf[ 16 ];

// Functions
	CMatrix( const int bIdentity = true )
	{
	 if ( bIdentity ) Identity();
	}

	void Identity( )
	{
	mf[ 0] = 1.0f;    mf[ 1] = 0.0f;      mf[ 2] = 0.0f;    mf[ 3] = 0.0f;  
	mf[ 4] = 0.0f;    mf[ 5] = 1.0f;      mf[ 6] = 0.0f;    mf[ 7] = 0.0f;  
	mf[ 8] = 0.0f;    mf[ 9] = 0.0f;      mf[10] = 1.0f;    mf[11] = 0.0f;  
	mf[12] = 0.0f;    mf[13] = 0.0f;      mf[14] = 0.0f;    mf[15] = 1.0f;
	}

	// Concatenate 2 matrices with the * operator
	inline CMatrix operator* (const CMatrix &InM) const
	{
	CMatrix Result( 0 );
	for (int i=0;i<16;i+=4)
		{
		for (int j=0;j<4;j++)
			{
			Result.mf[i + j] = mf[ i + 0] * InM.mf[ 0 + j] + mf[ i + 1] * InM.mf[ 4 + j]
				+ mf[ i + 2] * InM.mf[ 8 + j] + mf[ i + 3] * InM.mf[ 12 + j];
			}
		}
	return Result;
	}

	// Use a matrix to transform a 3D point with the * operator
	inline CVec3 operator* (const CVec3 &Point ) const
	{
	 float x = Point.x*mf[0] + Point.y*mf[4] + Point.z*mf[8]  + mf[12];
	 float y = Point.x*mf[1] + Point.y*mf[5] + Point.z*mf[9]  + mf[13];
	 float z = Point.x*mf[2] + Point.y*mf[6] + Point.z*mf[10] + mf[14]; 
	 return CVec3( x, y, z );
	}

	// Rotate the *this matrix fDegrees counter-clockwise around a single axis( either x, y, or z )
	void Rotate( float fDegrees, int x, int y, int z )
	{
	 CMatrix Temp;
	 if (x == 1) Temp.RotX( -fDegrees );
	 if (y == 1) Temp.RotY( -fDegrees );
	 if (z == 1) Temp.RotZ( -fDegrees );
	 *this = Temp * (*this);
	}

	void Scale( float sx, float sy, float sz )
	{
	 int x;
	 for (x = 0; x <  4; x++) mf[x]*=sx;
	 for (x = 4; x <  8; x++) mf[x]*=sy;
	 for (x = 8; x < 12; x++) mf[x]*=sz;
	}

	void Translate( const CVec3 &Test )
	{
	for (int j=0;j<4;j++)
		{
		mf[12+j] += Test.x * mf[j] + Test.y * mf[4+j] + Test.z * mf[8+j]; 
		}	 
	}
	
	CVec3 GetTranslate( )
	{
		return CVec3( mf[12], mf[13], mf[14] );
	}
	
	// Zero out the translation part of the matrix
	CMatrix RotationOnly( )
	{
	 CMatrix Temp = *this;
	 Temp.mf[12] = 0;
	 Temp.mf[13] = 0;
	 Temp.mf[14] = 0;
	 return Temp;
	}

	// Create a rotation matrix for a counter-clockwise rotation of fDegrees around an arbitrary axis(x, y, z)
	void RotateMatrix( float fDegrees, float x, float y, float z)
	{
	Identity();
	float cosA = cosf(fDegrees*TO_RAD);
	float sinA = sinf(fDegrees*TO_RAD);
	float m = 1.0f - cosA;
	mf[0] = cosA + x*x*m;
	mf[5] = cosA + y*y*m;
	mf[10]= cosA + z*z*m;

	float tmp1 = x*y*m;
	float tmp2 = z*sinA;
	mf[4] = tmp1 + tmp2;
	mf[1] = tmp1 - tmp2;

	tmp1 = x*z*m;
	tmp2 = y*sinA;
	mf[8] = tmp1 - tmp2;
	mf[2] = tmp1 + tmp2;

	tmp1 = y*z*m;
	tmp2 = x*sinA;
	mf[9] = tmp1 + tmp2;
	mf[6] = tmp1 - tmp2;
	}

	// Simple but not robust matrix inversion. (Doesn't work properly if there is a scaling or skewing transformation.)
	inline CMatrix InvertSimple()
	{
	CMatrix R(0);
	R.mf[0]  = mf[0]; 		R.mf[1]  = mf[4];		R.mf[2]  = mf[8];	R.mf[3]  = 0.0f;
	R.mf[4]  = mf[1];		R.mf[5]  = mf[5];		R.mf[6]  = mf[9];	R.mf[7]  = 0.0f;
	R.mf[8]  = mf[2];		R.mf[9]  = mf[6];		R.mf[10] = mf[10];	R.mf[11] = 0.0f;
	R.mf[12] = -(mf[12]*mf[0]) - (mf[13]*mf[1]) - (mf[14]*mf[2]);
	R.mf[13] = -(mf[12]*mf[4]) - (mf[13]*mf[5]) - (mf[14]*mf[6]);
	R.mf[14] = -(mf[12]*mf[8]) - (mf[13]*mf[9]) - (mf[14]*mf[10]);
	R.mf[15] = 1.0f;
	return R;
	}
	
	// Invert for only a rotation, any translation is zeroed out
	CMatrix InvertRot( )
	{
	 CMatrix R( 0 );
	 R.mf[0]  = mf[0]; 		R.mf[1]  = mf[4];		R.mf[2]  = mf[8];	R.mf[3]  = 0.0f;
	 R.mf[4]  = mf[1];		R.mf[5]  = mf[5];		R.mf[6]  = mf[9];	R.mf[7]  = 0.0f;
	 R.mf[8]  = mf[2];		R.mf[9]  = mf[6];		R.mf[10] = mf[10];	R.mf[11] = 0.0f;
	 R.mf[12] = 0;			R.mf[13] = 0;			R.mf[14] = 0;		R.mf[15] = 1.0f;
	 return R;
	}	


private:
	// helpers for Rotate
	void RotX(float angle)
        {  
        mf[5]  = cosf(angle*TO_RAD);
        mf[6]  = sinf(angle*TO_RAD);
        mf[9]  = -sinf(angle*TO_RAD);
        mf[10] = cosf(angle*TO_RAD);
        }
	void RotY(float angle)
        {
        mf[0]  =  cosf(angle*TO_RAD);
        mf[2]  =  -sinf(angle*TO_RAD);
        mf[8]  =  sinf(angle*TO_RAD);
        mf[10] =  cosf(angle*TO_RAD);    
        }
	void RotZ(float angle)
        {
        mf[0] =  cosf(angle*TO_RAD);
        mf[1] =  sinf(angle*TO_RAD);
        mf[4] =  -sinf(angle*TO_RAD);
        mf[5] =  cosf(angle*TO_RAD);
        }
};

//
// Oriented Bounding Box
//
// Stored as a matrix( without scaling ) and Extents( x, y, z )
//
class CBBox
{
public:
	CBBox() {}
	CBBox( const CMatrix &M, const CVec3 &Extent ) 
		{ Set( M, Extent );	}
	// BL = Low values corner point, BH = High values corner point
	CBBox( const CMatrix &M, const CVec3 &BL, const CVec3 &BH ) 
		{ Set( M, BL, BH );	}
	
	void Set( const CMatrix &M, const CVec3 &Extent )
	{
	 m_M = M;
	 m_Extent = Extent;
	}	
	void Set( const CMatrix &M, const CVec3 &BL, const CVec3 &BH )
	{
	 m_M = M;
	 m_M.Translate( (BH + BL) * 0.5f );
	 m_Extent = (BH - BL) / 2.0f;
	}
	
	CVec3 GetSize() 
		{ return m_Extent * 2.0f; }
	CVec3 GetCenterPoint() 
		{ return m_M.GetTranslate(); }		
	void GetInvRot( CVec3 *pvRot );
	
	bool IsPointInBox( const CVec3& P );
	bool IsBoxInBox( CBBox &BBox );
	bool IsSphereInBox( const CVec3& P, float fRadius );
	bool IsLineInBox( const CVec3& L1, const CVec3& L2 );
	bool BoxOutsidePlane( const CVec3 &Norm, const CVec3 &P );
	
	// Data
	CMatrix m_M;
	CVec3 m_Extent;		
};