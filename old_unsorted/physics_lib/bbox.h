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