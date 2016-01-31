////////////////////////////////////////////////////////////////////////
//
//									MATHL.CPP
//
//						General math utilities
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "mathl_.h"
#include <math.h>

// Matrix index
#define	M_11		0
#define	M_12		1
#define	M_13		2
#define	M_14		3
#define	M_21		4
#define	M_22		5
#define	M_23		6
#define	M_24		7
#define	M_31		8
#define	M_32		9
#define	M_33		10
#define	M_34		11
#define	M_41		12
#define	M_42		13
#define	M_43		14
#define	M_44		15

HRESULT mathBinary ( int iOp, const ADTVALUE &vL, const ADTVALUE &vR, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Perform a binary operation on values.
	//
	//	PARAMETERS
	//		-	iOp is the operation to perform
	//		-	vL is the left side
	//		-	vR is the right side
	//		-	v will receive the result
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Dereference values
	if (hr == S_OK && vL.vtype == (VTYPE_VALUE|VTYPE_BYREF) && vL.pval != NULL)
		return mathBinary ( iOp, *(vL.pval), vR, v );
	if (hr == S_OK && vR.vtype == (VTYPE_VALUE|VTYPE_BYREF) && vR.pval != NULL)
		return mathBinary ( iOp, vL, *(vR.pval), v );

	// Setup
	CCLOK   ( adtValue::clear ( v ); )
	CCLTRYE ( adtValue::empty(vL) == false, ERROR_INVALID_STATE );
	CCLTRYE ( adtValue::empty(vR) == false, ERROR_INVALID_STATE );

	// Special case.  If either side is a dictionary, it is assumed operation	
	// involves tuples (vectors).
	if (hr == S_OK && (vL.vtype == VTYPE_UNK || vR.vtype == VTYPE_UNK))
		return mathBinaryV ( iOp, vL, vR, v );

	// Value type of left side
	switch ((int)adtValue::type(vL))
		{
		// String.
		case VTYPE_STR :
			{
			adtString	strR,strV;

			// Currently only 'add' is supported
			CCLTRYE( iOp == MATHOP_ADD, E_NOTIMPL );

			// String version of right side
			CCLTRY ( adtValue::toString ( vR, strR ) );

			// Create appended version
			CCLOK ( strV = vL; )
			CCLTRY( strV.append ( strR ) );

			// Result
			CCLTRY ( adtValue::copy ( strV, v ) );
			}	// VTYPE_STR
			break;

		// Integer
		case VTYPE_I4 :
			switch (vR.vtype)
				{
				case VTYPE_I4  :
					v.vtype	= VTYPE_I4;
					v.vint	=	(iOp == MATHOP_ADD) ? vL.vint+vR.vint :
									(iOp == MATHOP_SUB) ? vL.vint-vR.vint :
									(iOp == MATHOP_MUL) ? vL.vint*vR.vint :
									(iOp == MATHOP_DIV && vR.vint != 0) ? vL.vint/vR.vint : 
									(iOp == MATHOP_MOD) ? (vL.vint % vR.vint) : 
									(iOp == MATHOP_AND) ? (vL.vint & vR.vint) : 
									(iOp == MATHOP_MIN) ? ((vL.vint < vR.vint) ? vL.vint : vR.vint) :
									(iOp == MATHOP_MAX) ? ((vL.vint > vR.vint) ? vL.vint : vR.vint) :
									vL.vint;
					break;
				case VTYPE_I8 :
					v.vtype	= VTYPE_I8;
					v.vlong	=	(iOp == MATHOP_ADD) ? vL.vint+vR.vlong :
									(iOp == MATHOP_SUB) ? vL.vint-vR.vlong :
									(iOp == MATHOP_MUL) ? vL.vint*vR.vlong :
									(iOp == MATHOP_DIV && vR.vlong != 0) ? vL.vint/vR.vlong :
									(iOp == MATHOP_MOD) ? (vL.vint % vR.vlong) : 
									(iOp == MATHOP_AND) ? (vL.vint & vR.vlong) : 
									(iOp == MATHOP_MIN) ? ((vL.vint < vR.vlong) ? vL.vint : vR.vlong) :
									(iOp == MATHOP_MAX) ? ((vL.vint > vR.vlong) ? vL.vint : vR.vlong) :
									vL.vint;
					break;
				case VTYPE_R4  :
					v.vtype	= VTYPE_R4;
					v.vflt	=	(iOp == MATHOP_ADD) ? vL.vint+vR.vflt :
									(iOp == MATHOP_SUB) ? vL.vint-vR.vflt :
									(iOp == MATHOP_MUL) ? vL.vint*vR.vflt :
									(iOp == MATHOP_DIV && vR.vflt != 0) ? vL.vint/vR.vflt : 
									(iOp == MATHOP_MIN) ? ((vL.vint < vR.vflt) ? vL.vint : vR.vflt) :
									(iOp == MATHOP_MAX) ? ((vL.vint > vR.vflt) ? vL.vint : vR.vflt) :
									vL.vint;
					break;
				case VTYPE_R8  :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vint+vR.vdbl :
									(iOp == MATHOP_SUB) ? vL.vint-vR.vdbl :
									(iOp == MATHOP_MUL) ? vL.vint*vR.vdbl :
									(iOp == MATHOP_DIV && vR.vdbl != 0) ? vL.vint/vR.vdbl : 
									(iOp == MATHOP_MIN) ? ((vL.vint < vR.vdbl) ? vL.vint : vR.vdbl) :
									(iOp == MATHOP_MAX) ? ((vL.vint > vR.vdbl) ? vL.vint : vR.vdbl) :
									vL.vint;
					break;
				case VTYPE_DATE  :
					v.vtype	= VTYPE_DATE;
					v.vdate	=	(iOp == MATHOP_ADD) ? vL.vint+vR.vdate :
									(iOp == MATHOP_SUB) ? vL.vint-vR.vdate :
									(iOp == MATHOP_MUL) ? vL.vint*vR.vdate :
									(iOp == MATHOP_DIV && vR.vdate != 0) ? vL.vint/vR.vdate : 
									(iOp == MATHOP_MIN) ? ((vL.vint < vR.vdate) ? vL.vint : vR.vdate) :
									(iOp == MATHOP_MAX) ? ((vL.vint > vR.vdate) ? vL.vint : vR.vdate) :
									vL.vint;
					break;
				case VTYPE_STR :
				case VTYPE_STR | VTYPE_BYREF:
					{
					adtString	strV;

					// Currently only 'add' is supported
					CCLTRYE( iOp == MATHOP_ADD, E_NOTIMPL );

					// String version of left side
					CCLTRY ( adtValue::toString ( vL, strV ) );

					// Create appended version
					CCLTRY( strV.append ( vR.pstr ) );

					// Result
					CCLTRY ( adtValue::copy ( strV, v ) );
					}	// VTYPE_STR
					break;
				default :
					hr = E_NOTIMPL;
				}	// switch
			break;

		// Long
		case VTYPE_I8 :
			switch (vR.vtype)
				{
				case VTYPE_I4  :
					v.vtype	= VTYPE_I8;
					v.vlong	=	(iOp == MATHOP_ADD) ? vL.vlong+vR.vint :
									(iOp == MATHOP_SUB) ? vL.vlong-vR.vint :
									(iOp == MATHOP_MUL) ? vL.vlong*vR.vint :
									(iOp == MATHOP_DIV && vR.vint != 0) ? vL.vlong/vR.vint :
									(iOp == MATHOP_MOD) ? (vL.vlong % vR.vint) : 
									(iOp == MATHOP_AND) ? (vL.vlong & vR.vint) : 
									(iOp == MATHOP_MIN) ? ((vL.vlong < vR.vint) ? vL.vlong : vR.vint) :
									(iOp == MATHOP_MAX) ? ((vL.vlong > vR.vint) ? vL.vlong : vR.vint) :
									vL.vlong;
					break;
				case VTYPE_I8 :
					v.vtype	= VTYPE_I8;
					v.vlong	=	(iOp == MATHOP_ADD) ? vL.vlong+vR.vlong :
									(iOp == MATHOP_SUB) ? vL.vlong-vR.vlong :
									(iOp == MATHOP_MUL) ? vL.vlong*vR.vlong :
									(iOp == MATHOP_DIV && vR.vlong != 0) ? vL.vlong/vR.vlong :
									(iOp == MATHOP_MOD) ? (vL.vlong % vR.vlong) : 
									(iOp == MATHOP_AND) ? (vL.vlong & vR.vlong) : 
									(iOp == MATHOP_MIN) ? ((vL.vlong < vR.vlong) ? vL.vlong : vR.vlong) :
									(iOp == MATHOP_MAX) ? ((vL.vlong > vR.vlong) ? vL.vlong : vR.vlong) :
									vL.vlong;
					break;
				case VTYPE_R4  :
					v.vtype	= VTYPE_R4;
					v.vflt	=	(iOp == MATHOP_ADD) ? vL.vlong+vR.vflt :
									(iOp == MATHOP_SUB) ? vL.vlong-vR.vflt :
									(iOp == MATHOP_MUL) ? vL.vlong*vR.vflt :
									(iOp == MATHOP_DIV && vR.vflt != 0) ? vL.vlong/vR.vflt : 
									(iOp == MATHOP_MIN) ? ((vL.vlong < vR.vflt) ? vL.vlong : vR.vflt) :
									(iOp == MATHOP_MAX) ? ((vL.vlong > vR.vflt) ? vL.vlong : vR.vflt) :
									vL.vlong;
					break;
				case VTYPE_R8  :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vlong+vR.vdbl :
									(iOp == MATHOP_SUB) ? vL.vlong-vR.vdbl :
									(iOp == MATHOP_MUL) ? vL.vlong*vR.vdbl :
									(iOp == MATHOP_DIV && vR.vdbl != 0) ? vL.vlong/vR.vdbl : 
									(iOp == MATHOP_MIN) ? ((vL.vlong < vR.vdbl) ? vL.vlong : vR.vdbl) :
									(iOp == MATHOP_MAX) ? ((vL.vlong > vR.vdbl) ? vL.vlong : vR.vdbl) :
									vL.vlong;
					break;
				case VTYPE_DATE  :
					v.vtype	= VTYPE_DATE;
					v.vdate	=	(iOp == MATHOP_ADD) ? vL.vlong+vR.vdate :
									(iOp == MATHOP_SUB) ? vL.vlong-vR.vdate :
									(iOp == MATHOP_MUL) ? vL.vlong*vR.vdate :
									(iOp == MATHOP_DIV && vR.vdate != 0) ? vL.vlong/vR.vdate : 
									(iOp == MATHOP_MIN) ? ((vL.vlong < vR.vdate) ? vL.vlong : vR.vdate) :
									(iOp == MATHOP_MAX) ? ((vL.vlong > vR.vdate) ? vL.vlong : vR.vdate) :
									vL.vlong;
					break;
				default :
					hr = E_NOTIMPL;
				}	// switch
			break;

		// Float
		case VTYPE_R4 :
			switch (vR.vtype)
				{
				case VTYPE_I4  :
					v.vtype	= VTYPE_R4;
					v.vflt	=	(iOp == MATHOP_ADD) ? vL.vflt+vR.vint :
									(iOp == MATHOP_SUB) ? vL.vflt-vR.vint :
									(iOp == MATHOP_MUL) ? vL.vflt*vR.vint :
									(iOp == MATHOP_DIV && vR.vint != 0) ? vL.vflt/vR.vint : 
									(iOp == MATHOP_MIN) ? ((vL.vflt < vR.vint) ? vL.vflt : vR.vint) :
									(iOp == MATHOP_MAX) ? ((vL.vflt > vR.vint) ? vL.vflt : vR.vint) :
									vL.vflt;
					break;
				case VTYPE_I8 :
					v.vtype	= VTYPE_R4;
					v.vflt	=	(iOp == MATHOP_ADD) ? vL.vflt+vR.vlong :
									(iOp == MATHOP_SUB) ? vL.vflt-vR.vlong :
									(iOp == MATHOP_MUL) ? vL.vflt*vR.vlong :
									(iOp == MATHOP_DIV && vR.vlong != 0) ? vL.vflt/vR.vlong : 
									(iOp == MATHOP_MIN) ? ((vL.vflt < vR.vlong) ? vL.vflt : vR.vlong) :
									(iOp == MATHOP_MAX) ? ((vL.vflt > vR.vlong) ? vL.vflt : vR.vlong) :
									vL.vflt;
					break;
				case VTYPE_R4  :
					v.vtype	= VTYPE_R4;
					v.vflt	=	(iOp == MATHOP_ADD) ? vL.vflt+vR.vflt :
									(iOp == MATHOP_SUB) ? vL.vflt-vR.vflt :
									(iOp == MATHOP_MUL) ? vL.vflt*vR.vflt :
									(iOp == MATHOP_DIV && vR.vflt != 0) ? vL.vflt/vR.vflt : 
									(iOp == MATHOP_MIN) ? ((vL.vflt < vR.vflt) ? vL.vflt : vR.vflt) :
									(iOp == MATHOP_MAX) ? ((vL.vflt > vR.vflt) ? vL.vflt : vR.vflt) :
									vL.vflt;
					break;
				case VTYPE_R8  :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vflt+vR.vdbl :
									(iOp == MATHOP_SUB) ? vL.vflt-vR.vdbl :
									(iOp == MATHOP_MUL) ? vL.vflt*vR.vdbl :
									(iOp == MATHOP_DIV && vR.vdbl != 0) ? vL.vflt/vR.vdbl : 
									(iOp == MATHOP_MIN) ? ((vL.vflt < vR.vdbl) ? vL.vflt : vR.vdbl) :
									(iOp == MATHOP_MAX) ? ((vL.vflt > vR.vdbl) ? vL.vflt : vR.vdbl) :
									vL.vflt;
					break;
				case VTYPE_DATE  :
					v.vtype	= VTYPE_DATE;
					v.vdate	=	(iOp == MATHOP_ADD) ? vL.vflt+vR.vdate :
									(iOp == MATHOP_SUB) ? vL.vflt-vR.vdate :
									(iOp == MATHOP_MUL) ? vL.vflt*vR.vdate :
									(iOp == MATHOP_DIV && vR.vdate != 0) ? vL.vflt/vR.vdate : 
									(iOp == MATHOP_MIN) ? ((vL.vflt < vR.vdate) ? vL.vflt : vR.vdate) :
									(iOp == MATHOP_MAX) ? ((vL.vflt > vR.vdate) ? vL.vflt : vR.vdate) :
									vL.vflt;
					break;
				default :
					hr = E_NOTIMPL;
				}	// switch
			break;

		// Double
		case VTYPE_R8 :
			switch (vR.vtype)
				{
				case VTYPE_I4  :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vdbl+vR.vint :
									(iOp == MATHOP_SUB) ? vL.vdbl-vR.vint :
									(iOp == MATHOP_MUL) ? vL.vdbl*vR.vint :
									(iOp == MATHOP_DIV && vR.vint != 0) ? vL.vdbl/vR.vint : 
									(iOp == MATHOP_MIN) ? ((vL.vdbl < vR.vint) ? vL.vdbl : vR.vint) :
									(iOp == MATHOP_MAX) ? ((vL.vdbl > vR.vint) ? vL.vdbl : vR.vint) :
									vL.vdbl;
					break;
				case VTYPE_I8 :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vdbl+vR.vlong :
									(iOp == MATHOP_SUB) ? vL.vdbl-vR.vlong :
									(iOp == MATHOP_MUL) ? vL.vdbl*vR.vlong :
									(iOp == MATHOP_DIV && vR.vlong != 0) ? vL.vdbl/vR.vlong : 
									(iOp == MATHOP_MIN) ? ((vL.vdbl < vR.vlong) ? vL.vdbl : vR.vlong) :
									(iOp == MATHOP_MAX) ? ((vL.vdbl > vR.vlong) ? vL.vdbl : vR.vlong) :
									vL.vdbl;
					break;
				case VTYPE_R4  :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vdbl+vR.vflt :
									(iOp == MATHOP_SUB) ? vL.vdbl-vR.vflt :
									(iOp == MATHOP_MUL) ? vL.vdbl*vR.vflt :
									(iOp == MATHOP_DIV && vR.vflt != 0) ? vL.vdbl/vR.vflt : 
									(iOp == MATHOP_MIN) ? ((vL.vdbl < vR.vflt) ? vL.vdbl : vR.vflt) :
									(iOp == MATHOP_MAX) ? ((vL.vdbl > vR.vflt) ? vL.vdbl : vR.vflt) :
									vL.vdbl;
					break;
				case VTYPE_R8  :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vdbl+vR.vdbl :
									(iOp == MATHOP_SUB) ? vL.vdbl-vR.vdbl :
									(iOp == MATHOP_MUL) ? vL.vdbl*vR.vdbl :
									(iOp == MATHOP_DIV && vR.vdbl != 0) ? vL.vdbl/vR.vdbl : 
									(iOp == MATHOP_MIN) ? ((vL.vdbl < vR.vdbl) ? vL.vdbl : vR.vdbl) :
									(iOp == MATHOP_MAX) ? ((vL.vdbl > vR.vdbl) ? vL.vdbl : vR.vdbl) :
									vL.vdbl;
					break;
				case VTYPE_DATE  :
					v.vtype	= VTYPE_DATE;
					v.vdate	=	(iOp == MATHOP_ADD) ? vL.vdbl+vR.vdate :
									(iOp == MATHOP_SUB) ? vL.vdbl-vR.vdate :
									(iOp == MATHOP_MUL) ? vL.vdbl*vR.vdate :
									(iOp == MATHOP_DIV && vR.vdate != 0) ? vL.vdbl/vR.vdate : 
									(iOp == MATHOP_MIN) ? ((vL.vdbl < vR.vdate) ? vL.vdbl : vR.vdate) :
									(iOp == MATHOP_MAX) ? ((vL.vdbl > vR.vdate) ? vL.vdbl : vR.vdate) :
									vL.vdbl;
					break;
				default :
					hr = E_NOTIMPL;
				}	// switch
			break;

		// Date
		case VTYPE_DATE :
			switch (vR.vtype)
				{
				case VTYPE_I4  :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vdate+vR.vint :
									(iOp == MATHOP_SUB) ? vL.vdate-vR.vint :
									(iOp == MATHOP_MUL) ? vL.vdate*vR.vint :
									(iOp == MATHOP_DIV && vR.vint != 0) ? vL.vdate/vR.vint : 
									(iOp == MATHOP_MIN) ? ((vL.vdate < vR.vint) ? vL.vdate : vR.vint) :
									(iOp == MATHOP_MAX) ? ((vL.vdate > vR.vint) ? vL.vdate : vR.vint) :
									vL.vdate;
					break;
				case VTYPE_I8 :
					v.vtype	= VTYPE_DATE;
					v.vdate	=	(iOp == MATHOP_ADD) ? vL.vdate+vR.vlong :
									(iOp == MATHOP_SUB) ? vL.vdate-vR.vlong :
									(iOp == MATHOP_MUL) ? vL.vdate*vR.vlong :
									(iOp == MATHOP_DIV && vR.vlong != 0) ? vL.vdate/vR.vlong : 
									(iOp == MATHOP_MIN) ? ((vL.vdate < vR.vlong) ? vL.vdate : vR.vlong) :
									(iOp == MATHOP_MAX) ? ((vL.vdate > vR.vlong) ? vL.vdate : vR.vlong) :
									vL.vdate;
					break;
				case VTYPE_R4  :
					v.vtype	= VTYPE_DATE;
					v.vdate	=	(iOp == MATHOP_ADD) ? vL.vdate+vR.vflt :
									(iOp == MATHOP_SUB) ? vL.vdate-vR.vflt :
									(iOp == MATHOP_MUL) ? vL.vdate*vR.vflt :
									(iOp == MATHOP_DIV && vR.vflt != 0) ? vL.vdate/vR.vflt : 
									(iOp == MATHOP_MIN) ? ((vL.vdate < vR.vflt) ? vL.vdate : vR.vflt) :
									(iOp == MATHOP_MAX) ? ((vL.vdate > vR.vflt) ? vL.vdate : vR.vflt) :
									vL.vdate;
					break;
				case VTYPE_R8  :
					v.vtype	= VTYPE_R8;
					v.vdbl	=	(iOp == MATHOP_ADD) ? vL.vdate+vR.vdbl :
									(iOp == MATHOP_SUB) ? vL.vdate-vR.vdbl :
									(iOp == MATHOP_MUL) ? vL.vdate*vR.vdbl :
									(iOp == MATHOP_DIV && vR.vdbl != 0) ? vL.vdate/vR.vdbl : 
									(iOp == MATHOP_MIN) ? ((vL.vdate < vR.vdbl) ? vL.vdate : vR.vdbl) :
									(iOp == MATHOP_MAX) ? ((vL.vdate > vR.vdbl) ? vL.vdate : vR.vdbl) :
									vL.vdate;
					break;
				case VTYPE_DATE  :
					v.vtype	= VTYPE_DATE;
					v.vdate	=	(iOp == MATHOP_ADD) ? vL.vdate+vR.vdate :
									(iOp == MATHOP_SUB) ? vL.vdate-vR.vdate :
									(iOp == MATHOP_MUL) ? vL.vdate*vR.vdate :
									(iOp == MATHOP_DIV && vR.vdate != 0) ? vL.vdate/vR.vdate :	
									(iOp == MATHOP_MIN) ? ((vL.vdate < vR.vdate) ? vL.vdate : vR.vdate) :
									(iOp == MATHOP_MAX) ? ((vL.vdate > vR.vdate) ? vL.vdate : vR.vdate) :
									vL.vdate;
					break;

				// Not implemented
				default :
					hr = E_NOTIMPL;
				}	// switch
			break;

		// Not implemented
		default :
			hr = E_NOTIMPL;
		}	// switch

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"mathBinary:hr 0x%x:iOp %d\r\n", hr, iOp );

	return hr;
	}	// mathBinary

HRESULT mathBinaryV ( int iOp, const ADTVALUE &vL, const ADTVALUE &vR, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Perform a binary (vector) operation on values.
	//
	//	PARAMETERS
	//		-	iOp is the operation to perform
	//		-	vL is the left side
	//		-	vR is the right side
	//		-	v will receive the result
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pDctL	= NULL;
	IDictionary	*pDctR	= NULL;
	adtDouble	dblL,dblR;

	// Dereference values
	if (hr == S_OK && vL.vtype == (VTYPE_VALUE|VTYPE_BYREF) && vL.pval != NULL)
		return mathBinaryV ( iOp, *(vL.pval), vR, v );
	if (hr == S_OK && vR.vtype == (VTYPE_VALUE|VTYPE_BYREF) && vR.pval != NULL)
		return mathBinaryV ( iOp, vL, *(vR.pval), v );

	// Each side will either be treated as a vector or a scalar (double)
	if (hr == S_OK && vL.vtype == VTYPE_UNK)
		{
		CCLTRY(_QISAFE(vL.punk,IID_IDictionary,&pDctL));
		}	// if
	else
		dblL = adtDouble(vL);
	if (hr == S_OK && vR.vtype == VTYPE_UNK)
		{
		CCLTRY(_QISAFE(vR.punk,IID_IDictionary,&pDctR));
		}	// if
	else
		dblR = adtDouble(vR);

	// Vector/vector
	if (hr == S_OK && pDctL != NULL && pDctR != NULL)
		{
		// Dot product
		if (iOp == MATHOP_DOT)
			{
			IIt		*pItL	= NULL;
			IIt		*pItR	= NULL;
			double	dDot	= 0.0;
			U32		szL,szR;
			adtValue	vL,vR;

			// Dot product requires both vectors to be the same size
			CCLTRY ( pDctL->size ( &szL ) );
			CCLTRY ( pDctR->size ( &szR ) );
			CCLTRYE( szL == szR, E_INVALIDARG );

			// Iterate keys in both dictionaries
			CCLTRY ( pDctL->iterate ( &pItL ) );
			CCLTRY ( pDctR->iterate ( &pItR ) );

			// Peform dot product
			while (	hr == S_OK && 
						pItL->read ( vL ) == S_OK &&
						pItR->read ( vR ) == S_OK )
				{
				// Compute
				dDot += adtDouble(vL) * adtDouble(vR);

				// Next pair
				pItL->next();
				pItR->next();
				}	// while

			// Clean up
			_RELEASE(pItR);
			_RELEASE(pItL);

			// Result
			CCLTRY ( adtValue::copy ( adtDouble(dDot), v ) );
			}	// if

		// Cross product
		else if (iOp == MATHOP_CROSS)
			{
			IDictionary	*pDctRes	= NULL;
			IIt			*pIt		= NULL;
			U32			szL,szR;
			double		dL[3],dR[3];
			adtValue		vK[3],vV;

			// Create a dictionary for the results
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctRes ) );

			// Cross product requires both vectors to be 3-vectors
			CCLTRY ( pDctL->size ( &szL ) );
			CCLTRY ( pDctR->size ( &szR ) );
			CCLTRYE( (szL == 3) && (szR == 3), E_INVALIDARG );

			// Keys
			CCLTRY ( pDctL->keys		( &pIt ) );
			for (int i = 0;hr == S_OK && i < 3;++i)
				{
				CCLTRY ( pIt->read ( vK[i] ) );
				CCLOK ( pIt->next(); )
				}	// for

			// Values
			for (int i = 0;hr == S_OK && i < 3;++i)
				{
				// Left
				CCLTRY ( pDctL->load ( vK[i], vV ) );
				CCLOK  ( dL[i] = adtDouble(vV); )

				// Right
				CCLTRY ( pDctR->load ( vK[i], vV ) );
				CCLOK  ( dR[i] = adtDouble(vV); )
				}	// for

			// Cross product
			CCLTRY ( pDctRes->store ( vK[0], adtDouble ( dL[1]*dR[2] - dL[2]*dR[1] ) ) );
			CCLTRY ( pDctRes->store ( vK[1], adtDouble ( dL[2]*dR[0] - dL[0]*dR[2] ) ) );
			CCLTRY ( pDctRes->store ( vK[2], adtDouble ( dL[0]*dR[1] - dL[1]*dR[0] ) ) );

			// Result
			CCLTRY ( adtValue::copy ( adtIUnknown(pDctRes), v ) );

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pDctRes);
			}	// else if

		// Basic operations
		else if (iOp == MATHOP_ADD || iOp == MATHOP_SUB)
			{
			IDictionary	*pDctRes	= NULL;
			IIt			*pItL	= NULL;
			IIt			*pItR	= NULL;
			U32			szL,szR;
			adtValue		vKL,vKR,vVL,vVR;

			// Create a dictionary for the results
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctRes ) );

			// Operation product requires both vectors to be the same size
			CCLTRY ( pDctL->size ( &szL ) );
			CCLTRY ( pDctR->size ( &szR ) );
			CCLTRYE( szL == szR, E_INVALIDARG );

			// Iterate keys in both dictionaries
			CCLTRY ( pDctL->keys ( &pItL ) );
			CCLTRY ( pDctR->keys ( &pItR ) );

			// Apply operation
			while (	hr == S_OK && 
						pItL->read ( vKL ) == S_OK &&
						pItR->read ( vKR ) == S_OK )
				{
				// Load the values for keys
				CCLTRY ( pDctL->load ( vKL, vVL ) );
				CCLTRY ( pDctR->load ( vKR, vVR ) );

				// Operation
				double dRes = 0.0;
				switch ( iOp )
					{
					case MATHOP_ADD :
						dRes = adtDouble(vVL) + adtDouble(vVR);
						break;
					case MATHOP_SUB :
						dRes = adtDouble(vVL) - adtDouble(vVR);
						break;
					default :
						dbgprintf ( L"mathBinaryV::Unsupported vector/vector operation %d\r\n", iOp );
						hr = E_NOTIMPL;
					}	// switch

				// Store result under the same key
				CCLTRY ( pDctRes->store ( vKL, adtDouble(dRes) ) );

				// Next values
				pItL->next();
				pItR->next();
				}	// while

			// Result
			CCLTRY ( adtValue::copy ( adtIUnknown(pDctRes), v ) );

			// Clean up
			_RELEASE(pItR);
			_RELEASE(pItL);
			_RELEASE(pDctRes);
			}	// else if

		else
			{
			hr = E_NOTIMPL;
			dbgprintf ( L"mathBinaryV::Unsupported dual vector operation %d\r\n", iOp );
			}	// else
 
		}	// if

	// Scalar/vector operation
	else if (hr == S_OK && 
			(	(pDctL != NULL && pDctR == NULL) ||
				(pDctL == NULL && pDctR != NULL) ) )
		{
		// Select the vector and scalar
		adtDouble	dScl		= (pDctL != NULL) ? dblR : dblL;
		IDictionary	*pDctV	= (pDctL != NULL) ? pDctL : pDctR;
		IDictionary	*pDctRes	= NULL;
		IIt			*pIt		= NULL;
		adtValue		vK,vV;
		_ADDREF(pDctV);

		// Create a dictionary for the results
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctRes ) );

		// Iterate tuple
		CCLTRY ( pDctV->keys ( &pIt ) );

		// Apply operation
		while (hr == S_OK && pIt->read ( vK ) == S_OK)
			{
			// Load the value for key
			CCLTRY ( pDctV->load ( vK, vV ) );

			// Operation
			double dRes = 0.0;
			switch ( iOp )
				{
				case MATHOP_MUL :
					dRes = adtDouble(vV) * dScl;
					break;
				case MATHOP_DIV :
					if (dScl != 0.0)
						dRes = adtDouble(vV) / dScl;
					else
						dRes = 0;
					break;
				default :
					dbgprintf ( L"mathBinaryV::Unsupported vector/scalar operation %d\r\n", iOp );
					hr = E_NOTIMPL;
				}	// switch

			// Store result under the same key
			CCLTRY ( pDctRes->store ( vK, adtDouble(dRes) ) );

			// Next entry in tuple
			pIt->next();
			}	// while

		// Result
		CCLTRY ( adtValue::copy ( adtIUnknown(pDctRes), v ) );

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pDctRes);
		_RELEASE(pDctV);
		}	// if

	// Unsupported combo (scalar/scalar handled by other function)
	else
		{
		dbgprintf ( L"mathBinaryV::Unsupported vector operation %d\r\n", iOp );
		hr = E_NOTIMPL;
		}	// else

	// Clean up
	_RELEASE(pDctL);
	_RELEASE(pDctR);

	return hr;	
	}	// mathBinaryV

HRESULT mathInv ( double *dA, double *dC )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Invert 4x4 matrix
	//
	//	PARAMETERS
	//		-	dA is the source matrix
	//		-	dC will receive the result
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	double	det;

	// 0	= _11	1	= _12	2	= _13	3	= _14
	// 4	= _21	5	= _22	6	= _23	7	= _24
	// 8	= _31	9	= _32	10 = _33 11 = _34
	// 12 = _41 13 = _42 14 = _43 15 = _44

	// Compute determinate
	det =		dA[M_11]*dA[M_22]*dA[M_33]*dA[M_44] 
			+	dA[M_11]*dA[M_23]*dA[M_34]*dA[M_42] 
			+	dA[M_11]*dA[M_24]*dA[M_32]*dA[M_43] 

			+ dA[M_12]*dA[M_21]*dA[M_34]*dA[M_43] 
			+ dA[M_12]*dA[M_23]*dA[M_31]*dA[M_44] 
			+ dA[M_12]*dA[M_24]*dA[M_33]*dA[M_41] 

			+ dA[M_13]*dA[M_21]*dA[M_32]*dA[M_44] 
			+ dA[M_13]*dA[M_22]*dA[M_34]*dA[M_41] 
			+ dA[M_13]*dA[M_24]*dA[M_31]*dA[M_42] 

			+ dA[M_14]*dA[M_21]*dA[M_33]*dA[M_42] 
			+ dA[M_14]*dA[M_22]*dA[M_31]*dA[M_43] 
			+ dA[M_14]*dA[M_23]*dA[M_32]*dA[M_41] 

			- dA[M_11]*dA[M_22]*dA[M_34]*dA[M_43] 
			- dA[M_11]*dA[M_23]*dA[M_32]*dA[M_44] 
			- dA[M_11]*dA[M_24]*dA[M_33]*dA[M_42] 

			- dA[M_12]*dA[M_21]*dA[M_33]*dA[M_44] 
			- dA[M_12]*dA[M_23]*dA[M_34]*dA[M_41] 
			- dA[M_12]*dA[M_24]*dA[M_31]*dA[M_43] 

			- dA[M_13]*dA[M_21]*dA[M_34]*dA[M_42] 
			- dA[M_13]*dA[M_22]*dA[M_31]*dA[M_44] 
			- dA[M_13]*dA[M_24]*dA[M_32]*dA[M_41]

			- dA[M_14]*dA[M_21]*dA[M_32]*dA[M_43] 
			- dA[M_14]*dA[M_22]*dA[M_33]*dA[M_41] 
			- dA[M_14]*dA[M_23]*dA[M_31]*dA[M_42];

	// If determinate is zero, non-invertible
	if (det == 0)
		return S_FALSE;

	// Compute inverse

	// Row 1
	dC[M_11] =	(	dA[M_22]*dA[M_33]*dA[M_44] + dA[M_23]*dA[M_34]*dA[M_42] + dA[M_24]*dA[M_32]*dA[M_43]
					-	dA[M_22]*dA[M_34]*dA[M_43] - dA[M_23]*dA[M_32]*dA[M_44] - dA[M_24]*dA[M_33]*dA[M_42] ) / det;
	dC[M_12] =	(	dA[M_12]*dA[M_34]*dA[M_43] + dA[M_13]*dA[M_32]*dA[M_44] + dA[M_14]*dA[M_33]*dA[M_42]
					-	dA[M_12]*dA[M_33]*dA[M_44] - dA[M_13]*dA[M_34]*dA[M_42] - dA[M_14]*dA[M_32]*dA[M_43] ) / det;
	dC[M_13] =	(	dA[M_12]*dA[M_23]*dA[M_44] + dA[M_13]*dA[M_24]*dA[M_42] + dA[M_14]*dA[M_22]*dA[M_43]
					-	dA[M_12]*dA[M_24]*dA[M_43] - dA[M_13]*dA[M_22]*dA[M_44] - dA[M_14]*dA[M_23]*dA[M_42] ) / det;
	dC[M_14] =	(	dA[M_12]*dA[M_24]*dA[M_33] + dA[M_13]*dA[M_22]*dA[M_34] + dA[M_14]*dA[M_23]*dA[M_32]
					-	dA[M_12]*dA[M_23]*dA[M_34] - dA[M_13]*dA[M_24]*dA[M_32] - dA[M_14]*dA[M_22]*dA[M_33] ) / det;

	// Row 2
	dC[M_21] =	(	dA[M_21]*dA[M_34]*dA[M_43] + dA[M_23]*dA[M_31]*dA[M_44] + dA[M_24]*dA[M_33]*dA[M_41]
					-	dA[M_21]*dA[M_33]*dA[M_44] - dA[M_23]*dA[M_34]*dA[M_41] - dA[M_24]*dA[M_31]*dA[M_43] ) / det;
	dC[M_22] =	(	dA[M_11]*dA[M_33]*dA[M_44] + dA[M_13]*dA[M_34]*dA[M_41] + dA[M_14]*dA[M_31]*dA[M_43]
					-	dA[M_11]*dA[M_34]*dA[M_43] - dA[M_13]*dA[M_31]*dA[M_44] - dA[M_14]*dA[M_33]*dA[M_41] ) / det;
	dC[M_23] =	(	dA[M_11]*dA[M_24]*dA[M_43] + dA[M_13]*dA[M_21]*dA[M_44] + dA[M_14]*dA[M_23]*dA[M_41]
					-	dA[M_11]*dA[M_23]*dA[M_44] - dA[M_13]*dA[M_24]*dA[M_41] - dA[M_14]*dA[M_21]*dA[M_43] ) / det;
	dC[M_24] =	(	dA[M_11]*dA[M_23]*dA[M_34] + dA[M_13]*dA[M_24]*dA[M_31] + dA[M_14]*dA[M_21]*dA[M_33]
					-	dA[M_11]*dA[M_24]*dA[M_33] - dA[M_13]*dA[M_21]*dA[M_34] - dA[M_14]*dA[M_23]*dA[M_31] ) / det;

	// Row 3
	dC[M_31] =	(	dA[M_21]*dA[M_32]*dA[M_44] + dA[M_22]*dA[M_34]*dA[M_41] + dA[M_24]*dA[M_31]*dA[M_42]
					-	dA[M_21]*dA[M_34]*dA[M_42] - dA[M_22]*dA[M_31]*dA[M_44] - dA[M_24]*dA[M_32]*dA[M_41] ) / det;
	dC[M_32] =	(	dA[M_11]*dA[M_34]*dA[M_42] + dA[M_12]*dA[M_31]*dA[M_44] + dA[M_14]*dA[M_32]*dA[M_41]
					-	dA[M_11]*dA[M_32]*dA[M_44] - dA[M_12]*dA[M_34]*dA[M_41] - dA[M_14]*dA[M_31]*dA[M_42] ) / det;
	dC[M_33] =	(	dA[M_11]*dA[M_22]*dA[M_44] + dA[M_12]*dA[M_24]*dA[M_41] + dA[M_14]*dA[M_21]*dA[M_42]
					-	dA[M_11]*dA[M_24]*dA[M_42] - dA[M_12]*dA[M_21]*dA[M_44] - dA[M_14]*dA[M_22]*dA[M_41] ) / det;
	dC[M_34] =	(	dA[M_11]*dA[M_24]*dA[M_32] + dA[M_12]*dA[M_21]*dA[M_34] + dA[M_14]*dA[M_22]*dA[M_31]
					-	dA[M_11]*dA[M_22]*dA[M_34] - dA[M_12]*dA[M_24]*dA[M_31] - dA[M_14]*dA[M_21]*dA[M_32] ) / det;

	// Row 4
	dC[M_41] =	(	dA[M_21]*dA[M_33]*dA[M_42] + dA[M_22]*dA[M_31]*dA[M_43] + dA[M_23]*dA[M_32]*dA[M_41]
					-	dA[M_21]*dA[M_32]*dA[M_43] - dA[M_22]*dA[M_33]*dA[M_41] - dA[M_23]*dA[M_31]*dA[M_42] ) / det;
	dC[M_42] =	(	dA[M_11]*dA[M_32]*dA[M_43] + dA[M_12]*dA[M_33]*dA[M_41] + dA[M_13]*dA[M_31]*dA[M_42]
					-	dA[M_11]*dA[M_33]*dA[M_42] - dA[M_12]*dA[M_31]*dA[M_43] - dA[M_13]*dA[M_32]*dA[M_41] ) / det;
	dC[M_43] =	(	dA[M_11]*dA[M_23]*dA[M_42] + dA[M_12]*dA[M_21]*dA[M_43] + dA[M_13]*dA[M_22]*dA[M_41]
					-	dA[M_11]*dA[M_22]*dA[M_43] - dA[M_12]*dA[M_23]*dA[M_41] - dA[M_13]*dA[M_21]*dA[M_42] ) / det;
	dC[M_44] =	(	dA[M_11]*dA[M_22]*dA[M_33] + dA[M_12]*dA[M_23]*dA[M_31] + dA[M_13]*dA[M_21]*dA[M_32]
					-	dA[M_11]*dA[M_23]*dA[M_32] - dA[M_12]*dA[M_21]*dA[M_33] - dA[M_13]*dA[M_22]*dA[M_31] ) / det;

	return S_OK;
	}	// mathInv

HRESULT mathMult ( double *dA, U32 nA, double *dB, U32 nB, 
							double *dC, U32 *nC )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Multiply two 4x4 matrices, A * B = C.
	//
	//	PARAMETERS
	//		-	dA,dB are the source matrices
	//		-	nA,nB are the element counts
	//		-	dC will receive the result
	//		-	nC will receive the element count
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// 4x4 matrix multiplied with another 4x4 matrix
	*nC	= 0;
	if (nA == 16 && nB == 16)
		{
		// 0	= _11	1	= _12	2	= _13	3	= _14
		// 4	= _21	5	= _22	6	= _23	7	= _24
		// 8	= _31	9	= _32	10 = _33 11 = _34
		// 12 = _41 13 = _42 14 = _43 15 = _44

		// Row 1
		dC[(*nC)++] = dA[0]*dB[0] + dA[1]*dB[4] + dA[2]*dB[8] + dA[3]*dB[12];
		dC[(*nC)++] = dA[0]*dB[1] + dA[1]*dB[5] + dA[2]*dB[9] + dA[3]*dB[13];
		dC[(*nC)++] = dA[0]*dB[2] + dA[1]*dB[6] + dA[2]*dB[10] + dA[3]*dB[14];
		dC[(*nC)++] = dA[0]*dB[3] + dA[1]*dB[7] + dA[2]*dB[11] + dA[3]*dB[15];

		// Row 2
		dC[(*nC)++] = dA[4]*dB[0] + dA[5]*dB[4] + dA[6]*dB[8] + dA[7]*dB[12];
		dC[(*nC)++] = dA[4]*dB[1] + dA[5]*dB[5] + dA[6]*dB[9] + dA[7]*dB[13];
		dC[(*nC)++] = dA[4]*dB[2] + dA[5]*dB[6] + dA[6]*dB[10] + dA[7]*dB[14];
		dC[(*nC)++] = dA[4]*dB[3] + dA[5]*dB[7] + dA[6]*dB[11] + dA[7]*dB[15];

		// Row 3
		dC[(*nC)++]	= dA[8]*dB[0] + dA[9]*dB[4] + dA[10]*dB[8] + dA[11]*dB[12];
		dC[(*nC)++]	= dA[8]*dB[1] + dA[9]*dB[5] + dA[10]*dB[9] + dA[11]*dB[13];
		dC[(*nC)++]	= dA[8]*dB[2] + dA[9]*dB[6] + dA[10]*dB[10] + dA[11]*dB[14];
		dC[(*nC)++]	= dA[8]*dB[3] + dA[9]*dB[7] + dA[10]*dB[11] + dA[11]*dB[15];

		// Row 4
		dC[(*nC)++]	= dA[12]*dB[0] + dA[13]*dB[4] + dA[14]*dB[8] + dA[15]*dB[12];
		dC[(*nC)++]	= dA[12]*dB[1] + dA[13]*dB[5] + dA[14]*dB[9] + dA[15]*dB[13];
		dC[(*nC)++]	= dA[12]*dB[2] + dA[13]*dB[6] + dA[14]*dB[10] + dA[15]*dB[14];
		dC[(*nC)++]	= dA[12]*dB[3] + dA[13]*dB[7] + dA[14]*dB[11] + dA[15]*dB[15];
		}	// if

	// 4x4 matrix times 4x1 matrix
	else if (nA == 4 && nB == 16)
		{
		// Row 1
		dC[(*nC)++] = dB[M_11]*dA[0] + dB[M_21]*dA[1] + dB[M_31]*dA[2] + dB[M_41]*dA[3];

		// Row 2
		dC[(*nC)++] = dB[M_12]*dA[0] + dB[M_22]*dA[1] + dB[M_32]*dA[2] + dB[M_42]*dA[3];

		// Row 3
		dC[(*nC)++] = dB[M_13]*dA[0] + dB[M_23]*dA[1] + dB[M_33]*dA[2] + dB[M_43]*dA[3];

		// Row 4
		dC[(*nC)++] = dB[M_14]*dA[0] + dB[M_24]*dA[1] + dB[M_34]*dA[2] + dB[M_44]*dA[3];
		}	// else if

	// Not implemented
	else
		hr = E_NOTIMPL;

	return hr;
	}	// mathMult

HRESULT mathOp ( const WCHAR *wOp, int *piOp )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert a string representation of an operation into its
	//			definition.
	//
	//	PARAMETERS
	//		-	wOp is the string version ("Add", etc)
	//		-	piOp will receive the integer value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Setup
	(*piOp) = MATHOP_NOP;

	// Arithmetic
	if (!WCASENCMP(wOp,L"Add",3))
		(*piOp) = MATHOP_ADD;
	else if (!WCASENCMP(wOp,L"Sub",3))
		(*piOp) = MATHOP_SUB;
	else if (!WCASENCMP(wOp,L"Mul",3))
		(*piOp) = MATHOP_MUL;
	else if (!WCASENCMP(wOp,L"Div",3))
		(*piOp) = MATHOP_DIV;
	else if (!WCASENCMP(wOp,L"Mod",3))
		(*piOp) = MATHOP_MOD;

	// Bitwise
	else if (!WCASENCMP(wOp,L"And",3))
		(*piOp) = MATHOP_AND;

	// Vector
	else if (!WCASENCMP(wOp,L"Dot",3))
		(*piOp) = MATHOP_DOT;
	else if (!WCASENCMP(wOp,L"Cross",5))
		(*piOp) = MATHOP_CROSS;

	// Trig
	else if (!WCASENCMP(wOp,L"COS",3))
		(*piOp) = MATHOP_COS;
	else if (!WCASENCMP(wOp,L"SIN",3))
		(*piOp) = MATHOP_SIN;
	else if (!WCASENCMP(wOp,L"TAN",3))
		(*piOp) = MATHOP_TAN;
	else if (!WCASENCMP(wOp,L"ACOS",4))
		(*piOp) = MATHOP_ACOS;
	else if (!WCASENCMP(wOp,L"ASIN",4))
		(*piOp) = MATHOP_ASIN;
	else if (!WCASENCMP(wOp,L"ATAN",4))
		(*piOp) = MATHOP_ATAN;

	// Other
	else if (!WCASENCMP(wOp,L"Abs",3))
		(*piOp) = MATHOP_ABS;
	else if (!WCASENCMP(wOp,L"Norm",4))
		(*piOp) = MATHOP_NORM;
	else if (!WCASENCMP(wOp,L"Ceil",4))
		(*piOp) = MATHOP_CEIL;
	else if (!WCASENCMP(wOp,L"Floor",5))
		(*piOp) = MATHOP_FLOOR;
	else if (!WCASENCMP(wOp,L"Sqrt",4))
		(*piOp) = MATHOP_SQRT;
	else if (!WCASENCMP(wOp,L"Min",3))
		(*piOp) = MATHOP_MIN;
	else if (!WCASENCMP(wOp,L"Max",3))
		(*piOp) = MATHOP_MAX;

	// Casting
	else if (!WCASENCMP(wOp,L"Int",3))
		(*piOp) = MATHOP_INT;
	else if (!WCASENCMP(wOp,L"Long",4))
		(*piOp) = MATHOP_LONG;
	else if (!WCASENCMP(wOp,L"Float",5))
		(*piOp) = MATHOP_FLOAT;
	else if (!WCASENCMP(wOp,L"Double",6))
		(*piOp) = MATHOP_DOUBLE;
	else if (!WCASENCMP(wOp,L"Date",4))
		(*piOp) = MATHOP_DATE;
	else if (!WCASENCMP(wOp,L"String",6))
		(*piOp) = MATHOP_STRING;

	// Debug
	else
		{
		dbgprintf ( L"mathOp:%s:Unrecognized operation", wOp );
		#ifdef	_DEBUG
		DebugBreak();
		#endif
		}	// else

	return hr;
	}	// mathOp

HRESULT mathSRT ( double dA[16], double dS[3], double dR[3], double dT[3], 
						double dC[16] )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Apply scale, rotation and translation to a matrix.
	//
	//	PARAMETERS
	//		-	dA is the source matrix
	//		-	dS are the X,Y,Z scaling factors
	//		-	dR are the X,Y,Z rotation factors
	//		-	dT are the X,Y,Z translation factors
	//		-	dC will receive the result
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	double	dX[16],dI[16];
	int		sz = 16*sizeof(double);
	U32		cnt;

	// Start with source
	memcpy ( dC, dA, sz );

	// Translation
	memset ( dX, 0, sz );
	dX[0]		= dX[5] = dX[10] = dX[15] = 1;
	dX[12]	= dT[0];
	dX[13]	= dT[1];
	dX[14]	= dT[2];
	mathMult ( dX, 16, dC, 16, dI, &cnt );
//	mathMult ( dC, dX, dI );
	memcpy ( dC, dI, sz );

	// Each rotation
	for (int r = 0;r < 3;++r)
		{
		// Any rotation ?
		if (dR[r] == 0.0)
			continue;

		// Trig
		float cosAng = cosf((float)(DEG_TO_RAD(dR[r])));
		float sinAng = sinf((float)(DEG_TO_RAD(dR[r])));

		// Prepare transformation matrix
		memset ( dX, 0, sz );
		dX[0] = dX[5] = dX[10] = dX[15] = 1;

		// 0	= _11	1	= _12	2	= _13	3	= _14
		// 4	= _21	5	= _22	6	= _23	7	= _24
		// 8	= _31	9	= _32	10 = _33 11 = _34
		// 12 = _41 13 = _42 14 = _43 15 = _44

		// Components depend on axis
		if (r == 0)
			{
			dX[5]		=	cosAng;	
			dX[11]	=	cosAng;	
			dX[6]		=	sinAng;	
			dX[9]		=	-sinAng;	
			}	// if
		else if (r == 1)
			{
			dX[0]		=	cosAng;	
			dX[10]	=	cosAng;	
			dX[2]		=	-sinAng;	
			dX[8]		=	sinAng;	
			}	// if
		else
			{
			dX[0]		=	cosAng;	
			dX[5]		=	cosAng;	
			dX[1]		=	sinAng;	
			dX[4]		=	-sinAng;	
			}	// if

		// Apply
		mathMult ( dX, 16, dC, 16, dI, &cnt );
//		mathMult ( dC, dX, dI );
		memcpy ( dC, dI, sz );
		}	// for

	// Scaling
	memset ( dX, 0, sz );
	dX[0]		= dS[0];
	dX[5]		= dS[1];
	dX[10]	= dS[2];
//if (dX[0] != 0) dX[0] = 1/dX[0];
//if (dX[5] != 0) dX[5] = 1/dX[5];
//if (dX[10] != 0) dX[10] = 1/dX[10];
	dX[15]	= 1;
	mathMult ( dX, 16, dC, 16, dI, &cnt );
//	mathMult ( dC, dX, dI );
	memcpy ( dC, dI, sz );

	return S_OK;
	}	// mathSRT

HRESULT mathUnary ( int iOp, const ADTVALUE &vP, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Perform a unary operation on a value.
	//
	//	PARAMETERS
	//		-	iOp is the operation to perform
	//		-	vP is the value
	//		-	v will receive the result
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Dereference values
	if (hr == S_OK && vP.vtype == (VTYPE_VALUE|VTYPE_BYREF) && vP.pval != NULL)
		return mathUnary ( iOp, *(vP.pval), v );

	// Value type 
	switch (vP.vtype)
		{
		// Integer
		case VTYPE_I4 :
			if (iOp == MATHOP_ABS)
				hr = adtValue::copy ( adtInt(abs(vP.vint)), v );
			else if (iOp == MATHOP_CEIL)
				hr = adtValue::copy ( vP, v );
			else if (iOp == MATHOP_FLOOR)
				hr = adtValue::copy ( adtDouble(floor((double)vP.vint)), v );
			else if (iOp == MATHOP_SQRT)
				hr = adtValue::copy ( adtDouble(sqrt((double)vP.vint)), v );

			// Casting
			else if (iOp == MATHOP_INT)
				hr = adtValue::copy ( adtInt(vP.vint), v );
			else if (iOp == MATHOP_LONG)
				hr = adtValue::copy ( adtLong(vP.vint), v );
			else if (iOp == MATHOP_FLOAT)
				hr = adtValue::copy ( adtFloat((float)vP.vint), v );
			else if (iOp == MATHOP_DOUBLE)
				hr = adtValue::copy ( adtDouble((double)vP.vint), v );
			else if (iOp == MATHOP_DATE)
				hr = adtValue::copy ( adtDate((double)vP.vint), v );
			else if (iOp == MATHOP_STRING)
				{
				adtString	str;
				CCLTRY ( adtValue::toString ( vP, str ) );
				CCLTRY ( adtValue::copy ( str, v ) );
				}	// else if
			else
				hr = E_NOTIMPL;
			break;

		case VTYPE_I8 :
			if (iOp == MATHOP_ABS)
				hr = adtValue::copy ( adtLong(abs(vP.vlong)), v );
			else if (iOp == MATHOP_CEIL)
				hr = adtValue::copy ( adtDouble(ceil((double)vP.vlong)), v );
			else if (iOp == MATHOP_FLOOR)
				hr = adtValue::copy ( adtDouble(floor((double)vP.vlong)), v );
			else if (iOp == MATHOP_SQRT)
				hr = adtValue::copy ( adtDouble(sqrt((double)vP.vlong)), v );

			// Casting
			else if (iOp == MATHOP_INT)
				hr = adtValue::copy ( adtInt((int)vP.vlong), v );
			else if (iOp == MATHOP_LONG)
				hr = adtValue::copy ( adtLong(vP.vlong), v );
			else if (iOp == MATHOP_FLOAT)
				hr = adtValue::copy ( adtFloat((float)vP.vlong), v );
			else if (iOp == MATHOP_DOUBLE)
				hr = adtValue::copy ( adtDouble((double)vP.vlong), v );
			else if (iOp == MATHOP_DATE)
				hr = adtValue::copy ( adtDate((double)vP.vlong), v );
			else if (iOp == MATHOP_STRING)
				{
				adtString	str;
				CCLTRY ( adtValue::toString ( vP, str ) );
				CCLTRY ( adtValue::copy ( str, v ) );
				}	// else if
			else
				hr = E_NOTIMPL;
			break;
		case VTYPE_R4  :
			if (iOp == MATHOP_ABS)
				hr = adtValue::copy ( adtFloat(fabs(vP.vflt)), v );
			else if (iOp == MATHOP_COS)
				hr = adtValue::copy ( adtFloat(cos(vP.vflt)), v );
			else if (iOp == MATHOP_SIN)
				hr = adtValue::copy ( adtFloat(sin(vP.vflt)), v );
			else if (iOp == MATHOP_TAN)
				hr = adtValue::copy ( adtFloat(tan(vP.vflt)), v );
			else if (iOp == MATHOP_ACOS)
				hr = adtValue::copy ( adtFloat(acos(vP.vflt)), v );
			else if (iOp == MATHOP_ASIN)
				hr = adtValue::copy ( adtFloat(asin(vP.vflt)), v );
			else if (iOp == MATHOP_ATAN)
				hr = adtValue::copy ( adtFloat(atan(vP.vflt)), v );
			else if (iOp == MATHOP_CEIL)
				hr = adtValue::copy ( adtFloat(ceil(vP.vflt)), v );
			else if (iOp == MATHOP_FLOOR)
				hr = adtValue::copy ( adtFloat(floor(vP.vflt)), v );
			else if (iOp == MATHOP_SQRT)
				hr = adtValue::copy ( adtDouble(sqrt(vP.vflt)), v );

			// Casting
			else if (iOp == MATHOP_INT)
				hr = adtValue::copy ( adtInt((int)vP.vflt), v );
			else if (iOp == MATHOP_LONG)
				hr = adtValue::copy ( adtLong((S64)vP.vflt), v );
			else if (iOp == MATHOP_FLOAT)
				hr = adtValue::copy ( adtFloat((float)vP.vflt), v );
			else if (iOp == MATHOP_DOUBLE)
				hr = adtValue::copy ( adtDouble((double)vP.vflt), v );
			else if (iOp == MATHOP_DATE)
				hr = adtValue::copy ( adtDate((double)vP.vflt), v );
			else if (iOp == MATHOP_STRING)
				{
				adtString	str;
				CCLTRY ( adtValue::toString ( vP, str ) );
				CCLTRY ( adtValue::copy ( str, v ) );
				}	// else if
			else
				hr = E_NOTIMPL;
			break;
		case VTYPE_R8  :
			if (iOp == MATHOP_ABS)
				hr = adtValue::copy ( adtDouble(fabs(vP.vdbl)), v );
			else if (iOp == MATHOP_COS)
				hr = adtValue::copy ( adtDouble(cos(vP.vdbl)), v );
			else if (iOp == MATHOP_SIN)
				hr = adtValue::copy ( adtDouble(sin(vP.vdbl)), v );
			else if (iOp == MATHOP_TAN)
				hr = adtValue::copy ( adtDouble(tan(vP.vdbl)), v );
			else if (iOp == MATHOP_ACOS)
				hr = adtValue::copy ( adtDouble(acos(vP.vdbl)), v );
			else if (iOp == MATHOP_ASIN)
				hr = adtValue::copy ( adtDouble(asin(vP.vdbl)), v );
			else if (iOp == MATHOP_ATAN)
				hr = adtValue::copy ( adtDouble(atan(vP.vdbl)), v );
			else if (iOp == MATHOP_CEIL)
				hr = adtValue::copy ( adtDouble(ceil(vP.vdbl)), v );
			else if (iOp == MATHOP_FLOOR)
				hr = adtValue::copy ( adtDouble(floor(vP.vdbl)), v );
			else if (iOp == MATHOP_SQRT)
				hr = adtValue::copy ( adtDouble(sqrt(vP.vdbl)), v );

			// Casting
			else if (iOp == MATHOP_INT)
				hr = adtValue::copy ( adtInt((int)vP.vdbl), v );
			else if (iOp == MATHOP_LONG)
				hr = adtValue::copy ( adtLong((S64)vP.vdbl), v );
			else if (iOp == MATHOP_FLOAT)
				hr = adtValue::copy ( adtFloat((float)vP.vdbl), v );
			else if (iOp == MATHOP_DOUBLE)
				hr = adtValue::copy ( adtDouble((double)vP.vdbl), v );
			else if (iOp == MATHOP_DATE)
				hr = adtValue::copy ( adtDate((double)vP.vdbl), v );
			else if (iOp == MATHOP_STRING)
				{
				adtString	str;
				CCLTRY ( adtValue::toString ( vP, str ) );
				CCLTRY ( adtValue::copy ( str, v ) );
				}	// else if
			else
				hr = E_NOTIMPL;
			break;
		case VTYPE_DATE  :
			if (iOp == MATHOP_ABS)
				hr = adtValue::copy ( adtDate(fabs(vP.vdate)), v );
			else if (iOp == MATHOP_COS)
				hr = adtValue::copy ( adtDate(cos(vP.vdate)), v );
			else if (iOp == MATHOP_SIN)
				hr = adtValue::copy ( adtDate(sin(vP.vdate)), v );
			else if (iOp == MATHOP_TAN)
				hr = adtValue::copy ( adtDate(tan(vP.vdate)), v );
			else if (iOp == MATHOP_ACOS)
				hr = adtValue::copy ( adtDate(acos(vP.vdate)), v );
			else if (iOp == MATHOP_ASIN)
				hr = adtValue::copy ( adtDate(asin(vP.vdate)), v );
			else if (iOp == MATHOP_ATAN)
				hr = adtValue::copy ( adtDate(atan(vP.vdate)), v );
			else if (iOp == MATHOP_CEIL)
				hr = adtValue::copy ( adtDate(ceil(vP.vdate)), v );
			else if (iOp == MATHOP_FLOOR)
				hr = adtValue::copy ( adtDouble(floor(vP.vdate)), v );
			else if (iOp == MATHOP_SQRT)
				hr = adtValue::copy ( adtDouble(sqrt(vP.vdate)), v );

			// Casting
			else if (iOp == MATHOP_INT)
				hr = adtValue::copy ( adtInt((int)vP.vdate), v );
			else if (iOp == MATHOP_LONG)
				hr = adtValue::copy ( adtLong((S64)vP.vdate), v );
			else if (iOp == MATHOP_FLOAT)
				hr = adtValue::copy ( adtFloat((float)vP.vdate), v );
			else if (iOp == MATHOP_DOUBLE)
				hr = adtValue::copy ( adtDouble((double)vP.vdate), v );
			else if (iOp == MATHOP_DATE)
				hr = adtValue::copy ( adtDate((double)vP.vdate), v );
			else if (iOp == MATHOP_STRING)
				{
				adtString	str;
				CCLTRY ( adtValue::toString ( vP, str ) );
				CCLTRY ( adtValue::copy ( str, v ) );
				}	// else if
			else
				hr = E_NOTIMPL;
			break;
		case VTYPE_UNK :
			// Vector operations
			if (iOp == MATHOP_NORM)
				{
				IDictionary	*pDct	= NULL;
				IIt			*pIt	= NULL;
				double		n		= 0;
				adtValue		vIt;

				// Access vector dictionary
				CCLTRY ( _QISAFE(vP.punk,IID_IDictionary,&pDct) );

				// Iterate and compute sum of squares
				CCLTRY ( pDct->iterate ( &pIt ) );
				while (hr == S_OK && pIt->read ( vIt ) == S_OK)
					{
					adtDouble	dV(vIt);

					// Sum
					n += (dV.vdbl*dV.vdbl);

					// Next value
					pIt->next();
					}	// while

				// Result
				CCLOK ( n = sqrt(n); )
				CCLTRY( adtValue::copy ( adtDouble(n), v ) );

				// Clean up
				_RELEASE(pIt);
				_RELEASE(pDct);
				}	// if
			break;
		default :
			hr = E_NOTIMPL;
		}	// switch

	return hr;
	}	// mathUnary
