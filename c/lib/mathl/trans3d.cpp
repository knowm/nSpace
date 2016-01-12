////////////////////////////////////////////////////////////////////////
//
//									TRANS3D.CPP
//
//					Implementation of a 3D transform matrix node.
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>

Transform3D :: Transform3D ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	memset ( dIn, 0, sizeof(dIn) );
	memset ( dSrt, 0, sizeof(dSrt) );
	dIn[0]	= dIn[5]		= dIn[10]	= 1;
	memcpy ( dOut, dIn, sizeof(dOut) );
	memcpy ( dOutP, dIn, sizeof(dOutP) );
	dSrt[0]	= dSrt[1]	= dSrt[2]	= 1;
	}	// Transform3D

HRESULT Transform3D :: input ( double *dA, int idx, double v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Update the index of the input array.
	//
	//	PARAMETERS
	//		-	dA is the array
	//		-	idx is the index of the entry
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Update on change
	if (dA[idx] != v)
		{
		dA[idx] = v;
		hr = update();
		}	// if

	return hr;
	}	// input

HRESULT Transform3D :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when this behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		}	// if

	// Detach
	else
		{
		// Clean up
		}	// else

	return hr;
	}	// onAttach

HRESULT Transform3D :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// State

	// Input matrix
	if (_RCP(11))
		input ( dIn, 0, (vD = v) );
	else if (_RCP(12))
		input ( dIn, 1, (vD = v) );
	else if (_RCP(13))
		input ( dIn, 2, (vD = v) );
	else if (_RCP(21))
		input ( dIn, 4, (vD = v) );
	else if (_RCP(22))
		input ( dIn, 5, (vD = v) );
	else if (_RCP(23))
		input ( dIn, 6, (vD = v) );
	else if (_RCP(31))
		input ( dIn, 8, (vD = v) );
	else if (_RCP(32))
		input ( dIn, 9, (vD = v) );
	else if (_RCP(33))
		input ( dIn, 10, (vD = v) );
	else if (_RCP(41))
		input ( dIn, 12, (vD = v) );
	else if (_RCP(42))
		input ( dIn, 13, (vD = v) );
	else if (_RCP(43))
		input ( dIn, 14, (vD = v) );

	// Scale/Rotate/Translate
	else if (_RCP(ScaleX))
		input ( dSrt, 0, (vD=v) );
	else if (_RCP(ScaleY))
		input ( dSrt, 1, (vD=v) );
	else if (_RCP(ScaleZ))
		input ( dSrt, 2, (vD=v) );
	else if (_RCP(RotateX))
		input ( dSrt, 3, (vD=v) );
	else if (_RCP(RotateY))
		input ( dSrt, 4, (vD=v) );
	else if (_RCP(RotateZ))
		input ( dSrt, 5, (vD=v) );
	else if (_RCP(TranslateX))
		input ( dSrt, 6, (vD=v) );
	else if (_RCP(TranslateY))
		input ( dSrt, 7, (vD=v) );
	else if (_RCP(TranslateZ))
		input ( dSrt, 8, (vD=v) );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT Transform3D :: update ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Update the transformation matrix.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Debug
//	dbgprintf ( L"Transform3D:update:%g,%g,%g %g,%g,%g %g,%g,%g %g,%g,%g\r\n",
//					dIn[0], dIn[1], dIn[2],
//					dIn[4], dIn[5], dIn[6],
//					dIn[8], dIn[9], dIn[10],
//					dIn[12], dIn[13], dIn[14] );
//	dbgprintf ( L"Transform3D:update:%g,%g,%g %g,%g,%g %g,%g,%g\r\n",
//					dSrt[0], dSrt[1], dSrt[2],
//					dSrt[3], dSrt[4], dSrt[5],
//					dSrt[6], dSrt[7], dSrt[8] );

	// Apply SRT
	CCLTRY ( mathSRT ( dIn, &dSrt[0], &dSrt[3], &dSrt[6], dOut ) );

	// Output changes
	for (int i = 0;i < 16;++i)
		if (dOut[i] != dOutP[i])
			{
			// Output receptor, skip constant/unused matrix entries
			IReceptor	*rcp	=	(i == 0)		? peOn11 :
										(i == 1)		? peOn12 :
										(i == 2)		? peOn13 :
										(i == 4)		? peOn21 :
										(i == 5)		? peOn22 :
										(i == 6)		? peOn23 :
										(i == 8)		? peOn31 :
										(i == 9)		? peOn32 :
										(i == 10)	? peOn33 :
										(i == 12)	? peOn41 :
										(i == 13)	? peOn42 :
										(i == 14)	? peOn43 : NULL;

			// Output value
			if (rcp != NULL)
				rcp->receive ( this, L"Value", (vD = dOut[i]) );
			}	// if

	// 'Previous' result
	CCLOK ( memcpy ( dOutP, dOut, sizeof(dOut) ); )

	return hr;
	}	// update

