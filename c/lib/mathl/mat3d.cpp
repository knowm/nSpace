////////////////////////////////////////////////////////////////////////
 //
//									MAT3D.CPP
//
//						Implementation of a 3D, 4x4 matrix.
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>

Matrix3D :: Matrix3D ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	memset ( dA, 0, sizeof(dA) );
	memset ( dB, 0, sizeof(dB) );
	memset ( dC, 0, sizeof(dC) );
	nA			= 0;
	nB			= 0;
	pA			= NULL;
	pB			= NULL;
	pC			= NULL;
	dScl[0]	= dScl[1]	= dScl[2]	= 1;
	dTrns[0] = dTrns[1]	= dTrns[2]	= 0;
	dRot[0]	= dRot[1]	= dRot[2]	= 0;
	}	// Matrix3D

HRESULT Matrix3D :: onAttach ( bool bAttach )
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
		_RELEASE(pA);
		_RELEASE(pB);
		_RELEASE(pC);
		}	// else

	return hr;
	}	// onAttach

HRESULT Matrix3D :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// NOTE: It is assumed matrix dictionaries stored the individual elements
	// as left-to-right/top-to-bottom key/value pairs.

//if (!WCASECMP(strnName,L"WidgetMat"))
//	dbgprintf ( L"Hi\r\n" );

	// Multiply
	if (_RCP(Multiply))
		{
		IIt		*pIt	= NULL;
		U32		i,nR,nC;

		// State check
		CCLTRYE ( pA != NULL && pB != NULL && pC != NULL, ERROR_INVALID_STATE );

		// Always use 4x1
		nR = nA;
		if (hr == S_OK && nR == 3)
			dA[nR++] = 1;

		// Mutiply
		CCLTRY ( mathMult ( dA, nR, dB, nB, dC, &nC ) );

		// Store result, use same keys as source dictionaries
		CCLTRY ( pA->keys ( &pIt ) );
		for (i = 0;hr == S_OK && i < nC && i < nA;++i)
			{
			// Read next key and assign
			CCLTRY ( pIt->read ( vK ) );
			CCLOK  ( pIt->next(); )
			CCLTRY ( pC->store ( vK, (vD=dC[i]) ) );
			}	// for

		// Clean up
		_RELEASE(pIt);

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pC) );
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Identity
	else if (_RCP(Identity))
		{
		IIt		*pIt	= NULL;
		U32		i;

		// State check
		CCLTRYE ( pA != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( nA == 3 || nA == 4 || nA == 16, ERROR_INVALID_STATE );

		// Set components to identity
		CCLTRY ( pA->keys ( &pIt ) );
		for (i = 0;hr == S_OK && i < nA;++i)
			{
			// Read next key and assign
			CCLTRY ( pIt->read ( vK ) );
			CCLOK  ( pIt->next(); )

			// Identity depends on size
			if (hr == S_OK && nA <= 4)
				hr = pA->store ( vK, (vD=1.0) );
			else if (hr == S_OK && nA == 16)
				hr = pA->store ( vK, (vD =
							(i == 0 || i == 5 || i == 10 || i == 15 ) ? 1.0 : 0.0 ) );
			}	// for

		// Clean up
		_RELEASE(pIt);

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pA) );
		else
			_EMT(Error,adtInt(hr));
		}	// else if

	// Invert
	else if (_RCP(Invert))
		{
		IIt		*pIt	= NULL;
		U32		i;

		// State check
		CCLTRYE ( pA != NULL && pC != NULL, ERROR_INVALID_STATE );

		// Inverse current requires 4x4 matrix
		CCLTRYE ( nA == 16, ERROR_INVALID_STATE );

		// Invert
		CCLTRY ( mathInv ( dA, dC ) );

		// Store result, use same keys as source dictionaries
		CCLTRY ( pA->keys ( &pIt ) );
		for (i = 0;hr == S_OK && i < nA;++i)
			{
			// Read next key and assign
			CCLTRY ( pIt->read ( vK ) );
			CCLOK  ( pIt->next(); )
			CCLTRY ( pC->store ( vK, (vD=dC[i]) ) );
			}	// for

		// Clean up
		_RELEASE(pIt);

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pC) );
		else
			_EMT(Error,adtInt(hr));
		}	// else if

	// A/B matrix
	else if (_RCP(A) || _RCP(B))
		{
		IDictionary	*pMat	= NULL;
		IIt			*pIt	= NULL;
		U32			sz;
		double		*dV;

		// Assign array
		dV = (_RCP(A)) ? dA : dB;

		// State check
		CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pMat) );
		CCLTRY ( pMat->size ( &sz ) );

		// Allow for a 4x4, 4x1 (x,y,z,1) or 3x1 (x,y,z) matrix
		CCLTRYE( sz == 16 || sz == 4 || sz == 3, ERROR_INVALID_STATE );

		// Cache the values for later for speed
		CCLOK  ( sz = 0; )
		CCLTRY ( pMat->iterate ( &pIt ) );
		while (hr == S_OK && pIt->read ( vL ) == S_OK)
			{
			// Assign value
			dV[sz++] = (vD=vL);

			// Clean up
			pIt->next();
			}	// while

		// Assign side
		if (hr == S_OK && _RCP(A))
			{
			_RELEASE(pA);
			pA = pMat;
			_ADDREF(pA);
			nA = sz;
			}	// if
		else if (hr == S_OK && _RCP(B))
			{
			_RELEASE(pB);
			pB = pMat;
			_ADDREF(pB);
			nB = sz;
			}	// if

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pMat);
		}	// if

	// C matrix (output)
	else if (_RCP(C))
		{
		_RELEASE(pC);
		hr = _QISAFE((unkV=v),IID_IDictionary,&pC);
		}	// else if

	// Apply SRT
	else if (_RCP(Apply))
		{
		IIt		*pIt	= NULL;
		U32		i;

		// State check
		CCLTRYE ( pA != NULL, ERROR_INVALID_STATE );

		// Must be 4x4 matrix for this operation to make sense
		CCLTRYE ( nA == 16, ERROR_INVALID_STATE );

		// Apply SRT
		CCLTRY ( mathSRT ( dA, dScl, dRot, dTrns, dC ) );

		// Store components in result
		CCLTRY ( pA->keys ( &pIt ) );
		for (i = 0;hr == S_OK && i < nA;++i)
			{
			// Read next key and assign
			CCLTRY ( pIt->read ( vK ) );
			CCLOK  ( pIt->next(); )
			CCLTRY ( pC->store ( vK, (vD=dC[i]) ) );
			}	// for

		// Clean up
		_RELEASE(pIt);

		// Result
		if (hr == S_OK)
			_EMT(Fire,(adtIUnknown(pC)) );
		else
			_EMT(Error,adtInt(iV));
		}	// else if

	// Scale/Rotate/Translate
	else if (_RCP(ScaleX))
		dScl[0] = (vD=v);
	else if (_RCP(ScaleY))
		dScl[1] = (vD=v);
	else if (_RCP(ScaleZ))
		dScl[2] = (vD=v);
	else if (_RCP(RotateX))
		dRot[0] = (vD=v);
	else if (_RCP(RotateY))
		dRot[1] = (vD=v);
	else if (_RCP(RotateZ))
		dRot[2] = (vD=v);
	else if (_RCP(TranslateX))
		dTrns[0] = (vD=v);
	else if (_RCP(TranslateY))
		dTrns[1] = (vD=v);
	else if (_RCP(TranslateZ))
		dTrns[2] = (vD=v);
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


