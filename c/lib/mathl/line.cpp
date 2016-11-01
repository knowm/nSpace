////////////////////////////////////////////////////////////////////////
//
//									LINE.CPP
//
//					Implementation of the line node
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>
#include <math.h>

Line :: Line ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////

	// Defaults
	dM		= 1.0;
	bVert = false;

	}	// Line

HRESULT Line :: onAttach ( bool bAttach )
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
	adtValue	vL;

	// State check
	if (!bAttach) return S_OK;

	// Default states
	if (pnDesc->load ( adtString(L"X1"), vL ) == S_OK)
		dX1 = vL;
	if (pnDesc->load ( adtString(L"X2"), vL ) == S_OK)
		dX2 = vL;
	if (pnDesc->load ( adtString(L"Y1"), vL ) == S_OK)
		dY1 = vL;
	if (pnDesc->load ( adtString(L"Y2"), vL ) == S_OK)
		dY2 = vL;
	if (pnDesc->load ( adtString(L"M"), vL ) == S_OK)
		dM = vL;
	if (pnDesc->load ( adtString(L"B"), vL ) == S_OK)
		dB = vL;

	return S_OK;
	}	// onAttach

HRESULT Line :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Debug
//	if (!WCASECMP(this->strnName,L"LightAvgX"))
//		dbgprintf ( L"Hi\r\n" );

	// Evaluate line at point
	if (_RCP(X))
		{
		adtDouble	dX(v),dRes;

		// Given X, evaluate Y

		// Y = MX + B
		if (bVert)
			dRes = dB;
		else
			dRes = dM * dX + dB;

		// Result
		_EMT(Y,dRes);
		}	// if
	else if (_RCP(Y))
		{
		adtDouble	dY(v),dRes;

		// Given Y, evaluate X

		// X = (Y - B)/M
		if (bVert)
			dRes = dB;
		else
			dRes = (dY - dB) / dM;

		// Result
		_EMT(X,dRes);
		}	// if

	// Evaluate with two-point formuls
	else if (_RCP(Two))
		{
		double	dX,dY;

		// Y - Y1 = ((Y2 - Y1)/(X2 - X1))*(X - X1)
		dX = (dX2-dX1);
		dY = (dY2-dY1);

		// Vertical ?
		bVert = (fabs(dX) < 1.e-20);

		// Evalulate
		if (!bVert)
			{
			// Slope
			dM = dY/dX;

			// Y-intercept
			dB = dY1 - dM*dX1;
			}	// if
		else
			{
			// Vertical line
			dM = 1.0;
			dB = dY1;
			}	// else

		// Result
		_EMT(M,dM);
		_EMT(B,dB);
		}	// else if

	// State
	else if (_RCP(M))
		dM = v;
	else if (_RCP(B))
		dB = v;
	else if (_RCP(X1))
		dX1 = v;
	else if (_RCP(X2))
		dX2 = v;
	else if (_RCP(Y1))
		dY1 = v;
	else if (_RCP(Y2))
		dY2 = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


