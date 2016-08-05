////////////////////////////////////////////////////////////////////////
//
//								IMAGE.CPP
//
//						Interface to the plot image node
//
////////////////////////////////////////////////////////////////////////

#define INITGUID
#include "plotl_.h"
#include "../../lib/mathl/mathl.h"
#include <stdio.h>
#include <math.h>

// Single server shared across plot objects
static	GnuPlotSrvr	*pGnuSrvr	= NULL;
static	U32			uGnuCnt		= 0;

// String references
extern adtString	strRefOnImg;
extern adtString	strRefWidth;
extern adtString	strRefHeight;
extern adtString	strRefLeft;
extern adtString	strRefRight;
extern adtString	strRefTop;
extern adtString	strRefBottom;
extern adtString	strRefBits;

Image :: Image ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for object.
	//
	//	PARAMETERS
	//		-	hInst is the application instance
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	iIdx			= -1;
	iPlotW		= 800;
	iPlotH		= 600;
	pData			= NULL;
	pReq			= NULL;
	}	// Image

HRESULT Image :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Plot request
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pReq ) );

	return hr;
	}	// construct

void Image :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// GnuPlot server
	onAttach(false);

	// Clean up
	_RELEASE(pData);
	_RELEASE(pReq);
	}	// destruct

HRESULT Image :: onAttach ( bool bAttach )
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
		adtValue vL;

		// Defaults
		if (pnDesc->load ( adtString(L"Title"), vL ) == S_OK)
			adtValue::toString ( vL, strTitle );
		if (pnDesc->load ( adtString(L"Index"), vL ) == S_OK)
			iIdx = vL;
		if (pnDesc->load ( adtString(L"Width"), vL ) == S_OK)
			iPlotW = vL;
		if (pnDesc->load ( adtString(L"Height"), vL ) == S_OK)
			iPlotH = vL;
		if (pnDesc->load ( adtString(L"LabelX0"), vL ) == S_OK)
			adtValue::toString ( vL, strLblX0 );
		if (pnDesc->load ( adtString(L"LabelX1"), vL ) == S_OK)
			adtValue::toString ( vL, strLblX1 );
		if (pnDesc->load ( adtString(L"LabelY0"), vL ) == S_OK)
			adtValue::toString ( vL, strLblY0 );
		if (pnDesc->load ( adtString(L"LabelY1"), vL ) == S_OK)
			adtValue::toString ( vL, strLblY1 );

		// Single GnuPlot server
		if (hr == S_OK && uGnuCnt == 0)
			{
			// Create the server object
			CCLTRYE ( (pGnuSrvr = new GnuPlotSrvr()) != NULL, E_OUTOFMEMORY );
			CCLOK	  ( pGnuSrvr->AddRef(); )
			CCLTRY  ( pGnuSrvr->construct() );

			// Start running immediately
			CCLTRY ( pGnuSrvr->run(true) );

			// Do not fail attachment if GnuPlot is not available
			if (hr != S_OK)
				{
				lprintf ( LOG_ERR, L"Unable to create GnuPlot server, installed ?" );
				hr = S_OK;
				}	// if
			}	// if
		CCLOK ( ++uGnuCnt; )
		}	// if

	// Detach
	else
		{
		// Shutdown
		if (uGnuCnt == 1)
			{
			// Shutdown
			if (pGnuSrvr != NULL)
				pGnuSrvr->run(false);
			_RELEASE(pGnuSrvr);
			}	// if
		if (uGnuCnt > 0)
			--uGnuCnt;
		}	// else

	return hr;
	}	// onAttach

HRESULT Image :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Generate plot
	if (_RCP(Fire))
		{
		// Update plot
		update();
		}	// if

	// Plot range.  Ranges are specified in percent of each axis
	else if (_RCP(Range))
		{
		IDictionary	*pDct	= NULL;
		adtIUnknown	unkV(v);
		adtValue		vL;

		// State check
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDct) );

		// Transfer into request
		if (hr == S_OK && pDct->load ( strRefLeft, vL ) == S_OK)
			hr = pReq->store ( strRefLeft, adtFloat(vL) );
		if (hr == S_OK && pDct->load ( strRefBottom, vL ) == S_OK)
			hr = pReq->store ( strRefBottom, adtFloat(vL) );
		if (hr == S_OK && pDct->load ( strRefRight, vL ) == S_OK)
			hr = pReq->store ( strRefRight, adtFloat(vL) );
		if (hr == S_OK && pDct->load ( strRefTop, vL ) == S_OK)
			hr = pReq->store ( strRefTop, adtFloat(vL) );

		// If a valid state is currently valid, refresh the plot with the new size automatically
		CCLOK ( update(); )

		// Clean up
		_RELEASE(pDct);
		}	// else if

	// Size
	else if (_RCP(Width) || _RCP(Height))
		{
		adtInt	iSz(v);

		// Double word aligned
		if (_RCP(Width))
			iSz = ((iSz % 4) == 0) ? iSz : ((iSz/4)*4 + 4);

		// Store size in request
		CCLTRY ( pReq->store ( _RCP(Width) ? strRefWidth : strRefHeight, iSz ) );

		// Update state
		CCLOK ( update(); )
		}	// else if

	// State
	else if (_RCP(Data))
		{
		adtIUnknown unkV(v);
		_RELEASE(pData);
		_QISAFE(unkV,IID_IDictionary,&pData);
		}	// else if

	return hr;
	}	// receive

HRESULT Image :: update ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Update the plot if request is currently valid.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	adtValue vL;

	// State check
	CCLTRYE ( pGnuSrvr != NULL, ERROR_INVALID_STATE );
	CCLTRYE ( pData != NULL, ERROR_INVALID_STATE );

	// Data block		
	CCLTRY ( pReq->store ( adtString(L"Data"), adtIUnknown(pData) ) );

	// Latest title
	if (hr == S_OK && strTitle.length() > 0)
		hr = pReq->store ( adtString(L"Title"), strTitle );
	else if (hr == S_OK)
		pReq->remove ( adtString(L"Title") );

	// Latest labels
	if (hr == S_OK && strLblX0.length() > 0)
		hr = pReq->store ( adtString ( L"LabelX0" ), strLblX0 );
	else
		pReq->remove ( adtString(L"LabelX0") );
	if (hr == S_OK && strLblX1.length() > 0)
		hr = pReq->store ( adtString ( L"LabelX1" ), strLblX1 );
	else
		pReq->remove ( adtString(L"LabelX1") );
	if (hr == S_OK && strLblY0.length() > 0)
		hr = pReq->store ( adtString ( L"LabelY0" ), strLblY0 );
	else
		pReq->remove ( adtString(L"LabelY0") );
	if (hr == S_OK && strLblY1.length() > 0)
		hr = pReq->store ( adtString ( L"LabelY1" ), strLblY1 );
	else
		pReq->remove ( adtString(L"LabelY1") );

	// Send request to GNU server.  In order to support multiple
	// synchronized clients, the server stores the result directly
	// in the request
	CCLTRY ( pGnuSrvr->plot ( pReq ) );

	// Was a plot generated ?
	if (hr == S_OK && pReq->load ( strRefOnImg, vL ) == S_OK)
		_EMT(Fire,vL);
	else
		_EMT(Error,adtInt(hr));

	return hr;
	}	// update

/*
HRESULT GnuPlot :: onStore ( const ADTVALUE &vKey, const ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ILocationNotify
	//
	//	PURPOSE
	//		-	Notifies this location that a key/value pair has been stored
	//			in its dictionary
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	adtString	strKey(vKey);

	// Plot size.  The plot size can change, as an example, when the hosted window changes size.
	else if (!WCASECMP(L"Size",strKey))
		{
		IDictionary	*pDct	= NULL;
		adtIUnknown	unkV(vValue);
		adtValue		vL;

		// State check
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDct) );

		// Transfer size into request
		CCLTRY ( pDct->load ( strRefWidth, vL ) );
		CCLTRY ( pReq->store ( strRefWidth, vL ) );
		CCLTRY ( pDct->load ( strRefHeight, vL ) );
		CCLTRY ( pReq->store ( strRefHeight, vL ) );

		// If a valid state is currently valid, refresh the plot with the new size automatically
		if (hr == S_OK && bReq)
			onStore ( adtString(L"Fire"), adtInt(0) );

		// Clean up
		_RELEASE(pDct);
		}	// else if

	// Label closest point
	else if (!WCASECMP(L"LabelPt",strKey))
		{
		IDictionary	*pDct	= NULL;
		adtIUnknown	unkV(vValue);
		adtValue		vL;

		// State check
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDct) );

		// Transfer into request
		CCLTRY ( pDct->load ( strRefX, vL ) );
		CCLTRY ( pReq->store ( adtString(L"LabelX"), vL ) );
		CCLTRY ( pDct->load ( strRefY, vL ) );
		CCLTRY ( pReq->store ( adtString(L"LabelY"), vL ) );

		// If a valid state is currently valid, refresh the plot with the new size automatically
		if (hr == S_OK && bReq)
			onStore ( adtString(L"Fire"), adtInt(0) );

		// Clean up
		_RELEASE(pDct);
		}	// else if

	// Plot is 3D
	else if (!WCASECMP(L"3D",strKey))
		hr = pReq->store ( vKey, vValue );

	// Plot title
	else if (!WCASECMP(L"Title",strKey))
		hr = pReq->store ( vKey, vValue );

	// State

	// Active vector
	else if (!WCASECMP(L"Vector",strKey))
		{
		adtIUnknown	unkV(vValue);
		_RELEASE(pVct);
		CCLTRY(_QISAFE(unkV,IID_IDictionary,&pVct));
		}	// else if

	// Active column
	else if (!WCASECMP(L"Column",strKey))
		iCol = adtInt(vValue);

	// Active count of rows to plot (0 = all)
	else if (!WCASECMP(L"Count",strKey))
		iCnt = adtInt(vValue);

	return hr;
	}	// onStore

*/
