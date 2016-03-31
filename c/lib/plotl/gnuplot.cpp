////////////////////////////////////////////////////////////////////////
//
//								GNUPLOT.CPP
//
//						Interface to the GnuPlot application
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

GnuPlot :: GnuPlot ( void )
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
	pReq			= NULL;
	pVcts			= NULL;
//	pVct			= NULL;
//	iCol			= 0;
//	punkLoc		= NULL;
//	pdctLoc		= NULL;
	bReq			= false;
//	iCnt			= 0;

	}	// GnuPlot
/*
HRESULT GnuPlot :: addCol ( IDictionary *pDct, U32 col, U32 rows )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Add a column of data to the current data block.
	//
	//	PARAMETERS
	//		-	pDct is the vector
	//		-	col is the column
	//		-	rows is the number of rows in plot to count (0 = all)
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pDctV	= NULL;
	U32			sz			= 0;
	U32			nc,nr;
	WCHAR			wName[21];
	adtValue		vL;

	// Add the current column specifications to the client plot request

	// Current number of plot requests
	CCLTRY ( pVcts->size ( &sz ) );

	// Next vector name
	CCLOK ( swprintf ( SWPF(wName,21), L"V%d", sz ); )

	// Number of rows in source vector
	CCLTRY ( mathVector ( pDct, &nc, &nr, NULL ) );

	// Row count
	if (hr == S_OK && rows != 0 && rows < nr)
		nr = rows;

	// Create a new vector dictionary for the column
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctV ) );
	CCLTRY ( mathVectorSize ( pDctV, 1, nr ) );

	// Extract the specified column number from the provided vector dictionary
	CCLTRY ( mathVectorCopyCol ( pDct, col, pDctV, 0 ) );

	// Generate label based on optional name and units
	if (hr == S_OK && pDct->load ( adtString(L"Name"), vL ) == S_OK)
		{
		adtString	strLabel(vL);

		// Units ?
		if (pDct->load ( adtString(L"Units"), vL ) == S_OK)
			{
			adtString strUnit(vL);

			// Append to label
			CCLTRY ( strLabel.append ( L" (" ) );
			CCLTRY ( strLabel.append ( strUnit ) );
			CCLTRY ( strLabel.append ( L")" ) );
			}	// if

		// Store as label
		CCLTRY ( pDctV->store ( adtString(L"Label"), strLabel ) );
		}	// if

	// Store in request
	CCLTRY ( pVcts->store ( adtString(wName), adtIUnknown(pDctV) ) );

	// Clean up
	_RELEASE(pDctV);

	return hr;
	}	// addCol
*/
HRESULT GnuPlot :: construct ( void )
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
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pVcts ) );
	CCLTRY ( pReq->store ( adtString(L"Vector"), adtIUnknown(pVcts) ) );

	return hr;
	}	// construct

void GnuPlot :: destruct ( void )
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
//	_RELEASE(pVct);
	_RELEASE(pVcts);
	_RELEASE(pReq);
	}	// destruct

HRESULT GnuPlot :: onAttach ( bool bAttach )
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
		// Single GnuPlot server
		if (hr == S_OK && uGnuCnt == 0)
			{
			// Create the server object
			CCLTRYE ( (pGnuSrvr = new GnuPlotSrvr()) != NULL, E_OUTOFMEMORY );
			CCLOK	  ( pGnuSrvr->AddRef(); )
			CCLTRY  ( pGnuSrvr->construct() );

			// Start running immediately
			CCLTRY ( pGnuSrvr->run(true) );
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

HRESULT GnuPlot :: receive ( IReceptor *pr, const WCHAR *pl, 
										const ADTVALUE &v )
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
		U32			sz	= 0;
		adtValue		vL;

		// State check
		CCLTRYE ( pGnuSrvr != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pVcts->size ( &sz ) == S_OK && sz > 0, ERROR_INVALID_STATE );

		// Request validity
		bReq = (hr == S_OK);

		// Send request to GNU server.  In order to support multiple
		// synchronized clients, the server stores the result directly
		// in the request
		CCLTRY ( pGnuSrvr->plot ( pReq ) );

		// Was a plot generated ?
		if (hr == S_OK && pReq->load ( strRefOnImg, vL ) == S_OK)
			_EMT(Fire,vL);
		else
			_EMT(Error,adtInt(hr));
		}	// if

	return hr;
	}	// receive

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


	// Add the current vector column to the plot
	else if (!WCASECMP(L"Add",strKey))
		{
		// State check
		CCLTRYE ( pVct != NULL && iCol >= 0, ERROR_INVALID_STATE );

		// Add column to request
		CCLTRY ( addCol ( pVct, iCol, iCnt ) );
		}	// else if

	// Reset the current plot state
	else if (!WCASECMP(L"Reset",strKey))
		{
		// Clear vectors of current request
		CCLTRY ( pVcts->clear() );
		bReq = false;
		}	// else if

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

	// Plot range.  Ranges are specified in percent of each axis
	else if (!WCASECMP(L"Range",strKey))
		{
		IDictionary	*pDct	= NULL;
		adtIUnknown	unkV(vValue);
		adtValue		vL;

		// State check
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDct) );

		// Transfer into request
		if (hr == S_OK && pDct->load ( strRefLeft, vL ) == S_OK)
			hr = pReq->store ( strRefLeft, vL );
		if (hr == S_OK && pDct->load ( strRefBottom, vL ) == S_OK)
			hr = pReq->store ( strRefBottom, vL );
		if (hr == S_OK && pDct->load ( strRefRight, vL ) == S_OK)
			hr = pReq->store ( strRefRight, vL );
		if (hr == S_OK && pDct->load ( strRefTop, vL ) == S_OK)
			hr = pReq->store ( strRefTop, vL );

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
