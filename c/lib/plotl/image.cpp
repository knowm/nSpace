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
	pDataIn		= NULL;
	iIdx			= -1;
	iPlotW		= 800;
	iPlotH		= 600;
	
	pData			= NULL;
	pDataBits	= NULL;
	iDataW		= 0;
	iDataH		= 0;
	pReq			= NULL;
	bReq			= false;

	}	// Image

HRESULT Image :: addRow ( IDictionary *pDct, U32 iRow )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a row of data to the current data block.
	//
	//	PARAMETERS
	//		-	pDct contains the data
	//		-	col is the column
	//		-	rows is the number of rows in plot to count (0 = all)
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IMemoryMapped	*pBits	= NULL;
	VOID				*pvBits	= NULL;
	VOID				*pvData	= NULL;
	float				*pfData	= NULL;
	adtInt			iW,iH;
	adtString		strFmt;
	WCHAR				wName[21];
	adtValue			vL;
	adtIUnknown		unkV;

	// Add the data row to the client plot request

	// Next vector name
	CCLOK ( swprintf ( SWPF(wName,21), L"V%d", (U32)iH ); )

	// Format of incoming data
	CCLTRY ( pDct->load ( adtString(L"Format"), vL ) );
	CCLTRYE( (strFmt = vL).length() > 0, E_INVALIDARG );

	// Lock down incoming data
	CCLTRY ( pDct->load ( strRefWidth, vL ) );
	CCLOK  ( iW = vL; )
	CCLTRY ( pDct->load ( strRefHeight, vL ) );
	CCLOK  ( iH = vL; )
	CCLTRY ( pDct->load ( strRefBits, vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,&pBits) );
	CCLTRY ( pBits->lock ( 0, 0, &pvBits, NULL ) );

	// Valid row specified ?
	CCLTRYE ( iRow < iH, E_INVALIDARG );

	// Another row
	CCLOK ( iDataH = iDataH + 1; )

	// First vector ?
	if (hr == S_OK && iDataW == 0)
		iDataW = iW;

	// For now vectors must be same length
	CCLTRYE ( iW == iDataW, E_INVALIDARG );

	// Allocate space for another row
	CCLTRY ( pDataBits->setSize ( iDataW*iDataH*sizeof(float) ) );
	CCLTRY ( pDataBits->lock ( 0, 0, (void **) &pvData, NULL ) );
	CCLOK  ( pfData = (float *)pvData; )
	CCLOK  ( pfData += (iDataH-1)*iDataW; )

	// Add more formts as necessary
	if (!WCASECMP(strFmt,L"U16x2"))
		{
		// Source bits
		U16	*piSrc = ((U16 *) pvBits) + (iRow*iW);

		// Copy into new row
		for (U32 c = 0;c < iW;++c)
			pfData[c] = piSrc[c];
		}	// if
/*
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
//		CCLTRY ( pDctV->store ( adtString(L"Label"), strLabel ) );
		}	// if
*/

	// Update plot data information
	CCLTRY ( pData->store ( strRefWidth, iDataW ) );
	CCLTRY ( pData->store ( strRefHeight, iDataH ) );

	// Clean up
	_UNLOCK(pDataBits,pvData);
	_UNLOCK(pBits,pvBits);
	_RELEASE(pBits);

	return hr;
	}	// addRow

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

	// Plot data
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pData ) );
	CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pDataBits ) );
	CCLTRY ( pData->store ( adtString(L"Bits"), adtIUnknown(pDataBits) ) );
	CCLTRY ( pData->store ( adtString(L"Format"), adtString(L"F32x2") ) );

	// Plot request
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pReq ) );
	CCLTRY ( pReq->store ( adtString(L"Data"), adtIUnknown(pData) ) );

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
	_RELEASE(pDataIn);
	_RELEASE(pDataBits);
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

HRESULT Image :: receive ( IReceptor *pr, const WCHAR *pl, 
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
		CCLTRYE ( iDataW > 0 && iDataH > 0, ERROR_INVALID_STATE );

		// Request validity
		bReq = (hr == S_OK);

		// Latest title
		if (hr == S_OK && strTitle.length() > 0)
			hr = pReq->store ( adtString(L"Title"), strTitle );
		else if (hr == S_OK)
			pReq->remove ( adtString(L"Title") );

		// Latest size
		CCLTRY ( pReq->store ( strRefWidth, iPlotW ) );
		CCLTRY ( pReq->store ( strRefHeight, iPlotH ) );

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

	// Add to plot
	else if (_RCP(Add))
		{
		// State check
		CCLTRYE ( pDataIn != NULL && iIdx >= 0, ERROR_INVALID_STATE );

		// Add column to request
		CCLTRY ( addRow ( pDataIn, iIdx ) );
		}	// else if

	// Reset plot
	else if (_RCP(Reset))
		{
		// Clear vectors of current request
		iDataW = iDataH = 0;
		bReq = false;
		}	// else if

	// State
	else if (_RCP(Index))
		iIdx = v;
	else if (_RCP(Data))
		{
		adtIUnknown		unkV(v);
		_RELEASE(pDataIn);
		hr = _QISAFE(unkV,IID_IDictionary,&pDataIn);
		}	// else if

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