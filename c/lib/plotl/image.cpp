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

HRESULT Image :: addRow ( IUnknown *pUnk, U32 iRow )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a row of data to the current data block.
	//
	//	PARAMETERS
	//		-	pUnk contains the data
	//		-	iRow is the row number to add
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr				= S_OK;
	IDictionary		*pDctData	= NULL;
	IMemoryMapped	*pBits		= NULL;
	VOID				*pvBits		= NULL;
	VOID				*pvData		= NULL;
	float				*pfData		= NULL;
	adtInt			iW,iH;
	adtString		strFmt;
	adtValue			vL;
	adtIUnknown		unkV;

	//
	// Add row from active data object.
	// Add as many different formats as necessary over time.
	//

	// Dictionary ?
	if (hr == S_OK && _QI(pUnk,IID_IDictionary,&pDctData) == S_OK)
		{
		// Format specified for incoming data ?
		if (hr == S_OK && pDctData->load ( adtString(L"Format"), vL ) == S_OK)
			{
			// Format of incoming data
			CCLTRYE( (strFmt = vL).length() > 0, E_INVALIDARG );

			// Lock down incoming data
			CCLTRY ( pDctData->load ( strRefWidth, vL ) );
			CCLOK  ( iW = vL; )
			CCLTRY ( pDctData->load ( strRefHeight, vL ) );
			CCLOK  ( iH = vL; )
			CCLTRY ( pDctData->load ( strRefBits, vL ) );
			CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,&pBits) );
			CCLTRY ( pBits->lock ( 0, 0, &pvBits, NULL ) );
			}	// if

		// No format, for now assume list of values
		else if (hr == S_OK)
			{
			U32	sz;

			// Count is 'width'
			CCLTRY ( pDctData->size ( &sz ) );
			CCLOK  ( iW = sz; )
			CCLOK  ( iH = 1; )
			}	// else if

		}	// if

	// Valid row specified ?
	CCLTRYE ( iRow < iH, E_INVALIDARG );

	// Another row
	CCLOK	( iDataH = iDataH + 1; )
	CCLTRY( pData->store ( strRefHeight, iDataH ) );

	// First vector ?
	if (hr == S_OK && iDataW == 0)
		{
		// First vector determines width
		iDataW = iW;

		// Update data information
		CCLTRY ( pData->store ( strRefWidth, iDataW ) );
		}	// if

	// For now vectors must be same length
	CCLTRYE ( iW == iDataW, E_INVALIDARG );

	// Allocate space for another row
	CCLTRY ( pDataBits->setSize ( iDataW*iDataH*sizeof(float) ) );
	CCLTRY ( pDataBits->lock ( 0, 0, (void **) &pvData, NULL ) );
	CCLOK  ( pfData = (float *)pvData; )
	CCLOK  ( pfData += (iDataH-1)*iDataW; )

	// Add more formats as necessary
	if (hr == S_OK && pDctData != NULL)
		{
		// Debug
//		for (U32 c = 0;c < iW;++c)
//			pfData[c] = 0;

		if (!WCASECMP(strFmt,L"U16x2"))
			{
			// Source bits
			U16	*piSrc = ((U16 *) pvBits) + (iRow*iW);

			// Copy into new row
			for (U32 c = 0;c < iW;++c)
				pfData[c] = piSrc[c];
			}	// if
		else if (!WCASECMP(strFmt,L"U8x2"))
			{
			// Source bits
			U8	*pcSrc = ((U8 *) pvBits) + (iRow*iW);

			// Copy into new row
			for (U32 c = 0;c < iW;++c)
				pfData[c] = pcSrc[c];
			}	// if
		else if (!WCASECMP(strFmt,L"F32x2"))
			{
			// Source bits
			float *pfBits = ((float *)pvBits) + (iRow*iW);

			// Copy row
			memcpy ( pfData, pfBits, iW*sizeof(float) );
			}	// else if
		
		// No format specified
		else if (strFmt.length() == 0)
			{
			IIt	*pIt	= NULL;
			U32	c		= 0;

			// Iterate and add values from list into vector
			CCLTRY ( pDctData->iterate ( &pIt ) );
			while (hr == S_OK && pIt->read ( vL ) == S_OK && c < iW)
				{
				// Add to array
				pfData[c++] = adtFloat(vL);

				// Clean up
				pIt->next();
				}	// while

			// Clean up
			_RELEASE(pIt);
			}	// else if

		// Unknown
		else
			hr = E_INVALIDARG;

		}	// if

	// Clean up
	_UNLOCK(pDataBits,pvData);
	_UNLOCK(pBits,pvBits);
	_RELEASE(pBits);
	_RELEASE(pDctData);

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
		CCLTRYE ( bReq == true, ERROR_INVALID_STATE );

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
		}	// if

	// Add to plot
	else if (_RCP(Add))
		{
		// State check
		CCLTRYE ( pDataIn != NULL && iIdx >= 0, ERROR_INVALID_STATE );

		// Add column to request
		CCLTRY ( addRow ( pDataIn, iIdx ) );

		// Request is valid with at least 2 rows
		CCLOK ( bReq = (iDataH > 1); )
		}	// else if

	// Reset plot
	else if (_RCP(Reset))
		{
		// Clear vectors of current request
		iDataW = iDataH = 0;
		bReq = false;
//		pReq->remove ( strRefOnImg );
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

		// If current request is valid, update plot
//		if (hr == S_OK && bReq)
//			receive ( prFire, L"", v );
		}	// else if

	// State
	else if (_RCP(Index))
		iIdx = v;
	else if (_RCP(Data))
		{
		_RELEASE(pDataIn);
		pDataIn = adtIUnknown(v);
		_ADDREF(pDataIn);
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
