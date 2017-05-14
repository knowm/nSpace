////////////////////////////////////////////////////////////////////////
//
//									DATABLK.CPP
//
//					Implementation of the data block node
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>

// Globals
static adtString strRefWidth	( L"Width" );
static adtString strRefHeight	( L"Height" );
static adtString strRefFormat	( L"Format" );
static adtString strRefBits	( L"Bits" );

DataBlock :: DataBlock ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pBlk		= NULL;
	plsBits	= NULL;
	plsfBits	= NULL;
	pSrc		= NULL;
	iX			= 0;
	iY			= 0;
	}	// DataBlock

HRESULT DataBlock :: addRow ( IDictionary *pDctDst,
										IDictionary *pDctSrc, U32 iRow )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a row of data to a data block.
	//
	//	PARAMETERS
	///	-	pDctDst will receive the row
	//		-	pDctSrc contains the row
	//		-	iRow is the row number to add
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr				= S_OK;
	IMemoryMapped	*pBitsDst	= NULL;
	IMemoryMapped	*pBitsSrc	= NULL;
	void				*pvBitsSrc	= NULL;
	void				*pvBitsDst	= NULL;
	float				*pfDst		= NULL;
	adtInt			iDstW,iSrcW,iDstH,iSrcH;
	adtString		strFmt;
	adtValue			vL;
	adtIUnknown		unkV;

	//
	// Add row from active data object.
	// Be as flexible as possible, add as many different formats 
	//	as necessary over time.
	//

	// Allow for a 'blank' destination to be provided, initialize as necessary
	if (hr == S_OK)
		{
		if (pDctDst->load ( strRefWidth, vL ) == S_OK)
			iDstW = vL;
		if (pDctDst->load ( strRefHeight, vL ) == S_OK)
			iDstH = vL;
		if (pDctDst->load ( strRefFormat, vL ) == S_OK)
			adtValue::toString(vL,strFmt);
		if (WCASECMP(strFmt,L"F32x2"))
			hr = pDctDst->store ( strRefFormat, adtString(L"F32x2") );

		// Data bits 
		if (pDctDst->load ( strRefBits, vL ) == S_OK)
			{
			// Access block
			CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,&pBitsDst) );
			}	// if
		else
			{
			// Create a data block for destination
			CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pBitsDst ) );
			CCLTRY ( pDctDst->store ( adtString(L"Bits"), adtIUnknown(pBitsDst) ) );
			}	// else

		}	// if

	// Format specified for incoming data ?
	if (hr == S_OK && pDctSrc->load ( adtString(L"Format"), vL ) == S_OK)
		{
		// Access incoming data
		CCLTRY ( lock ( pDctSrc, iSrcW, iSrcH, strFmt, &pBitsSrc, &pvBitsSrc ) );
		}	// if

	// No format, for now assume list of values
	else if (hr == S_OK)
		{
		U32	sz;

		// Count is 'width'
		CCLTRY ( pDctSrc->size ( &sz ) );
		CCLOK  ( iSrcW = sz; )
		CCLOK  ( iSrcH = 2; )
		CCLOK  ( strFmt = L""; )
		}	// else if

	// Valid row specified ?
	CCLTRYE ( iRow < iSrcH, E_INVALIDARG );

	// Another row
	CCLOK	( iDstH = iDstH + 1; )
	CCLTRY( pDctDst->store ( strRefHeight, iDstH ) );

	// First vector ?
	if (hr == S_OK && iDstW == 0)
		{
		// First vector determines width
		iDstW = iSrcW;

		// Update data information
		CCLTRY ( pDctDst->store ( strRefWidth, iDstW ) );
		}	// if

	// Vectors must be same length for a block
	CCLTRYE ( iSrcW == iDstW, E_INVALIDARG );

	// Allocate space for another row
	CCLTRY ( pBitsDst->setSize ( iDstW*iDstH*sizeof(float) ) );
	CCLTRY ( pBitsDst->lock ( 0, 0, (void **) &pvBitsDst, NULL ) );
	CCLOK  ( pfDst = (float *)pvBitsDst; )
	CCLOK  ( pfDst += (iDstH-1)*iDstW; )

	// Add data
	if (hr == S_OK)
		{
		// Add more formats as necessary
		if (!WCASECMP(strFmt,L"U16x2"))
			{
			// Source bits
			U16	*piSrc = ((U16 *) pvBitsSrc) + (iRow*iSrcW);

			// Copy into new row
			if (piSrc != NULL)
				for (U32 c = 0;c < iSrcW;++c)
					pfDst[c] = piSrc[c];
			}	// if
		else if (!WCASECMP(strFmt,L"S16x2"))
			{
			// Source bits
			S16	*piSrc = ((S16 *) pvBitsSrc) + (iRow*iSrcW);

			// Copy into new row
			if (piSrc != NULL)
				for (U32 c = 0;c < iSrcW;++c)
					pfDst[c] = piSrc[c];
			}	// if
		else if (!WCASECMP(strFmt,L"U8x2"))
			{
			// Source bits
			U8	*pcSrc = ((U8 *) pvBitsSrc) + (iRow*iSrcW);

			// Copy into new row
			if (pcSrc != NULL)
				for (U32 c = 0;c < iSrcW;++c)
					pfDst[c] = pcSrc[c];
			}	// if
		else if (!WCASECMP(strFmt,L"F32x2"))
			{
			// Source bits
			float *pfBits = ((float *)pvBitsSrc) + (iRow*iSrcW);

			// Copy row
			if (pfBits != NULL)
				memcpy ( pfDst, pfBits, iSrcW*sizeof(float) );
			}	// else if
		
		// No format specified
		else if (strFmt.length() == 0)
			{
			IIt	*pIt	= NULL;
			U32	c		= 0;

			// Iterate and add values from list into vector
			if (hr == S_OK)
				{
				if (iRow == 0) hr = pDctSrc->keys ( &pIt );
				else				hr = pDctSrc->iterate ( &pIt );
				}	// if
	//		CCLTRY ( pDctSrc->iterate ( &pIt ) );
			while (hr == S_OK && pIt->read ( vL ) == S_OK && c < iSrcW)
				{
				// Add to array
				pfDst[c++] = adtFloat(vL);

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
	_UNLOCK(pBitsDst,pvBitsDst);
	_RELEASE(pBitsDst);
	_UNLOCK(pBitsSrc,pvBitsSrc);
	_RELEASE(pBitsSrc);

	return hr;
	}	// addRow

HRESULT DataBlock :: construct ( void )
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

	return hr;
	}	// construct

void DataBlock :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// Clean up
	_RELEASE(pSrc);
	_RELEASE(pBlk);
	}	// destruct

HRESULT DataBlock :: lock ( IUnknown *punkDct, adtInt &iW, adtInt &iH,
										adtString &strFmt, IMemoryMapped **ppBits,
										void **ppvBits )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Access/lock data block information inside a dictionary.
	//
	//	PARAMETERS
	//		-	punkDct is the dictionary containing the information
	//		-	iW,iH will receive the width and height
	//		-	strFmt will receive the format
	//		-	ppBits will receive the interface to the data block.
	//		-	ppvBits will receive a ptr. to the data (if there is any).
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	IDictionary		*pDct = NULL;
	adtIUnknown		unkV(punkDct);
	adtValue			vL;

	// Container
	CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDct) );

	// Parameters
	CCLTRY ( pDct->load ( strRefWidth, vL ) );
	CCLOK  ( iW = vL; )
	CCLTRY ( pDct->load ( strRefHeight, vL ) );
	CCLOK  ( iH = vL; )
	CCLTRY ( pDct->load ( strRefFormat, vL ) );
	CCLTRY ( adtValue::toString ( vL, strFmt ) );

	// Data bits
	if (hr == S_OK && ppBits != NULL)
		{
		// Get bits and lock
		CCLTRY ( pDct->load ( strRefBits, vL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,ppBits) );
		CCLTRY ( (*ppBits)->lock ( 0, 0, ppvBits, NULL ) );

		// If there is an error here, signal in return value
		if (hr != S_OK)
			hr = E_OUTOFMEMORY;
		}	// if

	// Clean up
	_RELEASE(pDct);

	return hr;
	}	// lock

HRESULT DataBlock :: onAttach ( bool bAttach )
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

	// Attach	
	if (bAttach)
		{
		adtValue	vL;

		// Default states
		if (pnDesc->load ( adtStringSt(L"X"), vL ) == S_OK)
			iX = vL;
		if (pnDesc->load ( adtStringSt(L"Y"), vL ) == S_OK)
			iY = vL;
		}	// if
	else
		{
		// Clean up
		_RELEASE(pSrc);
		_UNLOCK(plsBits,plsfBits);
		_RELEASE(plsBits);
		_RELEASE(pBlk);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT DataBlock :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Add row index to data set
	if (_RCP(Add))
		{
		// State check
		CCLTRYE ( pBlk != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pSrc != NULL, ERROR_INVALID_STATE );

		// Debug
//		if (!WCASECMP(strnName,L"RasterBlk"))
//			dbgprintf ( L"Hi\r\n" );

		// Add the specified row
		CCLTRY ( addRow ( pBlk, pSrc, iY ) );

		// Result
		if (hr != S_OK)
			_EMT(Error,adtInt(hr));
		}	// if

	// Load single value
	else if (_RCP(Load))
		{
		adtValue			vL;

		// State check
		CCLTRYE ( pBlk != NULL, ERROR_INVALID_STATE );

		// Access data block
		if (hr == S_OK && plsBits == NULL)
			{
			adtString	strFmt;

			// Obtain info.
			CCLTRY ( lock ( pBlk, ilsW, ilsH, strFmt, &plsBits, (void **) &plsfBits ) );

			// For now only base format supported
			CCLTRYE ( !WCASECMP(strFmt,L"F32x2"), ERROR_INVALID_STATE );
			}	// if

		// Valid lock ?
		CCLTRYE ( plsfBits != NULL, E_UNEXPECTED );

		// Valid range ?
		CCLTRYE ( iX >= 0 && iX < ilsW, E_INVALIDARG );
		CCLTRYE ( iY >= 0 && iY < ilsH, E_INVALIDARG );

		// Access value
		CCLTRY ( adtValue::copy ( adtFloat(plsfBits[iY*ilsW+iX]), vL ) );

		// Result
		if (hr == S_OK)
			_EMT(Load,vL);
		else
			_EMT(Error,adtInt(hr));

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_WARN, L"%s: Unable to load value from %d,%d\r\n", 
							(LPCWSTR)strnName, (S32)iX, (S32)iY );
		}	// if

	// Store single value
	else if (_RCP(Store))
		{
		adtValue			vL;

		// State check
		CCLTRYE ( pBlk != NULL, ERROR_INVALID_STATE );

		// Access data block
		if (hr == S_OK && plsBits == NULL)
			{
			adtString	strFmt;

			// Obtain info.
			CCLTRY ( lock ( pBlk, ilsW, ilsH, strFmt, &plsBits, (void **) &plsfBits ) );

			// For now only base format supported
			CCLTRYE ( !WCASECMP(strFmt,L"F32x2"), ERROR_INVALID_STATE );
			}	// if

		// Valid lock ?
		CCLTRYE ( plsfBits != NULL, E_UNEXPECTED );

		// Valid range ?
		CCLTRYE ( iX >= 0 && iX < ilsW, E_INVALIDARG );
		CCLTRYE ( iY >= 0 && iY < ilsH, E_INVALIDARG );

		// Store
		CCLOK ( plsfBits[iY*ilsW+iX] = adtFloat(vValue); )

		// Result
		if (hr == S_OK)
			_EMT(Store,vL);
		else
			_EMT(Error,adtInt(hr));

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_WARN, L"%s: Unable to store value to %d,%d\r\n", 
							(LPCWSTR)strnName, (S32)iX, (S32)iY );
		}	// if

	// Set size
	else if (_RCP(Set))
		{
		IMemoryMapped	*pBits = NULL;

		// State check
		CCLTRYE ( pBlk != NULL && iX > 0 && iY > 0, ERROR_INVALID_STATE );

		// Allocate the memory first to ensure success
		CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pBits ) );
		CCLTRY ( pBits->setSize ( iX*iY*sizeof(float) ) );
		CCLTRY ( pBlk->store ( adtString(L"Bits"), adtIUnknown(pBits) ) );

		// Initialize dictionary
		CCLTRY ( pBlk->store ( strRefWidth, iX ) );
		CCLTRY ( pBlk->store ( strRefHeight, iY ) );
		CCLTRY ( pBlk->store ( strRefFormat, adtString(L"F32x2") ) );

		// Result
		if (hr == S_OK)
			_EMT(Set,adtIUnknown(pBlk));
		else
			_EMT(Error,adtInt(hr));

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_WARN, L"%s: Unable to set size to %d,%d\r\n", 
							(LPCWSTR)strnName, (S32)iX, (S32)iY );
		}	// else if

	// Reset state
	else if (_RCP(Reset))
		{
		// State check
		CCLTRYE ( pBlk != NULL, ERROR_INVALID_STATE );

		// Clear internal information
		CCLTRY ( pBlk->store ( strRefWidth, adtInt(0) ) );
		CCLTRY ( pBlk->store ( strRefHeight, adtInt(0) ) );
		}	// else if

	// State
	else if (_RCP(Block))
		{
		adtIUnknown unkV(v);
		_RELEASE(pBlk);
		_UNLOCK(plsBits,plsfBits);
		_RELEASE(plsBits);
		_QISAFE(unkV,IID_IDictionary,&pBlk);
		}	// else if
	else if (_RCP(Source))
		{
		adtIUnknown unkV(v);
		_RELEASE(pSrc);
		_QISAFE(unkV,IID_IDictionary,&pSrc);
		}	// else if
	else if (_RCP(X))
		iX = v;
	else if (_RCP(Y))
		iY = v;
	else if (_RCP(Value))
		adtValue::copy ( v, vValue );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

