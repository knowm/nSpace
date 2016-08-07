////////////////////////////////////////////////////////////////////////
//
//									NSPACE.CPP
//
//				nSpace side of Apache interface module
//
////////////////////////////////////////////////////////////////////////

#include "nspace.h"

// Globals
nSpaceLink	*pLink = NULL;

nSpaceLink :: nSpaceLink ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pThrd = NULL;
	pTick	= NULL;
	pnCli = NULL;
	pPer	= NULL;
	AddRef();
	}	// nSpaceLink

HRESULT nSpaceLink :: available ( U64 *puAv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Retrieve the number of bytes available for reading.
	//
	//	PARAMETERS
	//		-	puAv will receive the available bytes
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Stream used for saving so none available
	*puAv = 0;

	return S_OK;
	}	// available

HRESULT nSpaceLink :: construct ( void )
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

	// Worker object
	CCLTRYE ( (pTick = new nSpaceLink_t()) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pTick->pThis = this; )

	// Start worker thread
	CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd));
	CCLTRY(pThrd->threadStart(pTick, 10000));

	// Parser for values
	CCLTRY(COCREATE(L"Io.StmPrsXML", IID_IStreamPersist, &pPer));

	return hr;
	}	// construct

HRESULT nSpaceLink :: copyTo ( IByteStream *pStmDst, U64 uSz, U64 *puSz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Copies the specified # of bytes to another stream.
	//
	//	PARAMETERS
	//		-	pStmDst is the target stream
	//		-	uSz is the amount to copy
	//		-	puSz is the amount copied
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return E_NOTIMPL;
	}	// copyTo

void nSpaceLink :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	if (pThrd != NULL)
		{
		pThrd->threadStop(5000);
		_RELEASE(pThrd);
		}	// if
	_RELEASE(pTick);
	_RELEASE(pPer);
	}	// destruct

HRESULT nSpaceLink :: flush ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Flush the stream state.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Load only stream
	return S_OK;
	}	// flush

HRESULT nSpaceLink :: load ( const WCHAR *pwLoc, request_rec *_req )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Load a namespace value into the request 
	//
	//	PARAMETERS
	//		-	pwLoc is the namespace location
	//		-	_req is the active request
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtValue		vL;

	// Thread safety
	csVl.enter();

	// Active request
	req = _req;

	// Access the value
	CCLTRY ( pnCli->load ( pwLoc, vL ) );

	// Persist value to stream
	CCLTRY ( pPer->save ( this, vL ) );

	// Clean up
	csVl.leave();

	return hr;
	}	// load

HRESULT nSpaceLink :: onReceive (	const WCHAR *pwRoot, 
												const WCHAR *pwLoc,
												const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		nSpaceClientCB
	//
	//	PURPOSE
	//		-	Called when a listened location receives a value.
	//
	//	PARAMETERS
	//		-	pwRoot is the path to the listened location
	//		-	pwLoc is the location relative to the root for the value
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	lprintf ( LOG_INFO, L"%s:%s", pwRoot, pwLoc );
	return S_OK;
	}	// onReceive

HRESULT nSpaceLink :: read ( void *pvBfr, U64 nio, U64 *pnio )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Reads the specified # of bytes from the stream.
	//
	//	PARAMETERS
	//		-	pvBfr will receive the data
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// read

HRESULT nSpaceLink :: seek ( S64 sPos, U32 uFrom, U64 *puPos )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Places the current byte position at the specified location.
	//
	//	PARAMETERS
	//		-	sPos is the new position
	//		-	uFrom specified where to start seek from
	//		-	puPos will receive the new position
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_FALSE;
	}	// seek

HRESULT nSpaceLink :: setSize ( U64 uSz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Sets the size of the stream.
	//
	//	PARAMETERS
	//		-	uSz is the new size
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_FALSE;
	}	// setSize

HRESULT nSpaceLink :: store ( const WCHAR *pwLoc, IByteStream *pStm )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Load a value from a stream and store into the namespace.
	//
	//	PARAMETERS
	//		-	pwLoc is the namespace location
	//		-	pStm contains the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtValue		vL;

	// Thread safety
	csVl.enter();

	// Load the value from the stream
	CCLTRY ( pPer->load ( pStm, vL ) );

	// Store into namespace
	CCLTRY ( pnCli->store ( pwLoc, vL ) );

	// Clean up
	csVl.leave();

	// Debug
	lprintf(LOG_INFO, L"hr 0x%x", hr);

	return hr;
	}	// store

//
// nSpaceLink_t
//

nSpaceLink_t :: nSpaceLink_t ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pThis		= NULL;
	bWork		= true;

	// Addref self
	AddRef();
	}	// nSpaceLink_t

HRESULT nSpaceLink_t :: tickAbort ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' should abort.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Shtudown
	bWork = false;

	return S_OK;
	}	// tickAbort

HRESULT nSpaceLink_t :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Perform one 'tick's worth of work.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Clean up
	Sleep(1000);

	// Continue ?
	CCLTRYE ( bWork == true, S_FALSE );

	return hr;
	}	// tick

HRESULT nSpaceLink_t:: tickBegin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that it should get ready to 'tick'.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Initialize COM
	CCLTRY ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) );

	// Create nSpace connection
	CCLTRYE ( (pThis->pnCli = new nSpaceClient()) != NULL, E_OUTOFMEMORY );
	CCLTRY (pThis->pnCli->open ( L"{ Namespace Apache }", TRUE ) );

	return hr;
	}	// tickBegin

HRESULT nSpaceLink_t :: tickEnd ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' is to stop.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	lprintf ( LOG_INFO, L"End" );

	// Clean up
	if (pThis->pnCli != NULL)
		{
		pThis->pnCli->close();
		delete pThis->pnCli;
		pThis->pnCli = NULL;
		}	// if

	// Uninitialize COM
	CoUninitialize();

	return S_OK;
	}	// tickEnd

HRESULT nSpaceLink :: write ( void const *pcvBfr, U64 nio, U64 *pnio )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Writes the specified # of bytes to the stream.
	//
	//	PARAMETERS
	//		-	pvBfr contains the data to write
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Write to response stream
	CCLOK ( ap_rwrite ( pcvBfr, (int)nio, req ); )

	// Result
	CCLOK ( *pnio = nio; )

	return hr;
	}	// write

//
// nSpaceLink
//

bool nspace_init ( bool bInit )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initialize/uninitialize nSpace link.
	//
	//	PARAMETERS
	//		-	bInit is true to initialize, false to uninitialize.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Debug
	lprintf(LOG_INFO, L"nspace_init %d\r\n", bInit);

	// Initialize
	if (bInit)
		{
		// Initialize COM if it is not already initialized
		CCLTRY ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) );

		// State check
		CCLTRYE ( (pLink == NULL), ERROR_INVALID_STATE );

		// Create the link object
		CCLTRYE ( (pLink = new nSpaceLink()) != NULL, E_OUTOFMEMORY );
		CCLTRY  ( pLink->construct() );

		// Clean up
		if (hr != S_OK)
			{
			_RELEASE(pLink);
			}	// if
		}	// if

	// Uninitialize
	else
		{
		// Shutdown
		_RELEASE(pLink);

		// Clean up
		CoUninitialize();
		}	// else

	return (hr == S_OK);
	}	// nspace_init

/*
HRESULT ByteCache :: close ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IResource
	//
	//	PURPOSE
	//		-	Close the resource.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	flush();
	_RELEASE(pStm);
	_RELEASE(pStmC);
	return S_OK;
	}	// close



void ByteCache :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	close();
//	dbgprintf ( L"ByteCache::destruct:%p\r\n", this );
	}	// destruct

HRESULT ByteCache :: flush ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Flush the stream state.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// State check
	CCLTRYE ( (pStmC != NULL && pStm != NULL), ERROR_INVALID_STATE );

	// Write
	if (hr == S_OK && !bRO && nCache > 0)
		{
		// Reset position of cache
		CCLTRY ( pStmC->seek ( 0, STREAM_SEEK_SET, NULL ) );

		// Copy valid portion of cache to destination
		CCLTRY ( pStmC->copyTo ( pStm, nCache, NULL ) );
		}	// else if

	// Cache empty
	CCLTRY ( pStmC->seek ( 0, STREAM_SEEK_SET, NULL ) );
	CCLOK  ( nCache = 0; )

	return hr;
	}	// flush

HRESULT ByteCache :: getResId ( ADTVALUE &vId )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IResource
	//
	//	PURPOSE
	//		-	Return an identifier for the resource.
	//
	//	PARAMETERS
	//		-	vId will receive the identifer.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;

	// Underlying stream
	CCLTRY ( adtValue::copy ( adtIUnknown(pStm), vId ) );
	
	return hr;
	}	// getResId

HRESULT ByteCache :: open ( IDictionary *pOpts )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IResource
	//
	//	PURPOSE
	//		-	Open a byte stream on top of a file.
	//
	//	PARAMETERS
	//		-	pOpts contain options for the file.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	adtValue		v;
	adtIUnknown	unkV;

	// Stream and size of cache (required)
	CCLTRY ( pOpts->load ( adtString(L"Stream"), v ) );
	CCLTRY ( _QISAFE((unkV=v),IID_IByteStream,&pStm) );
	CCLTRY ( pOpts->load ( adtString(L"Size"), v ) );
	CCLTRYE( (nAllocd = adtInt(v)) > 0, E_INVALIDARG );

	// Read only (or write)
	CCLTRY ( pOpts->load ( adtString(L"ReadOnly"), v ) );
	CCLOK  ( bRO = adtBool(v); )

	// Create an internal memory based stream for the cache
	CCLTRY ( COCREATE ( L"Io.StmMemory", IID_IByteStream, &pStmC ) );
	CCLTRY ( pStmC->setSize ( nAllocd ) );

	// Cache is immediately invalid
	CCLOK ( flush(); )

	return hr;
	}	// open

HRESULT ByteCache :: read ( void *pvBfr, U64 nio, U64 *pnio )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Reads the specified # of bytes from the stream.
	//
	//	PARAMETERS
	//		-	pvBfr will receive the data
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	U8					*pcBfr	= (U8 *)pvBfr;
	U64				nleft		= nio;
	U64				nr,nx;

	// State check
	CCLTRYE ( (pStmC != NULL && pStm != NULL && bRO), ERROR_INVALID_STATE );

	// Setup
	if (pnio != NULL) *pnio = 0;

	// Transfer data from the cache and underlying stream into provided buffer.
	while (hr == S_OK && nleft > 0)
		{
		// Amount to read from cache
		nr = (nCache < nleft) ? nCache : nleft;

		// Read from the cache
		if (nr > 0 && pStmC->read ( pcBfr, nr, &nx ) == S_OK)
			{
			// Cache read successful
			nleft		-= nx;
			nCache	-= (U32)nx;
			pcBfr		+= nx;
			if (pnio != NULL)
				*pnio += nx;
			}	// if

		// Cache empty, read from underlying stream
		else 
			{
			// Reset position of memory buffer
			CCLTRY ( pStmC->seek ( 0, STREAM_SEEK_SET, NULL ) );

			// Read buffer amount from stream
			CCLTRY ( pStm->copyTo ( pStmC, nAllocd, &nx ) );

			// Reset position of memory buffer
			CCLOK  ( nCache = (U32)nx; )
			CCLTRY ( pStmC->seek ( 0, STREAM_SEEK_SET, NULL ) );
			}	// else
		}	// while

	return hr;
	}	// read

HRESULT ByteCache :: seek ( S64 sPos, U32 uFrom, U64 *puPos )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Places the current byte position at the specified location.
	//
	//	PARAMETERS
	//		-	sPos is the new position
	//		-	uFrom specified where to start seek from
	//		-	puPos will receive the new position
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// State check
	CCLTRYE ( (pStmC != NULL && pStm != NULL), ERROR_INVALID_STATE );

	// Seek in cache
	CCLTRY ( pStmC->seek ( sPos, uFrom, puPos ) );

	return hr;
	}	// seek

HRESULT ByteCache :: setSize ( U64 uSz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Sets the size of the stream.
	//
	//	PARAMETERS
	//		-	uSz is the new size
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;

	// State check
	CCLTRYE ( (pStmC != NULL && pStm != NULL && !bRO), ERROR_INVALID_STATE );

	// Invalidates the cache
	CCLTRY ( pStmC->seek ( 0, STREAM_SEEK_END, NULL ) );

	// Underlying stream
	CCLTRY ( pStm->setSize ( uSz ) );

	return hr;
	}	// setSize


*/