////////////////////////////////////////////////////////////////////////
//
//									BYTECACH.CPP
//
//						Byte buffer cache for another stream
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"

ByteCache :: ByteCache ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pStm		= NULL;
	pStmC		= NULL;
	nAllocd	= 0;
	nCache	= 0;
	}	// ByteCache

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

HRESULT ByteCache :: available ( U64 *puAv )
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
	HRESULT	hr	= S_OK;
	U64		av;

	// State check
	CCLTRYE ( (pStmC != NULL && pStm != NULL), ERROR_INVALID_STATE );

	// The available size is whatever the underlying stream has plus this cache
	*puAv	= nCache;
	if (hr == S_OK && pStm->available ( &av ) == S_OK)
		(*puAv) += av;

	return hr;
	}	// available

HRESULT ByteCache :: copyTo ( IByteStream *pStmDst, U64 uSz, U64 *puSz )
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

HRESULT ByteCache :: write ( void const *pcvBfr, U64 nio, U64 *pnio )
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
	HRESULT			hr			= S_OK;
	const U8			*pcBfr	= (const U8 *)pcvBfr;
	U64				nleft		= nio;
	U64				nw,nx;

	// State check
	CCLTRYE ( (pStmC != NULL && pStm != NULL && !bRO), ERROR_INVALID_STATE );

	// Setup
	if (pnio != NULL) *pnio = 0;

	// Transfer data to cache until full, the flush to main stream.
	while (hr == S_OK && nleft > 0)
		{
		// Compute amount to write
		CCLOK	 ( nw = ((nAllocd-nCache) < nleft) ? (nAllocd-nCache) : nleft; )

		// Write to cache
		CCLTRY ( pStmC->write ( pcBfr, nw, &nx ) );

		// Next block
		if (hr == S_OK)
			{
			nleft		-= nx;
			nCache	+= (U32)nx;
			pcBfr		+= nx;
			if (pnio != NULL)
				*pnio += nx;
			}	// if

		// Is cache full ?
		if (hr == S_OK && nCache >= nAllocd)
			flush();
		}	// while

	return hr;
	}	// write

