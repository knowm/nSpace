////////////////////////////////////////////////////////////////////////
//
//									Persis.CPP
//
//					Implementation of the socket persistence node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"
#include <stdio.h>

// Definitions

PersistSkt :: PersistSkt ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pSkt		= NULL;
	pPrs		= NULL;
	skt		= INVALID_SOCKET;
	pStmSv	= NULL;
	pStmLd	= NULL;
	pStmPer	= NULL;
	memset ( szBfrZ, 0, sizeof(szBfrZ) );
	}	// PersistSkt

HRESULT PersistSkt :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pDct	= NULL;
	IResource	*pRes	= NULL;

	// Persistence stream object
	CCLTRYE ( (pStmPer = new PersistSktStm ( this )) != NULL, E_OUTOFMEMORY );
	_ADDREF(pStmPer);

	// Create byte caches
	CCLTRY ( COCREATE ( L"Io.ByteCache", IID_IByteStream, &pStmSv ) );
	CCLTRY ( COCREATE ( L"Io.ByteCache", IID_IByteStream, &pStmLd ) );

	// 'Open' the byte caches on persistence stream
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
	CCLTRY ( pDct->store ( adtString(L"Stream"),		adtIUnknown((IByteStream *)pStmPer) ) );
	CCLTRY ( pDct->store ( adtString(L"Size"),		adtInt(SIZE_PERSIST_CACHE) ) );

	// Input stream
	CCLTRY ( _QI(pStmLd,IID_IResource,&pRes) );
	CCLTRY ( pDct->store ( adtString(L"ReadOnly"),	adtBool(true) ) );
	CCLTRY ( pRes->open ( pDct ) );
	_RELEASE(pRes);

	// Output stream
	CCLTRY ( _QI(pStmSv,IID_IResource,&pRes) );
	CCLTRY ( pDct->store ( adtString(L"ReadOnly"),	adtBool(false) ) );
	CCLTRY ( pRes->open ( pDct ) );
	_RELEASE(pRes);

	// Clean up
	_RELEASE(pDct);

	return hr;
	}	// construct

void PersistSkt :: destruct ( void )
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
	_RELEASE(pSkt);
	_RELEASE(pPrs);
	_RELEASE(pStmSv);
	_RELEASE(pStmLd);
	}	// destruct

HRESULT PersistSkt :: onAttach ( bool bAttach )
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

	// Clean up
	if (!bAttach)
		{
		_RELEASE(pSkt);
		_RELEASE(pPrs);
		adtValue::clear(vSave);
		}	// if

	return S_OK;
	}	// onAttach

HRESULT PersistSkt :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Save
	if (_RCP(Save))
		{
		adtValue	vSkt;
		adtLong	lSkt;

		// Value to use
		const ADTVALUE	*pUseV	= (!adtValue::empty(vSave)) ? &vSave : &v;

		// State check
		CCLTRYE	( pSkt != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )
		CCLTRYE	( pPrs != NULL, ERROR_INVALID_STATE );
		CCLTRYE	( !adtValue::empty(*pUseV), ERROR_INVALID_STATE );

		// Save value to stream
//		dbgprintf ( L"PersistSkt::Save:hr 0x%x {\r\n", hr );
		CCLTRY ( pPrs->save ( pStmSv, *pUseV ) );

		// Flush remaining data
		CCLOK ( pStmSv->flush(); )

		// Result
//		dbgprintf ( L"} PersistSkt::Save:hr 0x%x\r\n", hr );
		if (hr == S_OK)
			_EMT(Save,adtIUnknown(*pUseV) );
		else
			{
			dbgprintf ( L"PersistSkt::receive:Save:hr 0x%x\r\n", hr );
			_EMT(Error,adtInt(hr) );
			}	// else
		}	// if

	// Load
	else if (_RCP(Load))
		{
		adtValue	vSkt;
		adtLong	lSkt;
		adtValue	vL;
		U64		avail;

		// State check
		CCLTRYE	( pSkt != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )
		CCLTRYE	( pPrs != NULL, ERROR_INVALID_STATE );

		// Flush remaining data
//		dbgprintf ( L"PersistSkt::Load:hr 0x%x {\r\n", hr );
		CCLOK ( pStmLd->flush(); )

		// Keep loading values until cache is empty, this is necessary since data will
		// arrive in different size blocks.  This will ensure loop ends on a value boundary.
		while (hr == S_OK && pStmLd->available ( &avail ) == S_OK && avail > 0)
			{
			// Load value from stream
			CCLTRY ( pPrs->load ( pStmLd, vL ) );
//			dbgprintf ( L"PersistSkt::Load:hr 0x%x\r\n", hr );

			// Result
			if (hr == S_OK)
				_EMT(Load,vL);
			else
				{
				dbgprintf ( L"PersistSkt::receive:Load:hr 0x%x\r\n", hr );
				_EMT(Error,adtInt(hr) );
				}	// else
			}	// while

//		dbgprintf ( L"} PersistSkt::Load:hr 0x%x\r\n", hr );
		}	// else if

	// State
	else if (_RCP(Socket))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pSkt);
		hr = _QISAFE(unkV,IID_IDictionary,&pSkt);
		}	// else if
	else if (_RCP(Parser))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pPrs);
		hr = _QISAFE(unkV,IID_IStreamPersist,&pPrs);
		}	// else if
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vSave );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

/////////////////
// PersistStkStm
/////////////////

PersistSktStm :: PersistSktStm ( PersistSkt *_pSkt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_pSkt is the parent socket object
	//
	////////////////////////////////////////////////////////////////////////
	pSkt	= _pSkt;
	}	// PersistSktStm

HRESULT PersistSktStm :: available ( U64 *puAv )
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
	HRESULT	hr					= S_OK;
	struct	timeval	to		= { 0, 0 };
	int		ret				= 0;
	fd_set	rfds;

	// Initialize descriptor
	CCLOK ( FD_ZERO ( &rfds ); )
	CCLOK ( FD_SET ( pSkt->skt, &rfds ); )

	// Socket readable ?
	CCLTRYE	( (ret = select ( (int)pSkt->skt+1, &rfds, NULL, NULL, &to )) 
					!= SOCKET_ERROR, WSAGetLastError() );
	CCLTRYE	( (ret != 0), ERROR_TIMEOUT );
	CCLTRYE	( FD_ISSET ( pSkt->skt, &rfds ), ERROR_TIMEOUT );

	// If socket is readbale, assume at least one byte is available.
	(*puAv) = (hr == S_OK) ? 1 : 0;

	return hr;
	}	// available

HRESULT PersistSktStm :: copyTo ( IByteStream *pStmDst, U64 uSz, U64 *puSz )
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
	HRESULT	hr 	= S_OK;
	U8			*fp	= NULL;
	U64		nleft	= 0;
	U8			cpybufr[4096];
	U64		nio,nw,nr;

	// State check
	CCLTRYE ( pSkt->skt != INVALID_SOCKET, ERROR_INVALID_STATE );
	CCLTRYE ( uSz != 0, ERROR_INVALID_STATE );
 
	// Setup
	if (puSz != NULL)
		*puSz	= 0;

	// Read/write file
	while (hr == S_OK && uSz)
		{
		// Read next block
		CCLOK ( nio = (sizeof(cpybufr) < uSz) ? sizeof(cpybufr) : uSz; )
		CCLTRY( read ( cpybufr, nio, &nr ) );

		// Write full block to stream
		CCLOK ( fp		= cpybufr; )
		CCLOK ( nleft	= nr; )
		while (hr == S_OK && nleft)
			{
			// Write next block
			CCLTRY ( pStmDst->write ( fp, nleft, &nw ) );

			// Next block
			CCLOK ( nleft -= nw; )
			CCLOK ( fp += nw; )
			}	// while

		// Next block
		CCLOK ( uSz -= nr; )
		if (hr == S_OK && puSz != NULL)
			*puSz += nr;

		// If at end of file before request has been satisfied, stop
		if (uSz && (nr < nio))
			break;
		}	// while

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"PersistSktStm::copyTo:Failed:0x%x\r\n", hr );

	return hr;
	}	// copyTo

HRESULT PersistSktStm :: flush ( void )
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

	// Nothing to do for socket

	return hr;
	}	// flush

HRESULT PersistSktStm :: read ( void *pvBfr, U64 nio, U64 *pnio )
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
	HRESULT	hr		= S_OK;
	U32		nr;

	// State check
	CCLTRYE ( (pSkt->skt != INVALID_SOCKET), ERROR_INVALID_STATE );

	// Read next block of data
	CCLTRY ( NetSkt_Receive ( pSkt->skt, pvBfr, (U32)nio, &nr, 10000 ) );

	// Results
	if (hr == S_OK && pnio != NULL)
		*pnio = nr;

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"PersistSktStm::read:Failed:0x%x\r\n", hr );

	return hr;
	}	// read

HRESULT PersistSktStm :: seek ( S64 sPos, U32 uFrom, U64 *puPos )
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
	// Not supported on a socket
	return E_NOTIMPL;
	}	// seek

HRESULT PersistSktStm :: setSize ( U64 uSz )
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
	// A socket can be thought of has having any amount of data
	return S_OK;
	}	// setSize

HRESULT PersistSktStm :: write ( void const *pcvBfr, U64 nio, U64 *pnio )
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
	HRESULT		hr		= S_OK;
	const char	*pc	= (const char *)pcvBfr;
	U32			nleft,nw;

	// State check
	CCLTRYE ( (pSkt->skt != INVALID_SOCKET), ERROR_INVALID_STATE );

	// Write all the data
	CCLOK ( nleft = (U32)nio; )
	while (hr == S_OK && nleft > 0)
		{
		// Next write
		CCLTRY ( NetSkt_Send ( pSkt->skt, pc, nleft, &nw, 10000 ) );
//		dbgprintf ( L"PersistSktStm::write:nleft %d:nw %d\r\n", nleft, nw );

		// Result
		CCLOK ( nleft	-= nw; )
		CCLOK ( pc		+= nw; )
		}	// while

	// Result
	if (hr == S_OK && pnio != NULL)
		*pnio = nio;

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"PersistSktStm::write:Failed:0x%x\r\n", hr );

	return hr;
	}	// write

