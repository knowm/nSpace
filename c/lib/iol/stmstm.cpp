////////////////////////////////////////////////////////////////////////
//
//										STMSTM.CPP
//
//			Implementation of the 'IStream' on 'IByteStream' object.
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

StmOnByteStm :: StmOnByteStm ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pStm	= NULL;
	}	// StmOnByteStm

void StmOnByteStm :: destruct ( void )
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
	_RELEASE(pStm);
	}	// destruct

HRESULT StmOnByteStm :: getValue ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IHaveValue
	//
	//	PURPOSE
	//		-	Returns the value for the object.
	//
	//	PARAMETERS
	//		-	ppV will receive the value.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return adtValue::copy ( adtIUnknown(pStm), v );
	}	// getValue

HRESULT StmOnByteStm :: Read ( void *pvBfr, ULONG nr, ULONG *pnr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISequentialStream
	//
	//	PURPOSE
	//		-	Reads the specified # of bytes from the stream.
	//
	//	PARAMETERS
	//		-	pvBfr will receive the data
	//		-	nr is the # of bytes to transfer
	//		-	pnr is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	U64		nrd	= 0;

	// State check
	CCLTRYE	( pStm != NULL, E_UNEXPECTED );
	if (hr == S_OK)
		{
		// Perform read
		CCLTRY ( pStm->read ( pvBfr, nr, &nrd ) );

		// ISequentialStream protocol
		if (hr == S_OK && nrd < nr)
			hr = S_FALSE;
		}	// if

	// Result
	if (pnr != NULL) *pnr = (ULONG)nrd;

	return hr;
	}	// Read

HRESULT StmOnByteStm :: Seek ( LARGE_INTEGER dlibMove, 
											DWORD dwOrigin, ULARGE_INTEGER *plibNew )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IStream
	//
	//	PURPOSE
	//		-	Seeks to new location
	//
	//	PARAMETERS
	//		-	dlibMove is new location request
	//		-	dwOrigin specifies origin
	//		-	plibNew is the new location
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	U64		pos;

	// Seek
	CCLTRYE ( pStm != NULL, E_UNEXPECTED );
	CCLTRY  ( pStm->seek ( dlibMove.LowPart, dwOrigin, &pos ) );
	if (hr == S_OK && plibNew != NULL)
		plibNew->QuadPart = pos;

	return hr;
	}	// Seek

HRESULT StmOnByteStm :: setValue ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IHaveValue
	//
	//	PURPOSE
	//		-	Sets the value for the object.
	//
	//	PARAMETERS
	//		-	pV contains the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtIUnknown	unkV(v);

	// Previous stream
	_RELEASE(pStm);

	// New stream
	CCLTRY(_QISAFE(unkV,IID_IByteStream,&pStm));

	return hr;
	}	// setValue

HRESULT StmOnByteStm :: Stat ( STATSTG *pstat, DWORD flag )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IStream
	//
	//	PURPOSE
	//		-	Retrieves information about the stream.
	//
	//	PARAMETERS
	//		-	pstat will receive the information
	//		-	flag controls which fields are returned
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;
	U64		opos,sz;

	// Default name
	if (hr == S_OK && !(flag & STATFLAG_NONAME))
		{
		// Allocate memory and fill
		CCLTRYE ( (pstat->pwcsName = (LPOLESTR) CoTaskMemAlloc (
			(wcslen(L"ByteStream")+1)*sizeof(WCHAR) )) != NULL, E_OUTOFMEMORY );
		CCLOK   ( WCSCPY ( pstat->pwcsName, 10, L"ByteStream" ); )
		}	// if

	// Size of specified stream
	CCLTRY ( pStm->seek ( 0, STREAM_SEEK_CUR, &opos ) );
	CCLTRY ( pStm->seek ( 0, STREAM_SEEK_END, &sz ) );
	CCLTRY ( pStm->seek ( opos, STREAM_SEEK_SET, NULL ) );
	CCLOK  ( pstat->cbSize.QuadPart	= sz; )

	// Other info
	CCLOK  ( pstat->type = STGTY_STREAM; )

	return hr;
	}	// Stat

HRESULT StmOnByteStm :: Write ( void const *pcvBfr, ULONG nw, ULONG *pnw )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISequentialStream
	//
	//	PURPOSE
	//		-	Writes the specified # of bytes from the stream.
	//
	//	PARAMETERS
	//		-	pvBfr contains the data
	//		-	nw is the # of bytes to transfer
	//		-	pnw is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	U64		nwrt	= 0;

	// Perform write
	CCLTRYE	( pStm != NULL, E_UNEXPECTED );
	CCLTRY	( pStm->write ( pcvBfr, nw, &nwrt ) );

	// Result
	if (pnw != NULL) *pnw = (ULONG)nwrt;

	return hr;
	}	// Write

//
// Unimplemented functions
//

HRESULT StmOnByteStm :: Clone ( IStream ** )
	{
	return E_NOTIMPL;
	}	// Clone

HRESULT StmOnByteStm :: Commit ( DWORD )
	{
	return E_NOTIMPL;
	}	// Commit

HRESULT StmOnByteStm :: CopyTo ( IStream *, ULARGE_INTEGER, ULARGE_INTEGER *, ULARGE_INTEGER * )
	{
	return E_NOTIMPL;
	}	// CopyTo

HRESULT StmOnByteStm :: LockRegion ( ULARGE_INTEGER, ULARGE_INTEGER, DWORD )
	{
	return E_NOTIMPL;
	}	// LockRegion

HRESULT StmOnByteStm :: Revert ( void )
	{
	return E_NOTIMPL;
	}	// Revert

HRESULT StmOnByteStm :: SetSize ( ULARGE_INTEGER )
	{
	return E_NOTIMPL;
	}	// SetSize

HRESULT StmOnByteStm :: UnlockRegion ( ULARGE_INTEGER, ULARGE_INTEGER, DWORD )
	{
	return E_NOTIMPL;
	}	// UnlockRegion

