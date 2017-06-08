////////////////////////////////////////////////////////////////////////
//
//									STMSTG.CPP
//
//						Compound document stream
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"

//#ifdef	_WIN32
StmStg :: StmStg ( IStream *_pStm )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pStm is the open compond document stream
	//
	////////////////////////////////////////////////////////////////////////
	pStm		= _pStm;
	_ADDREF(pStm);
	}	// StmStg

HRESULT StmStg :: available ( U64 *puAv )
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
	HRESULT	hr = S_OK;
	U64		pos;

	// Use seek to get remaining size
	CCLTRY ( seek ( 0, STREAM_SEEK_CUR, &pos ) );
	CCLTRY ( seek ( 0, STREAM_SEEK_END, puAv ) );
	CCLTRY ( seek ( pos, STREAM_SEEK_SET, NULL ) );

	// Compute remaining size from current position
	CCLOK ( (*puAv) -= pos; )

	return hr;
	}	// available

HRESULT StmStg :: close ( void )
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
	_RELEASE(pStm);
	return S_OK;
	}	// close

HRESULT StmStg :: copyTo ( IByteStream *pStmDst, U64 uSz, U64 *puSz )
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
	U8			filebufr[4096];
	U64		nio,nw;
	ULONG		unio;

	// Setup
	if (puSz != NULL) *puSz = 0;

	// If size is not specified, assume entire stream is to be copied
	if (hr == S_OK && uSz == 0)
		{
		STATSTG	stat;

		// Obtain status of stream
		CCLTRY ( pStm->Stat ( &stat, STATFLAG_NONAME ) );

		// Size
		CCLOK ( uSz = stat.cbSize.LowPart; )
		}	// if

	// Read/write file
	while (hr == S_OK && uSz)
		{
		// Read next block
		CCLOK ( nio = (sizeof(filebufr) < uSz) ? sizeof(filebufr) : uSz; )
		CCLTRY( pStm->Read ( filebufr, (ULONG)nio, &unio ) );

		// End of stream ?
		if (hr == S_OK && unio == 0)
			break;

		// Write full block to stream
		CCLOK ( fp		= filebufr; )
		CCLOK ( nleft	= unio; )
		while (hr == S_OK && nleft)
			{
			// Write next block
			CCLTRY ( pStmDst->write ( fp, nleft, &nw ) );

			// Next block
			CCLOK ( nleft -= nw; )
			CCLOK ( fp += nw; )
			}	// while

		// Next block
		CCLOK ( uSz -= unio; )
		}	// while

	return hr;
	}	// copyTo

void StmStg :: destruct ( void )
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
//	dbgprintf ( L"StmStg::destruct:%p\r\n", this );
	}	// destruct

HRESULT StmStg :: flush ( void )
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
	return S_OK;
	}	// flush

HRESULT StmStg :: getResId ( ADTVALUE &vId )
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

	// Setup
	adtValue::clear ( vId );

	// Handle/descriptor
	if (pStm != NULL)
		hr = adtValue::copy ( adtIUnknown(pStm), vId );
	
	return hr;
	}	// getResId

HRESULT StmStg :: open ( IDictionary *pOpts )
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
	return E_NOTIMPL;
	}	// open

HRESULT StmStg :: read ( void *pvBfr, U64 nio, U64 *pnio )
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
	HRESULT	hr	= S_OK;
	ULONG		nr;

	// Check
	CCLTRYE ( (pStm != NULL), E_UNEXPECTED );

	// Execute
	CCLTRY ( pStm->Read ( pvBfr, (ULONG)nio, &nr ) );

	// Result
	if (hr == S_OK && pnio != NULL) *pnio = nr;

	return hr;
	}	// read

HRESULT StmStg :: seek ( S64 sPos, U32 uFrom, U64 *puPos )
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
	HRESULT			hr	= S_OK;
	LARGE_INTEGER	liMove;
	ULARGE_INTEGER	liPos;

	// Check
	CCLTRYE ( (pStm != NULL), E_UNEXPECTED );

	// Execute
	CCLOK ( liMove.QuadPart = sPos; )
	CCLTRY ( pStm->Seek ( liMove, uFrom, &liPos ) );

	// Result
	if (hr == S_OK && puPos != NULL) *puPos = liPos.QuadPart;

	return hr;
	}	// seek

HRESULT StmStg :: setSize ( U64 uSz )
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
	HRESULT			hr		= S_OK;
	ULARGE_INTEGER	liSz;

	// Check
	CCLTRYE ( (pStm != NULL), E_UNEXPECTED );

	// Execute
	CCLOK ( liSz.QuadPart = uSz; )
	CCLTRY ( pStm->SetSize ( liSz ) );

	return hr;
	}	// setSize

HRESULT StmStg :: write ( void const *pcvBfr, U64 nio, U64 *pnio )
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
	ULONG		nw;

	// Check
	CCLTRYE ( (pStm != NULL), E_UNEXPECTED );

	// Execute
	CCLTRY ( pStm->Write ( pcvBfr, (ULONG)nio, &nw ) );

	// Result
	if (hr == S_OK && pnio != NULL) *pnio = nw;

	return hr;
	}	// write

