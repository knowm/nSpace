////////////////////////////////////////////////////////////////////////
//
//									STMZLIB.CPP
//
//						ZLib compression/decompression based stream 
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include "../ext/zlib/zlib.h"

// Definitions
#undef	SIZE_BUFFER
#define	SIZE_BUFFER	0x10000

// Local functions
static voidpf	zalloc	( voidpf, uInt, uInt );
static void		zfree		( voidpf, voidpf );

StmZlib :: StmZlib ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pStm			= NULL;
	pMemWr		= NULL;
	pvMemWr		= NULL;
	pMemRd		= NULL;
	pvMemRd		= NULL;
	bReadOnly	= true;
	pvstm			= NULL;
	}	// StmZlib

HRESULT StmZlib :: available ( U64 *puAv )
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

	// TODO
	lprintf ( LOG_ERR, L"Not implemented\r\n" );

	return E_NOTIMPL;
	}	// available

HRESULT StmZlib :: close ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IResource
	//
	//	PURPOSE
	//		-	Closes the stream with the specified options.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Flush remaining writes
	flushWrite();

	// Compression done
	if (pvstm != NULL)
		{
		// Compressing ?
		if (bReadOnly == false)
			{
			if (deflateEnd ( (z_streamp) pvstm ) != Z_OK)
				dbgprintf ( L"SysCompressStm::close:deflateEnd returned an error\r\n" );
			}	// if

		// Decompressing
		else
			{
			if (inflateEnd ( (z_streamp) pvstm ) != Z_OK)
				dbgprintf ( L"SysCompressStm::close:inflateEnd returned an error\r\n" );
			}	// else

		_FREEMEM(pvstm);
		}	// if

	// Clean up
	_RELEASE(pStm);

	return S_OK;
	}	// close

HRESULT StmZlib :: construct ( void )
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
	HRESULT	hr		= S_OK;

	// Create new memory block to use as buffer
	CCLTRY(COCREATE(L"Io.MemoryBlock",IID_IMemoryMapped,&pMemWr));
	CCLTRY(COCREATE(L"Io.MemoryBlock",IID_IMemoryMapped,&pMemRd));

	// Set sizes of buffers and lock
	CCLTRY(pMemWr->setSize ( SIZE_BUFFER ));
	CCLTRY(pMemWr->lock ( 0, 0, &pvMemWr, NULL ));
	CCLTRY(pMemRd->setSize ( SIZE_BUFFER ));
	CCLTRY(pMemRd->lock ( 0, 0, &pvMemRd, NULL ));

	return hr;
	}	// construct

HRESULT StmZlib :: copyTo ( IByteStream *pStmDst, U64 uSz, U64 *puSz )
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
	HRESULT	hr = S_OK;
	U64		nleft,nio,nr;

	// State check
	CCLTRYE ( pStm != NULL,			ERROR_INVALID_STATE );
	CCLTRYE ( bReadOnly == true,	ERROR_INVALID_STATE );

	// Setup
	if (puSz != NULL) *puSz = 0;

	// Operation invalidates both caches (not really needed ?)
	CCLTRY ( flushWrite() );
	CCLTRY ( flushRead() );

	// Consume 'uSz' bytes from source stream and expand into however many bytes
	// it ends up being for the output.
	CCLOK ( nleft = uSz; )
	while (hr == S_OK && (nleft > 0 || uSz == 0))
		{
		// Read next block from source stream
		if (uSz)	nio = (SIZE_BUFFER < nleft) ? SIZE_BUFFER : nleft;
		else		nio = SIZE_BUFFER;
		CCLTRY ( read ( pvMemWr, nio, &nr ) );

		// If input stream ran out, copy is done
		if (nr == 0)
			break;

		// Update count
		if (puSz != NULL)	*puSz += nr;

		// Write to destination
		CCLTRY ( writeAll ( pStmDst, pvMemWr, nr ) );

		// Next block
		if (hr == S_OK && uSz != 0)
			nleft -= nr;
		}	// while

	// Execute
//	CCLTRY ( pStm->copyTo ( pStmDst, uSz, puSz ) );

	return hr;
	}	// copyTo

void StmZlib :: destruct ( void )
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
	_UNLOCK(pMemRd,pvMemRd);
	_UNLOCK(pMemWr,pvMemWr);
	_RELEASE(pMemRd);
	_RELEASE(pMemWr);
	_RELEASE(pStm);
	}	// destruct

HRESULT StmZlib :: flush ( void )
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
	HRESULT	hr = S_OK;

	// Subsystem flushes
	CCLTRY ( flushRead() );
	CCLTRY ( flushWrite() );

	return hr;
	}	// flush

HRESULT StmZlib :: flushRead ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Flushes the read cache.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr = S_OK;

	// Not sure what would make sense here.  Emptying available input would
	// mess up the state of the decompressor.

	return hr;
	}	// flushRead

HRESULT StmZlib :: flushWrite ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Flushes the write cache.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	z_stream	*pzstm	= (z_stream *) pvstm;
	int		ret		= Z_OK;

	// Read only mode ?
	if (bReadOnly == true)
		return S_OK;

	// State check
	CCLTRYE ( pStm != NULL,		ERROR_INVALID_STATE );
	CCLTRYE ( pzstm != NULL,	ERROR_INVALID_STATE );

	// Finish the deflation process.  Must be called until completed.
	while (hr == S_OK)
		{
		// Finish compression
		ret = deflate ( pzstm, Z_FINISH );

		// If deflate returns OK that means it needs more space to finish
		if (ret == Z_OK)
			{
			// Write data
			CCLTRY ( writeAll ( pStm, pvMemWr, SIZE_BUFFER ) );

			// Reset output buffer
			pzstm->next_out	= (Bytef *) pvMemWr;
			pzstm->avail_out	= SIZE_BUFFER;
			}	// if

		// If deflate returns Z_STREAM_END then compression is finished,
		// write remaining data.
		else if (ret == Z_STREAM_END)
			{
			// Write remaining data
			CCLTRY ( writeAll ( pStm, pvMemWr, SIZE_BUFFER-pzstm->avail_out ) );

			// Done
			break;
			}	// else if

		// Error
		else
			{
			hr = E_UNEXPECTED;
			dbgprintf ( L"SysCompressStm::write:Compression error:ret %d\r\n", ret );
			}	// else

		}	// while

	return hr;
	}	// flushWrite

HRESULT StmZlib :: getResId ( ADTVALUE &vId )
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

	// Not sure what would make sense here
	lprintf ( LOG_ERR, L"Not implemented\r\n" );
	return E_NOTIMPL;
	}	// getResId

HRESULT StmZlib :: open ( IDictionary *pOpts )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IResource
	//
	//	PURPOSE
	//		-	Opens the resource.
	//
	//	PARAMETERS
	//		-	pOpts contain options
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr 		= S_OK;
	z_stream			*pzstm	= NULL;
	adtIUnknown		unkV;
	adtValue			vL;

	// Already open ?
	CCLTRYE ( pStm == NULL, ERROR_INVALID_STATE );

	// Options
	bReadOnly	= true;
	if (hr == S_OK && pOpts->load ( adtString(L"ReadOnly"), vL ) == S_OK)
		bReadOnly = vL;

	// Destination stresm
	CCLTRY( pOpts->load ( adtString(L"Stream"), vL ));
	CCLTRY( _QISAFE((unkV=vL),IID_IByteStream,&pStm) );

	// Set up for compression/decompression based on 'ReadOnly' flag of
	// options context.

	// Inflate
	if (hr == S_OK && bReadOnly == true)
		{
		// Allocate memory for the ZLib stream
		CCLTRYE ( (pzstm = (z_stream *) _ALLOCMEM(sizeof(z_stream))) != NULL,
						E_OUTOFMEMORY );
		if (hr == S_OK)
			{
			memset ( pzstm, 0, sizeof(pzstm) );
			pzstm->zalloc	= zalloc;
			pzstm->zfree	= zfree;
			}	// if

		// Initialize decompression
		CCLTRYE ( inflateInit ( pzstm ) == Z_OK, E_UNEXPECTED );

		// Initialize decompression state
		if (hr == S_OK)
			{
			// Internal memory buffer
			pzstm->next_in		= (Bytef *) pvMemRd;
			pzstm->avail_in	= 0;

			// Transfer ptr.
			pvstm					= pzstm;
			}	// if

		}	// if

	// Deflate
	else if (hr == S_OK && bReadOnly == false)
		{
		// Allocate memory for the ZLib stream
		CCLTRYE ( (pzstm = (z_stream *) _ALLOCMEM(sizeof(z_stream))) != NULL,
						E_OUTOFMEMORY );
		if (hr == S_OK)
			{
			memset ( pzstm, 0, sizeof(pzstm) );
			pzstm->zalloc	= zalloc;
			pzstm->zfree	= zfree;
			}	// if

		// Initialize compression
		CCLTRYE ( deflateInit ( pzstm, Z_DEFAULT_COMPRESSION ) == Z_OK, E_UNEXPECTED );

		// Initialize compression state
		if (hr == S_OK)
			{
			// Internal memory buffer
			pzstm->next_out	= (Bytef *) pvMemWr;
			pzstm->avail_out	= SIZE_BUFFER;

			// Transfer ptr.
			pvstm					= pzstm;
			}	// if
		
		}	// else if

	// Clean up
	_RELEASE(pOpts);
	if (hr != S_OK && pzstm != NULL)
		{
		_FREEMEM(pzstm);
		pvstm = NULL;
		}	// if

	return hr;
	}	// open

HRESULT StmZlib :: read ( PVOID pvBfr, U64 nio, U64 *pnio )
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
	HRESULT		hr			= S_OK;
	z_stream		*pzstm	= (z_stream *) pvstm;
	U64			nleft,nr;
	int			ret;

	// State check
	CCLTRYE ( pStm != NULL,			ERROR_INVALID_STATE );
	CCLTRYE ( bReadOnly == true,	ERROR_INVALID_STATE );
	CCLTRYE ( pzstm != NULL,		ERROR_INVALID_STATE );

	// Setup
	if (pnio != NULL) *pnio = 0;

	// Output buffer
	if (hr == S_OK)
		{
		pzstm->next_out	= (Bytef *) pvBfr;
		pzstm->avail_out	= (U32)nio;
		}	// if

	// Continue decompression until read is satified or input runs out.
	CCLOK ( nleft = nio; )
	while (hr == S_OK)
		{
		// Continue decompression from last read
		ret = inflate ( pzstm, Z_SYNC_FLUSH );

		// A buffer error will occur if decompression needs more input
		if ((ret == Z_OK || ret == Z_BUF_ERROR) && pzstm->avail_in == 0)
			{
			// Read another block into memory
			CCLTRY ( pStm->read ( pvMemRd, SIZE_BUFFER, &nr ) );

			// Prepare for more decompression
			if (hr == S_OK && nr > 0)
				{
				pzstm->avail_in	= (U32)nr;
				pzstm->next_in		= (Bytef *) pvMemRd;
				}	// if

			// End of stream
			else if (nr == 0)
				{
				// Update how much was read
				if (pnio != NULL)
					*pnio += (nio - pzstm->avail_out);
				hr = S_OK;
				break;
				}	// else if
			}	// if

		// Read satisfied when output is used up
		else if (ret == Z_OK && pzstm->avail_out == 0)
			{
			// Read satifisfied.
			if (pnio != NULL) *pnio = nio;
			break;
			}	// else if

		// End of compressed data has been reached
		else if (ret == Z_STREAM_END)
			{
			// Stream is done regardless of read request.
			if (pnio != NULL)
				*pnio = (nio - pzstm->avail_out);
			break;
			}	// else if

		// Error
		else
			{
			hr = E_UNEXPECTED;
			dbgprintf ( L"SysCompressStm::read:Decompression error:%d\r\n", ret );
			}	// else

		}	// while

	return hr;
	}	// read

HRESULT StmZlib :: seek ( S64 sPos, U32 uFrom, U64 *puPos )
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
	// Seek is not supported on compressed streams
	lprintf ( LOG_ERR, L"Not implemented\r\n" );
	return E_NOTIMPL;
	}	// seek

HRESULT StmZlib :: setSize ( U64 uSz )
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
	HRESULT	hr = S_OK;

	// State check
	CCLTRYE ( pStm != NULL, ERROR_INVALID_STATE );

	// Operation invalidates both caches
	CCLTRY ( flushWrite() );
	CCLTRY ( flushRead() );

	// Perform operation
	CCLTRY ( pStm->setSize ( uSz ) );

	return hr;
	}	// setSize

HRESULT StmZlib :: write ( void const *pcvBfr, U64 nio, U64 *pnio )
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
	//		-	pvBfr contains the data
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	z_stream		*pzstm	= (z_stream *) pvstm;
	int			ret;

	// State check
	CCLTRYE ( pStm != NULL,			ERROR_INVALID_STATE );
	CCLTRYE ( bReadOnly == false, ERROR_INVALID_STATE );
	CCLTRYE ( pzstm != NULL,		ERROR_INVALID_STATE );

	// Transfer uncompressed input data into compressed internal buffer.
	// If input runs dry, simply return.
	// If output runs dry, write contents of memory to destination stream
	// and start again.

	// Setup
	if (pnio != NULL) *pnio = 0;

	// Incoming data
	if (hr == S_OK)
		{
		pzstm->next_in		= (Bytef *) pcvBfr;
		pzstm->avail_in	= (U32) nio;
		}	// if

	// Continue compression until input buffer runs dry
	while (hr == S_OK && pzstm->avail_in > 0)
		{
		// Compress block
		ret = deflate ( pzstm, Z_NO_FLUSH );

		// Success ?
		if (ret == Z_OK)
			{
			// Result
			if (pnio != NULL)
				*pnio = (nio - pzstm->avail_in);

			// Output buffer out of room ?
			if (pzstm->avail_out == 0)
				{
				// Write data
				CCLTRY ( writeAll ( pStm, pvMemWr, SIZE_BUFFER ) );

				// Reset output buffer
				pzstm->next_out	= (Bytef *) pvMemWr;
				pzstm->avail_out	= SIZE_BUFFER;
				}	// if

			}	// if

		// Error
		else
			{
			hr = E_UNEXPECTED;
			dbgprintf ( L"SysCompressStm::write:Compression error\r\n" );
			}	// else

		}	// while

	return hr;
	}	// write

HRESULT StmZlib :: writeAll ( IByteStream *pStmDst,
										void const *pcvBfr, U64 nio )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Writes all the data to the destination stream.
	//
	//	PARAMETERS
	//		-	pStmDst is the destination stream
	//		-	pcvBfr contains the data
	//		-	nio is the # of bytes to transfer
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U64		nx,nw;

	// State check
	CCLTRYE ( pStmDst != NULL,	ERROR_INVALID_STATE );

	// Write memory buffer to destination
	CCLOK ( nw = 0; )
	while (hr == S_OK && nw < nio)
		{
		// Write block
		CCLTRY ( pStmDst->write ( &(((BYTE *)pcvBfr)[nw]), nio-nw, &nx ) );

		// Next block
		CCLOK ( nw += nx; )
		}	// while

	return hr;
	}	// writeAll

//
// Memory allocate redirection
//

voidpf zalloc ( voidpf, uInt items, uInt size )
	{
	return _ALLOCMEM ( items*size );
	}	// zalloc

void zfree ( voidpf, voidpf addr )
	{
	_FREEMEM(addr);
	}	// zfree

