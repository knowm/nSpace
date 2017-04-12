////////////////////////////////////////////////////////////////////////
//
//									STMFS.CPP
//
//						File system based stream 
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"

#if      __unix__ || __APPLE__
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#endif

StmFile :: StmFile ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pFile		= NULL;

	// Frequently used keys
	strkLoc	= L"Location";
	strkRO	= L"ReadOnly";
	strkCr	= L"Create";
	strkTr	= L"Truncate";
	strkAsync= L"Async";

//	dbgprintf ( L"StmFile::StmFile:%p\r\n", this );
	}	// StmFile

HRESULT StmFile :: available ( U64 *puAv )
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

HRESULT StmFile :: close ( void )
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
	_RELEASE(pFile);
	return S_OK;
	}	// close

HRESULT StmFile :: copyTo ( IByteStream *pStmDst, U64 uSz, U64 *puSz )
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
	U64		nio,nw,nr;

	// Setup
	CCLTRYE ( pFile != NULL, ERROR_INVALID_STATE );
	if (puSz != NULL)
		*puSz = 0;

	// If size is not specified, assume entire stream is to be copied
	if (hr == S_OK && uSz == 0)
		hr = available(&uSz);

	// Read/write file
	while (hr == S_OK && uSz)
		{
		// Read next block
		CCLOK ( nio = (sizeof(filebufr) < uSz) ? sizeof(filebufr) : uSz; )
		CCLTRY( read ( filebufr, nio, &nr ) );

		// Write full block to stream
		CCLOK ( fp		= filebufr; )
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
		CCLOK ( uSz -= nio; )
		if (hr == S_OK && puSz != NULL)
			*puSz += nio;

		// If at end of file before request has been satisfied, stop
		if (uSz && (nr < nio))
			break;
		}	// while

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"StmFile::copyTo:Failed:0x%x\r\n", hr );

	return hr;
	}	// copyTo

void StmFile :: destruct ( void )
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
//	dbgprintf ( L"StmFile::destruct:%p\r\n", this );
	}	// destruct

HRESULT StmFile :: flush ( void )
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

	// Handle/descriptor
	#ifdef	_WIN32
	if (pFile != NULL)
		hr = (FlushFileBuffers ( (*pFile) ) == TRUE) ? S_OK : GetLastError();
	#else
	if (pFile != NULL)
		fsync ( (*pFile) );
	#endif

	return hr;
	}	// flush

HRESULT StmFile :: getResId ( ADTVALUE &vId )
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
	#ifdef	_WIN32
	if (pFile != NULL)
		hr = adtValue::copy ( adtLong ( (U64)(HANDLE)(*pFile) ), vId );
	#else
	if (pFile != NULL)
		hr = adtValue::copy ( adtLong ( (int)(*pFile) ), vId );
	#endif
	
	return hr;
	}	// getResId

HRESULT StmFile :: open ( IDictionary *pOpts )
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
	#ifdef		_WIN32
	HANDLE		hFile		= INVALID_HANDLE_VALUE;
	#else
	int			fd;
	#endif
	char			*pcFile	= NULL;
	adtString	strLoc;
	adtBool		bRO(true),bCr(false),bTr(false),bAsync(false);
	adtValue		v;

	// Stream location (required)
	CCLTRY ( pOpts->load ( strkLoc, v ) );
	CCLTRYE( (strLoc = v).length() > 0, ERROR_INVALID_STATE );

	// Read only access ?
	if (hr == S_OK && pOpts->load ( strkRO, v ) == S_OK)
		bRO = v;

	// Asynchronous ?
	if (hr == S_OK && pOpts->load ( strkAsync, v ) == S_OK)
		bAsync = v;

	// Already open ?
	if (pFile != NULL)
		{
		dbgprintf ( L"StmFile::open:Attempt to open an already open file:%s\r\n",
							(const WCHAR *)strLoc );
		hr = E_UNEXPECTED;		
		}	// if

	#ifdef	_WIN32
	// Handle long filenames
//	CCLTRY ( strLoc.prepend ( L"\\\\?\\" ) );
//	CCLOK  ( strLoc.replace ( '/', '\\' ); )

	// Read-only file
	if (hr == S_OK && bRO)
		{
		// Attempt access
		CCLTRYE ( (hFile = CreateFile ( strLoc, GENERIC_READ, 
							FILE_SHARE_READ|FILE_SHARE_WRITE,
							NULL, OPEN_EXISTING, 
							(bAsync) ? FILE_FLAG_OVERLAPPED : 0, NULL )) != 
							INVALID_HANDLE_VALUE, GetLastError() );
		if (hr != S_OK)
			dbgprintf ( L"StmFile::open:Read only access failed:%s\r\n",
								(const WCHAR *) strLoc );
		}	// if

	// Writable file
	else if (hr == S_OK && !bRO)
		{
		// Creation/truncation of file requested ?
		CCLOK ( pOpts->load ( strkCr, bCr ); )
		CCLOK ( pOpts->load ( strkTr, bTr ); )

		// Flags for options
		DWORD
		dwFlags =	(bCr == true && bTr == true)	? CREATE_ALWAYS :
						(bCr == true && bTr == false)	? OPEN_ALWAYS : OPEN_EXISTING;

		// Attempt access
		CCLTRYE ( (hFile = CreateFile ( strLoc, GENERIC_READ|GENERIC_WRITE, 
							FILE_SHARE_READ|FILE_SHARE_WRITE, NULL, dwFlags, 
							(bAsync) ? FILE_FLAG_OVERLAPPED : 0, NULL )) 
							!= INVALID_HANDLE_VALUE, GetLastError() );

		// If there was an error, perhaps the path to the file does not exist ?
		if (hr != S_OK)
			dbgprintf ( L"StmFile::open:Writable file access failed:%s,0x%x\r\n",
								(const WCHAR *) strLoc, GetLastError() );
		}	// else if

	// Result
	CCLTRYE( ((pFile = new StmFileRes ( hFile )) != NULL), E_OUTOFMEMORY );
	CCLTRY ( pFile->construct() );
	CCLOK  ( pFile->AddRef(); )

	// Create events only when needed
	CCLOK	 ( pFile->bAsync = bAsync; )
	if (hr == S_OK && pFile->bAsync)
		{
		CCLTRYE ( (pFile->hevRd = CreateEvent ( NULL, TRUE, FALSE, NULL )) != NULL,
						GetLastError() );
		CCLTRYE ( (pFile->hevWr = CreateEvent ( NULL, TRUE, FALSE, NULL )) != NULL,
						GetLastError() );
		}	// if
	#else

	// Not yet implemented
	CCLTRYE ( bAsync == false, E_NOTIMPL );

	// ASCII version of filename
	CCLTRY ( strLoc.toAscii(&pcFile) );

	// Access file
	if (bRO == true)
		{
		// Open file
		CCLTRYE ( (fd = ::open ( pcFile, O_RDONLY )) != -1, errno );
		}	// if
	else
		{
		adtBool	bCreate(false),bTruncate(false);
		int		flags = O_RDWR;

		// Additional options
		if (pOpts->load ( strkCr, bCreate ) == S_OK && bCreate == true)
			flags |= O_CREAT;
		if (pOpts->load ( strkTr, bTruncate ) == S_OK && bTruncate == true)
			flags |= O_TRUNC;

		// Open file
		CCLTRYE ( (fd = ::open ( pcFile, flags, S_IRUSR|S_IWUSR )) != -1, errno );
		}	// else

	// Result
	CCLTRYE( ((pFile = new StmFileRes ( fd )) != NULL), E_OUTOFMEMORY );
	CCLTRY ( pFile->construct() );
	CCLOK  ( pFile->AddRef(); )

	// Clean up
	_FREEMEM(pcFile);
	#endif

//	dbgprintf ( L"StmFile::open:%p:%s\r\n", this, (LPCWSTR)strLoc );
	
	return hr;
	}	// open

HRESULT StmFile :: read ( void *pvBfr, U64 nio, U64 *pnio )
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
	HRESULT			hr = S_OK;
	#ifdef	_WIN32
	DWORD				nr = 0;
	#else
	off_t				nr	= 0;
	#endif

	// State check
	CCLTRYE ( pFile != NULL, ERROR_INVALID_STATE );

	// Protection
	if (pFile != NULL) pFile->csRd.enter();

	#ifdef	_WIN32
	// Synchronous
	if (hr == S_OK && !pFile->bAsync)
		{
		CCLTRYE ( ReadFile ( *pFile, pvBfr, (DWORD)nio, &nr, NULL ), GetLastError() );
		CCLTRYE ( (nr > 0), S_FALSE );
		}	// if

	// Async
	else if (hr == S_OK && pFile->bAsync)
		{
		OVERLAPPED	ov;

		// Overlapped setup
		memset ( &ov, 0, sizeof(ov) );
		ResetEvent ( pFile->hevRd );
		ov.hEvent = pFile->hevRd;

		// Perform read
		if (ReadFile ( *pFile, pvBfr, (DWORD)nio, NULL, &ov ))
			hr = (GetOverlappedResult ( *pFile, &ov, &nr, FALSE )) ? S_OK : GetLastError();
		else if (GetLastError() == ERROR_IO_PENDING)
			{
			// Wait for completion
			CCLTRYE ( (WaitForSingleObject ( pFile->hevRd, INFINITE ) == WAIT_OBJECT_0), ERROR_TIMEOUT );
			CCLTRYE ( GetOverlappedResult ( *pFile, &ov, &nr, FALSE ), GetLastError() );
			}	// else if
		else
			hr = GetLastError();
		}	// else if
	#else
	CCLTRYE ( (nr = ::read ( *pFile, pvBfr, nio )) != -1, GetLastError() );
	CCLTRYE ( (nr > 0), S_FALSE );
	#endif

	// Result
	if (hr == S_OK && pnio != NULL)
		*pnio = nr;

	// Protection
	if (pFile != NULL) pFile->csRd.leave();

	// Debug
//	if (hr != S_OK)
//		dbgprintf ( L"StmFile::read:Failed:0x%x\r\n", hr );

	return hr;
	}	// read

HRESULT StmFile :: seek ( S64 sPos, U32 uFrom, U64 *puPos )
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
	HRESULT	hr		= S_OK;

	// State check
	CCLTRYE ( pFile != NULL, ERROR_INVALID_STATE );

	#ifdef	_WIN32
	LARGE_INTEGER	li;
	S32				pos	= 0;
	S32				lHigh	= ((sPos >> 32) & 0xffffffff);

	// Attempt requested seek
	li.QuadPart = sPos;
	CCLTRYE ( SetFilePointerEx ( *pFile, li, &li,
					(uFrom == STREAM_SEEK_SET) ? FILE_BEGIN :
					(uFrom == STREAM_SEEK_CUR) ? FILE_CURRENT : FILE_END ) 
					== TRUE, GetLastError() );

	// New position
	if (puPos != NULL)
		*puPos = li.QuadPart;
	#else
	off_t			pos	= 0;

	// Attempt seek
	CCLTRYE ( (pos =
					lseek ( *pFile, sPos,
						(uFrom == STREAM_SEEK_SET) ? SEEK_SET :
						(uFrom == STREAM_SEEK_CUR) ? SEEK_CUR : SEEK_END )) != -1,
						GetLastError() );

	// New position
	if (puPos != NULL)
		*puPos = pos;
	#endif

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"StmFile::seek:Failed:0x%x\r\n", hr );

	return hr;
	}	// seek

HRESULT StmFile :: setSize ( U64 uSz )
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
	CCLTRYE ( pFile != NULL, ERROR_INVALID_STATE );

	#ifdef	_WIN32
	LARGE_INTEGER	li;

	// Set size of file by seek to desired position
	li.QuadPart = uSz;
	CCLTRYE ( SetFilePointerEx ( *pFile, li, &li, FILE_BEGIN ) == TRUE, 
					GetLastError() );
	CCLTRYE ( SetEndOfFile ( *pFile ) == TRUE, GetLastError() );
	#else
	CCLTRYE ( (lseek ( *pFile, uSz, SEEK_END ) != -1), GetLastError() );
	#endif

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"StmFile::setSize:Failed:0x%x\r\n", hr );

	return hr;
	}	// setSize

HRESULT StmFile :: write ( void const *pcvBfr, U64 nio, U64 *pnio )
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
	HRESULT			hr = S_OK;
	#ifdef	_WIN32
	DWORD				nw = 0;
	#else
	off_t				nw	= 0;
	#endif

	// State check
	CCLTRYE ( pFile != NULL, ERROR_INVALID_STATE );

	// Protection
	if (pFile != NULL) pFile->csWr.enter();

	// Write
	#ifdef	_WIN32
	// Synchronous
	if (hr == S_OK && !pFile->bAsync)
		{
		CCLTRYE ( WriteFile ( *pFile, pcvBfr, (DWORD)nio, &nw, NULL ), GetLastError() );
		CCLTRYE ( (nw > 0), S_FALSE );
		}	// if

	// Async
	else if (hr == S_OK && pFile->bAsync)
		{
		OVERLAPPED	ov;

		// Overlapped setup
		memset ( &ov, 0, sizeof(ov) );
		ResetEvent ( pFile->hevWr );
		ov.hEvent = pFile->hevWr;

		// Perform write
		if (WriteFile ( *pFile, pcvBfr, (DWORD)nio, NULL, &ov ))
			hr = (GetOverlappedResult ( *pFile, &ov, &nw, FALSE )) ? S_OK : GetLastError();
		else if (GetLastError() == ERROR_IO_PENDING)
			{
			// Wait for completion
			CCLTRYE ( (WaitForSingleObject ( pFile->hevWr, INFINITE ) == WAIT_OBJECT_0), ERROR_TIMEOUT );
			CCLTRYE ( GetOverlappedResult ( *pFile, &ov, &nw, FALSE ), GetLastError() );
			}	// else if
		else
			hr = GetLastError();
		}	// else if
	#else
	CCLTRYE ( (nw = ::write ( *pFile, pcvBfr, nio )) != -1, GetLastError() );
	CCLTRYE ( (nw > 0), S_FALSE );
	#endif

	// Result
	if (hr == S_OK && pnio != NULL)
		*pnio = nw;

	// Protection
	if (pFile != NULL) pFile->csWr.leave();

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"StmFile::write:Failed:0x%x\r\n", hr );

	return hr;
	}	// write

//////////////
// StmFileRes
//////////////

#ifdef	_WIN32
StmFileRes :: StmFileRes ( HANDLE _hFile )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_hFile is the handle to the file
	//
	////////////////////////////////////////////////////////////////////////
	hFile		= _hFile;
	hevRd		= NULL;
	hevWr		= NULL;
	StmFileRes();
	}	// StmFile
#else
StmFileRes :: StmFileRes ( int _fd )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_fd is the file descriptor
	//
	////////////////////////////////////////////////////////////////////////
	StmFileRes();
	fd			= _fd;
	}	// StmFile
#endif

StmFileRes :: StmFileRes ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	bAsync	= false;
	}	// StmFile

void StmFileRes :: destruct ( void )
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
	#ifdef	_WIN32
	if (hFile != INVALID_HANDLE_VALUE)
		{
		CloseHandle ( hFile );
		hFile = INVALID_HANDLE_VALUE;
		}	// if
	if (hevRd != NULL)
		{
		CloseHandle ( hevRd );
		hevRd = NULL;
		}	// if
	if (hevWr != NULL)
		{
		CloseHandle ( hevWr );
		hevWr = NULL;
		}	// if
	#else
	if (fd != 1)
		{
		::close(fd);
		fd = -1;
		}	// if
	#endif
	}	// destruct

