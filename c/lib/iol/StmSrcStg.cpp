////////////////////////////////////////////////////////////////////////
//
//									STMSRCSTG.CPP
//
//						Compound document storage stream source
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"

StmSrcStg :: StmSrcStg ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	punkDct	= NULL;
	pDct		= NULL;
	pStg		= NULL;
	pCache	= NULL;
	bStgRead	= TRUE;
	strFile	= L"";
	strRoot	= L"";
	}	// SysStmsFile

HRESULT StmSrcStg :: construct ( void )
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
	HRESULT			hr			= S_OK;

	// Aggregrate dictionary
	CCLTRY ( COCREATEA ( L"Adt.Dictionary", (ILocations *) this, &punkDct ) );

	// Interfaces
	CCLTRY ( _QI(punkDct,IID_IDictionary,&pDct) );
	CCLOK  ( pDct->Release(); )						// Cancel 'QI'

	// For cache
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pCache ) );

	return hr;
	}	// construct

void StmSrcStg :: destruct ( void )
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
	_RELEASE(pCache);
	_RELEASE(pStg);
	_RELEASE(punkDct);
	}	// destruct

HRESULT StmSrcStg :: locations ( const WCHAR *wLoc, IIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	ILocations
	//
	//	PURPOSE
	//		-	Returns an iterator for the stream locations at the specified
	//			location.
	//
	//	PARAMETERS
	//		-	wLoc specifies the stream location
	//		-	ppIt will receive a stream location iterator.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return E_NOTIMPL;
	}	// locations

HRESULT StmSrcStg :: open ( IDictionary *pOpts, IUnknown **ppLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	ILocations
	//
	//	PURPOSE
	//		-	Open a a location inside the locations.
	//
	//	PARAMETERS
	//		-	pOpts contain options for the stream.
	//		-	ppLoc will receive the location object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IIt			*pIt			= NULL;
	IStorage		*pStgNext	= NULL;
	IStorage		*pStgStm		= NULL;
	IStream		*pStm			= NULL;
	WCHAR			*wb,*wa;
	DWORD			mode;
	adtIUnknown	unkV;
	adtString	strLoc;
	adtBool		bRead,bCreate;
	adtValue		vL;
	U32			len;

	// State check
	CCLTRY ( pOpts->load ( adtString(L"Location"), strLoc ) );
	CCLTRYE ( ((len = strLoc.length()) > 0),	ERROR_INVALID_STATE );
	CCLOK ( pOpts->load ( adtString(L"ReadOnly"), bRead ); )
	CCLOK ( pOpts->load ( adtString(L"Create"), bCreate ); )
	CCLOK ( strLoc.replace ( WCHAR('/'), WCHAR('\\') ); )

	// If root storage was opened read only and caller is requesting a
	// writable stream, everything must be closed and reopened.
	if (hr == S_OK && bRead == false && pStg != NULL && bStgRead == TRUE)
		{
		dbgprintf ( L"SysStmsStg::open:WARNING, flushing storage cache\r\n" );
		// Clear cache and close storage
		CCLOK ( pCache->clear(); )
		_RELEASE(pStg);
		}	// if

	// Root storage open ?
	if (hr == S_OK && pStg == NULL)
		{
		adtString	strLoc;

		// File specified ?
		CCLTRY ( pDct->load ( adtString ( L"File" ), vL ) );
		CCLTRYE ( (strFile = vL).length() > 0, ERROR_INVALID_STATE );

		// Generate full path
		CCLTRY ( toPath ( strFile, strLoc ) );

		// Selected mode
		mode = STGM_DIRECT;
		if (bRead == false)	mode |= (STGM_READWRITE|STGM_SHARE_EXCLUSIVE);
		else						mode |= (STGM_READ|STGM_SHARE_DENY_WRITE);

		// Access root storage object
		if (hr == S_OK)
			{
			// Attempt to open existing file
			// If options specify the create flag, always create
			if (bCreate == true)
				hr = STG_E_FILENOTFOUND;
			else
				hr = StgOpenStorage ( strLoc, NULL, mode, NULL, 0, &pStg );

			// If storage does not exist, and 'create' is selected
			// proceed to create a blank storage object
			if (hr == STG_E_FILENOTFOUND && bCreate == true)
				{
				// Allow creation and try again
				mode |= (STGM_CREATE);
				hr		= StgCreateDocfile ( strLoc, mode, 0, &pStg );
				}	// if
			}	// if

		// Read-only storage ?
		if (hr == S_OK) bStgRead = (bRead == true);
		}	// if

	// Support opening a stream given a path, which means opening/creating
	// sub-storages as necessary to get to the requested stream.

	// NOTE: The storage cache is for speed but mostly its to keep the generated
	// streams valid.  No storage objects above an open stream can be
	// 'released' without invalidating the stream...
	CCLOK ( wb = &(strLoc.at(0)); )

	// First, see if storage path is already in cache
	if (hr == S_OK)
		{
		// Find last slash to extract just the path
		if ( (wb = wcsrchr ( &strLoc.at(), WCHAR('\\') )) != NULL )
			*wb = WCHAR('\0');

		// In cache ?
		if (pCache->load ( strLoc, unkV ) == S_OK)
			hr = _QISAFE(unkV,IID_IStorage,&pStgStm);

		// Restore full path and point to filename
		if (wb != NULL)
			{
			*wb = WCHAR('\\');
			wb++;
			}	// if

		}	// if

	// Open sub-storage path
	if (hr == S_OK && pStgStm == NULL)
		{
		pStgStm = pStg;
		_ADDREF(pStgStm);
		wb = &(strLoc.at(0));
		for (	wa = wcschr ( &strLoc.at(), WCHAR('\\') );
				hr == S_OK && wa != NULL;
				wb = wa+1, wa = wcschr ( wb, WCHAR('\\') ) )
			{
			// Selected mode
			mode = STGM_DIRECT|STGM_SHARE_EXCLUSIVE;
			if (bRead == false)	mode |= (STGM_READWRITE);
			else						mode |= (STGM_READ);

			// Terminate string at next slash
			*wa = WCHAR('\0');

			// Storage in cache ?
			if (hr == S_OK && pCache->load ( strLoc, unkV ) == S_OK)
				{
				hr = _QISAFE(unkV,IID_IStorage,&pStgNext);
				}	// if
			else
				{
				// Access sub storage
				CCLTRY ( pStgStm->OpenStorage ( wb, NULL, mode, NULL, 0, &pStgNext ) );

				// If storage does not exist, and 'create' is selected
				// proceed to create a blank storage object
				if (hr == STG_E_FILENOTFOUND && bCreate == true)
					{
					mode |= (STGM_CREATE);
					hr		= pStgStm->CreateStorage ( wb, mode, 0, 0, &pStgNext );
					}	// if

				// Store storage in cache
				CCLTRY ( pCache->store ( adtString((LPCWSTR)strLoc), adtIUnknown(pStgNext) ) );
				}	// else

			// Restore path
			*wa = WCHAR('\\');

			// Next storage level, swap ptrs/reference counts
			_RELEASE(pStgStm);
			pStgStm	= pStgNext;
			pStgNext	= NULL;
			}	// for
		}	// if

	// At this point, 'pStgStm' points to the correct storage level
	// and 'wb' points to the name for the actual stream.
	// The stream is cached because of some ridiculous timing issue inside the compound
	// storage document where opening/closing the same stream too quickly causes an 'access denied'
	// error.  A 'clone' of the 'real' stream is returned to the caller.
	if (hr == S_OK)
		{
		IStream		*pStmClone	= NULL;

		// See if stream is already open
		if (pCache->load ( strLoc, unkV ) != S_OK)
			{
			// Selected mode
			mode = STGM_DIRECT|STGM_SHARE_EXCLUSIVE;
			if (bRead == false)	mode |= (STGM_READWRITE);
			else						mode |= (STGM_READ);

			// Open specified stream on storage object.
			CCLTRY(pStgStm->OpenStream ( wb, NULL, mode, 0, &pStm ));
			if (hr == STG_E_FILENOTFOUND && bRead == false)
				{
				// Create stream if it does not exist
				hr = pStgStm->CreateStream ( wb, mode, 0, 0, &pStm );
				}	// if

			// DEBUG
			#ifdef	_DEBUG
			if (hr == STG_E_ACCESSDENIED)
				DebugBreak();
			#endif

			// Store stream in cache
			CCLTRY ( pCache->store ( adtString((LPCWSTR)strLoc), adtIUnknown(pStm) ) );
			}	// if
		else
			{
			CCLTRY(_QISAFE(unkV,IID_IStream,&pStm));
			}	// else

		// Clone a stream for the caller, cache keeps 'original' version.
		CCLTRY ( pStm->Clone ( &pStmClone ) );

		// Wrap COM 'IStream' object in our 'IByteStream' interface
		CCLTRYE ( ((*ppLoc) = (IByteStream *) new StmStg ( pStmClone )) != NULL, E_OUTOFMEMORY );
		CCLOK   ( (*ppLoc)->AddRef(); )

		// Clean up
		_RELEASE(pStmClone);
		_RELEASE(pStm);
		}	// if

	// Debug
	if (hr != S_OK)
		lprintf ( LOG_DBG, L"Failed, hr %d,0x%x (%s)\r\n",
									hr, hr, (LPCWSTR)strLoc );

	// Clean up
	_RELEASE(pStgStm);

	return hr;
	}	// open

HRESULT StmSrcStg :: resolve ( const WCHAR *pwLoc, bool bAbs, 
											ADTVALUE &vLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	ILocations
	//
	//	PURPOSE
	//		-	Resolve provided location to an absolute or relative path
	//			within the stream source.
	//
	//	PARAMETERS
	//		-	pwLoc specifies the stream location
	//		-	bAbs is true to produce an absolute path, or false to produce
	//			a relative path
	//		-	vLoc will receive the new location
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return E_NOTIMPL;
	}	// resolve

HRESULT StmSrcStg :: status ( const WCHAR *wLoc, IDictionary *pSt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	ILocations
	//
	//	PURPOSE
	//		-	Returns information about the stream at the specified location.
	//
	//	PARAMETERS
	//		-	wLoc specifies the stream location
	//		-	pSt will receive the information about the stream.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return E_NOTIMPL;
	}	// status

HRESULT StmSrcStg :: toPath ( const WCHAR *wLoc, adtString &sLocFull )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ILocations
	//
	//	PURPOSE
	//		-	Generates a 'full path' to the specified file.
	//
	//	PARAMETERS
	//		-	wLoc contains the file or partial path to a file.
	//		-	sFull will receive the full path to the file
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr 		= S_OK;

	// Generate a full path if not already an absolute path
	if (	hr == S_OK					&& 
			wcslen(wLoc) > 0			&&
			wLoc[0] != WCHAR('\\')	&&
			wLoc[0] != WCHAR('/')
			#if	defined(_WIN32) && !defined(UNDER_CE)
			&& wLoc[1] != WCHAR(':')					// Drive letter ?
			#endif
		)
		{
		// Load root in case it has changed
		CCLTRY ( pDct->load ( adtString(L"Root"), strRoot ) );

		// Generate path that includes the default root
		U32
		len = strRoot.length()+(U32)wcslen(wLoc)+1;
		CCLTRY ( sLocFull.allocate ( len )  );
		CCLOK	 ( WCSCPY ( &sLocFull.at(), len, strRoot ); )
		CCLOK	 ( WCSCAT ( &sLocFull.at(), len, wLoc ); )
		}	// if

	// Using root directly ?
	else if (wcslen(wLoc) == 0)
		hr = pDct->load ( adtString(L"Root"), sLocFull );

	// Absolute paths are unmodified
	else if (hr == S_OK)
		sLocFull = wLoc;

	return hr;
	}	// toPath
