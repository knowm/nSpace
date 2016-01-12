////////////////////////////////////////////////////////////////////////
//
//									STMSRCFS.CPP
//
//						File system based stream source
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#if defined(__APPLE__) || defined(__unix__)
#include <dirent.h>
#endif

StmSrcFile :: StmSrcFile ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	punkDct	= NULL;
	pDct		= NULL;
	pStmOpts	= NULL;
	strRoot	= L"";

	// Frequently used keys
	strkLoc		= L"Location";
	strkRO		= L"ReadOnly";
	strkCr		= L"Create";
	strkTr		= L"Truncate";
	strkAsync	= L"Async";
	}	// SysStmsFile

HRESULT StmSrcFile :: construct ( void )
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

	// For stream options
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pStmOpts ) );

	// Default root location
	CCLTRY(defaultRoot());

	return hr;
	}	// construct

HRESULT StmSrcFile :: createLoc ( const WCHAR *pLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to create a stream location.
	//
	//	PARAMETERS
	//		-	pLoc is the location.  A filename at the end is ignored.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	int				j,i	= -1;
	adtString		strLoc(pLoc),strDir;

	// Verify/create location one folder at a time
	while (	hr == S_OK &&
				(	(j = strLoc.indexOf ( '/', i+1 ))	!= -1 ||
					(j = strLoc.indexOf ( '\\', i+1 ))	!= -1 ) )
		{
		// Generate path up to this point (allow for extra long path names)
		CCLOK  ( i = j; )
		CCLTRY ( strLoc.substring ( 0, i+1, strDir ) );
//		CCLTRY ( strDir.prepend ( L"\\\\?\\" ) );
//		CCLOK  ( strDir.replace ( '/', '\\' ); )

		#ifdef			_WIN32
      U32				fa;
      
		// Does the directory already exist ?
		CCLOK ( fa = GetFileAttributes ( strDir ); )

		// Does not exist
		if (hr == S_OK && fa == INVALID_FILE_ATTRIBUTES)
			{
			// Create the location 
			CCLTRYE ( CreateDirectory ( strDir, NULL ) == TRUE, GetLastError() );

			// Update file attributes
			CCLOK ( fa = GetFileAttributes ( strDir ); )
			}	// if

		// Exists but not a directory, cannot proceed
		CCLTRYE ( (fa & FILE_ATTRIBUTE_DIRECTORY), E_UNEXPECTED );
		#elif __APPLE__ || __unix__
      {
      char        *pc = NULL;
      struct stat s;
      
      // Convert to ASCII for system calls
      CCLTRY ( strDir.toAscii(&pc) );
      
      // Get status of location
      if (stat ( pc, &s ) != 0)
         {
         // Create the directory
         mkdir ( pc, S_IRWXU );
         }  // if
         
      // Clean up
      _FREEMEM(pc);
      }
		#endif
		}	// while
	
	// Debug
	if (hr != S_OK)
		dbgprintf ( L"StmSrcFile::createLoc:Failed:0x%x\r\n", hr );

	return hr;
	}	// createLoc

HRESULT StmSrcFile :: defaultRoot ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Generate a system-specific default root location for streams.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	#ifdef	_WIN32
	HKEY		hKey	= NULL;
	LONG		lRes;
	DWORD		dwType,dwSz,len;
	WCHAR		strBfr[1024];

	///////////////////////////////////////////////////////////////////
	// Multiple options for setting the default root directory.  Avoid
	// using 'working directory' since it changes so much between OSes
	///////////////////////////////////////////////////////////////////

	// Option 1 - Default is to use the same path where the process EXE is located
	if (hr == S_OK)
		{
		WCHAR		*pwSlash;
		DWORD		len;

		// Get path to running executable
		CCLOK   ( len = sizeof(strBfr)/sizeof(strBfr[0]); )
		CCLTRYE ( (len = GetModuleFileName ( NULL, strBfr, len )) != 0, GetLastError() );

		// Path only
		CCLTRYE ( (pwSlash = wcsrchr ( strBfr, WCHAR('\\') )) != NULL, E_UNEXPECTED );
		CCLOK   ( *(pwSlash+1) = WCHAR('\0'); )

		// Paths in nSpace are always forward slash
		for (size_t i = 0;hr == S_OK && i < wcslen(strBfr);++i)
			if (strBfr[i] == '\\')
				strBfr[i] = '/';

		// New root
		strRoot = strBfr;
		}	// if
	/*
	// Option 2 - For development the 'nsh.exe' is put into a 'debug' folder but
	#ifdef	_DEBUG
	if (	hr == S_OK										&& 
			(len = strRoot.length()) > wcslen(L"Debug/")	&&
			!WCASECMP ( strRoot, L"Debug/" ) )
		{
		// Previous path
		CCLOK ( wcscpy_s ( strBfr, sizeof(strBfr)/sizeof(strBfr[0]), strRoot ); )
		CCLOK ( strBfr[len-wcslen(L"Debug/")] = '\0'; )

		// Allocate memory for new path
		CCLTRY ( strRoot.allocate ( wcslen(strBfr)+10 ) );

		// New path
		CCLOK ( wcscpy_s ( &strRoot.at(), wcslen(strBfr)+10, strBfr ); )
		CCLOK ( wcscat_s ( &strRoot.at(), wcslen(strBfr)+10, L"graphs/" ); )
		}	// if
	#endif
	*/

	// Option 3 - Environment variable.  This is useful for using the debugger
	// or setting a root via the command line.
	if (hr == S_OK && GetEnvironmentVariable ( L"NSHROOT", 
			strBfr, sizeof(strBfr)/sizeof(strBfr[0]) ) > 0)
		strRoot = strBfr;

	// Option 4 - Hard coded root path in registry for all nSpace executions.
	if (	(hr == S_OK)
			&& 
			((lRes = RegCreateKeyEx ( HKEY_LOCAL_MACHINE, L"SOFTWARE\\nSpace",
												0, NULL, REG_OPTION_NON_VOLATILE, 
												KEY_READ, NULL,
												&hKey, NULL )) == ERROR_SUCCESS)
			&&
			((lRes = RegQueryValueEx ( hKey, L"Root", NULL, &dwType, NULL, &dwSz ))
													== ERROR_SUCCESS) )
		{
		// Allocate (+1 for possible slash)
		CCLTRY ( strRoot.allocate ( dwSz/sizeof(WCHAR) ) );

		// Read
		CCLTRYE ( ((lRes = RegQueryValueEx ( hKey, L"Root", NULL, &dwType,
						(BYTE *) &strRoot.at(), &dwSz )) == ERROR_SUCCESS), lRes );

		// Our paths always end in '\\'
		CCLOK ( len = strRoot.length(); )
		if (	hr == S_OK && strRoot[len-1] != WCHAR('/') &&
				strRoot[len-1] != WCHAR('\\') )
			{
			strRoot.at(len)	= WCHAR('\\');		// Add a slash
			strRoot.at(len+1)	= WCHAR('\0');		// Terminate at new position
			}	// if

		// Debug
//		dbgprintf ( L"SysStmsFile::open:Root nSpace Path:%s\r\n", (LPWSTR)strRoot );
		}	// if

	// Clean up
	if (hKey != NULL) RegCloseKey ( hKey );

	// Unix
	#elif	__unix__ || __APPLE__
	if ( hr == S_OK )
		{
		char	cwd[1024];

		// Working directory
		CCLTRYE ( getcwd ( cwd, sizeof(cwd) ) != NULL, errno );

		// Paths always end in '/'
		CCLOK ( strcat ( cwd, "/" ); )

		// New root
		strRoot = cwd;
		}	// else if
	#endif

	// the graphs are somewhere else
	// Default root
	CCLTRY ( pDct->store ( adtString(L"Root"), strRoot ) );
	CCLOK  ( lprintf ( LOG_DBG, L"%s", (const WCHAR *)strRoot ); )

	return hr;
	}	// defaultRoot

void StmSrcFile :: destruct ( void )
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
	_RELEASE(pStmOpts);
	_RELEASE(punkDct);
	}	// destruct

HRESULT StmSrcFile :: locations ( const WCHAR *wLoc, IIt **ppIt )
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
	HRESULT				hr		= S_OK;
	IList					*pLst	= NULL;
	adtString			strLoc;

	// Create a list of locations and use that lists iterator
	CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pLst ) );

	// Generate full path to stream using source options
	CCLTRY ( toPath ( wLoc, strLoc ) );

	// Iterate the files in the specified location
	#ifdef	_WIN32
		{
		HANDLE				hFind	= INVALID_HANDLE_VALUE;
		BOOL					bMore;
		WIN32_FIND_DATA	wfd;
		CCLTRY ( strLoc.append ( L"*" ) );
		for (	bMore = ((hFind = FindFirstFile ( strLoc, &wfd )) != INVALID_HANDLE_VALUE);
				bMore;
				bMore = FindNextFile ( hFind, &wfd ) )
			{
			// Ignore directory entries
			if (!WCASECMP(wfd.cFileName,L".") || !WCASECMP(wfd.cFileName,L".."))
				continue;

			// nSpace locations end with a slash
			if (wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
				{
				// Add slash
				wfd.cFileName[wcslen(wfd.cFileName)+1] = '\0';
				wfd.cFileName[wcslen(wfd.cFileName)] = '/';
				}	// if

			// Add name to list
			CCLTRY ( pLst->write ( adtString(wfd.cFileName) ) );
			}	// for
			
		// Clean up
		if (hFind != INVALID_HANDLE_VALUE)
			FindClose ( hFind );
		}
	#elif defined(__APPLE__) || defined(__unix__)
      {
		DIR				*dir		= NULL;
		char				*pcLoc	= NULL;
		struct dirent	*ent		= NULL;
		adtString		strName;
		
		// ASCII version of location
		CCLTRY ( strLoc.toAscii ( &pcLoc ) );
		
		// Access location
		if (hr == S_OK && (dir = opendir ( pcLoc )) != NULL)
			{	
			// Read entries in directory
			while (hr == S_OK && (ent = readdir ( dir )) != NULL)
				{
				// Ignore directory entries
				if (	ent->d_name[0] == '.' &&
						(	ent->d_name[1] == '\0' ||
							(ent->d_name[1] == '.' && ent->d_name[2] == '\0') ) )
					continue;

				// nSpace locations end with a slash
				if (ent->d_type & DT_DIR)
					{
					// Add slash
					ent->d_name[strlen(ent->d_name)+1] = '\0';
					ent->d_name[strlen(ent->d_name)] = '/';
					}	// if
				
				// Add file to list
				CCLTRY ( pLst->write ( (strName = ent->d_name) ) );
				}	// while
			}	// if

		// Clean up
		if (dir != NULL)
			closedir ( dir );
		_FREEMEM(pcLoc);
      }
	#endif

	// Create iterator for locations
	CCLTRY ( pLst->iterate ( ppIt ) );

	// Clean up
	_RELEASE(pLst);

	return hr;
	}	// locations

HRESULT StmSrcFile :: open ( IDictionary *pOpts, IUnknown **ppLoc )
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
	HRESULT		hr		= S_OK;
	IResource	*pRes	= NULL;
	adtValue		v;
	adtString	strLoc,strLocFull;
	adtBool		bRO(true),bCr(false);
	
	// Setup
	*ppLoc	= NULL;

	// Stream location (required)
	CCLTRY ( pOpts->load ( strkLoc, v ) );

	// Generate full path to stream using source options
	CCLTRY ( toPath ( (strLoc = v), strLocFull ) );

	// Copy relevant options for stream
	CCLTRY ( pStmOpts->store ( strkLoc, strLocFull ) );
	if (pOpts->load ( strkRO, bRO ) == S_OK)
		pStmOpts->store ( strkRO, bRO );
	if (pOpts->load ( strkCr, bCr ) == S_OK)
		pStmOpts->store ( strkCr, bCr );
	if (pOpts->load ( strkTr, v ) == S_OK)
		pStmOpts->store ( strkTr, v );
	if (pOpts->load ( strkAsync, v ) == S_OK)
		pStmOpts->store ( strkAsync, v );

	// Ensure that the location of the stream exists within the source
//	dbgprintf ( L"StmSrcFile::open:0x%x:%s:RO:%d:Cr:%d\r\n", hr, (LPCWSTR)strLocFull, 
//					(bool) bRO, (bool) bCr );
	if (hr == S_OK && bRO == false && bCr == true)
		hr = createLoc ( strLocFull );
	
	// Create a file stream resource
	CCLTRY ( COCREATE ( L"Io.StmFile", IID_IResource, &pRes ) );

	// Attempt to open stream
	CCLTRY ( pRes->open ( pStmOpts ) );

	// Result
	CCLTRY ( _QISAFE(pRes,IID_IByteStream,ppLoc) );

	// Clean up
	_RELEASE(pRes);

	return hr;
	}	// open

HRESULT StmSrcFile :: status ( const WCHAR *wLoc, IDictionary *pSt )
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
	HRESULT		hr		= S_OK;
	adtString	strLoc;
	adtDate		dC,dA,dM;
	
	// Generate full path to stream using source options
	CCLTRY ( toPath ( wLoc, strLoc ) );

	#ifdef	_WIN32
		{
		HANDLE							hFile	= INVALID_HANDLE_VALUE;
		BY_HANDLE_FILE_INFORMATION	hfi;
		SYSTEMTIME						sct,sat,smt;
		
		// Access file
		CCLTRYE( (hFile = CreateFile ( strLoc, GENERIC_READ,
												FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,
												NULL, OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, NULL )) != INVALID_HANDLE_VALUE,
						GetLastError() );

		// File attributes
		CCLTRYE ( GetFileInformationByHandle ( hFile, &hfi ) == TRUE,  GetLastError() );

		// Convert file times to system date
		CCLTRYE ( FileTimeToSystemTime ( &(hfi.ftCreationTime), &sct ) == TRUE, GetLastError() );
		CCLTRY  ( adtDate::fromSystemTime ( &sct, &(dC.vdate) ) );
		CCLTRYE ( FileTimeToSystemTime ( &(hfi.ftLastAccessTime), &sat ) == TRUE, GetLastError() );
		CCLTRY  ( adtDate::fromSystemTime ( &sat, &(dA.vdate) ) );
		CCLTRYE ( FileTimeToSystemTime ( &(hfi.ftLastWriteTime), &smt ) == TRUE, GetLastError() );
		CCLTRY  ( adtDate::fromSystemTime ( &smt, &(dM.vdate) ) );

		// Store information
		CCLTRY ( pSt->store ( adtString(L"Size"), adtInt(hfi.nFileSizeLow) ) );
		
		// Directory ?
		if (hr == S_OK && (hfi.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			hr = pSt->store ( adtString(L"Folder"), adtBool(true) );

		// Clean up
		if (hFile != INVALID_HANDLE_VALUE)
			CloseHandle ( hFile );
		}
	#elif __APPLE__ || __unix__
	char			*pcLoc	= NULL;
	struct stat	lstat;
	
	// ASCII version of location
	CCLTRY ( strLoc.toAscii ( &pcLoc ) );

	// File status
	CCLTRYE ( (stat ( pcLoc, &lstat ) == 0), GetLastError() );

	// Convert times to dates
	CCLTRY ( adtDate::fromEpochSeconds ( (S32)lstat.st_ctime, 0, &(dC.vdate) ) );
	CCLTRY ( adtDate::fromEpochSeconds ( (S32)lstat.st_atime, 0, &(dA.vdate) ) );
	CCLTRY ( adtDate::fromEpochSeconds ( (S32)lstat.st_mtime, 0, &(dM.vdate) ) );

	// File size
	CCLTRY ( pSt->store ( adtString(L"Size"), adtLong(lstat.st_size) ) );
	
	// Clean up
	_FREEMEM(pcLoc);
	#endif
	
	// Store information
	CCLTRY ( pSt->store ( adtString(L"Creation"), dC ) );
	CCLTRY ( pSt->store ( adtString(L"Accessed"), dA ) );
	CCLTRY ( pSt->store ( adtString(L"Modified"), dM ) );

	return hr;
	}	// status

HRESULT StmSrcFile :: toPath ( const WCHAR *wLoc, adtString &sLocFull )
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

