////////////////////////////////////////////////////////////////////////
//
//									NSPCL.CPP
//
//						General namespace utilities
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "nspcl_.h"

// Globals
extern GlobalNspc	nspcglb;
adtStringSt	strnRefName		( STR_NSPC_NAME );
adtStringSt	strnRefNames	( STR_NSPC_NAMES );
adtStringSt	strnRefItms		( STR_NSPC_ITEMS );
adtStringSt	strnRefBehave	( STR_NSPC_BEHAVE );
adtStringSt	strnRefLocn		( STR_NSPC_LOC );
adtStringSt	strnRefType		( STR_NSPC_TYPE );
adtStringSt	strnRefMod		( STR_NSPC_MOD );
adtStringSt	strnRefDesc		( STR_NSPC_DESC );
adtStringSt	strnRefOnDesc	( STR_NSPC_ONDESC );
adtStringSt	strnRefPar		( STR_NSPC_PARENT );
adtStringSt	strnRefNspc		( STR_NSPC_NSPC );
adtStringSt	strnRefConn		( STR_NSPC_CONN );
adtStringSt	strnRefConntr	( STR_NSPC_CONNTR );
adtStringSt	strnRefPers		( STR_NSPC_PERS );
adtStringSt	strnRefAct		( STR_NSPC_ACTIVE );
adtStringSt	strnRefRcvr		( STR_NSPC_RCVR );
adtStringSt	strnRefRef		( STR_NSPC_REF );

adtStringSt	strnRefFrom		( L"From" );
adtStringSt	strnRefTo		( L"To" );
adtStringSt	strnRefRO		( L"ReadOnly" );
adtStringSt	strnRefLoc		( L"Location" );
adtStringSt	strnRefKey		( L"Key" );
adtStringSt	strnRefVal		( L"Value" );


HRESULT nspcLoadPath ( IDictionary *pRoot, const WCHAR *wPath, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Load a value from a path of keys.
	//
	//	PARAMETERS
	//		-	pRoot is the top level dictionary
	//		-	wPath is the namespace location
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IDictionary	*pDct			= NULL;
	adtString	strName,strPath;
	adtValue		vAt;
	U32			cnt,i;
	adtIUnknown	unkV;
	WCHAR			*token,*n = NULL;

	// Global thread safety
	nspcglb.cs.enter();

	// Count number of tokens in path
	strPath = wPath;
	for (	token = WCSTOK ( &strPath.at(0), L"/", &n ),cnt = 0;
			token != NULL;
			token = WCSTOK ( NULL, L"/", &n ))
		++cnt;

	// Start at root
	adtValue::copy ( adtIUnknown(pRoot), v );

	// Retrieve/create each level of the path.
	strPath = wPath;
	for (	token = WCSTOK ( &strPath.at(0), L"/", &n ),i = 0;
			token != NULL && hr == S_OK;
			token = WCSTOK ( NULL, L"/", &n ),++i)
		{
		// Next name
		CCLOK  ( strName = token; )

		// Next dictionary
		CCLTRY  ( _QISAFE((unkV=v),IID_IDictionary,&pDct) );

		// Support relative path specifications (./ and ../)

		// Same level ?
		if (hr == S_OK && !WCASECMP(strName,L"."))
			{
			// Same level just means stay where we are
			hr = adtValue::copy ( adtIUnknown(pDct), v );
			}	// if

		// Up ?
		else if (hr == S_OK && !WCASECMP(strName,L".."))
			{
			// NOTE: nSpace specific.  There must be a 'parent' present in order
			// to support going 'up' in a hierarchy.
			CCLTRY ( pDct->load ( strnRefPar, v ) );

			// _Parent is stored as U64 to avoid circular reference count, convert
			// it to ptr. reference for this loop
			if (hr == S_OK && v.vtype == VTYPE_I8)
				hr = adtValue::copy ( adtIUnknown((IUnknown *)(U64)v.vlong), v );
			if (hr != S_OK)
				{
				dbgprintf ( L"WARNING: Relative 'up' specified but no parent %s\r\n", wPath );
				#ifdef	_DEBUG
				DebugBreak();
				#endif
				}	// if
			}	// if

		// Next level
		else
			{
			// Load path
			CCLTRY ( pDct->load ( strName, v ) );
			}	// else

		// Clean up
		_RELEASE(pDct);
		}	// for

	// Global thread safety
	nspcglb.cs.leave();

	return hr;
	}	// nspcLoadPath

HRESULT nspcTokens ( const WCHAR *wStr, const WCHAR *wDelim,  
								IList *pLst )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Generate a list of tokens in the string separated by the
	//			delimiters.
	//
	//	PARAMETERS
	//		-	wStr is the string to search
	//		-	wDelim is the list of delimiters
	//		-	pLst will receive the list
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	WCHAR			*token,*n	= NULL;
	adtString	strSrch(wStr);

	// Setup
	CCLTRYE ( pLst != NULL, ERROR_INVALID_STATE );
	CCLTRY  ( pLst->clear() );

	// Add tokens to list
	if (hr == S_OK)
		for (token = WCSTOK ( &strSrch.at(0), wDelim, &n );
				token != NULL;
				token = WCSTOK ( NULL, wDelim, &n ))
			pLst->write ( adtString(token) );

	return hr;
	}	// nspcTokens

HRESULT nspcStoreValue ( IDictionary *pRoot, const WCHAR *wPath, 
									const ADTVALUE &v, bool bPar )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Store a value to a path of keys.  Keys are created as necessary.
	//
	//	PARAMETERS
	//		-	pRoot is the top level dictionary
	//		-	wPath is the namespace location
	//		-	v contains the value to store
	//		-	bPar is true to store 'parent' values during dictionary creation
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IDictionary	*pDct			= NULL;
	U32			cnt,i;
	adtValue		vAt;
	adtString	strName,strPath;
	WCHAR			*token,*n	= NULL;

	// Global thread safety
	nspcglb.cs.enter();

	// Count number of tokens in path
	strPath = wPath;
	for (	token = WCSTOK ( &strPath.at(0), L"/", &n ),cnt = 0;
			token != NULL;
			token = WCSTOK ( NULL, L"/", &n ))
		++cnt;

	// Start at root
	CCLTRY ( adtValue::copy ( adtIUnknown(pRoot), vAt ) );

	// Retrieve/create each level of the path then store value under last key.
	strPath = wPath;
	for (	token = WCSTOK ( &strPath.at(0), L"/", &n ),i = 0;
			token != NULL && hr == S_OK;
			token = WCSTOK ( NULL, L"/", &n ),++i)
		{
		// Next name
		CCLOK  ( strName = token; )

		// Next dictionary
		CCLTRYE ( (vAt.vtype == VTYPE_UNK && vAt.punk != NULL), ERROR_NOT_FOUND );
		CCLTRY  ( _QI(vAt.punk,IID_IDictionary,&pDct) );

		// Support relative path specifications (./ and ../)

		// Same level ?
		if (hr == S_OK && strName[0] == '.' && strName[1] == '\0')
			{
			// Same level just means stay where we are
			hr = adtValue::copy ( adtIUnknown(pDct), vAt );
			}	// if

		// Up ?
		else if (hr == S_OK && strName[0] == '.' && strName[1] == '.' && strName[2] == '\0')
			{
			// NOTE: nSpace specific.  There must be a 'parent' present in order
			// to support going 'up' in a hierarchy.
			CCLTRY ( pDct->load ( strnRefPar, vAt ) );

			// _Parent is stored as U64 to avoid circular reference count, convert
			// it to ptr. reference for this loop
			if (hr == S_OK && vAt.vtype == VTYPE_I8)
				hr = adtValue::copy ( adtIUnknown((IUnknown *)(U64)vAt.vlong), vAt );
			if (hr != S_OK)
				{
				dbgprintf ( L"WARNING: Relative 'up' specified but no parent %s\r\n", wPath );
				#ifdef	_DEBUG
				DebugBreak();
				#endif
				}	// if
			}	// if

		// Next level
		else
			{
			// If not at last key, move to next level
			if (hr == S_OK && i+1 < cnt)
				{
				// Load next level
				CCLTRY ( pDct->load ( strName, vAt ) );

				// If not found create dictionary for next level
				if (hr == ERROR_NOT_FOUND)
					{
					IDictionary	*pDctSub	= NULL;
					hr							= S_OK;

					// New dictionary
//					if (nspcglb.pcfDct != NULL)
//						hr = nspcglb.pcfDct->CreateInstance ( NULL, IID_IDictionary, (void **) &pDctSub );
//					else
						hr = COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctSub );

					// Store parent (avoid reference count)
					if (hr == S_OK && bPar)
						hr = pDctSub->store ( strnRefPar, adtLong((U64)pDct) );

					// Current value
					CCLTRY ( adtValue::copy ( adtIUnknown(pDctSub), vAt ) );

					// Store under parent
					CCLTRY ( pDct->store ( strName, vAt ) );

					// Clean up
					_RELEASE(pDctSub);
					}	// if

				}	// if

			// If at last key in the path then store value
			else if (hr == S_OK && i+1 == cnt)
				hr = pDct->store ( strName, v );
			}	// else

		// Clean up
		_RELEASE(pDct);
		}	// for

	// Global thread safety
	nspcglb.cs.leave();

	return hr;
	}	// nspcStoreValue

HRESULT nspcPathTo ( IDictionary *pAt, const WCHAR *wPath, 
							adtString &nspcPathTo, IDictionary *pRoot )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert a namespace path to an 'full' path between the 'at'
	//			and 'root' locations.  Also 
	//			resolves/removes any '../' (up a location) locations.
	//
	//	PARAMETERS
	//		-	pAt is the current graph for the path
	//		-	wPath is the relative path
	//		-	nspcPathTo will receive the full namespace path
	//		-	pRoot is the root/'to' dictionary to stop search 
	//			(NULL to go to top of hierarchy)
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IDictionary	*pDct			= NULL;
	int			len			= 0;
	WCHAR			*up,*sm,*f,*t;
	adtValue		vL;

	// Use a local stack to keep track of all the names in the path
	// so that generation of the path can be done at once and lots
	// of calls to prepend can be avoided.

	// Sharing stack, must thread protect
	nspcglb.csAbs.enter();

	// Place initial path onto stack
	CCLTRY ( nspcglb.pStkAbs->clear() );
	CCLTRY ( nspcglb.pStkAbs->write ( adtString(wPath) ) );
	CCLOK  ( len = (int)wcslen(wPath)+1; )	// +1 is for extra slash at end

	// Setup
//	CCLOK ( nspcPathTo.at(0) = '\0'; )
//	CCLTRY( nspcPathTo.append ( wPath ) );
//	nspcPathTo = wPath;
	
	// Relative path ?
	if (wPath[0] != WCHAR('/'))
		{
		// Start with given location and at current level
		CCLOK ( pDct = pAt; )
		_ADDREF(pDct);

		// Append parent locations
		while (hr == S_OK && pDct != pRoot && pDct != NULL)
			{
			adtString	strName;

			// Length of slash separator
			len += 1;

			// Place next name onto stack
			if (	pDct->load ( strnRefName, vL ) == S_OK &&
					adtValue::type(vL) == VTYPE_STR			&&
					vL.pstr != NULL )
				{
				// Place on stack
				hr = nspcglb.pStkAbs->write ( vL );

				// Length of string
				len += (int)wcslen(vL.pstr);
				}	// if

			// Slash for next level up

			// Obtain name of current level
//			if (pDct->load ( strnRefName, vL ) == S_OK)
//				strName = vL;
//			else
//				strName = L"";

			// Prepend the next level name to the path
//			CCLTRY ( nspcPathTo.prepend ( L"/" ) );
//			CCLTRY ( nspcPathTo.prepend ( strName ) );
//			CCLTRY ( strName.append ( L"/" ) );
//			CCLTRY ( strName.append ( nspcPathTo ) );
//			CCLTRY ( adtValue::copy ( strName, nspcPathTo ) );

			// Obtain parent
			if (hr == S_OK && pDct->load ( strnRefPar, vL ) == S_OK)
				{
				adtIUnknown	unkV(vL);

				// Parent
				_RELEASE(pDct);
				CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDct) );
				}	// if
			else
				{
				_RELEASE(pDct);
				}	// else
			}	// while

		// Clean up
		_RELEASE (pDct);
		}	// if

	// Allocate string memory for entire path
	CCLTRY ( nspcPathTo.allocate ( len ) );

	// Pop stack and append strings
	while (hr == S_OK && nspcglb.pItAbs->read ( vL ) == S_OK)
		{
		// Slash and string (string type has already been validated)
		CCLTRY ( nspcPathTo.append ( vL.pstr ) );
		CCLTRY ( nspcPathTo.append ( L"/" ) );

		// Next string
		nspcglb.pItAbs->next();
		}	// while

	// Sharing stack, must thread protect
	nspcglb.csAbs.leave();

	// Last slash in invalid
	CCLOK ( nspcPathTo.at(nspcPathTo.length()-1) = '\0'; )

	// Collapse any 'up' (../) or any 'same' (./)
	sm = t = f = NULL;
	while (hr == S_OK && 
				(	((up = wcsstr ( &nspcPathTo.at(), L"/../" )) != NULL) ||
					((sm = wcsstr ( &nspcPathTo.at(), L"/./" )) != NULL) ) )
		{
		// 'Up' effectively removes the location above the current one.
		if (up != NULL)
			{
			// An 'up' specification is invalid at root
			CCLTRYE ( up > &nspcPathTo.at(), E_UNEXPECTED );
		
			// Find beginning of previous location
			for (t = up-1;hr == S_OK && *t != '/';--t)
				{}

			// Skip to characters after 'up'
			f = up+4;
			t = t+1;
			}	// if

		// 'Same' just stays at the same level
		else if (sm != NULL)
			{
			// Copy characters after slash to beginning of '.'
			f = sm+3;
			t = sm+1;
			}	// else if

		// Move characters characters back in string
		if (hr == S_OK)
			{
			while (*f != '\0')
				*t++ = *f++;
			*t = '\0';
			}	// if

		}	// while

	return hr;
	}	// nspcPathTo

