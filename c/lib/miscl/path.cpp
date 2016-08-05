////////////////////////////////////////////////////////////////////////
//
//									PATH.CPP
//
//					Implementation of the path node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"

Path :: Path ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pNames	= NULL;
	pNamesIt	= NULL;
	bAbs		= false;
	}	// Path

void Path :: destruct ( void )
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
	_RELEASE(pNamesIt);
	_RELEASE(pNames);
	}	// destruct

HRESULT Path :: onAttach ( bool bAttach )
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
	HRESULT			hr		= S_OK;

	// Attach
	if (bAttach)
		{
		adtValue vL;

		// Create list to use for the names in the path
		CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pNames ) );
		CCLTRY ( pNames->iterate ( &pNamesIt ) );

		// Defaults
		if (pnDesc->load(adtString(L"Path"),vL) == S_OK)
			strPath = vL;
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pNamesIt);
		_RELEASE(pNames);
		}	// else
	return hr;
	}	// onAttach

HRESULT Path :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IReceptor
	//
	//	PURPOSE
	//		-	A location has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	pl is the location
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Up.  Move 'up' one level in the path.
	if (_RCP(Up))
		{
		adtString	strRes;
		U32			i,cnt;
		adtValue		vN;

		// If path originally had root separator, preserve it.
		if (hr == S_OK && bAbs)
			hr = strRes.append ( L"/" );

		// Re-generate path minus the last name
		CCLTRY ( pNames->size ( &cnt ) );
		CCLTRY ( pNamesIt->begin() );
		for (i = 0;hr == S_OK && i < cnt-1;++i)
			{
			adtString strN;

			// Next entry
			CCLTRY ( pNamesIt->read ( vN ) );
			CCLTRYE( (strN = vN).length() > 0, E_UNEXPECTED );

			// Continue path
			CCLTRY ( strRes.append ( strN ) );
			CCLTRY ( strRes.append ( L"/" ) );

			// Next entry in list
			CCLOK ( pNamesIt->next(); )
			}	// for

		// If original path was not a location, remove trailing slash
		if (hr == S_OK && !bLoc)
			strRes.at(strRes.length()-1) = '\0';

		// Result
		CCLOK ( _EMT(Path,strRes); )
		}	// if

	// State
	else if (_RCP(Path))
		{
		int	len;

		// Active path
		strPath	= v;
		len		= strPath.length();

		// Break path into its names
		CCLTRY ( pNames->clear() );
		CCLTRY ( nspcTokens ( strPath, L"/\\", pNames ) );

		// Absolute or location path ?
		bAbs = bLoc = false;
		if (hr == S_OK && len > 0)
			{
			CCLOK ( bAbs = (strPath[0] == '/' || strPath[0] == '\\'); )
			CCLOK ( bLoc = (strPath[len-1] == '/' || strPath[len-1] == '\\'); )
			}	// if
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

