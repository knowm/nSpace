////////////////////////////////////////////////////////////////////////
//
//									TEMPLOC.CPP
//
//				Implementation of the temporal location
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"
#include <stdio.h>

// Globals
extern GlobalNspc	nspcglb;

TemporalLoc :: TemporalLoc ( TemporalImpl *_pTmp, const WCHAR *_pwLoc,
										bool _bRO )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pTmp is the temporal implementation backing dictionary
	//		-	_pwLoc is the namespace location of the dictionary
	//		-	_bRO is true for read-only, false for writable
	//
	////////////////////////////////////////////////////////////////////////
	pTmp			= _pTmp; _ADDREF(pTmp);
	strLoc		= _pwLoc;
	strLoc.at();
	bRO			= _bRO;
	punkLoc		= NULL;
	pLocDct		= NULL;
	pLocRcp		= NULL;
	pLocLoc		= NULL;
	}	// TemporalLoc

HRESULT TemporalLoc :: clear ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Resets the container.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	IIt		*pIt	= NULL;
	adtValue	vK;

	// Clear a dictionary removes all the values in the dictionary.
	// Remove each entry individually for simplication.

	// Iterate the keys for values
	CCLTRY ( pKeys->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		// Debug
//		if (vK.vtype == VTYPE_STR)
//			dbgprintf ( L"TemporalLoc::clear:%s\r\n", vK.pstr );

		// Remove the key
		CCLOK ( remove ( vK ); )

		// The key should already be removed but in case of failure
		// it is removed here to ensure loop terminates.
		CCLOK ( pKeys->remove ( vK ); )

		// Reset iteration since key will be removed
		pIt->begin();
		}	// while

	// Clean up
	_RELEASE(pIt);

	return hr;
	}	// clear

HRESULT TemporalLoc :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	CCLObject
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	IIt		*pItLocs	= NULL;
	adtValue	vL;

	// Aggregate a default locaton
	CCLTRY ( COCREATEA(L"Nspc.Location",(ILocation *)this,&punkLoc) );

	// Interfaces, do not hold reference
	CCLTRY ( _QI(punkLoc,IID_IDictionary,&pLocDct) );
	CCLOK  ( pLocDct->Release(); )
	CCLTRY ( _QI(punkLoc,IID_IReceptor,&pLocRcp) );
	CCLOK  ( pLocRcp->Release(); )
	CCLTRY ( _QI(punkLoc,IID_ILocation,&pLocLoc) );
	CCLOK  ( pLocLoc->Release(); )

	// Debug
//	dbgprintf(L"TemporalLoc::construct:%s:punkLoc %p pLocRcp %p\r\n", (LPCWSTR)strLoc, punkLoc, pLocRcp);

	//
	// Keys.
	// Pre-load the list of valid keys for this location and mark
	// as sub-locations or values.
	//

	// Create a contained dictionary for storing items
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pKeys ) );

	// Process list of locations
	if (hr == S_OK && pTmp->locations ( strLoc, &pItLocs ) == S_OK)
		{
		while (hr == S_OK && pItLocs->read ( vL ) == S_OK)
			{
			adtBool		bLoc = false;
			adtString	strL(vL);
			int			len;

			// A temporal sub-location has a trailing slash.
			bLoc = ((len = strL.length()) > 0 && strL[len-1] == '/');

			// Remove trailing slash for keys
			if (bLoc) strL.at(strL.length()-1) = '\0';

			// Store key
			CCLTRY ( pKeys->store ( strL, bLoc ) );

			// Clean up
			pItLocs->next();
			}	// while
		}	// if

	// Clean up
	_RELEASE(pItLocs);

	return hr;
	}	// construct

HRESULT TemporalLoc :: copyTo ( IContainer *pCont )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ICopyTo
	//
	//	PURPOSE
	//		-	Copies values from this container to another container.
	//
	//	PARAMETERS
	//		-	pCont will receive the values
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pDst	= NULL;
	IIt			*pIt	= NULL;
	adtValue		vK,vV;

	// Destination dictionary
	CCLTRY ( _QISAFE(pCont,IID_IDictionary,&pDst) );

	// Iterate own keys and copy values to destination
	CCLTRY ( pKeys->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		// Access own value and store in destination
		// Ignore empty(removed) values
		if (load ( vK, vV ) == S_OK && !adtValue::empty(vV))
			hr = pDst->store ( vK, vV );

		// Clean up
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pDst);

	return hr;
	}	// copyTo

HRESULT TemporalLoc :: create ( const WCHAR *pwKey, ILocation **ppLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ILocation
	//
	//	PURPOSE
	//		-	Called to create a new location of the same type.
	//
	//	PARAMETERS
	//		-	pwKey is the key name that will be used for the location.
	//		-	ppLoc will receive the location
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Create new location object
	if (hr == S_OK && (*ppLoc) == NULL)
		{
		TemporalLoc	*pLoc	= NULL;
		adtString	strPath;

		// Generate what the path will be for the new location
		CCLTRY ( adtValue::copy ( strLoc, strPath ) );
		CCLTRY ( strPath.append ( pwKey ) );
		CCLTRY ( strPath.append ( L"/" ) );

		// Create temporal location
		CCLTRYE( (pLoc = new TemporalLoc(pTmp,strPath,bRO)) != NULL, E_OUTOFMEMORY );
		CCLOK  ( pLoc->AddRef(); )
		CCLTRY ( pLoc->construct() );

		// Result
		CCLTRY ( _QI(pLoc,IID_ILocation,ppLoc) );

		// Clean up
		_RELEASE(pLoc);
		}	// if

	// Default behaviour
	CCLTRY ( pLocLoc->create ( pwKey, ppLoc ) );

	return hr;
	}	// create

void TemporalLoc :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
//	if (strLoc.length() > 0)
//		dbgprintf(L"TemporalLoc::destruct:%s {\r\n", (LPCWSTR)strLoc );

	// Clean up
	_RELEASE(pTmp);
	_RELEASE(pKeys);
	_RELEASE(punkLoc);

//	if (strLoc.length ( ) > 0)
//		dbgprintf ( L"} TemporalLoc::destruct:%s\r\n", (LPCWSTR)strLoc );
	}	// destruct

HRESULT TemporalLoc :: isEmpty ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns the empty state of the container
	//
	//	RETURN VALUE
	//		S_OK if container is empty
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	bool		bEmpty	= true;
	IIt		*pKeys	= NULL;
	adtValue	vK,vV;

	// In order to determine if a temporal location is truly empty, all
	// the values must be loaded to check for any non-empty values.
	CCLTRY ( keys ( &pKeys ) );
	while (hr == S_OK && bEmpty && pKeys->read ( vK ) == S_OK)
		{
		// Value at key
		CCLTRY ( load ( vK, vV ) );

		// If non-empty value then not empty
		if (hr == S_OK && !adtValue::empty(vV))
			bEmpty = false;

		// Next entry
		pKeys->next();
		}	// while

	// Clean up
	_RELEASE(pKeys);

	return (bEmpty) ? S_OK : S_FALSE;
	}	// isEmpty

HRESULT TemporalLoc :: iterate ( IIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns an iterator for the container.
	//
	//	PARAMETERS
	//		-	ppIt will receive a ptr. to the iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	TemporalLocIt	*pIt	= NULL;

	// State check
	CCLTRYE ( pKeys != NULL, ERROR_INVALID_STATE );

	// Create emitter for caller
	CCLTRYE ( (pIt = new TemporalLocIt(this)) != NULL, E_OUTOFMEMORY );
	CCLTRY ( pIt->construct() );

	// Result
	(*ppIt)	= pIt;
	_ADDREF(*ppIt);

	return hr;
	}	// iterate

HRESULT TemporalLoc :: keys ( IIt **ppKeys )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITemporalLoc
	//
	//	PURPOSE
	//		-	Returns an object to iterate through the keys in the tree.
	//
	//	PARAMETERS
	//		-	ppKeys will receive the iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;

	// State check
	CCLTRYE ( pKeys != NULL, ERROR_INVALID_STATE );

	// Iterate the valid keys
	CCLTRY ( pKeys->keys ( ppKeys ) );

	return hr;
	}	// keys

HRESULT TemporalLoc :: load ( const ADTVALUE &vKey, ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITemporalLoc
	//
	//	PURPOSE
	//		-	Loads a value from the TemporalLoc with the given key.
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	adtBool		bLoc	= false;
	adtValue		vL;
	adtString	strKey(vKey);
	adtString	strKeyLoc;

	// Debug
//	if (!WCASECMP(strKey,L"_Connectors"))
//		dbgprintf ( L"Hi\r\n" );

	// State check
	CCLTRYE ( pKeys != NULL, ERROR_INVALID_STATE );

	// Is value for key already cached ?
	if (pLocDct->load ( vKey, vValue ) == S_OK)
		return S_OK;

	// Valid key ?
	CCLTRY ( pKeys->load ( vKey, vL ) );

	// If key is a location and it is not yet cached, create a temporal
	// sub-location and place at key.
	if (hr == S_OK && (bLoc = vL) == true)
		{
		ILocation	*pLoc		= NULL;

		// Create new temporal location
		CCLTRY ( create ( strKey, &pLoc ) );

		// Result value
		CCLTRY ( adtValue::copy ( adtIUnknown(pLoc), vValue ) );

		// Store in cache
		CCLTRY ( pLocDct->store ( strKey, vValue ) );

		// Clean up
		_RELEASE(pLoc);
		}	// if

	// Key is a value, load it
	else if (hr == S_OK)
		{
		U64			uId;
		HDRIDX		hdrI;
		HDRLOC		hdrE;

		// Generate full path to key
		CCLTRY ( adtValue::copy ( strLoc, strKeyLoc ) );
		CCLTRY ( strKeyLoc.append ( strKey ) );
		CCLTRY ( strKeyLoc.append ( L"/" ) );

		// Obtain Id for location
		CCLTRY ( pTmp->locId ( strKeyLoc, true, &uId ) );

		// By default the latest value is always loaded.

		// Read the latest sequence number
		CCLTRY ( pTmp->locGet ( uId, &hdrI ) );

		// Read header
		CCLTRY ( pTmp->locGet ( hdrI.last, &hdrE ) );

		// Read value
		CCLTRY ( pTmp->locGet ( hdrE.value, vValue ) );

		// Cache value if valid
		if (hr == S_OK && !adtValue::empty(vValue))
			hr = pLocDct->store ( strKey, vValue );
		}	// else if

	return hr;
	}	// load

HRESULT TemporalLoc :: remove ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Removes an item from the container identified by the specified
	//			value.
	//
	//	PARAMETERS
	//		-	v identifies the key to remove
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	adtValue	vV;

	// Load key type to see if it is a location
	CCLTRY ( pKeys->load ( v, vV ) );

	// A location will have to delete its own contents so load and clear it
	if (hr == S_OK && adtBool(vV) == true)
		{
		// Load the location
		if (load ( v, vV ) == S_OK)
			{
			IDictionary *pDct	= NULL;
			adtIUnknown	unkV(vV);

			// Clear contents
			CCLTRY(_QISAFE((unkV=vV),IID_IDictionary,&pDct));
			CCLOK (pDct->clear();)
			_RELEASE(pDct);
			}	// if

		// Remove from contained location
		CCLOK ( pLocDct->remove ( v ); )

		// Invalid key
		CCLOK ( pKeys->remove ( v ); )
		}	// if

	// In order to retain temporal history of value, a value is 'removed'
	// by simply storing a empty value as the latest value.
	else
		hr = store ( v, adtValue() );

	return hr;
	}	// remove

HRESULT TemporalLoc :: size ( U32 *s )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns the # of items in the container.
	//
	//	PARAMETERS
	//		-	s will return the size of the container
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return pKeys->size(s);
	}	// size

HRESULT TemporalLoc :: store (	const ADTVALUE &vKey, 
											const ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITemporalLoc
	//
	//	PURPOSE
	//		-	Stores a value in the TemporalLoc with the given key.
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	ILocation	*pSub	= NULL;
	adtString	strKey(vKey);
	adtString	strKeyLoc;
	U64			uId;
	bool			bP;

	// Certain internal, run-time only keys are not persisted
	bP = (	(	strKey[0] != '_' ) || 
				(	WCASECMP(strKey,STR_NSPC_PARENT) &&
					WCASECMP(strKey,STR_NSPC_NSPC)	&&
					WCASECMP(strKey,STR_NSPC_ACTIVE)	&&
					WCASECMP(strKey,STR_NSPC_NAME) ) );

	// Sub-locations are not directly persisted as they
	// will take care of themselves.
	if (bP)
		bP = (	adtValue::type(vValue) != VTYPE_UNK ||
					vValue.punk == NULL						||
					_QI(vValue.punk,IID_ILocation,&pSub) != S_OK );

	// In order to avoid storing duplicate values, compare 'simple'
	// values and do not store if the same.
	if (bP && adtValue::type(vValue) != VTYPE_UNK)
		{
		adtValue	vNow;

		// Compare on load
		if (	load ( strKey, vNow ) == S_OK		&&
				adtValue::compare ( vNow, vValue ) == 0 )
			bP = false;
		}	// if

	// Persist key/value pair.  
	if (hr == S_OK && bP)
		{
		// Generate full path to values
		CCLTRY ( adtValue::copy ( strLoc, strKeyLoc ) );
		CCLTRY ( strKeyLoc.append ( strKey ) );
		CCLTRY ( strKeyLoc.append ( L"/" ) );

		// Obtain Id for location
		CCLTRY ( pTmp->locId ( strKeyLoc, false, &uId ) );

		// Store value at latest version
		CCLTRY ( pTmp->locPut ( uId, vValue ) );
		}	// if

	// Update cache
	if (hr == S_OK)
		{
		// Empty value means remove
		if (!adtValue::empty(vValue))
			{
			// Cache in contained location
			CCLTRY ( pLocDct->store ( strKey, vValue ) );

			// Valid key
			CCLTRY ( pKeys->store ( strKey, adtBool(false) ) );
			}	// if
		else
			{
			// Remove from contained location
			pLocDct->remove ( strKey );

			// Invalid key
			pKeys->remove ( strKey );
			}	// else
		}	// if

	// Clean up
	_RELEASE(pSub);

	return hr;
	}	// store

//
// Unmodified contained functions
//

HRESULT TemporalLoc :: connect ( IReceptor *pR, bool bC, bool bM )
	{
	return pLocLoc->connect ( pR, bC, bM );
	}	// connect

HRESULT TemporalLoc :: connected ( IReceptor *pR, bool bC, bool bM )
	{
	return pLocLoc->connected ( pR, bC, bM );
	}	// connected

HRESULT TemporalLoc :: receive ( IReceptor *prSrc, const WCHAR *pwLoc, 
											const ADTVALUE &v )
	{
	return pLocRcp->receive ( prSrc, pwLoc, v );
	}	// receive

HRESULT TemporalLoc :: reflect ( const WCHAR *pwLoc, IReceptor *prDst )
	{
	return pLocLoc->reflect ( pwLoc, prDst );
	}	// reflect

HRESULT TemporalLoc :: stored ( ILocation *pAt, bool bAtRcp, IReceptor *prSrc,
										const WCHAR *pwKey, const ADTVALUE &vValue )
	{
	return pLocLoc->stored ( pAt, bAtRcp, prSrc, pwKey, vValue );
	}	// stored

/////////////////
// TemporalLocIt
/////////////////

TemporalLocIt :: TemporalLocIt ( TemporalLoc *_pDct )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pDct is the parent
	//
	////////////////////////////////////////////////////////////////////////
	pDct		= _pDct;
	_ADDREF(pDct);
	pItKeys	= NULL;
	}	// TemporalLocIt

HRESULT TemporalLocIt :: begin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Resets the iterator position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;

	// Previous iteration
	_RELEASE(pItKeys);

	// Iterate the list of valid keys
	CCLTRY ( pDct->keys ( &pItKeys ) );

	return hr;
	}	// begin

HRESULT TemporalLocIt :: construct ( void )
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
	HRESULT		hr			= S_OK;

	// Initialize to beginning
	CCLOK ( begin(); )

	return hr;
	}	// construct

void TemporalLocIt :: destruct ( void )
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
	_RELEASE(pDct);
	}	// destruct

HRESULT TemporalLocIt :: end ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Resets the iterator position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// End of iteration
	_RELEASE(pItKeys);

	return hr;
	}	// end

HRESULT TemporalLocIt :: next ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Moves to the next position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr 		= S_OK;

	// State check
	CCLTRYE ( pItKeys != 0, ERROR_INVALID_STATE );

	// Next location
	CCLTRY ( pItKeys->next() );

	return hr;
	}	// next

HRESULT TemporalLocIt :: prev ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Moves to the previous position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// State check
	CCLTRYE ( pItKeys != 0, ERROR_INVALID_STATE );

	// Previous location
	CCLTRY ( pItKeys->prev() );

	return hr;
	}	// prev

HRESULT TemporalLocIt :: read ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Reads the current item from the container.
	//
	//	PARAMETERS
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtValue		vKey;

	// State check
	CCLTRYE ( pItKeys != NULL,	ERROR_INVALID_STATE );

	// Read the next key
	CCLTRY ( pItKeys->read ( vKey ) );

	// Load the value for the key
	CCLTRY ( pDct->load ( vKey, v ) );

	return hr;
	}	// read


