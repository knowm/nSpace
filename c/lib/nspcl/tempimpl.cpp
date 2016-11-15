////////////////////////////////////////////////////////////////////////
//
//								tempimpl.CPP
//
//				Default implementation of the temporal nSpace object.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"
#include <stdio.h>

// Globals
extern GlobalNspc	nspcglb;

TemporalImpl :: TemporalImpl ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pStmSrc		= NULL;
	pStmIdx		= NULL;
	pStmHdr		= NULL;
	pStmVal		= NULL;
	pDctMap		= NULL;
	uSeqNum		= 1;
	uLocId		= 1;
	pDctOpts		= NULL;

	pPrsrL		= NULL;
	pPrsrS		= NULL;
	}	// TemporalImpl

HRESULT TemporalImpl :: construct ( void )
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
	HRESULT	hr		= S_OK;

	// Create parsers used during load/save
	CCLTRY(COCREATE(L"Io.StmPrsBin",IID_IStreamPersist,&pPrsrL));
	CCLTRY(COCREATE(L"Io.StmPrsBin",IID_IStreamPersist,&pPrsrS));

	// Dictionary options
	CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctOpts));

	return hr;
	}	// construct

void TemporalImpl :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////

	// Clean up
	_RELEASE(pPrsrL);
	_RELEASE(pPrsrS);
	_RELEASE(pDctOpts);
	_RELEASE(pDctMap);
	_RELEASE(pStmIdx);
	_RELEASE(pStmHdr);
	_RELEASE(pStmVal);
	_RELEASE(pStmSrc);
	}	// destruct

HRESULT TemporalImpl :: locGet ( U64 uId, HDRIDX *pHdr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Obtain the index header.
	//
	//	PARAMETERS
	//		-	uId is the location Id
	//		-	hdr will receive the information
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	U64			uRd;

	// Thread safety
	csIO.enter();

	// Seek to correct position for location
	CCLTRY ( pStmIdx->seek ( uId*sizeof(HDRIDX), STREAM_SEEK_SET, &uRd ) );
	CCLTRYE( uRd == uId*sizeof(HDRIDX), E_UNEXPECTED );

	// Read header
	CCLTRY ( pStmIdx->read ( pHdr, sizeof(HDRIDX), &uRd ) );
	CCLTRYE( uRd == sizeof(HDRIDX), E_UNEXPECTED );

	// Thread safety
	csIO.leave();

	return hr;
	}	// locGet

HRESULT TemporalImpl :: locGet ( NSSQNM uSq, HDRLOC *pHdr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Obtain the emission header for a sequence number.
	//
	//	PARAMETERS
	//		-	uSq is the sequence number
	//		-	pHdr will receive the header information
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	U64			uRd;

	// Thread safety
	csIO.enter();

	// Valid sequence number ?
	CCLTRYE ( uSq != 0, ERROR_NOT_FOUND );

	// Seek to correct position for sequence number
	CCLTRY ( pStmHdr->seek ( uSq*sizeof(HDRLOC), STREAM_SEEK_SET, &uRd ) );
	CCLTRYE( uRd == uSq*sizeof(HDRLOC), E_UNEXPECTED );

	// Read header
	CCLTRY ( pStmHdr->read ( pHdr, sizeof(HDRLOC), &uRd ) );
	CCLTRYE( uRd == sizeof(HDRLOC), E_UNEXPECTED );

	// Thread safety
	csIO.leave();

	return hr;
	}	// locGet

HRESULT TemporalImpl :: locGet ( U64 uPos, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Read a stored value from the specified position.
	//
	//	PARAMETERS
	//		-	uPos is the value position
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	U64			uRd;

	// Thread safety
	csIO.enter();

	// Seek to correct position for value (skip over header)
	CCLTRY ( pStmVal->seek ( uPos+sizeof(HDRVAL), STREAM_SEEK_SET, &uRd ) );
	CCLTRYE( uRd == uPos+sizeof(HDRVAL), E_UNEXPECTED );

	// Load value from position
	CCLTRY ( pPrsrL->load ( pStmVal, v ) );

	// Thread safety
	csIO.leave();

	return hr;
	}	// locGet

HRESULT TemporalImpl :: locId ( const WCHAR *path, bool bRO,
											U64 *pId )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Retrieve or create the location Id for the given path.
	//
	//	PARAMETERS
	//		-	path is the path of the location
	//		-	bRO is true for read-only, false for write/create
	//		-	pId will receive the location Id
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtValue		v;
	adtLong		vId;

	// Debug
//	if (wcsstr ( path, L"_Parent") != NULL)
//		dbgprintf ( L"Hi\r\n" );

	// Attempt access on location
	CCLTRY ( nspcLoadPath ( pDctMap, path, v ) );
//	CCLTRY ( pDctMap->load ( adtString(path), v ) );

	// Caller is asking for a location than is currently just a path
	CCLTRYE( v.vtype == VTYPE_I8, E_UNEXPECTED );
	CCLOK  ( vId = v; )

	// Not found but write requested
	if (hr == ERROR_NOT_FOUND && !bRO)
		{
		HDRIDX	hdrI;

		// Reset status
		hr = S_OK;

		// Next available location Id
		CCLOK ( vId = uLocId++; )

		// Store location Id map
		CCLTRY ( nspcStoreValue ( pDctMap, path, vId, false ) );
//		CCLTRY ( pDctMap->store ( adtString(path), vId ) );

		// Write a blank location index header for new location
		CCLOK ( memset ( &hdrI, 0, sizeof(hdrI) ); )
		CCLTRY( locPut ( vId, &hdrI ) );

		// The first value that is stored is the namespace path of the location.
		CCLTRY ( locPut ( vId, adtString(path) ) );

		// Storing value sets 'last' to the latest value.  Update header by 
		// setting the 'first' position to the same value.
		CCLTRY ( locGet ( vId, &hdrI ) );
		CCLOK ( hdrI.first = hdrI.last; )
		CCLTRY( locPut ( vId, &hdrI ) );
		}	// if

	// Location Id
	CCLOK ( *pId = vId; )

	return hr;
	}	// locId

HRESULT TemporalImpl :: locPut ( U64 uId, const HDRIDX *pHdr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Save the index header.
	//
	//	PARAMETERS
	//		-	uId is the location Id
	//		-	pHdr container the information
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	U64			uWr;

	// Thread safety
	csIO.enter();

	// Seek to correct position for location
	CCLTRY ( pStmIdx->seek ( uId*sizeof(HDRIDX), STREAM_SEEK_SET, &uWr ) );
	CCLTRYE( uWr == uId*sizeof(HDRIDX), E_UNEXPECTED );

	// Write header
	CCLTRY ( pStmIdx->write ( pHdr, sizeof(HDRIDX), &uWr ) );
	CCLTRYE( uWr == sizeof(HDRIDX), E_UNEXPECTED );

	// Thread safety
	csIO.leave();

	return hr;
	}	// locPut

HRESULT TemporalImpl :: locPut ( NSSQNM uSq, const HDRLOC *pHdr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Save the emission header for a sequence number.
	//
	//	PARAMETERS
	//		-	uSq is the sequence number
	//		-	pHdr container the information
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	U64			uWr;

	// Thread safety
	csIO.enter();

	// Valid sequence number ?
	CCLTRYE ( uSq != 0, ERROR_NOT_FOUND );

	// Seek to correct position for sequence number
	CCLTRY ( pStmHdr->seek ( uSq*sizeof(HDRLOC), STREAM_SEEK_SET, &uWr ) );
	CCLTRYE( uWr == uSq*sizeof(HDRLOC), E_UNEXPECTED );

	// Write header
	CCLTRY ( pStmHdr->write ( pHdr, sizeof(HDRLOC), &uWr ) );
	CCLTRYE( uWr == sizeof(HDRLOC), E_UNEXPECTED );

	// Thread safety
	csIO.leave();

	return hr;
	}	// locPut

HRESULT TemporalImpl :: locPut ( U64 uId, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to write a value to a location stream.
	//
	//	PARAMETERS
	//		-	uId is the location Id
	//		-	v is the value to store.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U64		uIo,uPos;
	HDRVAL	hdrV;
	HDRLOC	hdrE;
	HDRIDX	hdrI;
	NSSQNM	uSeq;

	// Thread safety
	csIO.enter();

	// Next sequence number
	uSeq = uSeqNum++;

	//
	// Value
	//
	if (hr == S_OK)
		{
		// Obtain end-of-file position for next value
		CCLTRY ( pStmVal->seek ( 0, STREAM_SEEK_END, &uPos ) );

		// Save value header
		hdrV.lid		= uId;
		hdrV.seqnum = uSeq;
		CCLTRY ( pStmVal->write ( &hdrV, sizeof(hdrV), &uIo ) );
		CCLTRYE( uIo == sizeof(hdrV), E_UNEXPECTED );

		// Save value
		CCLTRY ( pPrsrS->save ( pStmVal, v ) );
		}	// if

	// Read location index header 
	CCLTRY ( locGet ( uId, &hdrI ) );

	//
	// Location header
	//

	// Previous emission ?
	if (hr == S_OK && hdrI.last != 0)
		{
		// Read latest emission
		CCLTRY ( locGet ( hdrI.last, &hdrE ) );

		// New latest sequence number
		CCLOK ( hdrE.next = uSeq; )

		// Update
		CCLTRY ( locPut ( hdrI.last, &hdrE ) );
		}	// if

	// Save latest emission header
	if (hr == S_OK)
		{
		hdrE.lid		= uId;
		hdrE.value	= uPos;
		hdrE.prev	= hdrI.last;
		hdrE.next	= 0;
		CCLTRY ( locPut ( uSeq, &hdrE ) );
		}	// if

	// New latest value
	CCLOK ( hdrI.last = uSeq; )

	// Write index header.
	CCLTRY ( locPut ( uId, &hdrI ) );

	// Thread safety
	csIO.leave();

	return hr;
	}	// locPut
/*
HRESULT TemporalImpl :: emitter ( const WCHAR *path, bool bWrite,
												IDictionary **ppD )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	ITemporalImpl
	//
	//	PURPOSE
	//		-	Open access to a TemporalImplized emitter.
	//
	//	PARAMETERS
	//		-	path is the namespace path of the emitter
	//		-	bWrite is true for writable access.
	//		-	ppD will receive a ptr. to the emitter
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	}	// emitter
*/
HRESULT TemporalImpl :: locations ( const WCHAR *wLoc, IIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	ILocations
	//
	//	PURPOSE
	//		-	Returns an iterator for the sub-locations at the 
	//			specified location.
	//
	//	PARAMETERS
	//		-	wLoc specifies the location
	//		-	ppIt will receive an location iterator.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pLoc	= NULL;
	IList			*pLst	= NULL;
	IIt			*pIt	= NULL;
	adtValue		v;
	adtIUnknown	unkV;

	// Create a list to receive the list of locations
	CCLTRY(COCREATE(L"Adt.List",IID_IList,&pLst));

	// Obtain location in map
	CCLTRY ( nspcLoadPath ( pDctMap, wLoc, v ) );
//	CCLTRY ( pDctMap->load ( adtString(wLoc), v ) );

	// A location is a dictionary
	CCLTRY ( _QISAFE((unkV = v),IID_IDictionary,&pLoc) );
	CCLTRY ( pLoc->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( v ) == S_OK)
		{
		adtString	vK(v),vV;

		// Value at location
		CCLTRY ( pLoc->load ( vK, vV ) );

		// Ignore parent that stored in path hierarchy
		if (hr == S_OK && WCASECMP(vK,STR_NSPC_PARENT))
			{
			// A number at location means path is a value
			if (hr == S_OK && vV.vtype == VTYPE_I8)
				{
				// Add name to list
				CCLTRY ( pLst->write ( vK ) );
				}	// if

			// An object (dictionary) means path is a location
			else if (hr == S_OK && vV.vtype == VTYPE_UNK)
				{
				// Locations are identified by trailing slashes
				CCLTRY ( vK.append ( L"/" ) );

				// Add name to list
				CCLTRY ( pLst->write ( vK ) );
				}	// else if
			}	// if

		// Next location
		pIt->next();
		}	// while

	// Result
	CCLTRY ( pLst->iterate ( ppIt ) );

	// Clean up
	_RELEASE(pLst);
	_RELEASE(pIt);
	_RELEASE(pLoc);

	return hr;
	}	// locations

HRESULT TemporalImpl :: open ( IDictionary *pOpts )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM		ITemporalImpl
	//
	//	PURPOSE
	//		-	Open a TemporalImpl database.
	//
	//	PARAMETERS
	//		-	pOpts is a ptr. to the options dictionary
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IDictionary	*pStmOpts	= NULL;
	IByteStream	*pStmMap		= NULL;
	IUnknown		*pLoc			= NULL;
	adtValue		v;
	adtString	strFile;
	adtIUnknown	unkV;
	U64			pos;
	bool			bMap;

	// Stream locations (required)
	CCLTRY ( pOpts->load ( adtStringSt(L"Locations"), v ) );
	CCLTRY ( _QISAFE(v,IID_ILocations,&pStmSrc) );

	// Location inside stream source for root of database (required)
	CCLTRY ( pOpts->load ( adtStringSt(L"Location"), v ) );
	CCLTRYE( (strLoc = v).length() > 0, E_UNEXPECTED );

	// Options for streams
	CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pStmOpts));

	//
	// LOCATION.NDX contains the sequence number of the first/lastest value
	// of the location.
	//

	// Location of file
	CCLOK  ( strFile = strLoc; )
	CCLTRY ( strFile.append ( L"location.ndx" ) );

	// Access stream
	lprintf ( LOG_DBG, L"Location index:%s", (LPCWSTR) strFile );
	CCLTRY ( pStmOpts->clear() );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Location"),	strFile ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"ReadOnly"),	adtBool(false) ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Create"),	adtBool(true) ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Truncate"),	adtBool(false) ) );
	CCLTRY ( pStmSrc->open ( pStmOpts, &pLoc ) );
	CCLTRY ( _QI(pLoc,IID_IByteStream,&pStmIdx) );
	_RELEASE(pLoc);

	// The size of the file will correlate with the next available location Id.
	CCLTRY ( pStmIdx->seek ( 0, STREAM_SEEK_END, &uLocId ) );
	if (hr == S_OK && uLocId == 0)
		{
		HDRIDX	idx;

		// If the size if zero, write version of file into zero entry
		idx.first	= 0x1;
		idx.last		= 0x0;
		uLocId		= sizeof(idx);

		// Write header
		CCLTRY ( pStmIdx->write ( &idx, sizeof(idx), &uLocId ) );
		}	// if

	// Compute next available location Id
	uLocId = uLocId/sizeof(HDRIDX);

	//
	// LOCATION.NBN contains fixed size emission headers.  Position in
	// file is correlated with the sequence number.  Each header contains
	//	pointer to value in value file.
	//

	// Location of file
	CCLOK  ( strFile = strLoc; )
	CCLTRY ( strFile.append ( L"location.nbn" ) );

	// Access stream
	lprintf ( LOG_DBG, L"Location hdrs:%s", (LPCWSTR) strFile );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Location"),	strFile ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"ReadOnly"),	adtBool(false) ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Create"),	adtBool(true) ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Truncate"),	adtBool(false) ) );
	CCLTRY ( pStmSrc->open ( pStmOpts, &pLoc ) );
	CCLTRY ( _QI(pLoc,IID_IByteStream,&pStmHdr) );
	_RELEASE(pLoc);

	// The size of the file will provide the next available sequence number.
	CCLTRY ( pStmHdr->seek ( 0, STREAM_SEEK_END, &uSeqNum ) );
	if (hr == S_OK && uSeqNum == 0)
		{
		// If the size if zero, write version of file into zero entry
		HDRLOC	hdr;

		// Write version
		memset ( &hdr, 0, sizeof(hdr) );
		hdr.lid	= 0x1;
		CCLTRY ( pStmHdr->write ( &hdr, sizeof(hdr), &uSeqNum ) );
		}	// if

	// Compute next available sequence number
	uSeqNum = uSeqNum/sizeof(HDRLOC);

	//
	// VALUE.NBN.  Recorded values for location, variable size entries.
	//

	// Location of file
	CCLOK  ( strFile = strLoc; )
	CCLTRY ( strFile.append ( L"value.nbn" ) );
	lprintf ( LOG_DBG, L"Location vals:%s", (LPCWSTR) strFile );

	// Access stream
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Location"),	strFile ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"ReadOnly"),	adtBool(false) ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Create"),	adtBool(true) ) );
	CCLTRY ( pStmOpts->store ( adtStringSt(L"Truncate"),	adtBool(false) ) );
	CCLTRY ( pStmSrc->open ( pStmOpts, &pLoc ) );
	CCLTRY ( _QI(pLoc,IID_IByteStream,&pStmVal) );
	_RELEASE(pLoc);

	// If a new value, store version as first value.
	CCLTRY ( pStmVal->seek ( 0, STREAM_SEEK_END, &pos ) );
	if (hr == S_OK && pos == 0)
		{
		HDRVAL	val;
		val.lid		= 1;
		val.seqnum	= 0;
		CCLTRY ( pStmVal->write ( &val, sizeof(val), NULL ) );
		}	// if

	//
	// LOCATION.MAP
	//

	// Location of file
//	CCLOK  ( strFile = strLoc; )
//	CCLTRY ( strFile.append ( L"location.nmp" ) );
//	lprintf ( LOG_DBG, L"Location map:%s", (LPCWSTR) strFile );

	// Access stream
//	CCLTRY ( pStmOpts->store ( adtString(L"Location"),	strFile ) );
//	CCLTRY ( pStmOpts->store ( adtString(L"ReadOnly"),	adtBool(true) ) );

	// Attempt to read in location map
//	bMap =	(	hr == S_OK &&
//					pStmSrc->open ( pStmOpts, &pStmMap ) == S_OK &&
//					pPrsrL->load ( pStmMap, v ) == S_OK &&
//					(IUnknown *)(NULL) != (unkV = v) &&
//					_QI(unkV,IID_IDictionary,&pDctMap) == S_OK );
	bMap	= false;
	_RELEASE(pStmMap);
	if (hr == S_OK && !bMap)
		{
		//
		// Generate location map.  A dictionary tree is cache to keep track of
		// location vs. Id for quick look-up.  Generate cache
		// by scanning the location index and reading the first location
		// value for the location, this contains the namespace path of the location.
		//
		U64	i = 0;

		// Create new dictionary map
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctMap));

		// Scan all existing location headers (skip version entry)
		for (i = 1;i < uLocId && hr == S_OK;++i)
			{
			HDRIDX	hdrI;
			HDRLOC	hdrE;

			// Read emission index header current location Id
			CCLTRY ( locGet ( i, &hdrI ) );

			// Read emission header for first sequence number
			CCLTRY ( locGet ( hdrI.first, &hdrE ) );

			// Read value for position
			CCLTRY ( locGet ( hdrE.value, v ) );

			// Expecting namespace path string
			CCLTRYE ( (adtValue::type(v) == VTYPE_STR) && v.pstr != NULL, E_UNEXPECTED );

			// TODO: Sanity check with value header to ensure matching location Ids ?

			// Associate path with Id
//			CCLOK ( dbgprintf ( L"TemporalImpl::open:%s\r\n", v.pstr ); )
			CCLTRY ( nspcStoreValue ( pDctMap, v.pstr, adtLong(i) ) );
//			CCLTRY ( pDctMap->store ( v, adtLong(i) ) );
			}	// for

		//
		// Save generated location map
		//

		// Access stream
//		CCLTRY ( pStmOpts->store ( adtString(L"ReadOnly"),	adtBool(false) ) );
//		CCLTRY ( pStmOpts->store ( adtString(L"Create"),	adtBool(true) ) );
//		CCLTRY ( pStmOpts->store ( adtString(L"Truncate"),	adtBool(true) ) );
//		CCLTRY ( pStmSrc->open ( pStmOpts, &pStmMap ) );

		// Save map
//		CCLTRY ( pPrsrS->save ( pStmMap, adtIUnknown(pDctMap) ) );

		// Clean up
		_RELEASE(pStmMap);
		}	// if

	// Clean up
	_RELEASE(pStmOpts);

	return hr;
	}	// open

HRESULT TemporalImpl :: open ( IDictionary *pOpts, IUnknown **ppLoc )
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
	TemporalLoc	*pDct	= NULL;
	int			len;
	adtValue		vL;
	adtBool		bRO(true);
	adtString	strLoc;

	// Access options
	CCLTRY ( pOpts->load ( strnRefLoc, vL ) );
	CCLTRYE( (len = (strLoc = vL).length()) > 0, E_UNEXPECTED );
	CCLTRY ( pOpts->load ( strnRefRO, vL ) );
	CCLOK  ( bRO = vL; )

	// If read-only, ensure location even exists
//	if (hr == S_OK && bRO)
//		hr = locId ( strLoc, true, &uId );

	// Ensure trailing slash for location
	if (hr == S_OK && strLoc[len-1] != '/')
		hr = strLoc.append ( L"/" );

	// Create temporal dictionary for location
	CCLTRYE	( (pDct = new TemporalLoc ( this, strLoc, bRO )) != NULL, 
					E_OUTOFMEMORY );
	CCLOK		( pDct->AddRef(); )
	CCLTRY	( pDct->construct() );

	// Result
	CCLTRY	( _QI(pDct,IID_IUnknown,ppLoc) );

	// Clean up
	_RELEASE(pDct);

	return hr;
	}	// open

HRESULT TemporalImpl :: resolve ( const WCHAR *pwLoc, bool bAbs, 
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
	// TODO: Make sense ?
	return E_NOTIMPL;
	}	// resolve

HRESULT TemporalImpl :: status ( const WCHAR *wLoc, IDictionary *pSt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	ITemporalImpl
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
	adtValue		v;

	// Check for location in map
	CCLTRY ( nspcLoadPath ( pDctMap, wLoc, v ) );
//	CCLTRY ( pDctMap->load ( adtString(wLoc), v ) );

	// A number at location means path is a value
	if (hr == S_OK && v.vtype == VTYPE_I8)
		{
		// Value location
		CCLTRY ( pSt->store ( strnRefVal, adtBool(true) ) );
		}	// if

	// An object (dictionary) means path is a location
	else if (hr == S_OK && v.vtype == VTYPE_UNK)
		{
		// Mark as location
		CCLTRY ( pSt->store ( strnRefLoc, adtBool(true) ) );
		}	// else if

	else if (hr == S_OK)
		hr = E_UNEXPECTED;

	return hr;
	}	// status

