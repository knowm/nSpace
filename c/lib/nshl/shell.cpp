////////////////////////////////////////////////////////////////////////
//
//									SHELL.CPP
//
//					Implementation of the nSpace shell base class
//
////////////////////////////////////////////////////////////////////////

#include "nshl_.h"
#include <stdio.h>

// Testing
#include "../adtl/adtl_.h"

Shell :: Shell ( IDictionary *_pDctCmd ) 
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	//	PARAMETERS
	//		-	pDctCmd contains command line information
	//
	////////////////////////////////////////////////////////////////////////
	pStmSrc		= NULL;
	pTmp			= NULL;
	pSpc			= NULL;
	pDctCmd		= _pDctCmd;
	_ADDREF(pDctCmd);
	evRun.init();
	}	// Shell

Shell :: ~Shell ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pDctCmd);
	}	// Shell

HRESULT Shell :: batch ( INamespace *pSpc, ILocations *pTmp, 
									ILocations *pSrc, IContainer *pCont )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process a batch list of commands.
	//
	//	PARAMETERS
	//		-	pSpc is the namespace
	//		-	pTmp is the TemporalImpl database
	//		-	pSrc is the stream source
	//		-	pCont contains the list of commands
	//
	//	RETURN VALUE	
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pDct	= NULL;
	IIt			*pIt	= NULL;
	adtValue		v;
	adtIUnknown	unkV;
	adtString	strCmd;

	// Process each command
	CCLTRY ( pCont->iterate ( &pIt ) );
	while (hr == S_OK && pIt->read ( v ) == S_OK)
		{
		// Expecting dictionary
		CCLTRY ( _QISAFE ( (unkV = v),IID_IDictionary,&pDct) );

		// Load the command (same as command line)
		CCLTRY ( pDct->load ( adtString(L"Command"), v ) );
		CCLTRYE( (strCmd = v).length() > 0, E_UNEXPECTED );

		// Process command
		if (hr == S_OK && !WCASECMP(strCmd,L"def"))
			{
			adtString	strPath;

			// Path to the definitions
			CCLTRY ( pDct->load ( adtString(L"Path"), v ) );
			CCLTRYE( (strPath = v).length() > 0, E_UNEXPECTED );

			// Process definition
			CCLOK ( lprintf ( LOG_DBG, L"definitions @ %s", (LPCWSTR)strPath ); )
//			CCLOK ( definitions ( pTmp, pSrc, strPath ); )
			}	// if
		else if (hr == S_OK && !WCASECMP(strCmd,L"put"))
			{
			IDictionary	*pEmit	= NULL;
			IDictionary	*pOpts	= NULL;
			IUnknown		*pLoc		= NULL;
			adtString	strPath,strPut;
			adtValue		vPut;

			// Stream options
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pOpts ) );

			// Path for put
			CCLTRY ( pDct->load ( adtString(L"Path"), v ) );
			CCLTRYE( (strPath = v).length() > 0, E_UNEXPECTED );

			// A stream location can be specified for a stream
			if (hr == S_OK && pDct->load ( adtString(L"Stream"), vPut ) == S_OK)
				{
				IByteStream	*pStmIn	= NULL;
				IByteStream	*pStmOut	= NULL;
				adtString	strLoc(vPut);

				// Options for stream access
				CCLTRY ( pOpts->store ( adtString(L"Location"), strLoc ) );
				CCLTRY ( pOpts->store ( adtString(L"ReadOnly"), adtBool(true) ) );

				// Access stream
				CCLOK ( lprintf ( LOG_DBG, L"Stream @ %s\r\n", (LPCWSTR)strLoc ); )
				CCLTRY ( pStmSrc->open ( pOpts, &pLoc ) );
				CCLTRY ( _QI(pLoc,IID_IByteStream,&pStmIn) );
				_RELEASE(pLoc);

				// Create a byte stream to receive the stream
				CCLTRY ( COCREATE ( L"Io.StmMemory", IID_IByteStream, &pStmOut ) );

				// Copy input stream to output stream
				CCLTRY ( pStmIn->copyTo ( pStmOut, 0, NULL ) );

				// Reset position of output stream
				CCLTRY ( pStmOut->seek ( 0, STREAM_SEEK_SET, NULL ) );

				// Use as value to 'put'
				CCLTRY ( adtValue::copy ( adtIUnknown(pStmOut), vPut ) );

				// Clean up
				_RELEASE(pStmOut);
				_RELEASE(pStmIn);
				}	// if 

			// A value can be imported from an external file (NSPC file)
			else if (hr == S_OK && pDct->load ( adtString(L"Import"), vPut ) == S_OK)
				{
				adtString	strLoc(vPut);

				// Access value
				CCLTRY ( load ( pStmSrc, strLoc, vPut ) );
				}	// if 

			// An value is required
			else if (hr == S_OK)
				hr = pDct->load ( adtString(L"Value"), vPut );

			// Debug
//			if (hr == S_OK && !WCASECMP(L"/State/List/Render/Names/Render1",strPath))
//				dbgprintf ( L"Hi\r\n" );

			// Access a temporal location
			CCLTRY ( pOpts->store ( adtString(L"Location"), strPath ) );
			CCLTRY ( pOpts->store ( adtString(L"ReadOnly"), adtBool(false) ) );
			CCLTRY ( pTmp->open ( pOpts, &pLoc ) );
			CCLTRY ( _QI(pLoc,IID_IDictionary,&pEmit) );

			// Record value
			CCLTRY ( pEmit->store ( adtString(L"Value"), vPut ) );

			// Clean up
			CCLTRY ( adtValue::toString ( vPut, strPut ) );
			lprintf ( LOG_DBG, L"put @ %s : %s : 0x%x",  (LPCWSTR)strPath, (LPCWSTR)strPut, hr );
			_RELEASE(pEmit);
			_RELEASE(pLoc);
			_RELEASE(pOpts);
			}	// else if
		else if (hr == S_OK && !WCASECMP(strCmd,L"get"))
			{
			IDictionary	*pDctGet	= NULL;
			adtString	strPath,strValue;
			adtValue		vGet;

			// Path for get
			CCLTRY ( pDct->load ( adtString(L"Path"), v ) );
			CCLTRYE( (strPath = v).length() > 0, E_UNEXPECTED );

			// Get value
//			CCLOK ( dbgprintf ( L"Batch:get @ %s : 0x%x\r\n", 
//										(LPCWSTR)strPath, pSpc->get ( strPath, vGet ) ); )

			// Further debug in case of retrieving state
			if (	hr == S_OK												&& 
					(IUnknown *)NULL != (unkV=vGet)					&&
					_QI(unkV,IID_IDictionary,&pDctGet) == S_OK	&&
					pDctGet->load ( adtString(L"Value"), vGet ) == S_OK &&
					adtValue::toString ( vGet, strValue ) == S_OK)
				lprintf ( LOG_DBG, L"get:Value %s", (LPCWSTR)strValue );

			// Clean up
			_RELEASE(pDctGet);
			}	// else if
		else if (hr == S_OK && !WCASECMP(strCmd,L"record"))
			{
			adtString	strPath;

			// Path to record
			CCLTRY ( pDct->load ( adtString(L"Path"), v ) );
			CCLTRYE( (strPath = v).length() > 0, E_UNEXPECTED );

			// Record value
//			CCLOK ( dbgprintf ( L"Batch:record @ %s : 0x%x\r\n", 
//										(LPCWSTR)strPath, pSpc->record ( strPath, true ) ); )
			}	// else if

		// Clean up
		_RELEASE(pDct);
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);

	return hr;
	}	// batch

HRESULT Shell :: construct ( void )
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

	// Initialize run event
	CCLTRYE ( evRun.init() == true, E_UNEXPECTED );

	return hr;
	}	// construct

HRESULT Shell :: dbInit ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initialize the TemporalImpl database.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IDictionary		*pDct		= NULL;
	ITemporalImpl	*pImpl	= NULL;

	// Determine location of target database in order of priority

	// Option 1 - Default location is a TemporalImpl folder under the root directory
	if (hr == S_OK)
		{
		// Allocate memory for new path
		CCLTRY ( strDB.allocate ( strRoot.length()+10 ) );

		// New path
		CCLOK ( WCSCPY ( &strDB.at(), strRoot.length()+10, strRoot ); )
		CCLOK ( WCSCAT ( &strDB.at(), strRoot.length()+10, L"temporal/" ); )
		}	// if

	// TemporalImpl nSpace object
	CCLTRY ( COCREATE ( L"Nspc.TemporalImpl", IID_ILocations, &pTmp ) );

	// Initialize options for database
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
	CCLTRY ( pDct->store ( adtString(L"Locations"), adtIUnknown(pStmSrc) )) ;
	CCLTRY ( pDct->store ( adtString(L"Location"), strDB )) ;

	// Initialize database
	CCLTRY ( _QI(pTmp,IID_ITemporalImpl,&pImpl) );
	CCLTRY ( pImpl->open ( pDct ) );

	// Clean up
	_RELEASE(pImpl);
	_RELEASE(pDct);

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"Shell::dbInit:hr 0x%x\r\n", hr );

	return hr;
	}	// dbInit

HRESULT Shell :: definitions ( const WCHAR *pwRoot )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Record updated definitions.
	//
	//	PARAMETERS
	//		-	pTmp is the TemporalImpl database
	//		-	pSrc is the stream source
	//		-	pwRoot is the root location containing the definitions.
	//
	//	RETURN VALUE	
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IStreamPersist	*pParser	= NULL;
	IByteStream		*pStmC	= NULL;
	IResource		*pResC	= NULL;
	IList				*pLocs	= NULL;
	IDictionary		*pOpts	= NULL;
	IIt				*pLocsIt	= NULL;
	IIt				*pIt		= NULL;
	adtValue			v;

	// Create a parser for the definitions
	CCLTRY ( COCREATE ( L"Nspc.PersistTxt", IID_IStreamPersist, &pParser ) );

	// Create container to receive options and location information
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pOpts ) );

	// Create byte cache for reading in definitioins
	CCLTRY ( COCREATE ( L"Io.ByteCache", IID_IByteStream, &pStmC ) );
	CCLTRY ( _QI(pStmC,IID_IResource,&pResC) );

	// Create a queue of hold unprocessed locations
	CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pLocs ) );

	// Add the initial location to the list
	CCLTRY ( pLocs->write ( adtString (pwRoot) ) );

	// Iterate locations
	CCLTRY ( pLocs->iterate ( &pLocsIt ) );

	// Continue until no more locations
	CCLOK ( lprintf ( LOG_INFO, L"definition scan:Root:%s {", pwRoot ); )
	while (hr == S_OK && pLocsIt->read ( v ) == S_OK)
		{
		adtString	strLoc;

		// Current location
		CCLTRYE ( (strLoc = v).length() > 0, E_UNEXPECTED );

		// Iterate the sub-locations in the stream source
//		dbgprintf ( L"definitions:strLoc %s\r\n", (LPCWSTR)strLoc );
		CCLTRY ( pStmSrc->locations ( strLoc, &pIt ) );

		// Scan the provided locations
		while (hr == S_OK && pIt->read ( v ) == S_OK)
			{
			adtString	strName;

			// Location
			if ((strName = v).length() > 0)
				{
				// Full path to location
				adtString	strPath((LPCWSTR)strLoc);
				CCLTRY ( strPath.append ( strName ) );
//				dbgprintf ( L"definitions:location found:%s\r\n", (LPCWSTR)strPath );

				// Location ?
				if (strName[strName.length()-1] == '/')
					{
					// Put location on queue for future processing
					CCLTRY ( pLocs->write ( strPath ) );
			 		}	// if

				// Stream (looking for '.nspc' files)
				else if (strName.length() > 4 && !WCASECMP(&(strName[strName.length()-5]),L".nspc"))
					{
					IUnknown		*pLoc			= NULL;
//					IDictionary	*pEmt			= NULL;
					IDictionary	*pDctTmp		= NULL;
					adtDate		vDateE,vDateF;
					adtValue		vL;
					adtString	strBase,strDescE,strPathRel;
					adtIUnknown	unkV;

					// Get the status and modified date of the stream location.
					CCLTRY ( pStmSrc->status ( strPath, pOpts ) );
					CCLTRY ( pOpts->load ( adtString(L"Modified"), v ) );
					CCLOK  ( vDateF = v; )

					// Remove the '.nspc' from the filename
					CCLOK  ( strPathRel = &(strPath[wcslen(pwRoot)]); )
					CCLOK  ( strPathRel.at(strPathRel.length()-5) = '\0'; )
					CCLTRY ( strPathRel.append ( L"/" ) );

					// Definitions are placed in reference layer
					CCLTRY ( strPathRel.prepend ( LOC_NSPC_REF ) );

					// Path to modified value
//					CCLTRY ( adtValue::copy ( strPathRel, strDescE ) );
//					CCLTRY ( strDescE.append ( L"/" ) );
//					CCLTRY ( strDescE.append ( STR_NSPC_MOD ) );
//					CCLTRY ( strDescE.append ( L"/" ) );

					// Access temporal dictionary at location (if it exists)
					// Load latest descriptor at location
//					CCLOK ( dbgprintf ( L"%s\r\n", (LPCWSTR) strDescE ); )
					vDateE	= 0;
					CCLTRY ( pOpts->store ( adtString(L"Location"), strPathRel ) );
					CCLTRY ( pOpts->store ( adtString(L"ReadOnly"), adtBool(true) ) );
					if (	hr == S_OK && 
							pTmp->open ( pOpts, &pLoc ) == S_OK &&
							_QI(pLoc,IID_IDictionary,&pDctTmp) == S_OK)
						{
						// Access the last import date
						if (hr == S_OK && pDctTmp->load ( adtString(STR_NSPC_MOD), vL ) == S_OK)
							vDateE = vL;
						}	// if

					// Clean up
					_RELEASE(pDctTmp);
					_RELEASE(pLoc);

					// If the modified date in the stream source is later than the
					// last import dates then it is time to record a new definition.
					if (hr == S_OK && vDateE < vDateF)
						{
						IByteStream	*pStm	= NULL;
						IDictionary	*pDef	= NULL;
						adtIUnknown	unkV;

						// Options for stream
//						dbgprintf ( L"record:%s\r\n", (LPCWSTR)strPath );
						CCLTRY ( pOpts->store ( adtString(L"Location"), adtString(strPath) ) );
						CCLTRY ( pOpts->store ( adtString(L"ReadOnly"), adtBool(true) ) );

						// Access stream
						CCLTRY ( pStmSrc->open ( pOpts, &pLoc ) );
						CCLTRY ( _QI(pLoc,IID_IByteStream,&pStm) );

						// Initialize byte cache
						CCLTRY ( pOpts->store ( adtString(L"Stream"), adtIUnknown(pStm) ) );
						CCLTRY ( pOpts->store ( adtString(L"ReadOnly"), adtBool(true) ) );
						CCLTRY ( pOpts->store ( adtString(L"Size"), adtInt(1024) ) );
						CCLTRY ( pResC->open ( pOpts ) );

						// Load the definition
						dbgprintf ( L"Shell::definitions:load:%s\r\n", (LPCWSTR)strPath );
						CCLTRY ( pParser->load ( pStmC, v ) );

						// Excepting a definition
						CCLTRY ( _QISAFE((unkV = v),IID_IDictionary,&pDef) );

						// Import definition, ok if it fails
						CCLOK ( defNspc ( strPathRel, pDef ); )

						// Clean up
						pResC->close();
						_RELEASE(pDef);
						_RELEASE(pStm);
						_RELEASE(pLoc);
						}	// if

					}	// else if

				}	// if

			// Clean up
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);
		pLocsIt->next();
		}	// while

	// Clean up
	lprintf ( LOG_DBG, L"} definition scan:0x%x", hr );
	_RELEASE(pLocsIt);
	_RELEASE(pLocs);
	_RELEASE(pResC);
	_RELEASE(pStmC);
	_RELEASE(pOpts);
	_RELEASE(pParser);
	
	return hr;
	}	// definitions

HRESULT Shell :: defNspc ( const WCHAR *wDef, IDictionary *pDef )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Import the provided definition loaded from an NSPC file
	//			into the temporal namespace.
	//
	//	PARAMETERS
	//		-	wDef is the definition path
	//		-	pDef contains the definition
	//
	//	RETURN VALUE	
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IDictionary	*pDct			= NULL;
	IDictionary	*pDctDef		= NULL;
	IDictionary	*pDctLoc		= NULL;
	IDictionary	*pOpts		= NULL;
	IDictionary	*pNodes		= NULL;
	IDictionary	*pSubs		= NULL;
	IDictionary	*pConns		= NULL;
	IContainer	*pNames		= NULL;
	IIt			*pIt			= NULL;
	IIt			*pItConn		= NULL;
	IUnknown		*pLoc			= NULL;
	adtValue		v;
	adtIUnknown	unkV;
	adtString	strPath;

	//
	// Import
	//

	// Options for locations
	CCLTRY (COCREATE(L"Adt.Dictionary",IID_IDictionary,&pOpts));
	CCLTRY ( pOpts->store ( adtString(L"ReadOnly"), adtBool(false) ) );

	//
	// Previous definition
	// Clear all existing information since anything could have changed.
	//

	// Clear reference information
	CCLOK	 ( strPath = wDef; )
	CCLTRY ( pOpts->store ( adtString(L"Location"), strPath ) );
	CCLTRY ( pTmp->open ( pOpts, &pLoc ) );
	CCLTRY ( _QI(pLoc,IID_IDictionary,&pDctDef) );
	CCLTRY ( pDctDef->clear() );
	_RELEASE(pLoc);

	// Store new reference information
	CCLTRY (pDctDef->store ( adtString(STR_NSPC_MOD), adtDate().now() ));
	CCLTRY (pDctDef->store ( adtString(STR_NSPC_REF), adtBool(true)));

	//
	// Process items
	//

	// Access names, nodes, subgraphs and connection lists
	CCLTRY ( pDef->load ( adtString(L"Names"), v ) );
	CCLTRY ( _QISAFE((unkV = v),IID_IContainer,&pNames) );
	CCLTRY ( pDef->load ( adtString(L"Subgraph"), v ) );
	CCLTRY ( _QISAFE((unkV = v),IID_IDictionary,&pSubs) );
	CCLTRY ( pDef->load ( adtString(L"Node"), v ) );
	CCLTRY ( _QISAFE((unkV = v),IID_IDictionary,&pNodes) );

	// Store names list directly in definition
	CCLTRY ( pDctDef->store ( strnRefNames, adtIUnknown(pNames) ) );

	// Process items in order
	CCLTRY ( pNames->iterate ( &pIt ) );
	while (hr == S_OK && pIt->read ( v ) == S_OK)
		{
		IDictionary	*pAttr	= NULL;
		adtString	strName(v);

		// Location of this item
		CCLOK	 ( strPath = wDef; )
		CCLTRY ( strPath.append ( strName ) );
		CCLTRY ( strPath.append ( L"/" ) );
//		dbgprintf ( L"Shell::defNspc:Path %s\r\n", (LPCWSTR)strPath );

		// Temporal dictionary for location
		CCLTRY ( pOpts->store ( adtString(L"Location"), strPath ) );
		CCLTRY ( pTmp->open ( pOpts, &pLoc ) );
		CCLTRY ( _QI(pLoc,IID_IDictionary,&pDctLoc) );

		// Subgraph
		if (hr == S_OK && pSubs->load ( strName, v ) == S_OK)
			{
			// Attributes
			CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pAttr) );

			// Set type
			CCLTRY ( pAttr->store ( adtString(STR_NSPC_TYPE), adtString(L"Location") ) );

			// Set revision
			CCLTRY ( pAttr->store ( adtString(STR_NSPC_REV), adtInt(1) ) );
			}	// if

		// Node
		else if (hr == S_OK && pNodes->load ( strName, v ) == S_OK)
			{
			// Attributes
			CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pAttr) );

			// Set type
			CCLTRY ( pAttr->store ( adtString(STR_NSPC_TYPE), adtString(L"Behaviour") ) );

			// Set revision
			CCLTRY ( pAttr->store ( adtString(STR_NSPC_REV), adtInt(1) ) );
			}	// if

		// Store descriptor at location
		if (hr == S_OK && pAttr != NULL)
			hr = pDctLoc->store ( strnRefDesc, adtIUnknown(pAttr) );

		// Clean up
		pIt->next();
		_RELEASE(pAttr);
		_RELEASE(pDctLoc);
		_RELEASE(pLoc);
		}	// while

	// Access temporal location for connections
	CCLOK	 ( strPath = wDef; )
	CCLTRY ( strPath.append ( STR_NSPC_CONN L"/" ) );
	CCLTRY ( pOpts->store ( adtString(L"Location"), strPath ) );
	CCLTRY ( pTmp->open ( pOpts, &pLoc ) );
	CCLTRY ( _QI(pLoc,IID_IDictionary,&pDctLoc) );

	//
	// Connections.
	// Store connections under the associated connectors.
	//
	CCLTRY ( pDef->load ( adtString(L"Connection"), v ) );
	CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pConns) );
	CCLTRY ( pConns->keys ( &pItConn ) );
	while (hr == S_OK && pItConn->read ( v ) == S_OK)
		{
		adtValue	vL;

		// Access connection dscriptor
		CCLTRY ( pConns->load ( v, vL ) );

		// Store connection under position at location
		CCLTRY ( pDctLoc->store ( v, vL ) );

		// Clean up
		pItConn->next();
		}	// while


	// Store connection list directly in definition
//	if (hr == S_OK && pDef->load ( adtString(L"Connection"), v ) == S_OK)
//		hr = pDctDef->store ( strnRefConn, v );

	// Clean up	
	_RELEASE(pLoc);
	_RELEASE(pDctLoc);
	_RELEASE(pOpts);
	_RELEASE(pConns);
	_RELEASE(pIt);
	_RELEASE(pNodes);
	_RELEASE(pSubs);
	_RELEASE(pNames);
	_RELEASE(pDctDef);
	_RELEASE(pDct);

	// Debug
	lprintf ( LOG_DBG, L"Definition update:%s:0x%x:%s", wDef, hr, (hr == S_OK) ? L"Ok" : L"Fail" );

	return hr;
	}	// defNspc

HRESULT Shell :: load ( ILocations *pSrc, const WCHAR *pwLoc, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Load a value from a stream location.
	//
	//	PARAMETERS
	//		-	pSrc is the stream source
	//		-	pwLoc is the stream location containing a list of recordings
	//		-	v will receive the value
	//
	//	RETURN VALUE	
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IByteStream		*pStm		= NULL;
	IUnknown			*pLoc		= NULL;
	IDictionary		*pOpts	= NULL;
	IStreamPersist	*pParser	= NULL;

	// Options for stream
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pOpts ) );
	CCLTRY ( pOpts->store ( adtString(L"Location"), adtString(pwLoc) ) );
	CCLTRY ( pOpts->store ( adtString(L"ReadOnly"), adtBool(true) ) );

	// Access stream
	CCLTRY ( pSrc->open ( pOpts, &pLoc ) );
	CCLTRY ( _QI(pLoc,IID_IByteStream,&pStm) );

	// Create a parser for the stream
	CCLTRY ( COCREATE ( L"Nspc.PersistTxt", IID_IStreamPersist, &pParser ) );

	// Load it
	CCLTRY ( pParser->load ( pStm, v ) );

	// Clean up
	_RELEASE(pParser);
	_RELEASE(pLoc);
	_RELEASE(pStm);
	_RELEASE(pOpts);

	return hr;
	}	// load

HRESULT Shell :: stgInit ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initialize the storage subsystem for the shell.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pDct	= NULL;

	// Default stream source
	CCLTRY ( COCREATE ( L"Io.StmSrcFile", IID_ILocations, &pStmSrc ) );
	CCLTRY ( _QI(pStmSrc,IID_IDictionary,&pDct) );

	// If a root location was not specified, use default stream source root
	if (hr == S_OK && strRoot.length() == 0)
		{
		adtValue	vL;

		// Access default root
		CCLTRY ( pDct->load ( adtString(L"Root"), vL ) );
		CCLTRYE( (strRoot = vL).length() > 0, ERROR_INVALID_STATE );
		}	// if

	// Otherwise set default root
	else if (hr == S_OK && strRoot.length() > 0)
		hr = pDct->store ( adtString(L"Root"), strRoot );

	// Set the default root location
	_RELEASE(pDct);

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"Shell::stgInit:hr 0x%x\r\n", hr );

	return hr;
	}	// stgInit

HRESULT Shell :: tickAbort ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' should abort.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Signal end
	evRun.signal();

	return S_OK;
	}	// tickAbort

HRESULT Shell :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Perform one 'tick's worth of work.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;

	// Default behaviour is simply to wait for event to be signalled.
//	dbgprintf ( L"Shel::tick {\r\n" );
	hr = (evRun.wait(1000) == FALSE) ? S_OK : S_FALSE;
//	dbgprintf ( L"} Shel::tick\r\n" );

	// Debug
//	Sleep(2000);
//	hr = S_FALSE;

	return hr;
	}	// tick

HRESULT Shell :: tickBegin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that it should get ready to 'tick'.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IDictionary	*pDct			= NULL;
	adtString	strExec;
	adtValue		vL;

	// Some system components require this
	#if	defined(_WIN32) && !defined(UNDER_CE)
	CCLOK ( CoInitializeSecurity ( NULL, -1, NULL, NULL,
					RPC_C_AUTHN_LEVEL_DEFAULT, RPC_C_IMP_LEVEL_IMPERSONATE,
					NULL, EOAC_NONE, NULL ); )
	#endif

	// Storage/stream source initialization
	CCLTRY ( stgInit() );

	// TemporalImpl database
	CCLTRY ( dbInit() );

	// Debug
	CCLOK ( lprintf ( LOG_DBG, L"nsh:Root location     : %s", (LPCWSTR) strRoot ); )
	CCLOK ( lprintf ( LOG_DBG, L"nsh:Database location : %s", (LPCWSTR) strDB ); )

	//
	// Command line
	//

	// Optional 'execute' command
	if (hr == S_OK && pDctCmd->load ( adtString(L"Execute"), vL ) == S_OK)
		strExec = vL;

	// DEBUG
	#ifdef _DEBUG
	// Currently the source tree is laid out such that the 'graph' directory is
	// at the same relative location to either debug or release EXE.  Use relative
	// directory in order to support different source trees.
	if (hr == S_OK)
		{
		adtString	strLocDef;

		// Use the same root as the default file system root for the storage object
		CCLTRY ( _QI(pStmSrc,IID_IDictionary,&pDct) );
		CCLTRY ( pDct->load ( adtString(L"Root"), vL ) );
		CCLTRY ( adtValue::toString ( vL, strLocDef ) );

		// Relative location to definitions
		#ifdef	_WIN64
		CCLTRY ( strLocDef.append ( L"../../../graph/" ) );
		#else
		CCLTRY ( strLocDef.append ( L"../../graph/" ) );
		#endif

		// Scan for definitions
		CCLTRY ( definitions ( strLocDef ) );

		// Clean up
		_RELEASE(pDct);
		}	// if

	// To support a 'compiler-less' mode of graph development in release mode, 
	// scan for graph definitions at startup in the working directory in case
	// someone wants to just drop some graphs under the binaries and make
	// changes to just those definitions.
	#else
	CCLTRY ( definitions ( L"./graph/" ) );
	#endif

	/////////////
	// Namespace
	/////////////

	// Create a namespace
	CCLTRY ( COCREATE ( L"Nspc.Namespace", IID_INamespace, &pSpc ) );

	// Open the namespace on top of the database
	CCLTRY ( _QI(pSpc,IID_IDictionary,&pDct) );
	CCLTRY ( pDct->store ( adtString(L"Locations"), adtIUnknown(pStmSrc) ) );
	CCLTRY ( pSpc->open ( pTmp ) );
	_RELEASE(pDct);

	//
	// TESTING
	//
/*
	// Key/value array
	if (hr == S_OK)
		{
		KeyValArray	a;
		adtString	strKey1(L"YouCanByte"),strKey2(L"MeNow");
		adtDouble	dVal1(3.14159265);
		adtInt		iVal2(1234);
		adtValue		vL;

		// Test
		a.store ( strKey1, dVal1 );
		a.store ( strKey2, iVal2 );
		a.load ( strKey1, vL );
		a.load ( adtString(L"Hello"), vL );

		// Clean up
		dbgprintf ( L"Done\r\n" );
		}	// if


	// Locations
	if (hr == S_OK)
		{
		IDictionary		*pRoot1	= NULL;
		IDictionary		*pRoot2	= NULL;
		IEmitterLoc		*pEmt		= NULL;
		IReceptorLoc	*pRcp1	= NULL;
		IReceptorLoc	*pRcp2	= NULL;

		// Test locations
		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pRoot1 ) );
		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pRoot2 ) );

		// Create hierarchy
		CCLTRY ( _QI(pRoot1,IID_IReceptorLoc,&pRcp1) );
		CCLTRY ( pRcp1->receive ( NULL, L"You/Can/Byte/Me/Now", adtDouble(3.141591653) ) );
		CCLTRY ( _QI(pRoot2,IID_IReceptorLoc,&pRcp2) );
		CCLTRY ( pRcp2->receive ( NULL, L"You/Can/Byte/Me/Now", adtDouble(6.28) ) );

		// Connect hierarchy
//		CCLTRY(pRcp1->connect ( pRcp2, true ) );
		CCLTRY(pRcp2->connect ( pRcp1, true ) );

		// Store new value to test propagation
		CCLTRY ( pRcp1->receive ( NULL, L"You/Can/Byte/Me/Now", adtDouble(1.234) ) );
		_RELEASE(pRcp1);
		_RELEASE(pRcp2);

		// Set up mirroring between roots
//		CCLTRY ( _QI(pRoot1,IID_IEmitterLoc,&pEmt) );
//		CCLTRY ( _QI(pRoot2,IID_IReceptorLoc,&pRcp) );
//		CCLTRY ( pEmt->connect ( pRcp, true ) );
//		_RELEASE(pRcp);
//		_RELEASE(pEmt);

		// Clean up
		_RELEASE(pRoot1);
		_RELEASE(pRoot2);

		hr = S_FALSE;
*/
/*
		IDictionary	*pLoc		= NULL;
		IDictionary	*pLocSub	= NULL;
		IDictionary	*pSrc		= NULL;
		IDictionary	*pDst		= NULL;

		//
		// Root 1
		//

		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pLocSub ) );
		CCLTRY ( pRoot1->store ( adtString(L"Level1_1"), adtIUnknown(pLocSub) ) );
		_RELEASE(pLocSub);
		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pLocSub ) );
		CCLTRY ( pRoot1->store ( adtString(L"Level1_2"), adtIUnknown(pLocSub) ) );
		pLoc	= pLocSub;
		_ADDREF(pLoc);
		_RELEASE(pLocSub);

		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pLocSub ) );
		CCLTRY ( pLoc->store ( adtString(L"Level2_1"), adtIUnknown(pLocSub) ) );
		_RELEASE(pLocSub);
		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pLocSub ) );
		CCLTRY ( pLoc->store ( adtString(L"Level2_2"), adtIUnknown(pLocSub) ) );
		_RELEASE(pLoc);
		pLoc = pLocSub;
		_ADDREF(pLoc);
		_RELEASE(pLocSub);

		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pLocSub ) );
		CCLTRY ( pLoc->store ( adtString(L"Level3_1"), adtIUnknown(pLocSub) ) );
		pSrc = pLocSub;
		_ADDREF(pSrc);
		_RELEASE(pLocSub);
		_RELEASE(pLoc);

		// 
		// Root 2
		//
		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pRoot2 ) );

		CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pLocSub ) );
		CCLTRY ( pRoot2->store ( adtString(L"Level1_1"), adtIUnknown(pLocSub) ) );
		_RELEASE(pLocSub);

		// Clean up
		_RELEASE(pSrc);
		_RELEASE(pRoot1);
		_RELEASE(pRoot2);
*/
//		}	// if



/*
	// Graph instancing performance
	if (hr == S_OK)
		{
		adtString	strDef = L"Lib/Graph/Link/Inst";
		adtString	strInst;
		adtValue		v;

		// Timing
		DWORD dwThen = GetTickCount();

		// Instance graphs
		for (int i = 0;hr == S_OK && i < 50;++i)
			{
			// Retrieve instance
			CCLOK  ( strInst = adtInt(i); )
			CCLTRY ( strInst.prepend ( strDef ) );
			CCLTRY ( strInst.append ( L"/Debug/Fire" ) );
			CCLTRY ( pSpc->get ( strInst, v, NULL ) );
			}	// for

		// Timing
		DWORD dwNow = GetTickCount();
		dbgprintf ( L"Shell::tickBegin:%d ms\r\n", dwNow-dwThen );

		hr = S_FALSE;
		}	// if

	if (hr == S_OK)
		{
		IDictionary	*pGr		= NULL;
		IDictionary	*pItms	= NULL;
		IDictionary	*pCns		= NULL;
		IDictionary	*pDsc		= NULL;
		IReceptor	*pRcp		= NULL;
		adtValue		vL;
		adtIUnknown	unkV;

		// Create graph object
		CCLTRY ( COCREATE ( L"Nspc.Graph", IID_IDictionary, &pGr ) );

		// Namespace object
		CCLTRY ( pGr->store ( adtString(STR_NSPC_NSPC), 
										adtLong((U64)(INamespace *)pSpc) ) );

		// Access dictionaries
		CCLTRY ( pGr->load ( adtString(STR_NSPC_ITEMS), vL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pItms) );
		CCLTRY ( pGr->load ( adtString(STR_NSPC_CONN), vL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pCns) );

		//
		// Node
		//

		// Create test descriptor
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDsc ) );
//		CCLTRY ( pDsc->store ( adtString(STR_NSPC_NAME), adtString(L"Initialize") ) );
		CCLTRY ( pDsc->store ( adtString(STR_NSPC_BEHAVE), adtString(L"Misc.Dist") ) );
		CCLTRY ( pDsc->store ( adtString(STR_NSPC_TYPE), adtString(L"Node") ) );

		// Store in items
		CCLTRY ( pItms->store ( adtString(L"Initialize"), adtIUnknown (pDsc) ) );
		_RELEASE(pDsc);

		//
		// Node
		//

		// Create test descriptor
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDsc ) );
//		CCLTRY ( pDsc->store ( adtString(STR_NSPC_NAME), adtString(L"Initialize") ) );
		CCLTRY ( pDsc->store ( adtString(STR_NSPC_BEHAVE), adtString(L"Misc.Debug") ) );
		CCLTRY ( pDsc->store ( adtString(STR_NSPC_TYPE), adtString(L"Node") ) );

		// Store in items
		CCLTRY ( pItms->store ( adtString(L"Debug"), adtIUnknown (pDsc) ) );
		_RELEASE(pDsc);

		//
		// Connection
		//

		// Create test descriptor
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDsc ) );
//		CCLTRY ( pDsc->store ( adtString(STR_NSPC_NAME), adtString(L"Initialize") ) );
		CCLTRY ( pDsc->store ( adtString("From"), adtString(L"Initialize/OnFire") ) );
		CCLTRY ( pDsc->store ( adtString("To"), adtString(L"Debug/Fire") ) );

		// Store in connections
		CCLTRY ( pCns->store ( adtString(L"1"), adtIUnknown (pDsc) ) );
		_RELEASE(pDsc);

		// Test signal
		CCLTRY ( nspcLoadPath ( pGr, L"Initialize/Fire", vL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IReceptor,&pRcp) );
		CCLOK  ( pRcp->receive ( adtString(L"YouCanByteMeNow" ) ); )

		// Clean up
		_RELEASE(pRcp);
		_RELEASE(pCns);
		_RELEASE(pItms);
		_RELEASE(pGr);

		hr = S_FALSE;
		}	// if
*/

/*
	if (hr == S_OK)
		{
		IDictionary		*dctStm	= NULL;
		IDictionary		*dctOpt	= NULL;
		IStreamPersist	*per		= NULL;
		IByteStream		*stm		= NULL;
		IResource		*res		= NULL;
		IIt				*it		= NULL;
		adtValue			v;

		// Options for stream
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &dctOpt ) );
		CCLTRY ( dctOpt->store ( adtString(L"ReadOnly"), adtBool(false) ) );
		CCLTRY ( dctOpt->store ( adtString(L"Create"), adtBool(true) ) );
		CCLTRY ( dctOpt->store ( adtString(L"Truncate"), adtBool(true) ) );
		CCLTRY ( dctOpt->store ( adtString(L"Location"), adtString(L"c:/dev/nspace/c/debug/dctstm.bin") ) );

		// Stream
		CCLTRY ( pStmSrc->open ( dctOpt, &stm ) );

		// Value persistence
		CCLTRY ( COCREATE ( L"Io.StmPrsBin", IID_IStreamPersist, &per ) );

		// Initialize dictionary stream
		CCLTRY ( COCREATE ( L"Adt.DictionaryStm", IID_IDictionary, &dctStm ) );
		CCLTRY ( _QI(dctStm,IID_IResource,&res) );
		CCLTRY ( dctOpt->store ( adtString(L"Stream"), adtIUnknown(stm) ) );
		CCLTRY ( dctOpt->store ( adtString(L"Persist"), adtIUnknown(per) ) );
		CCLTRY ( res->open ( dctOpt ) );

		// Test storage
		CCLTRY ( dctStm->store ( adtInt(4), adtInt(4) ) );
		CCLTRY ( dctStm->store ( adtInt(2), adtInt(2) ) );
		CCLTRY ( dctStm->store ( adtInt(6), adtInt(6) ) );
		CCLTRY ( dctStm->store ( adtInt(1), adtInt(1) ) );
		CCLTRY ( dctStm->store ( adtInt(3), adtInt(3) ) );
		CCLTRY ( dctStm->store ( adtInt(5), adtInt(5) ) );
		CCLTRY ( dctStm->store ( adtInt(7), adtInt(7) ) );
//		CCLTRY ( dctStm->store ( adtString(L"YouCanByteMe"), adtInt(2) ) );
//		CCLTRY ( dctStm->store ( adtString(L"YouCanByteMeNow!!"), adtInt(4) ) );
//		CCLTRY ( dctStm->store ( adtString(L"YouCanByteMeNow"), adtInt(3) ) );
//		CCLTRY ( dctStm->store ( adtString(L"YouCanByteMeNow!!!!"), adtInt(5) ) );
//		CCLTRY ( dctStm->store ( adtString(L"YouCanByte"), adtInt(1) ) );

		// Test load
//		CCLTRY ( dctStm->load ( adtString(L"YouCanByteMe"), v ) );
		CCLTRY ( dctStm->load ( adtInt(1), v ) );

		// Test iterate
		CCLTRY ( dctStm->keys ( &it ) );
		while (hr == S_OK && it->read(v) == S_OK)
			{
			adtValue		vV;
			adtString	str;

			// Output
			if (adtValue::toString ( v, str ) == S_OK)
				dbgprintf ( L"Key : %s\r\n", (LPCWSTR)str );

			// Read value associated with key
			if (	dctStm->load ( v, vV ) == S_OK &&
					adtValue::toString ( vV, str ) == S_OK)
				dbgprintf ( L"Val : %s\r\n", (LPCWSTR)str );

			// Clean up
			it->next();
			}	// while

		// Test remove
//		CCLOK ( dctStm->remove ( adtString(L"YouCanByteMe") ); )
		CCLOK ( dctStm->remove ( adtInt(4) ); )
		CCLOK ( dctStm->remove ( adtInt(6) ); )
		CCLOK ( dctStm->remove ( adtInt(2) ); )
		CCLOK ( dctStm->remove ( adtInt(3) ); )
		CCLOK ( dctStm->remove ( adtInt(1) ); )
		CCLOK ( dctStm->remove ( adtInt(5) ); )
//		CCLOK ( dctStm->remove ( adtInt(7) ); )

		// Test iterate
		CCLTRY ( dctStm->keys ( &it ) );
		while (hr == S_OK && it->read(v) == S_OK)
			{
			adtValue		vV;
			adtString	str;

			// Output
			if (adtValue::toString ( v, str ) == S_OK)
				dbgprintf ( L"Key : %s\r\n", (LPCWSTR)str );

			// Read value associated with key
			if (	dctStm->load ( v, vV ) == S_OK &&
					adtValue::toString ( vV, str ) == S_OK)
				dbgprintf ( L"Val : %s\r\n", (LPCWSTR)str );

			// Clean up
			it->next();
			}	// while

		// Clean up
		_RELEASE(it);
		_RELEASE(res);
		_RELEASE(per);
		_RELEASE(stm);
		_RELEASE(dctOpt);
		_RELEASE(dctStm);
		}	// if
*/

	// 'Batch' requests via external file
	if (hr == S_OK && !WCASECMP(strExec,L"batch" ))
		{
		IContainer	*pCnt	= NULL;
		adtString	strPath;
		adtIUnknown	unkV;

		// Location of file required
		CCLTRY ( pDctCmd->load ( adtString(L"Location"), vL ) );
		CCLTRYE( (strPath = vL).length() > 0, ERROR_INVALID_STATE );

		// Load value from path
		CCLTRY ( load ( pStmSrc, strPath, vL ) );
	
		// Expecting list
		CCLTRY ( _QISAFE((unkV = vL),IID_IContainer,&pCnt) );

		// Process list
		CCLTRY ( batch ( pSpc, pTmp, pStmSrc, pCnt ) );

		// Clean up
		_RELEASE(pCnt);

		// Result
		lprintf ( LOG_DBG, L"Batch(0x%x) %s", hr, (LPCWSTR)strPath );
		}	// else if

	// Manually start the nSpace shell graph
	if (hr == S_OK)
		{
		adtValue		vL;
		#ifdef	_WIN32
		DWORD			dwNow,dwThen = GetTickCount();
		#endif
	
		// Auto-instance default shell
		CCLTRY ( pSpc->get ( L"/App/Shell/Default/Debug/Fire", vL, NULL ) );

		// Timing
		#ifdef	_WIN32
		dwNow = GetTickCount();
		lprintf ( LOG_DBG, L"Shell::tickBegin:%d ms\r\n", (dwNow-dwThen) );
		#endif
		}	// if


	// Testing
	if (hr == S_OK)
		{
/*
		IDictionary	*pDct	= NULL;
		IDictionary	*pV	= NULL;
		IList			*pQ	= NULL;
		IIt			*pQIt	= NULL;
		adtIUnknown	unkV;
		adtValue		vL;
		DWORD			dwThen,dwNow;

		// Retrieve hierarchy
		CCLTRY ( pSpc->get ( L"/Apps/Auto/Default/EditVis/", vL, NULL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pDct) );

		// Objects
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pV ) );
		CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pQ ) );
		CCLTRY ( pQ->iterate ( &pQIt ) );

		// Test
		dwThen = GetTickCount();
		dbg ( pDct, pV, pQ, pQIt );
		dwNow = GetTickCount();
		dbgprintf ( L"Test : %d ms\r\n", (dwNow-dwThen) );

		// Clean up
		_RELEASE(pV);
		_RELEASE(pDct);
*/
		// Ok if fails
		hr = S_OK;
		}	// if

	// Debug
//	Sleep(2000);
//	hr = S_FALSE;
	lprintf ( LOG_DBG, L"Shell::tickBegin:hr 0x%x", hr );
	
	// Clean up

	return (hr == S_OK) ? 0 : hr;
	}	// tickBegin

HRESULT Shell :: tickEnd ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' is to stop.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr = S_OK;

	// Clean up
	_RELEASE(pSpc);
	_RELEASE(pTmp);
	_RELEASE(pStmSrc);

	return hr;
	}	// tickEnd

HRESULT Shell::dbg(IDictionary *pDct, IDictionary *pV, IList *pQ, IIt *pQIt)
	{
	HRESULT		hr = S_OK;
	IDictionary	*pSub = NULL;
	IIt			*pIt = NULL;
	adtValue		vK, vV;

	CCLTRY(pQ->write(adtIUnknown(pDct)));
	CCLTRY(pDct->keys(&pIt));
	while (hr == S_OK && pIt->read(vK) == S_OK)
		{
		// Visit sub-dictionary
		if (hr == S_OK &&
			vK.vtype == VTYPE_STR &&
			WCASECMP(vK.pstr, STR_NSPC_PARENT) &&
			pDct->load(vK, vV) == S_OK &&
			vV.vtype == VTYPE_UNK &&
			vV.punk != NULL &&
			_QI(vV.punk, IID_IDictionary, &pSub) == S_OK)
			{
			adtLong	lK;
			lK = (U64)(pSub);

			// Visit check
			//			pV->load ( lK, vV );
			//			pV->store ( lK, adtInt(0) );
			pSub->load(adtString(L"_Location"), vV);
			dbg(pSub, pV, pQ, pQIt);
			//			pV->load ( lK, vV );
			//			pV->store ( lK, adtInt(1) );
			//			pV->remove ( lK );
			}	// if

		// Clean up
		_RELEASE(pSub);
		pIt->next();
		}	// while
	pQIt->next();

	// Clean up
	_RELEASE(pIt);

	return hr;
	}	// dbg
