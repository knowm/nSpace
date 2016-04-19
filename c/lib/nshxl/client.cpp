////////////////////////////////////////////////////////////////////////
//
//								CLIENT.CPP
//
//					nSpace shell external client
//
////////////////////////////////////////////////////////////////////////

#include	"nshxl.h"

// Objects in this module (for CCL linkage, no objects created in app)
CCL_OBJLIST_BEGIN()
CCL_OBJLIST_END()

////////////////
// nSpaceClient
////////////////

nSpaceClient :: nSpaceClient()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pGIT		= NULL;
	dwSvc		= 0;
	pLstns	= NULL;
	pwRoot	= NULL;
	pCB		= NULL;
	}	// nSpaceClient

nSpaceClient :: ~nSpaceClient()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"nSpaceClient::~nSpaceClient\r\n" );
	close();
	_RELEASE(pLstns);
	_RELEASE(pGIT);
	}	// ~nSpaceClient

HRESULT nSpaceClient :: close ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Close connection to namespace.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	INamespaceX	*pSpc		= NULL;
	IIt			*pKeys	= NULL;
	adtValue		v;

	// Access service interface
	CCLTRY ( getSvc ( &pSpc ) );

	// Unlisten remaining listens
	CCLTRY ( pLstns->keys ( &pKeys ) );
	while (hr == S_OK && pKeys->read ( v ) == S_OK)
		{
		adtString	vStr(v);

		// Unlisten and move to next listen
		listen ( vStr, FALSE );

		// Unlistens are removed so restart iteration
		pKeys->begin();
		}	// while
	CCLOK ( pLstns->clear(); )

	// Clean up
	_RELEASE(pKeys);
	_RELEASE(pSpc);

	// Global interface table
	if (pGIT != NULL)
		{
		// Unregister service
		if (dwSvc != 0)
			pGIT->RevokeInterfaceFromGlobal ( dwSvc );
		dwSvc = 0;
		_RELEASE(pGIT);
		}	// if

	return hr;
	}	// close

HRESULT nSpaceClient :: getSvc ( INamespaceX **ppSvc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Get service pointer for current thread from global interface
	//			table.
	//
	//	PARAMETERS
	//		-	ppSvc will receive the ptr.
	//
	//	RETURN TYPE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// State check
	CCLTRYE	( dwSvc != 0, ERROR_INVALID_STATE );

	// Obtain ptr.
	CCLTRY ( pGIT->GetInterfaceFromGlobal ( dwSvc, __uuidof(INamespaceX),
														(void **) ppSvc ) );

	return hr;
	}	// getSvc

HRESULT nSpaceClient :: listen ( const WCHAR *szPath, BOOL bL,
											nSpaceClientCB *pCBAlt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Listen/unlisten to an emitter in the namespace.
	//
	//	PARAMETERS
	//		-	szPath is the namespace path
	//		-	bL is TRUE to listen, FALSE to unlisten
	//		-	pCBAlt is an alternate callback to receive notifications.
	//
	//	RETURN TYPE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	BSTR				bstrPath	= NULL;
	INamespaceX		*pSpc		= NULL;
	IListenX			*pLstn	= NULL;
	adtValue			vL;

	// Service ptr.
	CCLTRY ( getSvc ( &pSpc ) );

	// Create full path
	CCLTRY ( pathCreate ( szPath, &bstrPath ) );

	// Debug
	dbgprintf ( L"nSpaceClient::%s:bstrPath %s pCBAlt %p {\r\n", 
						(bL) ? L"listen" : L"unlisten", bstrPath, pCBAlt );

	// Listen
	if (hr == S_OK && bL)
		{
		// Create new listen object
		CCLTRYE ( (pLstn = new nSpaceClientL(this,(pCBAlt != NULL) ? pCBAlt : pCB)) != NULL, E_OUTOFMEMORY );
		_ADDREF(pLstn);

		// Execute listen
		CCLTRY ( pSpc->listen ( bstrPath, pLstn ) );

		// Keep track of listen
		CCLTRY ( pLstns->store ( adtString(bstrPath), adtIUnknown(pLstn) ) );
		}	// if

	// Unlisten
	else if (hr == S_OK && !bL)
		{
		// Access previous listen
		CCLTRY	( pLstns->load ( adtString(bstrPath), vL ) );

		// Object
		CCLTRYE	( vL.vtype == VTYPE_UNK && vL.punk != NULL, E_UNEXPECTED );
		CCLTRY	( _QI(vL.punk,__uuidof(IListenX),&pLstn) );

		// Disable callback
		CCLOK ( ((nSpaceClientL *)pLstn)->pCB = NULL; )

		// Unlisten
		CCLTRY	( pSpc->unlisten ( bstrPath, pLstn ) );

		// Remove from dictionary
		pLstns->remove ( adtString(bstrPath) );
		}	// else if

	// Debug
//	if (hr != S_OK)
		dbgprintf ( L"} nSpaceClient::%s:pLstn %p hr 0x%x\r\n", 
						(bL) ? L"listen" : L"unlisten", pLstn, hr );

	// Clean up
	_FREEBSTR(bstrPath);
	_RELEASE(pLstn);
	_RELEASE(pSpc);

	return hr;
	}	// listen

HRESULT nSpaceClient :: load ( const WCHAR *szPath, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Get a value from the namespace.
	//
	//	PARAMETERS
	//		-	szPath is the namespace path
	//		-	v will receive the value 
	//
	//	RETURN TYPE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	INamespaceX	*pSpc		= NULL;
	BSTR			bstrPath	= NULL;

	// Timing
	DWORD	dwThen = GetTickCount();

	// Service ptr.
	CCLTRY	( getSvc ( &pSpc ) );

	// Create full path
	CCLTRY	( pathCreate ( szPath, &bstrPath ) );

	// Retrieve value
	CCLTRY ( varL.clear() );
	CCLTRY ( pSpc->load ( bstrPath, &varL ) );

	// Convert to local value
	CCLTRY ( varL.toValue(v) );

	// Clean up
	_FREEBSTR(bstrPath);
	_RELEASE(pSpc);

	// Clean up
	if (hr != S_OK)
		dbgprintf ( L"nSpaceClient::load:%s:%d ms\r\n", szPath, (GetTickCount()-dwThen) );

	return hr;
	}	// load

HRESULT nSpaceClient :: open ( const WCHAR *pwCmdLine, BOOL bShare,
											nSpaceClientCB *_pCB )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Open a connection to an nSpace namespace.
	//
	//	PARAMETERS
	//		-	pCmdLine is the command line to use for the namespace
	//		-	bShare is true to open an existing or create a new service or 
	//			false to create a private one.
	//		-	_pCB is the callback client for notifications.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	IShellX			*pSh	= NULL;
	INamespaceX		*pSpc	= NULL;
	IUnknown			*pUnk	= NULL;
	BSTR				bstr	= NULL;
	CLSID				clsid;
	int				dbg	= 0;

	// Setup
	pCB = _pCB;

	// Obtain class Id
	CCLTRY ( CLSIDFromProgID ( L"nSpace.ShellX", &clsid ) );

	// Create nSpace link
	CCLTRY ( CoCreateInstance ( clsid, NULL, CLSCTX_LOCAL_SERVER, IID_IUnknown, (void **) &pUnk ) );

	// Interface
	CCLTRY ( _QI ( pUnk, IID_IShellX, &pSh ) );

	// Access namespace
	CCLTRYE ( (bstr = SysAllocString ( pwCmdLine )) != NULL, E_OUTOFMEMORY );
	CCLTRY  ( pSh->open  ( bstr, bShare, &pSpc ) );

	// Register interface with global interface table in order to support calls from any
	// apartment thread.
	CCLTRY  ( CoCreateInstance ( CLSID_StdGlobalInterfaceTable, NULL, CLSCTX_ALL,
											IID_IGlobalInterfaceTable, (void **) &pGIT ) );
	CCLTRY ( pGIT->RegisterInterfaceInGlobal ( pSpc, IID_INamespaceX, &dwSvc ) );

	// Create dictionary to keep track of listens
	CCLTRY  ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pLstns ) );

	// Clean up
	if (bstr != NULL)
		SysFreeString(bstr);
	_RELEASE(pSpc);
	_RELEASE(pSh);
	_RELEASE(pUnk);
	if (hr != S_OK)
		{
		if (dwSvc != 0)
			pGIT->RevokeInterfaceFromGlobal ( dwSvc );
		dwSvc = 0;
		_RELEASE(pGIT);
		}	// if

	// Debug
	dbgprintf ( L"nSpaceClient::open:0x%x:%s\r\n", hr, pwCmdLine );

	return hr;
	}	// open

HRESULT nSpaceClient :: pathCreate ( const WCHAR *szPath, BSTR *pbstrPath )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	CreatePut a value into the namespace.
	//
	//	PARAMETERS
	//		-	szPath is a relative or absolute path to use.
	//		-	pbstrPath will receive the full path
	//
	//	RETURN TYPE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	size_t	len;

	// Allocate BSTRs for the path
	CCLOK		(  len = ((pwRoot != NULL) ? wcslen(pwRoot) : 0) + wcslen(szPath); )
	CCLTRYE	( (*pbstrPath = SysAllocStringLen ( NULL, (UINT)len )) != NULL, E_OUTOFMEMORY );

	// If the path is relative, prepend the root if available
	CCLOK ( wcscpy_s (	*pbstrPath, len+1, 
								(pwRoot != NULL && szPath[0] != WCHAR('/')) ? pwRoot : L"" ); )

	// Provided path
	CCLOK		( wcscat_s ( *pbstrPath, len+1, szPath ); )

	return hr;
	}	// pathCreate

HRESULT nSpaceClient :: root ( const WCHAR *_pwRoot )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Set a default root location for namespace operations.
	//
	//	PARAMETERS
	//		-	_pwRoot is the root location
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	pwRoot	= _pwRoot;
	return S_OK;
	}	// root

HRESULT nSpaceClient :: store ( const WCHAR *szPath, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Store a value into the namespace.
	//
	//	PARAMETERS
	//		-	szPath is the namespace path
	//		-	v is the value to put into the namespace.
	//
	//	RETURN TYPE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	INamespaceX		*pSpc		= NULL;
	BSTR				bstrPath	= NULL;

	// Timing
	DWORD	dwThen = GetTickCount();

	// Service ptr.
	CCLTRY ( getSvc ( &pSpc ) );

	// Create BSTR based full path
	CCLTRY	( pathCreate ( szPath, &bstrPath ) );

	// Convert to variant
	CCLOK ( varS = v; )

	// Execute
	CCLTRY ( pSpc->store ( bstrPath , &varS ) );

	// Clean up
	if (hr != S_OK)
		dbgprintf ( L"nSpaceClient::store:bstrPath %s:type %d:hr 0x%x:%d ms\r\n", 
						bstrPath, v.vtype, hr, (GetTickCount()-dwThen) );

	// Clean up
	_FREEBSTR(bstrPath);
	_RELEASE(pSpc);

	return hr;
	}	// store

/////////////////
// nSpaceClientL
/////////////////

nSpaceClientL :: nSpaceClientL ( nSpaceClient *_pThis, nSpaceClientCB *_pCB )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pThis is the parent object.
	//		-	_pCB is the callback object
	//
	////////////////////////////////////////////////////////////////////////
	pThis		= _pThis;
	pCB		= _pCB;
	}	// nSpaceClientL

HRESULT nSpaceClientL :: receive ( BSTR bstrRoot, BSTR bstrLoc, VARIANT *var )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a listened emitter emits a value.  Value is
	//			contained in persistence buffer.
	//
	//	PARAMETERS
	//		-	bstRoot is the root listened location
	//		-	bstrLoc is the relative path to the value from the root.
	//		-	var is the received value
	//
	//	RETURN TYPE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtValue		vRx;

	// Callback still valid ?
	if (pCB != NULL)
		{
		// Thread protection
		csRx.enter();

		// Convert to local value
		CCLOK ( varL = var; )
		CCLTRY( varL.toValue(vRx) );

		// Notify callback object
//		dbgprintf ( L"nSpaceClientL::receive:pCB %p\r\n", pCB );
		pCB->onReceive ( bstrRoot, bstrLoc, vRx );

		// Clean up
		adtValue::clear(vRx);

		// Thread protection
		csRx.leave();
		}	// if

	return S_OK;
	}	// onReceive

///////////////
// nSpacePerX
///////////////
/*
nSpacePerX :: nSpacePerX ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pPerX	= NULL;
	pPerL	= NULL;
	pStmL	= NULL;
	pPerS	= NULL;
	pStmS	= NULL;
	}	// nSpacePerX

HRESULT nSpacePerX :: available ( U64 *puAv )
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
	HRESULT	hr	= S_OK;

	// Can only return the number of bytes available in the cache
	CCLTRY ( pStmL->available ( puAv ) );

	return hr;
	}	// available

HRESULT nSpacePerX :: construct ( void )
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
	HRESULT		hr		= S_OK;

	// Create parsers for values
	CCLTRY ( COCREATE(L"Io.StmPrsBin",IID_IStreamPersist,&pPerL) );
	CCLTRY ( COCREATE(L"Io.StmPrsBin",IID_IStreamPersist,&pPerS) );

	// Create load buffer for incoming values
	CCLTRY ( COCREATE ( L"Io.StmMemory", IID_IByteStream, &pStmL ) );

	return hr;
	}	// construct

HRESULT nSpacePerX :: copyTo ( IByteStream *pStmDst, U64 uSz, U64 *puSz )
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
	U8			cpybufr[4096];
	U64		nio,nr;

	// State check
	CCLTRYE ( pPerX != NULL, ERROR_INVALID_STATE );
	CCLTRYE ( uSz != 0, ERROR_INVALID_STATE );
 
	// Setup
	if (puSz != NULL)
		*puSz	= 0;

	// Read/write buffer
	while (hr == S_OK && uSz)
		{
		// Read next block
		CCLOK ( nio = (sizeof(cpybufr) < uSz) ? sizeof(cpybufr) : uSz; )
		CCLTRY( read ( cpybufr, nio, &nr ) );

		// Write block to byte cache
		CCLTRY ( pStmS->write ( cpybufr, nr, NULL ) );

		// Next block
		CCLOK ( uSz -= nr; )
		if (hr == S_OK && puSz != NULL)
			*puSz += nr;
		}	// while

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"nSpacePerX::copyTo:Failed:0x%x\r\n", hr );

	return hr;
	}	// copyTo

void nSpacePerX :: destruct ( void )
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
	_RELEASE(pStmS);
	_RELEASE(pStmL);
	_RELEASE(pPerX);
	_RELEASE(pPerL);
	_RELEASE(pPerS);
	}	// destruct

HRESULT nSpacePerX :: flush ( void )
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
	return pStmS->flush();
	}	// flush

HRESULT nSpacePerX :: load ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Load the current value from the buffer.
	//
	//	PARAMETERS
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Prepare load stream
	CCLTRY ( pStmL->seek ( 0, STREAM_SEEK_SET, NULL ) );

	// Load value from stream
//	CCLTRY ( pPerL->load ( pStmL, v ) );

	return hr;
	}	// load

HRESULT nSpacePerX :: read ( void *pvBfr, U64 nio, U64 *pnio )
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
	return pStmL->read ( pvBfr, nio, pnio );
	}	// read

HRESULT nSpacePerX :: reset ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IPersistX
	//
	//	PURPOSE
	//		-	Reset the state of the value buffer.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Prepare stream for new incoming data
	CCLTRY ( pStmL->seek ( 0, STREAM_SEEK_SET, NULL ) );

	return hr;
	}	// reset

HRESULT nSpacePerX :: save ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Save a value to the remote buffer
	//
	//	PARAMETERS
	//		-	v contains the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// State check
	CCLTRYE ( pPerX != NULL, ERROR_INVALID_STATE );

	// Prepare other side to receive new value
	CCLTRY ( pPerX->reset() );

	// Convert value to variant
	CCLOK ( varS = v; )

	// Send value
	CCLTRY ( pPerX->write ( &varS ) );

	return hr;
	}	// save

HRESULT nSpacePerX :: seek ( S64 sPos, U32 uFrom, U64 *puPos )
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
	// Not supported
	return E_NOTIMPL;
	}	// seek

HRESULT nSpacePerX :: setOutput ( IUnknown *pUnk )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Set the output value persistence interface.
	//
	//	PARAMETERS
	//		-	pUnk is the interface
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pDct	= NULL;
	IResource	*pRes	= NULL;

	// Release previous state
	_RELEASE(pPerX);
	_RELEASE(pStmS);

	// Obtain interface
	CCLTRY ( _QISAFE(pUnk,IID_IPersistX,&pPerX) );

	// Create a byte cache for outgoing saved values
	CCLTRY ( COCREATE ( L"Io.ByteCache", IID_IByteStream, &pStmS ) );

	// Prepare byte cache options
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
	CCLTRY ( pDct->store ( adtString(L"Stream"),		adtIUnknown((IByteStream *)this) ) );
	CCLTRY ( pDct->store ( adtString(L"Size"),		adtInt(SIZE_PERSIST_CACHE) ) );
	CCLTRY ( pDct->store ( adtString(L"ReadOnly"),	adtBool(false) ) );

	// 'Open' the byte cache on own stream
	CCLTRY ( _QI(pStmS,IID_IResource,&pRes) );
	CCLTRY ( pRes->open ( pDct ) );

	// Clean up
	_RELEASE(pRes);
	_RELEASE(pDct);

	return hr;
	}	// setOutput

HRESULT nSpacePerX :: setSize ( U64 uSz )
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
	// Not supported
	return E_NOTIMPL;
	}	// setSize

HRESULT nSpacePerX :: write ( void const *pcvBfr, U64 nio, U64 *pnio )
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
	//		-	pcvBfr contains the data to write
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	SAFEARRAY		*psa		= NULL;
	VOID				*pvData	= NULL;
	SAFEARRAYBOUND	sab;

	// State check
	CCLTRYE ( pPerX != NULL, ERROR_INVALID_STATE );

	// Allocate array
	if (hr == S_OK)
		{
		sab.lLbound		= 0;
		sab.cElements	= (ULONG)nio;
		CCLTRYE ( (psa = SafeArrayCreate ( VT_I1, 1, &sab )) != NULL,
						E_OUTOFMEMORY );
		}	// if

	// Can't avoid this copy ? Ridiculous.. should be able to use
	// an existing buffer
	CCLTRY ( SafeArrayAccessData ( psa, &pvData ) );
	CCLOK  ( memcpy ( pvData, pcvBfr, nio ); )

	// Clean up
	if (pvData != NULL)
		SafeArrayUnaccessData ( psa );

	// Write to remote buffer
	CCLTRY ( pPerX->write ( psa ) );

	// Result
	if (pnio != NULL)
		*pnio = (hr == S_OK) ? nio : 0;

	// Clean up
	if (psa != NULL)
		SafeArrayDestroyDescriptor ( psa );

	// Use existing buffer ?  This does not work..

	// Create safe array
//	CCLTRY ( SafeArrayAllocDescriptorEx ( VT_I1, 1, &psa ) );

	// Use own data in safe array
//	if (hr == S_OK)
//		{
//		psa->pvData							= (PVOID)pcvBfr;
//		psa->rgsabound[0].cElements	= (ULONG)nio;
//		psa->rgsabound[0].lLbound		= 0;
//		}	// if


	return hr;
	}	// write

HRESULT nSpacePerX :: write ( VARIANT *var )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IPersistX
	//
	//	PURPOSE
	//		-	Write data to the value persistence buffer.
	//
	//	PARAMETERS
	//		-	psa is the array of data
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	void		*pvData	= NULL;
	U64		nx	= 0;

	// Access ptr to data
	CCLTRY ( SafeArrayAccessData ( psa, &pvData ) );

	// Append data to current stream
	CCLTRY ( pStmL->write ( pvData, psa->rgsabound[0].cElements, &nx ) );
//	CCLOK ( dbgprintf ( L"nSpacePerX::write %d bytes\r\n", nx ); )

	// Success ?
	CCLTRYE ( (nx == psa->rgsabound[0].cElements), E_OUTOFMEMORY );

	// Clean up
	if (pvData != NULL)
		SafeArrayUnaccessData ( psa );

	return hr;
	}	// write

*/
