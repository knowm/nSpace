////////////////////////////////////////////////////////////////////////
//
//									CCL.CPP
//
//					Main file for the COM Compatibility Layer
//
////////////////////////////////////////////////////////////////////////

#include "ccl.h"
#include "../adtl/adtl.h"
#if      __unix__ || __APPLE__
#include <libgen.h>
#include <dlfcn.h>
#endif

// Globals
extern		CCLENTRY		cclobjlist[];				// Objects for module
HINSTANCE	ccl_hInst	= NULL;						// Module handle
U32			dwLockCount = 0;							// Module lock count
U32			dwObjCount	= 0;							// Object reference count
#ifdef		_DEBUG
LONG			dwCtd[255];									// Constructed
LONG			dwDtd[255];									// Destructed
#endif

// Prototypes
STDAPI cclRegister	( CLSID, const wchar_t *, bool );

#ifdef	_WIN32

BOOL cclDllMain ( HANDLE _hInst, U32 dwReason )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	CCL Entry point into DLL
	//
	//	PARAMETERS
	//		-	_hInst is a handle to the module
	//		-	dwReason specifies the reason for calling this function
	//
	//	RETURN VALUE
	//		TRUE if successful
	//
	////////////////////////////////////////////////////////////////////////
	if			(dwReason == DLL_PROCESS_ATTACH)
		{
		ccl_hInst = (HINSTANCE) _hInst;
		#ifdef	_DEBUG
		memset ( dwCtd, 0, sizeof(dwCtd) );
		memset ( dwDtd, 0, sizeof(dwDtd) );
		#endif
		}	// if
	#ifdef	_DEBUG
	else if	(dwReason == DLL_PROCESS_DETACH)
		{
		WCHAR	dllname[MAX_PATH];
		U32	i,j = 0;
		GetModuleFileName ( ccl_hInst, dllname, sizeof(dllname)/sizeof(WCHAR) );
		for (i = 0;i < sizeof(dwCtd)/sizeof(U32);++i)
			if (dwCtd[i] != dwDtd[i])
				{
				if (!j) { dbgprintf ( L"** MISMATCH:%s:", dllname ); j = 1; }
				dbgprintf ( L"(%s(%d),%d(+%d,-%d)) ", cclobjlist[i].progid, i, 
								dwCtd[i]-dwDtd[i], dwCtd[i], dwDtd[i] );
				}	// if
		if (j) dbgprintf ( L"\r\n" );
		}	// else if
	#endif

	return TRUE;
	}	// cclDllMain

extern "C"
HRESULT cclCreateObject ( const wchar_t *pId, IUnknown *pOuter, 
									REFIID iid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Creates the specified object.
	//
	//	PARAMETERS
	//		-	pId is the class Id
	//		-	pOuter is the outer unknown
	//		-	iid is the interface to query for
	//		-	ppv will receive the object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	CLSID			clsid;
	WCHAR			wIdFull[255];
	const WCHAR	*dot;
	U32			cntIdFull;

	// The Windows implementation uses the object Id as the 'ProgId' and
	// uses COM to create the object.

	// System prefix is assumed to be nSpace but not required.  Prepend if missing.
	cntIdFull = sizeof(wIdFull)/sizeof(wIdFull[0]);
	if (	(dot = wcschr ( pId, '.' )) == NULL ||
			(dot = wcschr ( dot+1, '.' )) == NULL )
		{
		WCSCPY ( wIdFull, cntIdFull, L"nSpace." );
		WCSCAT ( wIdFull, cntIdFull, pId );
		}	// if
	else
		WCSCPY ( wIdFull, cntIdFull, pId );

	// Object class Id
	CCLTRY ( CLSIDFromProgID ( wIdFull, &clsid ) );

	// Create object
	CCLTRY ( CoCreateInstance ( clsid, pOuter, CLSCTX_ALL, iid, ppv ) );

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"cclCreateObject:Fail:%s:0x%x\r\n", pId, hr );

	return hr;
	}	// cclCreateObject

extern "C"
HRESULT cclGetFactory ( const wchar_t *pId, REFIID iid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Obtain class factory for specified object.
	//
	//	PARAMETERS
	//		-	pId is the class Id
	//		-	iid is the factory interface to query for
	//		-	ppv will receive the factory object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;
	CLSID		clsid;

	// The Windows implementation uses the object Id as the 'ProgId' and
	// uses COM to get the class factory

	// Object class Id
	CCLTRY ( CLSIDFromProgID ( pId, &clsid ) );
	
	// Class factory	
	CCLTRY ( CoGetClassObject ( clsid, CLSCTX_ALL, NULL, iid, ppv ) );

	return hr;
	}	// cclGetFactory
	
#endif

STDAPI DllCanUnloadNow ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called by the operating system to see if is ok to unload
	//			the DLL.
	//
	//	RETURN VALUE
	//		S_OK if ok to be unloaded
	//
	////////////////////////////////////////////////////////////////////////
	return (!dwLockCount && !dwObjCount) ? S_OK : S_FALSE;
	}	// DllCanUnloadNow

STDAPI DllGetClassObject ( REFCLSID refclsid, REFIID refiid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called by the operating system to grab a class factory for
	//			the specified object.
	//
	//	PARAMETERS
	//		-	refclsid specifies the class ID
	//		-	refiid specifies the interface on the class factory
	//		-	ppv will receive a ptr. to the factory
	//
	//	RETURN VALUE
	//		S_OK if ok to be unloaded
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Known factory capabilities
	if (	!IsEqualGUID ( refiid, IID_IUnknown ) &&
			!IsEqualGUID ( refiid, IID_IClassFactory ) )
		return E_NOINTERFACE;

	// Factory for object
	CCLFactory *
	factory = new CCLFactory ( &refclsid );
	_ADDREF(factory);

	// Valid object ?
	if (factory != NULL && factory->objidx != 0xffffffff)
		{
		// Class factory interface
		IClassFactory	*
		fct = factory;

		// Result
		fct->AddRef();
		(*ppv) = fct;
		}	// if
	else
		hr = CLASS_E_CLASSNOTAVAILABLE;

	// Clean up
	_RELEASE(factory);

	return hr;
	}	// DllGetClassObject

STDAPI DllRegisterServer ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called by the operating system to register the objects
	//			in the module.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	CCLObject	*cclobj	= NULL;

	// register each object
	for (U16 i = 0;cclobjlist[i].clsid != NULL && hr == S_OK;++i)
		{
		// Global registration
		CCLTRY (cclRegister (	*(cclobjlist[i].clsid),
										(cclobjlist[i].progid),
										true ));

		// Object specific registration
		// TODO: For performance on do this on request, add flag to table ?
//		CCLTRYE	((cclobj	= cclobjlist[i].create()), E_OUTOFMEMORY );
//		CCLTRY	(cclobj->cclRegister(true));

		// Clean up
		if (cclobj != NULL)
			delete cclobj;
		}	// for

	return hr;
	}	// DllRegisterServer

STDAPI DllUnregisterServer ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called by the operating system to unregister the objects
	//			in the module.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	CCLObject	*cclobj	= NULL;

	// register each object
	for (U16 i = 0;cclobjlist[i].clsid != NULL && hr == S_OK;++i)
		{
		// Object specific unregistration
		CCLTRYE	((cclobj	= cclobjlist[i].create()), E_OUTOFMEMORY );
		CCLTRY	(cclobj->cclRegister(false));

		// Global deregistration
		CCLTRY (cclRegister (	*(cclobjlist[i].clsid),
										(cclobjlist[i].progid),
										false ));

		// Clean up
		if (cclobj != NULL)
			delete cclobj;
		}	// for

	return hr;
	}	// DllUnregisterServer

// Linux specific stuff

#if	__unix__ || __APPLE__

BOOL cclDllMain ( HANDLE _hInst, U32 dwReason )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	CCL Entry point into DLL
	//
	//	PARAMETERS
	//		-	_hInst is a handle to the module
	//		-	dwReason specifies the reason for calling this function
	//
	//	RETURN VALUE
	//		TRUE if successful
	//
	////////////////////////////////////////////////////////////////////////
	return TRUE;
	}	// cclDllMain

STDAPI cclGetClassObject ( const wchar_t *pwId, REFIID refiid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called by the operating system to grab a class factory for
	//			the specified object.
	//
	//	PARAMETERS
	//		-	pwId is the class Id
	//		-	refiid specifies the interface on the class factory
	//		-	ppv will receive a ptr. to the factory
	//
	//	RETURN VALUE
	//		S_OK if ok to be unloaded
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Known factory capabilities
	if (	!IsEqualGUID ( refiid, IID_IUnknown ) &&
			!IsEqualGUID ( refiid, IID_IClassFactory ) )
		return E_NOINTERFACE;

	// Factory for object
	CCLFactory *
	factory = new CCLFactory ( pwId );
	_ADDREF(factory);

	// Valid object ?
	if (factory != NULL && factory->objidx != 0xffffffff)
		{
		// Class factory interface
		IClassFactory	*
		fct = factory;

		// Result
		fct->AddRef();
		(*ppv) = fct;
		}	// if
	else
		hr = CLASS_E_CLASSNOTAVAILABLE;

	// Clean up
	_RELEASE(factory);

	return hr;
	}	// cclGetClassObject

// Gcc has linker problems without this defined using pure functions...

extern "C"
void __pure_virtual()
	{
	}	// __pure_virtual

#endif
