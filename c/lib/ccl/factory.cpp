////////////////////////////////////////////////////////////////////////
//
//								FACTORY.CPP
//
//				Implementation of the CCL factory class
//
////////////////////////////////////////////////////////////////////////

#include "ccl.h"

// Globals
extern CCLENTRY		cclobjlist[];
extern U32				dwLockCount;
#ifdef	_DEBUG
extern LONG				dwCtd[];
#endif

CCLFactory::CCLFactory ( const CLSID *pclsid )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IUnknown
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	pclsid is the class ID of the object to create
	//
	////////////////////////////////////////////////////////////////////////
	U16 i;

	// Setup
	objidx	= 0xffffffff;
	unkOuter	= (IUnknown *) ((IInnerUnknown *)this);

	// Search for object in table
//	dbgprintf ( L"CCLFactory::CCLFactory:this %p\r\n", this );
	for (i = 0;cclobjlist[i].clsid != NULL && objidx == 0xffffffff;++i)
		{
//		dbgprintf ( L"CCLFactory::CCLFactory:0x%x,0x%x\n", ((U32 *) &(*(cclobjlist[i].clsid)))[0], ((U32 *) pclsid)[0] );
//		dbgprintf ( L"CCLFactory::CCLFactory:0x%x,0x%x\n", ((U32 *) &(*(cclobjlist[i].clsid)))[1], ((U32 *) pclsid)[1] );
//		dbgprintf ( L"CCLFactory::CCLFactory:0x%x,0x%x\n", ((U32 *) &(*(cclobjlist[i].clsid)))[2], ((U32 *) pclsid)[2] );
//		dbgprintf ( L"CCLFactory::CCLFactory:0x%x,0x%x\n", ((U32 *) &(*(cclobjlist[i].clsid)))[3], ((U32 *) pclsid)[3] );

		if (IsEqualGUID ( *(cclobjlist[i].clsid), *pclsid ))
			objidx = i;
		}	// for
	if (objidx == 0xffffffff)
		{
//		#ifdef	UNDER_CE
		dbgprintf ( L"CCLFactory::CCLFactory:Object not found 0x%x : objidx 0x%x (i %d)\r\n",
						pclsid->Data1, objidx, i );
//		#endif
		}	// if

	}	// CCLFactory

CCLFactory::CCLFactory ( const WCHAR *pwId )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IUnknown
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	pwId is the string Id of the object to create
	//
	////////////////////////////////////////////////////////////////////////
	U16 i;

	// Setup
	objidx	= 0xffffffff;
	unkOuter	= (IUnknown *) ((IInnerUnknown *)this);

	// Search for object in table
//	dbgprintf ( L"CCLFactory::CCLFactory:%p,%p,%s\r\n", 
//					cclobjlist, cclobjlist[0].clsid, pwId );
	for (i = 0;cclobjlist[i].clsid != NULL && objidx == 0xffffffff;++i)
		{
//		dbgprintf ( L"CCLFactory::CCLFactory:%s\n", cclobjlist[i].progid );
//		dbgprintf ( L"CCLFactory::CLLFactory:0x%x,0x%x\n", ((U32 *) &(*(cclobjlist[i].clsid)))[1], ((U32 *) &clsid)[1] );
//		dbgprintf ( L"CCLFactory::CLLFactory:0x%x,0x%x\n", ((U32 *) &(*(cclobjlist[i].clsid)))[2], ((U32 *) &clsid)[2] );
//		dbgprintf ( L"CCLFactory::CLLFactory:0x%x,0x%x\n", ((U32 *) &(*(cclobjlist[i].clsid)))[3], ((U32 *) &clsid)[3] );
//		dbgprintf ( L"CCLFactory::CCLFactory:%d,%s\r\n", i, cclobjlist[i].progid );
		if (!WCASECMP(cclobjlist[i].progid, pwId ))
			objidx = i;
		}	// for
	if (objidx == 0xffffffff)
		{
//		#ifdef	UNDER_CE
		dbgprintf ( L"CCLFactory::CCLFactory:Object not found '%s' : objidx 0x%x (i %d)\r\n",
						pwId, objidx, i );
//		#endif
		}	// if

	}	// CCLFactory

HRESULT CCLFactory :: CreateInstance ( IUnknown *pUnkOuter,
													REFIID iid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IClassFactory
	//
	//	PURPOSE
	//		-	Creates an instance of the specified object.
	//
	//	PARAMETERS
	//		-	pUnkOuter is a ptr. to the outer unknown
	//		-	iid is the interface ID
	//		-	ppv will receive a ptr. to the object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	CCLObject		*pObj		= NULL;
	IInnerUnknown	*pUnkIn	= NULL;

	// Setup
	(*ppv) = NULL;

	// Object found in table ?
	if (objidx == 0xffffffff) return ERROR_NOT_FOUND;

	// Valid aggregation ?
	if (pUnkOuter != NULL && !IsEqualGUID ( iid, IID_IUnknown ))
		return CLASS_E_NOAGGREGATION;

	// Initialize object
	if (hr == S_OK)
		{
		// Create object
		pObj		= (*cclobjlist[objidx].create)();
		pUnkIn	= pObj;									// Inner unknown ptr.

		// Enable aggregation protection
		pObj->refcnt++;

		// Outer unknown
		pObj->unkOuter =	(pUnkOuter != NULL) ? pUnkOuter :
								(IUnknown *) ((IInnerUnknown *) pObj);

		// Construct object
		hr = pObj->construct();

		// Debug (reference counting)
		#ifdef	_DEBUG
		if (hr == S_OK && objidx < 255)
			{
			pObj->objidx = objidx;
			InterlockedIncrement(&(dwCtd[objidx]));
			}	// if
		#endif

		// Release aggregation protection
		pObj->refcnt--;
		}	// if

	// Required interface
	if (hr == S_OK)
		{
		// If aggregating, return a ptr. to the object's 'InnerUnknown'
		if (pUnkOuter != NULL)
			{
			// 'InnerUnknown'
			(*ppv)	= (IUnknown *) (pUnkIn);
			pUnkIn->InnerAddRef();
			}	// if
		else
			hr = pObj->QueryInterface ( iid, ppv );
		}	// if

	// Debug
	#ifdef	_DEBUG
	#endif

	// Clean up
	if (hr != S_OK && pObj != NULL) delete pObj;

	return hr;
	}	// CreateInstance

HRESULT CCLFactory :: LockServer ( BOOL bLock )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IClassFactory
	//
	//	PURPOSE
	//		-	Locks modules for use.
	//
	//	PARAMETERS
	//		-	bLock is TRUE to lock
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	if			(bLock)						++dwLockCount;
	else if	(!bLock && dwLockCount) --dwLockCount;
	return S_OK;
	}	// LockServer

