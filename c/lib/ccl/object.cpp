////////////////////////////////////////////////////////////////////////
//
//								OBJECT.CPP
//
//				Implementation of the CCL base class
//
////////////////////////////////////////////////////////////////////////

#include "ccl.h"

// Globals
extern U32		dwObjCount;
#ifdef	_DEBUG
extern LONG		dwDtd[];
#endif

CCLObject::CCLObject ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IUnknown
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	refcnt	= 0;

	// For manually created objects...
	unkOuter	= (IUnknown *) ((IInnerUnknown *) this);

	// Module
	++dwObjCount;

	// Debug
	#ifdef	_DEBUG
	objidx	= -1;
	#endif
	}	// CCLObject

CCLObject::~CCLObject ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IUnknown
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////

	// Module
	if (dwObjCount) --dwObjCount;

	// Debug
	#ifdef	_DEBUG
	if (objidx != -1)
		InterlockedIncrement(&(dwDtd[objidx]));
	#endif
	}	// ~CCLObject

HRESULT CCLObject :: cclRegister ( bool bReg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when object is registered/unregistered.
	//
	//	PARAMETERS
	//		-	bReg is true when object is registered
	//
	//	RETURN VALUE
	//		S_OK is successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// cclRegister

HRESULT CCLObject :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK is successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// construct

void CCLObject :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
	}	// destruct

HRESULT CCLObject :: _InnerQueryInterface ( const CCLINTF *pintfs,
															REFIID iid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Search interface table for interface.
	//
	//	PARAMETERS
	//		-	pintfs is the interface table
	//		-	iid specifies the interface ID
	//		-	ppv will receive the ptr.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	const CCLINTF	*pi;

	// IUnknown ?
	if (IsEqualGUID ( IID_IUnknown, iid ))
		(*ppv) = (IUnknown *) ((IInnerUnknown *)this);
	else
		{
		for (pi = pintfs,(*ppv)=NULL;pi->piid;++pi)
			if (IsEqualGUID ( *(pi->piid), iid ))
				{
				// (uip) 0 = end, 1 = offset, 2 = pointer
				if (pi->itype == 1)
					(*ppv) = (UINT_PTR *) (((UINT_PTR)(this)) + pi->uip);
				else if (pi->itype == 2)
					{
					// Ptr. is the inner unknown of the aggregated object
					IUnknown		*punk	= (IUnknown *) (*((UINT_PTR *)(((UINT_PTR)this) + pi->uip)));
					return punk->QueryInterface ( iid, ppv );
					}	// else
				break;
				}	// if
		}	// else

	// Found ?
	if ( (*ppv) != NULL) ((IUnknown *)(*ppv))->AddRef();
	else						hr = E_NOINTERFACE;

	return hr;
	}	// _InnerQueryInterface

ULONG CCLObject :: InnerAddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IUnknown
	//
	//	PURPOSE
	//		-	Increments the reference count on the object.
	//
	//	RETURN VALUE
	//		New reference count
	//
	////////////////////////////////////////////////////////////////////////
	InterlockedIncrement(&refcnt);
	return refcnt;
	}	// InnerAddRef

ULONG CCLObject :: InnerRelease ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IUnknown
	//
	//	PURPOSE
	//		-	Decrements the reference count on the object.
	//
	//	RETURN VALUE
	//		New reference count
	//
	////////////////////////////////////////////////////////////////////////
	InterlockedDecrement(&refcnt);
	S32	lrefcnt = refcnt;

	// If not zero, done
	if (lrefcnt != 0) return lrefcnt;

	// Destroy object
	destruct();
	delete this;

	return 0;
	}	// InnerRelease

void CCLObject :: operator delete ( void *p )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Delete operator for class
	//
	//	PARAMETERS
	//		-	p is the object
	//
	////////////////////////////////////////////////////////////////////////
	_FREEMEM(p);
	}	// operator delete

void *CCLObject :: operator new ( size_t sz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	New operator for class
	//
	//	PARAMETERS
	//		-	sz is the size of the object
	//
	//	RETURN VALUE
	//		Ptr to object
	//
	////////////////////////////////////////////////////////////////////////
	return _ALLOCMEM((U32)sz);
	}	// operator new

