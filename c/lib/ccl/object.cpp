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

ULONG CCLObject :: AddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Increments the reference count for an interface on an object.
	//! \return The method returns the new reference count. 
	//
	////////////////////////////////////////////////////////////////////////
	return unkOuter->AddRef();
	}	// AddRef

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
	//! \brief Called when an object has been created, a reference count is
	//!	held on the object to allow for outer unknown creation.
	//! \return S_OK if successful, on error object creation will fail.
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// construct

void CCLObject :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Called when the object's reference count reaches zero but
	//!	before the object is 'delete'd.
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

HRESULT CCLObject :: QueryInterface ( REFIID iid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Retrieves pointers to the supported interfaces on an object.
	//! \param iid is the identifier of the interface being requested.
	//! \param ppv will receive the address of a pointer that receives the 
	//!	interface pointer requested in the riid parameter.
	//! \return S_OK if the interface is supported
	//
	////////////////////////////////////////////////////////////////////////
	return unkOuter->QueryInterface ( iid, ppv );
	}	// QueryInterface

ULONG CCLObject :: Release ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Decrements the reference count for an interface on an object.
	//! \return The method returns the new reference count. 
	//
	////////////////////////////////////////////////////////////////////////
	return unkOuter->Release();
	}	// Release
/*
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
*/
