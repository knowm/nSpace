////////////////////////////////////////////////////////////////////////
//
//									CCL.H
//
//			Include file for the COM Compatibility Layer library
//
////////////////////////////////////////////////////////////////////////

#ifndef	CCL_H
#define	CCL_H

// Includes
#include	"../sysl/sysl.h"

// COM implementation
#define	CCL_OBJLIST_BEGIN()			CCLENTRY cclobjlist[] = {
#ifdef	_WIN32
#define	CCL_OBJLIST_ENTRY(a)			{ &(CLSID_##a), a::CreateObject, CCL_OBJ_PREFIX L"." CCL_OBJ_MODULE L"." L#a },
#else
#define	CCL_OBJLIST_ENTRY(a)			{ &(CLSID_##a), a::CreateObject, CCL_OBJ_PREFIX L"." CCL_OBJ_MODULE L"." L## #a },
#endif
#define	CCL_OBJLIST_END()				{ NULL, NULL } };															\
												CCL_DLLMAIN()
#define	CCL_OBJLIST_END_EX(a)		{ NULL, NULL } };															\
												CCL_DLLMAIN_EX(a)
#define	CCL_OBJECT_BEGIN_INT(a)		public :																		\
												STDMETHOD_(ULONG,AddRef)	( void )									\
													{ return CCLObject::AddRef(); }									\
												STDMETHOD(QueryInterface)	( REFIID iid, void **ppv )			\
													{ return CCLObject::QueryInterface(iid,ppv); }				\
												STDMETHOD_(ULONG,Release)	( void )									\
													{ return CCLObject::Release(); }									\
												STDMETHOD(InnerQueryInterface)( REFIID iid, void **ppv )		\
													{ return _InnerQueryInterface(_getIntfs(),iid,ppv ); }	\
												typedef a _intf_class;													\
												/** \brief Returns ptr to the supported interfaces table */	\
												const static CCLINTF *_getIntfs() {									\
												static const CCLINTF _intfs[] = {
#define	CCL_OBJECT_BEGIN(a)			static CCLObject *CreateObject ( void )							\
													{ return new (a); }													\
												CCL_OBJECT_BEGIN_INT(a)
												// Note, this v just gets offset of interface in class
#define	CCL_INTF(a)						{ &IID_##a, ((UINT_PTR)(static_cast<a*>((_intf_class *)8))-8), 1 },
#define	CCL_INTF_FROM(a,b)			{ &IID_##a, ((UINT_PTR)(static_cast<a*>(static_cast<b *>((_intf_class *)8)))-8), 1 },
#define	CCL_INTF_AGG(a,b)				{ &IID_##a, ((UINT_PTR)(&((static_cast<_intf_class *>((void *)8))->b)) - 8), 2 },
#define	CCL_OBJECT_END()				{ NULL, 0 } }; return _intfs; }

// Helper macros
#define	CCLTRY(a)						if (hr == S_OK) hr = a
#define	CCLTRYE(a,b)					if (hr == S_OK && !((a))) hr = (b)
#define	CCLOK(a)							if (hr == S_OK) { a }

#define	_ADDREF(a)						if ((a) != NULL) (a)->AddRef();
#define	_RELEASE(a)						if ((a) != NULL) { (a)->Release(); (a) = NULL; }
#define	_QI(a,b,c)						(a)->QueryInterface ( (b), (void **) (c) )
#define	_QISAFE(a,b,c)					((IUnknown *)(NULL) != (IUnknown *)(a)) ? \
												((IUnknown *)(a))->QueryInterface ( (b), (void **) (c) ) : E_UNEXPECTED;

// COM Creation
#define	COCREATE(a,b,c)				cclCreateObject ( a, NULL, (b), (void **) (c) )
#define	COCREATEA(a,b,c)				cclCreateObject ( a, (b), IID_IUnknown, (void **) (c) )

// Modules
#ifdef	_WIN32
#define	CCL_DLLMAIN()					EXTERN_C BOOL WINAPI DllMain ( HANDLE _hInst, DWORD dwReason, void * )	\
													{ return cclDllMain ( _hInst, dwReason ); }
#define	CCL_DLLMAIN_EX(a)				BOOL WINAPI	DllMain ( HANDLE _hInst, DWORD dwReason, void * )				\
													{ if (! (a) (_hInst, dwReason) || !cclDllMain ( _hInst, dwReason ))	\
														return FALSE; else return TRUE; }
#else
#define	CCL_DLLMAIN()
#define	CCL_DLLMAIN_EX(a)
#endif

//
//	Structure - CCLENTRY.  An entry for a list of objects in a module.
//
class CCLObject;											// Forward dec.
typedef struct tagCCLENTRY
	{
	const GUID		*clsid;								// Class ID
	CCLObject *		(*create)();						// Creation function
	const WCHAR 	*progid;								// Prog ID of class
	} CCLENTRY;

//
// Structure - CCLINTF.  An interface.
//

typedef struct tagCCLINTF
	{
	const	IID	*piid;									// Interface ID ptr.
	UINT_PTR		uip;										// Interface ptr.
	U32			itype;									// (uip) 0 = end, 1 = offset, 2 = pointer
	} CCLINTF;

//
// Interface - IInnerUnknown.  Ptr. to the inner unknown of an object.
//

class IInnerUnknown
	{
	public :
	STDMETHOD(InnerQueryInterface)	( REFIID, void ** )	PURE;
	STDMETHOD_(ULONG,InnerAddRef)		( void )					PURE;
	STDMETHOD_(ULONG,InnerRelease)	( void )					PURE;

	// GCC gets upset without a virtual destructor even though its a pure virtual interface
	#ifdef	__GNUC__
	virtual ~IInnerUnknown				( void ) {}
	#endif
	};

// Forward declarations
class		CCLFactory;
struct	IDictionary;

//
// Class - CCLObject.  Base class for a COM object.
//

class CCLObject :
	public IUnknown,										// Interface
	public IInnerUnknown									// Interface
	{
	public :
	CCLObject				( void );					// Constructor
	virtual ~CCLObject	( void );					// Destructor

	// 'CCLObject' members
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object
	virtual HRESULT	cclRegister	( bool );		// Object (un)registration

	// 'IUnknown' members
	STDMETHOD_(ULONG,AddRef)	( void )
			{ return unkOuter->AddRef(); }
	STDMETHOD(QueryInterface)	( REFIID iid, void **ppv )
			{ return unkOuter->QueryInterface ( iid, ppv ); }
	STDMETHOD_(ULONG,Release)	( void )
			{ return unkOuter->Release(); }

	// 'IInnerUnknown' members
	STDMETHOD_(ULONG,InnerAddRef)		( void );
	STDMETHOD(InnerQueryInterface)	( REFIID, void ** ) = 0;
	STDMETHOD_(ULONG,InnerRelease)	( void );

	// Operators
	void *operator new		( size_t );
	void operator	delete	( void * );

// Operator overload to use global memory manager
//#define			_NEW			new
//#define			_DELETE(a)	delete a
//void *operator new		( size_t );
//void operator	delete	( void * );
//void *operator new[]		( size_t );
//void operator	delete[]	( void * );

//wchar_t	*sysStringAlloc		( const wchar_t * );
//wchar_t	*sysStringAllocLen	( const wchar_t *, U32 );
//void		sysStringFree			( wchar_t * );


	protected :

	// Run-time data
	IUnknown		*unkOuter;								// Ptr. to outer unknown

	// Utilities
	HRESULT	_InnerQueryInterface	( const CCLINTF *, REFIID, void ** );

	public :
	#ifdef		_DEBUG
	LONG		objidx;										// Object number (debug)
	#endif

	private :
	LONG		refcnt;										// Reference count ob object
	friend	class CCLFactory;							// Friend class
	};

//
// Class - CCLFactory.  Class factory, creates objects.
//

class CCLFactory :
	public CCLObject,										// Base class
	public IClassFactory									// Interface
	{
	public :
	CCLFactory ( const CLSID * = NULL );			// Constructor
	CCLFactory ( const WCHAR * );						// Constructor

	// Run-time data
	U32		objidx;										// Index of object into CCLENTRY table

	// 'IClassFactory' members
	STDMETHOD(CreateInstance)	( IUnknown *, REFIID, void ** );
	STDMETHOD(LockServer)		( BOOL );

	private :

	// CCL
	CCL_OBJECT_BEGIN(CCLFactory)
		CCL_INTF(IClassFactory)
	CCL_OBJECT_END()
	};

//
// Class - CCLFactoriesCache.  Caches mutiple factories to speed up
//		object creation.
//

class CCLFactoriesCache
	{
	public :
	CCLFactoriesCache				( void );			// Constructor
	virtual ~CCLFactoriesCache	( void );			// Destructor

	// Run-time data
	IDictionary		*pDictFact;							// Factories
	LONG				refcnt;								// Reference count on cache
	IDictionary		*pDictProg;							// Prog IDs

	// Utilities
	HRESULT	AddRef	( void );						// Increment usage count
	HRESULT	Create	( REFCLSID, IUnknown *,		// Create specified object
								REFIID, void ** );
	HRESULT	Create	( const WCHAR *,				// Create specified object from Prog ID
								IUnknown *, REFIID,
								void ** );
	HRESULT	Release	( void );						// Decrement usage count

	};

//
// Class - CCLFactoryCache.  Caches the class factory for a single class ID
//		to speed up object creation.
//

class CCLFactoryCache
	{
	public :
	CCLFactoryCache				( REFCLSID );		// Constructor
	virtual ~CCLFactoryCache	( void );			// Destructor

	// Run-time data
	IClassFactory	*pFact;								// Factory
	CLSID				clsid;								// Class ID of target object
	LONG				refcnt;								// Reference count on cache

	// Utilities
	HRESULT	AddRef	( void );						// Increment usage count
	HRESULT	Create	( REFIID, IUnknown **,		// Create object
								IUnknown * = NULL );
	HRESULT	Release	( void );						// Decrement usage count
	};

//////////////
// Interfaces
//////////////

//
// Interface - IObjectFactory.  Interface to create objects.
//

DEFINE_GUID	(	IID_IObjectFactory, 0x2534d01a, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IObjectFactory,IUnknown)
	{
	public :
	STDMETHOD(allocObject)	( const WCHAR *, IUnknown ** )	PURE;
	};

// Prototypes
BOOL		cclDllMain	( HANDLE, U32 );
#if      __unix__ || __APPLE__
extern "C"
HRESULT CLSIDFromProgID	( const WCHAR *, CLSID * );
#endif
extern "C"
HRESULT cclCreateObject	( const WCHAR *, IUnknown *, REFIID, void ** );
extern "C"
HRESULT cclGetFactory 	( const WCHAR *, REFIID, void ** );
#endif

