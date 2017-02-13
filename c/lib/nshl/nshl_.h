////////////////////////////////////////////////////////////////////////
//
//										NSHL_.H
//
//			Implementaiton include file for the nSpace shell library
//
////////////////////////////////////////////////////////////////////////

#ifndef	NSHL__H
#define	NSHL__H

#include "nshl.h"
#include "../../lib/nspcl/nspcl.h"
#include "../../lib/iol/iol.h"

// nSpace client
#ifdef 	_WIN32
#include "../nshxl/nshxl.h"
#endif

// Forward dec.
class Shell;
class ShellX;
class ReceptorX;

#ifdef	_WIN32
//
// Class - NamespaceX.  External namespace control object.
//

class NamespaceX :
	public CCLObject,										// Base class
	public INamespaceX									// Interface
	{
	public :
	NamespaceX ( ShellX *, IDictionary * );		// Constructor

	// Run-time data
	ShellX			*pShellX;							// Owner object
	Shell				*pShell;								// Shell object for namespace
	IThread			*pThrd;								// Shell thread
	IDictionary		*pDctCmd;							// Command line dictionary
	IDictionary		*pListens;							// Listens
	IList				*pAutoUn;							// Auto unlisten list
	adtVariant		varL,varS;//,varX;				// Variant helpers
	sysCS				csListen;							// Listen mutex

	// Utilities
	STDMETHOD (received)	( const WCHAR *,			// Value received
									const WCHAR *, VARIANT * );

	// 'INamespaceX' members
	STDMETHOD (load)		( BSTR, VARIANT * );
	STDMETHOD (listen)	( BSTR, IListenX * );
	STDMETHOD (store)		( BSTR, VARIANT * );
	STDMETHOD (unlisten)	( BSTR, IListenX * );

	// CCL
	CCL_OBJECT_BEGIN_INT(NamespaceX)
		CCL_INTF(INamespaceX)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object
	};

//
// Class - NamespaceX_t.  External namespace control thread object.
//

class NamespaceX_t :
	public CCLObject,										// Base class
	public ITickable										// Interface
	{
	public :
	NamespaceX_t ( NamespaceX * );					// Constructor

	// Run-time data
	NamespaceX		*pSpc;								// Parent object

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN_INT(NamespaceX_t)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	};

//
// Class - ReceptorX.  Receptor helper class for external control.
//

class ReceptorX :
	public CCLObject,										// Base class
	public IReceptor										// Interface
	{
	public :
	ReceptorX ( NamespaceX *, const WCHAR *,		// Constructor
					ILocation * );							
	virtual ~ReceptorX ( void );						// Destructor

	// Run-time data
	NamespaceX	*pSpc;									// Owner object
	adtString	strRoot;									// Root location for receiver
	ILocation	*pLoc;									// Listen location
	adtVariant	varX;										// Variant helper
	sysCS			csRx;										// Thread protection

	// 'IReceptor' members
	STDMETHOD(receive)	( IReceptor *, const WCHAR *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN_INT(ReceptorX)
		CCL_INTF(IReceptor)
	CCL_OBJECT_END()

	};
#endif

//
// Class - Shell.  Base nSpace shell.
//

class Shell :
	public ITickable,										// Interface
	public CCLObject										// Base class
	{
	public :
	Shell ( IDictionary * );							// Constructor
	virtual ~Shell ( void );							// Destructor

	// Run-time data
	ILocations		*pStmSrc;							// Default stream source
	ILocations		*pTmp;								// Temporal database
	INamespace		*pSpc;								// Namespace object
	IDictionary		*pDctCmd;							// Command line dictionary
	adtString		strRoot,strDB;						// Default 'root' for storage and database
	sysEvent			evRun;								// Run event

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// Shell members
	STDMETHOD(batch)			( INamespace *, ILocations *, ILocations *, IContainer * );
	STDMETHOD(definitions)	( const WCHAR * );
	STDMETHOD(load)			( ILocations *, const WCHAR *, ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN_INT(Shell)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object

	protected :

	// Internal utilities
	STDMETHOD(dbInit)		( void );
	STDMETHOD(defNspc)	( const WCHAR *, IDictionary * );
	STDMETHOD(stgInit)	( void );
	STDMETHOD(dbg)			( IDictionary *, IDictionary *, IList *, IIt * );
	};

#ifdef	_WIN32
//
// Class - ShellX.  Shell object for external nSpace control.
//

class ShellX :
	public CCLObject,										// Base class
	public IShellX											// Interface
	{
	public :
	ShellX ( void );										// Constructor

	// Run-time data
	IDictionary	*pSpcs;									// Namespaces

	// Utilities
	void down ( NamespaceX * );						// Namespace is down

	// 'IShellX' members
	STDMETHOD (open)	( BSTR, BOOL, INamespaceX ** );

	// CCL
	CCL_OBJECT_BEGIN_INT(ShellX)
		CCL_INTF(IShellX)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	};

//
// Class - ShellFactX.  Singleton nSpace external shell factory.
//

class ShellFactX :
	public CCLObject,										// Base class
	public IClassFactory									// Interface
	{
	public :
	ShellFactX ( void );									// Constructor

	// Run-time data
	ShellX		*pX;										// External shell object

	// Utilities
	static HRESULT reg ( BOOL );

	// 'IClassFactory' members
	STDMETHOD(LockServer)		( BOOL );
	STDMETHOD(CreateInstance)	( IUnknown *, REFIID, void ** );

	// CCL
	CCL_OBJECT_BEGIN_INT(ShellFactX)
		CCL_INTF(IClassFactory)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object
	};
#endif

// Prototypes
HRESULT			strToVal	( const adtString &, ADTVALUE & );
HRESULT			valToStr	( const ADTVALUE &, adtString & );
HRESULT WINAPI defMain	( IDictionary * );
HRESULT WINAPI xMain		( IDictionary * );
HRESULT WINAPI svcMain	( IDictionary * );

#endif
