////////////////////////////////////////////////////////////////////////
//
//										NSHXL.H
//
//			Include file for the external nSpace client library.
//
////////////////////////////////////////////////////////////////////////

#ifndef	NSHXL_H
#define	NSHXL_H

// nSpace
#include "../../lib/nspcl/nspcl.h"

// Generated .H from IDL file
#ifdef 	_WIN32
#include "../nshl/nshl_h.h"
#endif

// Size of byte cache for persisted values
#define	SIZE_PERSIST_CACHE		8192
/*
//
// Class - nSpacePerX.  Internal value persistence buffer.
//

class nSpacePerX :
	public CCLObject,										// Base class
	public IPersistX,										// Interface
	public IByteStream									// Interface
	{
	public :
	nSpacePerX ( void );									// Constructor

	// Run-time data
	IPersistX		*pPerX;								// Value persistence buffer of caller
	IStreamPersist	*pPerL,*pPerS;						// Parsers
	IByteStream		*pStmL,*pStmS;						// Streams
	adtVariant		varL,varS;							// Variants

	// Utilities
	STDMETHOD(setOutput)		( IUnknown * );
	STDMETHOD(load)			( ADTVALUE & );
	STDMETHOD(save)			( const ADTVALUE & );

	// 'IByteStream' members
	STDMETHOD(available)	( U64 * );
	STDMETHOD(copyTo)		( IByteStream *, U64, U64 * );
	STDMETHOD(flush)		( void );
	STDMETHOD(read)		( void *, U64, U64 * );
	STDMETHOD(seek)		( S64, U32, U64 * );
	STDMETHOD(setSize)	( U64 );
	STDMETHOD(write)		( void const *, U64, U64 * );

	// 'IPersistX' members
	STDMETHOD(reset)		( void );
	STDMETHOD(write)		( VARIANT * );

	// CCL
	CCL_OBJECT_BEGIN_INT(nSpacePerX)
		CCL_INTF(IPersistX)
		CCL_INTF(IByteStream)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object
	};
*/
//
// Class - nSpaceClient.  An nSpace client object to communication
//		with the ActiveX nSpace object.
//

class nSpaceClientCB;
class nSpaceClient
	{
	public :
	nSpaceClient ( void );								// Constructor
	virtual ~nSpaceClient ( void );					// Destructor

	// Run-time data
	DWORD							dwSvc;					// Service Id in global interface table
	IGlobalInterfaceTable	*pGIT;					// Global interface table
	IDictionary					*pLstns;					// Listen dictionary
	nSpaceClientCB				*pCB;						// Default callback
	const WCHAR					*pwRoot;					// Default root
	adtVariant					varS,varL;				// Variant helper
	sysCS							csMtx;					// Thread mutex

	// Utilities
	STDMETHOD(close)	( void );						// Close connection
	STDMETHOD(load)	( const WCHAR *,				// Get value from namespace
								ADTVALUE & );
	STDMETHOD(listen)	( const WCHAR *, BOOL,		// Listen to path
								nSpaceClientCB * = NULL );
	STDMETHOD(open)	( const WCHAR *, BOOL,		// Open connection to namespace
								nSpaceClientCB * = NULL );
	STDMETHOD(root)	( const WCHAR * );			// Set default root location
	STDMETHOD(store)	( const WCHAR *,				// Put value into namespace
								const ADTVALUE & );

	// Internal utilities
	STDMETHOD(pathCreate)	( const WCHAR *,		// Allocate/create full path
										BSTR * );					

	private :

	// Internal utilities
	STDMETHOD(getSvc)	( INamespaceX ** );			// Obtain service ptr.
	};

//
// Class - nSpaceClientL.  Internal listening object.
//

class nSpaceClientL :
	public CCLObject,										// Base class
	public IListenX										// Interface
	{
	public :
	nSpaceClientL ( nSpaceClient *,					// Constructor
							nSpaceClientCB * );	

	// Run-time data
	nSpaceClient	*pThis;								// Parent object
	nSpaceClientCB	*pCB;									// Callback object
	adtVariant		varL;									// Variant helper
	sysCS				csRx;									// Thread mutex

	// 'IListenX' members
	STDMETHOD(receive)			( BSTR, BSTR, VARIANT * );

	// CCL
	CCL_OBJECT_BEGIN_INT(nSpaceClientL)
		CCL_INTF(IListenX)
	CCL_OBJECT_END()
	};

//
// Class - nSpaceClientCB.  Callback object for received values.
//

class nSpaceClientCB
	{
	public :
	// A listened path has been received
	STDMETHOD(onReceive)	( const WCHAR *, const WCHAR *, const ADTVALUE & )	PURE;
	};

#endif
