////////////////////////////////////////////////////////////////////////
//
//									NSPACE.H
//
//				Include file for nSpace side of Apache interface module
//
////////////////////////////////////////////////////////////////////////

#ifndef	NSPACE_H
#define	NSPACE_H

// For some reason these includes are necessary to also allow nspace.h
// with  errors.
#include <winsock2.h>
#include <ws2tcpip.h>
#include <IPHlpApi.h>

// Apache
#include <httpd.h>
#include <http_config.h>
#include <http_protocol.h>
#include <ap_config.h>

// nSpace client
#include "../../../lib/nshxl/nshxl.h"

//
// Class - nSpaceLink.  nSpace link object.
//
class nSpaceLink_t;
class nSpaceLink :
	public CCLObject,										// Base class
	public IByteStream,									// For persistance
	public nSpaceClientCB								// Base class
	{
	public :
	nSpaceLink ( void );									// Constructor

	// Run-time data
	nSpaceLink_t	*pTick;								// Tickable object
	IThread			*pThrd;								// Thread object
	nSpaceClient	*pnCli;								// Client interface
	IStreamPersist	*pPer;								// Persistence
	sysCS				csVl;									// Thread safety
	request_rec		*req;									// Current request

	// Utilities
	STDMETHOD(load)	( const WCHAR *, request_rec * );
	STDMETHOD(store)	( const WCHAR *, IByteStream * );

	// 'nSpaceClientCB' members
	STDMETHOD(onReceive)	(const WCHAR *, const WCHAR *, const ADTVALUE &);

	// 'IByteStream' members
	STDMETHOD(available)	(U64 *);
	STDMETHOD(copyTo)		(IByteStream *, U64, U64 *);
	STDMETHOD(flush)		(void);
	STDMETHOD(read)		(void *, U64, U64 *);
	STDMETHOD(seek)		(S64, U32, U64 *);
	STDMETHOD(setSize)	(U64);
	STDMETHOD(write)		(void const *, U64, U64 *);

	// CCL
	CCL_OBJECT_BEGIN(nSpaceLink)
		CCL_INTF(IByteStream)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	};

//
// Class - nSpaceLink_t.  Worker thread object.
//

class nSpaceLink_t :
	public CCLObject,										// Base class
	public ITickable										// Interface
	{
	public :
	nSpaceLink_t ( void );								// Constructor

	// Run-time data
	nSpaceLink		*pThis;								// Parent object
	bool				bWork;								// Work flag

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN(nSpaceLink_t)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	};

// Prototypes

#endif
