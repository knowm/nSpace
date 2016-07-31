////////////////////////////////////////////////////////////////////////
//
//									NSPACE.H
//
//				Include file for nSpace side of Apache interface module
//
////////////////////////////////////////////////////////////////////////

#ifndef	NSPACE_H
#define	NSPACE_H

// nSpace client
#include "../../../lib/nshxl/nshxl.h"

//
// Class - nSpaceLink.  nSpace link object.
//
class nSpaceLink_t;
class nSpaceLink :
	public CCLObject,										// Base class
	public nSpaceClientCB								// Base class
	{
	public :
	nSpaceLink ( void );									// Constructor

	// Run-time data
	nSpaceLink_t	*pTick;								// Tickable object
	IThread			*pThrd;								// Thread object

	// 'nSpaceClientCB' members
	STDMETHOD(onReceive)	(const WCHAR *, const WCHAR *, const ADTVALUE &);

	// CCL
	CCL_OBJECT_BEGIN(nSpaceLink)
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
	nSpaceClient	*pnCli;								// Client interface
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
