////////////////////////////////////////////////////////////////////////
//
//										NSHL.H
//
//								nSpace shell library
//
////////////////////////////////////////////////////////////////////////

#ifndef	NSHL_H
#define	NSHL_H

// System includes
#include "../adtl/adtl.h"

//////////////
// Interfaces
//////////////

// Declared here for convenience, must match IDL file.

//
// Interface - IListenX.  Callback for received values on listened emitters.
//

DEFINE_GUID	(	IID_IListenX, 0x2534d084, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IListenX,IUnknown)
	{
	public :
	STDMETHOD(receive)	(	BSTR szPath, BSTR szLoc, 
									VARIANT *pVar )					PURE;

	};

//
// Interface - INamespaceX.  Utilities for using the attached namespace
//

DEFINE_GUID	(	IID_INamespaceX, 0x2534d085, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(INamespaceX,IUnknown)
	{
	public :
	STDMETHOD(load)		( BSTR szPath, VARIANT *pVar )	PURE;
	STDMETHOD(listen)		( BSTR szPath, IListenX *ppL )	PURE;
	STDMETHOD(store)		( BSTR szPath, VARIANT *pVar )	PURE;
	STDMETHOD(unlisten)	( BSTR szPath, IListenX *pL )		PURE;
	};

//
// Interface - IShellX.  Utilities for using the attached namespace
//

DEFINE_GUID	(	IID_IShellX, 0x2534d086, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IShellX,IUnknown)
	{
	public :
	STDMETHOD(open)		( BSTR szCmdLine, BOOL bShare, 
									INamespaceX **ppSpc )			PURE;
	};

//
// GUID
//

// Defined here again to avoid having to include output from MIDL compiler.

DEFINE_GUID	(	CLSID_ShellX, 0x2534d087, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

#endif
