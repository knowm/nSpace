////////////////////////////////////////////////////////////////////////
//
//										NSPCL.H
//
//								nSpace library
//
////////////////////////////////////////////////////////////////////////

#ifndef	NSPCL_H
#define	NSPCL_H

// System includes
#include "../adtl/adtl.h"
#include "../iol/iol.h"

///////////////
// Definitions
///////////////

// Predefined location for references in temporal nSpace
#define	LOC_NSPC_REF			L"ref/"

// Internal names
#define	STR_NSPC_NAME			L"_Name"
#define	STR_NSPC_NAMES			L"_Names"
#define	STR_NSPC_ITEMS			L"_Items"
#define	STR_NSPC_BEHAVE		L"_Behaviour"
#define	STR_NSPC_LOC			L"_Location"
#define	STR_NSPC_TYPE			L"_Type"
#define	STR_NSPC_MOD			L"_Modified"
#define	STR_NSPC_DESC			L"_Descriptor"
#define	STR_NSPC_ONDESC		L"_OnDescriptor"
#define	STR_NSPC_PARENT		L"_Parent"
#define	STR_NSPC_NSPC			L"_Namespace"
#define	STR_NSPC_CONN			L"_Connect"
#define	STR_NSPC_CONNTR		L"_Connector"
#define	STR_NSPC_PERS			L"_Persist"
#define	STR_NSPC_ACTIVE		L"_Active"
#define	STR_NSPC_REV			L"_Revision"
#define	STR_NSPC_RCVR			L"_Receiver"
#define	STR_NSPC_REF			L"_Reference"

// Pre-generated string reference objects
extern	adtStringSt	strnRefName;
extern	adtStringSt	strnRefNames;
extern	adtStringSt	strnRefItms;
extern	adtStringSt	strnRefBehave;
extern	adtStringSt	strnRefLocn;
extern	adtStringSt	strnRefType;
extern	adtStringSt	strnRefMod;
extern	adtStringSt	strnRefDesc;
extern	adtStringSt	strnRefOnDesc;
extern	adtStringSt	strnRefPar;
extern	adtStringSt	strnRefNspc;
extern	adtStringSt	strnRefConn;
extern	adtStringSt	strnRefConntr;
extern	adtStringSt	strnRefPers;
extern	adtStringSt	strnRefAct;
extern	adtStringSt	strnRefRcvr;
extern	adtStringSt	strnRefRef;

extern	adtStringSt	strnRefFrom;
extern	adtStringSt	strnRefTo;
extern	adtStringSt	strnRefRO;
extern	adtStringSt	strnRefLoc;
extern	adtStringSt	strnRefKey;
extern	adtStringSt strnRefVal;

//
// BEGIN_BEHAVIOUR	- Begin a node behaviour
//	DECLARE_RECEPTOR	- Declare a receptor for the node
// DECLARE_EMITTER	- Declare an emitter for the node
//	END_BEHAVIOUR		- End behaviour
//
// (Do not keep references, node has reference on behaviour)
//

#define	DECLARE_LOC_NAME(a,b)	a##b
#define	DECLARE_RCP(a)														\
			IReceptor	*DECLARE_LOC_NAME(pr,a);
#define	DECLARE_EMT(a)														\
			IReceptor	*DECLARE_LOC_NAME(peOn,a);
#define	DECLARE_CON(a)														\
			DECLARE_RCP(a)														\
			DECLARE_EMT(a)

#define	BEGIN_BEHAVIOUR()													\
	INamespace		*pnSpc;													\
	IDictionary		*pnLoc;													\
	IDictionary		*pnDesc;													\
 	adtString		strnName;												\
	STDMETHOD(receive)	( IReceptor *, const WCHAR *,				\
									const ADTVALUE & );						\
	STDMETHOD(attach)		( IDictionary *_pnLoc, bool bAttach )	\
		{																			\
		HRESULT	hr	= S_OK;													\
		pnLoc	= _pnLoc;														\
		if (bAttach)															\
			{																		\
			adtValue		v;														\
			adtIUnknown	unkV;													\
																					\
			CCLTRY(pnLoc->load(strnRefNspc,v));							\
			CCLTRY(_QISAFE(v.punk,IID_INamespace,&pnSpc));			\
																					\
			CCLTRY(pnLoc->load(strnRefDesc,v));							\
			CCLTRY(_QISAFE((unkV=v),IID_IDictionary,&pnDesc));		\
																					\
			CCLTRY(pnLoc->load(strnRefName,v));							\
			CCLOK(strnName = v;)												\
																					\
			if (pnSpc != NULL)	pnSpc->Release();						\
			if (pnDesc != NULL)	pnDesc->Release();					\
			}

#if		defined(__GNUC__) || _MSC_VER >= 1900

#define	DEFINE_EMT(a)														\
			CCLTRY(pnSpc->connection ( pnLoc, L"On" #a,				\
							L"Emitter", this,									\
							(bAttach) ? &DECLARE_LOC_NAME(pe,On ## a) : NULL ));
#define	DEFINE_RCP(a)														\
			CCLTRY(pnSpc->connection ( pnLoc, L## #a,					\
							L"Receptor", this,								\
							(bAttach) ? &DECLARE_LOC_NAME(pr,a) : NULL ));
#define	DEFINE_CON(a)														\
			DEFINE_RCP(a)														\
			DEFINE_EMT(a)		
#else

#define	DEFINE_EMT(a)														\
			if (bAttach && hr == S_OK)										\
				hr = pnSpc->connection ( pnLoc, L"On" ## L#a,		\
								L"Emitter", this,								\
								&DECLARE_LOC_NAME(pe,On ## a) );

#define	DEFINE_RCP(a)														\
			if (bAttach && hr == S_OK)										\
				hr = pnSpc->connection ( pnLoc, L#a,					\
								L"Receptor", this,							\
								&DECLARE_LOC_NAME(pr,a) );
#define	DEFINE_CON(a)														\
			DEFINE_RCP(a)														\
			DEFINE_EMT(a)		

#endif

#define	END_BEHAVIOUR()													\
		if (!bAttach)															\
			pnSpc->connection ( pnLoc, L"", L"", this, NULL );		\
		return hr; }
 
#define	END_BEHAVIOUR_NOTIFY()											\
		CCLTRY(onAttach(bAttach));											\
		END_BEHAVIOUR()														\
	STDMETHOD(onAttach)( bool );

#define	_RCP(a)		(DECLARE_LOC_NAME(pr,a) == pr)
#define	_EMT(a,b)	(DECLARE_LOC_NAME(pe,On##a)->receive(this,L"Value",(b)))

// Prototypes
extern "C"
bool rcp ( const WCHAR *pwSrc, const WCHAR *pwDst );

////////////
// Typedefs
////////////

// Sequence numbers
typedef U64 NSSQNM;

//////////////
// Structures
//////////////

//
// Structure - HDRIDX.  Format of location index.
//

typedef struct
	{
	NSSQNM	last;											// Last stored value sequence number
	NSSQNM	first;										// First stored value sequence number
	} HDRIDX;

//
// Structure - HDRLOC.  Format of the value entry header.
//

typedef struct
	{
	NSSQNM	next;											// Next sequence number 
	NSSQNM	prev;											// Previous sequence number
	U64		lid;											// Location Id for entry
	U64		value;										// Position of location value
	} HDRLOC;

//
// Structure - HDRVAL.  Format of the location value header.
//

typedef struct
	{
	U64		lid;											// Location Id of value
	NSSQNM	seqnum;										// Sequence number of value
	} HDRVAL;

/*
//
// Structure - SEQRNG.  Structure to define a range of sequence numbers.
//		Range is lower <= sequence < upper.
//

typedef struct
	{
	NSSQNM	lower;										// Lower location (inclusive)
	NSSQNM	upper;										// Upper location (exclusive)
	} SEQRNG;

#define	MIN_SEQIDX	0
#define	MAX_SEQIDX	(1ll<<63)
*/
//////////////
// Interfaces
//////////////

// Forward dec.
//struct IEmitter;
struct INamespace;
struct INamespaceCB;
struct INamespaceLoc;
struct INamespaceLocCB;
struct INamespaceValue;
//struct INode;
struct IReceiver;
struct IReceptor;
struct ITemporalImpl;

//
// Interface - IReceptor.  Interface to a receptor for a location.
//

DEFINE_GUID	(	IID_IReceptor, 0x2534d080, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IReceptor,IUnknown)
	{
	STDMETHOD(receive)	( IReceptor *, const WCHAR *, const ADTVALUE & ) PURE;
	};

//
// Interface - IBehaviour.  Base interface to an object that
//		has defined node behaviour.
//

DEFINE_GUID	(	IID_IBehaviour, 0x2534d015, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IBehaviour,IReceptor)
	{
	STDMETHOD(attach)			( IDictionary *, bool )					PURE;
	};

//
// Interface - ILocation.  Interface to a namespace location.
//

DEFINE_GUID	(	IID_ILocation, 0x2534d0a2, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(ILocation,IUnknown)
	{
	STDMETHOD(connect)	( IReceptor *, bool, bool )				PURE;
	STDMETHOD(connected)	( IReceptor *, bool, bool )				PURE;
	STDMETHOD(create)		( const WCHAR *, ILocation ** )			PURE;
	STDMETHOD(reflect)	( const WCHAR *, IReceptor * )			PURE;
	STDMETHOD(stored)		( ILocation *, bool, IReceptor *,
									const WCHAR *, const ADTVALUE & )	PURE;
	};

//
// Interface - INamespace.  Interface to a namespace.
//

DEFINE_GUID	(	IID_INamespace, 0x2534d023, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(INamespace,IUnknown)
	{
	STDMETHOD(connection)	( IDictionary *, const WCHAR *, const WCHAR *, 
										IReceptor *, IReceptor ** )						PURE;
	STDMETHOD(get)				( const WCHAR *, ADTVALUE &, const WCHAR * )		PURE;
	STDMETHOD(link)			( const WCHAR *, const WCHAR *, bool )				PURE;
	STDMETHOD(open)			( ILocations * )											PURE;
	STDMETHOD(record)			( const WCHAR *, bool )									PURE;
	};

//
// Interface - ITemporalImpl.  Interface to temporal nSpace implementation.
//

DEFINE_GUID	(	IID_ITemporalImpl, 0x2534d002, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(ITemporalImpl,IUnknown)
	{
//	STDMETHOD(emitter)	( const WCHAR *, bool, IDictionary ** )	PURE;
//	STDMETHOD(locations)	( const WCHAR *, IIt ** )						PURE;
	STDMETHOD(open)		( IDictionary * )									PURE;
//	STDMETHOD(status)		( const WCHAR *, IDictionary * )				PURE;
	};

// Prototypes
HRESULT nspcLoadPath		( IDictionary *, const WCHAR *, ADTVALUE & );
HRESULT nspcStoreValue	( IDictionary *, const WCHAR *, const ADTVALUE &, bool = true );
HRESULT nspcPathTo		( IDictionary *, const WCHAR *, adtString &, IDictionary * = NULL );
HRESULT nspcTokens		( const WCHAR *, const WCHAR *, IList * );

///////////
// Classes
///////////

//
// Objects
//

DEFINE_GUID	(	CLSID_Location, 0x2534d070, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Namespace, 0x2534d024, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_PersistTxt, 0x2534d01e, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_TemporalImpl, 0x2534d006, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

//
// Nodes
// 

DEFINE_GUID	(	CLSID_Connect, 0x2534d0b2, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Connectors, 0x2534d02a, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_KeyPath, 0x2534d001, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Link, 0x2534d078, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Reflect, 0x2534d0ae, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Temporal, 0x2534d026, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_This, 0x2534d038, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Value, 0x2534d062, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Values, 0x2534d072, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

#endif
