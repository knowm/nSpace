////////////////////////////////////////////////////////////////////////
//
//										NSPCL_.H
//
//			Implementaiton include file for the nSpace library
//
////////////////////////////////////////////////////////////////////////

#ifndef	NSPCL__H
#define	NSPCL__H

#include "nspcl.h"
#include "../../lib/ccl/ccl.h"

// Forward dec.
class TemporalImpl;
class Emittert;
class Emitter;
class Receptor;
class Receptors;

//
// Objects
//
/*
//
// Class - Behaviour.  A wrapper object around a behaviour in order to
//			provide thread safety protection, etc.
//

class Behaviour :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Behaviour ( IBehaviour *);							// Constructor

	// Run-time data
	IBehaviour	*pBehave,*pBehaveR;					// Contained behaviour
	sysCS			csRx,csInt;								// Thread safety
	IList			*pRxQ;									// Receiver queue
	IIt			*pRxIt;									// Receiver iterator
	bool			bReceive,bReceiving;					// Receive flags

	// 'IBehaviour' members
	STDMETHOD(attach)		( IDictionary *, bool );

	// 'IReceptor' memebers
	STDMETHOD(receive)	( IReceptor *, const WCHAR *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN_INT(Behaviour)
		CCL_INTF(IBehaviour)
		CCL_INTF(IReceptor)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object
	};
*/
//
// Class - Connect.  A receptor object for managing "_Connect" location connections.
//

class Connect :
	public CCLObject,										// Base class
	public IReceptor										// Interface
	{
	public :
	Connect ( void );										// Constructor

	// Run-time data
	bool			bActive;									// Active state
	IReceptor	*pRcpLoc;								// Receptor for location
	IDictionary	*pDctLoc;								// Dictionary interface for location
	INamespace	*pSpc;									// Namespace object
	IDictionary	*pDctMk;									// 'Maked' connections
	IDictionary	*pDctSeq;								// Sequential ordering

	// 'IReceptor' members
	STDMETHOD(receive)	( IReceptor *, const WCHAR *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN_INT(Connect)
		CCL_INTF(IReceptor)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	// Internal utilities
	STDMETHOD(connect)		( IDictionary * );
	STDMETHOD(connect)		( const WCHAR *, const WCHAR *, bool );
	STDMETHOD(emitBrk)		( IDictionary * );
	STDMETHOD(emitBrk)		( const WCHAR * );
//	STDMETHOD(emitMk)			( IDictionary * );
//	STDMETHOD(disconnect)	( IDictionary * );
//	STDMETHOD(connectFrom)	( IDictionary *, bool );
	};

//
// Structure - CNNE.  Connector entry
//

typedef struct tagCNNE
	{
	tagCNNE		*next;									// Next connector in list
	IUnknown		*pConn;									// Connector
	bool			bRx;										// Connector receive 
	} CNNE;

//
// Class - ConnList.  Internal connector list.
//

class ConnList :
	public CCLObject										// Base class
	{
	public :
	ConnList ( void );									// Constructor

	// Utilities
	HRESULT	add		( IUnknown *, bool );		// Add connector to list
	CNNE *	head		( void ) { return h; }		// Obtain head of list
	CNNE *	next		( CNNE *e )						// Obtain next entry
							{ return (e != NULL) ? e->next : NULL; }
	HRESULT	remove	( IUnknown * );				// Remove entry

	// CCL
	CCL_OBJECT_BEGIN_INT(ConnList)
	CCL_OBJECT_END()

	private :

	// Run-time data
	CNNE	*h,*t;											// List info.
	sysCS	cs;												// Thread safety
	};

//
// Class - GlobalNspc.  Global nSpace object to contain static information.
//

class GlobalNspc
	{
	public :
	GlobalNspc ( void );									// Constructor
	virtual ~GlobalNspc ( void );						// Destructor

	// Run-time data
//	adtString		strName,strNames,strItms;		// String references
//	adtString		strBehave,strLocn,strType;		// String references
//	adtString		strMod,strDesc,strOnDesc;		// String references
//	adtString		strPar,strNspc,strConn,strPer;// String references
//	adtString		strRdy,strRev,strRcvr,strRef;	// String references
//	adtString		strRO,strLoc,strVal;				// String references

//	IClassFactory	*pcfDct;								// Class factories
//	IList				*pLstStVal,*pLstLd;				// nspcStoreValue, nspcLoadPath lists
	sysCS				cs;									// Thread safety for global objects
	ULONG				refcnt;								// Reference count

	// Re-usable stack objects for path creation
	sysCS				csAbs;
	IList				*pStkAbs;
	IIt				*pItAbs;

	// Utilities
	ULONG		AddRef	( void );						// Increase reference count
	ULONG		Release	( void );						// Decrease reference count
	};


//
// Class - Location.  Implementation of a namespace location.
//

class Location :
	public CCLObject,										// Base class
	public ILocation,										// Interface
	public IDictionary,									// Interface
	public IReceptor										// Interface
	{
	public :
	Location ( void );									// Constructor

	// Run-time data
	IDictionary		*pDctc;								// Contained dictionary
	IDictionary		*pDctOut;							// Outer dictionary (aggregation)
	ILocation		*pLocOut;							// Outer location (aggregation)
	IReceptor		*pRcpOut;							// Outer receptor (aggregation)
	ILocation		*pPar;								// Parent location
	IDictionary		*pDctPar;							// Parent location
	INamespace		*pSpc;								// Namespace object
	ConnList			*pConns;								// Connected receptors
	bool				bActive;								// Location active ?
	adtString		strName;								// This location name
	bool				bRx;									// Receiving ?
	sysCS				csInt;								// Thread safety
	IList				*pRxQ;								// Receiver queue
	IIt				*pRxIt;								// Receiver iterator
	IBehaviour		*pBehave;							// Attached behaviour
//	CLSID				clsidB;								// Class Id of behaviour
	bool				bRcp,bEmt;							// Location a connector ?
	bool				bBehaveV;							// nSpc value behavior ?
	adtValue			vLocThis;							// Latest 'value' for this location
	bool				bLocThis;							// First time storing value
//	adtString		strLocBfr;							// Internal location buffer
	IReceptor		*pRcpConn;							// Connect receptor

	// 'IDictionary' members
	STDMETHOD(keys)	( IIt ** );
	STDMETHOD(load)	( const ADTVALUE &, ADTVALUE & );
	STDMETHOD(store)	( const ADTVALUE &, const ADTVALUE & );

	// 'IContainer' members
	STDMETHOD(clear)		( void );
	STDMETHOD(copyTo)		( IContainer * );
	STDMETHOD(isEmpty)	( void );
	STDMETHOD(iterate)	( IIt ** );
	STDMETHOD(remove)		( const ADTVALUE & );
	STDMETHOD(size)		( U32 * );

	// 'ILocation' memebers
	STDMETHOD(connect)	( IReceptor *, bool, bool );
	STDMETHOD(connected)	( IReceptor *, bool, bool );
	STDMETHOD(create)		( const WCHAR *, ILocation ** );
	STDMETHOD(reflect)	( const WCHAR *, IReceptor * );
	STDMETHOD(stored)		( ILocation *, bool, IReceptor *, 
									const WCHAR *, const ADTVALUE & );

	// 'IReceptor' members
	STDMETHOD(receive)	( IReceptor *, const WCHAR *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(Location)
		CCL_INTF(IDictionary)
		CCL_INTF(IContainer)
		CCL_INTF(ILocation)
		CCL_INTF(IReceptor)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	private :

	// Internal utilities
	STDMETHOD(active)			( void );
	STDMETHOD(activeNotify)	( bool );
	STDMETHOD(desc)			( void );
	STDMETHOD(receive)		( IDictionary *, const WCHAR *, const ADTVALUE &, bool = false );
	STDMETHOD(receiveEmpty)	( IReceptor *, const WCHAR * );
	STDMETHOD(shutdown)		( void );

	};

//
// Class - Namespace.  Implementation of a namespace.
//

class Namespace :
	public CCLObject,										// Base class
	public INamespace										// Interface
	{
	public :
	Namespace ( void );									// Constructor

	// Run-time data
	IUnknown			*punkDct;							// Aggregated dictionary
	IDictionary		*pDctNs;								// Aggregated dictionary
	IDictionary		*pDctRt;								// Root namespace
//	IDictionary		*pDctLnk;							// Link layer dictionary
//	ITemporalImpl	*pTmp;								// Temporal nSpace
	ILocations		*pTmp;								// Temporal stream source
	IDictionary		*pDctRec;							// Recording dictionary
	IDictionary		*pDctStat;							// Status dictionary
	IDictionary		*pDctOpts;							// Options dictionary
	sysCS				csGet;								// Thread safety

	// 'INamespace' members
	STDMETHOD(connection)	( IDictionary *, const WCHAR *, const WCHAR *, IReceptor *, IReceptor ** );
	STDMETHOD(get)				( const WCHAR *, ADTVALUE &, const WCHAR * );
	STDMETHOD(link)			( const WCHAR *, const WCHAR *, bool );
	STDMETHOD(open)			( ILocations * );
	STDMETHOD(record)			( const WCHAR *, bool );

	// CCL
	CCL_OBJECT_BEGIN(Namespace)
		CCL_INTF(INamespace)
		CCL_INTF_AGG(IDictionary,punkDct)
		CCL_INTF_AGG(IContainer,punkDct)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	private :

	// Internal utilities
	HRESULT addLoc			( IDictionary *, const WCHAR *, IDictionary ** );
	HRESULT addValue		( IDictionary *, const WCHAR *, const ADTVALUE &,
									INamespaceValue ** );
	HRESULT load			( const WCHAR *, IDictionary ** );
	HRESULT pathBreak		( const WCHAR *, adtString &, adtString &, adtString & );
	HRESULT receive		( IDictionary *, const WCHAR *, const ADTVALUE & );
	HRESULT record			( const WCHAR *, bool, bool );
	HRESULT record			( const WCHAR *, IDictionary *, bool );
//	HRESULT recordNode	( const WCHAR *, bool );
	HRESULT relink			( const WCHAR *, IDictionary * );
	HRESULT resolve		( const WCHAR *, adtString & );
//	HRESULT tmpInit		( IDictionary *, bool );
	HRESULT tmpLoad		( const WCHAR *, ADTVALUE & );
	HRESULT tmpLocs		( const WCHAR *, IIt ** );
	HRESULT tokens			( const WCHAR *, const WCHAR *, IContainer ** );
	};

//
// Class - PersistTxt.  Implementation of the text-based nSpace persistence object.
//

class PersistTxt :
	public CCLObject,										// Base class
	public IStreamPersist									// Interface
	{
	public :
	PersistTxt ( void );									// Constructor

	// Run-time data
	IByteStream		*pStmOut;							// Output document stream
	IMemoryMapped	*pDocIn;								// Input document buffer
	char				*paDocIn;							// Input document buffer
	U64				uDocIn;								// Input document size
	U32				uLineBegin,uLineEnd,uLineEOL;	// Current line information
	U32				uDepth;								// Save 'depth'
	IDictionary		*pGraph;								// Current graph
	IDictionary		*pNodes;								// Nodes
	IDictionary		*pConns;								// Connections
	IDictionary		*pSubs;								// Subgraphs
	IList				*pNames;								// Name list

	// 'IStreamPersist' members
	STDMETHOD(load)	( IByteStream *, ADTVALUE & );
	STDMETHOD(save)	( IByteStream *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(PersistTxt)
		CCL_INTF(IStreamPersist)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	private :

	// Internal utilities
	HRESULT	nextChar		( U32 *, bool = true );
	HRESULT	nextString	( U32 *, U32 *, U32 = MAXDWORD );
	HRESULT	toValue		( U32, U32, ADTVALUE & );
	HRESULT	valueLoad	( ADTVALUE & );			// Load child
	HRESULT	valueSave	( const ADTVALUE & );	// Save child
	HRESULT	writeValue	( const ADTVALUE & );
	};

//
// Class - TemporalImpl.  Default nSpace temporal implementation class.
//

class TemporalImpl :
	public CCLObject,										// Base class
	public ITemporalImpl,								// Interface
	public ILocations										// Interface
	{
	public :
	TemporalImpl ( void );								// Constructor

	// Run-time data
	ILocations		*pStmSrc;							// Underlying stream source
	IByteStream		*pStmIdx,*pStmHdr,*pStmVal;	// Streams
	IDictionary		*pDctMap;							// Location map dictionary
	adtString		strLoc;								// Database location
	U64				uSeqNum;								// Current sequence number
	U64				uLocId;								// Location Id
	IStreamPersist	*pPrsrL,*pPrsrS;					// Value parsers
	sysCS				csIO;									// Critical sections
	IDictionary		*pDctOpts;							// Options dictionary

	// Utilities
	STDMETHOD(locGet)	( U64, HDRIDX * );
	STDMETHOD(locGet)	( NSSQNM, HDRLOC * );
	STDMETHOD(locGet)	( U64, ADTVALUE & );
	STDMETHOD(locId)	( const WCHAR *, bool, U64 * );
	STDMETHOD(locPut)	( U64, const HDRIDX * );
	STDMETHOD(locPut)	( U64, const HDRLOC * );
	STDMETHOD(locPut)	( U64, const ADTVALUE & );

	// 'ILocations' members
	STDMETHOD(open)		( IDictionary *,	IUnknown ** );
	STDMETHOD(locations)	( const WCHAR *,	IIt ** );
	STDMETHOD(resolve)	( const WCHAR *,	bool, ADTVALUE & );
	STDMETHOD(status)		( const WCHAR *,	IDictionary * );

	// 'ITemporalImpl' members
//	STDMETHOD(emitter)	( const WCHAR *, bool, IDictionary ** );
//	STDMETHOD(locations)	( const WCHAR *, IIt ** );
	STDMETHOD(open)		( IDictionary * );
//	STDMETHOD(status)		( const WCHAR *, IDictionary * );

	// CCL
	CCL_OBJECT_BEGIN(TemporalImpl)
		CCL_INTF(ILocations)
		CCL_INTF(ITemporalImpl)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	};


//
// Class - TemporalLoc.  A location that allows access/storage of temporal
//		values inside the temporal database.
//

class TemporalLoc :
	public CCLObject,										// Base class
	public IDictionary,									// Interface
	public ILocation,										// Interface
	public IReceptor									// Interface
	{
	public :
	TemporalLoc ( TemporalImpl *,
						const WCHAR *, bool );			// Constructor

	// Run-time data
	adtString		strLoc;								// Path to this dictionary location
	TemporalImpl	*pTmp;								// Temporal database
	bool				bRO;									// Read only
	IUnknown			*punkLoc;							// Aggregated object
	IDictionary		*pLocDct;							// Contained location
	IReceptor		*pLocRcp;							// Contained location
	ILocation		*pLocLoc;							// Contained location
	IDictionary		*pKeys;								// Valid key list

	// 'ILocation' memebers
	STDMETHOD(connect)	( IReceptor *, bool, bool );
	STDMETHOD(connected)	( IReceptor *, bool, bool );
	STDMETHOD(create)		( const WCHAR *, ILocation ** );
	STDMETHOD(reflect)	( const WCHAR *, IReceptor * );
	STDMETHOD(stored)		( ILocation *, bool, IReceptor *, 
									const WCHAR *, const ADTVALUE & );

	// 'IReceptor' members
	STDMETHOD(receive)	( IReceptor *, const WCHAR *, const ADTVALUE & );

	// 'IDictionary' members
	STDMETHOD(keys)		( IIt ** );
	STDMETHOD(load)		( const ADTVALUE &, ADTVALUE & );
	STDMETHOD(store)		( const ADTVALUE &, const ADTVALUE & );

	// 'IContainer' members
	STDMETHOD(clear)		( void );
	STDMETHOD(copyTo)		( IContainer * );
	STDMETHOD(isEmpty)	( void );
	STDMETHOD(iterate)	( IIt ** );
	STDMETHOD(remove)		( const ADTVALUE & );
	STDMETHOD(size)		( U32 * );

	// CCL
	CCL_OBJECT_BEGIN_INT(TemporalLoc)
		CCL_INTF(IDictionary)
		CCL_INTF(IContainer)
		CCL_INTF(ILocation)
		CCL_INTF(IReceptor)
//		CCL_INTF_AGG(IDictionary,punkDct)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :
	};

//
// Class - TemporalLocIt.  Iterator for the temporal dictionary.
//

class TemporalLocIt :
	public CCLObject,										// Base class
	public IIt												// Interface
	{
	public :
	TemporalLocIt ( TemporalLoc * ); 				// Constructor

	// Run-time data
	TemporalLoc		*pDct;								// Parent 
	IIt				*pItKeys;							// Valid keys

	// 'IIt' members
	STDMETHOD(begin)	( void );
	STDMETHOD(end)		( void );
	STDMETHOD(next)	( void );
	STDMETHOD(prev)	( void );
	STDMETHOD(read)	( ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN_INT(TemporalLocIt)
		CCL_INTF(IIt)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	};

//
// Nodes
//

//
// Class - Connectors.  A node for managing connectors.
//

class Connectors :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Connectors ( void );									// Constructor
	virtual ~Connectors ( void );						// Destructor

	// Run-time data
	adtValue		vV;										// Parameters
//	IEmitter		*pEmit;									// Current emitter
	IReceptor	*pRecep;									// Current receptor
//	IDictionary	*pConns;									// Dynamic connectors
	adtString	strType;									// Connector type
	adtString	strLoc;									// Location
	adtString	strName;									// Internal
	adtIUnknown	unkV;										// Internal
	adtValue		vGet,vVUse;								// Internal

	// CCL
	CCL_OBJECT_BEGIN(Connectors)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Node behaviour
	DECLARE_CON(Add)
	DECLARE_RCP(Clear)
	DECLARE_RCP(Connect)
	DECLARE_RCP(Disconnect)
	DECLARE_CON(Emit)
	DECLARE_RCP(Emitter)
	DECLARE_EMT(Error)
	DECLARE_CON(Get)
	DECLARE_RCP(Location)
	DECLARE_RCP(Receive)
	DECLARE_CON(Receptor)
	DECLARE_CON(Receptors)
	DECLARE_RCP(Type)
	DECLARE_CON(Value)	
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Add)
		DEFINE_RCP(Clear)
		DEFINE_RCP(Connect)
		DEFINE_RCP(Disconnect)
		DEFINE_CON(Emit)
		DEFINE_RCP(Emitter)
		DEFINE_EMT(Error)
		DEFINE_CON(Get)
		DEFINE_RCP(Location)
		DEFINE_RCP(Receive)
		DEFINE_CON(Receptor)
		DEFINE_CON(Receptors)
		DEFINE_RCP(Type)
		DEFINE_CON(Value)	
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - KeyPath.  A node to manage a dictionary that supports key paths.
//

class KeyPath :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	KeyPath ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct;									// Dictionary
	adtString	strKey;									// Key path
	adtValue		vValue,vL;								// Value
	adtIUnknown	unkV;										// Internal

	// CCL
	CCL_OBJECT_BEGIN(KeyPath)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_RCP(Dictionary)
	DECLARE_RCP(Key)
	DECLARE_CON(Load)
	DECLARE_EMT(NotFound)
	DECLARE_RCP(Value)
	DECLARE_CON(Visit)
	DECLARE_EMT(VisitEnd)
	DECLARE_CON(Store)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Key)
		DEFINE_CON(Load)
		DEFINE_EMT(NotFound)
		DEFINE_RCP(Value)
		DEFINE_CON(Visit)
		DEFINE_EMT(VisitEnd)
		DEFINE_CON(Store)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT visit ( IDictionary *, const WCHAR *, const WCHAR * );
	};

//
// Class - Link.  A node to link/unlink subgraphs.
//

class Link :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Link ( void );											// Constructor

	// Source emitters
	adtString		strSrc,strDst;						// Source/destination
	IDictionary		*pLocPar;							// Parent location 
	IDictionary		*pRootSrc,*pRootDst;				// Roots

	// CCL
	CCL_OBJECT_BEGIN(Link)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Node behaviour
	DECLARE_CON	(Link)
	DECLARE_CON	(Unlink)
	DECLARE_RCP	(Source)
	DECLARE_RCP	(Destination)
	DECLARE_EMT	(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON	(Link)
		DEFINE_CON	(Unlink)

		DEFINE_RCP	(Source)
		DEFINE_RCP	(Destination)

		DEFINE_EMT	(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - ReflectRx.  Location reflection receiver object.
//
class Reflect;
class ReflectRx :
	public CCLObject,										// Base class
	public IReceptor										// Interface
	{
	public :
	ReflectRx ( Reflect *, const WCHAR *,			// Constructor
						ILocation * );						

	// Run-time data
	Reflect		*pParent;								// Parent object
	adtString	strRoot;									// Root location for receiver
	ILocation	*pLoc;									// Connected location

	// 'IReceptor' members
	STDMETHOD(receive)	( IReceptor *, const WCHAR *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN_INT(ReflectRx)
		CCL_INTF(IReceptor)
	CCL_OBJECT_END()
	virtual void		destruct(void);				// Destruct object

	};

//
// Class - Reflect.  A location reflection node.
//

class Reflect :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Reflect ( void );										// Constructor

	// Run-time datas
	adtString	strRoot,strLoc;						// Active locations
	adtValue		vValue;									// Active value
	IDictionary	*pRef;									// Reflections dictionary
	sysCS			csRx;										// Receive mutex

	// CCL
	CCL_OBJECT_BEGIN(Reflect)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Add)
	DECLARE_CON(Remove)
	DECLARE_CON(Root)
	DECLARE_CON(Fire)
	DECLARE_CON(Location)
	DECLARE_RCP(Store)
	DECLARE_CON(Value)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Add)
		DEFINE_CON(Remove)
		DEFINE_CON(Root)
		DEFINE_CON(Fire)
		DEFINE_CON(Location)
		DEFINE_RCP(Store)
		DEFINE_CON(Value)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Temporal.  A node to access the temporal dimension of nSpace.
//

class Temporal :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Temporal ( void );									// Constructor

	// Source emitters/receptors
	IDictionary		*pRoot;								// Root 
	IDictionary		*pLocPar;							// Parent location 
	adtString		strLoc;								// Location
	adtBool			bRead;								// Read only flag

	// CCL
	CCL_OBJECT_BEGIN(Temporal)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Node behaviour
	DECLARE_CON(Emitter)
	DECLARE_CON(Record)
	DECLARE_RCP(Test)
	DECLARE_RCP(Location)
	DECLARE_RCP(Root)
	DECLARE_EMT(NotFound)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Emitter)
		DEFINE_CON(Record)
		DEFINE_RCP(Test)
		DEFINE_RCP(Location)
		DEFINE_RCP(Root)
		DEFINE_EMT(NotFound)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - This.  A node to reference the namespace.
//

class This :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	This ( void );											// Constructor

	// Source emitters/receptors
	IDictionary		*pDctF,*pDctT;						// Roots
	IDictionary		*pLocPar;							// Parent location 
	adtString		strLoc,strDef;						// Location/definition
	adtValue			vValue;								// Cached value

	// CCL
	CCL_OBJECT_BEGIN(This)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Node behaviour
	DECLARE_CON(Load)
	DECLARE_CON(Resolve)
	DECLARE_CON(Store)
	DECLARE_CON(Test)
	DECLARE_CON(Remove)
	DECLARE_RCP(Definition)
	DECLARE_RCP(Fire)
	DECLARE_RCP(Location)
	DECLARE_RCP(From)
	DECLARE_RCP(To)
	DECLARE_EMT(Graph)
	DECLARE_EMT(NotFound)
	DECLARE_EMT(Source)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Load)
		DEFINE_CON(Resolve)
		DEFINE_CON(Store)
		DEFINE_CON(Test)
		DEFINE_CON(Remove)

		DEFINE_RCP(Definition)
		DEFINE_RCP(Fire)
		DEFINE_RCP(Location)
		DEFINE_RCP(From)
		DEFINE_RCP(To)
		DEFINE_RCP(Value)

		DEFINE_EMT(Graph)
		DEFINE_EMT(NotFound)
		DEFINE_EMT(Source)

	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Value.  A single value.
//

class Value :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Value ( void );										// Constructor

	// Run-time datas
	adtValue		vE,vD,vL;								// Internal
	adtString	strT,strV;								// Internal
	adtString	strType;									// Value type
	IDictionary	*pDsc;									// Active descriptor
	bool			bDiff;									// Difference value ?

	// Utilities
	HRESULT validate ( IDictionary *, const ADTVALUE &, ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(Value)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Descriptor)
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Descriptor)
		DEFINE_CON(Fire)
	END_BEHAVIOUR_NOTIFY()

	private :

	};


//
// Class - Values.  Implementation of support for one or more nSpace values.
//

class Values :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Values ( void );										// Constructor

	// Run-time data
	IContainer		*pVals;								// Default values
	IDictionary		*pDctVals;							// Internal value dictionary
	IDictionary		*pDctNode;							// Node dictionary interface

	// CCL
	CCL_OBJECT_BEGIN(Values)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(_Add)
	DECLARE_CON(_Remove)
	DECLARE_CON(_Sync)
	BEGIN_BEHAVIOUR()
	END_BEHAVIOUR_NOTIFY()

	private :

	};

#endif
