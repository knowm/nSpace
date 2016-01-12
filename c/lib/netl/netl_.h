////////////////////////////////////////////////////////////////////////
//
//										NETL_.H
//
//				Implementation include file for the networking library
//
////////////////////////////////////////////////////////////////////////

#ifndef	NETL__H
#define	NETL__H

// In order the for includes to not give errors
#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif

// Includes
#include	"netl.h"

// Winsock
#if	defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <IPHlpApi.h>
#elif	defined(__unix__) || defined(__APPLE__)

// Includes
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include	<unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <net/route.h>
//#include <linux/sockios.h>
#include <ifaddrs.h>

// Definitions
#define	INVALID_SOCKET				-1
#define	SOCKET_ERROR				-1
#define	closesocket					close
#define	WSAGetLastError()			errno
#define	WSAECONNRESET				10054L
typedef	int							SOCKET;

#endif

#define	SIZE_FRAME_ETHERNET		1514				// Ethernet frame size
#define	SIZE_PERSIST_CACHE		8192

//
// Class - Address.  Network address node.
//

class Address :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Address ( void );										// Constructor

	// Run-time data
	adtInt			iAddr;								// Address
	adtInt			iPort;								// Port
	
	// CCL
	CCL_OBJECT_BEGIN(Address)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Resolve)
	DECLARE_CON(Address)
	DECLARE_CON(Port)
	DECLARE_CON(String)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Resolve)
		DEFINE_CON(Address)
		DEFINE_CON(Port)
		DEFINE_CON(String)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Avail.  Avails for readability on sockets.
//

class Avail :
	public CCLObject,										// Base class
	public IBehaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	Avail ( void );										// Constructor

	// Run-time data
	IThread		*pThrd;									// Worker thread
	IDictionary	*pAvails;								// Avail dictionary
	IIt			*pInSkt;									// Socket iterator
	IDictionary	*pSkt;									// Active socket
	bool			bAvail;									// Running ?
	adtBool		bRead;									// Readability ?
	adtBool		bWrite;									// Writeability ?
	adtInt		iTo;										// Configured timeout

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN(Avail)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Add)
	DECLARE_RCP(Remove)
	DECLARE_RCP(Socket)
	DECLARE_RCP(Start)
	DECLARE_RCP(Stop)
	DECLARE_RCP(Timeout)
	DECLARE_EMT(Read)
	DECLARE_EMT(Write)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Add)
		DEFINE_RCP(Remove)
		DEFINE_RCP(Socket)
		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)
		DEFINE_RCP(Timeout)

		DEFINE_EMT(Read)
		DEFINE_EMT(Write)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Client.  Socket client node.
//

class Client :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Client ( void );										// Constructor

	// Run-time data
	IDictionary		*pSkt;								// Socket context
	adtInt			iAddr;								// Address
	adtInt			iPort;								// Port
	adtInt			iTimeout;							// Timeout

	// CCL
	CCL_OBJECT_BEGIN(Client)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Accept)
	DECLARE_CON(Connect)
	DECLARE_CON(Connected)
	DECLARE_RCP(Address)
	DECLARE_RCP(Port)
	DECLARE_RCP(Socket)
	DECLARE_RCP(Timeout)
	DECLARE_EMT(Error)
	DECLARE_EMT(Pending)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Accept)
		DEFINE_CON(Connect)
		DEFINE_CON(Connected)

		DEFINE_RCP(Address)
		DEFINE_RCP(Port)
		DEFINE_RCP(Socket)
		DEFINE_RCP(Timeout)

		DEFINE_EMT(Error)
		DEFINE_EMT(Pending)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - DatagramOp.  Datagram operations node.
//

class DatagramOp :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	DatagramOp ( void );									// Constructor

	// Run-time data
	IDictionary		*pSkt;								// Socket context
	IByteStream		*pStm;								// Stream
	U8					cFrame[SIZE_FRAME_ETHERNET];	// Datagram buffer
	adtInt			iSz;									// Size
	adtInt			iAddr;								// Address
	adtInt			iPort;								// Port
	
	// CCL
	CCL_OBJECT_BEGIN(DatagramOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Address)
	DECLARE_CON(Port)
	DECLARE_CON(Receive)
	DECLARE_CON(Transmit)
	DECLARE_RCP(Size)
	DECLARE_RCP(Socket)
	DECLARE_RCP(Stream)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Address)
		DEFINE_CON(Port)
		DEFINE_CON(Receive)
		DEFINE_CON(Transmit)
		DEFINE_RCP(Size)
		DEFINE_RCP(Socket)
		DEFINE_RCP(Stream)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Interfaces.  Network interfaces node.
//

class Interfaces :
	public CCLObject,										// Base class
	public IBehaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	Interfaces ( void );									// Constructor

	// Run-time data
	#ifdef		_WIN32
	PIP_ADAPTER_ADDRESSES
						pAddrs;								// Adapter addresses
	#elif			__unix__ || __APPLE__
	struct ifaddrs	*ifap,*ifnxt;						// Interfaces
	#endif
	IThread			*pThrd;								// Worker thread
	HANDLE			hNotify;								// Notification handle
	HANDLE			hevNotify;							// Notificatione event
	bool				bRun;									// Run flag

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );
	
	// CCL
	CCL_OBJECT_BEGIN(Interfaces)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(First)
	DECLARE_CON(Next)
	DECLARE_EMT(Change)
	DECLARE_EMT(Last)
	DECLARE_RCP(Start)
	DECLARE_RCP(Stop)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(First)
		DEFINE_CON(Next)

		DEFINE_EMT(Change)
		DEFINE_EMT(Last)

		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)

	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - MulticastOp.  Multicast node.
//

class MulticastOp :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	MulticastOp ( void );								// Constructor

	// Run-time data
	IDictionary	*pSkt;									// Socket context
	adtInt		iAddrM,iAddrI;							// Address
	
	// CCL
	CCL_OBJECT_BEGIN(MulticastOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Add)
	DECLARE_CON(Remove)
	DECLARE_RCP(Interface)
	DECLARE_RCP(Multicast)
	DECLARE_RCP(Socket)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Add)
		DEFINE_CON(Remove)

		DEFINE_RCP(Interface)
		DEFINE_RCP(Multicast)
		DEFINE_RCP(Socket)

		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - PersistSktStm.  Stream portion of socket persist node.
//
class PersistSkt;
class PersistSktStm :
	public CCLObject,										// Base class
	public IByteStream									// Interface
	{
	public :
	PersistSktStm ( PersistSkt * );					// Constructor

	// Run-time data
	PersistSkt		*pSkt;								// Parent object

	// 'IByteStream' members
	STDMETHOD(available)	( U64 * );
	STDMETHOD(copyTo)		( IByteStream *, U64, U64 * );
	STDMETHOD(flush)		( void );
	STDMETHOD(read)		( void *, U64, U64 * );
	STDMETHOD(seek)		( S64, U32, U64 * );
	STDMETHOD(setSize)	( U64 );
	STDMETHOD(write)		( void const *, U64, U64 * );

	// CCL
	CCL_OBJECT_BEGIN_INT(PersistSktStm)
		CCL_INTF(IByteStream)
	CCL_OBJECT_END()
	};

//
// Class - PersistSkt.  Node to save/load values to/from sockets.
//

class PersistSkt :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	PersistSkt ( void );									// Constructor

	// Run-time data
	adtValue			vSave;								// Value to save
	IStreamPersist	*pPrs;								// Persistence parser
	IDictionary		*pSkt;								// Socket context
	SOCKET			skt;									// Active socket
	IByteStream		*pStmSv,*pStmLd;					// Byte caches
	PersistSktStm	*pStmPer;							// Stream persistence object
	char				szBfrZ[SIZE_PERSIST_CACHE];	// Zero buffer

	// CCL
	CCL_OBJECT_BEGIN(PersistSkt)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_CON(Load)
	DECLARE_CON(Save)
	DECLARE_RCP(Parser)
	DECLARE_RCP(Socket)
	DECLARE_RCP(Value)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Load)
		DEFINE_CON(Save)
		DEFINE_RCP(Parser)
		DEFINE_RCP(Socket)
		DEFINE_RCP(Value)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Recv.  Receive stream node.
//

class Recv :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Recv ( void );											// Constructor

	// Run-time data
	IDictionary		*pSkt;								// Socket context
	IByteStream		*pStm;								// Stream
	U8					cFrame[8192];						// Internal buffer
	adtInt			iTo;									// Timeout
	
	// CCL
	CCL_OBJECT_BEGIN(Recv)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Socket)
	DECLARE_RCP(Stream)
	DECLARE_RCP(Timeout)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Socket)
		DEFINE_RCP(Stream)
		DEFINE_RCP(Timeout)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Send.  Send stream node.
//

class Send :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Send ( void );											// Constructor

	// Run-time data
	IDictionary		*pSkt;								// Socket context
	IByteStream		*pStm;								// Stream
	adtInt			iSz;									// Size
	U8					*pBfr;								// Internal buffer
	U32				uSzBfr;								// Size of buffer
	adtInt			iTo;									// Timeout
	
	// CCL
	CCL_OBJECT_BEGIN(Send)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Size)
	DECLARE_RCP(Socket)
	DECLARE_RCP(Stream)
	DECLARE_RCP(Timeout)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Size)
		DEFINE_RCP(Socket)
		DEFINE_RCP(Stream)
		DEFINE_RCP(Timeout)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - SocketOp.  Socket context node.
//

class SocketOp :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	SocketOp ( void );										// Constructor

	// Run-time data
	IDictionary		*pSkt;								// Socket context
	adtInt			iAddr;								// Address
	adtInt			iPort;								// Port
	
	// CCL
	CCL_OBJECT_BEGIN(SocketOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Address)
	DECLARE_CON(Close)
	DECLARE_CON(Open)
	DECLARE_CON(Port)
	DECLARE_RCP(Query)
	DECLARE_RCP(Socket)
	DECLARE_EMT(Error)
	DECLARE_EMT(PeerAddress)
	DECLARE_EMT(PeerPort)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Address)
		DEFINE_CON(Close)
		DEFINE_CON(Open)
		DEFINE_CON(Port)

		DEFINE_RCP(Query)
		DEFINE_RCP(Socket)

		DEFINE_EMT(Error)
		DEFINE_EMT(PeerAddress)
		DEFINE_EMT(PeerPort)
	END_BEHAVIOUR_NOTIFY()

	private :

	};


// Protoypes
HRESULT	NetSkt_AddRef	( void );
HRESULT	NetSkt_Receive ( SOCKET, void *, U32, U32 *, U32 );
HRESULT	NetSkt_Release	( void );
HRESULT	NetSkt_Resolve ( const ADTVALUE &, adtInt &, adtInt & );
HRESULT	NetSkt_Send		( SOCKET, void const *, U32, U32 *, U32 );

#endif
