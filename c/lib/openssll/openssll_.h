////////////////////////////////////////////////////////////////////////
//
//										OPENSSLL_.H
//
//			Implementaiton include file for the Open SSL node library
//
////////////////////////////////////////////////////////////////////////

#ifndef	OPENSSLL__H
#define	OPENSSLL__H

// Includes
#include	"openssll.h"

// OpenSSL
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/err.h>

//
// Class - libSSL.  OpenSSL library object.
//

class libSSL
	{
	public :
	libSSL ( void );										// Constructor

	// Run-time data
	bool				bValid;								// Library valid
	S32				lcnt,lsslcnt;						// Reference counts
	SSL_CTX			*psslctx;							// Context

	// Utilities
	virtual LONG		AddRef		();				// Add reference to library
	virtual HRESULT	errors		( const WCHAR * );
	virtual LONG		Release		();				// Remove reference from library
	virtual LONG		sslAddRef	();				// Add reference to SSL library
	virtual LONG		sslRelease	();				// Release reference to SSL library
	};

//
// Class - objSSL.  Reference counted OpenSSL objects.
//

class objSSL :
	public CCLObject,										// Base class
	public IThis											// Interface
	{
	public :
	objSSL ( RSA * );										// Constructor

	// Run-time data
	RSA	*rsa;												// RSA Object

	// 'IThis' members
	STDMETHOD(getThis)	( void **ppv ) { *ppv = this; return S_OK; }

	// CCL
	CCL_OBJECT_BEGIN_INT(objSSL)
		CCL_INTF(IThis)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object
	};

//
// Class - EVPSign.  Node for EVP signing functionality.
//

class EVPSign :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	EVPSign ( void );										// Constructor

	// Run-time data
	adtString		strType;								// Signature type
	EVP_MD_CTX		*pctx;								// EVP context
	IByteStream		*pStm;								// Current stream
	char				cBfr[1024];							// Stream buffer
	IThis				*pKey;								// Private key

	// CCL
	CCL_OBJECT_BEGIN(EVPSign)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Begin)
	DECLARE_EMT(Error)
	DECLARE_CON(Final)
	DECLARE_RCP(Key)
	DECLARE_RCP(Stream)
	DECLARE_CON(Update)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Begin)
		DEFINE_EMT(Error)
		DEFINE_CON(Final)
		DEFINE_RCP(Key)
		DEFINE_RCP(Stream)
		DEFINE_CON(Update)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - EVPVerify.  Node for EVP signing verification functionality.
//

class EVPVerify :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	EVPVerify ( void );									// Constructor

	// Run-time data
	adtString		strType;								// Signature type
	EVP_MD_CTX		*pctx;								// EVP context
	IByteStream		*pStm;								// Current stream
	char				cBfr[1024];							// Stream buffer
	IThis				*pKey;								// Private key

	// CCL
	CCL_OBJECT_BEGIN(EVPVerify)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Begin)
	DECLARE_EMT(Error)
	DECLARE_CON(Final)
	DECLARE_RCP(Key)
	DECLARE_RCP(Stream)
	DECLARE_CON(Update)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Begin)
		DEFINE_EMT(Error)
		DEFINE_CON(Final)
		DEFINE_RCP(Key)
		DEFINE_RCP(Stream)
		DEFINE_CON(Update)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - PEMImpl.  Node to perform PEM algorithms.
//

class PEMImpl :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	PEMImpl ( void );										// Constructor

	// Run-time data
	IByteStream		*pStm;								// Byte stream
	IThis				*pObj;								// Active object
	adtBool			bPub;									// Public/private
	adtString		strType;								// Key type

	// CCL
	CCL_OBJECT_BEGIN(PEMImpl)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_RCP(Object)
	DECLARE_CON(Read)
	DECLARE_RCP(Stream)
	DECLARE_CON(Write)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_RCP(Object)
		DEFINE_CON(Read)
		DEFINE_RCP(Stream)
		DEFINE_CON(Write)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - RSAImpl.  Node to perform RSA algorithms.
//

class RSAImpl :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	RSAImpl ( void );											// Constructor

	// Run-time data
	adtInt			iBits;								// Run-time data

	// CCL
	CCL_OBJECT_BEGIN(RSAImpl)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Bits)
	DECLARE_EMT(Error)
	DECLARE_CON(Generate)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Bits)
		DEFINE_EMT(Error)
		DEFINE_CON(Generate)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - SSLConnect.  Node to handle SSL connection.
//

class SSLConnect :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SSLConnect ( void );									// Constructor

	// Run-time data
	IDictionary		*pDsc;								// Descriptor
	IByteStream		*pStm;								// I/O stream
	adtInt			iSz;									// I/O size

	// CCL
	CCL_OBJECT_BEGIN(SSLConnect)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Attach)
	DECLARE_CON(Detach)
	DECLARE_RCP(Descriptor)
	DECLARE_CON(Read)
	DECLARE_RCP(Size)
	DECLARE_RCP(Stream)
	DECLARE_CON(Write)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Attach)
		DEFINE_CON(Detach)
		DEFINE_RCP(Descriptor)
		DEFINE_CON(Read)
		DEFINE_RCP(Size)
		DEFINE_RCP(Stream)
		DEFINE_CON(Write)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :
	// Internal utilities
	HRESULT connect ( SSL * );

	};

#endif
