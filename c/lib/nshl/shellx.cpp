////////////////////////////////////////////////////////////////////////
//
//									ShellX.CPP
//
//			Implementation of the external nSpace ShellX object
//
////////////////////////////////////////////////////////////////////////

#include "nshl_.h"

#ifdef	_WIN32

// Globals
extern HANDLE	hevTermX;

ShellX :: ShellX ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	}	// ShellX

HRESULT ShellX :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;

	// Create a dictionary to keep track of running namespaces (Command line,Namespace)
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pSpcs ) );

	return hr;
	}	// construct

void ShellX :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pSpcs);
	}	// destruct

void ShellX :: down ( NamespaceX *pSpc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a namespace object has been shut down.
	//
	//	PARAMETERS
	//		-	pSpc is the namespace that is shutdown
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	IIt		*pIt	= NULL;
	U32		sz;
	adtValue	vK,vV;

	// Find service in dictionary
	CCLTRY ( pSpcs->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		// Namespace for key
		if (	pSpcs->load ( vK, vV ) == S_OK &&
				adtValue::type(vV) == VTYPE_UNK &&
				((INamespaceX *)vV.punk) == ((INamespaceX *)pSpc) )
			break;

		// Next entry
		pIt->next();
		}	// while

	// Found ?
	if (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		// Remove from dictionary
		pSpcs->remove ( vK );

		// If there are no more namespaces, it is time for the external shell to shutdown
		if (pSpcs->size ( &sz ) == S_OK && sz == 0)
			SetEvent ( hevTermX );
		}	// if

	// Clean up
	_RELEASE(pIt);
	}	// down

HRESULT ShellX :: open ( BSTR szCmdLine, BOOL bShare, INamespaceX **ppSpc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IShellX
	//
	//	PURPOSE
	//		-	Open a namespace with the specified command line.
	//
	//	PARAMETERS
	//		-	szCmdLine is the command line to use for new namespace
	//		-	bShare is TRUE if sharing an existing namespace with the same
	//			command line is allowed.
	//		-	ppSpc will receive the external namespace object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtValue		vL;
	adtIUnknown	unkV;
	adtString	strCmdLine(szCmdLine);

	// Debug
	dbgprintf ( L"ShellX::open:szCmdLine %s bShare %d ppSpc %p {\r\n", 
						szCmdLine, bShare, ppSpc );

	// Setup
	(*ppSpc)	= NULL;

	// If sharing is desired, see if a namespace with the specified command
	// line is already running.
	if (	hr == S_OK &&
			pSpcs->load ( strCmdLine, vL ) == S_OK )
		{
		// Namespace interface
		hr = _QISAFE((unkV=vL),IID_INamespaceX,ppSpc);
		}	// if

	// If namespace is new, start up a new one
	if (hr == S_OK && *ppSpc == NULL)
		{
		NamespaceX	*pX		= NULL;
		IDictionary	*pDctCmd	= NULL;

		// Command line options
		CCLTRY ( strToVal ( strCmdLine, vL ) );
		CCLTRY ( _QISAFE((unkV = vL),IID_IDictionary,&pDctCmd) );

		// Create new external namespace object
		CCLTRYE ( (pX = new NamespaceX(this,pDctCmd)) != NULL, E_OUTOFMEMORY );
		CCLOK   ( pX->AddRef(); )
		CCLTRY  ( pX->construct() );

		// Interface
		CCLTRY ( _QI(pX,IID_INamespaceX,ppSpc) );

		// Store in cache for command line.  Store as reference to allow namespace
		// to self-destruct when other end releases it.
		CCLTRY ( pSpcs->store ( strCmdLine, adtIUnknownRef((*ppSpc)) ));

		// Clean up
		_RELEASE(pX);
		_RELEASE(pDctCmd);
		}	// if

	// Clean up
	dbgprintf ( L"} ShellX::open:hr 0x%x\r\n", hr );

	return hr;
	}	// open

#endif

