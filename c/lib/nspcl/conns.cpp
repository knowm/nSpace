////////////////////////////////////////////////////////////////////////
//
//								CONNS.CPP
//
//			Implementation of the connectors node.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

Connectors :: Connectors ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
//	pEmit		= NULL;
	pRecep	= NULL;
//	pConns	= NULL;
	}	// Connectors

Connectors :: ~Connectors ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
//	_RELEASE(pEmit);
	_RELEASE(pRecep);
//	_RELEASE(pConns);
	}	// ~Connectors

HRESULT Connectors :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when Notify behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Attach
	if (bAttach)
		{
		adtValue	v;

		// Create a dictionary to keep track of dynamically added receptors
//		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pConns ) );

		// Defaults
//		if (pnDesc->load(adtString(L"Type"),v) == S_OK)
//			receive ( prType, L"Value", v );
		}	// if

	// Detach
	else
		{
		// Clean up
//		_RELEASE(pConns);
//		_RELEASE(pEmit);
		_RELEASE(pRecep);
		}	// else

	return hr;
	}	// onAttach

HRESULT Connectors :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has Connectorsd a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Receive value into receptor
	if (_RCP(Receive))
		{
		IReceptor	*pRcp		= pRecep;
		const WCHAR	*pwLoc	= prl;
		int			len		= strLoc.length();

		// Debug
//		if (!WCASECMP(strnName,L"ReceiveStore"))
//			dbgprintf ( L"Hi\r\n" );

		// State check
		_ADDREF(pRcp);
		if (hr == S_OK && pRcp == NULL && (IUnknown *)NULL != (unkV=v))
			hr = _QI(unkV,IID_IReceptor,&pRcp);
		CCLTRYE ( pRcp != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( !adtValue::empty(vV), ERROR_INVALID_STATE );

		// Ensure proper format for receiving locations
		if (hr == S_OK && len > 0)
			{
			// Do not end in slash
			if (strLoc[len-1] == '/')
				strLoc.at(len-1) = '\0';

			// Use this location
			pwLoc = strLoc;

			// All paths relative
			if (pwLoc[0]== '/')
				++pwLoc;
			}	// if

		// Receptors value
		CCLTRY ( adtValue::copy ( vV, vVUse ) );
		CCLTRY ( pRcp->receive ( pr, pwLoc, vVUse ) );

		// Clean up
		_RELEASE(pRcp);
		}	// if
/*
	// Create a new connector for this node
	else if (_RCP(Add))
		{
		CCLObject	*unkConn		= NULL;
		IReceptor	*pRcpA		= NULL;
		IEmitter		*pEmtA		= NULL;
		IReceiver	*pRxA			= NULL;

		// Receptor
//		if (hr == S_OK && !WCASECMP(strType,L"Receptor"))
//			hr = ((unkConn = new Receptor()) != NULL) ? S_OK : E_OUTOFMEMORY;

		// Emitter
//		else if (hr == S_OK && !WCASECMP(strType,L"Emitter"))
//			hr = ((unkConn = new Emitter()) != NULL) ? S_OK : E_OUTOFMEMORY;

		// ?
//		else 
			hr = ERROR_INVALID_STATE;

		// Finish object construction
		CCLTRY ( unkConn->construct() );
		CCLOK  ( unkConn->AddRef(); )

		// Use this node as a receiver for receptors
		if (hr == S_OK && _QI(unkConn,IID_IReceptor,&pRcpA) == S_OK)
			{
			IDictionary	*pDctR	= NULL;

			// Generate unique name from ptr.
			CCLTRY ( adtValue::toString ( adtIUnknown(pRcpA), strName ) );
			
			// Use this node for receiver
			CCLTRY	( _QI(pnNode,IID_IReceiver,&pRxA) );

			// Receiver
			CCLTRY  ( _QI(pRcpA,IID_IDictionary,&pDctR) );
			CCLTRY  ( pDctR->store ( adtString(STR_NSPC_RCVR), adtIUnknown(pRxA) ) );
			CCLTRY  ( pDctR->store ( adtString(STR_NSPC_NAME), strName ) );

			// Clean up
			_RELEASE(pDctR);
			}	// if

		// Emitter ?
		if (hr == S_OK)
			_QI(unkConn,IID_IEmitter,&pEmtA);

		// Store connector
//		CCLTRY	( pConns->store ( adtIUnknown(unkConn), adtInt(0) ) );

		// Result
		if (hr == S_OK)
			{
			if (pRcpA != NULL)
				_EMT(Add,adtIUnknown(pRcpA));
			else
				_EMT(Add,adtIUnknown(pEmtA));
			}	// if
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pRxA);
		_RELEASE(pRcpA);
		_RELEASE(pEmtA);
		_RELEASE(unkConn);
		}	// else if

	// Emit value out emitter
	else if (_RCP(Emit))
		{
		IEmitter		*pE	= pEmit;

		// State check
		_ADDREF(pE);
		if (hr == S_OK && pE == NULL && (IUnknown *)NULL != (unkV=v))
			hr = _QI(unkV,IID_IEmitter,&pE);
		CCLTRYE ( pE != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( !adtValue::empty(vV), ERROR_INVALID_STATE );

		// Connectors value
//if (hr == S_OK)
//	dbgprintf ( L"Connectors::receive:Emit:%p:%p\r\n", pE, vV.punk );
		CCLTRY ( pE->emit ( vV ) );

		// Result
		CCLOK ( _EMT(Emit,vV); )

		// Clean up
		_RELEASE(pE);
		}	// if

	// Get latest value from emitter
	else if (_RCP(Get))
		{
		IEmitter		*pE		= pEmit;
		IDictionary	*pDctE	= NULL;

		// State check
		_ADDREF(pE);
		if (hr == S_OK && pEmit == NULL && (IUnknown *)NULL != (unkV=v))
			hr = _QI(unkV,IID_IEmitter,&pE);
		CCLTRYE ( pE != NULL, ERROR_INVALID_STATE );

		// Get value
		CCLTRY ( _QI(pE,IID_IDictionary,&pDctE) );
		CCLTRY ( pDctE->load ( adtString(L"Value"), vGet ) );
//		CCLTRY ( pE->value ( vGet ) );

		// Result
		if (hr == S_OK)	_EMT(Get,vGet);
		else					_EMT(Error,adtIUnknown(pE));

		// Clean up
		_RELEASE(pDctE);
		_RELEASE(pE);
		}	// else if

	// Obtain receptors connected to emitter
	else if (_RCP(Receptors))
		{
		IEmitter		*pE		= pEmit;
		IContainer	*pRcps	= NULL;

		// State check
		_ADDREF(pE);
		if (hr == S_OK && pE == NULL && (IUnknown *)NULL != (unkV=v))
			hr = _QI(unkV,IID_IEmitter,&pE);
		CCLTRYE ( pE != NULL, ERROR_INVALID_STATE );

		// Receptor container
//		CCLTRY ( pE->receptors ( &pRcps ) );
		hr = ERROR_INVALID_STATE;
		dbgprintf ( L"Error::receptors\r\n" );

		// Result
		CCLOK ( _EMT(Receptors,(unkV=pRcps)); )

		// Clean up
		_RELEASE(pRcps);
		_RELEASE(pE);
		}	// else if

	// Clear latest value from emitter
	else if (_RCP(Clear))
		{
		IEmitter		*pE		= pEmit;
		IDictionary	*pDctE	= NULL;

		// State check
		_ADDREF(pE);
		if (hr == S_OK && pE == NULL && (IUnknown *)NULL != (unkV=v))
			hr = _QI(unkV,IID_IEmitter,&pE);
		CCLTRYE ( pE != NULL, ERROR_INVALID_STATE );

		// Clear emitter output
		CCLTRY ( _QI(pE,IID_IDictionary,&pDctE) );
		CCLTRY ( pDctE->clear() );

		// Clean up
		_RELEASE(pDctE);
		_RELEASE(pE);
		}	// else if

	// Connect receptor to emitter
	else if (_RCP(Connect))
		{
		// State check
		CCLTRYE ( pRecep != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pEmit != NULL, ERROR_INVALID_STATE );

//if (!WCASECMP(this->strnName,L"ConnRx"))
//	dbgprintf ( L"%s\r\n", (LPCWSTR)this->strnName );

		// Perform connect
		CCLTRY ( pEmit->connect ( pRecep, true ) );

		// Error ?
		if (hr != S_OK)
			_EMT(Error,adtInt(hr));
		}	// if

	// Disconnect receptor from emitter
	else if (_RCP(Disconnect))
		{
		// State check
		CCLTRYE ( pRecep != NULL,	ERROR_INVALID_STATE );
		CCLTRYE ( pEmit != NULL,	ERROR_INVALID_STATE );

		// Perform disconnect
		CCLTRY ( pEmit->connect ( pRecep, false ) );

		// Error ?
		if (hr != S_OK)
			_EMT(Error,adtInt(hr));
		}	// else if

	// Dynamically added receptor
	else
		{
		IEmitter		*pE	= NULL;

//if (!WCASECMP(this->strnName,L"ConnAdd"))
//	dbgprintf ( L"%s\r\n", (LPCWSTR)this->strnName );
		// Emit receptor and value recevied
//		dbgprintf ( L"Receptors::receive:Dynamic!\r\n" );
		_EMT(Receptor,(unkV=pr));
		_EMT(Value,v);

		// Connectors are also emitters, forward value out emitter
//		if (_QI(pr,IID_IEmitter,&pE) == S_OK)
//			{
			// Continue the flow...
//			pE->emit(v);
//			}	// if

		// Clean up
		_RELEASE(pE);
		}	// else

	// State
	else if (_RCP(Type))
		hr = adtValue::copy ( adtString(v), strType );
	else if (_RCP(Emitter))
		{
		_RELEASE(pEmit);
		_QISAFE((unkV=v),IID_IEmitter,&pEmit);
		}	// else if
*/
	else if (_RCP(Receptor) )
		{
		_RELEASE(pRecep);
		_QISAFE((unkV=v),IID_IReceptor,&pRecep);
		}	// else if
	else if (_RCP(Location))
		{
		adtString	strV(v);
		CCLTRY ( adtValue::copy ( strV, strLoc ) );
		}	// else if
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vV );

	return hr;
	}	// Connectors

