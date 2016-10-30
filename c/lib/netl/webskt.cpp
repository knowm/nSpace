////////////////////////////////////////////////////////////////////////
//
//									WEBSKT.CPP
//
//					Implementation of the web socket server
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"
#include <stdio.h>

#ifdef	USE_WEBSKTPP

WebSktSrvr :: WebSktSrvr ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the WebSktSrvrability node
	//
	////////////////////////////////////////////////////////////////////////
	pThrd		= NULL;
	iPort		= 0;
	}	// WebSktSrvr

HRESULT WebSktSrvr :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Server setup
	server.init_asio();
	server.set_open_handler(bind(&WebSktSrvr::on_open,this,std::placeholders::_1));
	server.set_close_handler(bind(&WebSktSrvr::on_close,this,std::placeholders::_1));
	server.set_message_handler(bind(&WebSktSrvr::on_message,this,
										std::placeholders::_1,std::placeholders::_2));

	return hr;
	}	// construct

void WebSktSrvr :: destruct ( void )
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

	// Shutdown thread
	if (pThrd != NULL)
		{
		pThrd->threadStop(5000);
		pThrd->Release();
		pThrd = NULL;
		}	// if

	}	// destruct

HRESULT WebSktSrvr :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when this behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		adtValue	v;

		// Attributes
		if (hr == S_OK && pnDesc->load ( adtString(L"Port"), v ) == S_OK)
			iPort = v;
		}	// if

	// Detach
	else
		{
		// Shutdown thread
		if (pThrd != NULL)
			{
			pThrd->threadStop(5000);
			pThrd->Release();
			pThrd = NULL;
			}	// if
		}	// if

	return hr;
	}	// onAttach

void WebSktSrvr :: on_close ( websocketpp::connection_hdl hdl )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a connection is closed.
	//
	//	PARAMETERS
	//		-	hdl is the connection
	//
	////////////////////////////////////////////////////////////////////////
	lprintf ( LOG_INFO, L"on_close\r\n" );
	m_connections.erase(hdl);
	}	// on_close

void WebSktSrvr :: on_message ( websocketpp::connection_hdl hdl,
											server_t::message_ptr msg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a message is received.
	//
	//	PARAMETERS
	//		-	hdl is the connection
	//		-	msg is the messasge
	//
	////////////////////////////////////////////////////////////////////////
	lprintf ( LOG_INFO, L"on_message\r\n" );
	for (auto it : m_connections)
		server.send(it,msg);
	}	// on_message

void WebSktSrvr :: on_open ( websocketpp::connection_hdl hdl )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a connection is opened.
	//
	//	PARAMETERS
	//		-	hdl is the connection
	//
	////////////////////////////////////////////////////////////////////////
	lprintf ( LOG_INFO, L"on_open\r\n" );
	m_connections.insert(hdl);
	}	// on_open

HRESULT WebSktSrvr :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has received a value on the specified receptor.
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

	// Start
	if (_RCP(Start))
		{
		// State check
		CCLTRYE ( pThrd == NULL, ERROR_INVALID_STATE );
//		dbgprintf ( L"%s:Avail::receive:Start\r\n", (LPCWSTR)strnName );

		// Start server thread
		CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
		CCLTRY(pThrd->threadStart ( this, 5000 ));
		}	// if

	// Stop
	else if (_RCP(Stop))
		{
		// State check
		CCLTRYE ( pThrd != NULL, ERROR_INVALID_STATE );
//		dbgprintf ( L"%s:Avail::receive:Stop\r\n", (LPCWSTR)strnName );

		// Shutdown worker thread
		if (hr == S_OK)
			pThrd->threadStop(5000);
		_RELEASE(pThrd);
		}	// if

	// State
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// onReceive

HRESULT WebSktSrvr :: tickAbort ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' should abort.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	server.stop_listening();
	return S_OK;
	}	// tickAbort

HRESULT WebSktSrvr :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Perform one 'tick's worth of work.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Run server
	server.listen(iPort);
	server.start_accept();
	server.run();

	// Done when server exits
	return S_FALSE;
	}	// tick

HRESULT WebSktSrvr :: tickBegin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that it should get ready to 'tick'.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// tickBegin

HRESULT WebSktSrvr :: tickEnd ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' is to stop.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// tickEnd

#endif
