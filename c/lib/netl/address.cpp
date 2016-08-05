////////////////////////////////////////////////////////////////////////
//
//									ADDRESS.CPP
//
//					Implementation of the address node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"
#include <stdio.h>

Address :: Address ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	iAddr		= INADDR_ANY;
	iPort		= 0;
	}	// Address

HRESULT Address :: onAttach ( bool bAttach )
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

	// Attach
	if (bAttach)
		NetSkt_AddRef();

	// Detach
	else
		NetSkt_Release();

	return S_OK;
	}	// onAttach

HRESULT Address :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Resolve
	if (_RCP(Resolve))
		{
		adtInt	iAddrR,iPortR;

		// Resolve address
		CCLTRY ( NetSkt_Resolve ( v, iAddrR, iPortR ) );

		// Result
		if (hr == S_OK)
			{
			_EMT(Address,iAddrR );
			if (iPortR != 0)
				_EMT(Port,iPortR );
			}	// if

		}	// if

	// String
	else if (_RCP(String))
		{
		struct in_addr	in;
		char				asciiStr[100],asciiPort[41];
		adtString		strRes;

		// Convert current address and port to a string format
		// Converts to : xx.xx.xx.xx:yyyy

		// Address
		if (hr == S_OK)
			{
			in.s_addr = htonl(iAddr);

			// Newer inet_ntop, etc not available on XP
			#if WINVER >= _WIN32_WINNT_VISTA
			CCLTRYE(inet_ntop(AF_INET, &in, asciiStr, sizeof(asciiStr)) != NULL, GetLastError() );
			#else
			char				*cp;
			hr = ((cp = inet_ntoa ( in )) != NULL) ? S_OK : S_FALSE;
			CCLOK ( STRCPY ( asciiStr, sizeof ( asciiStr ), cp ); )
			#endif
			}	// if

		// Port specified ?
		if (hr == S_OK && iPort > (U32)0)
			{
			SPF ( asciiPort, "%d", (U32)iPort );
			STRCAT ( asciiStr, sizeof(asciiStr), ":" );
			STRCAT ( asciiStr, sizeof(asciiStr), asciiPort );
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(String,(strRes = asciiStr) );
		}	// else if

	// State
	else if (_RCP(Address))
		hr = NetSkt_Resolve ( v, iAddr, iPort );
	else if (_RCP(Port))
		iPort = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

