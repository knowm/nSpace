////////////////////////////////////////////////////////////////////////
//
//								LINK.CPP
//
//					Implementation of the 'link' node.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

Link :: Link ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pRootSrc	= NULL;
	pRootDst	= NULL;
	pLocPar	= NULL;
	}	// Link

HRESULT Link :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when Link behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	adtValue		v;

	// Attach
	if (bAttach)
		{
		adtIUnknown	unkV;

		// Attributes
		if (pnDesc->load(adtString(L"Source"),v) == S_OK)
			strSrc = v;
		if (pnDesc->load(adtString(L"Destination"),v) == S_OK)
			strDst = v;

		// Obtain reference to parent location
		CCLTRY ( pnLoc->load ( strnRefPar, v ) );
		CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pLocPar) );
		if (hr == S_OK) pLocPar->Release();
		}	// if

	// Detach
	else
		{
		_RELEASE(pRootDst);
		_RELEASE(pRootSrc);
		}	// else

	return hr;
	}	// onAttach

HRESULT Link :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Link
	if (_RCP(Link) || _RCP(Unlink))
		{
		adtString	strSrcAbs,strDstAbs;

		// State check
		CCLTRYE ( strDst.length() > 0,	ERROR_INVALID_STATE );
		CCLTRYE ( strSrc.length() > 0,	ERROR_INVALID_STATE );

		// Debug
//		if (hr == S_OK && !WCASECMP(strSrc,L"./Browser/Interface/"))
// 			dbgprintf ( L"Hi\r\n" );

		// Ensure an absolute namespace is used
		CCLTRY ( nspcPathTo ( (pRootDst != NULL) ? pRootDst : pLocPar, strDst, strDstAbs ) );
		CCLTRY ( nspcPathTo ( (pRootSrc != NULL) ? pRootSrc : pLocPar, strSrc, strSrcAbs ) );

		// Debug
//		if (hr == S_OK && !WCASECMP(this->strnName,L"ItemStateLink"))
//		if (hr == S_OK && !WCASECMP(this->strnName,L"LinkRenderApp"))
//		if (hr == S_OK && _RCP(Unlink))
//		if (hr == S_OK && !WCASECMP(strSrcAbs,L"/Lib/Image/Cache/Client/Default/State/"))
//			dbgprintf ( L"Hi\r\n" );
//		if (hr == S_OK && !WCASECMP(this->strnName,L"ItemStateLink"))
//			dbgprintf ( L"Hi\r\n" );

		// Link subgraph
//		DWORD dwT0 = GetTickCount();
//		CCLOK ( dbgprintf ( L"Link::receive:link:%s:%s\r\n", (LPCWSTR) strDstAbs, (LPCWSTR) strSrcAbs ); )
//		dbgprintf ( L"Link::receive:Link:%s:%s:%s\r\n", 
//						(LPCWSTR)strnName, (LPCWSTR)strSrcAbs, (LPCWSTR)strDstAbs );
		CCLTRY ( pnSpc->link ( strDstAbs, strSrcAbs, _RCP(Link) ) );
//		dbgprintf ( L"} Link::receive:Link:%s:%s:%s:0x%x\r\n", 
//						(LPCWSTR)strnName, (LPCWSTR)strSrcAbs, (LPCWSTR)strDstAbs, hr );
//		dbgprintf ( L"Link::receive:Link:Time %d\r\n", (GetTickCount()-dwT0) );

		// Debug
		if (hr != S_OK && _RCP(Link))
			dbgprintf ( L"Link::receive:Link:failed:%s:%s:%s:0x%x\r\n", 
								(LPCWSTR)strnName, (LPCWSTR)strSrcAbs, (LPCWSTR)strDstAbs, hr );

		// Result
		if (hr == S_OK)
			{
			if (_RCP(Link))
				_EMT(Link,strDstAbs);
			else
				_EMT(Unlink,strDstAbs);
			}	// if
		else
			_EMT(Error,adtInt(hr));
		}	// else if

	// State
	else if (_RCP(Source))
		strSrc = v;
	else if (_RCP(Destination))
		strDst = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

