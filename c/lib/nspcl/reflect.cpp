////////////////////////////////////////////////////////////////////////
//
//									REFLECT.CPP
//
//					Implementation of the location reflection node
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

Reflect :: Reflect ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	strLoc	= L"";
	}	// Reflect

HRESULT Reflect :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when this behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN Reflect
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Attach
	if (bAttach)
		{
		adtValue		vL;

		// Create dictionary to keep track of active reflections
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pRef ) );

		// Defaults
 		if (pnDesc->load ( adtString(L"Root"), vL )	== S_OK)
			receive ( prRoot, L"Value", vL );
 		if (pnDesc->load ( adtString(L"Location"), vL )	== S_OK)
			receive ( prLocation, L"Value", vL );
 		if (pnDesc->load ( adtString(L"Value"), vL )	== S_OK)
			receive ( prValue, L"Value", vL );
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pRef);
		}	// else

	return hr;
	}	// onAttach

HRESULT Reflect :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IReceptor
	//
	//	PURPOSE
	//		-	A location has received a Reflect on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	pl is the location
	//		-	v is the Reflect
	//
	//	RETURN Reflect
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Add location
	if (_RCP(Add))
		{
		ReflectRx	*pRx	= NULL;
		ILocation	*pLoc	= NULL;
		adtString	strAbs;
		adtValue		vL;
		adtIUnknown	unkV;

		// Generate full path
		CCLTRY ( strAbs.append ( strRoot ) );
		CCLTRY ( strAbs.append ( strLoc ) );

		// State check
		CCLTRYE ( strAbs.length() > 0,	ERROR_INVALID_STATE );

		// Is location already reflected ?
		CCLTRYE ( pRef->load ( strAbs, vL ) != S_OK, ERROR_INVALID_STATE );

		// Obtain location
		CCLTRY ( pnSpc->get ( strAbs, vL, NULL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_ILocation,&pLoc) );

		// Create a local reflection reciever for location
		CCLTRYE ( (pRx = new ReflectRx(this,strAbs,pLoc)) != NULL, E_OUTOFMEMORY );
		CCLOK   ( pRx->AddRef(); )
		CCLTRY  ( pRx->construct() );

		// Associate location with receiver
		CCLTRY ( pRef->store ( strAbs, adtIUnknown((IReceptor *)pRx) ) );

		// Emit result before flood of reflected values start emitting
		if (hr == S_OK)
			_EMT(Add,strAbs);
		else
			_EMT(Error,adtInt(hr));

		// Reflect location
		CCLTRY ( pLoc->connect ( pRx, true, true ) );

		// Result

		// Clean up
		_RELEASE(pRx);
		_RELEASE(pLoc);
		}	// if

	// Remove location
	else if (_RCP(Remove))
		{
		IReceptor	*pRx		= NULL;
		adtValue		vL;
		adtIUnknown	unkV;

		// State check
		CCLTRYE ( strRoot.length() > 0, ERROR_INVALID_STATE );

		// Obtain recevier for location
		CCLTRY ( pRef->load ( strRoot, vL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IReceptor,&pRx) );

		// Remove from dictionary
		CCLOK ( pRef->remove ( strRoot ); )

		// Disconnect from location
		// Do NOT use 'get' to get location since that could cause a re-instancing of a graph
		if (hr == S_OK && ((ReflectRx *)pRx)->pLoc != NULL)
			((ReflectRx *)pRx)->pLoc->connect ( pRx, false, true );

		// Result
		if (hr == S_OK)
			_EMT(Remove,strRoot);
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pRx);
		}	// else if

	// Store
	else if (_RCP(Store))
		{
		IReceptor	*pRcp		= NULL;
		IReceptor	*pRcpSrc	= NULL;
		adtValue		vL;
		adtIUnknown	unkV;
		int			len;

		// State check
		CCLTRYE ( strRoot.length() > 0,	ERROR_INVALID_STATE );
		CCLTRYE ( (len = strLoc.length()) > 0,	ERROR_INVALID_STATE );

		// Debug
//		adtString	strV;
//		if (adtValue::toString ( vValue, strV ) == S_OK)
//			{
//			dbgprintf ( L"Reflect::Receive:%s:Store:%s:%s\r\n", (LPCWSTR)strnName, (LPCWSTR)strLoc, (LPCWSTR)strV );
//			if (hr == S_OK && !WCASENCMP(strV,L"/You/",5))
//				dbgprintf ( L"Hi\r\n" );
//			}	// if
//		if (hr == S_OK && !WCASECMP(strnName,L"Listen") && !WCASECMP(strRoot,L"/Apps/Auto/Default/StateA/"))
//			dbgprintf ( L"Hi\r\n" );
//		if (hr == S_OK && !WCASECMP(strLoc,L"Image/OnFire/Value"))
//			dbgprintf ( L"Hi\r\n" );

		// No leading or trailing slashes for locations
//		if (hr == S_OK && strLoc[len-1] == '/')
//			strLoc.at(len-1) = '\0';

		// No leading slashes for locations
		if (hr == S_OK && strLoc[0] == '/')
			{
			adtString	strLoc_;
			CCLTRY ( strLoc_.append ( &strLoc[1] ) );
			CCLTRY ( adtValue::copy ( strLoc_, strLoc ) );
			}	// if
			
		// State check
		CCLTRYE ( strLoc.length(),	ERROR_INVALID_STATE );

		// If the root received is added to the reflection dictionary then use
		// that receptor as the 'sender' to avoid stores immediately resulting
		// in output.
		if (hr == S_OK && pRef->load ( strRoot, vL ) == S_OK)
			{
			// Receptor for source
			CCLTRY ( _QISAFE((unkV=vL),IID_IReceptor,&pRcpSrc) );
			}	// if

		// Obtain location
		if (hr == S_OK && pnSpc->get ( strRoot, vL, NULL ) == S_OK)
			{
			// Store/receive value into location
			CCLTRY ( _QISAFE((unkV=vL),IID_IReceptor,&pRcp) );
			CCLTRY ( pRcp->receive ( (pRcpSrc != NULL) ? pRcpSrc : this, strLoc, vValue ) );
			}	// if
		else if (hr == S_OK)
			{
			adtString	strLocFull;

			// NOTE: Try and satisfy the store request by detecting that maybe the given 'root' path
			// may not yet exist, if it doesn't, store the root path first.

			// Get root namespace
			CCLTRY ( pnSpc->get ( L"/", vL, NULL ) );

			// Generate full path to location
			CCLTRY ( strLocFull.append ( strRoot ) );
			CCLTRY ( strLocFull.append ( strLoc ) );

			// Store/receive value into location
			CCLTRY ( _QISAFE((unkV=vL),IID_IReceptor,&pRcp) );
			CCLTRY ( pRcp->receive ( (pRcpSrc != NULL) ? pRcpSrc : this, strLocFull, vValue ) );
			}	// if

		// Debug
		if (hr != S_OK)
			dbgprintf ( L"Reflect::Store:%s:0x%x:%s:%s\r\n", 
								(LPCWSTR)strnName, hr, (LPCWSTR)strRoot, (LPCWSTR)strLoc );

		// Clean up
		_RELEASE(pRcpSrc);
		_RELEASE(pRcp);
		}	// else if

	// State
	else if (_RCP(Root))
		hr = adtValue::toString ( v, strRoot );
	else if (_RCP(Location))
		{
		hr = adtValue::toString ( v, strLoc );
//		dbgprintf ( L"Reflect::Receive:%s:Location:%s\r\n", (LPCWSTR)strnName, (LPCWSTR)strLoc );
		}	// else if
	else if (_RCP(Value))
		{
		hr = adtValue::copy ( v, vValue );
//		adtString	strV;
//		if (adtValue::toString ( v, strV ) == S_OK)
//			dbgprintf ( L"Reflect::Receive:%s:Value:%s\r\n", (LPCWSTR)strnName, (LPCWSTR)strV );
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

/////////////
// ReflectRx
/////////////

ReflectRx :: ReflectRx ( Reflect *_pParent, const WCHAR *wLoc,
									ILocation *_pLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//	
	//	PARAMETERS
	//		-	_pParent is the parent reflect node
	//		-	wLoc is the root location being reflected
	//		-	_pLoc is the connected location
	//
	////////////////////////////////////////////////////////////////////////
	pParent	= _pParent;
	strRoot	= wLoc;
	pLoc		= _pLoc;
	}	// ReflectRx

void ReflectRx :: destruct ( void )
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

	// Notify target of disconnection
	if (pLoc != NULL)
		pLoc->connected ( this, false, false );

	}	// destruct

HRESULT ReflectRx :: receive ( IReceptor *prSrc, const WCHAR *pwLoc, 
											const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IReceptor
	//
	//	PURPOSE
	//		-	Called to store a value at a location.
	//
	//	PARAMETERS
	//		-	prSrc is the source of the reception
	//		-	pwLoc is the location
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Emit reflection
	if (pParent->csRx.enter())
		{
		pParent->peOnRoot->receive		( this, L"Value", strRoot );
		pParent->peOnLocation->receive( this, L"Value", adtString(pwLoc) );
		pParent->peOnValue->receive	( this, L"Value", v );
		pParent->csRx.leave();
		}	// if

	return hr;
	}	// receive
