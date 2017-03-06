////////////////////////////////////////////////////////////////////////
//
//									DISPATCH.CPP
//
//					Implementation of the IDispatch node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

#ifdef 	_WIN32

Dispatch :: Dispatch ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pDctP	= NULL;
	}	// Dispatch

void Dispatch :: destruct ( void )
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
	_RELEASE(pDctP);
	_RELEASE(pDct);
	}	// destruct

HRESULT Dispatch :: onAttach ( bool bAttach )
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
	HRESULT			hr		= S_OK;
	adtIUnknown		unkV;

	// Attaching to graph
	if (bAttach)
		{
		adtValue		vL;

		// Defaults
		if (pnDesc->load ( adtString(L"Name"), vL ) == S_OK)
			adtValue::toString ( vL, strName );

		}	// if

	// Detaching from graph
	else
		{
		// Clean up
		_RELEASE(pDct);
		_RELEASE(pDctP);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Dispatch :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Open/access object
	if (_RCP(Open))
		{
		IDispatch	*pDsp		= NULL;
		CLSID			clsid		= GUID_NULL;
		DispIntf		*pIntf	= NULL;
		adtString	strId;
		adtValue		vL;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// An object should not already be active
		CCLTRYE ( pDct->load ( adtString(L"Object"), vL ) != S_OK,
						ERROR_ALREADY_INITIALIZED );

		// An "Id" is required
		CCLTRY ( pDct->load ( adtString(L"Id"), vL ) );
		CCLTRYE( (strId = vL).length() > 0, ERROR_INVALID_STATE );

		// If a non-GUID is specified, convert it
		if (hr == S_OK && strId[0] != '{')
			hr = CLSIDFromProgID ( strId, &clsid );
		else
			hr = CLSIDFromString ( strId, &clsid );

		// Attempt to create the object on an IDispatch interface
		CCLTRY ( CoCreateInstance ( clsid, NULL, CLSCTX_ALL, IID_IDispatch, (void **) &pDsp ) );

		// Create and assign interface to object
		CCLTRYE ( (pIntf = new DispIntf()) != NULL, E_OUTOFMEMORY );
		CCLTRY  ( pIntf->assign ( pDsp ) );

		// Store results
		CCLTRY ( pDct->store ( adtString(L"DispIntf"), adtIUnknown(pIntf) ) );


		// Result
		if (hr == S_OK)
			_EMT(Open,adtIUnknown(pDct));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pIntf);
		_RELEASE(pDsp);
		}	// if

	// Invoke method
	else if (_RCP(Fire))
		{
		DispIntf		*pIntf	= NULL;
		adtValue		vL;

		// State check
		CCLTRYE ( pDct != NULL && pDctP != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( strName.length() > 0, ERROR_INVALID_STATE );

		// Extract helper object	
		CCLTRY ( pDct->load ( adtString(L"DispIntf"), vL ) );
		CCLTRYE( (pIntf = (DispIntf *)(IUnknown *)adtIUnknown(vL)) != NULL,
					ERROR_INVALID_STATE );

		// Invoke function with parameters
		CCLTRY ( pIntf->invoke ( strName, pDctP ) );
		}	// else if

	// Close/release object
	else if (_RCP(Close))
		{
		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Closing
		_EMT(Close,adtIUnknown(pDct));

		// Remove active object
		CCLOK ( pDct->remove ( adtString(L"DispIntf") ); )
		}	// if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		hr = _QI(unkV,IID_IDictionary,&pDct);
		}	// else if
	else if (_RCP(Parameters))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDctP);
		hr = _QI(unkV,IID_IDictionary,&pDctP);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

///////////////////
// DispIntf object
///////////////////

DispIntf :: DispIntf ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDisp			= NULL;
	pDctFuncs	= NULL;

	// So creators do not have to do the initial AddRef
	AddRef();
	}	// DispIntf

HRESULT DispIntf :: assign ( IDispatch *_pDisp )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to assign an IDispatch interface to object.
	//
	//	PARAMETERS
	//		-	_pDisp is the dispatch interface
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;

	// Previous state
	unassign();

	// New state
	pDisp = _pDisp;
	_ADDREF(pDisp);

	// Valid ?
	if (pDisp != NULL)
		{
		ITypeInfo	*pInfo	= NULL;
		TYPEATTR		*pta		= NULL;

		// Create a dictionary to hold the function information
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctFuncs ) );

		// Query and allocate all information needed to make the required calls.
		// This saves time during execution and mapping provided parameters to/from object.
		CCLTRY ( pDisp->GetTypeInfo ( 0, LOCALE_USER_DEFAULT, &pInfo ) );

		// Get type attributes of interface
		CCLTRY ( pInfo->GetTypeAttr ( &pta ) );

		// Iterate the number of functions
		for (U32 f = 0;hr == S_OK && f < pta->cFuncs;++f)
			{
			IDictionary	*pFunc		= NULL;
			IDictionary	*pParams		= NULL;
			FUNCDESC		*fd			= NULL;
			BSTR			bstrName		= NULL;
			UINT			uNames		= 0;
			BSTR			bstrNames[100];

			// Function description
			CCLTRY ( pInfo->GetFuncDesc ( f, &fd ) );

			// Function name, etc
			CCLTRY ( pInfo->GetDocumentation ( fd->memid, &bstrName, NULL, NULL, NULL ) );

			// Store a dictionary for function under its name
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pFunc ) );
			CCLTRY ( pDctFuncs->store ( adtString(bstrName), adtIUnknown(pFunc) ) );
			CCLTRY ( pFunc->store ( adtString(L"Id"), adtLong(fd->memid) ) );

			// Names associated with function (parameters)
			memset ( bstrNames, 0, sizeof(bstrNames) );
			CCLTRY ( pInfo->GetNames ( fd->memid, bstrNames, 100, &uNames ) );

			// Store a dictionary for parameters under function
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pParams ) );
			CCLTRY ( pFunc->store ( adtString("Params"), adtIUnknown(pParams) ) );

			// Store parameter names along with the disp Id
			CCLOK ( lprintf ( LOG_INFO, L"%d) Function : %s,%d\r\n", f, bstrName, fd->memid ); )
			for (UINT i = 0;hr == S_OK && i < uNames;++i)
				{
				// Store parameter info, DISPID for parameters is index
				CCLTRY ( pParams->store ( adtString(bstrNames[i]), adtLong(i) ) );

				// Debug
				CCLOK  ( lprintf ( LOG_INFO, L"%d) Parameter : %s\r\n", i, bstrNames[i] ); )
				}	// for

			// Clean up
			for (UINT i = 0;i < uNames;++i)
				{
				_FREEBSTR(bstrNames[i]);
				}	// for
			_RELEASE(pParams);
			_RELEASE(pFunc);
			_FREEBSTR(bstrName);
			if (fd != NULL)
				pInfo->ReleaseFuncDesc(fd);
			}	// for

		// Clean up
		if (pta != NULL)
			pInfo->ReleaseTypeAttr ( pta );
		_RELEASE(pInfo);
		}	// if

	return hr;
	}	// assign

void DispIntf :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// Clean up
	unassign();
	}	// destruct

HRESULT DispIntf :: invoke ( const WCHAR *pwName, IDictionary *pDctP )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Invokes a function with parameters.
	//
	//	PARAMETERS
	//		-	pwName is the function name
	//		-	pDctP optionally contains input parameters and will receive
	//			output parameters.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pFunc	= NULL;
	IDictionary	*pParam	= NULL;
	DISPID		id			= 0;
	adtIUnknown	unkV;
	adtValue		vL;

	// State check
	CCLTRYE ( pDctFuncs != NULL, E_UNEXPECTED );

	// Obtain function and parameters dictionary
	CCLTRY ( pDctFuncs->load ( adtString(pwName), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pFunc));
	CCLTRY ( pFunc->load ( adtString(L"Id"), vL ) );
	CCLOK  ( id = adtInt(vL); )
	CCLTRY ( pFunc->load ( adtString(L"Params"), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pParam));

	// Invocation
	if (hr == S_OK)
		{
		VARIANT		*pvVars = NULL;
		DISPPARAMS	dprms;
		U32			sz;
		EXCEPINFO	ei;
		U32			ep;
		adtVariant	varRes;

		// Setup
		memset ( &dprms, 0, sizeof(dprms) );

		// Number of parameters, first one is a duplicate of the function name to skip that.
		CCLTRY	( pParam->size ( &sz ) );
		CCLOK		( sz--; )

		// Allocate memory for the full number of parameters
		if (sz > 0)
			{
			CCLOK ( dprms.cArgs = sz; )

			// Invoke array
			CCLTRYE	( (dprms.rgvarg	= (VARIANTARG *) _ALLOCMEM ( sz*sizeof(VARIANTARG) )) 
							!= NULL, E_OUTOFMEMORY );

			// Default variants for the parameters
			CCLTRYE	( (pvVars	= (VARIANT *) _ALLOCMEM ( sz*sizeof(VARIANT) )) 
							!= NULL, E_OUTOFMEMORY );
			}	// if

		// Initialize all the defaults
		for (U32 i = 0;hr == S_OK && i < sz;++i)
			{
			// Prepare structure
			VariantInit ( &dprms.rgvarg[i] );
			VariantInit ( &pvVars[i] );

			// Default parameter is a generic variant
			V_VT(&(dprms.rgvarg[0]))			= VT_BYREF|VT_VARIANT;
			V_VARIANTREF(&(dprms.rgvarg[0])) = &pvVars[i];
			}	// for

		// Execute
		CCLTRY ( pDisp->Invoke ( id, IID_NULL, LOCALE_USER_DEFAULT, DISPATCH_METHOD,
											&dprms, &varRes, &ei, &ep ) );

		// Clean up
		_FREEMEM(dprms.rgvarg);
		}	// if

	// Clean up
	_RELEASE(pParam);
	_RELEASE(pFunc);

	return hr;
	}	// invoke

		// Obtain function descriptor
//		CCLTRY ( pIntf->p)
/*
		LPOLESTR		pOleStr	= strName.pstr;
		adtIUnknown	unkV;
		adtValue		vL;
		DISPID		id;


		// Extract object
		CCLTRY ( pDct->load ( adtString(L"Object"), vL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IDispatch,&pDsp) );

		// Get the DISPID of the calling memeber
		CCLTRY ( pDsp->GetIDsOfNames ( IID_NULL, &pOleStr, 1, 
					LOCALE_USER_DEFAULT, &id ) );

		// Debug
		if (hr == S_OK)
			{
			ITypeInfo	*pInfo = NULL;
//			long			v;
			VARIANT		v;
			FUNCDESC		*fd = NULL;

			VariantInit(&v);

			// Type information test
			CCLTRY ( pDsp->GetTypeInfo(0,LOCALE_USER_DEFAULT,&pInfo) );
			CCLTRY ( pInfo->GetFuncDesc ( id, &fd ) );
if (fd != NULL)
	pInfo->ReleaseFuncDesc(fd);

			// Allocate parameter array

			// Initialize variant
			VariantInit ( &dprms.rgvarg[0] );
//			V_VT(&(dprms.rgvarg[0])) = VT_BYREF|VT_I4;
//			V_I4REF(&(dprms.rgvarg[0])) = &v;
			V_VT(&(dprms.rgvarg[0])) = VT_BYREF|VT_VARIANT;
			V_VARIANTREF(&(dprms.rgvarg[0])) = &v;
//			V_I4REF(&(dprms.rgvarg[0])) = &v;
			dprms.cArgs = 1;


			dbgprintf ( L"Hi\r\n" );

			}	// if
*/

void DispIntf :: unassign ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Clean up state associated with current interface
	//
	////////////////////////////////////////////////////////////////////////

	// Clean up
	_RELEASE(pDisp);
	_RELEASE(pDctFuncs);
	}	// unassign

#endif

