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

// Type information is generated for every type of IDispatch interface
// for ease of calling.  This dictionary caches already generated
// information for the same GUID type information.
static IDictionary	*pDctTypeInfo	= NULL;
static U32				uRefCnt			= 0;

Dispatch :: Dispatch ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pIntf	= NULL;
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
	_RELEASE(pIntf);
	_RELEASE(pIntfs);
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
		if (pnDesc->load ( adtString(L"Params"), vL ) == S_OK)
			{
			adtIUnknown	unkV(vL);
			_QISAFE(unkV,IID_IDictionary,&pDctP);
			}	// if

		// Created interface objects are tracked in order to allow for 'close'
		CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pIntfs ) );

		// Type information dictionary
		if (++uRefCnt == 1)
			{
			COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctTypeInfo );
			}	// if

		}	// if

	// Detaching from graph
	else
		{
		// Enusre interfaces are closed
		receive(prClose,L"",adtInt());

		// Clean up
		_RELEASE(pIntf);
		_RELEASE(pDctP);
		if (uRefCnt > 0 && --uRefCnt == 0)
			{
			_RELEASE(pDctTypeInfo);
			}	// if
		_RELEASE(pIntfs);
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
		IDispatch	*pDsp			= NULL;
		CLSID			clsid			= GUID_NULL;
		DispIntf		*pIntfNew	= NULL;
		adtValue		vL;

		// An object name ("Id") is required
		CCLTRYE( strName.length() > 0, ERROR_INVALID_STATE);

		// If a non-GUID is specified, convert it
		if (hr == S_OK && strName[0] != '{')
			hr = CLSIDFromProgID ( strName, &clsid );
		else
			hr = CLSIDFromString ( strName, &clsid );

		// Attempt to create the object on an IDispatch interface
		CCLTRY ( CoCreateInstance ( clsid, NULL, CLSCTX_ALL, IID_IDispatch, (void **) &pDsp ) );

		// Create and assign interface to object
		CCLTRYE ( (pIntfNew = new DispIntf(this)) != NULL, E_OUTOFMEMORY );
		CCLTRY  ( pIntfs->write ( adtIUnknown(pIntfNew) ) );
		CCLTRY  ( pIntfNew->assign ( pDsp ) );

		// Result
		if (hr == S_OK)
			_EMT(Open,adtIUnknown(pIntfNew));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pIntfNew);
		_RELEASE(pDsp);
		}	// if

	// Close all interfaces for this node
	else if (_RCP(Close))
		{
		IIt		*pIt	= NULL;
		adtValue vL;

		// Parent node for interfaces ?
		CCLTRYE( pIntfs != NULL, ERROR_INVALID_STATE );

		// Close interfaces
		CCLTRY ( pIntfs->iterate ( &pIt ) );
		while (hr == S_OK && pIt->read ( vL ) == S_OK)
			{
			adtIUnknown unkV(vL);

			// Unassign the interface
			if ((IUnknown *)(NULL) != unkV)
				((DispIntf *)(IUnknown *)unkV)->unassign();

			// Next interface
			pIt->next();
			}	// while

		// Done with interfaces
		if (pIntfs != NULL)
			pIntfs->clear();

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pIntf);
		}	// else if

	// Invoke method
	else if (_RCP(Fire))
		{
		IDispatch	*pDisp	= NULL;
		adtValue		vL;
		adtIUnknown	unkV;

		// State check
		CCLTRYE ( pIntf != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( strName.length() > 0, ERROR_INVALID_STATE );

		// Invoke function with parameters
		CCLTRY ( pIntf->invoke ( strName, pDctP, vL ) );

		// If the result is another interface, assign an object to it
		// so it can be used in this node to make other calls.
		if (	hr == S_OK && 
				(IUnknown *)(NULL) != (unkV=vL) &&
				_QI(unkV,IID_IDispatch,&pDisp) == S_OK )
			{
			DispIntf		*pIntfNew	= NULL;

			// Debug
//			if (!WCASECMP(strnName,L"Tables"))
//				dbgprintf ( L"Hi\r\n" );

			// Create and assign interface to object
			CCLTRYE ( (pIntfNew = new DispIntf(pIntf->prParent)) != NULL, E_OUTOFMEMORY );
			if (hr == S_OK && pIntf->prParent != NULL)
				hr = pIntf->prParent->pIntfs->write(adtIUnknown(pIntfNew));
			CCLTRY  ( pIntfNew->assign ( pDisp ) );

			// Object to emit
			CCLTRY ( adtValue::copy ( adtIUnknown(pIntfNew), vL ) );

			// Clean up
			_RELEASE(pIntfNew);
			_RELEASE(pDisp);
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,vL);
		else
			_EMT(Error,adtInt(hr));
		}	// else if

	// State
	else if (_RCP(Iface))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pIntf);
		pIntf = (DispIntf *)(IUnknown *)unkV;
		_ADDREF(pIntf);
		}	// else if
	else if (_RCP(Params))
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

DispIntf :: DispIntf ( Dispatch *_prParent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_prParent is a reference to the parent node
	//
	////////////////////////////////////////////////////////////////////////
	prParent		= _prParent;
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
	HRESULT	hr				= S_OK;
	bool		bNeedInfo	= true;

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

		// Query and allocate all information needed to make the required calls.
		// This saves time during execution and mapping provided parameters to/from object.
		CCLTRY ( pDisp->GetTypeInfo ( 0, LOCALE_USER_DEFAULT, &pInfo ) );

		// Get type attributes of interface
		CCLTRY ( pInfo->GetTypeAttr ( &pta ) );

		// Check if a function dictionary already exists for this type information
		if (hr == S_OK)
			{
			LPOLESTR			pId	= NULL;
			adtValue			vL;
			adtIUnknown		unkV;

			// Convert GUID to string for look-up
			CCLTRY ( StringFromCLSID ( pta->guid, &pId ) );

			// Already exists ?
			if (hr == S_OK && pDctTypeInfo->load ( adtString(pId), vL ) == S_OK)
				{
				// Extract function dictionary
				CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pDctFuncs) );

				// Do not need to re-generate
				CCLOK ( bNeedInfo = false; )
				}	// if
			else
				{
				// Create a new dictionary for the type information
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctFuncs ) );

				// Store under its GUID
				CCLTRY ( pDctTypeInfo->store ( adtString(pId), adtIUnknown(pDctFuncs) ) );
				}	// else 

			}	// if

		// Need to generate type information ?
		if (hr == S_OK && bNeedInfo)
			{
			// Iterate the number of functions
			for (U32 f = 0;hr == S_OK && f < pta->cFuncs;++f)
				{
				IDictionary	*pFunc		= NULL;
				IDictionary	*pParams		= NULL;
				FUNCDESC		*fd			= NULL;
				BSTR			bstrName		= NULL;
				UINT			uNames		= 0;
				BSTR			bstrNames[100];
				adtString	strName;

				// Function description
				CCLTRY ( pInfo->GetFuncDesc ( f, &fd ) );

				// Function name, etc
				CCLTRY ( pInfo->GetDocumentation ( fd->memid, &bstrName, NULL, NULL, NULL ) );

				// Store a dictionary for function under its name
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pFunc ) );
				CCLTRY ( pFunc->store ( adtString(L"Id"), adtLong(fd->memid) ) );
				CCLTRY ( pFunc->store ( adtString(L"Method"), adtInt (
												(fd->invkind == INVOKE_PROPERTYGET) ?	DISPATCH_PROPERTYGET :
												(fd->invkind == INVOKE_PROPERTYPUT) ?	DISPATCH_PROPERTYPUT :
																									DISPATCH_METHOD ) ) );

				// The same function can be used twice, once for 'put' another for 'get', 
				// use that prefix based on the invocation kind for unambiguous use
				if (hr == S_OK)
					{
					// Base name
					strName = bstrName;
					if (fd->invkind == INVOKE_PROPERTYGET)
						hr = strName.prepend(L"get_");
					else if (fd->invkind == INVOKE_PROPERTYPUT)
						hr = strName.prepend(L"put_");
					}	// if

				// Store under generated name
				CCLTRY ( pDctFuncs->store ( strName, adtIUnknown(pFunc) ) );

				// Names associated with function (parameters)
				memset ( bstrNames, 0, sizeof(bstrNames) );
				CCLTRY ( pInfo->GetNames ( fd->memid, bstrNames, 100, &uNames ) );

				// Store a dictionary for parameters under function
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pParams ) );
				CCLTRY ( pFunc->store ( adtString("Params"), adtIUnknown(pParams) ) );

				// DEBUG
	//			if (hr == S_OK && !WCASECMP(bstrName,L"Workbooks"))
	//			if (hr == S_OK && !WCASECMP(bstrName,L"Cells"))
	//				dbgprintf ( L"Hi\r\n" );

				// Store parameter names along with the disp Id
				// First name is the function name so skip for parameters.
//				CCLOK ( lprintf ( LOG_INFO, L"%d) Function : %s,%d,%d\r\n", f, (LPCWSTR)strName,
//										fd->memid, fd->invkind ); )
				for (UINT i = 1;hr == S_OK && i < uNames;++i)
					{
					// Store parameter info, DISPID for parameters is index
					CCLTRY ( pParams->store ( adtString(bstrNames[i]), adtLong(i-1) ) );

					// Debug
//					CCLOK  ( lprintf ( LOG_INFO, L"%d) Parameter : %s\r\n", i-1, bstrNames[i] ); )
					}	// for

				// Special case.  For 'property put' methods, allow a value parameter to be specified
				// as part of the call.
				if (hr == S_OK && fd->invkind == INVOKE_PROPERTYPUT)
					hr = pParams->store ( adtString(L"Value"), adtLong(0) );

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

			}	// if

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

HRESULT DispIntf :: invoke ( const WCHAR *pwName,
										IDictionary *pDctP, adtValue &vRes )
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
	//		-	vRes will receive the result.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pFunc	= NULL;
	IDictionary	*pParam	= NULL;
	IIt			*pItP		= NULL;
	DISPID		id			= 0;
	adtInt		iMethod	= 0;
	adtIUnknown	unkV;
	adtValue		vL;

	// State check
	CCLTRYE ( pDctFuncs != NULL, E_UNEXPECTED );

	// Obtain function and parameters dictionary
	CCLTRY ( pDctFuncs->load ( adtString(pwName), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pFunc));
	CCLTRY ( pFunc->load ( adtString(L"Id"), vL ) );
	CCLOK  ( id = adtInt(vL); )
	CCLTRY ( pFunc->load ( adtString(L"Method"), vL ) );
	CCLOK  ( iMethod = adtInt(vL); )
	CCLTRY ( pFunc->load ( adtString(L"Params"), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pParam));

	// Debug
//	if (!WCASECMP(pwName,L"Open"))
//		lprintf ( LOG_INFO, L"Hi\r\n" );
//	if (!WCASECMP(pwName,L"put_Value"))
//		lprintf ( LOG_INFO, L"Hi\r\n" );

	// Invocation
	if (hr == S_OK)
		{
		DISPID		idPut = DISPID_PROPERTYPUT;
		DISPPARAMS	dprms;
		U32			sz;
		EXCEPINFO	ei;
		U32			ep;
		adtVariant	varRes;

		// Setup
		memset ( &dprms, 0, sizeof(dprms) );
		memset ( &ei, 0, sizeof(ei) );

		// Number of parameters
		CCLTRY	( pParam->size ( &sz ) );

		// Allocate memory for the full number of parameters
		if (sz > 0)
			{
			// Total number of arguments
			CCLOK ( dprms.cArgs = sz; )

			// Invoke array
			CCLTRYE	( (dprms.rgvarg	= (VARIANTARG *) _ALLOCMEM ( sz*sizeof(VARIANTARG) )) 
							!= NULL, E_OUTOFMEMORY );
			}	// if

		// Initialize all the defaults
		for (U32 i = 0;hr == S_OK && i < sz;++i)
			{
			// This is how optional/missing IDispatch parameters are specified
			VariantInit ( &dprms.rgvarg[i] );
			dprms.rgvarg[i].vt		= VT_ERROR;
			dprms.rgvarg[i].scode	= DISP_E_PARAMNOTFOUND;
			}	// for

		// Iterate the provided parameters and use matching available names for function
		if (hr == S_OK && pDctP != NULL)
			{
			// Iterate names
			CCLTRY ( pDctP->keys ( &pItP ) );
			while (hr == S_OK && pItP->read ( vL ) == S_OK)
				{
				adtString	strName(vL);

				// See if the parameter is in the list retrieved from the type information
				if (pParam->load ( strName, vL ) == S_OK)
					{
					adtLong		iIdx(vL);
					adtVariant	varArg;

					// Parameter exists, put in list under its index

					// Value of provided parameter
					CCLTRY ( pDctP->load ( strName, vL ) );

					// DEBUG
//					if (hr == S_OK && adtValue::type(vL) == VTYPE_STR &&
//							!WCASECMP(vL.pstr,L"A95"))
//						dbgprintf ( L"Hi\r\n" );

					// Variant version of argument
					CCLOK ( varArg = vL; )

					// First parameter is function name so offset
					// Also, arguments are specified last to first
					CCLOK ( iIdx = (sz - iIdx) - 1; )

					// Replace missing value with variant
					CCLTRY ( VariantCopy ( &(dprms.rgvarg[iIdx]), &varArg ) );
					}	// if

				// Next parameter
				pItP->next();
				}	// while

			// Clean up
			_RELEASE(pItP);
			}	// if

		// Property put ?
		if (hr == S_OK && iMethod == DISPATCH_PROPERTYPUT)
			{
			// This is necessary for the 'DISPATCH_PROPERTYPUT' invocation
			dprms.rgdispidNamedArgs = &idPut;
			dprms.cNamedArgs			= 1;
			}	// if

		// Execute
		CCLTRY ( pDisp->Invoke ( id, IID_NULL, LOCALE_USER_DEFAULT, iMethod,
											&dprms, &varRes, &ei, &ep ) );

		// Exception ?
		if (hr == DISP_E_EXCEPTION)
			{
			lprintf ( LOG_ERR, L"Exception:%s:%s (hr 0x%x Scode 0x%x)\r\n", 
							ei.bstrSource, ei.bstrDescription, hr, ei.scode );
			hr = ei.scode;
			}	// if

		// Result
		CCLOK ( vRes = varRes; )

		// Clean up
		for (U32 i = 0;i < dprms.cArgs;++i)
			VariantClear ( &(dprms.rgvarg[i]) );
		_FREEMEM(dprms.rgvarg);
		}	// if

	// Clean up
	_RELEASE(pParam);
	_RELEASE(pFunc);

	// Debug
	if (hr != S_OK)
		lprintf ( LOG_ERR, L"%s failed:0x%x", pwName, hr );

	return hr;
	}	// invoke

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

