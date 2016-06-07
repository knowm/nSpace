////////////////////////////////////////////////////////////////////
//
//									nActor.cpp
//
//				Implementation of the main nSpace actor object
//
////////////////////////////////////////////////////////////////////

#include "nSpace.h"
#include "nActor.h"

AnActor::AnActor()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////

	// Set this actor to call Tick() every frame.  
	// You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	}	// AnActor

void AnActor::BeginPlay()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the game starts or when spawned.
	//
	////////////////////////////////////////////////////////////////////////

	// Debug
	UE_LOG(LogTemp, Warning, TEXT("AnActor::BeginPlay"));

	// Default behaviour
	Super::BeginPlay();

	}	// BeginPlay

void AnActor::EndPlay(const EEndPlayReason::Type rsn )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when play ends for this actor.
	//
	////////////////////////////////////////////////////////////////////////

	// Debug
	UE_LOG(LogTemp, Warning, TEXT("AnActor::EndPlay"));

	// Shutdown worker thread
//	if (pThrd != NULL)
//		{
//		pThrd->threadStop(30000);
//		_RELEASE(pThrd);
//		}	// if

	// Base behaviour
	Super::EndPlay(rsn);
	}	// EndPlay

//
// AnActort
//

AnActort :: AnActort ( AnActor *_pR )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pR is the parent object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	pR	= _pR;
	}	// AnActort

HRESULT AnActort :: onReceive (	const WCHAR *pwRoot, 
											const WCHAR *pwLoc,
											const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		nSpaceClientCB
	//
	//	PURPOSE
	//		-	Called when a listened location receives a value.
	//
	//	PARAMETERS
	//		-	pwRoot is the path to the listened location
	//		-	pwLoc is the location relative to the root for the value
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
//	return pR->onValue ( pwRoot, pwLoc, v );
	return S_OK;
	}	// onReceive

HRESULT AnActort :: tick ( void )
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
	HRESULT	hr	= S_OK;
/*
	// Wait for work
	CCLTRYE ( pR->evWork.wait ( -1 ), ERROR_TIMEOUT );

	// Execute scheduled work	
	while (	hr == S_OK		&& 
				pR->bWork && 
				pR->pWrkQ->isEmpty() != S_OK)
		{
		adtValue		vV;

		// Object for ticking
		CCLTRY ( pR->pWrkIt->read ( vV ) );

		// Tick work
		CCLOK ( ((nSpcElement *)(vV.punk))->workTick(); )

		// Move to next vlaue
		pR->pWrkIt->next();
		}	// while

	// Execute scheduled stores
	while (	hr == S_OK		&& 
				pR->bWork && 
				pR->pStQ->isEmpty() != S_OK)
		{
		adtValue		vLoc,vV;

		// Load location and value
		if (	pR->pStIt->read ( vLoc ) == S_OK &&
				adtValue::type(vLoc) == VTYPE_STR )
			{
			// Load value
			pR->pStIt->next();
			if (pR->pStIt->read ( vV ) == S_OK )
				pR->pCli->store ( vLoc.pstr, vV );
			}	// if

		// Move to next vlaue
		pR->pStIt->next();
		}	// while

	// Keep running ?
	CCLTRYE ( pR->bWork == true, S_FALSE );
*/
	return hr;
	}	// tick

HRESULT AnActort :: tickAbort ( void )
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
//	pR->bWork = false;
//	pR->evWork.signal();
	return S_OK;
	}	// tickAbort

HRESULT AnActort :: tickBegin ( void )
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
	HRESULT		hr			= S_OK;
//	nSpcElement	*pElem	= NULL;

	// Initialize COM for thread
	CCLTRYE ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) == S_OK,
					GetLastError() );
/*
	// Create dictionary for render locations
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pR->pDctRen ) );

	//
	// Work queues
	//

	// Create queue for scheduling work from main game loop
	CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pR->pMnQ ) );
	CCLTRY ( pR->pMnQ->iterate ( &pR->pMnIt ) );

	// Create queue and event for scheduling work from worker thread
	CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pR->pWrkQ ) );
	CCLTRY ( pR->pWrkQ->iterate ( &pR->pWrkIt ) );

	// Create queue for scheduling stores from worker thread
	CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pR->pStQ ) );
	CCLTRY ( pR->pStQ->iterate ( &pR->pStIt ) );

	//
	// Client
	//

	// Create client
	CCLTRYE((pR->pCli = new nSpaceClient()) != NULL, E_OUTOFMEMORY);

	// Open private namespace with own command line
	CCLTRY(pR->pCli->open(
				L"{ Execute Batch Location C:/dev/nspace/resource/record/install.nspc }", false, NULL));

	// Construct a shared image cache object
	CCLTRYE ( (pR->pImgC = new nSpcImgCache ( pR )) != NULL, E_OUTOFMEMORY );
	_ADDREF(pR->pImgC);
	CCLTRY ( pR->pImgC->construct() );

	// Listen to the default render locations which contains desired visuals to be rendered
	CCLTRY ( pR->pCli->listen ( pR->strRenLoc, true, this ) );

	// Debug
//	if (hr != S_OK)
//		MessageBox ( NULL, L"AnActorLoc::tickBegin", L"Error!", MB_OK );
*/
	return hr;
	}	// tickBegin

HRESULT AnActort :: tickEnd ( void )
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
	HRESULT	hr		= S_OK;
/*	IIt		*pIt	= NULL;

	// Stop listening to render location
	if (pR->pCli != NULL)
		pR->pCli->listen ( pR->strRenLoc, false );

	// Shutdown root element render locations
	if (pR->pDctRen != NULL && pR->pDctRen->iterate ( &pIt ) == S_OK)
		{
		adtValue		vN;

		// Clear render locations
		while (pIt->read ( vN ) == S_OK)
			{
			IDictionary		*pDct		= NULL;
			nSpcElement		*pElem	= NULL;
			adtIUnknown		unkV;

			// Obtain descriptor and element
			if (	(IUnknown *)(NULL) != (unkV=vN)							&&
					_QI(unkV,IID_IDictionary,&pDct) == S_OK				&&
					pDct->load ( adtString(L"Element"), vN ) == S_OK	&&
					(IUnknown *)(NULL) != (unkV=vN)							&&
					(pElem	= (nSpcElement *)(unkV.punk))->pRenLoc != NULL)
				pElem->pRenLoc->setRoot(NULL,L"");

			// Next entry
			_RELEASE(pDct);
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);
		}	// if

	// Clear queues
	if (pR->pMnQ != NULL)
		pR->pMnQ->clear();
	if (pR->pWrkQ != NULL)
		pR->pWrkQ->clear();
	if (pR->pStQ != NULL)
		pR->pStQ->clear();

	// Clean up
	_RELEASE(pR->pMnIt);
	_RELEASE(pR->pMnQ);
	_RELEASE(pR->pStIt);
	_RELEASE(pR->pStQ);
	_RELEASE(pR->pWrkIt);
	_RELEASE(pR->pWrkQ);

	// Clean up
	_RELEASE(pR->pDctRen);
	_RELEASE(pR->pImgC);
	if (pR->pCli != NULL)
		{
		// Close link
		pR->pCli->close();

		// Clean up
		delete pR->pCli;
		pR->pCli	= NULL;
		}	// if
*/
	// Clean up
	CoUninitialize();

	return hr;
	}	// tickEnd

