////////////////////////////////////////////////////////////////////
//
//									nActor.cpp
//
//				Implementation of the main nSpace actor object
//
////////////////////////////////////////////////////////////////////

#include "nSpace.h"
#include "nActor.h"
#include "nElement.h"

AnActor::AnActor()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pCli			= NULL;
	pDctRen		= NULL;

	// String references
	strRefActor = L"Actor";
//	strRenLoc	= L"/State/Interface/Dict/RenderList/Dict/";
	strRenLoc	= L"/State/Render/3D/State/Default/";

	// Worker thread
	pThrd			= NULL;
	pTick			= NULL;
	pMnQ			= NULL;
	pMnIt			= NULL;
	pWrkQ			= NULL;
	pWrkIt		= NULL;
	pStQ			= NULL;
	pStIt			= NULL;
	evWork.init();

	// Set this actor to call Tick() every frame.  
	// You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	}	// AnActor

HRESULT AnActor :: addMain ( nElement *pElem )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Add element to main queue.
	//
	//	PARAMETERS
	//		-	pElem is the element.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// State check
	if (!bWork || pMnQ == NULL)
		return S_OK;

	return pMnQ->write ( adtIUnknown(pElem) );
	}	// addMain

HRESULT AnActor :: addStore ( const WCHAR *pwLoc, const ADTVALUE &v, 
													const WCHAR *pwLocRen )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Add element to store queue.
	//
	//	PARAMETERS
	//		-	pwLoc is the store location
	//		-	v is the store value
	//		-	pwLocRen is the root render location
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	strLoc(pwLoc);

	// State check
	if (!bWork || pStQ == NULL)
		return S_OK;

	// State check
	CCLTRYE ( strLoc.length() > 0, E_INVALIDARG );

	// Prepend the render location for relative locations
	if (hr == S_OK && *pwLoc != '/' && pwLocRen != NULL)
		hr = strLoc.prepend ( pwLocRen );

	// Add to queue and signal thread
	dbgprintf ( L"AnActor::addStore:%s\r\n", (LPCWSTR)strLoc );
	CCLTRY ( pStQ->write ( strLoc ) );
	CCLTRY ( pStQ->write ( v ) );
	CCLOK  ( evWork.signal(); )

	return hr;
	}	// addStore

HRESULT AnActor :: addWork ( nElement *pElem )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Add element to worker queue.
	//
	//	PARAMETERS
	//		-	pElem is the element.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// State check
	if (!bWork || pWrkQ == NULL)
		return S_OK;

	// Add to queue and signal thread
	CCLTRY ( pWrkQ->write ( adtIUnknown(pElem) ) );
	CCLOK  ( evWork.signal(); )

	return hr;
	}	// addWork

void AnActor::BeginPlay()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the game starts or when spawned.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr = S_OK;

	// Debug
	UE_LOG(LogTemp, Warning, TEXT("AnActor::BeginPlay"));

	// Default behaviour
	Super::BeginPlay();

	// Create worker object
	CCLTRYE ( (pTick = new AnActort(this)) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pTick->AddRef(); )
	CCLTRY  ( pTick->construct() );

	// Create worker thread, no need to wait for startup
	CCLOK (bWork = true;)
	CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
	CCLTRY(pThrd->threadStart ( pTick, 0 ));

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
	if (pThrd != NULL)
		{
		pThrd->threadStop(30000);
		_RELEASE(pThrd);
		}	// if

	// Base behaviour
	Super::EndPlay(rsn);
	}	// EndPlay

void AnActor::Tick( float DeltaTime )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called every frame
	//
	//	PARAMETERS
	//		-	DeltaTime is amount of elapased time.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Base behaviour
	Super::Tick( DeltaTime );

	// Execute any scheduled work
	if (hr == S_OK && pMnQ != NULL && pMnQ->isEmpty() != S_OK)
		{
		U32			sz	= 0;
		adtValue		vV;

		// One pass through
		CCLTRY ( pMnQ->size ( &sz ) );
		for (U32 i = 0;i < sz;++i)
			{
			// Object for ticking
			CCLTRY ( pMnIt->read ( vV ) );

			// Tick work
			if (hr == S_OK)
				{
				// Requeue if still needs more work
				if (((nElement *)(vV.punk))->mainTick(DeltaTime))
					pMnQ->write ( vV );
				}	// if

			// Move to next vlaue
			pMnIt->next();
			}	// for

		}	// if

	}	// Tick

//
// AnActort
//

AnActort :: AnActort ( AnActor *_pThis )
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
	pThis	= _pThis;
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
//	return pThis->onValue ( pwRoot, pwLoc, v );
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
	CCLTRYE ( pThis->evWork.wait ( -1 ), ERROR_TIMEOUT );

	// Execute scheduled work	
	while (	hr == S_OK		&& 
				pThis->bWork && 
				pThis->pWrkQ->isEmpty() != S_OK)
		{
		adtValue		vV;

		// Object for ticking
		CCLTRY ( pThis->pWrkIt->read ( vV ) );

		// Tick work
		CCLOK ( ((nElement *)(vV.punk))->workTick(); )

		// Move to next vlaue
		pThis->pWrkIt->next();
		}	// while

	// Execute scheduled stores
	while (	hr == S_OK		&& 
				pThis->bWork && 
				pThis->pStQ->isEmpty() != S_OK)
		{
		adtValue		vLoc,vV;

		// Load location and value
		if (	pThis->pStIt->read ( vLoc ) == S_OK &&
				adtValue::type(vLoc) == VTYPE_STR )
			{
			// Load value
			pThis->pStIt->next();
			if (pThis->pStIt->read ( vV ) == S_OK )
				pThis->pCli->store ( vLoc.pstr, vV );
			}	// if

		// Move to next vlaue
		pThis->pStIt->next();
		}	// while

	// Keep running ?
	CCLTRYE ( pThis->bWork == true, S_FALSE );
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
//	pThis->bWork = false;
//	pThis->evWork.signal();
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
//	nElement	*pElem	= NULL;

	// Initialize COM for thread
	CCLTRYE ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) == S_OK,
					GetLastError() );
/*
	// Create dictionary for render locations
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pThis->pDctRen ) );

	//
	// Work queues
	//

	// Create queue for scheduling work from main game loop
	CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pThis->pMnQ ) );
	CCLTRY ( pThis->pMnQ->iterate ( &pThis->pMnIt ) );

	// Create queue and event for scheduling work from worker thread
	CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pThis->pWrkQ ) );
	CCLTRY ( pThis->pWrkQ->iterate ( &pThis->pWrkIt ) );

	// Create queue for scheduling stores from worker thread
	CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pThis->pStQ ) );
	CCLTRY ( pThis->pStQ->iterate ( &pThis->pStIt ) );
*/
	//
	// Client
	//

	// Create client
	CCLTRYE((pThis->pCli = new nSpaceClient()) != NULL, E_OUTOFMEMORY);

	// Open private namespace with own command line
	CCLTRY(pThis->pCli->open(L"{ Namespace Unreal }", false, NULL));

	// Listen to the default render locations which contains desired visuals to be rendered
//	CCLTRY ( pThis->pCli->listen ( pThis->strRenLoc, true, this ) );

	// Debug
//	if (hr != S_OK)
//		MessageBox ( NULL, L"AnActorLoc::tickBegin", L"Error!", MB_OK );

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
	if (pThis->pCli != NULL)
		pThis->pCli->listen ( pThis->strRenLoc, false );

	// Shutdown root element render locations
	if (pThis->pDctRen != NULL && pThis->pDctRen->iterate ( &pIt ) == S_OK)
		{
		adtValue		vN;

		// Clear render locations
		while (pIt->read ( vN ) == S_OK)
			{
			IDictionary		*pDct		= NULL;
			nElement		*pElem	= NULL;
			adtIUnknown		unkV;

			// Obtain descriptor and element
			if (	(IUnknown *)(NULL) != (unkV=vN)							&&
					_QI(unkV,IID_IDictionary,&pDct) == S_OK				&&
					pDct->load ( adtString(L"Element"), vN ) == S_OK	&&
					(IUnknown *)(NULL) != (unkV=vN)							&&
					(pElem	= (nElement *)(unkV.punk))->pRenLoc != NULL)
				pElem->pRenLoc->setRoot(NULL,L"");

			// Next entry
			_RELEASE(pDct);
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);
		}	// if

	// Clear queues
	if (pThis->pMnQ != NULL)
		pThis->pMnQ->clear();
	if (pThis->pWrkQ != NULL)
		pThis->pWrkQ->clear();
	if (pThis->pStQ != NULL)
		pThis->pStQ->clear();

	// Clean up
	_RELEASE(pThis->pMnIt);
	_RELEASE(pThis->pMnQ);
	_RELEASE(pThis->pStIt);
	_RELEASE(pThis->pStQ);
	_RELEASE(pThis->pWrkIt);
	_RELEASE(pThis->pWrkQ);

	// Clean up
	_RELEASE(pThis->pDctRen);
	_RELEASE(pThis->pImgC);
*/	if (pThis->pCli != NULL)
		{
		// Close link
		pThis->pCli->close();

		// Clean up
		delete pThis->pCli;
		pThis->pCli	= NULL;
		}	// if

	// Clean up
	CoUninitialize();

	return hr;
	}	// tickEnd

