// Fill out your copyright notice in the Description page of Project Settings.

#include "nSpace.h"
#include "nPlayerController.h"

AnPlayerController::AnPlayerController(const FObjectInitializer &oi) :
	APlayerController(oi)
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

 	// Set this actor to call Tick() every frame.  
	// You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Input
	bShowMouseCursor			= true;
	bEnableMouseOverEvents	= true;
	bEnableClickEvents		= true;

	// Setup
	pRen			= NULL;
	pElemCap	= NULL;
	bElemCap	= false;
	pDctRy	= NULL;
	pDctBt	= NULL;
	}	// AnPlayerController

void AnPlayerController::BeginPlay()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when play beings for this actor.
	//
	////////////////////////////////////////////////////////////////////////

	// Base beahviour
	Super::BeginPlay();

	// Always spawn nSpace renderer
	pRen = GetWorld()->SpawnActor<AnActor>(
				FVector(0,0,0),FRotator(0,0,0),FActorSpawnParameters());

	// Input dictionaries
	COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctRy );
	COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctBt );


//	APlayerCameraManager
//	*pMgr = this->PlayerCameraManager;
//	dbgprintf ( L"AnPlayerController::BeginPlay:Manager %p\r\n", pMgr );

	}	// BeginPlay

void AnPlayerController::EndPlay(const EEndPlayReason::Type rsn )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when play ends for this actor.
	//
	////////////////////////////////////////////////////////////////////////

	// Base behaviour
	Super::EndPlay(rsn);

	// Clean up
	pRen		= NULL;
	pElemCap	= NULL;
	bElemCap	= false;
	_RELEASE(pDctRy);
	_RELEASE(pDctBt);
	}	// EndPlay

bool AnPlayerController :: InputKey ( FKey Key, EInputEvent EventType, 
														float AmountDepressed, bool bGamePad)
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process input key
	//
	//	OVERLOAD FROM
	//		APlayerController
	//
	////////////////////////////////////////////////////////////////////////
/*
	// Debug
	dbgprintf ( L"AnPlayerController::InputKey:%s:%s:%d:%g:%d\r\n",
					*(Key.GetDisplayName().ToString()),
					*(Key.GetFName().ToString()), EventType, AmountDepressed, bGamePad );

	// Currently supporting down then up (not repeat)
	if (EventType == EInputEvent::IE_Pressed || EventType == EInputEvent::IE_Released)
		{
		FHitResult hr;

		// Component under point ?
		dbgprintf ( L"Pressed/Released\r\n" );
		if (GetHitResultUnderCursor ( ECollisionChannel::ECC_Visibility, false, hr ))
			{
			UnElement	*pElem;
			FVector			vAt,vDir;
			dbgprintf ( L"Hit!\r\n" );

			// Intersecting component
			UPrimitiveComponent *
			pcAt = hr.GetComponent();

			// Check if parent is an nSpace element
			pElem = Cast<UnElement> ( pcAt->GetAttachParent() );
			USceneComponent *pComp = pcAt->GetAttachParent();
			if (pcAt != NULL)
				dbgprintf ( L"pcAt %p pComp %p\r\n", pcAt, pComp );

			//
			// Send button down value to element
			//
			if (pElem != NULL)
				{
				HRESULT		hr		= S_OK;
				IDictionary	*pDct	= NULL;
				adtString	strLocEv,strName;

				// Generate location to button value
				CCLTRY ( adtValue::copy ( pElem->pParent->strLoc, strLocEv ) );
				CCLTRY ( strLocEv.append ( L"Element/Input/Button/Fire/Value" ) );

				// 'Display' name seems to be more appropriate, 'FName' produces 'four' instead of '4'
				CCLOK ( strName = *(Key.GetDisplayName().ToString()); )

				// Generate button event for element
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
				CCLTRY ( pDct->store ( adtString(L"Name"), strName ) );
				CCLTRY ( pDct->store ( adtString(L"State"), 
								adtString ( (EventType == EInputEvent::IE_Pressed) ? L"Down" : L"Up" ) ) );

				// Send
				CCLTRY ( pElem->pParent->pRenLoc->addStore ( strLocEv, adtIUnknown(pDct) ) );

				// Clean up
				_RELEASE(pDct);
				}	// if (pElemCap != NULL)

			}	// if (GetHitResultUnderCursor ( ECollisionChannel::ECC_Visibility, false, hr))

		}	// if
*/
	// Default behaviour
	return APlayerController::InputKey ( Key, EventType, AmountDepressed, bGamePad );
	}	// InputKey

void AnPlayerController :: onButton (	UnElement *pElem, 
														const WCHAR *wName,
														const WCHAR *wState )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called for a button event.
	//
	//	PARAMETERS
	//		-	pElem is the target element
	//		-	wName is the button name
	//		-	wState is the button state
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	strLoc;

	// Full location of element
	CCLTRY ( adtValue::copy ( pElem->pParent->pRenLoc->strLocRen, strLoc ) );
	CCLTRY ( strLoc.append ( pElem->pParent->strLoc ) );
	CCLTRY ( pDctBt->store ( adtString(L"Location"), strLoc ) );

	// Parent (group) location
	UnElement *pParent = 
			(pElem->pParent != NULL && pElem->pParent->pRoot != NULL) ?
			Cast<UnElement> ( pElem->pParent->pRoot->AttachParent ) : NULL;
	if (hr == S_OK && pParent != NULL)
		{
		// Full location of element
		CCLTRY ( adtValue::copy ( pParent->pParent->pRenLoc->strLocRen, strLoc ) );
		CCLTRY ( strLoc.append ( pParent->pParent->strLoc ) );
		CCLTRY ( pDctBt->store ( adtString(L"LocationParent"), strLoc ) );
		}	// if

	// Generate button event for element
	CCLTRY ( pDctBt->store ( adtString(L"Name"), adtString(wName) ) );
	CCLTRY ( pDctBt->store ( adtString(L"State"), adtString(wState) ) );

	// Generate location to button input
	CCLTRY ( adtValue::copy ( pRen->strRenLoc, strLoc ) );
	CCLTRY ( strLoc.append ( L"Input/Button/Fire/Value" ) );

	// Send
	CCLTRY ( pRen->addStore ( strLoc, adtIUnknown(pDctBt) ) );
	}	// onButton

void AnPlayerController::onClick( UnElement *pElem )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a component is clicked.
	//
	//	PARAMETERS
	//		-	pElem is the element component
	//
	////////////////////////////////////////////////////////////////////////
/*
	// Assign element.
	pElemClk = pElem;
	bElemClk = true;

	// Send event to element
	dbgprintf ( L"AnPlayerController::onClick:pElem %p\r\n", pElem );
*/
	}	// onClick

void AnPlayerController :: onRay  ( UnElement *pElem, 
													const FVector &vLoc, 
													const FVector &vDir )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called for a ray event.
	//
	//	PARAMETERS
	//		-	pElem is the target element
	//		-	vLoc is the ray intersection location
	//		-	vDir is the ray direction
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	strLoc;

	// Full location of element
	CCLTRY ( adtValue::copy ( pElem->pParent->pRenLoc->strLocRen, strLoc ) );
	CCLTRY ( strLoc.append ( pElem->pParent->strLoc ) );
	CCLTRY ( pDctRy->store ( adtString(L"Location"), strLoc ) );

	// Parent (group) location
	UnElement *pParent = 
			(pElem->pParent != NULL && pElem->pParent->pRoot != NULL) ?
			Cast<UnElement> ( pElem->pParent->pRoot->AttachParent ) : NULL;
	if (hr == S_OK && pParent != NULL)
		{
		// Full location of element
		CCLTRY ( adtValue::copy ( pParent->pParent->pRenLoc->strLocRen, strLoc ) );
		CCLTRY ( strLoc.append ( pParent->pParent->strLoc ) );
		CCLTRY ( pDctBt->store ( adtString(L"LocationParent"), strLoc ) );
		}	// if

	// Coordinates for ray
	CCLTRY ( pDctRy->store ( adtString(L"X"), adtDouble(vLoc.X) ) );
	CCLTRY ( pDctRy->store ( adtString(L"Y"), adtDouble(vLoc.Y) ) );
	CCLTRY ( pDctRy->store ( adtString(L"Z"), adtDouble(vLoc.Z) ) );
	CCLTRY ( pDctRy->store ( adtString(L"Xn"), adtDouble(vDir.X) ) );
	CCLTRY ( pDctRy->store ( adtString(L"Yn"), adtDouble(vDir.Y) ) );
	CCLTRY ( pDctRy->store ( adtString(L"Zn"), adtDouble(vDir.Z) ) );

	// Generate location to ray input
	CCLTRY ( adtValue::copy ( pRen->strRenLoc, strLoc ) );
	CCLTRY ( strLoc.append ( L"Input/Ray/Fire/Value" ) );

	// Send
	CCLTRY ( pRen->addStore ( strLoc, adtIUnknown(pDctRy) ) );
	}	// onRay

void AnPlayerController :: SetupInputComponent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Setup custom input bindings.
	//
	//	OVERLOAD FROM
	//		APlayerController
	//
	////////////////////////////////////////////////////////////////////////

	// Key bindings

	}	// SetupInputComponent

void AnPlayerController::Tick( float DeltaTime )
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

	// Base behaviour
	APlayerController::Tick(DeltaTime);

	// Debug
	if (true)
		{
//		FVector vAt,vDr;

		// Convert the mouse coordinates to world coordinates
//		DeprojectMousePositionToWorld ( vAt, vDr );

		// Debug
//		dbgprintf ( L"Mouse @ (%g,%g,%g) (%g,%g,%g)\r\n", 
//						vAt.X, vAt.Y, vAt.Z, vDr.X, vDr.Y, vDr.Z );
		}	// if

	// Debug
	if (IsInputKeyDown ( EKeys::Equals ))
		{
/*
APawn *pawn = this->GetPawn();
pawn->SetActorLocation(FVector(0,0,0),true);
*/
/*
		// Camera manager
		APlayerCameraManager 
		*pMgr = PlayerCameraManager;
		if (pMgr != NULL)
			{
			// Root component of camera actor
			USceneComponent *
			pRootCam = pMgr->GetRootComponent();
			if (pRootCam != NULL)
				{
//				pRootCam->SetWorldLocation(FVector(0,0,0), true );
//				pRootCam->SetRelativeLocation(FVector(0,0,0), true );
//pRootCam->Mov
//this->Get
				}	// if (pRootCam != NULL)
			}	// if (pMgr != NULL)
	
//		FHitResult r;
//		this->SetActorLocation ( FVector(0,0,0), true, &r );
*/
		}	// if

	// Element capture
	if (	pElemCap == NULL	&& 
			!bElemCap			&&
			IsInputKeyDown ( EKeys::LeftMouseButton ) )
		{
		FHitResult hr;

		// Capture enabled
		bElemCap = true;

		// Component under point ?
		if (GetHitResultUnderCursor ( ECollisionChannel::ECC_Visibility, false, hr))
			{
			FVector vAt,vDir;

			// Intersecting component
			UPrimitiveComponent *
			pcAt = hr.GetComponent();

			// Check if parent is an nSpace element
			pElemCap = Cast<UnElement> ( pcAt->GetAttachParent() );

			// To handle two levels deep
			if (pElemCap == NULL && pcAt->GetAttachParent() != NULL)
				pElemCap = Cast<UnElement> ( pcAt->GetAttachParent()->GetAttachParent() );

			// Initial capture point in world coordinates
			vElemAt	= hr.Location;
			vElemDr	= hr.Normal;

			// Convert the mouse coordinates to world coordinates
			DeprojectMousePositionToWorld ( vAt, vDir );

			// Distance from mouse to intersection location
			fElemCap = FVector::Dist ( vAt, hr.Location );
			dbgprintf ( L"Distance to object %g (%p, %p)\r\n", 
							fElemCap, pcAt, pElemCap );

			//
			// Compute input ray
			//
			if (pElemCap != NULL)
				{
				// Compute input location
				tElemCap	= pElemCap->GetComponentToWorld();
//				tE			= pcAt->GetComponentToWorld();
				vElemAt	= tElemCap.InverseTransformPosition ( vElemAt );
				vElemDr	= tElemCap.InverseTransformVector ( vElemDr );
				vElemDr.Normalize();

				// Debug
				dbgprintf ( L"Intersection @ (%g,%g,%g) (%g,%g,%g)\r\n", 
								vElemAt.X, vElemAt.Y, vElemAt.Z, vElemDr.X, vElemDr.Y, vElemDr.Z );
				}	// if

			//
			// Send input events for valid element
			//
			if (pElemCap != NULL)
				{
				// Ray/button input
				onRay ( pElemCap, vElemAt, vElemDr );
				onButton ( pElemCap, L"Button1", L"Down" );

				// Set ray
//				pElemCap->onRay ( pDctRy, vElemAt, vElemDr );

				// Button event
//				pElemCap->onButton ( pDctBt, L"Button1", L"Down" );
				}	// if (pElemCap != NULL)

			}	// if (GetHitResultUnderCursor ( ECollisionChannel::ECC_Visibility, false, hr))

		}	// if (pElemCap == NULL && !bElemCap && IsInputKeyDown ( EKeys::LeftMouseButton ))

	// Input is currently captured
	else if (bElemCap)
		{
		// Movement during capture ?
		if (pElemCap != NULL)
			{
			FVector		vFrom,vTo,vToDr;
			FVector		vAt,vDir,vDiff;

			// Obtain the current mouse position
			DeprojectMousePositionToWorld ( vAt, vDir );

			// Extend ray from current position to drag length
			vDir.Normalize();
			vFrom = vAt + fElemCap * vDir;

			// Compute the new location of the component using its frame of reference
			vTo	= tElemCap.InverseTransformPosition ( vFrom );
			vToDr	= tElemCap.InverseTransformVector ( vDir );
			vToDr.Normalize();

			// Change ?
			if (vElemAt != vTo)
				{
				dbgprintf ( L"@ (%g,%g,%g) (%g,%g,%g)\r\n", 
								vTo.X, vTo.Y, vTo.Z, vToDr.X, vToDr.Y, vToDr.Z );

				// New position
				vElemAt = vTo;

				// Ray input
				onRay ( pElemCap, vTo, vToDr );

				// Set ray
//				pElemCap->onRay ( pDctRy, vTo, vToDr );
				}	// if

			}	// if

		// Check for input
		if (!IsInputKeyDown ( EKeys::LeftMouseButton ))
			{
			// Button input
			if (pElemCap != NULL)
				onButton ( pElemCap, L"Button1", L"Up" );
//				pElemCap->onButton ( pDctBt, L"Button1", L"Up" );

			// Capture over
			bElemCap	= false;
			pElemCap	= NULL;
			}	// if

		}	// else if

	}	// Tick


/*
		{
		dbgprintf ( L"Hit component:%p @ Location (%g,%g,%g)\r\n", 
						r.GetComponent(), r.Location.X, r.Location.Y, r.Location.Z );

			tE		= pComp->GetComponentToWorld();
			vAtE	= tE.InverseTransformPosition ( vAt );

			// Debug
			dbgprintf ( L"C - World (%g,%g,%g) : Element (%g,%g,%g)\r\n", 
								vAt.X, vAt.Y, vAt.Z,
								vAtE.X, vAtE.Y, vAtE.Z );
		}	// if

	// Testing
	FVector vF = GetFocalLocation();
*/
/*
	// For a 'clicked' element, perform visual movement.
	if (pElemClk != NULL)
		{
		HRESULT			hr		= S_OK;
		IDictionary		*pDct = NULL;
		adtString		strLocEv;
		FVector			vAt,vDir,vAtE,vAtEi;
		FTransform		tE;
		float				dX,dY,dZ;

		// Retrieve the current world position of the cursor in component coordinate system
		DeprojectMousePositionToWorld ( vAt, vDir );

		if (pElemClk->GetAttachParent() != NULL)
			{
			USceneComponent *pComp =
			pElemClk->GetAttachParent();

			tE		= pComp->GetComponentToWorld();
			vAtE	= tE.InverseTransformPosition ( vAt );

			// Debug
			dbgprintf ( L"C - World (%g,%g,%g) : Element (%g,%g,%g)\r\n", 
								vAt.X, vAt.Y, vAt.Z,
								vAtE.X, vAtE.Y, vAtE.Z );
			}	// if

		if (pElemClk->GetAttachParent()->GetAttachParent() != NULL)
			{
			USceneComponent *pComp =
			pElemClk->GetAttachParent()->GetAttachParent();

			tE		= pComp->GetComponentToWorld();
			vAtE	= tE.InverseTransformPosition ( vAt );

			// Debug
			dbgprintf ( L"P - World (%g,%g,%g) : Element (%g,%g,%g)\r\n", 
								vAt.X, vAt.Y, vAt.Z,
								vAtE.X, vAtE.Y, vAtE.Z );
			}	// if

//		tE		= pElemClk->GetComponentTransform();
		tE		= pElemClk->GetComponentToWorld();
		vAtE	= tE.InverseTransformPosition ( vAt );

		// Debug
		dbgprintf ( L"E - World (%g,%g,%g) : Element (%g,%g,%g)\r\n", 
							vAt.X, vAt.Y, vAt.Z,
							vAtE.X, vAtE.Y, vAtE.Z );

		// First time processing click ?
		if (bElemClk)
			{
			// Generate a 'down' button event for initial click

			// Generate location to button value
			CCLTRY ( adtValue::copy ( pElemClk->pParent->strLoc, strLocEv ) );
			CCLTRY ( strLocEv.append ( L"Element/Input/Button/Fire/Value" ) );

			// Generate button event for element
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
			CCLTRY ( pDct->store ( adtString(L"Name"), adtString(L"Button1") ) );
			CCLTRY ( pDct->store ( adtString(L"State"), adtString(L"Down") ) );

			// Schedule store
			CCLTRY ( pElemClk->pParent->pRenLoc->addStore ( strLocEv, adtIUnknown(pDct) ) );

			// Clean up
			_RELEASE(pDct);

			// Remember where cursor was on first click
			vElemClk = vAtE;

			// First time has been processed
			bElemClk = false;
			}	// if

		// Process additional movement
		else
			{
			// Difference between starting point and current point
			dX = vAtE.X-vElemClk.X;
			dY = vAtE.Y-vElemClk.Y;
			dZ = vAtE.Z-vElemClk.Z;

			// Enough to count as movement ?
			if (fabs(dX) > 0.01 || fabs(dY) > 0.01 || fabs(dZ) > 0.01)
				{
				FVector vNow;

				// Retrieve the element's current relative translation
				vNow = pElemClk->GetRelativeTransform().GetTranslation();
				dbgprintf ( L"vNow (%g,%g,%g)\r\n", vNow.X, vNow.Y, vNow.Z );
				if (pElemClk->GetAttachParent() != NULL)
					{
					vNow = pElemClk->GetAttachParent()->GetRelativeTransform().GetTranslation();
					dbgprintf ( L"vNow (%g,%g,%g)\r\n", vNow.X, vNow.Y, vNow.Z );
					}	// if

				// Send signal for new translation parameters
				dbgprintf ( L"(%g,%g,%g) : (%g,%g,%g) -> (%g,%g,%g)\r\n", 
							dX, dY, dZ,		
							vNow.X, vNow.Y, vNow.Z,
							(vNow.X+dX), (vNow.Y+dY), (vNow.Z+dZ) );

				}	// if
			}	// else if

		// Has button been released ?
		if (!IsInputKeyDown ( EKeys::LeftMouseButton ))
			{
			// Generate an 'up' button event for release

			// Generate location to button value
			CCLTRY ( adtValue::copy ( pElemClk->pParent->strLoc, strLocEv ) );
			CCLTRY ( strLocEv.append ( L"Element/Input/Button/Fire/Value" ) );

			// Generate button event for element
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
			CCLTRY ( pDct->store ( adtString(L"Name"), adtString(L"Button1") ) );
			CCLTRY ( pDct->store ( adtString(L"State"), adtString(L"Up") ) );

			// Schedule store
			CCLTRY ( pElemClk->pParent->pRenLoc->addStore ( strLocEv, adtIUnknown(pDct) ) );

			// Clean up
			_RELEASE(pDct);

			// Click over
			pElemClk = NULL;
			bElemClk = false;
			}	// if

		}	// if
*/
/*
	// Input testing
	FHitResult r;
	if (GetHitResultUnderCursor ( ECollisionChannel::ECC_Visibility, false, r))
		{
		dbgprintf ( L"Hit component:%p @ Location (%g,%g,%g)\r\n", 
						r.GetComponent(), r.Location.X, r.Location.Y, r.Location.Z );

			tE		= pComp->GetComponentToWorld();
			vAtE	= tE.InverseTransformPosition ( vAt );

			// Debug
			dbgprintf ( L"C - World (%g,%g,%g) : Element (%g,%g,%g)\r\n", 
								vAt.X, vAt.Y, vAt.Z,
								vAtE.X, vAtE.Y, vAtE.Z );
		}	// if

	// Testing
	FVector vF = GetFocalLocation();
	dbgprintf ( L"Focus:(%g,%g,%g)\r\n", vF.X, vF.Y, vF.Z );
	if (IsInputKeyDown ( EKeys::LeftMouseButton ))
		dbgprintf ( L"LeftMouseDown!\r\n" );
*/
/*
			// Currently this visual 'mover' only works with the group that 'own' the components
			// so that grouped components stay together.
			if (	
					// This is the 'nElement' parent 
//					pElemCap->GetAttachParent() != NULL &&	

					// Group parent
					(pGrp = Cast<UnSpcGrp> ( pElemCap->GetAttachParent() )) != NULL
				)
				{
				HRESULT		hr = S_OK;
				FVector		vLoc;
				adtString	strLoc,strLocc;

				// Update translation for component
				tD		= pGrp->GetRelativeTransform();
				vLoc	= tD.GetTranslation();
				vLoc.X += vTo.X;
				vLoc.Y += vTo.Y;
				vLoc.Z += vTo.Z;
//				tD.SetTranslation ( vLoc );
//				pGrp->SetRelativeTransform ( tD );

				// Generate location to group translations
				CCLTRY ( adtValue::copy ( pGrp->pParent->strLoc, strLoc ) );
				CCLTRY ( strLoc.append ( L"Element/Transform/Translate/" ) );

				// Each component
				for (int i = 0;i < 3;++i)
					{
					// Location of component
					CCLTRY ( adtValue::copy ( strLoc, strLocc ) );
					CCLTRY ( strLocc.append (	(i == 0) ? L"A1" :
														(i == 1) ? L"A2" : L"A3" ) );
					CCLTRY ( strLocc.append ( L"/OnFire/Value" ) );

					// Send
					CCLTRY ( pElemCap->pParent->pRenLoc->addStore ( strLocc, 
								adtDouble ( (i == 0) ? vLoc.X : (i == 1) ? vLoc.Y : vLoc.Z ) ) );
					}	// for

				}	// if
			*/
