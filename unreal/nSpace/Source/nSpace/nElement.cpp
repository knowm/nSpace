// Fill out your copyright notice in the Description page of Project Settings.

#include "nSpace.h"
#include "nElement.h"
#include "nLabel.h"
#include "nShape.h"
#include "nGroup.h"
#include "nImage.h"
//#include "nSpcInSrc.h"
#include "nCamera.h"
#include "nPlayerController.h"

// Radians <-> degrees
#define	RAD_TO_DEG(a)		(a)*(180.0/3.14159265358979323846)
#define	DEG_TO_RAD(a)		(a)*(3.14159265358979323846/180.0)

nElement :: nElement (	AnActor *_pRen,
								const WCHAR *pwLoc, int iIdx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for a root element object.
	//
	//	PARAMETERS
	//		-	_pRen is the master render object
	//		-	pwLoc is the relative location of this element
	//		-	iIdx is the index of the root element in master render list
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pRen			= _pRen;
	pRenLoc		= NULL;
	strLoc		= pwLoc; strLoc.at();					// Own string
	strDef		= L"State/Visual/Group/";
	bRun			= false;
	iRoot			= iIdx;
	iState		= ELEM_STATE_INIT;
	pRoot			= NULL;
	}	// nElement

nElement :: nElement (	const WCHAR *pwLoc, const WCHAR *pwDef,
								AnLoc *_pRenLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pRenLoc is the render location object
	//		-	pwLoc is the relative location of this element
	//		-	pwDef is the definition of the graph at location
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pRenLoc		= _pRenLoc;
	if (pRenLoc != NULL)
		pRen		= pRenLoc->pRen;
	strLoc		= pwLoc; strLoc.at();					// Own string
	strDef		= pwDef;	strDef.at();					// Own string
	bRun			= false;
	iRoot			= 0;
	iState		= ELEM_STATE_INIT;
	pRoot			= NULL;
	}	// nElement

HRESULT nElement :: construct ( void )
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
	HRESULT	hr	= S_OK;

	// Running
	bRun = true;

	// Schedule work for creation of scene component
	CCLTRY ( pRen->addMain ( this ) );

	return hr;
	}	// construct

void nElement :: destruct ( void )
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

	// Clean up
	if (pRenLoc != NULL && strLstn.length() > 0)
		pRenLoc->pRen->pCli->listen ( strLstn, false );

	}	// destruct

bool nElement :: mainTick ( float fD )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Execute work for main game thread.
	//
	//	PARAMETERS
	//		-	fD is the amount of elapsed time since last game loop tick.
	//
	//	RETURN VALUE
	//		true if work is still needed
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	bool		bWrk	= false;

	// Element state
	switch (iState)
		{
		// Initialize element
		case ELEM_STATE_INIT :
			{
			nElement *pParent	= NULL;
			UnShape	*pShape	= NULL;

			// A root element will not have a render location yet
			if (hr == S_OK && iRoot > 0 && pRenLoc == NULL)
				{
				// New render location object
				CCLTRYE ( (pRenLoc = pRen->GetWorld()->SpawnActor<AnLoc>(
							FVector(0,0,0),FRotator(0,0,0),FActorSpawnParameters())) != NULL,
							E_OUTOFMEMORY );

				// Render object
				CCLOK ( pRenLoc->pRen = pRen; )
				}	// if

			// Retrieve parent element if present
			else if (hr == S_OK && strLoc.length() > 0)
				hr = pRenLoc->getParent ( strLoc, &pParent );

			// Create an actor component that handles Unreal side of the visual.
			if	(!WCASECMP(strDef,L"State/Visual/Group/")) 
				{
				// Currently a group is just a place holder for transformation
				// information to be applied to all children so create base
				// level component.
	//			CCLTRYE ( (pRoot = NewObject<UnElement>(UnElement::StaticClass(),pRenLoc))
	//							!= NULL, E_UNEXPECTED );
				CCLTRYE ( (pRoot = NewObject<UnGroup>(pRenLoc,UnGroup::StaticClass()))
								!= NULL, E_UNEXPECTED );
				}	// if

			// Label
			else if	(!WCASECMP(strDef,L"State/Visual/Label/"))
				{
				// Label component
				CCLTRYE ( (pRoot = NewObject<UnLabel>(pRenLoc,UnLabel::StaticClass()))
								!= NULL, E_UNEXPECTED );
				}	// else if

			// Shape
			else if	(	!WCASECMP(strDef,L"State/Visual/Shape/") )
				{
				CCLTRYE ( (pShape = NewObject<UnShape>(pRenLoc,UnShape::StaticClass()))
								!= NULL, E_UNEXPECTED );
				CCLOK   ( pRoot = pShape; )
				}	// else if

			// Camera
			else if	(	!WCASECMP(strDef,L"State/Visual/Camera/") )
				{
				CCLTRYE ( (pRoot = NewObject<UnCamera>(pRenLoc,UnCamera::StaticClass()))
								!= NULL, E_UNEXPECTED );
				}	// else if

			// Input source
//			else if	(	!WCASECMP(strDef,L"State/Visual/Input/Source/") )
//				{
//				CCLTRYE ( (pRoot = NewObject<UnSpcInSrc>(pRenLoc,UnSpcInSrc::StaticClass()))
//								!= NULL, E_UNEXPECTED );
//				}	// else if

			// Image
			else if	(!WCASECMP(strDef,L"State/Visual/Image/"))
				{
				// Image component
				CCLTRYE ( (pRoot = NewObject<UnImage>(pRenLoc,UnImage::StaticClass()))
								!= NULL, E_UNEXPECTED );
				}	// else if

			// If a root was created, prepare it
			if (hr == S_OK && pRoot != NULL)
				{
				// Attached to component in parent
	//			dbgprintf ( L"pC %p --> pParent %p\r\n", pRoot, (pParent != NULL) ? pParent->pRoot : NULL );
				if (pParent != NULL)
					{
					// Tell object about owner
					pRoot->pParent = this;

					// Attached to root of parent
					pRoot->AttachTo(pParent->pRoot);
					}	// if
				else if (pRenLoc->GetRootComponent() == NULL)
					{
					bool		bCamera	= false;
					FVector	fTrans(0,0,0);
					FVector	fScl(25,25,25);
//					FVector	fScl(1,1,1);

					// Compute the scaling factor required to enusre the current view port
					// is mapped to the nSpace unit cube.
//					UGameViewportClient
//					*pCli = pRen->GetWorld()->GetGameViewport();
//					FIntPoint
//					pt		= pCli->Viewport->GetSizeXY();
//					FVector
//					fScl	( pt.Y, pt.Y, pt.Y );

					// As a root element, attempt to retrieve default translations for the group
					if (iRoot > 0 && pRen != NULL && pRen->pDctRen != NULL)
						{
						HRESULT			hr		= S_OK;
						IDictionary		*pDct	= NULL;
						adtValue			vL;
						adtIUnknown		unkV;

						// Obtain descriptor
						CCLTRY(pRen->pDctRen->load ( adtInt(iRoot), vL ) );
						CCLTRY(_QISAFE((unkV=vL),IID_IDictionary,&pDct));

						// Default translations.
						// Take into account the rotated coordinate system below
						if (hr == S_OK && pDct->load ( adtString(L"X"), vL ) == S_OK)
							fTrans.Y = fScl.Y*adtDouble(vL);
						if (hr == S_OK && pDct->load ( adtString(L"Y"), vL ) == S_OK)
							fTrans.Z = fScl.Z*adtDouble(vL);
						if (hr == S_OK && pDct->load ( adtString(L"Z"), vL ) == S_OK)
							fTrans.X = fScl.X*adtDouble(vL);

						// Render location type
						if (hr == S_OK && pDct->load ( adtString(L"Type"), vL ) == S_OK)
							bCamera = !WCASECMP(adtString(vL),L"Camera");

						// Clean up
						_RELEASE(pDct);
						}	// if

					// Use as top level component
					pRenLoc->SetRootComponent ( pRoot );

					// Global transform
					pRoot->SetRelativeTransform ( FTransform (
						// Rotate axis so the nSpace default of XY plane facing user matches what
						// Unreal (and its input) seems to prefer : +X away, +Y right, +Z up
						FRotator (0,90,-90),

						// Translation
						fTrans,

						// Set appropriate scaling from nSpace to Unreal engine
						// This maps the default nSpace 'unit squares' to a usable scaling in Unreal.
						fScl
						) );

					// Is the render location flagged to be part of the camera hierarchy ?
					if (hr == S_OK && bCamera)
						{
						// This root component needs to be attached to the root component of the camera

						// Current player controller
						APlayerController
						*pCtl = pRen->GetWorld()->GetFirstPlayerController();
						if (pCtl != NULL)
							{
							// Camera manager
							APlayerCameraManager 
							*pMgr = pCtl->PlayerCameraManager;
							if (pMgr != NULL)
								{
								// Root component of camera actor
								USceneComponent *
								pRootCam = pMgr->GetRootComponent();
								if (pRootCam != NULL)
									{
//									FTransform
//									ft = pRootCam->GetRelativeTransform();

									// Attach this component to root of camera
									pRoot->AttachTo ( pRootCam );
									}	// if (pRootCam != NULL)
								}	// if (pMgr != NULL)
							}	// if (pCtl != NULL)
						}	// if

					}	// else if

				// Finish component registration
				pRoot->RegisterComponent();
				pRoot->SetVisibility(true,true);
				}	// if

			// Schedule worker thread for listening to remote location
			if (hr == S_OK && strLoc.length() > 0)
				{
				iState = ELEM_STATE_LISTEN;
				hr = pRenLoc->addWork ( this );
				}	// if

			// Error ?
			if (hr != S_OK)
				iState = ELEM_STATE_ERROR;
			break;
			}	// case ELEM_STATE_INIT

		// Running
		case ELEM_STATE_RUN :
			// Component ticking
			if (pRoot != NULL)
				bWrk = pRoot->mainTick(fD);

			// Stil running ?
			if (!bRun)
				{
				// Remove root component from environment
				if (pRoot != NULL)
					{
					pRoot->UnregisterComponent();
					pRoot = NULL;
					}	// if
				}	// if
			break;
		}	// switch

	return bWrk;
	}	// mainTick

HRESULT nElement :: onReceive (	const WCHAR *pwRoot, 
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
	bool			bSch	= false;
//	adtString	strDbg;

	// Debug
//	adtValue::toString(v,strDbg);
//	dbgprintf ( L"nElement::onReceive:%s:%s:%s:%d\r\n",
//			pwRoot, pwLoc, (LPCWSTR)strDbg, v.vtype );

	// Forward to root component
	if (pRoot != NULL)
		bSch = pRoot->onReceive ( this, pwRoot, pwLoc, v );

	// Schedule game loop work if requested
	if (bSch)
		pRenLoc->addMain ( this );

	return S_OK;
	}	// onReceive

bool nElement :: workTick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Execute work for worker thread.
	//
	//	RETURN VALUE
	//		true if element needs more work
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Element state
	switch (iState)
		{
		// Listening to remote element needs to be setup
		case ELEM_STATE_LISTEN :

			// A root element needs to set the render location state
			if (iRoot > 0)
				{
				// Initialize render location, this will trigger a 'listen'
				// at the render location.
				if (pRenLoc != NULL)
					hr = pRenLoc->setRoot ( this, strLoc );
				}	// if

			else
				{
				// Generate path to visual minus the _Location field
				CCLTRY ( adtValue::copy ( pRenLoc->strLocRen, strLstn ) );
				CCLTRY ( strLstn.append ( strLoc ) );
		//		CCLTRY ( strLstn.at(strLstn.length()-9) = '\0'; )

				// Request listen
				dbgprintf ( L"nElement::workTick:Listen %s\r\n", (LPCWSTR)strLstn );
				CCLTRY ( pRen->pCli->listen ( strLstn, true, this ) );
				}	// else

			// Result
			iState = (hr == S_OK) ? ELEM_STATE_RUN : ELEM_STATE_ERROR;
			pRenLoc->addMain ( this );
			if (hr != S_OK)
				strLstn = L"";
			break;
		}	// switch

	return false;
	}	// workTick

//
// UnElement
//

UnElement :: UnElement ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pParent is the parent object.
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pParent	= NULL;
	fTi.Set(0,0,0);
	fTf.Set(0,0,0);
	fTt.Set(0,0,0);
	fS.Set(0,0,0);
	fRotNow.Set(0,0,0);
	fSclLcl.Set(1,1,1);
	iVisible = -1;
	bRot[0] = bRot[1] = bRot[2] = false;

	// In case of sub-component initialization
	bWantsInitializeComponent = true;

	}	// UnElement

void UnElement :: InitializeComponent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Starts gameplay for this component
	//
	////////////////////////////////////////////////////////////////////////

	// Base behaviour
	USceneComponent::InitializeComponent();

	// Mouse input
//	OnBeginCursorOver.AddDynamic(this,&UnElement::OnOverBegin);
//	OnEndCursorOver.AddDynamic(this,&UnElement::OnOverEnd);
//	OnClicked.AddDynamic(this,&UnElement::OnClicked);
//	OnReleased.AddDynamic(this,&UnElement::OnReleased);

	}	// InitializeComponent

void UnElement :: inputAdd ( UPrimitiveComponent *pC )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Add component to input handling list
	//
	//	PARAMETERS
	//		-	pC is the component
	//
	////////////////////////////////////////////////////////////////////////

	// Delgates for receing input
	pC->OnBeginCursorOver.AddDynamic(this,&UnElement::OnOverBegin);
	pC->OnEndCursorOver.AddDynamic(this,&UnElement::OnOverEnd);
	pC->OnClicked.AddDynamic(this,&UnElement::OnClicked);
	pC->OnReleased.AddDynamic(this,&UnElement::OnReleased);

	}	// inputAdd

bool UnElement :: mainTick ( float fD )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Execute work for main game thread.
	//
	//	PARAMETERS
	//		-	fD is the amount of elapsed time since last game loop tick.
	//
	//	RETURN VALUE
	//		true if work is still needed
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	bool		bWrk = false;

	// Translate
	if (fTt.X > 0 || fTt.Y > 0 || fTt.Z > 0)
		{
		// Current transform
		FTransform	fX		= GetRelativeTransform();
		FVector		fNow	= fX.GetTranslation();

		// Debug.  'Snap to' location.
//		fD = 10;

		// Initial 'from' location
		if (fTt.X == SZ_TIME_MOVE)
			fTi.X = fNow.X;
		if (fTt.Y == SZ_TIME_MOVE)
			fTi.Y = fNow.Y;
		if (fTt.Z == SZ_TIME_MOVE)
			fTi.Z = fNow.Z;

		// Debug
//		if (fTf.X != 0 || fTf.Y != 0 || fTf.Z != 0)
//			dbgprintf ( L"(%g,%g,%g) -> (%g,%g,%g)\r\n", 
//							fTi.X, fTi.Y, fTi.Z, fTf.X, fTf.Y, fTf.Z );
//		if (fTf.X != 0 || fTf.Y != 0 || fTf.Z != 0)
//			dbgprintf ( L"Hi\r\n" );

		// Move component into position over given time.
		if (fTt.X > 0)
			{
			fTt.X		= (fTt.X < fD) ? 0 : (fTt.X-fD);
			fNow.X	= ((fTi.X-fTf.X)*(fTt.X/SZ_TIME_MOVE))+fTf.X;
			}	// if
		if (fTt.Y > 0)
			{
			fTt.Y		= (fTt.Y < fD) ? 0 : (fTt.Y-fD);
			fNow.Y	= ((fTi.Y-fTf.Y)*(fTt.Y/SZ_TIME_MOVE))+fTf.Y;
			}	// if
		if (fTt.Z > 0)
			{
			fTt.Z		= (fTt.Z < fD) ? 0 : (fTt.Z-fD);
			fNow.Z	= ((fTi.Z-fTf.Z)*(fTt.Z/SZ_TIME_MOVE))+fTf.Z;
			}	// if

		// Need work again ?
		bWrk = (fTt.X > 0 || fTt.Y > 0 || fTt.Z > 0);

//		if (!bWrk)
//			dbgprintf ( L"Hi\r\n" );
		// Update transform
//		fTt.Set(0,0,0);
//		fNow.Set(fTf.X,fTf.Y,fTf.Z);
//		FVector	fS = fX.GetScale3D();
//		if (!bWrk)
//			dbgprintf ( L"%p (%g,%g,%g) (%g,%g,%g)\r\n", this, 
//							fNow.X, fNow.Y, fNow.Z, fS.X, fS.Y, fS.Z );
		fX.SetTranslation ( fNow );
		SetRelativeTransform(fX);
		}	// if

	// Rotation
	if (bRot[0] || bRot[1] || bRot[2])
		{
		FTransform	t;

		// Set new rotation values
		if (bRot[0])	fRotNow.X = fR.X;
		if (bRot[1])	fRotNow.Y = fR.Y;
		if (bRot[2])	fRotNow.Z = fR.Z;

		// Current transform
		t = GetRelativeTransform();

		// Set rotation
		t.SetRotation(FQuat::MakeFromEuler(fRotNow));

		// New transform
		SetRelativeTransform ( t );

		// Done
		fR.Set(0,0,0);
		bRot[0] = bRot[1] = bRot[2] = false;
		}	// if

	// Scale
	if (fS.X != 0 || fS.Y != 0 || fS.Z != 0)
		{
		// Current transform
		FTransform	fX		= GetRelativeTransform();
		FVector		fScl	= fX.GetScale3D();

		// Update scale
		if (fS.X != 0)
			fScl.X = fS.X;
		if (fS.Y != 0)
			fScl.Y = fS.Y;
		if (fS.Z != 0)
			fScl.Z = fS.Z;

		// Update transform
		fX.SetScale3D ( fScl );
		SetRelativeTransform ( fX );

		// Done
		fS.Set(0,0,0);
		}	// if

	// Visibility
	if (iVisible != -1)
		{
		// Change ?
		if (	(iVisible == 1 && !bVisible) ||
				(iVisible == 0 && bVisible) )
			{
			dbgprintf ( L"UnElement::mainTick:Visible %d\r\n", iVisible );

			// Set new visible state
			SetVisibility ( (iVisible == 1) ? true : false, true );
			SetActive(bVisible);
			}	// if

		// Updated
		iVisible = -1;
		}	// if

	return bWrk;
	}	// mainTick

void UnElement :: onButton ( IDictionary *pDct, const WCHAR *wName,
											const WCHAR *wState )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called for a button event.
	//
	//	PARAMETERS
	//		-	pDct contains and will receive event information
	//		-	wName is the button name
	//		-	wState is the button state
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	strLocEv;

	// Generate button event for element
	CCLTRY ( pDct->store ( adtString(L"Name"), adtString(wName) ) );
	CCLTRY ( pDct->store ( adtString(L"State"), adtString(wState) ) );

	// Generate location to button value
	CCLTRY ( adtValue::copy ( pParent->strLoc, strLocEv ) );
	CCLTRY ( strLocEv.append ( L"Element/Input/Button/Fire/Value" ) );

	// Send
	CCLTRY ( pParent->pRenLoc->addStore ( strLocEv, adtIUnknown(pDct) ) );
	}	// onButton

void UnElement :: OnClicked ( UPrimitiveComponent *pComponent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Function delegate callback for left mouse button click event.
	//
	//	PARAMETERS
	//		-	pComponent is the component
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT						hr		= S_OK;
//	AnSpcPlayerController	*pCtl	= NULL;

	// Setup
	dbgprintf ( L"UnElement::OnClicked:%s:%p\r\n", (LPCWSTR)pParent->strLoc,
					pComponent );

	// Notify player controller of click.
//	pCtl = Cast<AnSpcPlayerController> (GetWorld()->GetFirstPlayerController());
//	if (pCtl != NULL)
//		pCtl->onClick ( this );
/*
	IDictionary	*pDct	= NULL;
	adtString	strLocEv;


	// Generate location to button value
	CCLTRY ( adtValue::copy ( pParent->strLoc, strLocEv ) );
	CCLTRY ( strLocEv.append ( L"Element/Input/Button/Fire/Value" ) );

	// Generate button event for element
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
	CCLTRY ( pDct->store ( adtString(L"Name"), adtString(L"Button1") ) );
	CCLTRY ( pDct->store ( adtString(L"State"), adtString(L"Down") ) );

	// Schedule store
	CCLTRY ( pParent->pRenLoc->addStore ( strLocEv, adtIUnknown(pDct) ) );

	// Clean up
	_RELEASE(pDct);
*/
	}	// OnClicked

void UnElement :: OnOverBegin ( UPrimitiveComponent *pComponent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Function delegate callback for begin mouse over event.
	//
	//	PARAMETERS
	//		-	pComponent is the component
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"UnElement::OnOverBegin:%p\r\n", pComponent );
	}	// OnOverBegin

void UnElement :: OnOverEnd ( UPrimitiveComponent *pComponent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Function delegate callback for end mouse over event.
	//
	//	PARAMETERS
	//		-	pComponent is the component
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"UnElement::OnOverEnd:%p\r\n", pComponent );
	}	// OnOverEnd

void UnElement :: OnReleased ( UPrimitiveComponent *pComponent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Function delegate callback for left mouse button release event.
	//
	//	PARAMETERS
	//		-	pComponent is the component
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pDct	= NULL;
	adtString	strLocEv;

	// Setup
//	dbgprintf ( L"UnElement::OnReleased:%s:%p\r\n", (LPCWSTR)pParent->strLoc,
//					pComponent );
/*
	// Generate location to button value
	CCLTRY ( adtValue::copy ( pParent->strLoc, strLocEv ) );
	CCLTRY ( strLocEv.append ( L"Element/Input/Button/Fire/Value" ) );

	// Generate button event for element
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
	CCLTRY ( pDct->store ( adtString(L"Name"), adtString(L"Button1") ) );
	CCLTRY ( pDct->store ( adtString(L"State"), adtString(L"Up") ) );

	// Schedule store
	CCLTRY ( pParent->pRenLoc->addStore ( strLocEv, adtIUnknown(pDct) ) );

	// Clean up
	_RELEASE(pDct);
*/
	}	// OnReleased

void UnElement :: onRay ( IDictionary *pDct, const FVector &vLoc,
										const FVector &vDir )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called for a ray event.
	//
	//	PARAMETERS
	//		-	pDct contains and will receive event information
	//		-	vLoc is the ray intersection location
	//		-	vDir is the ray direction
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	strLocEv;

	// Coordinates for ray
	CCLTRY ( pDct->store ( adtString(L"X"), adtDouble(vLoc.X) ) );
	CCLTRY ( pDct->store ( adtString(L"Y"), adtDouble(vLoc.Y) ) );
	CCLTRY ( pDct->store ( adtString(L"Z"), adtDouble(vLoc.Z) ) );
	CCLTRY ( pDct->store ( adtString(L"Xn"), adtDouble(vDir.X) ) );
	CCLTRY ( pDct->store ( adtString(L"Yn"), adtDouble(vDir.Y) ) );
	CCLTRY ( pDct->store ( adtString(L"Zn"), adtDouble(vDir.Z) ) );

	// Generate location to button value
	CCLTRY ( adtValue::copy ( pParent->strLoc, strLocEv ) );
	CCLTRY ( strLocEv.append ( L"Element/Input/Ray/Fire/Value" ) );

	// Send
	CCLTRY ( pParent->pRenLoc->addStore ( strLocEv, adtIUnknown(pDct) ) );
	}	// onRay

bool UnElement :: onReceive (	nElement *pElem,
											const WCHAR *pwRoot, 
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
	//		-	pElem is the nSpace element
	//		-	pwRoot is the path to the listened location
	//		-	pwLoc is the location relative to the root for the value
	//		-	v is the value
	//
	//	RETURN VALUE
	//		true if there is main game loop to be scheduled.
	//
	////////////////////////////////////////////////////////////////////////
	bool	bA1	= false;
	bool	bA2	= false;
	bool	bA3	= false;
	bool	bSch	= false;

	// Debug
//	if (adtValue::empty(v))
//		{
//		dbgprintf ( L"UnElement::onReceive:Empty!:%s:%s:%s\r\n", 
//						(LPCWSTR)pParent->strLoc, pwRoot, pwLoc );
//		}	// if

	//
	// Handle paths common to all elements
	//

	// Translation
	if (	(bA1 = !WCASECMP(pwLoc,L"Element/Transform/Translate/A1/OnFire/Value")) == true ||
			(bA2 = !WCASECMP(pwLoc,L"Element/Transform/Translate/A2/OnFire/Value")) == true ||
			(bA3 = !WCASECMP(pwLoc,L"Element/Transform/Translate/A3/OnFire/Value")) == true )
			{
			adtDouble	dV(v);

			// Debug
//			dbgprintf ( L"UnElement::onReceive:Translate:%s:%s:%s:%g\r\n", 
//							(LPCWSTR)pParent->strLoc, pwRoot, pwLoc, (double)dV );

			// Set component
			if			(bA1)
				{
				fTf.X	= dV;//*10;
				fTt.X	= SZ_TIME_MOVE;
				}	// if
			else if	(bA2)
				{
				fTf.Y	= dV;//*10;
				fTt.Y	= SZ_TIME_MOVE;
				}	// else if
			else				
				{
				fTf.Z	= dV;//*10;
				fTt.Z	= SZ_TIME_MOVE;
				}	// else

			// Schedule work on game thread to perform update
			bSch = true;
			}	// if

	// Scale
	else if ((bA1 = !WCASECMP(pwLoc,L"Element/Transform/Scale/A1/OnFire/Value")) == true ||
				(bA2 = !WCASECMP(pwLoc,L"Element/Transform/Scale/A2/OnFire/Value")) == true ||
				(bA3 = !WCASECMP(pwLoc,L"Element/Transform/Scale/A3/OnFire/Value")) == true )
			{
			adtDouble	dV(v);

//			if (dV != 1)
//				dbgprintf ( L"Hi\r\n" );

			// Set component
			if			(bA1)
				fS.X	= (dV != 0.0) ? (double)dV : 1.0;
			else if	(bA2)
				fS.Y	= (dV != 0.0) ? (double)dV : 1.0;
			else				
				fS.Z	= (dV != 0.0) ? (double)dV : 1.0;

			// Debug
//			if (fS.X == 0 || fS.Y == 0 || fS.Z == 0)
//				dbgprintf ( L"fS (%g,%g,%g)\r\n", fS.X, fS.Y, fS.Z );

			// Schedule work on game thread to perform update
			bSch = true;
			}	// if

	// Rotation
	else if ((bA1 = !WCASECMP(pwLoc,L"Element/Transform/Rotate/A1/OnFire/Value")) == true ||
				(bA2 = !WCASECMP(pwLoc,L"Element/Transform/Rotate/A2/OnFire/Value")) == true ||
				(bA3 = !WCASECMP(pwLoc,L"Element/Transform/Rotate/A3/OnFire/Value")) == true )
			{
			adtDouble	dV(v);

			// Debug
//			if (dV != 1)
//				dbgprintf ( L"Hi\r\n" );

			// Set component
			if			(bA1)
				{
				fR.X		= dV;
				bRot[0]	= true;
				}	// if
			else if	(bA2)
				{
				fR.Y		= dV;
				bRot[1]	= true;
				}	// if
			else				
				{
				fR.Z		= dV;
				bRot[2]	= true;
				}	// if

			// Schedule work on game thread to perform update
			bSch = true;
			}	// if

	// Visible
	else if (!WCASECMP(pwLoc,L"Element/Visible/OnFire/Value"))
		{
		// Notify of new state
//		dbgprintf ( L"Visible %d\r\n", v.vbool );
		iVisible = (adtBool(v) == true) ? 1 : 0;
		bSch = true;
		}	// else if

	// Color
	else if (!WCASECMP(pwLoc,L"Element/Color/OnFire/Value"))
		{
		// Notify of new state
		iColor	= adtInt(v);
		bColor	= true;
		bSch		= true;
		}	// else if

	return bSch;
	}	// onReceive

void UnElement :: UninitializeComponent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Ends gameplay for this component
	//
	////////////////////////////////////////////////////////////////////////

	// This component
//	if (pParent != NULL)
//		pParent->pRoot = NULL;
//	DetachFromParent();

	// Base behaviour
	USceneComponent::UninitializeComponent();
	}	// UninitializeComponent
