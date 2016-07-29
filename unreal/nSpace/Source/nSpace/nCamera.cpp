// Fill out your copyright notice in the Description page of Project Settings.

#include "nSpace.h"
#include "nCamera.h"

UnCamera::UnCamera()
	{
	bTx = bTy = bTz = false;
	}

UnCamera::~UnCamera()
{
}

bool UnCamera :: mainTick ( float fD )
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

	// Default behaviour
	bWrk = UnElement::mainTick(fD);

	// Have to handle own translation to update pawn.
	if (bTx || bTy || bTz)
		{
		FVector fLoc;

		// The camera element has been updated, use its world coordinates
		// to update the pawn position
		fLoc = GetComponentLocation();

		// Debug
//		dbgprintf ( L"UnnSpcCamera::mainTick:Update:%g,%g,%g\r\n", fTrans.X, fTrans.Y, fTrans.Z );

		// Active pawn
		APlayerController
		*pCtlr	= GetWorld()->GetFirstPlayerController();
		APawn
		*pPawn	= (pCtlr != NULL) ? pCtlr->GetPawn() : NULL;
		if (pPawn != NULL)
			pPawn->SetActorLocation(fLoc,true);
/*
			{
			// Current location
			fLoc = pPawn->GetActorLocation();

			// Update specifed components
			if (bTx)	fLoc.Y = fTrans.X;
			if (bTy)	fLoc.Z = fTrans.Y;
			if (bTz)	fLoc.X = fTrans.Z;

			// Update location
			pPawn->SetActorLocation(fLoc,true);
			}	// if
*/
		// Updating
		bTx = bTy = bTz = false;
		}	// if

	return bWrk;
	}	// mainTick

bool UnCamera :: onReceive (	nElement *pElem,
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
	bool			bA1	= false;
	bool			bA2	= false;
	bool			bA3	= false;
	bool			bSch	= false;

	// Base behaviour
	bSch = UnElement::onReceive(pElem,pwRoot,pwLoc,v);

	// Translation
	if (	(bA1 = !WCASECMP(pwLoc,L"Element/Transform/Translate/A1/OnFire/Value")) == true ||
			(bA2 = !WCASECMP(pwLoc,L"Element/Transform/Translate/A2/OnFire/Value")) == true ||
			(bA3 = !WCASECMP(pwLoc,L"Element/Transform/Translate/A3/OnFire/Value")) == true )
			{
			adtDouble	dV(v);

			// Set component
			if			(bA1)
				{
				fTrans.X = dV;
				bTx = true;
				}	// if
			else if	(bA2)
				{
				fTrans.Y = dV;
				bTy = true;
				}	// else if
			else				
				{
				fTrans.Z = dV;
				bTz = true;
				}	// else if

			// Schedule work on game thread to perform update
			bSch		= true;
			}	// if

	return bSch;
	}	// onReceive
