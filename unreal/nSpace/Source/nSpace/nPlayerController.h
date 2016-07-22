// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/PlayerController.h"
#include "nActor.h"
#include "nElement.h"
#include "nGroup.h"
#include "nPlayerController.generated.h"

/**
 * 
 */
UCLASS()
class NSPACE_API AnPlayerController : public APlayerController
	{
	GENERATED_BODY()

	AnPlayerController ( const FObjectInitializer & );

	public :

	// Run-time data
	AnActor			*pRen;								// Renderer
	UnElement		*pElemCap;							// Captured input element
	bool				bElemCap;							// Caputre enabled
	float				fElemCap;							// Caputre distance
	FTransform		tElemCap;							// Initial capture transform
	FVector			vElemAt,vElemDr;					// Latest location/direction
	IDictionary		*pDctRy,*pDctBt;					// Input dictionaries

	// Utilities
	virtual void onButton( UnElement *, const WCHAR *, const WCHAR * );
	virtual void onClick ( UnElement * );
	virtual void onRay	( UnElement *, const FVector &, const FVector & );

	// 'APlayerController' members
	virtual void BeginPlay				()											override;
	virtual void EndPlay					( const EEndPlayReason::Type )	override;
	virtual bool InputKey				( FKey, EInputEvent, float,
													bool )								override;
	virtual void SetupInputComponent	( void )									override;
	virtual void Tick						( float )								override;
	};
