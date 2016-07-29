// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "nElement.h"
#include "nCamera.generated.h"

UCLASS()
class NSPACE_API UnCamera :
	public UnElement
	{
	GENERATED_BODY()

	public:
	UnCamera();
	virtual ~UnCamera();

	// Run-time data
	bool		bTx,bTy,bTz;								// Need to translate
	FVector	fTrans;										// Translation

	// Utilities
	virtual bool	mainTick		( float );
	virtual bool	onReceive	( nElement *, const WCHAR *, const WCHAR *, const ADTVALUE & );

	};

