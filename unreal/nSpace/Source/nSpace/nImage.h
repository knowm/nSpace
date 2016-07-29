// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "nElement.h"
#include "nImage.generated.h"

/**
 * 
 */
UCLASS()
class NSPACE_API UnImage : public UnElement
	{
	GENERATED_BODY()

	public:
	UnImage();											// Constructor

	// Run-time data
	adtString	strTag;									// Image tag
	UTexture2D	*pTex;									// Active texture

	UStaticMeshComponent			*pcShp;
	UStaticMesh						*pMesh;
	UMaterialInterface			*pMat;
	UMaterialInstanceDynamic	*pMatDyn;

	// Base class memebers
	virtual void InitializeComponent ( void ) override;

	// Utilities
	virtual bool	mainTick		( float );
	virtual bool	onReceive	( nElement *, const WCHAR *, const WCHAR *, const ADTVALUE &);

	// Internal utilities
	void onImage ( IDictionary * );

	};
