// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "nElement.h"
#include "nShape.generated.h"

/**
 * 
 */
UCLASS()
class NSPACE_API UnShape : public UnElement
	{
	GENERATED_BODY()
	
	public:	

	// Sets default values for this actor's properties
	UnShape();

	// Run-time data
	adtString						strName;				// Shape name
	UStaticMeshComponent			*pcShp;
//	UStaticMesh						*pMesh;
	UMaterialInterface			*pMat;
	UMaterialInstanceDynamic	*pMatDyn;

	// Base class memebers
	virtual void InitializeComponent		( void ) override;
	virtual void UninitializeComponent	( void ) override;

	// Utilities
	virtual bool	mainTick		( float );
	virtual bool	onReceive	( nElement *, const WCHAR *, const WCHAR *, const ADTVALUE &);

	};
