// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "nElement.h"
#include "nGroup.generated.h"

/**
 * 
 */
UCLASS()
class NSPACE_API UnGroup : public UnElement
{
	GENERATED_BODY()

	public:

	UnGroup();

	// Run-time data
	UBoxComponent	*pcBox;

	// Base class memebers
	virtual void InitializeComponent ( void ) override;

};
