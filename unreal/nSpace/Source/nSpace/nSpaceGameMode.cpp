// Fill out your copyright notice in the Description page of Project Settings.

#include "nSpace.h"
#include "nSpaceGameMode.h"
#include "nPlayerController.h"

AnSpaceGameMode::AnSpaceGameMode(const FObjectInitializer &oi) :
	AGameMode(oi)
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	oi is a reference to the object initializer
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Own player controller
	PlayerControllerClass = AnPlayerController::StaticClass();
	}	// AnSpaceRender



