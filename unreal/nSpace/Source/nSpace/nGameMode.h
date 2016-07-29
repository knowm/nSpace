// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/GameMode.h"
#include "nGameMode.generated.h"

//
// Class - AnGameMode.  nSpace specific game mode object.  Originally
//		needed to create own player controller object on startup.  The use of
//		this object is specified in the Unreal editor : 
//		Editor|Project Settings|Maps & Modes|Default Game Mode|nSpaceGameMode
//

UCLASS()
class NSPACE_API AnGameMode : public AGameMode
	{
	GENERATED_BODY()

	AnGameMode (const FObjectInitializer & );

	// 'AGameMode' memebers
//	virtual void StartPlay() override;
	
	};
