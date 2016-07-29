// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"

// nSpace engine
#include "nSpace.h"

// nSpace actor
#include "nActor.h"

#include "nLoc.generated.h"

// Forward decs.
class nLocCB;
class nElement;

UCLASS()
class NSPACE_API AnLoc : public AActor
	{
	GENERATED_BODY()
	
	public:	
	AnLoc();													// Constructor
	virtual ~AnLoc();										// Destructor

	// Run-time data
	AnActor		*pRen;									// Renderer
	adtString	strLocRen;								// Render location
	IDictionary	*pDctRen;								// Render dictionary
	adtString	strRefActor;							// References
	nLocCB		*pCB;										// Callback

	// Utilities
	HRESULT addMain	( nElement * );
	HRESULT addWork	( nElement * );
	HRESULT addStore	( const WCHAR *, const ADTVALUE & );
	HRESULT getParent	( const WCHAR *, nElement ** );
	HRESULT setRoot	( nElement *, const WCHAR * );
 
	// Called when the game starts or when spawned
	virtual void BeginPlay	() override;
	virtual void EndPlay		( const EEndPlayReason::Type ) override;
	
	// Internal utilities
	HRESULT onValue	( const WCHAR *, const WCHAR *, const ADTVALUE & );
	};

//
// Class - nLocCB.  Callback object.
//

class NSPACE_API nLocCB : 
	public CCLObject,										// Base class
	public nSpaceClientCB								// Callback function
	{
	public :
	nLocCB ( AnLoc * );									// Constructor

	// Run-time datas
	AnLoc	*pParent;										// Parent object

	// 'nSpaceClientCB' members
	STDMETHOD(onReceive)	(const WCHAR *, const WCHAR *, const ADTVALUE &);

	// CCL
	CCL_OBJECT_BEGIN_INT(nLocCB)
	CCL_OBJECT_END()
	};
