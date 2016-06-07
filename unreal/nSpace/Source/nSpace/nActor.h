////////////////////////////////////////////////////////////////////
//
//									nActor.h
//
//			Include file for the nSpace actor
//
////////////////////////////////////////////////////////////////////

#pragma once

// Unreal
#include "GameFramework/Actor.h"
#include "nActor.generated.h"

//
// Class - AnActor.  Primary nSpace actor for game engine.  
//		Handles link to nSpace ActiveX client.
//

UCLASS()
class NSPACE_API AnActor : public AActor
	{
	GENERATED_BODY()
	
	public:	
	// Sets default values for this actor's properties
	AnActor();

	// 'AActor' memebers
	virtual void BeginPlay	( ) override;
	virtual void EndPlay		( const EEndPlayReason::Type) override;
//	virtual void Tick			( float DeltaSeconds) override;

	};

//
// Class - AnActort.  Worker thread class to handle nSpace link
//		and perform work on behalf of the engine.
//

class NSPACE_API AnActort : 
	public CCLObject,										// Base class
	public ITickable,										// Interface
	public nSpaceClientCB								// Callback function
	{
	public :
	AnActort ( AnActor * );								// Constructor

	// Run-time datas
	AnActor			*pR;									// Parent object
	nSpaceClient	*pnCli;								// nSpace client

	// 'nSpaceClientCB' members
	STDMETHOD(onReceive)	( const WCHAR *, const WCHAR *, const ADTVALUE & );

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN_INT(AnActort)
	CCL_OBJECT_END()
	};
