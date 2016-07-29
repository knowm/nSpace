////////////////////////////////////////////////////////////////////
//
//									nActor.h
//
//				Include file for the nSpace actor
//
////////////////////////////////////////////////////////////////////

#pragma once

// Unreal
#include "GameFramework/Actor.h"

// nSpace engine
#include "nSpace.h"

#include "nActor.generated.h"

// Forward decs.
class AnActort;
class nElement;

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
	AnActor();												// Constructor
	virtual ~AnActor();									// Destructor

	// Run-time data
	nSpaceClient		*pCli;							// nSpace client
	IDictionary			*pDctRen;						// Render location dictionary
	adtString			strRefActor;					// References
	adtString			strRenLoc;						// Render state location
	sysCS					csRen;							// Global render mutex

	// Worker thread
	AnActort				*pTick;							// Worker thread
	IThread				*pThrd;							// Worker thread
	IList					*pWrkQ;							// Worker queue
	IIt					*pWrkIt;							// Worker queue iterator
	sysEvent				evWork;							// Worker thread event
	bool					bWork;							// True to keep working
	IList					*pStQ;							// Store queue
	IIt					*pStIt;							// Store queue iterator

	// Game loop thread
	IList					*pMnQ;							// Main queue
	IIt					*pMnIt;							// Main queue iterator

	// Utilities
	HRESULT addMain	( nElement * );
	HRESULT addWork	( nElement * );
	HRESULT addStore	( const WCHAR *, const ADTVALUE &, const WCHAR * = NULL );

	// 'AActor' memebers
	virtual void BeginPlay	( ) override;
	virtual void EndPlay		( const EEndPlayReason::Type) override;
	virtual void Tick			( float DeltaSeconds) override;

	// Internal utilities
	HRESULT onValue(const WCHAR *, const WCHAR *, const ADTVALUE &);
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
	AnActor			*pThis;								// Parent object

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
