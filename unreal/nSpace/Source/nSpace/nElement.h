#pragma once

#include "GameFramework/Actor.h"
#include "nLoc.h"
#include "nElement.generated.h"

// Definitions
#define	SZ_TIME_MOVE			1.0					// Seconds to move

// Element initiallization states due to work being done in
// different threads
#define	ELEM_STATE_ERROR			-1					// Element in error state
#define	ELEM_STATE_INIT			0					// Initialize element
#define	ELEM_STATE_LISTEN			1					// Initialize element
#define	ELEM_STATE_RUN				2					// Element running

// Forward decs.
class AnActor;

//
// Class - nElement.  Base class for nSpace visual elements.
//

class UnElement;
class NSPACE_API nElement : 
	public CCLObject,										// Base class
	public nSpaceClientCB								// Callback function
	{
	public:	
	nElement ( const WCHAR *, const WCHAR *,		// Constructor
					AnLoc * );
	nElement ( AnActor *,								// Constructor
					const WCHAR *, int );

	// Run-time data
	AnActor		*pRen;									// Master render object
	AnLoc			*pRenLoc;								// Render location
	UnElement	*pRoot;									// Root scene component for this element
	adtString	strLoc,strDef;							// Namespace location and definition
	adtString	strLstn;									// Listen location
	bool			bRun;										// Element running
	int			iRoot;									// Index for root elements
	int			iState;									// Element state

	// Utilities
	bool mainTick	( float );							// Main thread ticking
	bool workTick	( void );							// Worker thread ticking

	// 'nSpaceClientCB' members
	STDMETHOD(onReceive)	( const WCHAR *, const WCHAR *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN_INT(nElement)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object
	
	};

//
// Class - UnElement.  Unreal scene component for nSpace visual element.
//

UCLASS()
class NSPACE_API UnElement : 
	public USceneComponent
	{
	GENERATED_BODY()
	
	public:	

	// Sets default values for this actor's properties
	UnElement ();											// Constructor

	// Run-time data
	nElement	*pParent;									// Parent object
	FVector	fTi,fTf,fS,fR;								// Target transformations
	FVector	fTt;											// Time for translation
	FVector	fSclLcl;										// Local scaling (if needed)
	FVector	fRotNow;										// Current rotation
	bool		bRot[3];										// Rotation update
	int		iVisible;									// Visible update
	int		iColor;										// 32-bit color
	bool		bColor;										// Color update required

	// Base class memebers
	virtual void	InitializeComponent		( void ) override;
	virtual void	UninitializeComponent	( void ) override;

	// Utilities
	virtual void	inputAdd		( UPrimitiveComponent * );
	virtual bool	mainTick		( float );
	virtual bool	onReceive	( nElement *, const WCHAR *, const WCHAR *, const ADTVALUE & );

	// Custom events
	virtual void	onButton		( IDictionary *, const WCHAR *, const WCHAR * );
	virtual void	onRay			( IDictionary *, const FVector &, const FVector & );

	// Events
	UFUNCTION()
	void OnOverBegin	( UPrimitiveComponent *TouchedComponent );
	UFUNCTION()
	void OnOverEnd		( UPrimitiveComponent *TouchedComponent );
	UFUNCTION()
	void OnClicked		( UPrimitiveComponent *TouchedComponent );
	UFUNCTION()
	void OnReleased	( UPrimitiveComponent *TouchedComponent );
	};
