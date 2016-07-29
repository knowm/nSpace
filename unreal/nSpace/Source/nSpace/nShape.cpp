// Fill out your copyright notice in the Description page of Project Settings.

#include "nSpace.h"
#include "nShape.h"

// Globals
static ConstructorHelpers::FObjectFinder<UObject> *FCub	= NULL;
static ConstructorHelpers::FObjectFinder<UObject> *FSph	= NULL;
static ConstructorHelpers::FObjectFinder<UObject> *FCyl	= NULL;
static ConstructorHelpers::FObjectFinder<UObject> *FCon	= NULL;
static ConstructorHelpers::FObjectFinder<UObject> *FMat	= NULL;

UnShape::UnShape()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pcShp		= NULL;
	pMatDyn	= NULL;
	pMat		= NULL;
	bColor	= false;
	iColor	= -1;

	// Globals
	if (FCub == NULL)
		FCub = new ConstructorHelpers::FObjectFinder<UObject> 
					( TEXT("/Engine/BasicShapes/Cube") );
	if (FSph == NULL)
		FSph = new ConstructorHelpers::FObjectFinder<UObject> 
					( TEXT("/Engine/BasicShapes/Sphere") );
	if (FCyl == NULL)
		FCyl = new ConstructorHelpers::FObjectFinder<UObject> 
					( TEXT("/Engine/BasicShapes/Cylinder") );
	if (FCon == NULL)
		FCon = new ConstructorHelpers::FObjectFinder<UObject> 
					( TEXT("/Engine/BasicShapes/Cone") );
	if (FMat == NULL)
		FMat = new ConstructorHelpers::FObjectFinder<UObject> 
					( TEXT("/Engine/BasicShapes/BasicShapeMaterial") );

	}	// UnShape

void UnShape :: InitializeComponent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Starts gameplay for this component
	//
	////////////////////////////////////////////////////////////////////////
	dbgprintf ( L"UnShape::InitializeComponent\r\n" );

	// Base behaviour
	UnElement::InitializeComponent();

	// Create static mesh component to use for shape
	pcShp					= NewObject<UStaticMeshComponent>
								(this,UStaticMeshComponent::StaticClass());
	pcShp->bVisible	= false;
	pcShp->AttachTo(this);

	// Testing
	pcShp->SetMobility(EComponentMobility::Movable);

	// Assign material
	if (FMat->Succeeded() && (pMat = Cast<UMaterialInterface> ( FMat->Object )) != NULL)
		{
		// In order to change the color a dynamic material must be used
		pMatDyn = UMaterialInstanceDynamic::Create(pMat,this);

		// Default color
		pMatDyn->SetVectorParameterValue ( FName("Color"), FLinearColor(.5,.5,.5) );

		// Assign material to component
		pcShp->SetMaterial(0,pMatDyn);
		}	// if

	// Place component in world
	pcShp->RegisterComponent();
	}	// InitializeComponent

void UnShape :: UninitializeComponent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Ends gameplay for this component
	//
	////////////////////////////////////////////////////////////////////////

	// Shape
	if (pcShp != NULL)
		{
//		pcShp->DetachFromParent();
//		pcShp->UninitializeComponent();
		pcShp->bVisible		= false;
		pcShp->bHiddenInGame	= true;
		pcShp						= NULL;
		}	// if

	// Base behaviour
	UnElement::UninitializeComponent();
	}	// UninitializeComponent

bool UnShape :: mainTick ( float fD )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Execute work for main game thread.
	//
	//	PARAMETERS
	//		-	fD is the amount of elapsed time since last game loop tick.
	//
	//	RETURN VALUE
	//		true if work is still needed
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	bool		bWrk = false;

	// Debug
//	if (fTt.X > 0 || fTt.Y > 0 || fTt.Z > 0)
//		{
//		dbgprintf ( L"UnShape::mainTick:Translate:%g,%g,%g\r\n",
//						fTf.X, fTf.Y, fTf.Z );
//		}	// if

	// Default behaviour
	bWrk = UnElement::mainTick(fD);

	// Color
	if (bColor)
		{
		// Set base color of dynamic texture
		if (pMatDyn != NULL)
			pMatDyn->SetVectorParameterValue (	FName("Color"), 
						FLinearColor(	((iColor>>16)&0xff)/255.0,
											((iColor>>8)&0xff)/255.0,
											((iColor>>0)&0xff)/255.0,
											((iColor>>24)&0xff)/255.0 ) );
//															FLinearColor(FColor(iColor)) );
		bColor = false;
		}	// if

	// Shape
	if (strName[0] != '\0')
		{
		UStaticMesh	*pMesh	= NULL;
		dbgprintf ( L"UnShape::mainTick:Shape:%s\r\n", (LPCWSTR)strName );

		// Previous shape
//		if (pcShp != NULL)
//			{
//			pcShp->UninitializeComponent();
//			pcShp = NULL;
//			}	// if

		// Load the appropriate mesh
		if			(	!WCASECMP(strName,L"Sphere")	&& 
						FSph != NULL						&& 
						FSph->Succeeded() )
			pMesh = Cast<UStaticMesh>(FSph->Object);
		else if	(	!WCASECMP(strName,L"Cylinder")	&& 
						FCyl != NULL							&& 
						FCyl->Succeeded() )
			pMesh = Cast<UStaticMesh>(FCyl->Object);
		else if	(	!WCASECMP(strName,L"Cone")			&& 
						FCon != NULL							&& 
						FCon->Succeeded() )
			pMesh = Cast<UStaticMesh>(FCon->Object);

		// Rather than error out, default to using a cube
		else if	(	FCub != NULL							&& 
						FCub->Succeeded() )
			pMesh = Cast<UStaticMesh>(FCub->Object);

		// Success ?
		if (pMesh == NULL)
			dbgprintf ( L"UnShape::mainTick:Failed to create mesh for %s\r\n", (LPCWSTR)strName );

		// Assign the loaded mesh
		if (pMesh != NULL)
			pcShp->SetStaticMesh ( pMesh );

		// Shape specific adjustments
		if	(!WCASECMP(strName,L"Cone"))
			pcShp->SetRelativeLocation(FVector(0,0,+0.20));

		// Update size of shape
		if (pMesh != NULL)
			{
			FVector	fMin,fMax;
			float		fSclMax;

			// Compute the max. bounds of the mesh in order to compute scaling
			// to a unit cube.
			pcShp->GetLocalBounds( fMin, fMax );
			fSclMax = 0;
			if ((fMax.X-fMin.X) > fSclMax)
				fSclMax = (fMax.X-fMin.X);
			if ((fMax.Y-fMin.Y) > fSclMax)
				fSclMax = (fMax.Y-fMin.Y);
			if ((fMax.Z-fMin.Z) > fSclMax)
				fSclMax = (fMax.Z-fMin.Z);

			// Scale to unit cube centered at origin (-0.5 to +0.5)
			fSclLcl.X = +1.0/fSclMax;
			fSclLcl.Y = fSclLcl.X;
			fSclLcl.Z = fSclLcl.X;

			// Initial scaling
			pcShp->SetRelativeScale3D(fSclLcl);
			}	// if

		// Done
		strName.at(0)	= '\0';
		}	// if

	return bWrk;
	}	// mainTick

bool UnShape :: onReceive (	nElement *pElem,
											const WCHAR *pwRoot, 
											const WCHAR *pwLoc,
											const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		nSpaceClientCB
	//
	//	PURPOSE
	//		-	Called when a listened location receives a value.
	//
	//	PARAMETERS
	//		-	pElem is the nSpace element
	//		-	pwRoot is the path to the listened location
	//		-	pwLoc is the location relative to the root for the value
	//		-	v is the value
	//
	//	RETURN VALUE
	//		true if there is main game loop to be scheduled.
	//
	////////////////////////////////////////////////////////////////////////
	bool	bSch	= false;

	// Base behaviour
	bSch = UnElement::onReceive(pElem,pwRoot,pwLoc,v);

	// Color
	if (!WCASECMP(pwLoc,L"Element/Color/OnFire/Value"))
		{
		// Color update
		iColor= adtInt(v);
		bColor= true;
		bSch	= true;
		}	// if

	// Name
	else if (!WCASECMP(pwLoc,L"Name/OnFire/Value"))
		{
		// Shape name update
		adtValue::copy ( adtString(v), strName );
 		strName.at();
		bSch	= true;
		}	// if

	return bSch;
	}	// onReceive

/*
// Called when the game starts or when spawned
void UnShape::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void UnShape::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

*/
