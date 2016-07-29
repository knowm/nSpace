// Fill out your copyright notice in the Description page of Project Settings.

#include "nSpace.h"
#include "nImage.h"
//#include "nSpcImgCache.h"

static ConstructorHelpers::FObjectFinder<UObject> *FCub	= NULL;
static ConstructorHelpers::FObjectFinder<UObject> *FMat = NULL;

UnImage::UnImage()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	strTag	= L"";
	pTex		= NULL;

	// Globals
	if (FCub == NULL)
		FCub = new ConstructorHelpers::FObjectFinder<UObject> 
					( TEXT("/Engine/BasicShapes/Cube") );
	if (FMat == NULL)
		FMat = new ConstructorHelpers::FObjectFinder<UObject> 
					( TEXT("/Game/ImageMaterial") );
//					( TEXT("/Engine/BasicShapes/BasicShapeMaterial") );

	}	// UnImage

void UnImage :: InitializeComponent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Starts gameplay for this component
	//
	////////////////////////////////////////////////////////////////////////
	FVector	fMin,fMax;
	float		fSclMax;

	// Base behaviour
	UnElement::InitializeComponent();

	// Create static mesh object based on definition
	if	(	FCub != NULL												&& 
			FCub->Succeeded() )
		pMesh = Cast<UStaticMesh>(FCub->Object);

	// Material
	if (FMat->Succeeded())
		pMat = Cast<UMaterialInterface> ( FMat->Object );

	// Success ?
	if (pMesh == NULL)
		{
		dbgprintf ( L"UnSpcShape::InitializeComponent:Failed to create mesh\r\n" );
		return;
		}	// if

	// Create static mesh component
	pcShp					= NewObject<UStaticMeshComponent>
								(this,UStaticMeshComponent::StaticClass());
	pcShp->bVisible	= false;
	pcShp->AttachTo(this);

	// Assign the loaded mesh
	if (pMesh != NULL)
		pcShp->SetStaticMesh ( pMesh );

	// Testing
	pcShp->SetMobility(EComponentMobility::Movable);

	// Assign material
	if (pMat != NULL)
		{
		// In order to change the color a dynamic material must be used
		pMatDyn = UMaterialInstanceDynamic::Create(pMat,this);

		// Assign material to component
		pcShp->SetMaterial(0,pMatDyn);
		}	// if

	// Place component in world
	pcShp->RegisterComponent();

	// Compute the max. bounds of the mesh in order to compute scaling
	// to a unit cube.
	pcShp->GetLocalBounds( fMin, fMax );
	fSclMax = 0;
	if (fMax.X > fSclMax)
		fSclMax = fMax.X;
	if (fMax.Y > fSclMax)
		fSclMax = fMax.Y;
	if (fMax.Z > fSclMax)
		fSclMax = fMax.Z;

	// Scale to unit cube centered at origin
	fSclLcl.X = +0.5/fSclMax;
	fSclLcl.Y = fSclLcl.X;
	fSclLcl.Z = fSclLcl.X;

	// Initial scaling
	pcShp->SetRelativeScale3D(fSclLcl);

	// Debug
//	dbgprintf ( L"Bounds : (%g,%g,%g) (%g,%g,%g) %g,%g\r\n",
//					fMin.X, fMin.Y, fMin.Z,
//					fMax.X, fMax.Y, fMax.Z, fSclMax, fSclLcl.X );

	// Debug
//	TArray<FName>	tn;
//	TArray<FGuid>	tg;
//	pMatDyn->GetBaseMaterial()->GetAllTextureParameterNames ( tn, tg );
//	for (int i = 0;i < tn.Num();++i)
//		dbgprintf ( L"Test : %s\r\n", *(tn[i].ToString()) );
//	pMatDyn->GetBaseMaterial()->GetAll
//	for (int i = 0;i < tn.Num();++i)
//		dbgprintf ( L"Test : %s\r\n", *(tn[i].ToString()) );

	// Use for capturing input
//	inputAdd ( pcShp );
	}	// InitializeComponent

bool UnImage :: mainTick ( float fD )
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

	// Default behaviour
	bWrk = UnElement::mainTick(fD);
/*
	// Image tag
	if (strTag[0] != '\0')
		{
		HRESULT			hri		= S_OK;
		IDictionary		*pDct		= NULL;

		// Check for image in cache
		hri = pParent->pRenLoc->pRen->pImgC->get ( strTag, &pDct );

		// Success ?
		if (hri == S_OK)
			{
//			dbgprintf ( L"UnImage::onReceive:0x%x:%s:%p!!\r\n",
//								hri, (LPCWSTR)strTag, pDct );
			onImage(pDct);
			}	// if

		// Need to try again later
		if (hri == S_FALSE && !bWrk)
			bWrk = true;

		// Image was retrieved successful, stop trying
		if (hri == S_OK)
			strTag.at(0) = '\0';
			
		// Clean up
		_RELEASE(pDct);
		}	// if
*/
	return bWrk;
	}	// mainTick

void UnImage :: onImage ( IDictionary *pImg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a new image is to be used for the element.
	//
	//	PARAMETERS
	//		-	pDct contains the image
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr				= S_OK;
	IMemoryMapped	*pBits		= NULL;
	U8					*pcBitsSrc	= NULL;
	U8					*pcBitsDst	= NULL;
	bool				bBGRA			= false;
	bool				bBGR			= false;
	bool				bRGB			= false;
	adtInt			iW,iH;
	adtValue			vL;
	adtString		strFmt;
	adtIUnknown		unkV;

	// Image properties
	CCLTRY ( pImg->load ( adtString(L"Width"), vL ) );
	CCLOK  ( iW = vL; )
	CCLTRY ( pImg->load ( adtString(L"Height"), vL ) );
	CCLOK  ( iH = vL; )
	CCLTRY ( pImg->load ( adtString(L"Format"), vL ) );
	CCLTRY ( adtValue::toString ( vL, strFmt ) );
	CCLTRY ( pImg->load ( adtString(L"Bits"), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,&pBits));
	CCLTRY ( pBits->lock ( 0, 0, (void **) &pcBitsSrc, NULL ) );

//	dbgprintf ( L"UnImage::onImage:%p:%d x %d:%s:%p\r\n", 
//					pImg, iW, iH, (LPCWSTR)strFmt, pcBitsSrc );

	// Is a new texture needed ?
	if (hr == S_OK && (pTex == NULL || pTex->GetSizeX() != iW || pTex->GetSizeY() != iH))
		{
		// Create new texture
		CCLTRYE ( (pTex = UTexture2D::CreateTransient ( iW, iH, 
						PF_B8G8R8A8 )) != NULL, E_UNEXPECTED );
//		dbgprintf ( L"UnImage::onImage:Texture:%p:%d x %d\r\n",
//						pTex, pTex->GetSizeX(), pTex->GetSizeY() );

		// Allocate the texture
		CCLOK ( pTex->UpdateResource(); )

		// Use as new texture for material
		if (pTex != NULL)
			pMatDyn->SetTextureParameterValue ( FName("DynamicTexture"), pTex );
		}	// if

	// Access data for texture
//	CCLOK		( pTex->TemporarilyDisableStreaming(); )
	CCLTRYE	( (pcBitsDst = static_cast<U8 *> ( pTex->PlatformData->Mips[0].BulkData.Lock 
					( LOCK_READ_WRITE ) )) != NULL, E_UNEXPECTED );

	// Supported Formats
	if (hr == S_OK)
		{
		if (!WCASECMP(strFmt,L"B8G8R8A8"))
			bBGRA = true;
		else if (!WCASECMP(strFmt,L"B8G8R8"))
			bBGR = true;
		else if (!WCASECMP(strFmt,L"R8G8B8"))
			bRGB = true;
		else
			hr = ERROR_NOT_SUPPORTED;
		}	// if

	// Transfer image bits to texture based on format
	if (hr == S_OK)
		{
		// Update texture
		for (int32 r = 0;hr == S_OK && r < pTex->GetSizeY() && r < (int32)iH;++r)
			for (int32 c = 0;hr == S_OK && c < pTex->GetSizeX() && c < (int32)iW;++c)
				{
				// Source index
				U32 srcidx = r*iW + c;
				srcidx *= ((bBGRA) ? 4 : 3);

				// Destination index
				U32 dstidx = (r*pTex->GetSizeX() + c)*4;

				// Copy pixel data
				pcBitsDst[dstidx+0] = pcBitsSrc[(bRGB) ? srcidx+2 : srcidx+0];
				pcBitsDst[dstidx+1] = pcBitsSrc[(bRGB) ? srcidx+1 : srcidx+1];
				pcBitsDst[dstidx+2] = pcBitsSrc[(bRGB) ? srcidx+0 : srcidx+2];
				if (bBGRA)
					pcBitsDst[dstidx+3] = pcBitsSrc[srcidx+3];
				else
					pcBitsDst[dstidx+3] = 0xff;
				}	// for

//		memset ( pcBitsDst, 0x80, pTex->GetSizeX()*pTex->GetSizeY()*4 );
		}	// if

	// Release and update texture
	if (pcBitsDst != NULL)
		{
		pTex->PlatformData->Mips[0].BulkData.Unlock();
		pTex->UpdateResource();
		}	// if

	// Clean up
	if (pcBitsSrc != NULL)
		pBits->unlock ( pcBitsSrc );
	_RELEASE(pBits);
	}	// onImage

bool UnImage :: onReceive (	nElement *pElem,
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

	// Tag
	if (!WCASECMP(pwLoc,L"Interface/Tag/OnFire/Value"))
		{
		// Tag update
		adtValue::toString ( v, strTag );
		bSch	= true;
		}	// if

	return bSch;
	}	// onReceive
