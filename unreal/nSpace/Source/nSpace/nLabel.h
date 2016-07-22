// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "nElement.h"
#include "nLabel.generated.h"

UCLASS()
class NSPACE_API UnLabel : 
	public UnElement
	{
	GENERATED_BODY()
	
	public:	
	// Sets default values for this actor's properties
	UnLabel();
	virtual ~UnLabel();

	// Base class memebers
	virtual void InitializeComponent		( void ) override;
	virtual void UninitializeComponent	( void ) override;
	virtual void onRay						( IDictionary *, const FVector &, const FVector & ) override;
 
	// Text rendering
	UTextRenderComponent	*pcLbl;
	UTextRenderComponent	*pcCrt;
	UBoxComponent			*pcBox;

	// Update label
	adtString	strHorz,strVert,strLbl;
	FColor		clr;
	float			*pfXs;
	int			iXs;
	int			iCaret,iCaretP;

	// Utilities
	virtual bool	mainTick		( float );
	virtual bool	onReceive	( nElement *, const WCHAR *, const WCHAR *, const ADTVALUE &);

	private :

	// Internal utilities	
	void measure ( void );
};
