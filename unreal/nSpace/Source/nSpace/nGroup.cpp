// Fill out your copyright notice in the Description page of Project Settings.

#include "nSpace.h"
#include "nGroup.h"

UnGroup::UnGroup()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pcBox		= NULL;
	}	// UnSpcShape

void UnGroup :: InitializeComponent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Starts gameplay for this component
	//
	////////////////////////////////////////////////////////////////////////

	// Base behaviour
	UnElement::InitializeComponent();

	}	// InitializeComponent

