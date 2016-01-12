////////////////////////////////////////////////////////////////////////
//
//									AVL.CPP
//
//			Implementation of an AVL binary search tree (dictionary)
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"
#include <stdio.h>

AVL :: AVL ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pRoot			= NULL;
	numnodes		= 0;
	pDctIt		= NULL;
	pKeyIt		= NULL;

//WCHAR wBfr[100];
//swprintf ( wBfr, L"Dictionary::Dictionary:Size %d Node %d ValueImpl %d (%d) CCLObject %d\n",
//				sizeof(Dictionary), sizeof(AVLNode), sizeof(adtValueImpl), sizeof(adtValue), sizeof(CCLObject) );
//OutputDebugString ( wBfr );
	}	// Dictionary

HRESULT AVL :: clear ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Resets the container.
	//
	////////////////////////////////////////////////////////////////////////

	// Thread safety
	if (cs.enter())
		{
		// Delete all nodes
		deleteNode ( &pRoot );
		numnodes = 0;

		// Thread safety
		cs.leave();
		}	// if

	return S_OK;
	}	// clear

HRESULT AVL :: clone ( IUnknown **ppUnk )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ICloneable
	//
	//	PURPOSE
	//		-	Clones the object.
	//
	//	PARAMETERS
	//		-	ppUnk will receive the clone
	//
	//	RETURN
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	AVL		*pDct	= NULL;
	IIt		*pIt	= NULL;

	// Create target object
//	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );
	CCLTRYE( (pDct = new AVL()) != NULL, E_OUTOFMEMORY );
	CCLOK  ( pDct->AddRef(); )
	CCLTRY ( pDct->construct() );

	// Iterate and store values into target
	CCLTRY ( keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		// Value for key
		CCLTRY ( load ( vK, vV ) );

		// Perform a 'deep' copy so clone the key and values as well.
		CCLTRY ( adtValue::clone ( vK, vKc ) );
		CCLTRY ( adtValue::clone ( vV, vVc ) );

		// Store pair in target
		CCLTRY ( pDct->store ( vKc, vVc ) );

		// Next key
		pIt->next();
		}	// while

	// Result
	(*ppUnk) = (IDictionary *)pDct;
	_ADDREF(*ppUnk);

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pDct);
		
	return hr;
	}	// clone

HRESULT AVL :: copyNode ( AVLNode *pNode, IDictionary *pDst )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Copies the key/value pair into the given dictionary.
	//
	//	PARAMETERS
	//		-	pNode is the current node
	//		-	pDst will receive the pair
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Base case
	if (pNode == NULL) return S_OK;

	// Copy nodes
	CCLTRY ( pDst->store ( pNode->key, pNode->value ) );
	CCLTRY ( copyNode ( pNode->pLeft, pDst ) );
	CCLTRY ( copyNode ( pNode->pRight, pDst ) );

	return hr;
	}	// copyNode

HRESULT AVL :: copyTo ( IContainer *pCont )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ICopyTo
	//
	//	PURPOSE
	//		-	Copies values from this container to another container.
	//
	//	PARAMETERS
	//		-	pCont will receive the values
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IDictionary	*pDict	= NULL;

	// Thread safety
	if (cs.enter())
		{
		// Must be dictionary
		CCLTRY ( _QISAFE(pCont,IID_IDictionary,&pDict) );

		// Copy nodes
		CCLTRY ( copyNode ( pRoot, pDict ) );

		// Clean up
		_RELEASE(pDict);

		// Thread safety
		cs.leave();
		}	// if

	return hr;
	}	// copyTo

void AVL :: deleteNode ( AVLNode **ppNode )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Deletes the specified node from the tree.
	//
	//	PARAMETERS
	//		-	ppNode is the node to delete.
	//
	////////////////////////////////////////////////////////////////////////

	// Base case
	if (*ppNode == NULL) return;

	// Delete children
	deleteNode ( &((*ppNode)->pLeft) );
	deleteNode ( &((*ppNode)->pRight) );

	// Debug
//	if((*ppNode) != NULL && 
//		adtValue::type((*ppNode)->key) == VTYPE_STR &&
//		(*ppNode)->key.pstr != NULL)
//		dbgprintf(L"AVL::deleteNode:%s\r\n", (*ppNode)->key.pstr );
	if (	adtValue::type ( (*ppNode)->key ) == VTYPE_STR &&
			(*ppNode)->key.pstr != NULL &&
			!WCASECMP((*ppNode)->key.pstr,L"OnNext") )
		dbgprintf ( L"Hi\r\n" );

	// Done
	(*ppNode)->pParent	= NULL;
	(*ppNode)->pLeft		= NULL;
	(*ppNode)->pRight		= NULL;
	_AVLRELEASE((*ppNode));
	}	// deleteNode

void AVL :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// Iterators
	if (pDctIt != NULL)
		pDctIt->reset();
	_RELEASE(pDctIt);
	if (pKeyIt != NULL)
		pKeyIt->reset();
	_RELEASE(pKeyIt);

	// Empty container
	clear();
	}	// destruct

int AVL :: findHighest ( AVLNode *pTarget, AVLNode **ppNode,
												AVL_RES *res )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Replace a node with a subtree's highest-ranking item.
	//
	//	PARAMETERS
	//		-	pTarget is the node to be replaced
	//		-	ppNode is the subtree
	//		-	res will receive the return value
	//
	//	RETURN VALUE
	//		1 means target was found and replaced
	//		0 means subtree was empty
	//
	////////////////////////////////////////////////////////////////////////
	AVLNode	*pTmp;

	// Setup
	*res = AVL_RES_BALANCE;

	// Base class
	if ((*ppNode) == NULL) return 0;

	// Check next right child
	if ((*ppNode)->pRight != NULL)
		{
		if (!findHighest ( pTarget, &((*ppNode)->pRight), res ))
			return 0;
		if ((*res) == AVL_RES_BALANCE)
			*res = shrunkRight ( ppNode );
		return 1;
		}	// if

	// Copy node information
	adtValue::copy ( (*ppNode)->key,		pTarget->key );
	adtValue::copy ( (*ppNode)->value,	pTarget->value );

	// Replace
	pTmp			= (*ppNode);
	(*ppNode)	= (*ppNode)->pLeft;

	// Update node children of new parent
	if ((*ppNode) != NULL)
		(*ppNode)->pParent = pTmp->pParent;

	// Remove node from tree
	pTmp->pParent	= NULL;
	pTmp->pLeft		= NULL;
	pTmp->pRight	= NULL;
	_AVLRELEASE(pTmp);
	--numnodes;

	return 1;
	}	// findHighest

int AVL :: findLowest ( AVLNode *pTarget, AVLNode **ppNode,
												AVL_RES *res )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Replace a node with a subtree's lowest-ranking item.
	//
	//	PARAMETERS
	//		-	pTarget is the node to be replaced
	//		-	ppNode is the subtree
	//		-	res will receive the return value
	//
	//	RETURN VALUE
	//		1 means target was found and replaced
	//		0 means subtree was empty
	//
	////////////////////////////////////////////////////////////////////////
	AVLNode	*pTmp;

	// Setup
	*res = AVL_RES_BALANCE;

	// Base class
	if ((*ppNode) == NULL) return 0;

	// Check next leftt child
	if ((*ppNode)->pLeft != NULL)
		{
		if (!findLowest ( pTarget, &((*ppNode)->pLeft), res ))
			return 0;
		if ((*res) == AVL_RES_BALANCE)
			*res = shrunkLeft ( ppNode );
		return 1;
		}	// if

	// Copy node information
	adtValue::copy ( (*ppNode)->key,		pTarget->key );
	adtValue::copy ( (*ppNode)->value,	pTarget->value	);

	// Replace
	pTmp				= (*ppNode);
	(*ppNode)		= (*ppNode)->pRight;

	// Update node children of new parent
	if ((*ppNode) != NULL)
		(*ppNode)->pParent = pTmp->pParent;

	// Remove node from tree
	pTmp->pParent	= NULL;
	pTmp->pLeft		= NULL;
	pTmp->pRight	= NULL;
	_AVLRELEASE(pTmp);
	--numnodes;

	return 1;
	}	// findLowest

AVL_RES AVL :: findNode ( AVLNode *pNode, const ADTVALUE &Key,
												AVLNode **ppFound )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Recursively searches the tree for the node with the specified
	//			key.
	//
	//	PARAMETERS
	//		-	pNode is the currnet node to search
	//		-	Key is the key
	//		-	ppFound will receive the found node.
	//
	//	RETURN VALUE
	//		AVL_RES_OK if found
	//
	////////////////////////////////////////////////////////////////////////
	AVL_RES	res = AVL_RES_ERROR;

	// Base case
	if (pNode == NULL) return AVL_RES_ERROR;

	// This node ?
	int c = adtValue::compare ( pNode->key, Key );
	switch (c)
		{
		// Found !
		case 0 :
			(*ppFound) = pNode;
			res = AVL_RES_OK;
			break;

		// Key lies to left
		case 1 :
			res = findNode ( pNode->pLeft, Key, ppFound );
			break;

		// Key lies to right
		case -1 :
			res = findNode ( pNode->pRight, Key, ppFound );
			break;

		// Error
		default :
			res = AVL_RES_ERROR;
		}	// switch

	return res;
	}	// findNode

AVL_RES AVL :: grewLeft ( AVLNode **ppNode )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the specified node has grew in height down the
	//			left side.
	//
	//	PARAMETERS
	//		-	ppNode is the node that has grew.
	//
	//	RETURN VALUE
	//		AVL_RES_OK if no further action required
	//
	////////////////////////////////////////////////////////////////////////
	AVL_RES	res = AVL_RES_BALANCE;

	// Process action based on current skew of node
	switch ((*ppNode)->skew)
		{
		// If skew is already left, need to balance nodes
		case AVL_SKEW_LEFT :
			res = AVL_RES_OK;								// Will be balanced

			// If left side is also skewed left, a right rotation will
			// balance nodes
			if ((*ppNode)->pLeft->skew == AVL_SKEW_LEFT)
				{
				// Rotation
				(*ppNode)->skew = (*ppNode)->pLeft->skew = AVL_SKEW_NONE;
				rotateRight ( ppNode );
				}	// if

			// Otherwise, need other rotations
			else
				{
				// Update skew flags of left side
				switch ((*ppNode)->pLeft->pRight->skew)
					{
					case AVL_SKEW_LEFT :
						(*ppNode)->skew			= AVL_SKEW_RIGHT;
						(*ppNode)->pLeft->skew	= AVL_SKEW_NONE;
						break;
					case AVL_SKEW_RIGHT :
						(*ppNode)->skew			= AVL_SKEW_NONE;
						(*ppNode)->pLeft->skew	= AVL_SKEW_LEFT;
						break;
					default :
						(*ppNode)->skew			= AVL_SKEW_NONE;
						(*ppNode)->pLeft->skew	= AVL_SKEW_NONE;
					}	// switch

				// Perform rotation to balance nodes
				(*ppNode)->pLeft->pRight->skew = AVL_SKEW_NONE;
				rotateLeft ( &((*ppNode)->pLeft) );
				rotateRight ( ppNode );
				}	// else

			break;

		// If skew is right, left growth balances node
		case AVL_SKEW_RIGHT :
			(*ppNode)->skew = AVL_SKEW_NONE;
			res = AVL_RES_OK;
			break;

		// If skew is none, now left side is skewed
		case AVL_SKEW_NONE :
			(*ppNode)->skew = AVL_SKEW_LEFT;
			res = AVL_RES_BALANCE;						// Need balancing
			break;

		}	// switch

	return res;
	}	// grewLeft

AVL_RES AVL :: grewRight ( AVLNode **ppNode )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the specified node has grew in height down the
	//			rigth side.
	//
	//	PARAMETERS
	//		-	ppNode is the node that has grew.
	//
	//	RETURN VALUE
	//		AVL_RES_OK if no further action required
	//
	////////////////////////////////////////////////////////////////////////
	AVL_RES	res = AVL_RES_BALANCE;

	// Process action based on current skew of node
	switch ((*ppNode)->skew)
		{
		// If skew is already right, need to balance nodes
		case AVL_SKEW_RIGHT :
			res = AVL_RES_OK;								// Will be balanced

			// If right side is also skewed right, a left rotation will
			// balance nodes
			if ((*ppNode)->pRight->skew == AVL_SKEW_RIGHT)
				{
				// Rotation
				(*ppNode)->skew = (*ppNode)->pRight->skew = AVL_SKEW_NONE;
				rotateLeft ( ppNode );
				}	// if

			// Otherwise, need other rotations
			else
				{
				// Update skew flags of right side
				switch ((*ppNode)->pRight->pLeft->skew)
					{
					case AVL_SKEW_RIGHT :
						(*ppNode)->skew			= AVL_SKEW_LEFT;
						(*ppNode)->pRight->skew	= AVL_SKEW_NONE;
						break;
					case AVL_SKEW_LEFT :
						(*ppNode)->skew			= AVL_SKEW_NONE;
						(*ppNode)->pRight->skew	= AVL_SKEW_RIGHT;
						break;
					default :
						(*ppNode)->skew			= AVL_SKEW_NONE;
						(*ppNode)->pRight->skew	= AVL_SKEW_NONE;
					}	// switch

				// Perform rotation to balance nodes
				(*ppNode)->pRight->pLeft->skew = AVL_SKEW_NONE;
				rotateRight ( &((*ppNode)->pRight) );
				rotateLeft ( ppNode );
				}	// else

			break;

		// If skew is left, right growth balances node
		case AVL_SKEW_LEFT :
			(*ppNode)->skew = AVL_SKEW_NONE;
			res = AVL_RES_OK;
			break;

		// If skew is none, now right side is skewed
		case AVL_SKEW_NONE :
			(*ppNode)->skew = AVL_SKEW_RIGHT;
			res = AVL_RES_BALANCE;						// Need balancing
			break;

		}	// switch

	return res;
	}	// grewRight

AVL_RES AVL :: insertNode ( AVLNode **ppNode, AVLNode *pParent,
												const ADTVALUE &Key, const ADTVALUE &Value )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Inserts a node in the tree with the specified key value.
	//
	//	PARAMETERS
	//		-	ppNode starts with the current node, ends with new node.
	//		-	pParent is the parent node
	//		-	Key is the key value
	//		-	Value is the node value
	//
	//	RETURN VALUE
	//		AVL_RES_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	AVL_RES	res = AVL_RES_ERROR;
	int		c;

	// If ppNode is NULL then we have reached the end of the current
	// search path, add the node here.
	if (*ppNode == NULL)
		{
		// Create a node to contain value
		if ( ((*ppNode) = (AVLNode *) _ALLOCMEM ( sizeof(AVLNode) )) != NULL )
			{
			// Node information
			_AVLINIT((*ppNode));
			adtValue::copy ( Key, (*ppNode)->key );
			adtValue::copy ( Value, (*ppNode)->value );
			(*ppNode)->pParent	= pParent;
			res						= AVL_RES_BALANCE;
			++numnodes;
			}	// if
		}	// if

	// Continue...
	else
		{
		// Compare keys
		c = adtValue::compare ( (*ppNode)->key, Key );

		// Continue down correct path

		// Go down left side...
		if ( c == 1 )
			{
			// Insert node
			res = insertNode ( &((*ppNode)->pLeft), (*ppNode), Key, Value );

			// Balance ?
			if (res == AVL_RES_BALANCE)
				res = grewLeft ( ppNode );
			}

		// Go down right side...
		else if ( c == -1 )
			{
			// Insert node
			res = insertNode ( &((*ppNode)->pRight), (*ppNode), Key, Value );

			// Balance ?
			if (res == AVL_RES_BALANCE)
				res = grewRight ( ppNode );
			}	// else if

		// Keys equal, replace value
		else if ( c == 0 )
			{
			adtValue::copy ( Value, (*ppNode)->value );
			res					= AVL_RES_OK;
			}	// else if

		else
			{
			// Debug
			#ifdef	_DEBUG
//			WCHAR	dbgbufr[MAX_PATH];
//			swprintf ( dbgbufr, L"Dictionary::insertNode:Compare error 0x%x\n", c );
//			OutputDebugString ( dbgbufr );
			#endif
			}	// else

		}	// else

	return res;
	}	// insertNode

HRESULT AVL :: isEmpty ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns the empty state of the container
	//
	//	RETURN VALUE
	//		S_OK if container is empty
	//
	////////////////////////////////////////////////////////////////////////
	return (pRoot == NULL) ? S_OK : S_FALSE;
	}	// isEmpty

HRESULT AVL :: iterate ( IIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns an iterator for the container.
	//
	//	PARAMETERS
	//		-	ppIt will receive a ptr. to the iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return iterators ( false, ppIt );
	}	// iterate

HRESULT AVL :: iterators ( bool bKeys, IIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Handles management of iterators.
	//
	//	PARAMETERS
	//		-	bKeys is true for keys, false for values.
	//		-	ppIt will receive a ptr. to the iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	AVLIt		*pIt	= NULL;

	// Setup
	(*ppIt) = NULL;

	// Internal iterators.  A pre-allocated iterator is kept ready for
	// fast back to back iterations.
	if ((bKeys && pKeyIt == NULL) || (!bKeys && pDctIt == NULL))
		{
		// Pre-create iterator
		CCLTRYE	( (pIt = new AVLIt(bKeys)) != NULL, E_OUTOFMEMORY );
		CCLTRY	( pIt->construct() );
		CCLOK		( pIt->AddRef(); )

		// Manually assign tree (no reference count)
		CCLOK		( pIt->pTree = this; )

		// Cache ptr.
		if (hr == S_OK && bKeys)
			pKeyIt = pIt;
		else if (hr == S_OK && !bKeys)
			pDctIt = pIt;
		}	// if

	// If internal iterator has a reference count of 1 then it is not
	// currently in use, use that
	if (hr == S_OK && ( 
			(bKeys && pKeyIt->lRefCnt == 1) ||
			(!bKeys && pDctIt->lRefCnt == 1) ) )
		{
		(*ppIt)	= (bKeys) ? pKeyIt : pDctIt;
		(*ppIt)->begin();
		_ADDREF((*ppIt));
		}	// if

	// Otherwise create a new iterator
	else
		{
		// Detached iterator
		CCLOK ( pIt = new AVLIt(this,bKeys); )
		CCLTRY( pIt->construct() );

		// Result ?
		CCLOK ( (*ppIt) = pIt; )
		_ADDREF((*ppIt));
		}	// else

	return hr;
	}	// iterators

HRESULT AVL :: keys ( IIt **ppKeys )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDictionary
	//
	//	PURPOSE
	//		-	Returns an object to iterate through the keys in the tree.
	//
	//	PARAMETERS
	//		-	ppKeys will receive the iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return iterators ( true, ppKeys );
	}	// keys

HRESULT AVL :: load ( const ADTVALUE &vKey, ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDictionary
	//
	//	PURPOSE
	//		-	Loads a value from the dictionary with the given key.
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	AVLNode	*pNode	= NULL;

	// Thread safety
	if (cs.enter())
		{
		// To support the loading of 'paths', support key conversion to
		// same type as root key.
		if (	pRoot != NULL													&& 
				adtValue::type(pRoot->key) != adtValue::type(vKey) &&
				adtValue::type(vKey) == VTYPE_STR )
			{
			adtValue	vKeyAlt;

			// Convert to the same type as the root key
			CCLTRY ( adtValue::fromString  ( vKey.pstr, (VALUETYPE) pRoot->key.vtype, vKeyAlt ) );

			// Find node with alternate key
			CCLTRYE ( findNode ( pRoot, vKeyAlt, &pNode ) == AVL_RES_OK, ERROR_NOT_FOUND );
			}	// if

		else
			{
			// Find node for key (if it exists)
			CCLTRYE ( findNode ( pRoot, vKey, &pNode ) == AVL_RES_OK, ERROR_NOT_FOUND );
			}	// else

		// Return vlaue
		CCLOK ( adtValue::copy ( pNode->value, vValue ); )

		// Thread safe
		cs.leave();
		}	// if

	return hr;
	}	// load

HRESULT AVL :: remove ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Removes an item from the container identified by the specified
	//			value.
	//
	//	PARAMETERS
	//		-	v identifies the key to remove
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;

	// Thread safe
	if (cs.enter())
		{
		// Remove key
		CCLTRYE ( (removeNode ( &pRoot, NULL, v ) !=  AVL_RES_ERROR), S_FALSE );

		// Thread safe
		cs.leave();
		}	// if

	return hr;
	}	// remove

AVL_RES AVL :: removeNode ( AVLNode **ppNode, AVLNode *pParent,
													const ADTVALUE &Key )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Removes a node in the tree with the specified key value.
	//
	//	PARAMETERS
	//		-	ppNode starts with the current node, ends with removed node.
	//		-	pParent is the parent node
	//		-	Key is the key value
	//
	//	RETURN VALUE
	//		AVL_RES_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	AVL_RES	res = AVL_RES_BALANCE;
	int		c;

	// If ppNode is NULL then we have reached the end of the current
	// search path, that would be an error (key not found)
	if (*ppNode == NULL)
		{
		res = AVL_RES_ERROR;
		}	// if

	// Continue...
	else
		{
		// Compare keys
		c = adtValue::compare ( (*ppNode)->key, Key );

		// Continue down correct path

		// Go down left side...
		if ( c == 1 )
			{
			// Remove node
			res = removeNode ( &((*ppNode)->pLeft), (*ppNode), Key );

			// Balance ?
			if (res == AVL_RES_BALANCE)
				res = shrunkLeft ( ppNode );
			}

		// Go down right side...
		else if ( c == -1 )
			{
			// Remove node
			res = removeNode ( &((*ppNode)->pRight), (*ppNode), Key );

			// Balance ?
			if (res == AVL_RES_BALANCE)
				res = shrunkRight ( ppNode );
			}	// else if

		// Keys equal.  Found node to remove.
		else if ( c == 0 )
			{

			// Standard AVL tree adjustment on removal

			// Node has left child ?
			if (	((*ppNode)->pLeft != NULL) &&
					findHighest ( (*ppNode), &((*ppNode)->pLeft), &res ) )
				{
				if (res == AVL_RES_BALANCE)
					res = shrunkLeft ( ppNode );
				}	// if

			// Node has right child ?
			else if (	((*ppNode)->pRight != NULL) &&
							findLowest ( (*ppNode), &((*ppNode)->pRight), &res ) )
				{
				if (res == AVL_RES_BALANCE)
					res = shrunkRight ( ppNode );
				}	// if

			else
				{
				// Node to release.  Updating parent will NULL ptr.
				AVLNode	*pRel	= (*ppNode);

				// Update parent
				if ((*ppNode)->pParent != NULL)
					{
					if ((*ppNode)->pParent->pLeft == (*ppNode))
						(*ppNode)->pParent->pLeft = NULL;
					else if ((*ppNode)->pParent->pRight == (*ppNode))
						(*ppNode)->pParent->pRight = NULL;
					}	// if

				// Clean up
				(*ppNode)		= NULL;
				pRel->pLeft		= NULL;
				pRel->pRight	= NULL;
				pRel->pParent	= NULL;
				_AVLRELEASE(pRel);
				res = AVL_RES_BALANCE;
				--numnodes;
				}	// else

			}	// else if

		else
			{
			// Debug
			#ifdef	_DEBUG
//			WCHAR	dbgbufr[MAX_PATH];
//			swprintf ( dbgbufr, L"Dictionary::removeNode:Compare error 0x%x\n", c );
//			OutputDebugString ( dbgbufr );
			#endif
			}	// else

		}	// else

	return res;
	}	// removeNode

void AVL :: rotateLeft ( AVLNode **ppNode )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Performs a 'left' rotation of a sub-tree.
	//
	//	PARAMETERS
	//		-	ppNode is the root node
	//
	////////////////////////////////////////////////////////////////////////
	AVLNode	*pTmp			= (*ppNode);

	// Rotate
	(*ppNode)				= (*ppNode)->pRight;
	pTmp->pRight			= (*ppNode)->pLeft;
	(*ppNode)->pLeft		= pTmp;

	// Parent nodes
	(*ppNode)->pParent	= pTmp->pParent;
	pTmp->pParent			= (*ppNode);
	if (pTmp->pRight != NULL)
		pTmp->pRight->pParent	= pTmp;

	}	// rotateLeft

void AVL :: rotateRight ( AVLNode **ppNode )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Performs a 'right' rotation of a sub-tree.
	//
	//	PARAMETERS
	//		-	ppNode is the root node
	//
	////////////////////////////////////////////////////////////////////////
	AVLNode	*pTmp			= (*ppNode);

	// Rotate
	(*ppNode)				= (*ppNode)->pLeft;
	pTmp->pLeft				= (*ppNode)->pRight;
	(*ppNode)->pRight		= pTmp;

	// Parent nodes
	(*ppNode)->pParent	= pTmp->pParent;
	pTmp->pParent			= (*ppNode);
	if (pTmp->pLeft != NULL)
		pTmp->pLeft->pParent	= pTmp;

	}	// rotateRight

HRESULT AVL :: size ( U32 *s )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns the # of items in the container.
	//
	//	PARAMETERS
	//		-	s will return the size of the container
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	(*s) = numnodes;
	return S_OK;
	}	// size

AVL_RES AVL :: shrunkLeft ( AVLNode **ppNode )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the specified node has shrunk in height down the
	//			left side.
	//
	//	PARAMETERS
	//		-	ppNode is the node that has shrunk.
	//
	//	RETURN VALUE
	//		AVL_RES_OK if no further action required
	//
	////////////////////////////////////////////////////////////////////////
	AVL_RES	res = AVL_RES_BALANCE;

	// Process action based on current skew of node
	switch ((*ppNode)->skew)
		{
		// If skew is already left, left shrink balances node
		case AVL_SKEW_LEFT :
			(*ppNode)->skew	= AVL_SKEW_NONE;
			res					= AVL_RES_BALANCE;	// Need balancing
			break;

		// Right skew.
		case AVL_SKEW_RIGHT :
			// Right child also right skewed
			if ((*ppNode)->pRight->skew == AVL_SKEW_RIGHT)
				{
				(*ppNode)->skew = (*ppNode)->pRight->skew = AVL_SKEW_NONE;
				rotateLeft ( ppNode );					// Rotate
				res = AVL_RES_BALANCE;					// Need balancing
				}	// if

			// Right child not skewed
			else if ((*ppNode)->pRight->skew == AVL_SKEW_NONE)
				{
				(*ppNode)->skew			= AVL_SKEW_RIGHT;
				(*ppNode)->pRight->skew = AVL_SKEW_LEFT;
				rotateLeft ( ppNode );
				res = AVL_RES_OK;
				}	// else if

			// More complicated cases
			else
				{

				// Skew of left child of right child
				switch ( (*ppNode)->pRight->pLeft->skew )
					{
					case AVL_SKEW_LEFT :
						(*ppNode)->skew			= AVL_SKEW_NONE;
						(*ppNode)->pRight->skew = AVL_SKEW_RIGHT;
						break;
					case AVL_SKEW_RIGHT :
						(*ppNode)->skew			= AVL_SKEW_LEFT;
						(*ppNode)->pRight->skew = AVL_SKEW_NONE;
						break;
					default :
						(*ppNode)->skew			= AVL_SKEW_NONE;
						(*ppNode)->pRight->skew = AVL_SKEW_NONE;
					}	// switch

				// Adjust
				(*ppNode)->pRight->pLeft->skew = AVL_SKEW_NONE;
				rotateRight ( & ((*ppNode)->pRight) );
				rotateLeft  ( ppNode );
				res = AVL_RES_BALANCE;
				}	// else

			break;

		// If skew is none, now skewed left
		default :
			(*ppNode)->skew = AVL_SKEW_RIGHT;
			res = AVL_RES_OK;
			break;

		}	// switch

	return res;
	}	// shrunkLeft

AVL_RES AVL :: shrunkRight ( AVLNode **ppNode )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the specified node has shrunk in height down the
	//			right side.
	//
	//	PARAMETERS
	//		-	ppNode is the node that has shrunk.
	//
	//	RETURN VALUE
	//		AVL_RES_OK if no further action required
	//
	////////////////////////////////////////////////////////////////////////
	AVL_RES	res = AVL_RES_BALANCE;

	// Process action based on current skew of node
	switch ((*ppNode)->skew)
		{
		// If skew is already right, right shrink balances node
		case AVL_SKEW_RIGHT :
			(*ppNode)->skew	= AVL_SKEW_NONE;
			res					= AVL_RES_BALANCE;	// Need balancing
			break;

		// Left skew.
		case AVL_SKEW_LEFT :

			// Left child also left skewed
			if ((*ppNode)->pLeft->skew == AVL_SKEW_LEFT)
				{
				(*ppNode)->skew = (*ppNode)->pLeft->skew = AVL_SKEW_NONE;
				rotateRight ( ppNode );					// Rotate
				res = AVL_RES_BALANCE;					// Need balancing
				}	// if

			// Left child not skewed
			else if ((*ppNode)->pLeft->skew == AVL_SKEW_NONE)
				{
				(*ppNode)->skew			= AVL_SKEW_LEFT;
				(*ppNode)->pLeft->skew	= AVL_SKEW_RIGHT;
				rotateRight ( ppNode );
				res = AVL_RES_OK;
				}	// else if

			// More complicated cases
			else
				{
				// Skew of right child of left child
				switch ( (*ppNode)->pLeft->pRight->skew )
					{
					case AVL_SKEW_LEFT :
						(*ppNode)->skew			= AVL_SKEW_RIGHT;
						(*ppNode)->pLeft->skew	= AVL_SKEW_NONE;
						break;
					case AVL_SKEW_RIGHT :
						(*ppNode)->skew			= AVL_SKEW_NONE;
						(*ppNode)->pLeft->skew	= AVL_SKEW_LEFT;
						break;
					default :
						(*ppNode)->skew			= AVL_SKEW_NONE;
						(*ppNode)->pLeft->skew	= AVL_SKEW_NONE;
					}	// switch

				// Adjust
				(*ppNode)->pLeft->pRight->skew = AVL_SKEW_NONE;
				rotateLeft ( & ((*ppNode)->pLeft) );
				rotateRight  ( ppNode );
				res = AVL_RES_BALANCE;
				}	// else

			break;

		// If skew is none, now skewed left
		default :
			(*ppNode)->skew = AVL_SKEW_LEFT;
			res = AVL_RES_OK;
			break;

		}	// switch

	return res;
	}	// shrunkRight

HRESULT AVL :: store ( const ADTVALUE &vKey, const ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDictionary
	//
	//	PURPOSE
	//		-	Stores a value in the dictionary with the given key.
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;

	// Thread safe
	if (cs.enter())
		{
		// Add value
		CCLTRYE ( (insertNode ( &pRoot, NULL, vKey, vValue ) !=  AVL_RES_ERROR), S_FALSE );

		cs.leave();
		}	// if

	return hr;
	}	// store

///////////
// 'AVLIt'
///////////

AVLIt :: AVLIt ( AVL *_tree, bool _bKeys )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_tree is a ptr. to the container for this iterator.
	//		-	_bKeys is true to iterate keys, false for values
	//
	////////////////////////////////////////////////////////////////////////
	pTree	= _tree; pTree->AddRef();					// Keep container alive
	bKeys	= _bKeys;
	pPos	= NULL;
	bRef	= true;

	// Start iterator at beginning
	begin();
	}	// DictionaryIt

AVLIt :: AVLIt ( bool _bKeys )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_bKeys is true to iterate keys, false for values
	//
	////////////////////////////////////////////////////////////////////////
	pTree	= NULL;											// Parent assigns object
	bKeys	= _bKeys;
	pPos	= NULL;
	bRef	= false;
	}	// DictionaryIt

HRESULT AVLIt :: begin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Resets the iterator position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	AVLNode	*pNode;

	// State check
	CCLTRYE ( pTree != NULL, ERROR_INVALID_STATE );

	// Thread safe
	if (hr == S_OK && pTree->cs.enter())
		{
		// Start at the 'left' most node.
		_AVLRELEASE(pPos);
		for (	pNode = pTree->pRoot; pNode != NULL; pNode = pNode->pLeft )
			pPos = pNode;
		_AVLADDREF(pPos);
		pTree->cs.leave();
		}	// if

	return hr;
	}	// begin

void AVLIt :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
	_AVLRELEASE(pPos);
	if (bRef)
		pTree->Release();
	}	// destruct

HRESULT AVLIt :: end ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Resets the iterator position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// State check
	CCLTRYE ( pTree != NULL, ERROR_INVALID_STATE );

	// Thread safe
	if (hr == S_OK && pTree->cs.enter())
		{
		// NULL = end position
		_AVLRELEASE(pPos);
		pTree->cs.leave();
		}	// if

	return hr;
	}	// end

ULONG AVLIt :: InnerAddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IInnerUnknown
	//
	//	PURPOSE
	//		-	Increments the reference count on the object.
	//
	//	RETURN VALUE
	//		New reference count
	//
	////////////////////////////////////////////////////////////////////////
	return (lRefCnt = CCLObject::InnerAddRef());
	}	// InnerAddRef

ULONG AVLIt :: InnerRelease ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IInnertUnknown
	//
	//	PURPOSE
	//		-	Decrements the reference count on the object.
	//
	//	RETURN VALUE
	//		New reference count
	//
	////////////////////////////////////////////////////////////////////////
	ULONG	lCnt = CCLObject::InnerRelease();
	if (lCnt > 0)
		{
		lRefCnt = lCnt;
		if (lRefCnt == 1)
			reset();
		}	// if

	return lCnt;
	}	// InnerRelease

HRESULT AVLIt :: next ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Moves to the next position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr 		= S_OK;
	AVLNode	*pLast	= NULL;
	AVLNode	*pNode   = NULL;

	// State check
	CCLTRYE ( pTree != NULL, ERROR_INVALID_STATE );

	// Thread safe
	if (hr == S_OK && pTree->cs.enter())
		{
		// Sanity check
		CCLTRYE ( (pPos != NULL), ERROR_NOT_FOUND );

		// Travel down the left side of the right child (if exists)
		if (hr == S_OK)
			{
			CCLOK ( pNode = pPos->pRight; )
			if (pNode != NULL)
				{
				// Follow path
				while ( pNode != NULL )
					{
					pLast	= pNode;
					pNode = pNode->pLeft;
					}	// while

				// New node
				_AVLRELEASE(pPos);
				pPos = pLast;
				_AVLADDREF(pPos);
				}	// if

			// No right child.  Move up until a parent is found for which we were
			// just in the left tree of...
			else
				{
				CCLOK ( pLast = pPos; )
				CCLOK ( pNode = pPos->pParent; )
				while (pNode != NULL)
					{
					// Left of parent ?
					if ( pNode->pLeft == pLast )
						break;

					// Next parent
					pLast	= pNode;
					pNode = pNode->pParent;
					}	// while

				// New node
				_AVLRELEASE(pPos);
				pPos = pNode;
				_AVLADDREF(pPos);
				}	// else

			}	// if

		// Valid position ?
		CCLTRYE ( (pPos != NULL), ERROR_NOT_FOUND );
		pTree->cs.leave();
		}	// if

	return hr;
	}	// next

HRESULT AVLIt :: prev ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Moves to the previous position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr 		= S_OK;
	AVLNode	*pNode	= NULL;
	AVLNode	*pLast	= NULL;

	// State check
	CCLTRYE ( pTree != NULL, ERROR_INVALID_STATE );

	// Thread safe
	if (hr == S_OK && pTree->cs.enter())
		{
		// No position means end of tree... follow right path from root to get
		// the last node
		if (pPos == NULL)
			{
			CCLOK ( pPos	= pTree->pRoot; )
			CCLOK ( pLast	= pPos; )
			while (pPos != NULL)
				{
				pLast = pPos;
				pPos	= pPos->pRight;
				}	// while

			// New position
			_AVLRELEASE(pPos);
			pPos = pLast;
			_AVLADDREF(pPos);
			}	// if

		// Travel down the right side of the left child (if exists)
		else
			{
			CCLOK ( pNode = pPos->pLeft; )
			if (pNode != NULL)
				{
				// Follow path
				while ( pNode != NULL )
					{
					pLast	= pNode;
					pNode = pNode->pRight;
					}	// while

				// New node
				_AVLRELEASE(pPos);
				pPos = pLast;
				_AVLADDREF(pPos);
				}	// if
		
			// No left child.  Move up until a parent is found for which we were
			// just in the right tree of...
			else
				{
				CCLOK ( pLast = pPos; )
				CCLOK ( pNode = pPos->pParent; )
				while (pNode != NULL)
					{
					// Right of parent ?
					if ( pNode->pRight == pLast )
						break;

					// Next parent
					pLast	= pNode;
					pNode = pNode->pParent;
					}	// while

				// New node (don't fall off front...)
				_AVLRELEASE(pPos);
				pPos = pNode;
				_AVLADDREF(pPos);
				}	// else

			}	// else

		// Valid position ?
		CCLTRYE ( (pPos != NULL), ERROR_NOT_FOUND );

		// Clean up
		pTree->cs.leave();
		}	// if

	return hr;
	}	// prev

HRESULT AVLIt :: read ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Reads the current item from the container.
	//
	//	PARAMETERS
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;

	// State check
	CCLTRYE ( pTree != NULL, ERROR_INVALID_STATE );

	// Thread safe
	if (hr == S_OK && pTree->cs.enter())
		{
		// Copy value
		CCLTRYE	( pPos != NULL, ERROR_NOT_FOUND );
		if (hr == S_OK)
			{
			if (bKeys)	hr = adtValue::copy ( pPos->key, v );
			else			hr = adtValue::copy ( pPos->value, v );
			}	// if
		pTree->cs.leave();
		}	// if

	return hr;
	}	// read

HRESULT AVLIt :: reset ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Reset internal state, called by parent.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Clean up
	if (pPos != NULL)
		{
		_AVLRELEASE(pPos);
		}	// if

	return S_OK;
	}	// reset
