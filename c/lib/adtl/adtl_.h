////////////////////////////////////////////////////////////////////////
//
//										ADTL_.H
//
//			Implementaiton include file for the ADT library
//
////////////////////////////////////////////////////////////////////////

#ifndef	ADTL__H
#define	ADTL__H

#include "adtl.h"
#include "../../lib/nspcl/nspcl.h"
#include "../../lib/iol/iol.h"

///////////
// Objects
///////////

//
// Enum - AVL_RES.	AVL tree result values
//

typedef enum
	{
	AVL_RES_ERROR,											// Error condition
	AVL_RES_OK,												// Success
	AVL_RES_BALANCE										// Need balancing
	} AVL_RES;

//
// Enum - AVL_SKEW.	AVL skew values.
//

typedef enum
	{
	AVL_SKEW_NONE = 0,									// None (balanced)
	AVL_SKEW_LEFT,											// Left skew
	AVL_SKEW_RIGHT											// Right skew
	} ADT_AVL_RES;

//
// Structure - AVLNode.  A node for AVL trees.
//

typedef struct tagAVLNode
	{
	adtValue		key;										// Node key
	adtValue		value;									// Node value
	tagAVLNode	*pLeft;									// Left child
	tagAVLNode	*pRight;									// Right child
	tagAVLNode	*pParent;								// Parent node
	U16			skew;										// Skew/balance
	U16			refcnt;									// Internal reference count on node
	} AVLNode;

// Macros for tree nodes
#define	_AVLINIT(a)				memset ( (a), 0, sizeof(AVLNode) ); (a)->refcnt = 1;
#define	_AVLFREE(a)				adtValue::clear((a)->key);		\
										adtValue::clear((a)->value);	\
										_FREEMEM((a));
#define	_AVLADDREF(a)			if ((a) != NULL) (++((a)->refcnt));
#define	_AVLRELEASE(a)			if ((a) != NULL) { if ((a)->refcnt) --((a)->refcnt);	\
										if (!((a)->refcnt)) { _AVLFREE((a)); } (a) = NULL; }

// Forward decs.
class AVLIt;

//
// Class - AVL.  Implementation of an AVL tree.
//

class AVL :
	public CCLObject,										// Base class
	public IDictionary,									// Interface
	public ICloneable										// Interface
	{
	public :
	AVL ( void );											// Constructor
	virtual ~AVL ( void ) {}							// Destructor
	
	// 'IDictionary' members
	STDMETHOD(keys)	( IIt ** );
	STDMETHOD(load)	( const ADTVALUE &, ADTVALUE & );
	STDMETHOD(store)	( const ADTVALUE &, const ADTVALUE & );

	// 'IContainer' members
	STDMETHOD(clear)		( void );
	STDMETHOD(copyTo)		( IContainer * );
	STDMETHOD(isEmpty)	( void );
	STDMETHOD(iterate)	( IIt ** );
	STDMETHOD(remove)		( const ADTVALUE & );
	STDMETHOD(size)		( U32 * );

	// 'ICloneable' members
	STDMETHOD(clone)		( IUnknown ** );

	// CCL
	CCL_OBJECT_BEGIN(AVL)
		CCL_INTF(IDictionary)
		CCL_INTF(IContainer)
		CCL_INTF(ICloneable)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	private :

	// Run-time data
	AVLNode		*pRoot;									// Root node
	U32			numnodes;								// # of nodes
	sysCS			cs;										// Mutex
	AVLIt			*pDctIt,*pKeyIt;						// Internal iterators
	adtValue		vK,vV,vKc,vVc;							// Internal variables

	// Iterator
	friend class AVLIt;

	// Internal utilities
	HRESULT	copyNode		( AVLNode *, IDictionary * );
	void		deleteNode	( AVLNode ** );
	int		findHighest	( AVLNode *, AVLNode **, AVL_RES * );
	int		findLowest	( AVLNode *, AVLNode **, AVL_RES * );
	HRESULT	iterators	( bool, IIt ** );
	AVL_RES	findNode		( AVLNode *, const ADTVALUE &, AVLNode ** );
	AVL_RES	grewLeft		( AVLNode ** );
	AVL_RES	grewRight	( AVLNode ** );
	AVL_RES	insertNode	( AVLNode **, AVLNode *, const ADTVALUE &, const ADTVALUE & );
	AVL_RES	removeNode	( AVLNode **, AVLNode *, const ADTVALUE & );
	void		rotateLeft	( AVLNode ** );
	void		rotateRight	( AVLNode ** );
	AVL_RES	shrunkLeft	( AVLNode ** );
	AVL_RES	shrunkRight	( AVLNode ** );
	};

//
// Class - AVLIt.  Iterator for the AVL tree container.
//

class AVLIt :
	public CCLObject,										// Base class
	public IIt												// Interface
	{
	public :
	AVLIt ( AVL *, bool );								// Constructor
	AVLIt ( bool );										// Constructor

	// Run-time data
	AVL			*pTree;									// Tree object
	AVLNode		*pPos;									// Current node
	bool			bKeys;									// Iterate keys ?
	bool			bRef;										// Reference on tree ?
	ULONG			lRefCnt;									// Reference count

	// Utilities
	STDMETHOD(reset)	( void );						// Reset internal state

	// 'IIt' members
	STDMETHOD(begin)	( void );
	STDMETHOD(end)		( void );
	STDMETHOD(next)	( void );
	STDMETHOD(prev)	( void );
	STDMETHOD(read)	( ADTVALUE & );

	// 'IInnerUnknown' members
	STDMETHOD_(ULONG,InnerAddRef)		( void );
	STDMETHOD_(ULONG,InnerRelease)	( void );

	private :

	// CCL
	CCL_OBJECT_BEGIN_INT(AVLIt)
		CCL_INTF(IIt)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object
	};

//
// Class - Dictionary.  Default dictionary implementation.
//

class Dictionary :
	public CCLObject,										// Base class
	public IDictionary,									// Interface
	public ICloneable										// Interface
	{
	public :
	Dictionary ( void );									// Constructor
	virtual ~Dictionary ( void ) {}					// Destructor

	// Run-time data
//	KeyValArray	*pCnt;									// Contained object
	AVL			*pCnt;									// Contained object

	// 'IDictionary' members
	STDMETHOD(keys)	( IIt ** );
	STDMETHOD(load)	( const ADTVALUE &, ADTVALUE & );
	STDMETHOD(store)	( const ADTVALUE &, const ADTVALUE & );

	// 'IContainer' members
	STDMETHOD(clear)		( void );
	STDMETHOD(copyTo)		( IContainer * );
	STDMETHOD(isEmpty)	( void );
	STDMETHOD(iterate)	( IIt ** );
	STDMETHOD(remove)		( const ADTVALUE & );
	STDMETHOD(size)		( U32 * );

	// 'ICloneable' members
	STDMETHOD(clone)		( IUnknown ** );

	// CCL
	CCL_OBJECT_BEGIN(Dictionary)
		CCL_INTF(IDictionary)
		CCL_INTF(IContainer)
		CCL_INTF(ICloneable)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	private :
	};

//
// Class - List.  Class to contain a simple list of values.
//

class List :
	public CCLObject,										// Base class
	public IDictionary,									// Interface
	public IList,											// Interface
	public ICloneable										// Interface
	{
	public :
	List ( void );											// Constructor

	// 'IDictionary' members
	STDMETHOD(keys)		( IIt ** );
	STDMETHOD(load)		( const ADTVALUE &, ADTVALUE & );
	STDMETHOD(store)		( const ADTVALUE &, const ADTVALUE & );

	// 'IContainer' members
	STDMETHOD(clear)		( void );
	STDMETHOD(copyTo)		( IContainer * );
	STDMETHOD(iterate)	( IIt ** );
	STDMETHOD(isEmpty)	( void );
	STDMETHOD(remove)		( const ADTVALUE & );
	STDMETHOD(size)		( U32 * );

	// 'IList' memebers
	STDMETHOD(write)		( const ADTVALUE & );

	// 'ICloneable' members
	STDMETHOD(clone)		( IUnknown ** );

	// CCL
	CCL_OBJECT_BEGIN(List)
		CCL_INTF(IDictionary)
		CCL_INTF(IList)
		CCL_INTF_FROM(IContainer,IList)
		CCL_INTF(ICloneable)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	private :

	// Run-time data
	ADTVALUE			*pVals;								// Value array
	U32				iCnt,iAlloc;						// Value/allocated count
	sysCS				cs;									// Thread safety

	// Iterator
	friend class ListIt;

	// Internal utilities
	HRESULT	addObject	( const ADTVALUE & );	// Add object to list

	};

//
// Class - ListIt.  Iterator for the vector container.
//

class ListIt :
	public CCLObject,										// Base class
	public IIt												// Interface
	{
	public :
	ListIt ( List *, bool );							// Constructor

	// 'IIt' members
	STDMETHOD(begin)	( void );
	STDMETHOD(end)		( void );
	STDMETHOD(next)	( void );
	STDMETHOD(prev)	( void );
	STDMETHOD(read)	( ADTVALUE & );

	private :

	// Run-time data
	List			*cont;									// List class
	int			iIdx;										// Iteration index
	bool			bKeys;									// Iterating 'keys'

	// CCL
	CCL_OBJECT_BEGIN_INT(ListIt)
		CCL_INTF(IIt)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object
	};

//
// Class - Queue.  Class to contain a queue of values.
//

class Queue :
	public CCLObject,										// Base class
	public IList											// Interface
	{
	public :
	Queue ( void );										// Constructor

	// 'IContainer' members
	STDMETHOD(clear)		( void );
	STDMETHOD(copyTo)		( IContainer * );
	STDMETHOD(iterate)	( IIt ** );
	STDMETHOD(isEmpty)	( void );
	STDMETHOD(remove)		( const ADTVALUE & );
	STDMETHOD(size)		( U32 * );

	// 'IList' members
	STDMETHOD(write)		( const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(Queue)
		CCL_INTF(IList)
		CCL_INTF(IContainer)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	private :

	// Run-time data
	adtValue		**pQ;										// Queue
	U32			fidx,bidx;								// Positions
	U32			qsize;									// # of entries
	sysCS			cs;										// Thread safety

	// Iterator
	friend class QueueIt;

	// Internal utilities
	HRESULT	resize	( U32 );							// Resize array
	};

//
// Class - QueueIt.  Iterator for the queue container.
//

class QueueIt :
	public CCLObject,										// Base class
	public IIt												// Interface
	{
	public :
	QueueIt ( Queue * );									// Constructor
	virtual ~QueueIt ( void );							// Destructor

	// 'IIt' members
	STDMETHOD(begin)	( void );
	STDMETHOD(end)		( void );
	STDMETHOD(next)	( void );
	STDMETHOD(prev)	( void );
	STDMETHOD(read)	( ADTVALUE & );

	private :

	// Run-time data
	Queue		*queue;									// Queue class

	// CCL
	CCL_OBJECT_BEGIN_INT(QueueIt)
		CCL_INTF(IIt)
	CCL_OBJECT_END()
	};

//
// Class - Stack.  Class to maintain a LIFO queue.
//

class Stack :
	public CCLObject,										// Base class
	public IList											// Interface
	{
	public :
	Stack ( void );										// Constructor

	// 'IContainer' members
	STDMETHOD(clear)		( void );
	STDMETHOD(copyTo)		( IContainer * );
	STDMETHOD(iterate)	( IIt ** );
	STDMETHOD(isEmpty)	( void );
	STDMETHOD(remove)		( const ADTVALUE & );
	STDMETHOD(size)		( U32 * );

	// 'IList' memebers
	STDMETHOD(write)		( const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(Stack)
		CCL_INTF(IList)
		CCL_INTF(IContainer)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	private :

	// Run-time data
	ADTVALUE		*pS;										// Stack
	U32			ssize,salloc;							// Items/allocated
	sysCS			cs;										// Thread safety

	// Iterator
	friend class StackIt;

	// Internal utilities
	HRESULT	resize	( U32 );							// Resize array

	};

//
// Class - StackIt.  Iterator for the stack container.
//

class StackIt :
	public CCLObject,										// Base class
	public IIt												// Interface
	{
	public :
	StackIt ( Stack * );									// Constructor
	virtual ~StackIt ( void );							// Destructor

	// 'IIt' members
	STDMETHOD(begin)	( void );
	STDMETHOD(end)		( void );
	STDMETHOD(next)	( void );
	STDMETHOD(prev)	( void );
	STDMETHOD(read)	( ADTVALUE & );

	private :

	// Run-time data
	Stack		*stack;									// Stack class

	// CCL
	CCL_OBJECT_BEGIN_INT(StackIt)
		CCL_INTF(IIt)
	CCL_OBJECT_END()
	};

/////////
// Nodes
/////////

//
//!	\brief Iterate values in a container or keys and values in a dictionary
//!	\nodetag Dictionary List Container
//

class Iterate :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Iterate ( void );										// Constructor

	// Run-time data
	IContainer		*pCnt;								// Container
	IDictionary		*pDct;								// Container
	IIt				*pIt;									// Iterator
	adtInt			iZ;									// Zero
	adtIUnknown		unkV;									// Internal variable
	adtValue			vValue,vKey;						// Key/value

	// CCL
	CCL_OBJECT_BEGIN(Iterate)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Specify the container from which to iterate.
	DECLARE_RCP(Container)
	//!	\brief Emit the first key/value pair in the container
	DECLARE_CON(First)
	//!	\brief Emit the last key/value pair in the container
	DECLARE_CON(Last)
	//!	\brief Emit the next key/value pair in the container
	DECLARE_CON(Next)
	//!	\brief Emit the previous key/value pair in the container
	DECLARE_CON(Previous)
	//!	\brief The latest key from the dictionary
	DECLARE_EMT(Key)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Container)
		DEFINE_CON(First)
		DEFINE_CON(Last)
		DEFINE_CON(Next)
		DEFINE_CON(Previous)
		DEFINE_EMT(Key)
	END_BEHAVIOUR_NOTIFY()

	};

//
//!	\brief A node to manage a set of keys/values in a dictionary
//!	\nodetag Dictionary 
//

class Keys :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Keys ( void );											// Constructor

	// Run-time data
	IDictionary	*pVals;									// Cached values
	IDictionary	*pDict;									// Current dictionary
	IContainer	*pKeys;									// Key context
	IDictionary	*pRcps;									// Receptors
	IDictionary	*pKeysOut;								// Output key names
	adtIUnknown	unkV;										// Internal variable
	adtValue		vRecep;									// Internal variable
	bool			bAuto;									// Auto store ?

	// CCL
	CCL_OBJECT_BEGIN(Keys)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	//! \name Connections 
	//@{
	//!	\brief Clear any values that have been cached.
	DECLARE_RCP(Clear)
	//!	\brief Copy the list of keys from the provided dictionary to the active dictionary.
	DECLARE_CON(Copy)
	//!	\brief Set the active dictionary
	DECLARE_RCP(Dictionary)
	//!	\brief Load the values for the active list of keys.
	DECLARE_CON(Load)
	//!	\brief Store the list of cached values from the list of keys.
	DECLARE_CON(Store)
	//!	\brief Contains a key that was not found in the dictionary during a load.
	DECLARE_EMT(NotFound)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Clear)
		DEFINE_CON(Copy)
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Load)
		DEFINE_CON(Store)
		DEFINE_EMT(NotFound)
	END_BEHAVIOUR_NOTIFY()

	};

//
//!	\brief Load a value associated with the active key
//!	\nodetag Dictionary 
//

class Load :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Load ( void );											// Constructor

	// Run-time data
	IDictionary		*pDct;								// Parameters
	adtValue			vKey;									// Parameters
	adtValue			vL;									// Internal variable
	adtIUnknown		unkV;									// Internal variable

	// CCL
	CCL_OBJECT_BEGIN(Load)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Set the active dictionary
	DECLARE_RCP(Dictionary)
	//!	\brief Set the active key
	DECLARE_RCP(Key)
	//!	\brief Load and emit the value associated with the active key
	DECLARE_CON(Fire)
	//!	\brief Emits key when not found in active dictionary.
	DECLARE_EMT(NotFound)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Key)
		DEFINE_CON(Fire)
		DEFINE_EMT(NotFound)
	END_BEHAVIOUR_NOTIFY()

	};

//
//!	\brief Remove a value from a container
//!	\nodetag Dictionary Container
//

class Remove :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Remove ( void );										// Constructor

	// Run-time data
	IContainer		*pCont;								// Parameters
	adtValue			vKey;									// Parameters
	adtIUnknown		unkV;									// Internal variable

	// CCL
	CCL_OBJECT_BEGIN(Remove)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Set the active container
	DECLARE_RCP(Container)
	//!	\brief Remove all values from the container
	DECLARE_RCP(Clear)
	//!	\brief Set the active key/value to remove
	DECLARE_RCP(Key)
	//!	\brief Remove the specified key/value from the container
	DECLARE_CON(Fire)
	//!	\brief Emits key when not found in container.
	DECLARE_EMT(NotFound)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Container)
		DEFINE_RCP(Clear)
		DEFINE_RCP(Key)
		DEFINE_CON(Fire)
		DEFINE_EMT(NotFound)
	END_BEHAVIOUR_NOTIFY()
	};

//
//!	\brief Obtain statistics about a container
//!	\nodetag Dictionary Container
//

class Stat :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Stat ( void );											// Constructor

	// Run-time data
	IContainer	*pCnt;									// Container
	adtIUnknown	unkV;										// Internal variable
	adtInt		iV;										// Internal variable

	// CCL
	CCL_OBJECT_BEGIN(Stat)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Set the active container
	DECLARE_RCP(Container)
	//!	\brief Compute statistics and emit the container
	DECLARE_CON(Fire)
	//!	\brief The number of items in the container
	DECLARE_EMT(Count)
	//!	\brief Signal if there are no items in the container
	DECLARE_EMT(Empty)
	//!	\brief Signal if there are items in the container
	DECLARE_EMT(NotEmpty)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Container)
		DEFINE_CON(Fire)
		DEFINE_EMT(Count)
		DEFINE_EMT(Empty)
		DEFINE_EMT(NotEmpty)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
//!	\brief Store a key/value into a dictionary
//!	\nodetag Dictionary 
//

class Store :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Store ( void );										// Constructor

	// Run-time data
	IDictionary		*pDct;								// Parameters
	adtValue			vKey,vValue;						// Parameters

	// CCL
	CCL_OBJECT_BEGIN(Store)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Set the active dictionary
	DECLARE_RCP(Dictionary)
	//!	\brief Set the active key
	DECLARE_RCP(Key)
	//!	\brief Set the active value
	DECLARE_RCP(Value)
	//!	\brief Store the active or provided value under the active key
	DECLARE_CON(Fire)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Key)
		DEFINE_RCP(Value)
		DEFINE_CON(Fire)
	END_BEHAVIOUR_NOTIFY()

	};

//
//!	\brief Append a value to a list.
//!	\nodetag List
//

class Write :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Write ( void );										// Constructor

	// Run-time data
	IList			*pLst;									// List
	adtValue		vValue;									// Active value
	adtIUnknown	unkV;										// Internal variable

	// CCL
	CCL_OBJECT_BEGIN(Write)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Set the active list
	DECLARE_RCP(List)
	//!	\brief Set the active value
	DECLARE_RCP(Value)
	//!	\brief Append the active or provided value to the active list
	DECLARE_CON(Fire)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(List)
		DEFINE_RCP(Value)
		DEFINE_CON(Fire)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

#endif

