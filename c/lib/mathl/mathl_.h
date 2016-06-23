////////////////////////////////////////////////////////////////////////
//
//										MATHL_.H
//
//				Implementaiton include file for the math library
//
////////////////////////////////////////////////////////////////////////

#ifndef	MATHL__H
#define	MATHL__H

// Includes
#include	"mathl.h"

// Operations
#define	MATHOP_NOP		-1

// Arithmetic
#define	MATHOP_ADD		1
#define	MATHOP_SUB		2
#define	MATHOP_MUL		3
#define	MATHOP_DIV		4
#define	MATHOP_MOD		5

// Bitwise
#define	MATHOP_AND		10
#define	MATHOP_OR		11
#define	MATHOP_XOR		12

// Vector
#define	MATHOP_DOT		20
#define	MATHOP_CROSS	21

// Trig
#define	MATHOP_COS		30
#define	MATHOP_SIN		31
#define	MATHOP_TAN		32
#define	MATHOP_ACOS		33
#define	MATHOP_ASIN		34
#define	MATHOP_ATAN		35

// Other
#define	MATHOP_ABS		40
#define	MATHOP_NORM		41
#define	MATHOP_CEIL		42
#define	MATHOP_FLOOR	43
#define	MATHOP_SQRT		44
#define	MATHOP_MIN		45
#define	MATHOP_MAX		46

// Casting
#define	MATHOP_INT		50
#define	MATHOP_LONG		51
#define	MATHOP_FLOAT	52
#define	MATHOP_DOUBLE	53
#define	MATHOP_DATE		54
#define	MATHOP_STRING	55

// Radians <-> degrees
#define	RAD_TO_DEG(a)		(a)*(180.0/3.14159265358979323846)
#define	DEG_TO_RAD(a)		(a)*(3.14159265358979323846/180.0)

//
// Class - Binary.  Node to perform a binary operation.
//

class Binary :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Binary ( void );										// Constructor

	// Run-time data
	adtValue		vL,vR;									// Parameters
	adtValue		vRes;										// Result
	int			iOp;										// Math operation

	// CCL
	CCL_OBJECT_BEGIN(Binary)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Left)
	DECLARE_RCP(Right)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Left)
		DEFINE_RCP(Right)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Counter.  Node for a Counter.
//

class Counter :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Counter ( void );										// Constructor

	// Run-time data
	adtInt			vCnt;									// Current count
	adtInt			vReset;								// Reset value

	// CCL
	CCL_OBJECT_BEGIN(Counter)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Decrement)
	DECLARE_RCP(Increment)
	DECLARE_RCP(Reset)
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Decrement)
		DEFINE_RCP(Increment)
		DEFINE_RCP(Reset)
		DEFINE_CON(Fire)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - DataBlock.  Node to manipulate a block of data.
//

class DataBlock :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	DataBlock ( void );									// Constructor

	// Run-time data
	IDictionary		*pBlk;								// Current data block
	IDictionary		*pSrc;								// Incoming data source
	adtInt			iX,iY;								// X,Y coordinates
	adtValue			vValue;								// Value

	// Utilities
	static 
	HRESULT lock ( IUnknown *, adtInt &, adtInt &, adtString &, IMemoryMapped **, void ** );

	// CCL
	CCL_OBJECT_BEGIN(DataBlock)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct(void);				// Construct object
	virtual void		destruct(void);				// Destruct object

	// Connections
	DECLARE_CON(Add)
	DECLARE_RCP(Block)
	DECLARE_EMT(Error)
	DECLARE_CON(Load)
	DECLARE_RCP(Reset)
	DECLARE_CON(Store)
	DECLARE_RCP(Source)
	DECLARE_RCP(X)
	DECLARE_RCP(Y)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Add)
		DEFINE_RCP(Block)
		DEFINE_EMT(Error)
		DEFINE_CON(Load)
		DEFINE_RCP(Reset)
		DEFINE_RCP(Source)
		DEFINE_CON(Store)
		DEFINE_RCP(X)
		DEFINE_RCP(Y)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT addRow ( IDictionary *, IDictionary *, U32 );
	};

//
// Class - Formula.  Node for a forumla.
//

class Formula :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Formula ( void );										// Constructor

	// Run-time data
	adtString	strForm;									// Forumla string
	IList			*pLstPost;								// Postfix list
	IDictionary	*pVals;									// Value dictionary
	IDictionary	*pRcps;									// Receptors
	IList			*pStkEval;								// Eval stack
	IIt			*pItStkEval;							// Eval stack iterator

	// CCL
	CCL_OBJECT_BEGIN(Formula)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Eval)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Eval)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :
	HRESULT eval		( IList *, IDictionary *, ADTVALUE & );
	HRESULT post		( IList *, IList ** );
	HRESULT symbols	( const WCHAR *, IList ** );

	};
/*
//
// Class - Function.  Node for a function.
//

class Function :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Function ( void );									// Constructor

	// Run-time data
	IReceptor	*prN,*prClr,*prPr,*prF;				// Receptors
	IEmitter		*peF,*peErr;							// Emitters
	adtString	strName;									// Function name
	IList			*pLstP;									// Parameter list

	// CCL
	CCL_OBJECT_BEGIN(Function)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Function)
	DECLARE_RCP(Clear)
	DECLARE_RCP(Param)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)

		DEFINE_RCP(Function)
		DEFINE_RCP(Clear)
		DEFINE_RCP(Param)

		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};
*/
//
// Class - Matrix3D.  Node for a 3D matrix.
//

class Matrix3D :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Matrix3D ( void );									// Constructor

	// Run-time data
	double			dScl[3],dRot[3],dTrns[3];		// Run-time data
	IDictionary		*pA,*pB,*pC;						// Matrix dictionaries
	double			dA[16],dB[16],dC[16];			// Matrix buffers
	U32				nA,nB;								// Count of elements
	adtValue			vK,vL;								// Key value
	adtIUnknown		unkV;									// Internal value
	adtInt			iV;									// Internal value
	adtDouble		vD;									// Internal value

	// CCL
	CCL_OBJECT_BEGIN(Matrix3D)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(A)
	DECLARE_CON(B)
	DECLARE_CON(C)
	DECLARE_EMT(Error)
	DECLARE_EMT(Fire)
	DECLARE_RCP(Identity)
	DECLARE_RCP(Invert)
	DECLARE_RCP(Multiply)
	DECLARE_RCP(Apply)
	DECLARE_RCP(ScaleX)
	DECLARE_RCP(ScaleY)
	DECLARE_RCP(ScaleZ)
	DECLARE_RCP(RotateX)
	DECLARE_RCP(RotateY)
	DECLARE_RCP(RotateZ)
	DECLARE_RCP(TranslateX)
	DECLARE_RCP(TranslateY)
	DECLARE_RCP(TranslateZ)
	BEGIN_BEHAVIOUR()
		// Operands/result
		DEFINE_RCP(A)
		DEFINE_RCP(B)
		DEFINE_RCP(C)

		DEFINE_EMT(Error)
		DEFINE_EMT(Fire)
		DEFINE_RCP(Identity)
		DEFINE_RCP(Invert)
		DEFINE_RCP(Multiply)

		// Scale/Rotate/Translate
		DEFINE_RCP(Apply)

		DEFINE_RCP(ScaleX)
		DEFINE_RCP(ScaleY)
		DEFINE_RCP(ScaleZ)
		DEFINE_RCP(RotateX)
		DEFINE_RCP(RotateY)
		DEFINE_RCP(RotateZ)
		DEFINE_RCP(TranslateX)
		DEFINE_RCP(TranslateY)
		DEFINE_RCP(TranslateZ)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities

	};

//
// Class - Transform3D.  Node for computing 3D transformation matrix.
//

class Transform3D :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Transform3D ( void );								// Constructor

	// Run-time data
	double			dIn[16],dOut[16],dOutP[16];	// Matrix values
	double			dSrt[9];								// Transform values
	adtDouble		vD;									// Internal value

	// CCL
	CCL_OBJECT_BEGIN(Transform3D)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(11)
	DECLARE_CON(12)
	DECLARE_CON(13)
	DECLARE_CON(21)
	DECLARE_CON(22)
	DECLARE_CON(23)
	DECLARE_CON(31)
	DECLARE_CON(32)
	DECLARE_CON(33)
	DECLARE_CON(41)
	DECLARE_CON(42)
	DECLARE_CON(43)

	DECLARE_RCP(ScaleX)
	DECLARE_RCP(ScaleY)
	DECLARE_RCP(ScaleZ)
	DECLARE_RCP(RotateX)
	DECLARE_RCP(RotateY)
	DECLARE_RCP(RotateZ)
	DECLARE_RCP(TranslateX)
	DECLARE_RCP(TranslateY)
	DECLARE_RCP(TranslateZ)
	BEGIN_BEHAVIOUR()
		// Matrix input/output
		DEFINE_CON(11)
		DEFINE_CON(12)
		DEFINE_CON(13)
		DEFINE_CON(21)
		DEFINE_CON(22)
		DEFINE_CON(23)
		DEFINE_CON(31)
		DEFINE_CON(32)
		DEFINE_CON(33)
		DEFINE_CON(41)
		DEFINE_CON(42)
		DEFINE_CON(43)

		// Scale/Rotate/Translate
		DEFINE_RCP(ScaleX)
		DEFINE_RCP(ScaleY)
		DEFINE_RCP(ScaleZ)
		DEFINE_RCP(RotateX)
		DEFINE_RCP(RotateY)
		DEFINE_RCP(RotateZ)
		DEFINE_RCP(TranslateX)
		DEFINE_RCP(TranslateY)
		DEFINE_RCP(TranslateZ)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT	emit		( void );
	HRESULT	input		( double *, int, double );
	HRESULT	update	( void );
	};

//
// Class - Unary.  Node to perform a unary operation.
//

class Unary :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Unary ( void );										// Constructor

	// Run-time data
	adtValue		vV;										// Parameter
	adtValue		vRes;										// Result
	int			iOp;										// Math operation

	// CCL
	CCL_OBJECT_BEGIN(Unary)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Vector3.  Node to perform operations on 3-vectors.
//

class Vector3 :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Vector3 ( void );										// Constructor

	// Run-time data
	double			dX[2],dY[2],dZ[2];				// Parameters

	// CCL
	CCL_OBJECT_BEGIN(Vector3)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(X0)
	DECLARE_RCP(Y0)
	DECLARE_RCP(Z0)
	DECLARE_RCP(X1)
	DECLARE_RCP(Y1)
	DECLARE_RCP(Z1)
	DECLARE_RCP(Angle)
	DECLARE_RCP(Cross)
	DECLARE_RCP(Dot)
	DECLARE_RCP(Length)
	DECLARE_RCP(Normalize)
	DECLARE_EMT(X)
	DECLARE_EMT(Y)
	DECLARE_EMT(Z)
	DECLARE_EMT(Fire)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		// Input
		DEFINE_RCP(X0)
		DEFINE_RCP(Y0)
		DEFINE_RCP(Z0)
		DEFINE_RCP(X1)
		DEFINE_RCP(Y1)
		DEFINE_RCP(Z1)

		// Operations
		DEFINE_RCP(Angle)
		DEFINE_RCP(Cross)
		DEFINE_RCP(Dot)
		DEFINE_RCP(Length)
		DEFINE_RCP(Normalize)

		// Output
		DEFINE_EMT(X)
		DEFINE_EMT(Y)
		DEFINE_EMT(Z)
		DEFINE_EMT(Fire)
		DEFINE_EMT(Error)

	END_BEHAVIOUR_NOTIFY()
	};

// Prototypes
HRESULT mathBinary	( int, const ADTVALUE &, const ADTVALUE &, ADTVALUE & );
HRESULT mathBinaryV	( int, const ADTVALUE &, const ADTVALUE &, ADTVALUE & );
HRESULT mathInv		( double *, double * );
HRESULT mathUnary		( int, const ADTVALUE &, ADTVALUE & );
HRESULT mathOp			( const WCHAR *, int * );
HRESULT mathSRT		( double [16], double [3], double [3], double [3], double [16] );

#endif
