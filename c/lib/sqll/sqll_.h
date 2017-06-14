////////////////////////////////////////////////////////////////////////
//
//									SQLN_.H
//
//		Internal include file for the SQL database node library
//
////////////////////////////////////////////////////////////////////////

#ifndef	SQLL__H
#define	SQLL__H

#include "sqll.h"

// SQLite
//#include "sqlite3.h"

// ODBC
#include <sqlext.h>

// Using "PDLL" to avoid having to link to possibly missing DLLs
//#include "../sysl/PDll.h"

/*
#define	USE_ODBC
//#define	USE_OLEDB

// ODBC
#ifdef	USE_ODBC
//#include <odbcinst.h>
#endif

// OLE DB
#if	defined(USE_OLEDB)
#if	defined(UNDER_CE)
#include <ssceoledb.h>
#include <urlmon.h>
#endif
#include <oledb.h>
#include <oledberr.h>
#endif

#define	SIZE_STM_BUFFER			8192

// Class factory cache
extern CCLFactoriesCache	FactCache;

// String references
extern	adtStringSt	strRefTableName;
extern	adtStringSt	strRefKey;
extern	adtStringSt	strRefOp;
extern	adtStringSt	strRefValue;
extern	adtStringSt	strRefIndexName;
extern	adtStringSt strRefFields;
extern	adtStringSt strRefPrimKey;
extern	adtStringSt strRefRemFlds;
extern	adtStringSt strRefLeft;
extern	adtStringSt strRefRight;
extern	adtStringSt strRefCount;
extern	adtStringSt strRefSort;
extern	adtStringSt strRefFrom;
extern	adtStringSt strRefTo;
*/
/*
DEFINE_REFSTR ( strRefEnv,			L"Environment" );
DEFINE_REFSTR ( strRefConn,		L"Connection" );
DEFINE_REFSTR ( strRefTableName,	L"TableName" );
DEFINE_REFSTR ( strRefIndexName,	L"IndexName" );
DEFINE_REFSTR ( strRefFields,		L"Fields" );
DEFINE_REFSTR ( strRefPrimKey,	L"Primary Key" );
DEFINE_REFSTR ( strRefKey,			L"Key" );
DEFINE_REFSTR ( strRefLeft,		L"Left" );
DEFINE_REFSTR ( strRefRight,		L"Right" );
DEFINE_REFSTR ( strRefSort,		L"Sort" );
DEFINE_REFSTR ( strRefCount,		L"Count" );
DEFINE_REFSTR ( strRefOp,			L"Operator" );
DEFINE_REFSTR ( strRefValue,		L"Value" );
DEFINE_REFSTR ( strRefRemFlds,	L"Remove Fields" );
DEFINE_REFSTR ( strRefFrom,		L"From" );
DEFINE_REFSTR ( strRefTo,			L"To" );
*/

//
// Objects
//
/*
//
// Class - SQLite DLL function handler.
//

class SQLiteDll : public PDLL
	{
	// Default constructors
	DECLARE_CLASS(SQLiteDll);

	// Functions
	DECLARE_FUNCTION1(int, sqlite3_column_count, sqlite3_stmt * );
	DECLARE_FUNCTION2(double, sqlite3_column_double, sqlite3_stmt *, int );
	DECLARE_FUNCTION2(int, sqlite3_column_int, sqlite3_stmt *, int );
	DECLARE_FUNCTION2(const void *, sqlite3_column_text16, sqlite3_stmt *, int );
	DECLARE_FUNCTION2(int, sqlite3_column_type, sqlite3_stmt *, int );
	DECLARE_FUNCTION2(const void *, sqlite3_column_name16, sqlite3_stmt *, int );
	DECLARE_FUNCTION1(int, sqlite3_close, sqlite3 *);
	DECLARE_FUNCTION1(const void *, sqlite3_errmsg16, sqlite3 *);
	DECLARE_FUNCTION1(int, sqlite3_finalize, sqlite3_stmt *);
	DECLARE_FUNCTION2(int, sqlite3_open, const char *, sqlite3 **);
	DECLARE_FUNCTION2(int, sqlite3_open16, void *, sqlite3 **);
	DECLARE_FUNCTION5(int, sqlite3_prepare_v2, sqlite3 *, const char *,
								int, sqlite3_stmt **, const char ** );
	DECLARE_FUNCTION5(int, sqlite3_prepare16_v2, sqlite3 *, const void *,
								int, sqlite3_stmt **, const void ** );
	DECLARE_FUNCTION1(int, sqlite3_step, sqlite3_stmt *);

	};
*/
/*
//
// Class - SQLRef.  Reference counted SQL resources.
//

class SQLRef :
	public CCLObject										// Base class
	{
	public :
	SQLRef ( void );										// Constructor

	// Run-time data
//	sqlite3			*plite_db;							// SQLite 3 database
//	sqlite3_stmt	*plite_stmt;						// SQLite 3 statement

	// CCL
	CCL_OBJECT_BEGIN_INT(SQLRef)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object
	};
*/

// Error handling
#define	SQLSTMT(a,b)	SQLHandleError ( SQL_HANDLE_STMT, (a), (b) );
#define	SQLFREESTMT(a)	if ((a) != NULL) {							\
									SQLFreeHandle(SQL_HANDLE_STMT,(a));	\
									(a) = NULL; }

///////////
// Objects
///////////

//
// Class - SQLHandle.  Internal class to handle node creation/destruction.
//

class SQLHandle :
	public CCLObject,										// Base class
	public IHaveValue										// Interface
	{
	public :
	SQLHandle ( SQLSMALLINT, SQLHANDLE );			// Constructor

	public :
	SQLHANDLE	Handle;									// Active handle
	SQLSMALLINT	HandleType;								// Handle type
	SQLHANDLE	InputHandle;							// Creation input handle

	// 'IHaveValue' members
	STDMETHOD(getValue)	( ADTVALUE & );
	STDMETHOD(setValue)	( const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN_INT(SQLHandle)
		CCL_INTF(IHaveValue)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	};

//
// Nodes
//

//
// Class - Connection.  Node to establish a connection to an SQL database.
//

class Connection :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Connection ( void );									// Constructor

	// Run-time data
	adtString		strLoc;								// Connection string
	SQLHANDLE		hSQLEnv;								// Environment handle
	SQLHandle		*pConn;								// Active connection

	// CCL
	CCL_OBJECT_BEGIN(Connection)
		CCL_INTF(IBehaviour)	
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_CON(Connect)
	DECLARE_CON(Disconnect)
	DECLARE_RCP(Location)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_CON(Connect)
		DEFINE_CON(Disconnect)
		DEFINE_RCP(Location)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};


/*
//
// Class - Connection.  Node to establish a connection to an SQL database.
//

class Connection :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Connection ( void );									// Constructor

	// Run-time data
	adtString	strConn;									// Target connection string
	bool			bSqlite;									// SQL lite ?
	bool			bODBC;									// ODBC ?

	// CCL
	CCL_OBJECT_BEGIN(Connection)
		CCL_INTF(IBehaviour)	
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Location)
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Location)
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Query.  Node to perform a generic SQL query.
//

class Query :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Query ( void );										// Constructor

	// Run-time data
	SQLRef			*pConn;								// Connection object
	adtString		sTableName;							// Table name
	adtBool			bDistinct;							// Distinct record result ?
	IContainer		*pCons;								// Constraints
	IIt				*pConsIt;							// Constraints iterator
//	DBBINDING		*pbCons;								// Constraint bindings
	U32				szCons;								// # of constraints
	IIt				*pFldsIt;							// Fields iterator
	adtBool			bSort;								// Sort result ?
	adtBool			bCount;								// Count query only ?
	IMemoryMapped	*pQryBfr;							// Query buffer
	WCHAR				*pwQryBfr;							// Query buffer
	IDictionary		*pJoin;								// Join information
	adtString		sSort;								// Sort field ?
	adtInt			iCount;								// Max. query count

	// CCL
	CCL_OBJECT_BEGIN(Query)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_RCP(Constraints)
	DECLARE_RCP(Count)
	DECLARE_RCP(Distinct)
	DECLARE_RCP(Fields)
	DECLARE_CON(Fire)
	DECLARE_RCP(Join)
	DECLARE_RCP(Sort)
	DECLARE_RCP(Table)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_RCP(Constraints)
		DEFINE_RCP(Count)
		DEFINE_RCP(Distinct)
		DEFINE_RCP(Fields)
		DEFINE_CON(Fire)
		DEFINE_RCP(Join)
		DEFINE_RCP(Sort)
		DEFINE_RCP(Table)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities

	};

//
// Class - RecordEnum.  Node to perform record enumeration on a result set.
//

class RecordEnum :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	RecordEnum ( void );									// Constructor

	// Run-time data
	SQLRef			*pStmt;								// Statement object
	IDictionary		*pDct;								// Results dictionary

	// CCL
	CCL_OBJECT_BEGIN(RecordEnum)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Dictionary)
	DECLARE_EMT(End)
	DECLARE_CON(Next)
	DECLARE_RCP(Statement)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_EMT(End)
		DEFINE_CON(Next)
		DEFINE_RCP(Statement)
	END_BEHAVIOUR_NOTIFY()

	private :
	};

//
// Class - SQLCreateDatabase.  Node to create a new database.
//

class SQLCreateDatabase :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLCreateDatabase ( void );						// Constructor

	// Run-time data
	adtString		sDriver,sLoc;						// Parameters

	// CCL
	CCL_OBJECT_BEGIN(SQLCreateDatabase)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// CreateDatabases
	DECLARE_RCP(Location)
	DECLARE_CON(Fire)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Location)
		DEFINE_CON(Fire)
		DEFINE_EMT(Error)
	END_BEHAVIOUR()

	private :

	};

#ifdef	USE_OLEDB

//
// Class - SQLDelete.  Node to perform record deletion.
//

class SQLDelete :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLDelete ( void );									// Constructor

	// Run-time data
	IReceptor		*prConn,*prCons,*prF,*prTbl;	// Receptors
	IEmitter			*peFire;								// Emitters
	IUnknown			*pConn;								// Connection object
	adtString		sTableName;							// Table name
	IContainer		*pCons;								// Constraints
	IInIt				*pConsIn;							// Constraints
	DBBINDING		*pbCons;								// Constraint bindings
	U32				szCons;								// # of constraints
	IMemoryMapped	*pSQLBfr;							// SQL buffer
	WCHAR				*pwSQLBfr;							// SQL buffer
	IMemoryMapped	*pBfr;								// Internal data buffer
	U32				uBfrSz;								// Current data buffer size

	// CCL
	CCL_OBJECT_BEGIN(SQLDelete)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DEFINE_RECEPTOR	(Connection,prConn)
		DEFINE_RECEPTOR	(Constraints,prCons)
		DEFINE_RECEPTOR	(Fire,prF)
		DEFINE_RECEPTOR	(Table,prTbl)
		DEFINE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

//
// Class - SQLQuery.  Node to perform a generic SQL query.
//

class SQLQuery :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLQuery ( void );									// Constructor

	// Run-time data
	IReceptor		*prConn,*prCons,*prCnt;			// Receptors
	IReceptor		*prDis,*prFlds,*prFire;			// Receptors
	IReceptor		*prJoin,*prSort,*prTbl;			// Receptors
	IEmitter			*peFire,*peFail;					// Emitters
	IUnknown			*pConn;								// Connection object
	adtString		sTableName;							// Table name
	adtBool			bDistinct;							// Distinct record result ?
	IContainer		*pCons;								// Constraints
	IInIt				*pConsIn;							// Constraints
	DBBINDING		*pbCons;								// Constraint bindings
	U32				szCons;								// # of constraints
	IInIt				*pFldsIn;							// Fields to 'select'
	adtBool			bSort;								// Sort result ?
	adtBool			bCount;								// Count query only ?
	IMemoryMapped	*pQryBfr;							// Query buffer
	WCHAR				*pwQryBfr;							// Query buffer
	IMemoryMapped	*pBfr;								// Internal data buffer
	U32				uBfrSz;								// Current data buffer size
	IDictionary		*pJoin;								// Join information
	adtString		sSort;								// Sort field ?
	adtInt			iCount;								// Max. query count

	// CCL
	CCL_OBJECT_BEGIN(SQLQuery)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DEFINE_RECEPTOR	(Connection,prConn)
		DEFINE_RECEPTOR	(Constraints,prCons)
		DEFINE_RECEPTOR	(Count,prCnt)
		DEFINE_RECEPTOR	(Distinct,prDis)
		DEFINE_RECEPTOR	(Fields,prFlds)
		DEFINE_RECEPTOR	(Fire,prFire)
		DEFINE_RECEPTOR	(Join,prJoin)
		DEFINE_RECEPTOR	(Sort,prSort)
		DEFINE_RECEPTOR	(Table,prTbl)
		DEFINE_EMITTER	(OnFail,peFail)
		DEFINE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

//
// Class - SQLRecordEnum.  Node to perform record enumeration on a result set.
//

class SQLRecordEnum :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLRecordEnum ( void );								// Constructor

	// Run-time data
	IReceptor		*prCtx,*prCnt,*prNext,*prPos;	// Receptors
	IEmitter			*peFire,*peEnd;					// Emitters
	IRowset			*pRowset;							// Rowset object
	adtString		*psCols;								// Column names
	DBBINDING		*pbCols;								// Column bindings
	HACCESSOR		hAccessor;							// Handle to current accessor
	DBORDINAL		nColumns;							// # of columns in rowset
	bool				bEnd;									// Enumeration done ?
	IMemoryMapped	*pBfr;								// Internal object loading buffer
	U32				uBfrSz;								// Current buffer size
	IParseStm		*pParse;								// Parser
	U32				iCount,iMax;						// Count

	// CCL
	CCL_OBJECT_BEGIN(SQLRecordEnum)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DEFINE_RECEPTOR	(Context,prCtx)
		DEFINE_RECEPTOR	(Count,prCnt)
		DEFINE_RECEPTOR	(Next,prNext)
		DEFINE_RECEPTOR	(Position,prPos)
		DEFINE_EMITTER	(OnEnd,peEnd)
		DEFINE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT	prepare		( void );
//	HRESULT	getObject	( SQLHANDLE, SQLUSMALLINT, adtValue & );
	};

//
// Class - SQL2Table.  Node to handle existence of a table.
//

class SQL2Table :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQL2Table ( void );									// Constructor

	// Run-time data
	IReceptor			*prCols,*prConn,*prFlds;	// Receptors
	IReceptor			*prFire;							// Receptors
	IEmitter				*peFire,*peCols;				// Emitters
	IDictionary			*pFlds;							// Table fields
	IUnknown				*pConn;							// Connection object
	adtString			sTableName;						// Table name
	adtBool				bRemove;							// Remove unused fields ?

	// CCL
	CCL_OBJECT_BEGIN(SQL2Table)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DEFINE_RECEPTOR(Columns,prCols)
		DEFINE_RECEPTOR(Connection,prConn)
		DEFINE_RECEPTOR(Fields,prFlds)
		DEFINE_RECEPTOR(Fire,prFire)
		DEFINE_EMITTER	(OnColumns,peCols)
		DEFINE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT fieldsAdd		( IUnknown *, DBID *, IDictionary * );
	};

//
// Class - SQLTableWrite.  Node to write to a table.
//

class SQLTableWrite :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLTableWrite ( void );								// Constructor

	// Run-time data
	IReceptor			*prConn,*prFlds,*prFire;	// Receptors
	IEmitter				*peFire;							// Emitters
	IUnknown				*pConn;							// Connection object
	IDictionary		*pFlds;							// Current fields
	IMemoryMapped		*pQryBfr;						// Internal query buffer
	WCHAR					*pwQryBfr;						// Internal query buffer
	IMemoryMapped		*pBfr;							// Internal buffer
	U32					uBfrSz;							// Current buffer size

	// CCL
	CCL_OBJECT_BEGIN(SQLTableWrite)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DEFINE_RECEPTOR	(Connection,prConn)
		DEFINE_RECEPTOR	(Fields,prFlds)
		DEFINE_RECEPTOR	(Fire,prFire)
		DEFINE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	};

// Prototypes
HRESULT OLEDBAppendConstraints	( IContainer *, WCHAR * );
HRESULT OLEDBApplyConstraints		( IContainer *, IMemoryMapped *, DBBINDING *, U32 * );
HRESULT OLEDBBindVariant			( DBORDINAL, adtValue *, DBBINDING *, U32 );
HRESULT OLEDBCopyVariant			( PVOID, adtValue * );

#endif

#ifdef	USE_ODBC

//
// Class - SQLCol.  Object to contain information about a column.
//

class SQLCol
	{
	public :
	SQLCol ( void ) { uSz = 0; }

	// Run-time data
	adtString			sName;							// Column name
	adtValue				sData;							// Column data
	SQLINTEGER			uSz;								// Column size
	SQLSMALLINT			DataType;						// SQL data type
	TIMESTAMP_STRUCT	TimeStamp;						// For 'datetime' data type
	SQLLEN				StrLen_or_Ind;					// String len or indicator
	};

//
// Class - SQLDelete.  Node to perform a generic SQL deletion.
//

class SQLDelete :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLDelete ( void );									// Constructor

	// Run-time data
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	IContainer		*pCons;								// Constraints
	IIt				*pConsIn;							// Constraints
	SQLCol			*pvCons;								// Constraint values
	U32				szCons;								// # of constraints
	IMemoryMapped	*pSQLBfr;							// SQL buffer
	WCHAR				*pwSQLBfr;							// SQL buffer

	// CCL
	CCL_OBJECT_BEGIN(SQLDelete)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_RCP(Constraints)
	DECLARE_CON(Fire)
	DECLARE_RCP(Table)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_RCP(Constraints)
		DEFINE_CON(Fire)
		DEFINE_RCP(Table)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

//
// Class - SQLQuery.  Node to perform a generic SQL query.
//

class SQLQuery :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLQuery ( void );									// Constructor

	// Run-time data
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	adtBool			bDistinct;							// Distinct record result ?
	IContainer		*pCons;								// Constraints
	IIt				*pConsIn;							// Constraints
	SQLCol			*pvCons;								// Constraint values
	U32				szCons;								// # of constraints
	IIt				*pFldsIn;							// Fields to 'select'
	adtBool			bSort;								// Sort result ?
	adtBool			bCount;								// Count query only ?
	IMemoryMapped	*pQryBfr;							// Query buffer
	WCHAR				*pwQryBfr;							// Query buffer
	IContainer		*pJoin;								// Join information
	adtString		sSort;								// Sort field ?
	adtInt			iCount;								// Max. query count

	// CCL
	CCL_OBJECT_BEGIN(SQLQuery)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_RCP(Constraints)
	DECLARE_RCP(Count)
	DECLARE_RCP(Distinct)
	DECLARE_RCP(Fields)
	DECLARE_CON(Fire)
	DECLARE_RCP(Join)
	DECLARE_RCP(Sort)
	DECLARE_RCP(Table)
	DECLARE_EMT(Fail)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_RCP(Constraints)
		DEFINE_RCP(Count)
		DEFINE_RCP(Distinct)
		DEFINE_RCP(Fields)
		DEFINE_CON(Fire)
		DEFINE_RCP(Join)
		DEFINE_RCP(Sort)
		DEFINE_RCP(Table)
		DEFINE_EMT(Fail)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

//
// Class - SQLQueryKey.  Node to perform an SQL query for a specific key.
//

class SQLQueryKey :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLQueryKey ( void );								// Constructor

	// Run-time data
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	adtString		*pKeys;								// Key names
	SQLCol			*pKeyBinds;							// Active key values
	adtString		*pFlds;								// Field names
	U32				uNumKeys,uNumFlds;				// # of keys/fields
	adtString		sQuery;								// Active query string

	// CCL
	CCL_OBJECT_BEGIN(SQLQueryKey)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_RCP(Key)
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_RCP(Key)
		DEFINE_CON(Fire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT initializeFields	( void );
	HRESULT initializeKeys		( void );
	HRESULT initializeQuery		( void );

	};

//
// Class - SQLQueryRange.  Node to perform an SQL query within a range.
//

class SQLQueryRange :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLQueryRange ( void );								// Constructor

	// Run-time data
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	adtString		sKey;									// Range key
	IIt				*pFlds;								// Field names to query
	adtValue			vLeft,vRight;						// Desired range
	SQLINTEGER		uLeftSz,uRightSz;					// Column sizes
	adtString		sQuery;								// Current query string
	adtBool			bSort;								// Sort result ?
	adtBool			bCount;								// Count query only ?

	// CCL
	CCL_OBJECT_BEGIN(SQLQueryRange)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_CON(Fire)
	DECLARE_RCP(Left)
	DECLARE_RCP(Right)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_CON(Fire)
		DEFINE_RCP(Left)
		DEFINE_RCP(Right)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT	genQuery ( void );
	HRESULT	validate	( void );

	};

//
// Class - SQLRecordEnum.  Node to perform record enumeration on a result set.
//

class SQLRecordEnum :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLRecordEnum ( void );								// Constructor

	// Run-time data
	IHaveValue		*pStmt;								// Statement object
	SQLHANDLE		hStmt;								// Statement
	SQLCol			*pCols;								// Column info.
	SQLSMALLINT		ColumnCount;						// Current column count
	bool				bEnd;									// Enumeration done ?
	IMemoryMapped	*pBfr;								// Internal object loading buffer
	U32				uBfrSz;								// Current buffer size
	IStreamPersist	*pParse;								// Parser

	// CCL
	CCL_OBJECT_BEGIN(SQLRecordEnum)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Context)
	DECLARE_CON(Count)
	DECLARE_RCP(Next)
	DECLARE_RCP(Position)
	DECLARE_EMT(Fire)
	DECLARE_EMT(End)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Context)
		DEFINE_CON(Count)
		DEFINE_RCP(Next)
		DEFINE_RCP(Position)
		DEFINE_EMT(Fire)
		DEFINE_EMT(End)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT	prepare		( void );
	HRESULT	getObject	( SQLHANDLE, SQLUSMALLINT, adtValue & );
	};

//
// Class - SQLTableCreate.  Node to create a table.
//

class SQLTableCreate :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLTableCreate ( void );							// Constructor

	// Run-time data
	IDictionary			*pDef;							// Table definition
	IHaveValue			*pConn;							// Connection object
	SQLHANDLE			hConn;							// Connection
	IMemoryMapped		*pBfr;							// Internal buffer
	WCHAR					*pwBfr;							// Internal buffer

	// CCL
	CCL_OBJECT_BEGIN(SQLTableCreate)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_RCP(Definition)
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_RCP(Definition)
		DEFINE_CON(Fire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT fieldsAdd		( SQLHANDLE, adtString &, IDictionary * );
	HRESULT primaryKey	( SQLHANDLE, adtString &, IIt * );
	HRESULT tableCreate	( SQLHANDLE, adtString & );

	};

//
// Class - SQLTableWrite.  Node to write to a table.
//

class SQLTableWrite :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLTableWrite ( void );								// Constructor

	// Run-time data
	IHaveValue			*pConn;							// Connection object
	SQLHANDLE			hConn;							// Connection
	IDictionary			*pFlds;							// Current fields
	IMemoryMapped		*pBfr;							// Internal buffer
	WCHAR					*pwBfr;							// Internal buffer
	IMemoryMapped		*pStmBfr;						// Internal buffer
	VOID					*pvStmBfr;						// Internal buffer
	adtString			sTableName;						// Table name

	// CCL
	CCL_OBJECT_BEGIN(SQLTableWrite)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_RCP(Fields)
	DECLARE_RCP(TableName)
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_RCP(Fields)
		DEFINE_RCP(TableName)
		DEFINE_CON(Fire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT bindVariant	( SQLHANDLE, U32, SQLCol * );
	HRESULT putData		( SQLHANDLE, IUnknown * );
	HRESULT streamObj		( IUnknown *, IUnknown ** );
//	HRESULT fieldsAdd		( SQLHANDLE, adtString &, IDictionary * );
//	HRESULT tableWrite	( SQLHANDLE, adtString & );

	};

//
// Class - SQL2Index.  Node to handle an index on a table.
//

class SQL2Index :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQL2Index ( void );									// Constructor

	// Run-time data
	IContainer			*pFlds;							// Index definition
	IHaveValue			*pConn;							// Connection object
	SQLHANDLE			hConn;							// Connection
	IMemoryMapped		*pQryBfr;						// Internal buffer
	WCHAR					*pwQryBfr;						// Internal buffer
	adtString			sTableName;						// Table name
	adtString			sIndexName;						// Index name

	// CCL
	CCL_OBJECT_BEGIN(SQL2Index)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_RCP(Create)
	DECLARE_RCP(Fields)
	DECLARE_RCP(TableName)
	DECLARE_EMT(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_RCP(Create)
		DEFINE_RCP(Fields)
		DEFINE_RCP(TableName)
		DEFINE_EMT(Fire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
//	HRESULT fieldsAdd		( SQLHANDLE, adtString &, IDictionary * );
//	HRESULT primaryKey	( SQLHANDLE, adtString &, IInIt * );
//	HRESULT tableCreate	( SQLHANDLE, adtString & );

	};

//
// Class - SQL2Table.  Node to handle existence of a table.
//

class SQL2Table :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQL2Table ( void );									// Constructor

	// Run-time data
	IDictionary			*pFlds;							// Table fields
	IHaveValue			*pConn;							// Connection object
	SQLHANDLE			hConn;							// Connection
	IMemoryMapped		*pQryBfr;						// Internal buffer
	WCHAR					*pwQryBfr;						// Internal buffer
	adtString			sTableName;						// Table name
	adtBool				bRemove;							// Remove unused fields ?

	// CCL
	CCL_OBJECT_BEGIN(SQL2Table)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_CON(Columns)
	DECLARE_RCP(Connection)
	DECLARE_RCP(Fields)
	DECLARE_CON(Fire)
	DECLARE_RCP(TableName)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Columns)
		DEFINE_RCP(Connection)
		DEFINE_RCP(Fields)
		DEFINE_CON(Fire)
		DEFINE_RCP(TableName)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT fieldsAdd		( SQLHANDLE, adtString &, IDictionary * );
	HRESULT fieldsRemove	( SQLHANDLE, adtString &, IDictionary * );

	};

//
// Class - SQLUpdate.  Node to perform a table update.
//

class SQLUpdate :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	SQLUpdate ( void );									// Constructor

	// Run-time data
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	IContainer		*pCons;								// Constraints
	IIt				*pConsIn;							// Constraints
	SQLCol			*pvCons;								// Constraint values
	U32				szCons;								// # of constraints
	IDictionary		*pFlds;								// Current fields
	IMemoryMapped	*pQryBfr;							// Query buffer
	WCHAR				*pwQryBfr;							// Query buffer

	// CCL
	CCL_OBJECT_BEGIN(SQLUpdate)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Connection)
	DECLARE_RCP(Constraints)
	DECLARE_RCP(Fields)
	DECLARE_CON(Fire)
	DECLARE_RCP(Table)
	DECLARE_EMT(Fail)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Connection)
		DEFINE_RCP(Constraints)
		DEFINE_RCP(Fields)
		DEFINE_CON(Fire)
		DEFINE_RCP(Table)
		DEFINE_EMT(Fail)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

// Prototypes
HRESULT SQLBindVariantParam	( SQLHANDLE, U32, adtValue *, SQLINTEGER * );
HRESULT SQLHandleError			( SQLSMALLINT, SQLHANDLE, SQLRETURN );
HRESULT SQLVtToSQLC				( VARTYPE, SQLSMALLINT * );
HRESULT SQLVtToSQLType			( VARTYPE, SQLSMALLINT * );
HRESULT SQLStringItLen			( IIt *, U32 *, U32 * );

#endif
*/

#endif

