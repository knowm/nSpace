////////////////////////////////////////////////////////////////////////
//
//									SQLN_.H
//
//		Internal include file for the SQL database node library
//
////////////////////////////////////////////////////////////////////////

/*
   Copyright (c) 2003, nSpace, LLC
   All right reserved

   Redistribution and use in source and binary forms, with or without 
   modification, are permitted provided that the following conditions
   are met:

      - Redistributions of source code must retain the above copyright 
        notice, this list of conditions and the following disclaimer.
      - nSpace, LLC as the copyright holder reserves the right to 
        maintain, update, and provide future releases of the source code.
      - Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in 
        the documentation and/or other materials provided with the 
        distribution.
      - Neither the name of nSpace, nor the names of its contributors may 
        be used to endorse or promote products derived from this software
        without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT 
   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSBILITY OF SUCH DAMAGE.
*/

#ifndef	SQLN__H
#define	SQLN__H

#include "sqln.h"
#define	USE_ODBC
//#define	USE_OLEDB

// ODBC
#ifdef	USE_ODBC
//#include <odbcinst.h>
#include <sqlext.h>
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

#ifdef	USE_ODBC

// Error handling
#define	SQLSTMT(a,b)	SQLHandleError ( SQL_HANDLE_STMT, (a), (b) );
#define	SQLFREESTMT(a)	if ((a) != NULL) {							\
									SQLFreeHandle(SQL_HANDLE_STMT,(a));	\
									(a) = NULL; }

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

#endif

//
// Class - SQLConnection.  Node to establish an SQL connection.
//

class SQLConnection :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	SQLConnection ( void );								// Constructor

	// Run-time data
	adtString		sConn;								// Connection string
	#ifdef			USE_ODBC
	SQLHANDLE		hSQLEnv;								// Environment handle
	#endif
	#ifdef			USE_OLEDB
	#endif

	// CCL
	CCL_OBJECT_BEGIN(SQLConnection)
		CCL_INTF(IBehaviour)	
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_CON(Connection)
	DECLARE_RCP(Fire)
	DECLARE_EMT(Connect)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Connection)
		DEFINE_RCP(Fire)
		DEFINE_EMT(Connect)
	END_BEHAVIOUR()

	private :

	};

//
// Class - SQLCreateDatabase.  Node to create a new database.
//

class SQLCreateDatabase :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	IContainer	*pCons;								// Constraints
	IInIt			*pConsIn;							// Constraints
	DBBINDING		*pbCons;								// Constraint bindings
	U32				szCons;								// # of constraints
	IInIt			*pFldsIn;							// Fields to 'select'
	adtBool			bSort;								// Sort result ?
	adtBool			bCount;								// Count query only ?
	IMemoryMapped	*pQryBfr;							// Query buffer
	WCHAR				*pwQryBfr;							// Query buffer
	IMemoryMapped	*pBfr;								// Internal data buffer
	U32				uBfrSz;								// Current data buffer size
	IDictionary	*pJoin;								// Join information
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
	{
	public :
	SQL2Table ( void );									// Constructor

	// Run-time data
	IReceptor			*prCols,*prConn,*prFlds;	// Receptors
	IReceptor			*prFire;							// Receptors
	IEmitter				*peFire,*peCols;				// Emitters
	IDictionary		*pFlds;							// Table fields
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
		DEFINE_RECEPTOR	(Columns,prCols)
		DEFINE_RECEPTOR	(Connection,prConn)
		DEFINE_RECEPTOR	(Fields,prFlds)
		DEFINE_RECEPTOR	(Fire,prFire)
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
	public IBehaviour										// Interface
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
	SQLINTEGER			StrLen_or_Ind;					// String len or indicator
	};

//
// Class - SQLDelete.  Node to perform a generic SQL deletion.
//

class SQLDelete :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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

#endif

