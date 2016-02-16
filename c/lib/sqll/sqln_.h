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
	STDMETHOD(getValue)	( adtValue & );
	STDMETHOD(setValue)	( const adtValue & );

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
	public INodeBehaviour								// Interface
	{
	public :
	SQLConnection ( void );								// Constructor

	// Run-time data
	IReceptor		*prConn,*prFire;					// Receptors
	IEmitter			*peEnv,*peConn;					// Emitters
	adtString		sConn;								// Connection string
	#ifdef			USE_ODBC
	SQLHANDLE		hSQLEnv;								// Environment handle
	#endif
	#ifdef			USE_OLEDB
	#endif

	// CCL
	CCL_OBJECT_BEGIN(SQLConnection)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_EMITTER	(OnConnect,peConn)
	END_BEHAVIOUR()

	private :

	};

//
// Class - SQLCreateDatabase.  Node to create a new database.
//

class SQLCreateDatabase :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQLCreateDatabase ( void );						// Constructor

	// Run-time data
	IReceptor		*prLoc,*prFire;					// Receptors
	IEmitter			*peFire,*peErr;					// Emitters
	adtString		sDriver,sLoc;						// Parameters

	// CCL
	CCL_OBJECT_BEGIN(SQLCreateDatabase)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()

	// CreateDatabases
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Location,prLoc)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_EMITTER	(OnError,peErr)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	};

#ifdef	USE_OLEDB

//
// Class - SQLDelete.  Node to perform record deletion.
//

class SQLDelete :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQLDelete ( void );									// Constructor

	// Run-time data
	IReceptor		*prConn,*prCons,*prF,*prTbl;	// Receptors
	IEmitter			*peFire;								// Emitters
	IUnknown			*pConn;								// Connection object
	adtString		sTableName;							// Table name
	IADTContainer	*pCons;								// Constraints
	IADTInIt			*pConsIn;							// Constraints
	DBBINDING		*pbCons;								// Constraint bindings
	U32				szCons;								// # of constraints
	IMemoryMapped	*pSQLBfr;							// SQL buffer
	WCHAR				*pwSQLBfr;							// SQL buffer
	IMemoryMapped	*pBfr;								// Internal data buffer
	U32				uBfrSz;								// Current data buffer size

	// CCL
	CCL_OBJECT_BEGIN(SQLDelete)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Constraints,prCons)
		DECLARE_RECEPTOR	(Fire,prF)
		DECLARE_RECEPTOR	(Table,prTbl)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

//
// Class - SQLQuery.  Node to perform a generic SQL query.
//

class SQLQuery :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
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
	IADTContainer	*pCons;								// Constraints
	IADTInIt			*pConsIn;							// Constraints
	DBBINDING		*pbCons;								// Constraint bindings
	U32				szCons;								// # of constraints
	IADTInIt			*pFldsIn;							// Fields to 'select'
	adtBool			bSort;								// Sort result ?
	adtBool			bCount;								// Count query only ?
	IMemoryMapped	*pQryBfr;							// Query buffer
	WCHAR				*pwQryBfr;							// Query buffer
	IMemoryMapped	*pBfr;								// Internal data buffer
	U32				uBfrSz;								// Current data buffer size
	IADTDictionary	*pJoin;								// Join information
	adtString		sSort;								// Sort field ?
	adtInt			iCount;								// Max. query count

	// CCL
	CCL_OBJECT_BEGIN(SQLQuery)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Constraints,prCons)
		DECLARE_RECEPTOR	(Count,prCnt)
		DECLARE_RECEPTOR	(Distinct,prDis)
		DECLARE_RECEPTOR	(Fields,prFlds)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_RECEPTOR	(Join,prJoin)
		DECLARE_RECEPTOR	(Sort,prSort)
		DECLARE_RECEPTOR	(Table,prTbl)
		DECLARE_EMITTER	(OnFail,peFail)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

//
// Class - SQLRecordEnum.  Node to perform record enumeration on a result set.
//

class SQLRecordEnum :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
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
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Context,prCtx)
		DECLARE_RECEPTOR	(Count,prCnt)
		DECLARE_RECEPTOR	(Next,prNext)
		DECLARE_RECEPTOR	(Position,prPos)
		DECLARE_EMITTER	(OnEnd,peEnd)
		DECLARE_EMITTER	(OnFire,peFire)
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
	public INodeBehaviour								// Interface
	{
	public :
	SQL2Table ( void );									// Constructor

	// Run-time data
	IReceptor			*prCols,*prConn,*prFlds;	// Receptors
	IReceptor			*prFire;							// Receptors
	IEmitter				*peFire,*peCols;				// Emitters
	IADTDictionary		*pFlds;							// Table fields
	IUnknown				*pConn;							// Connection object
	adtString			sTableName;						// Table name
	adtBool				bRemove;							// Remove unused fields ?

	// CCL
	CCL_OBJECT_BEGIN(SQL2Table)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Columns,prCols)
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Fields,prFlds)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_EMITTER	(OnColumns,peCols)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT fieldsAdd		( IUnknown *, DBID *, IADTDictionary * );
	};

//
// Class - SQLTableWrite.  Node to write to a table.
//

class SQLTableWrite :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQLTableWrite ( void );								// Constructor

	// Run-time data
	IReceptor			*prConn,*prFlds,*prFire;	// Receptors
	IEmitter				*peFire;							// Emitters
	IUnknown				*pConn;							// Connection object
	IADTDictionary		*pFlds;							// Current fields
	IMemoryMapped		*pQryBfr;						// Internal query buffer
	WCHAR					*pwQryBfr;						// Internal query buffer
	IMemoryMapped		*pBfr;							// Internal buffer
	U32					uBfrSz;							// Current buffer size

	// CCL
	CCL_OBJECT_BEGIN(SQLTableWrite)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Fields,prFlds)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	};

// Prototypes
HRESULT OLEDBAppendConstraints	( IADTContainer *, WCHAR * );
HRESULT OLEDBApplyConstraints		( IADTContainer *, IMemoryMapped *, DBBINDING *, U32 * );
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
	adtValueImpl		sData;							// Column data
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
	public INodeBehaviour								// Interface
	{
	public :
	SQLDelete ( void );									// Constructor

	// Run-time data
	IReceptor		*prConn,*prCons,*prF,*prTbl;	// Receptors
	IEmitter			*peFire;								// Emitters
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	IADTContainer	*pCons;								// Constraints
	IADTInIt			*pConsIn;							// Constraints
	SQLCol			*pvCons;								// Constraint values
	U32				szCons;								// # of constraints
	IMemoryMapped	*pSQLBfr;							// SQL buffer
	WCHAR				*pwSQLBfr;							// SQL buffer

	// CCL
	CCL_OBJECT_BEGIN(SQLDelete)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Constraints,prCons)
		DECLARE_RECEPTOR	(Fire,prF)
		DECLARE_RECEPTOR	(Table,prTbl)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

//
// Class - SQLQuery.  Node to perform a generic SQL query.
//

class SQLQuery :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQLQuery ( void );									// Constructor

	// Run-time data
	IReceptor		*prConn,*prCons,*prCnt;			// Receptors
	IReceptor		*prDis,*prFlds,*prFire;			// Receptors
	IReceptor		*prJoin,*prSort,*prTbl;			// Receptors
	IEmitter			*peFire,*peFail;					// Emitters
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	adtBool			bDistinct;							// Distinct record result ?
	IADTContainer	*pCons;								// Constraints
	IADTInIt			*pConsIn;							// Constraints
	SQLCol			*pvCons;								// Constraint values
	U32				szCons;								// # of constraints
	IADTInIt			*pFldsIn;							// Fields to 'select'
	adtBool			bSort;								// Sort result ?
	adtBool			bCount;								// Count query only ?
	IMemoryMapped	*pQryBfr;							// Query buffer
	WCHAR				*pwQryBfr;							// Query buffer
	IADTContainer	*pJoin;								// Join information
	adtString		sSort;								// Sort field ?
	adtInt			iCount;								// Max. query count

	// CCL
	CCL_OBJECT_BEGIN(SQLQuery)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Constraints,prCons)
		DECLARE_RECEPTOR	(Count,prCnt)
		DECLARE_RECEPTOR	(Distinct,prDis)
		DECLARE_RECEPTOR	(Fields,prFlds)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_RECEPTOR	(Join,prJoin)
		DECLARE_RECEPTOR	(Sort,prSort)
		DECLARE_RECEPTOR	(Table,prTbl)
		DECLARE_EMITTER	(OnFail,peFail)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

//
// Class - SQLQueryKey.  Node to perform an SQL query for a specific key.
//

class SQLQueryKey :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQLQueryKey ( void );								// Constructor

	// Run-time data
	IReceptor		*prConn,*prKey,*prFire;			// Receptors
	IEmitter			*peFire;								// Emitters
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
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Key,prKey)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_EMITTER	(OnFire,peFire)
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
	public INodeBehaviour								// Interface
	{
	public :
	SQLQueryRange ( void );								// Constructor

	// Run-time data
	IReceptor		*prConn,*prFire,*prL,*prR;		// Receptors
	IEmitter			*peFire;								// Emitters
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	adtString		sKey;									// Range key
	IADTInIt			*pFlds;								// Field names to query
	adtValueImpl	vLeft,vRight;						// Desired range
	SQLINTEGER		uLeftSz,uRightSz;					// Column sizes
	adtString		sQuery;								// Current query string
	adtBool			bSort;								// Sort result ?
	adtBool			bCount;								// Count query only ?

	// CCL
	CCL_OBJECT_BEGIN(SQLQueryRange)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_RECEPTOR	(Left,prL)
		DECLARE_RECEPTOR	(Right,prR)
		DECLARE_EMITTER	(OnFire,peFire)
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
	public INodeBehaviour								// Interface
	{
	public :
	SQLRecordEnum ( void );								// Constructor

	// Run-time data
	IReceptor		*prCtx,*prCnt,*prNext,*prPos;	// Receptors
	IEmitter			*peFire,*peEnd;					// Emitters
	IHaveValue		*pStmt;								// Statement object
	SQLHANDLE		hStmt;								// Statement
	SQLCol			*pCols;								// Column info.
	SQLSMALLINT		ColumnCount;						// Current column count
	bool				bEnd;									// Enumeration done ?
	IMemoryMapped	*pBfr;								// Internal object loading buffer
	U32				uBfrSz;								// Current buffer size
	IParseStm		*pParse;								// Parser

	// CCL
	CCL_OBJECT_BEGIN(SQLRecordEnum)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Context,prCtx)
		DECLARE_RECEPTOR	(Count,prCnt)
		DECLARE_RECEPTOR	(Next,prNext)
		DECLARE_RECEPTOR	(Position,prPos)
		DECLARE_EMITTER	(OnEnd,peEnd)
		DECLARE_EMITTER	(OnFire,peFire)
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
	public INodeBehaviour								// Interface
	{
	public :
	SQLTableCreate ( void );							// Constructor

	// Run-time data
	IReceptor			*prConn,*prDef,*prFire;		// Receptors
	IEmitter				*peFire;							// Emitters
	IADTDictionary		*pDef;							// Table definition
	IHaveValue			*pConn;							// Connection object
	SQLHANDLE			hConn;							// Connection
	IMemoryMapped		*pBfr;							// Internal buffer
	WCHAR					*pwBfr;							// Internal buffer

	// CCL
	CCL_OBJECT_BEGIN(SQLTableCreate)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Definition,prDef)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT fieldsAdd		( SQLHANDLE, adtString &, IADTDictionary * );
	HRESULT primaryKey	( SQLHANDLE, adtString &, IADTInIt * );
	HRESULT tableCreate	( SQLHANDLE, adtString & );

	};

//
// Class - SQLTableWrite.  Node to write to a table.
//

class SQLTableWrite :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQLTableWrite ( void );								// Constructor

	// Run-time data
	IReceptor			*prConn,*prFlds,*prFire;	// Receptors
	IReceptor			*prTbl;							// Receptors
	IEmitter				*peFire;							// Emitters
	IHaveValue			*pConn;							// Connection object
	SQLHANDLE			hConn;							// Connection
	IADTDictionary		*pFlds;							// Current fields
	IMemoryMapped		*pBfr;							// Internal buffer
	WCHAR					*pwBfr;							// Internal buffer
	IMemoryMapped		*pStmBfr;						// Internal buffer
	VOID					*pvStmBfr;						// Internal buffer
	adtString			sTableName;						// Table name

	// CCL
	CCL_OBJECT_BEGIN(SQLTableWrite)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Fields,prFlds)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_RECEPTOR	(TableName,prTbl)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT bindVariant	( SQLHANDLE, U32, SQLCol * );
	HRESULT putData		( SQLHANDLE, IUnknown * );
	HRESULT streamObj		( IUnknown *, IUnknown ** );
//	HRESULT fieldsAdd		( SQLHANDLE, adtString &, IADTDictionary * );
//	HRESULT tableWrite	( SQLHANDLE, adtString & );

	};

//
// Class - SQL2Index.  Node to handle an index on a table.
//

class SQL2Index :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQL2Index ( void );									// Constructor

	// Run-time data
	IReceptor			*prConn,*prCr,*prFlds;		// Receptors
	IReceptor			*prTbl;							// Receptors
	IEmitter				*peFire;							// Emitters
	IADTContainer		*pFlds;							// Index definition
	IHaveValue			*pConn;							// Connection object
	SQLHANDLE			hConn;							// Connection
	IMemoryMapped		*pQryBfr;						// Internal buffer
	WCHAR					*pwQryBfr;						// Internal buffer
	adtString			sTableName;						// Table name
	adtString			sIndexName;						// Index name

	// CCL
	CCL_OBJECT_BEGIN(SQL2Index)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Create,prCr)
		DECLARE_RECEPTOR	(Fields,prFlds)
		DECLARE_RECEPTOR	(TableName,prTbl)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
//	HRESULT fieldsAdd		( SQLHANDLE, adtString &, IADTDictionary * );
//	HRESULT primaryKey	( SQLHANDLE, adtString &, IADTInIt * );
//	HRESULT tableCreate	( SQLHANDLE, adtString & );

	};

//
// Class - SQL2Table.  Node to handle existence of a table.
//

class SQL2Table :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQL2Table ( void );									// Constructor

	// Run-time data
	IReceptor			*prCols,*prConn,*prFlds;	// Receptors
	IReceptor			*prFire,*prTbl;				// Receptors
	IEmitter				*peFire,*peCols;				// Emitters
	IADTDictionary		*pFlds;							// Table fields
	IHaveValue			*pConn;							// Connection object
	SQLHANDLE			hConn;							// Connection
	IMemoryMapped		*pQryBfr;						// Internal buffer
	WCHAR					*pwQryBfr;						// Internal buffer
	adtString			sTableName;						// Table name
	adtBool				bRemove;							// Remove unused fields ?

	// CCL
	CCL_OBJECT_BEGIN(SQL2Table)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Columns,prCols)
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Fields,prFlds)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_RECEPTOR	(TableName,prTbl)
		DECLARE_EMITTER	(OnColumns,peCols)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities
	HRESULT fieldsAdd		( SQLHANDLE, adtString &, IADTDictionary * );
	HRESULT fieldsRemove	( SQLHANDLE, adtString &, IADTDictionary * );

	};

//
// Class - SQLUpdate.  Node to perform a table update.
//

class SQLUpdate :
	public CCLObject,										// Base class
	public INodeBehaviour								// Interface
	{
	public :
	SQLUpdate ( void );									// Constructor

	// Run-time data
	IReceptor		*prConn,*prCons,*prFlds;		// Receptors
	IReceptor		*prFire,*prTbl;					// Receptors
	IEmitter			*peFire,*peFail;					// Emitters
	IHaveValue		*pConn;								// Connection object
	SQLHANDLE		hConn;								// Connection
	adtString		sTableName;							// Table name
	IADTContainer	*pCons;								// Constraints
	IADTInIt			*pConsIn;							// Constraints
	SQLCol			*pvCons;								// Constraint values
	U32				szCons;								// # of constraints
	IADTDictionary	*pFlds;								// Current fields
	IMemoryMapped	*pQryBfr;							// Query buffer
	WCHAR				*pwQryBfr;							// Query buffer

	// CCL
	CCL_OBJECT_BEGIN(SQLUpdate)
		CCL_INTF(INodeBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	// Connections
	BEGIN_BEHAVIOUR()
		DECLARE_RECEPTOR	(Connection,prConn)
		DECLARE_RECEPTOR	(Constraints,prCons)
		DECLARE_RECEPTOR	(Fields,prFlds)
		DECLARE_RECEPTOR	(Fire,prFire)
		DECLARE_RECEPTOR	(Table,prTbl)
		DECLARE_EMITTER	(OnFail,peFail)
		DECLARE_EMITTER	(OnFire,peFire)
	END_BEHAVIOUR()

	private :

	// Internal utilities

	};

// Prototypes
HRESULT SQLBindVariantParam	( SQLHANDLE, U32, adtValue *, SQLINTEGER * );
HRESULT SQLHandleError			( SQLSMALLINT, SQLHANDLE, SQLRETURN );
HRESULT SQLVtToSQLC				( VARTYPE, SQLSMALLINT * );
HRESULT SQLVtToSQLType			( VARTYPE, SQLSMALLINT * );
HRESULT SQLStringItLen			( IADTInIt *, U32 *, U32 * );

#endif

#endif

