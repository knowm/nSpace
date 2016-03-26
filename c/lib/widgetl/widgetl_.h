////////////////////////////////////////////////////////////////////////
//
//										WIDGETL_.H
//
//		Implementaiton include file for the user interface widget library
//
////////////////////////////////////////////////////////////////////////

#ifndef	WIDGETL__H
#define	WIDGETL__H

// Includes
#include	"widgetl.h"
#include <commctrl.h>
#include	<GdiPlus.h>

// For internal math functions
#include "../mathl/mathl.h"
/*
// Forward dec.
class gdiWindow;
class gdiSubWindow;

// Internal messages
#define	WIDGET_USER_BASE				(WM_USER+300)
#define	WIDGET_USER_RECEIVE			(WIDGET_USER_BASE+0)
#define	WIDGET_USER_DESTROY			(WIDGET_USER_BASE+1)
#define	WIDGET_USER_EMIT				(WIDGET_USER_BASE+2)
#define	WIDGET_USER_DRAGTYPE			(WIDGET_USER_BASE+3)
#define	WIDGET_USER_DRAGENTER		(WIDGET_USER_BASE+4)
#define	WIDGET_USER_DRAGLEAVE		(WIDGET_USER_BASE+5)
#define	WIDGET_USER_DRAGOVER			(WIDGET_USER_BASE+6)
#define	WIDGET_USER_DRAGDROP			(WIDGET_USER_BASE+7)

// Radians <-> degrees
#define	RAD_TO_DEG(a)		(a)*(180.0/3.14159265358979323846)
#define	DEG_TO_RAD(a)		(a)*(3.14159265358979323846/180.0)

// Pre-generated string reference objects
#ifdef	INITGUID
adtStringSt	strRefX			( L"X" );
adtStringSt	strRefY			( L"Y" );
adtStringSt	strRefZ			( L"Z" );
adtStringSt	strRefType		( L"Type" );
adtStringSt	strRefDragDrop	( L"DragDrop" );
adtStringSt	strRefWindow	( L"Window" );
#else
extern			adtStringSt	strRefX;
extern			adtStringSt	strRefY;
extern			adtStringSt	strRefZ;
extern			adtStringSt	strRefType;
extern			adtStringSt	strRefDragDrop;
extern			adtStringSt	strRefWindow;
#endif

///////////
// Objects
///////////

//
// Class - gdiWindow.  GDI window.
//

class gdiWindow
	{
	public :
	gdiWindow ( HINSTANCE = NULL );					// Constructor
	virtual ~gdiWindow ( void );						// Destructor

	// Utilities
	virtual HRESULT construct ( HWND );				// Construct/create class

	// Base class members
	virtual HWND createWindow( HWND, PCWSTR );			// Create window
	virtual void getClassInfo( WNDCLASSW * );				// Window class information
	virtual LRESULT onMsg ( UINT, WPARAM, LPARAM );		// Process message

	// Extractors
	operator HWND()		const	{ return ui_hWnd; }
	operator HINSTANCE() const	{ return ui_hInst; }

	protected :

	// Run-time data
	HINSTANCE	ui_hInst;								// Application instance
	HWND			ui_hWnd;									// Window handle

	// Win32 window callback
	static LRESULT CALLBACK gdiWindowProc	( HWND, UINT, WPARAM, LPARAM );

	private :

	// Run-time data
	WNDCLASS		ui_wndclass;							// Window class info
	bool			bClassReg;								// Class registered ?

	};

//
// Class - gdiSubWindow.  Object for a window to sub-class an existing class.
//

class gdiSubWindow : public gdiWindow
	{
	public :
	gdiSubWindow	( HINSTANCE = NULL  );			// Constructor

	// Utilities
	virtual HRESULT construct ( HWND );				// Construct/create class

	// 'uiSubWindow' class members
	virtual HWND		createWindow( HWND );		// Create window
	virtual HRESULT	Subclass		( HWND );
	virtual HRESULT	Unsubclass	( void );

	// 'uiWindow' class members
	virtual LRESULT	onMsg ( UINT, WPARAM, LPARAM );	// Process message

	private :

	// Run-time data
	WNDPROC		ui_pSuperProc;							// Original handler

	};

//
// Structure - CTLRX.  A receive structure for a control.
//

typedef struct tagCTLRX
	{
	IReceptor		*prRxT;								// Receive 'to' receptor
	IReceptor		*prRxF;								// Receive 'from' receptor
	const WCHAR		*plRx;								// Receive location
	const ADTVALUE	*pvRx;								// Receive value
	} CTLRX;

//
// Class - gdiControl.  Base class for GDI controls.
//

class gdiControl
	{
	public :
	gdiControl ( void );									// Constructor
	virtual ~gdiControl ( void );						// Destructor

	// 'gdiControl' members
	virtual HRESULT assign			( HWND, IDictionary *, bool = true );
	virtual LRESULT forwardInput	( IDictionary *, UINT, WPARAM, LPARAM );
	virtual LRESULT onMessage		( IDictionary *, UINT, WPARAM, LPARAM );
	virtual HRESULT receive			( IDictionary *, IReceptor *, IReceptor *, const WCHAR *, const ADTVALUE & );
	virtual HRESULT transform		( HWND, IUnknown *, RECT *, POINT[4] = NULL );
	virtual HRESULT unassign		( IDictionary * );

	// Internal utilities
	LRESULT controlProc ( HWND, UINT, WPARAM, LPARAM );

	// GDI+ usage
	static	ULONG	gdiAddRef	( void );
	static	ULONG	gdiRelease	( void );

	// Extractors
	operator HWND()		const	{ return hWndCtl; }

	// Window procedure
	static LRESULT CALLBACK controlProcS	( HWND, UINT, WPARAM, LPARAM );	

	protected :

	// Run-time data
	double	dUnitSq[4][4];								// Unit square mapping
	RECT		ctlRect;										// Control bounding rectangle
	POINT		ptMap[3];									// Mapping of unit square pts. to control coordinates.

	private :

	// Run-time data
	IDictionary		*pCtls;								// Control map
	HWND				hWndCtl;								// Current control window
	};

//
// Class - uiDialog.  Dialog class.
//

class gdiDialog
	{
	public :
	gdiDialog				( HINSTANCE, PCWSTR );	// Constructor
	virtual ~gdiDialog	( void );					// Destructor

	// Utilities

	// Base class members
	virtual HWND		create		( HWND );		// Create modeless dialog
	virtual INT_PTR	doModal		( HWND );		// Create modal dialog
	virtual INT_PTR	onCommand	( WPARAM, LPARAM );// Process WM_COMMAND
	virtual INT_PTR	onMsg			( UINT, WPARAM, LPARAM );
	virtual INT_PTR	onNotify		( WPARAM, LPNMHDR );

	// Extractors
	operator HWND()		const	{ return ui_hWnd; }
	operator HINSTANCE() const	{ return ui_hInst; }

	private :

	// Run-time data
	HINSTANCE	ui_hInst;								// Application instance
	PCWSTR		ui_lpTemplate;							// Dialog template
	HWND			ui_hWnd;									// Window handle

	// Dialog callback
	static INT_PTR CALLBACK gdiDialogProc ( HWND, UINT, WPARAM, LPARAM );
	};

//
// Class - gdiButton.  Object for button controls.
//

class gdiButton : public gdiControl
	{
	public :
	gdiButton ( void );										// Constructor

	// 'gdiButton' members
	virtual LRESULT	onClicked ( IDictionary * );	// Button clicked

	// 'gdiControl' class members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );
	};

//
// Class - gdiCombobox.  Object for list controls.
//

class gdiCombobox : public gdiControl
	{
	public :
	gdiCombobox ( void );									// Constructor

	// Utilities
	HRESULT getSelected ( adtString & );

	// 'gdiCombobox' members
	virtual LRESULT onSelChange ( IDictionary *, LRESULT );

	// 'gdiControl' class members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );
	};

//
// Class - gdiEdit.  Object for edit controls.
//

class gdiEdit : public gdiControl
	{
	public :
	gdiEdit ( void );										// Constructor

	// 'gdiEdit' members
	virtual LRESULT	onFocus	( IDictionary *, bool );
	virtual LRESULT	onEnter	( IDictionary * );

	// 'gdiControl' class members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );

	// Utilities
	HRESULT getText ( adtString & );
	};

//
// Class - gdiListbox.  Object for list controls.
//

class gdiListbox : public gdiControl
	{
	public :
	gdiListbox ( void );									// Constructor

	// 'gdiListbox' members
	virtual LRESULT onSelChange ( IDictionary *, LRESULT );

	// 'gdiControl' class members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );
	};

/////////
// Nodes
/////////

//
// Class - Bezier.  Bezier rendering node.
//

class Bezier :
	public CCLObject,										// Base class
	public gdiControl,									// Base class
	public IBehaviour										// Interface
	{
	public :
	Bezier ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct,*pOwn;							// Run-time data
	HWND			hWnd;										// Window handle

	// 'gdiControl' members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );

	// CCL
	CCL_OBJECT_BEGIN(Bezier)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_CON(Open)
	DECLARE_CON(Dictionary)
	DECLARE_RCP(Owner)
	DECLARE_RCP(Vertex)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_CON(Open)
		DEFINE_CON(Dictionary)
		DEFINE_RCP(Owner)
		DEFINE_RCP(Vertex)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT	vertices ( int **, U32 * );
	};
*/

//
// Class - Button.  Node for a button.
//

class Button :
	public CCLObject,										// Base class
//	public gdiButton,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Button ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct,*pOwn;							// Run-time data
	HWND			hWnd;										// Window handle

	// 'gdiButton' members
//	virtual LRESULT	onClicked ( IDictionary * );	// Button clicked

	// CCL
	CCL_OBJECT_BEGIN(Button)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Click)
	DECLARE_CON(Close)
	DECLARE_CON(Dictionary)
	DECLARE_CON(Open)
	DECLARE_RCP(Enable)
	DECLARE_RCP(Highlight)
	DECLARE_RCP(Owner)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Click)
		DEFINE_CON(Close)
		DEFINE_CON(Dictionary)
		DEFINE_CON(Open)

		DEFINE_RCP(Enable)
		DEFINE_RCP(Highlight)
		DEFINE_RCP(Owner)

	END_BEHAVIOUR_NOTIFY()

	};

/*
//
// Class - Combobox.  Node for a combobox.
//

class Combobox :
	public CCLObject,										// Base class
	public gdiCombobox,									// Base class
	public IBehaviour										// Interface
	{
	public :
	Combobox ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct,*pOwn;							// Run-time data

	// 'gdiCombobox' members
	virtual LRESULT onSelChange ( IDictionary *, LRESULT );

	// CCL
	CCL_OBJECT_BEGIN(Combobox)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_CON(Dictionary)
	DECLARE_CON(Open)
	DECLARE_CON(Select)
	DECLARE_RCP(Enable)
	DECLARE_RCP(Owner)
	DECLARE_RCP(Add)
	DECLARE_RCP(Remove)
	DECLARE_RCP(Reset)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_CON(Dictionary)
		DEFINE_CON(Open)
		DEFINE_CON(Select)

		// General widget connectors
		DEFINE_RCP(Enable)
		DEFINE_RCP(Owner)

		// Combobox specific
		DEFINE_RCP(Add)
		DEFINE_RCP(Remove)
		DEFINE_RCP(Reset)
	END_BEHAVIOUR_NOTIFY()

	};

//
// Drag and Drop in Win32 is complicated when running in a free-threaded COM environment.
// The helper objects below are necessary so that system calls can be run in a required
//	apartment thread.
//
class DragDropSrc;										// Helper object
class DragDropDst;										// Helper object
class DragDropDstA;									// Helper object

//
// Class - DragDrop.  Drag and drop interactivity node.
//

class DragDrop :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	DragDrop ( void );									// Constructor

	// Run-time data
	IDictionary		*pOwn;								// Owner dictionary
	HWND				hWnd;									// Active window
	DragDropDst		*pdDst;								// Drag helper
	DragDropDstA	*pdDstA;								// Drag helper
	DragDropSrc		*pdSrc;								// Drag helper
	HANDLE			hevDst;								// Synchronization event

	// CCL
	CCL_OBJECT_BEGIN(DragDrop)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Drag)
	DECLARE_RCP(Owner)
	DECLARE_RCP(Start)
	DECLARE_RCP(Stop)
	DECLARE_RCP(Type)
	DECLARE_EMT(Drop)
	DECLARE_EMT(Enter)
	DECLARE_EMT(Leave)
	DECLARE_EMT(Over)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Drag)
		DEFINE_RCP(Owner)
		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)
		DEFINE_RCP(Type)
		DEFINE_EMT(Drop)
		DEFINE_EMT(Enter)
		DEFINE_EMT(Leave)
		DEFINE_EMT(Over)
	END_BEHAVIOUR_NOTIFY()

	private :
	// Internal utilities
	HRESULT start	( void );							// Start operations
	HRESULT stop	( void );							// Stop operations
	};

//
// Class - DragDropSrc.  Helper object to be a drag and drop 'source'.  It is also
//		the container for the data object being dragged.
//

class DragDropSrc :
	public CCLObject,										// Base class
	public ITickable,										// Interface
	public IDataObject,									// Interface
	public IDropSource									// Interface
	{
	public :
	DragDropSrc ( DragDrop *,							// Constructor
						const ADTVALUE & );

	// Run-time data
	DragDrop		*pParent;								// Parent object
	IThread			*pThrd;								// Worker thread
	DWORD				dwThrdId,dwThrdInstId;			// Thread Ids
	adtValue			vDrag;								// Drag value
	BOOL				bAttached;							// Threads attached
	BOOL				bDrag;								// Dragging ?
	STGMEDIUM		stgDrag;								// Storage value
//	FORMATETC		fmtDrag;								// Drag format
	adtString		sDragType;							// Drag value type
	adtString		sTmpLoc;								// Temporary file location

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// 'IDropSource' members
	STDMETHOD(QueryContinueDrag)	( BOOL, DWORD );
	STDMETHOD(GiveFeedback)			( DWORD );

	// 'IDataObject' members
	STDMETHOD(DAdvise)					( FORMATETC *, DWORD, IAdviseSink *, DWORD * );
	STDMETHOD(DUnadvise)					( DWORD );
	STDMETHOD(EnumDAdvise)				( IEnumSTATDATA ** );
	STDMETHOD(EnumFormatEtc)			( DWORD, IEnumFORMATETC ** );
	STDMETHOD(GetCanonicalFormatEtc)	( FORMATETC *, FORMATETC * );
	STDMETHOD(GetData)					( FORMATETC *, STGMEDIUM * );
	STDMETHOD(GetDataHere)				( FORMATETC *, STGMEDIUM * );
	STDMETHOD(QueryGetData)				( FORMATETC * );
	STDMETHOD(SetData)					( FORMATETC *, STGMEDIUM *, BOOL );

	// CCL
	CCL_OBJECT_BEGIN_INT(DragDropSrc)
		CCL_INTF(IDropSource)
		CCL_INTF(IDataObject)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual HRESULT	construct( void );			// Construct object
	virtual void		destruct	( void );			// Destruct object

	};

//
// Class - DragDropEnum.  Helper object to enumerate available formats.
//

class DragDropEnum :
	public CCLObject,										// Base class
	public IEnumFORMATETC								// Interface
	{
	public :
	DragDropEnum ( const adtString & );				// Constructor

	// Run-time data
	DWORD			idx;										// Format index
	adtString	sDragType;								// Dragging type

	// 'IEnumFORMATETC' members
	STDMETHOD(Next)	( ULONG, FORMATETC *, ULONG * );
	STDMETHOD(Skip)	( ULONG );
	STDMETHOD(Reset)	( void );
	STDMETHOD(Clone)	( IEnumFORMATETC ** );

	// CCL
	CCL_OBJECT_BEGIN_INT(DragDropEnum)
		CCL_INTF(IEnumFORMATETC)
	CCL_OBJECT_END()
	};

// The drag and drop destination logic needs two threaded objects.
// One is for communication with Win32 from inside an apartment thread.
// The other is for communication with the outside graph via thread messages sent
//	by the apartment thread.

//
// Class - DragDropDst.  Helper object to be a drag and drop 'destination'.
//

class DragDropDst :
	public CCLObject,										// Base class
	public ITickable,										// Interface
	public IDropTarget									// Interface
	{
	public :
	DragDropDst ( DragDrop * );						// Constructor

	// Run-time data
	DragDrop		*pParent;								// Parent object
	IThread		*pThrd;									// Worker thread
	DWORD			dwThrdId;								// Thread Id
	POINT			ptDrag;									// Current point
	DWORD			dwDragState;							// Current drag state
	DWORD			dwDragEff;								// Current drag effect
	FORMATETC	fmtDrag;									// Current drag format
	STGMEDIUM	stgDrag;									// Current drag data
	bool			bAcceptf;								// Accept enumerated format

	// 'DragDropDst' members
	HRESULT	accept		( void );					// Accept drops
	HRESULT	deny			( void );					// Deny drops

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// 'IDropTarget' members
	STDMETHOD(DragEnter)	( IDataObject *, DWORD, POINTL, DWORD * );
	STDMETHOD(DragLeave)	( void );
	STDMETHOD(DragOver)	( DWORD, POINTL, DWORD * );
	STDMETHOD(Drop)		( IDataObject *, DWORD, POINTL, DWORD * );

	// CCL
	CCL_OBJECT_BEGIN_INT(DragDropDst)
		CCL_INTF(IDropTarget)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()

	};

//
// Class - DragDropDstA.  Asynchronous destination object.
//

class DragDropDstA :
	public CCLObject,										// Base class
	public ITickable										// Interface
	{
	public :
	DragDropDstA ( DragDrop * );						// Constructor

	// Run-time data
	DragDrop		*pParent;								// Parent object
	IThread		*pThrd;									// Worker thread
	DWORD			dwThrdId;								// Thread Id
	IDictionary	*pCtxNotify;							// Parameters

	// 'DragDropDstA' members
	HRESULT	start		( void );						// Start processing
	HRESULT	stop		( void );						// Stop processing

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN_INT(DragDropDstA)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()

	private :

	// Internal utilities
	HRESULT dragEnter ( void );
	};

//
// Class - Edit.  Node for a edit/text box.
//

class Edit :
	public CCLObject,										// Base class
	public gdiEdit,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Edit ( void );											// Constructor

	// Run-time data
	IDictionary	*pDct,*pOwn;							// Run-time data
	adtString	strText;									// Current text

	// 'gdiEdit' members
	virtual LRESULT	onFocus	( IDictionary *, bool );
	virtual LRESULT	onEnter	( IDictionary * );

	// CCL
	CCL_OBJECT_BEGIN(Edit)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_CON(Dictionary)
	DECLARE_CON(Open)
	DECLARE_RCP(Enable)
	DECLARE_RCP(Owner)
	DECLARE_CON(Text)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_CON(Dictionary)
		DEFINE_CON(Open)

		// General widget connectors
		DEFINE_RCP(Enable)
		DEFINE_RCP(Owner)

		// Edit specific
		DEFINE_CON(Text)
	END_BEHAVIOUR_NOTIFY()

	};

//
// Class - FileSelect.  Node for file selection.
//

class FileSelect :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	FileSelect ( void );									// Constructor

	// Run-time data
	IDictionary	*pOwn;									// Run-time data
	adtString	strF,strT,strLoc,strExt;			// Run-time data
	adtBool		bSave;									// Save mode ?

	// CCL
	CCL_OBJECT_BEGIN(FileSelect)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Location)
	DECLARE_RCP(Owner)
	DECLARE_RCP(Select)
	DECLARE_EMT(NotSelect)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Location)
		DEFINE_RCP(Owner)
		DEFINE_RCP(Select)
		DEFINE_EMT(NotSelect)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Image.  Image rendering node.
//

class Image :
	public CCLObject,										// Base class
	public gdiControl,									// Base class
	public IBehaviour										// Interface
	{
	public :
	Image ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct,*pOwn;							// Run-time data

	// 'gdiControl' members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );

	// CCL
	CCL_OBJECT_BEGIN(Image)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_CON(Open)
	DECLARE_CON(Dictionary)
	DECLARE_RCP(Image)
	DECLARE_RCP(Matrix)
	DECLARE_RCP(Owner)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_CON(Open)
		DEFINE_CON(Dictionary)
		DEFINE_RCP(Image)
		DEFINE_RCP(Matrix)
		DEFINE_RCP(Owner)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT image ( IDictionary * );
	};

//
// Class - Label.  Node for a label.
//

class Label :
	public CCLObject,										// Base class
	public gdiControl,									// Base class
	public IBehaviour										// Interface
	{
	public :
	Label ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct,*pOwn;							// Run-time data

	// CCL
	CCL_OBJECT_BEGIN(Label)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_RCP(Dictionary)
	DECLARE_RCP(Enable)
	DECLARE_RCP(Matrix)
	DECLARE_CON(Open)
	DECLARE_RCP(Owner)
	DECLARE_RCP(Text)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Enable)
		DEFINE_RCP(Matrix)
		DEFINE_CON(Open)
		DEFINE_RCP(Owner)
		DEFINE_RCP(Text)
	END_BEHAVIOUR_NOTIFY()

	};

//
// Class - Listbox.  Node for a listbox.
//

class Listbox :
	public CCLObject,										// Base class
	public gdiListbox,									// Base class
	public IBehaviour										// Interface
	{
	public :
	Listbox ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct,*pOwn;							// Run-time data

	// 'WIDGETListbox' members
	virtual LRESULT onSelChange ( IDictionary *, LRESULT );

	// CCL
	CCL_OBJECT_BEGIN(Listbox)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_CON(Dictionary)
	DECLARE_CON(Open)
	DECLARE_CON(Select)
	DECLARE_RCP(Enable)
	DECLARE_RCP(List)
	DECLARE_RCP(Owner)
	DECLARE_RCP(Add)
	DECLARE_RCP(Remove)
	DECLARE_RCP(Reset)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_CON(Dictionary)
		DEFINE_CON(Open)
		DEFINE_CON(Select)

		// General widget connectors
		DEFINE_RCP(Enable)
		DEFINE_RCP(Owner)

		// Listbox specific
		DEFINE_RCP(Add)
		DEFINE_RCP(List)
		DEFINE_RCP(Remove)
		DEFINE_RCP(Reset)
	END_BEHAVIOUR_NOTIFY()

	};

//
// Class - Table.  Node for a table view.
//

class Table :
	public CCLObject,										// Base class
	public gdiControl,									// Base class
	public IBehaviour										// Interface
	{
	public :
	Table ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct;									// Run-time data

	// CCL
	CCL_OBJECT_BEGIN(Table)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_RCP(Dictionary)
	DECLARE_CON(Open)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Open)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Tree.  Node for a tree view.
//

class Tree :
	public CCLObject,										// Base class
	public gdiControl,									// Base class
	public IBehaviour										// Interface
	{
	public :
	Tree ( void );											// Constructor

	// Run-time data
	IDictionary	*pDct,*pOwn;							// Run-time data
	HWND			hWnd;										// Window handle
	IList			*pLstPath;								// Path list
//	IList			*pLstSel,*pLstSelR;					// List selection
	WCHAR			wBfr[1024];								// General buffer

	// 'gdiControl' members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );

	// CCL
	CCL_OBJECT_BEGIN(Tree)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_CON(Dictionary)
	DECLARE_CON(Open)
	DECLARE_CON(Select)
	DECLARE_RCP(Matrix)
	DECLARE_RCP(Owner)
	DECLARE_EMT(Activate)
	DECLARE_RCP(Path)
	DECLARE_RCP(Store)
	DECLARE_RCP(Remove)
	DECLARE_RCP(Reset)
//	DECLARE_RCP(Tree)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_CON(Dictionary)
		DEFINE_CON(Open)
		DEFINE_CON(Select)

		// General widget connectors
		DEFINE_RCP(Matrix)
		DEFINE_RCP(Owner)

		// Tree specific
//		DEFINE_RCP(Tree)
		DEFINE_EMT(Activate)
		DEFINE_RCP(Path)
		DEFINE_RCP(Store)
		DEFINE_RCP(Remove)
		DEFINE_RCP(Reset)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT find	( HTREEITEM, const WCHAR *, HTREEITEM * );
	HRESULT sync	( HTREEITEM, IDictionary * );
	};

//
// Class - View.  View container for controls.
//

class View :
	public CCLObject,										// Base class
	public gdiControl,									// Base class
	public IBehaviour										// Interface
	{
	public :
	View ( void );											// Constructor

	// Run-time data
	IDictionary	*pDct,*pCtl;							// Run-time data

	// 'gdiControl' members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );

	// Utilities
	static HWND GetProcessWnd();						// Get HWND for process

	// 'uiSubWindow' class members
//	virtual HWND		createWindow( HWND );		// Create window

	// CCL
	CCL_OBJECT_BEGIN(View)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_CON(Open)
	DECLARE_RCP(Add)
	DECLARE_RCP(Control)
	DECLARE_RCP(Dictionary)
	DECLARE_RCP(Remove)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_CON(Open)

		DEFINE_RCP(Add)
		DEFINE_RCP(Control)
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Remove)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	static BOOL CALLBACK EnumWindowsProc ( HWND, LPARAM );
	};

//
// Class - Window.  Window node.
//

class Window :
	public CCLObject,										// Base class
	public gdiControl,									// Base class
	public ITickable,										// Interface
	public IBehaviour										// Interface
	{
	public :
	Window ( void );										// Constructor

	// Run-time data
	IDictionary		*pDct,*pOwn;						// Run-time data
	IDictionary		*pDctInM;							// Mouse input dictionary
	IDictionary		*pDctInK;							// Keyboard input dictionary
	IReceptor		*prOut;								// Output receptor
	adtValue			vOut;									// Value to output
	HWND				hWndModal;							// Modal state
	IThread			*pThrd;								// Window thread
	U32				dwThrdId;							// Thread ID
	IDictionary		*pIn,*pOut;							// Connectors
	adtStringSt		strX,strY,strUp,strDn;			// String references
	adtStringSt		strBt,strBt1,strBt2,strBt3;	// String references
	adtStringSt		strEv,strMv;						// String references

	// Utilities
	static U32 stylesToInt ( IUnknown *, U32 * );

	// 'gdiControl' members
	virtual LRESULT onMessage	( IDictionary *, UINT, WPARAM, LPARAM );

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN(Window)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Close)
	DECLARE_CON(Open)
	DECLARE_CON(Dictionary)
	DECLARE_RCP(Enable)
	DECLARE_EMT(Key)
	DECLARE_RCP(Matrix)
	DECLARE_RCP(Modal)
	DECLARE_RCP(Owner)
	DECLARE_EMT(Mouse)
	DECLARE_RCP(Text)
	DECLARE_RCP(Type)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_CON(Open)

		DEFINE_CON(Dictionary)
		DEFINE_RCP(Enable)
		DEFINE_RCP(Matrix)
		DEFINE_RCP(Modal)
		DEFINE_RCP(Owner)

		DEFINE_EMT(Key)
		DEFINE_EMT(Mouse)
		DEFINE_RCP(Text)
		DEFINE_RCP(Type)
	END_BEHAVIOUR_NOTIFY()

	private :
	};
*/

#endif
