

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Tue Jan 12 13:28:38 2016
 */
/* Compiler settings for nshle.idl:
    Oicf, W1, Zp8, env=Win32 (32b run), target_arch=X86 8.00.0595 
    protocol : dce , ms_ext, c_ext, robust
    error checks: allocation ref bounds_check enum stub_data 
    VC __declspec() decoration level: 
         __declspec(uuid()), __declspec(selectany), __declspec(novtable)
         DECLSPEC_UUID(), MIDL_INTERFACE()
*/
/* @@MIDL_FILE_HEADING(  ) */

#pragma warning( disable: 4049 )  /* more than 64k source lines */


/* verify that the <rpcndr.h> version is high enough to compile this file*/
#ifndef __REQUIRED_RPCNDR_H_VERSION__
#define __REQUIRED_RPCNDR_H_VERSION__ 475
#endif

#include "rpc.h"
#include "rpcndr.h"

#ifndef __RPCNDR_H_VERSION__
#error this stub requires an updated version of <rpcndr.h>
#endif // __RPCNDR_H_VERSION__


#ifndef __nshle_h_h__
#define __nshle_h_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __IListenX_FWD_DEFINED__
#define __IListenX_FWD_DEFINED__
typedef interface IListenX IListenX;

#endif 	/* __IListenX_FWD_DEFINED__ */


#ifndef __INamespaceX_FWD_DEFINED__
#define __INamespaceX_FWD_DEFINED__
typedef interface INamespaceX INamespaceX;

#endif 	/* __INamespaceX_FWD_DEFINED__ */


#ifndef __IShellX_FWD_DEFINED__
#define __IShellX_FWD_DEFINED__
typedef interface IShellX IShellX;

#endif 	/* __IShellX_FWD_DEFINED__ */


#ifndef __ShellX_FWD_DEFINED__
#define __ShellX_FWD_DEFINED__

#ifdef __cplusplus
typedef class ShellX ShellX;
#else
typedef struct ShellX ShellX;
#endif /* __cplusplus */

#endif 	/* __ShellX_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"

#ifdef __cplusplus
extern "C"{
#endif 



#ifndef __nSpace_LIBRARY_DEFINED__
#define __nSpace_LIBRARY_DEFINED__

/* library nSpace */
/* [version][lcid][uuid] */ 


EXTERN_C const IID LIBID_nSpace;

#ifndef __IListenX_INTERFACE_DEFINED__
#define __IListenX_INTERFACE_DEFINED__

/* interface IListenX */
/* [oleautomation][dual][uuid][object] */ 


EXTERN_C const IID IID_IListenX;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("2534d084-8628-11d2-868c-006008addfed")
    IListenX : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE receive( 
            /* [in] */ BSTR szPath,
            /* [in] */ BSTR szLoc,
            /* [in] */ VARIANT *__MIDL__IListenX0000) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IListenXVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IListenX * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IListenX * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IListenX * This);
        
        HRESULT ( STDMETHODCALLTYPE *receive )( 
            IListenX * This,
            /* [in] */ BSTR szPath,
            /* [in] */ BSTR szLoc,
            /* [in] */ VARIANT *__MIDL__IListenX0000);
        
        END_INTERFACE
    } IListenXVtbl;

    interface IListenX
    {
        CONST_VTBL struct IListenXVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IListenX_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IListenX_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IListenX_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IListenX_receive(This,szPath,szLoc,__MIDL__IListenX0000)	\
    ( (This)->lpVtbl -> receive(This,szPath,szLoc,__MIDL__IListenX0000) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IListenX_INTERFACE_DEFINED__ */


#ifndef __INamespaceX_INTERFACE_DEFINED__
#define __INamespaceX_INTERFACE_DEFINED__

/* interface INamespaceX */
/* [oleautomation][dual][uuid][object] */ 


EXTERN_C const IID IID_INamespaceX;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("2534d085-8628-11d2-868c-006008addfed")
    INamespaceX : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE load( 
            /* [in] */ BSTR szPath,
            /* [out] */ VARIANT *__MIDL__INamespaceX0000) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE listen( 
            /* [in] */ BSTR szPath,
            /* [in] */ IListenX *ppL) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE store( 
            /* [in] */ BSTR szPath,
            /* [in] */ VARIANT *__MIDL__INamespaceX0001) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE unlisten( 
            /* [in] */ BSTR szPath,
            /* [in] */ IListenX *pL) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct INamespaceXVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            INamespaceX * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            INamespaceX * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            INamespaceX * This);
        
        HRESULT ( STDMETHODCALLTYPE *load )( 
            INamespaceX * This,
            /* [in] */ BSTR szPath,
            /* [out] */ VARIANT *__MIDL__INamespaceX0000);
        
        HRESULT ( STDMETHODCALLTYPE *listen )( 
            INamespaceX * This,
            /* [in] */ BSTR szPath,
            /* [in] */ IListenX *ppL);
        
        HRESULT ( STDMETHODCALLTYPE *store )( 
            INamespaceX * This,
            /* [in] */ BSTR szPath,
            /* [in] */ VARIANT *__MIDL__INamespaceX0001);
        
        HRESULT ( STDMETHODCALLTYPE *unlisten )( 
            INamespaceX * This,
            /* [in] */ BSTR szPath,
            /* [in] */ IListenX *pL);
        
        END_INTERFACE
    } INamespaceXVtbl;

    interface INamespaceX
    {
        CONST_VTBL struct INamespaceXVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define INamespaceX_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define INamespaceX_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define INamespaceX_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define INamespaceX_load(This,szPath,__MIDL__INamespaceX0000)	\
    ( (This)->lpVtbl -> load(This,szPath,__MIDL__INamespaceX0000) ) 

#define INamespaceX_listen(This,szPath,ppL)	\
    ( (This)->lpVtbl -> listen(This,szPath,ppL) ) 

#define INamespaceX_store(This,szPath,__MIDL__INamespaceX0001)	\
    ( (This)->lpVtbl -> store(This,szPath,__MIDL__INamespaceX0001) ) 

#define INamespaceX_unlisten(This,szPath,pL)	\
    ( (This)->lpVtbl -> unlisten(This,szPath,pL) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __INamespaceX_INTERFACE_DEFINED__ */


#ifndef __IShellX_INTERFACE_DEFINED__
#define __IShellX_INTERFACE_DEFINED__

/* interface IShellX */
/* [oleautomation][dual][uuid][object] */ 


EXTERN_C const IID IID_IShellX;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("2534d086-8628-11d2-868c-006008addfed")
    IShellX : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE open( 
            /* [in] */ BSTR szCmdLine,
            /* [in] */ BOOL bShare,
            /* [out] */ INamespaceX **ppSpc) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IShellXVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IShellX * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IShellX * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IShellX * This);
        
        HRESULT ( STDMETHODCALLTYPE *open )( 
            IShellX * This,
            /* [in] */ BSTR szCmdLine,
            /* [in] */ BOOL bShare,
            /* [out] */ INamespaceX **ppSpc);
        
        END_INTERFACE
    } IShellXVtbl;

    interface IShellX
    {
        CONST_VTBL struct IShellXVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IShellX_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IShellX_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IShellX_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IShellX_open(This,szCmdLine,bShare,ppSpc)	\
    ( (This)->lpVtbl -> open(This,szCmdLine,bShare,ppSpc) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IShellX_INTERFACE_DEFINED__ */


EXTERN_C const CLSID CLSID_ShellX;

#ifdef __cplusplus

class DECLSPEC_UUID("2534d087-8628-11d2-868c-006008addfed")
ShellX;
#endif
#endif /* __nSpace_LIBRARY_DEFINED__ */

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


