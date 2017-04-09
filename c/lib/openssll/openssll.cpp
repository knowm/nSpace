////////////////////////////////////////////////////////////////////////
//
//									OPENSSLL.CPP
//
//								General utilities
//
////////////////////////////////////////////////////////////////////////

#include "openssll_.h"

// Globals
libSSL	libS;

//
// libSSL
//

libSSL :: libSSL ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	bValid			= false;
	lsslcnt			= 0;
	lcnt				= 0;
	psslctx			= NULL;
	}	// libSSL

LONG libSSL :: AddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a reference count to the library.  Attempts load
	//			on first count.
	//
	//	RETURN VALUE
	//		Current reference count
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Reference count
	if (lcnt > 0)
		return ++lcnt;
/*
	// Need function ptrs ?
	if ((HMODULE)NULL == dlleay || (HMODULE)NULL == dllssl)
		return 0;

	// Access function ptrs

	// BIO_XXX
	CCLTRYE ( ( bio_ctrl = 
						(long (*) ( BIO *, int, long, void * ))
						GetProcAddress ( dlleay, "BIO_ctrl" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( bio_free = 
						(int (*) ( BIO * ))
						GetProcAddress ( dlleay, "BIO_free" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( bio_new = 
						(BIO * (*) ( BIO_METHOD * ))
						GetProcAddress ( dlleay, "BIO_new" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( bio_new_mem_buf = 
						(BIO * (*) ( void *, int))
						GetProcAddress ( dlleay, "BIO_new_mem_buf" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( bio_s_mem = 
						(BIO_METHOD * (*) ( void ))
						GetProcAddress ( dlleay, "BIO_s_mem" ) ) != NULL,
						GetLastError() );

	// ERR_XXX
	CCLTRYE ( ( err_get_error = 
						(unsigned long (*) ( void ))
						GetProcAddress ( dlleay, "ERR_get_error" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( err_err_str = 
						(char * (*) ( unsigned long, char *, size_t ))
						GetProcAddress ( dlleay, "ERR_error_string_n" ) ) != NULL,
						GetLastError() );

	// EVP_XXX
 	CCLTRYE ( ( evp_ctx_new =
						(EVP_MD_CTX * (*) ( void ))
						GetProcAddress ( dlleay, "EVP_MD_CTX_create" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_ctx_free = 
						(void (*) ( EVP_MD_CTX *))
						GetProcAddress ( dlleay, "EVP_MD_CTX_destroy" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_digestfinal =
						(int (*) ( EVP_MD_CTX *, unsigned char *, unsigned int * ))
						GetProcAddress ( dlleay, "EVP_DigestFinal" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_digestinit =
						(int (*) ( EVP_MD_CTX *, const EVP_MD * ))
						GetProcAddress ( dlleay, "EVP_DigestInit" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_digestup =
						(int (*) ( EVP_MD_CTX *, const void *, unsigned int ))
						GetProcAddress ( dlleay, "EVP_DigestUpdate" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_signfinal =
						(int (*) ( EVP_MD_CTX *, unsigned char *, unsigned int *, EVP_PKEY * ))
						GetProcAddress ( dlleay, "EVP_SignFinal" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_verifyfinal =
						(int (*) ( EVP_MD_CTX *, const unsigned char *, unsigned int, EVP_PKEY * ))
						GetProcAddress ( dlleay, "EVP_VerifyFinal" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_md5 =
						(const EVP_MD * (*) ( void ))
						GetProcAddress ( dlleay, "EVP_md5" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_pkey_free =
						(void (*) ( EVP_PKEY * ))
						GetProcAddress ( dlleay, "EVP_PKEY_free" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_pkey_new =
						(EVP_PKEY * (*) ( void ))
						GetProcAddress ( dlleay, "EVP_PKEY_new" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_pkey_set_rsa =
						(int (*) ( EVP_PKEY *, struct rsa_st * ))
						GetProcAddress ( dlleay, "EVP_PKEY_set1_RSA" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_pkey_size =
						(int (*) ( EVP_PKEY * ))
						GetProcAddress ( dlleay, "EVP_PKEY_size" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( evp_sha1 =
						(const EVP_MD * (*) ( void ))
						GetProcAddress ( dlleay, "EVP_sha1" ) ) != NULL,
						GetLastError() );

	// PEM_XXX
	CCLTRYE ( ( pem_rd_rsa_priv = 
					(RSA * (*) ( BIO *, RSA **, pem_password_cb *, void * ))
						GetProcAddress ( dlleay, "PEM_read_bio_RSAPrivateKey" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( pem_rd_rsa_pub = 
					(RSA * (*) ( BIO *, RSA **, pem_password_cb *, void * ))
						GetProcAddress ( dlleay, "PEM_read_bio_RSAPublicKey" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( pem_wr_rsa_priv = 
					(int (*) (	BIO *, RSA *, const EVP_CIPHER *,
									unsigned char *, int, pem_password_cb *, void * ))
						GetProcAddress ( dlleay, "PEM_write_bio_RSAPrivateKey" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( pem_wr_rsa_pub = 
					(int (*) (	BIO *, RSA *, const EVP_CIPHER *,
									unsigned char *, int, pem_password_cb *, void * ))
						GetProcAddress ( dlleay, "PEM_write_bio_RSAPublicKey" ) ) != NULL,
						GetLastError() );

	// RSA_XXX
	CCLTRYE ( ( rsa_free = 
						(void (*)	 ( RSA * ))
						GetProcAddress ( dlleay, "RSA_free" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( rsa_gen_key = 
		(RSA * (*) ( int, unsigned long, void (*) ( int, int, void * ), void * ))
						GetProcAddress ( dlleay, "RSA_generate_key" ) ) != NULL,
						GetLastError() );
//	CCLTRYE ( ( rsa_gen_key = 
//						(int (*) ( RSA *, int, BIGNUM *, BN_GENCB * ))
//						GetProcAddress ( dlleay, "RSA_generate_key_ex" ) ) != NULL,
//						GetLastError() );
	CCLTRYE ( ( rsa_new = 
						(RSA * (*)	 ( void ))
						GetProcAddress ( dlleay, "RSA_new" ) ) != NULL,
						GetLastError() );

	// SSL_XXX
	CCLTRYE ( ( ssl_connect = 
						(int (*) ( SSL * ))
						GetProcAddress ( dllssl, "SSL_connect" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_ctx_free = 
						(void (*)	 ( SSL_CTX * ))
						GetProcAddress ( dllssl, "SSL_CTX_free" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_ctx_new = 
						(SSL_CTX * (*)	 ( const SSL_METHOD * ))
						GetProcAddress ( dllssl, "SSL_CTX_new" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_lib_init = 
						(int (*) ( void ))
//						GetProcAddress ( dllssl, "OPENSSL_init_ssl" ) ) != NULL,
						GetProcAddress ( dllssl, "SSL_library_init" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_load_err = 
						(void (*) ( void ))
//						GetProcAddress ( dllssl, "ERR_load_SSL_strings" ) ) != NULL,
						GetProcAddress ( dllssl, "SSL_load_error_strings" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_free = 
						(void (*)	 ( SSL * ))
						GetProcAddress ( dllssl, "SSL_free" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_new = 
						(SSL * (*)	 ( SSL_CTX * ))
						GetProcAddress ( dllssl, "SSL_new" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_set_fd = 
						(int (*) ( SSL *, int ))
						GetProcAddress ( dllssl, "SSL_set_fd" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_v3_client = 
						(SSL_METHOD * (*)	 ( void ))
						GetProcAddress ( dllssl, "SSLv3_client_method" ) ) != NULL,
//						GetProcAddress ( dllssl, "SSLv23_client_method" ) ) != NULL,
//						GetProcAddress ( dllssl, "TLS_client_method" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_write = 
						(int (*) ( SSL *, const void *, int ))
						GetProcAddress ( dllssl, "SSL_write" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_get_error = 
						(int (*) ( const SSL *, int ))
						GetProcAddress ( dllssl, "SSL_get_error" ) ) != NULL,
						GetLastError() );
	CCLTRYE ( ( ssl_read = 
						(int (*) ( SSL *, void *, int ))
						GetProcAddress ( dllssl, "SSL_read" ) ) != NULL,
						GetLastError() );
*/
	// Valid ?
	CCLOK ( bValid = TRUE; )
	if (!bValid)
		lprintf ( LOG_WARN, L"libSSL not in a valid state, 0x%x\r\n", hr );

	return (bValid) ? ++lcnt : 0;
	}	// AddRef

HRESULT libSSL :: errors ( const wchar_t *strCtx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Internal utility to check for error conditions.
	//
	//	PARAMETERS
	//		-	strCtx is the context of the call
	//
	//	RETURN VALUE
	//		S_OK if no error
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr = S_OK;
	unsigned long	e;
	char				bfr[1024];

	// Continue until no more errors
	while ((e = ERR_get_error()) != 0)
		{
		ERR_error_string_n ( e, bfr, sizeof(bfr) );
		dbgprintf ( L"%s:Error:%S:%d(0x%x)\r\n", strCtx, bfr, e, e );
		if (hr == S_OK)
			hr = e;
		}	// while

	return hr;
	}	// errors

LONG libSSL :: Release ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a reference count to the library.  Attempts load
	//			on first count.
	//
	//	RETURN VALUE
	//		Current reference count
	//
	////////////////////////////////////////////////////////////////////////

	// Closure
 	if (lcnt > 1)
		return --lcnt;

	// Clean up
	bValid	= FALSE;

	return (lcnt = 0);
	}	// Release

LONG libSSL :: sslAddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a reference count to the SSL library.
	//
	//	RETURN VALUE
	//		Current reference count
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT				hr				= S_OK;
	const SSL_METHOD	*psslmthd	= NULL;

	// State check
	if (!bValid)
		return 0;

	// Reference count
	if (lsslcnt++ > 0)
		return lsslcnt;

	// Initialize library
	CCLTRYE ( SSL_library_init() == 1, E_UNEXPECTED );
	CCLOK   ( SSL_load_error_strings(); )

	// Application state
	CCLTRYE ( (psslmthd = SSLv3_client_method()) != NULL, E_UNEXPECTED );

	// Context
	CCLTRYE ( (psslctx = SSL_CTX_new(psslmthd)) != NULL, E_UNEXPECTED );

	// Result
	if (hr != S_OK)
		lsslcnt = 0;

	return lsslcnt;
	}	// sslAddRef

LONG libSSL :: sslRelease ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a reference count to the library.  Attempts load
	//			on first count.
	//
	//	RETURN VALUE
	//		Current reference count
	//
	////////////////////////////////////////////////////////////////////////

	// Closure
 	if (lsslcnt == 1)
		{
		// Clean up
		if (psslctx != NULL)
			{
			SSL_CTX_free ( psslctx );
			psslctx = NULL;
			}	// if
		}	// if

	return --lsslcnt;
	}	// sslRelease

//
// objSSL
//

objSSL :: objSSL ( RSA *_rsa )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_rsa is the object to wrap
	//
	////////////////////////////////////////////////////////////////////////
	rsa		= _rsa;
	AddRef();
	}	// objSSL

void objSSL :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	if (rsa != NULL)
		RSA_free(rsa);
	}	// destruct
