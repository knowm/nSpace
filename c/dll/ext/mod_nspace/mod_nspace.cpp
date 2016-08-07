////////////////////////////////////////////////////////////////////////
//
//								MOD_NSPACE.CPP
//
//				Main file for the nSpace Apache module
//
////////////////////////////////////////////////////////////////////////

// nSpace
#include "nspace.h"

// Globals
extern nSpaceLink	*pLink;

// Prototypes
void	nspace_child_init			( apr_pool_t *p, server_rec *s );
int	nspace_handler				( request_rec *r );
bool	nspace_init					( bool );
void	nspace_write_req	( apr_pool_t *p );

//
// Dispatch list
//
extern "C"
module AP_MODULE_DECLARE_DATA nspace_module =
	{
	STANDARD20_MODULE_STUFF,
	NULL,														// create per-dir
	NULL,														// merge per-dir
	NULL,														// create per-server
	NULL,														// merge per-server
	NULL,														// table of config file commands
	nspace_write_req								// Register hooks
	};

apr_status_t nspace_child_cleanup ( void *pv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when when child process allocation pool is shutdown.
	//
	//	PARAMETERS
	//		-	pv is the pool data
	//
	////////////////////////////////////////////////////////////////////////
	OutputDebugStringA("nspace_child_cleanup");

	// Shutdown nSpace link
	nspace_init(false);

	return APR_SUCCESS;
	}	// nspace_child_cleanup

void nspace_child_init ( apr_pool_t *p, server_rec *s )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a child process is spawned.
	//
	//	PARAMETERS
	//		-	p is the allocation pool
	//		-	s contains information about the virtual server
	//
	////////////////////////////////////////////////////////////////////////
	OutputDebugStringA("nspace_child_init");

	// Register a clean-up routine
	apr_pool_cleanup_register ( p, s, nspace_child_cleanup, 
											apr_pool_cleanup_null );

	// Initialize nSpace link
	nspace_init(true);
	}	// nspace_child_init

int nspace_handler ( request_rec *r )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Handle incoming request.
	//
	//	PARAMETERS
	//		-	r is the request
	//
	//	RETURN VALUE
	//		Request result
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	strUri;

	// Request for this handler ?
	if (	r->handler == NULL || strcmp ( r->handler, "nspace" ) )
		return DECLINED;

	// Debug
	OutputDebugStringA(r->method);
	OutputDebugStringA(r->filename);
	OutputDebugStringA(r->uri);
	OutputDebugStringA(r->args);
	OutputDebugStringA(r->useragent_ip);
//	lprintf ( LOG_INFO, "", r->co

	// State check
	CCLTRYE ( strncasecmp(r->uri,"/nspace/",8) == 0, E_UNEXPECTED );

	// Translate URI into a namespace path
	CCLOK ( strUri = &(r->uri[7]); )

	//
	// Method
	//

	// Get = Load
	if (hr == S_OK && !strcasecmp(r->method,"GET"))
		{
		adtValue		vL;

		// Responses are XML nSpace values
		ap_set_content_type ( r, "text/xml" );

		// Load value from namespace into the response stream
		CCLTRY ( pLink->load ( strUri, r ) );
		}	// if

	// Post = Store
	else if (hr == S_OK && !strcasecmp(r->method,"POST"))
		{
		IByteStream	*pStm	= NULL;
		int			rc;

		// Create memory byte stream
		CCLTRY ( COCREATE ( L"Io.StmMemory", IID_IByteStream, &pStm ) );

		// Prepare to read content
		CCLTRYE ( (rc = ap_setup_client_block ( r, 
						REQUEST_CHUNKED_ERROR )) == OK, S_FALSE );

		// Ready to receive data
		if (hr == S_OK && ap_should_client_block ( r ))
			{
			char	argsbuffer[HUGE_STRING_LEN];
			int	len_read;

			// Read data
			while ( hr == S_OK &&
						(len_read = ap_get_client_block ( r, argsbuffer,
						sizeof(argsbuffer) )) > 0 )
				{
				// Debug
				argsbuffer[len_read] = '\0';
				lprintf(LOG_INFO, L"len_read %d:%S", len_read, argsbuffer);

				// Write bufers to stream
				// TODO: Create byte stream interface for 'ap_get_client_block' 
				// usage to avoid copy/allocation
				CCLTRY ( pStm->write ( argsbuffer, len_read, NULL ) );
				}	// while

//			lprintf ( LOG_INFO, L"Content read hr 0x%x", hr );
			}	// if

		// Parse and store value into namespace
		CCLTRY ( pStm->seek ( STREAM_SEEK_SET, 0, NULL ) );
		CCLTRY ( pLink->store ( strUri, pStm ) );

		// Clean up
		_RELEASE(pStm);
		}	// else if

	return (hr == S_OK) ? OK : HTTP_NOT_FOUND;
	}	// nspace_handler

void nspace_write_req ( apr_pool_t *p )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Register the calbacks with the server.
	//
	//	PARAMETERS
	//		-	p is the allocation pool
	////////////////////////////////////////////////////////////////////////
	OutputDebugStringA("nspace_write_req");

	// Request handler
	ap_hook_handler ( nspace_handler, NULL, NULL, APR_HOOK_LAST );

	// Initialization
	ap_hook_child_init ( nspace_child_init, NULL, NULL, APR_HOOK_LAST );

	}	// nspace_write_req

