////////////////////////////////////////////////////////////////////////
//
//								MOD_NSPACE.CPP
//
//				Main file for the nSpace Apache module
//
////////////////////////////////////////////////////////////////////////

// Apache
#include <httpd.h>
#include <http_config.h>
#include <http_protocol.h>
#include <ap_config.h>

// Prototypes
void	nspace_child_init			( apr_pool_t *p, server_rec *s );
int	nspace_handler				( request_rec *r );
bool	nspace_init					( bool );
void	nspace_register_hooks	( apr_pool_t *p );

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
	nspace_register_hooks								// Register hooks
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

	// Request for this handler ?
	if (	r->handler == NULL || strcmp ( r->handler, "nspace" ) )
		return DECLINED;

	// Responses are XML nSpace values
	r->content_type = "text/xml";

	// Response
	ap_rputs ( "<Value  Type=\"Double\">3.141592653</Value>", r );

	// Debug
	OutputDebugStringA ( r->method );
	OutputDebugStringA ( r->filename  );
	OutputDebugStringA ( r->uri );
	OutputDebugStringA ( r->args );
	OutputDebugStringA ( r->useragent_ip );

	return OK;
	}	// nspace_handler

void nspace_register_hooks ( apr_pool_t *p )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Register the calbacks with the server.
	//
	//	PARAMETERS
	//		-	p is the allocation pool
	////////////////////////////////////////////////////////////////////////
	OutputDebugStringA("nspace_register_hooks");

	// Request handler
	ap_hook_handler ( nspace_handler, NULL, NULL, APR_HOOK_LAST );

	// Initialization
	ap_hook_child_init ( nspace_child_init, NULL, NULL, APR_HOOK_LAST );

	}	// nspace_register_hooks
