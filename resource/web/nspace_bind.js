////////////////////////////////////////////////////////////////////////
//
//									NSPACE_BIND.JS
//
//		nSpace script to bind existing HTML elements to nSpace web 
//		sockets server.
//
////////////////////////////////////////////////////////////////////////

// Create new instance of this object immediately
new function ()
	{
	var	ws			= null;								// WebSocket object
	var	nElems	= {};									// Bounded element states

	nSpace_bind =
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Object to bind HTML elements on page to nSpace server.
		//
		////////////////////////////////////////////////////////////////////////

		init : function() 
			{ 
			////////////////////////////////////////////////////////////////////////
			//
			//	PURPOSE
			//		-	Initialize state of the binding object
			//
			////////////////////////////////////////////////////////////////////////

			// Parser
			nSpaceXML.init();

			// Establish connection to webserver
			ws					= new WebSocket("ws://localhost:8080/nspace");
			ws.onopen		= this.onOpen;
			ws.onmessage	= this.onMessage;		
			},

		onMessage : function(event)
			{
			////////////////////////////////////////////////////////////////////////
			//
			//	PURPOSE
			//		-	Callback when a WebSocket message is received.
			//
			//	PARAMETERS
			//		-	event contains the message
			//
			////////////////////////////////////////////////////////////////////////
			var msg		= null;
			var elem		= null;
			var loc		= null;
			var type		= null;
			var value	= null;

			// Extract nSpace value from incoming XML
			parser	= new DOMParser();
			xmlDoc	= parser.parseFromString(event.data,"text/xml");
			msg		= nSpaceXML.load ( xmlDoc.documentElement );

			// Debug
			console.log(msg);

			// Root path of value mapped to element
			elem = nElems[msg["Root"]];
			if (elem == null)
				return;

			// To make it easier to process string
			loc = msg["Location"].toLowerCase();
			type = elem.nodeName.toLowerCase();

			//
			// Process value based on element type
			//
			console.log("Location:"+loc+":Type:"+type);

			// Button
			if (type == "button")
				{
				// Enable
				if (loc == "element/enable/onfire/value")
					elem.disabled = !msg["Value"];
				}	// if

			// Checkbox
			else if (type == "checkbox")
				{
				// Enable
				if (loc == "element/enable/onfire/value")
					elem.disabled = !msg["Value"];

				// Activate
				else if (loc == "activate/onfire/value")
					elem.checked = msg["Value"];
				}	// else if

			},

		onOpen : function ()
			{
			////////////////////////////////////////////////////////////////////////
			//
			//	PURPOSE
			//		-	Callback when a WebSocket connection is made.
			//
			////////////////////////////////////////////////////////////////////////
			var root		= "";
			var elems	= null;
			var path		= null;

			// Enumerate nSpace elements and bind them to their paths.
 
			// Enumerate all of the elements on the page
			var
			elems = document.getElementsByTagName("*");

			// Find every element that specifies and nSpace path attribute.
			for (var i = 0, n = elems.length;i < n;++i)
				{
				// A 'root' path can be specified so that subsequent elements
				// can specify relative paths
				if (elems[i].getAttribute("data-nroot") != null)
					root = elems[i].getAttribute("data-nroot");

				// nSpace path specified ?
				if (elems[i].getAttribute("data-npath") != null)
					{
					// Generate full bind path
					path = root + elems[i].getAttribute("data-npath");

					// Assign full path for quick reference
					elems[i].attributes["data-nabs"] = path;

					// Associate the path with the HTML element
					nElems[path] = elems[i];

					// Set-up events
					elems[i].onclick = onClickn;

					// Send listen request for path
					listen(path);
					}	// if

				}	// for

			// Debug message
//			ws.send("<Dictionary><Value>You</Value><Value Type=\"Double\">3.14159265358979323</Value></Dictionary>");
//			alert ( "Open!!" );
			},

		}	// nSpace_bind

	var listen = function(srcPath)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Issue a 'listen' request for the specified path
		//
		//	PARAMETERS
		//		-	srcPath is the path to listen to
		//
		////////////////////////////////////////////////////////////////////////
		var req = 
			{
			verb: "Listen",
			path: ""
			};

		// Path for listening
		req["path"] = srcPath;

		// Send request
		if (ws != null)
			ws.send(nSpaceXML.save(req));
		}	// listen

	//
	// Events
	//

	var onClickn = function(event)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Called when an element is 'clicked'.
		//
		//	PARAMETERS
		//		-	event contains the event information
		//
		////////////////////////////////////////////////////////////////////////
		var type = event.srcElement.nodeName.toLowerCase(); 
		var dct  = {};
		var xml	= null;

		// Button
		alert ( type );
		if (type == "button")
			{
			// Send activate value
			dct["Verb"]			= "Store";
			dct["Location"]	= event.srcElement.attributes["data-nabs"] + "Activate/OnFire/Value";
			dct["Value"]		= 0;
			xml					= nSpaceXML.save(dct);
			ws.send ( xml );
			}	// if

		}	// onClickn

	}	// function

// Execute main on document loaded
document.onreadystatechange = function ()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the documents ready state has changed.
	//
	////////////////////////////////////////////////////////////////////////

	// Document ready ?
	if (document.readyState == "complete")
		nSpace_bind.init();		
	}	// onreadystatechange