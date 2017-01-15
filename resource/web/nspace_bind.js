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
			ws.binaryType	= "arraybuffer";
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

			// Type of received message
			if (typeof event.data == "string")
				{
				// Extract nSpace value from incoming XML
				parser	= new DOMParser();
				xmlDoc	= parser.parseFromString(event.data,"text/xml");
				msg		= nSpaceXML.load ( xmlDoc.documentElement );

				// Debug
				console.log(msg);
				}	// if

			// Binary
			else if (event.data instanceof ArrayBuffer)
				{
				var bfr	= new Uint8Array(event.data);
				var idx	= new Uint32Array(1);

				// Extract nSpace value from  ArrayBuffer
				console.log("ArrayBuffer:"+event.data);
				msg		= nSpaceBin.load ( bfr, idx );
				}	// else if

			// Sanity check
			if (msg == null || !("Root" in msg) || !("Location" in msg))
				{
				console.log("onMessage:Missing fields:"+msg);
				return;
				}	// if

			// Root path of value mapped to element
			elem = nElems[msg["Root"]];
			if (elem == null)
				return;

			// To make it easier to process string
			loc	= msg["Location"].toLowerCase();
//			type	= elem.nodeName.toLowerCase();
			type	= elem["type"];
			value	= msg["Value"];

			//
			// Process value based on element type
			//
			console.log("Location:"+loc+":Type:"+type);

			// Remove common '/onfire/value' postfix
			loc = loc.substring(0,loc.length-13);

			// Non-type specific

			// Enable
			if (loc == "element/enable")
				elem.disabled = !value;

			// Button
			else if (type == "button")
				{
				}	// if

			// Checkbox
			else if (type == "checkbox")
				{
				// Activate
				if (loc == "activate")
					elem.checked = value;
				}	// else if

			// List box
			else if (type == "select-one")
				{
				// Current list
				if (loc == "list")
					{
					var str;

					// Remove existing elements
					while (elem.length > 0)
						elem.remove(0);

					// Add strings to list
					for (idx in value)
						{
						var option = document.createElement("option");
						option.text = value[idx];
						elem.add(option);
						}	// for

					}	// if

				}	// else if

			// Text/edit box
			else if (type == "text")
				{
				// Value
				if (loc == "element/default")
					elem.text = value;
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

			var req = 
				{
				verb: "Store",
				path: "Just/Testing/The/Message",
				count: 0
				};

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
				if (elems[i].getAttribute("data-nloc") != null)
					{
					// Generate full bind path
					path = root + elems[i].getAttribute("data-nloc");

					// Assign full path for quick reference
					elems[i].attributes["data-nabs"] = path;

					// Associate the path with the HTML element
					nElems[path] = elems[i];

					// Set-up events
					elems[i].onclick	= onClickn;
					elems[i].onchange	= onChangen;

					// Send listen request for path
					listen(path);
					}	// if

				}	// for

			// Debug
//			req["count"] = "O";
//			ws.send(nSpaceXML.save(req));
//			req["count"] = "Tw";
//			ws.send(nSpaceXML.save(req));

			// Debug message
//			ws.send("<Dictionary><Value>You</Value><Value Type=\"Double\">3.14159265358979323</Value></Dictionary>");
//			alert ( "Open!!" );
			},

		}	// nSpace_bind

	var listen = function(srcLoc)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Issue a 'listen' request for the specified path
		//
		//	PARAMETERS
		//		-	srcLoc is the location to listen to
		//
		////////////////////////////////////////////////////////////////////////
		var req = 
			{
			verb: "Listen",
			path: ""
			};

		// Location for listening
		req["location"] = srcLoc;

		// Send request
		if (ws != null)
			ws.send(nSpaceXML.save(req));
		}	// listen

	//
	// Events
	//

	var onChangen = function(event)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Called when the an option changes.
		//
		//	PARAMETERS
		//		-	event contains the event information
		//
		////////////////////////////////////////////////////////////////////////
		var type = event.srcElement["type"];
		var dct  = {};
		var xml	= null;
		var send = true;

		console.log("onChangem:"+event);

		// Store template
		dct["Verb"]			= "Store";
		dct["Location"]	= event.srcElement.attributes["data-nabs"] + 
									"Element/Default/Fire";

		// Select
		if (type == "select-one")
			{
			// Selected index
			dct["Value"]	= event.srcElement.selectedIndex;
			}	// if

		// Text/edit
		else if (type == "text")
			dct["Value"]	= event.srcElement.value;

		// Unhandled type
		else
			send = false;

		// Transmit store
		if (send == true)
			{
			// Convert to XML
			xml = nSpaceXML.save(dct);

			// Transmit
			ws.send ( xml );
			}	// if
		}	// onChangen

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
		var type = event.srcElement["type"];
		var dct  = {};
		var xml	= null;
		var send = true;

		// Store template
		dct["Verb"]			= "Store";
		dct["Location"]	= event.srcElement.attributes["data-nabs"] + "Activate/Fire";

		// Button
		if (type == "button")
			{
			// For a button, value does not matter
			dct["Value"]		= 0;
			}	// if

		// Checkbox
		else if (type == "checkbox")
			{
			// Checked state
			dct["Value"]		= (event.srcElement.checked == true) ? true : false;
			}	// else

		// Unhandled type
		else
			send = false;

		// Transmit store
		if (send == true)
			{
			// Convert to XML
			xml = nSpaceXML.save(dct);

			// Transmit
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
	/////////////	///////////////////////////////////////////////////////////

	// Document ready ?
	if (document.readyState == "complete")
		nSpace_bind.init();		
	}	// onreadystatechange
