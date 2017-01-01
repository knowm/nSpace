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
	var	ws = null;										// WebSocket object
	var	nElems = {};									// Bounded element states

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
			msg		= xmlToValue ( xmlDoc.documentElement );

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

			// Button
			if (type == "button")
				{
				// Enable
				if (loc == "element/enable/onfire/value")
					elem.disabled = !msg["Value"];
				}	// if

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
					elems[i].onclick = onClickElem;

					// Send listen request for path
					listen(path);
					}	// if

				}	// for

			// Debug message
//			ws.send("<Dictionary><Value>You</Value><Value Type=\"Double\">3.14159265358979323</Value></Dictionary>");
//			alert ( "Open!!" );
			},

		}	// nSpace_bind

	var listen = function(path)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Issue a 'listen' request for the specified path
		//
		//	PARAMETERS
		//		-	path is the path to listen to
		//
		////////////////////////////////////////////////////////////////////////
		var req = null;

		// Send XML request string.
		req = "<Dictionary>";

		// Verb
		req += "<Value>Verb</Value><Value>Listen</Value>";

		// Path
		req += "<Value>Path</Value><Value>"+path+"</Value>";

		// End of dictionary
		req += "</Dictionary>";

		// Send request
		if (ws != null)
			ws.send(req);

		}	// listen

	var xmlToValue = function(elem)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Convert XML document/node into a dictionary of values
		//
		//	PARAMETERS
		//		-	elem is the XML element
		//
		// RETURN VALUE
		//		Converted value
		//
		////////////////////////////////////////////////////////////////////////
		var ret = null;

		// Type
		if (elem.nodeName == "Dictionary")
			{
			var key = null;
			var val = null;

			// Empty dictionary
			var dct = {};

			// Load key/value pairs into dictionary
			for (var i = 0;i < elem.childNodes.length;i+=2)
				if (	((key = xmlToValue(elem.childNodes[i+0])) != null) &&
						((val = xmlToValue(elem.childNodes[i+1])) != null) )
					dct[key] = val;

			// Use as return value
			ret = dct;
			}	// if
		else if (elem.nodeName == "List")
			{
			var val = null;

			// Empty list
			var lst = [];

			// Load child values into list
			for (var i = 0;i < elem.childNodes.length;++i)
				if ((val = xmlToValue(elem.childNodes[i])) != null)
					lst.push ( val );

			// Use as return value
			ret = lst;
			}	// else if

		// Single value
		else if (elem.nodeName == "Value")
			{
			var val = null;

			// Type specified ?
			var type = (elem.attributes.length > 0) ?
							elem.attributes["Type"].nodeValue : null;

			// It is possible to receive an 'empty' value in which
			// case there is not 'firstChild'.
			if (	(elem.firstChild != null) &&
					(val = elem.firstChild.nodeValue) != null)
				{
				// Default
				if (type == null)
					type = "string";
				else 
					type = type.toLowerCase();

				// Convert type
				if (type == "float" || type == "double")
					val = parseFloat(val);
				else if (type == "int" || type == "long")
					val = parseInt(val);
				else if (type == "boolean")
					val = Boolean(val.toLowerCase() == "true");
				}	// if

			// Use as return value
			ret = val;
			}	// else if

		return ret;
		}	// xmlToValue

	var valueToXml = function(value)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Convert value to XML.
		//
		//	PARAMETERS
		//		-	value is the value to convert
		//
		// RETURN VALUE
		//		XML string representing value
		//
		////////////////////////////////////////////////////////////////////////
		var ret = null;
		var dct = {};
		var lst = [];

		// Dictionary
		if (value instanceof Object)
			{
			// Begin dictionary
			ret = "<Dictionary>"

			// Append values
			for (var key in value)
				{
				// Key then value
				ret += valueToXml(key);
				ret += valueToXml(value[key]);
				}	// for

			// End dictionary
			ret += "</Dictionary>";
			}	// if

		// List
		else if (value instanceof Array)
			{
			console.log("Array");
			}	// else if

		// Value
		else
			{
			// Begin value
			ret = "<Value";

			// String
			console.log(typeof value);
			if (typeof value == "string")
				ret += ">"+value.toString();

			// Number
			else if (typeof value == "number")
				{
				// Integer
				if (Number.isInteger(value))
					{
					ret += " Type=\"Integer\">";
					ret += value.toString();
					}	// else if
				else
					{
					ret += " Type=\"Float\">";
					ret += value.toString();
					}	// else

				}	// else if

			// Boolean
			else if (typeof value == "boolean")
				{
				ret += " Type=\"Boolean\">";
				ret += value.toString();
				}	// else if

			// Default
			else
				ret += ">"+value.toString();

			// End value
			ret += "</Value>";
			}	// else

		return ret;
		}	// valueToXml

	//
	// Events
	//

	// Buttons
	var onClickElem = function(event)
		{
		var dct  = {};
		var type = event.srcElement.nodeName.toLowerCase(); 

		// Button
		if (type == "button")
			{
			// Send activate value
			dct["Verb"]			= "Store";
			dct["Location"]	= event.srcElement.attributes["data-nabs"] + "Activate/OnFire/Value";
			dct["Value"]		= 0;
			var xml = valueToXml ( dct );
			ws.send ( xml );
			}	// if

		}	// onClickElem

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
