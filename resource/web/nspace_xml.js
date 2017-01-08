////////////////////////////////////////////////////////////////////////
//
//									NSPACE_XML.JS
//
//							nSpace XML value parser.
//
////////////////////////////////////////////////////////////////////////

//
// Class - nSpaceXML. Object to parse nSpace values from/to XML strings.
//
 
var nSpaceXML =
	{
	init : function() 
		{ 
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Initialize state of the object
		//
		////////////////////////////////////////////////////////////////////////
		},

	load : function(elem)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Load a value from an XML document
		//
		//	PARAMETERS
		//		-	elem is the root element of the XML document
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
				if (	((key = nSpaceXML.load(elem.childNodes[i+0])) != null) &&
						((val = nSpaceXML.load(elem.childNodes[i+1])) != null) )
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
				if ((val = nSpaceXML.load(elem.childNodes[i])) != null)
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
		},	// load

	save : function(value)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Save value to XML string
		//
		//	PARAMETERS
		//		-	value is the value to save
		//
		// RETURN VALUE
		//		XML string
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
				ret += nSpaceXML.save(key);
				ret += nSpaceXML.save(value[key]);
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
		}	// load

	}	// nSpaceXML
