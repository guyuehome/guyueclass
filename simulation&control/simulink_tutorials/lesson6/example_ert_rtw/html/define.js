function CodeDefine() { 
this.def = new Array();
this.def["example_U"] = {file: "example_c.html",line:20,type:"var"};
this.def["example_Y"] = {file: "example_c.html",line:23,type:"var"};
this.def["example_M_"] = {file: "example_c.html",line:26,type:"var"};
this.def["example_M"] = {file: "example_c.html",line:27,type:"var"};
this.def["example_step"] = {file: "example_c.html",line:30,type:"fcn"};
this.def["example_initialize"] = {file: "example_c.html",line:44,type:"fcn"};
this.def["example_terminate"] = {file: "example_c.html",line:59,type:"fcn"};
this.def["ExtU_example_T"] = {file: "example_h.html",line:43,type:"type"};
this.def["ExtY_example_T"] = {file: "example_h.html",line:48,type:"type"};
this.def["P_example_T"] = {file: "example_types_h.html",line:23,type:"type"};
this.def["RT_MODEL_example_T"] = {file: "example_types_h.html",line:26,type:"type"};
this.def["example_P"] = {file: "example_data_c.html",line:20,type:"var"};
this.def["int8_T"] = {file: "rtwtypes_h.html",line:47,type:"type"};
this.def["uint8_T"] = {file: "rtwtypes_h.html",line:48,type:"type"};
this.def["int16_T"] = {file: "rtwtypes_h.html",line:49,type:"type"};
this.def["uint16_T"] = {file: "rtwtypes_h.html",line:50,type:"type"};
this.def["int32_T"] = {file: "rtwtypes_h.html",line:51,type:"type"};
this.def["uint32_T"] = {file: "rtwtypes_h.html",line:52,type:"type"};
this.def["real32_T"] = {file: "rtwtypes_h.html",line:53,type:"type"};
this.def["real64_T"] = {file: "rtwtypes_h.html",line:54,type:"type"};
this.def["real_T"] = {file: "rtwtypes_h.html",line:60,type:"type"};
this.def["time_T"] = {file: "rtwtypes_h.html",line:61,type:"type"};
this.def["boolean_T"] = {file: "rtwtypes_h.html",line:62,type:"type"};
this.def["int_T"] = {file: "rtwtypes_h.html",line:63,type:"type"};
this.def["uint_T"] = {file: "rtwtypes_h.html",line:64,type:"type"};
this.def["ulong_T"] = {file: "rtwtypes_h.html",line:65,type:"type"};
this.def["char_T"] = {file: "rtwtypes_h.html",line:66,type:"type"};
this.def["uchar_T"] = {file: "rtwtypes_h.html",line:67,type:"type"};
this.def["byte_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};
this.def["creal32_T"] = {file: "rtwtypes_h.html",line:78,type:"type"};
this.def["creal64_T"] = {file: "rtwtypes_h.html",line:83,type:"type"};
this.def["creal_T"] = {file: "rtwtypes_h.html",line:88,type:"type"};
this.def["cint8_T"] = {file: "rtwtypes_h.html",line:95,type:"type"};
this.def["cuint8_T"] = {file: "rtwtypes_h.html",line:102,type:"type"};
this.def["cint16_T"] = {file: "rtwtypes_h.html",line:109,type:"type"};
this.def["cuint16_T"] = {file: "rtwtypes_h.html",line:116,type:"type"};
this.def["cint32_T"] = {file: "rtwtypes_h.html",line:123,type:"type"};
this.def["cuint32_T"] = {file: "rtwtypes_h.html",line:130,type:"type"};
this.def["pointer_T"] = {file: "rtwtypes_h.html",line:148,type:"type"};
}
CodeDefine.instance = new CodeDefine();
var testHarnessInfo = {OwnerFileName: "", HarnessOwner: "", HarnessName: "", IsTestHarness: "0"};
var relPathToBuildDir = "../ert_main.c";
var fileSep = "\\";
var isPC = true;
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["example_c.html"] = "../example.c";
	this.html2Root["example_c.html"] = "example_c.html";
	this.html2SrcPath["example_h.html"] = "../example.h";
	this.html2Root["example_h.html"] = "example_h.html";
	this.html2SrcPath["example_private_h.html"] = "../example_private.h";
	this.html2Root["example_private_h.html"] = "example_private_h.html";
	this.html2SrcPath["example_types_h.html"] = "../example_types.h";
	this.html2Root["example_types_h.html"] = "example_types_h.html";
	this.html2SrcPath["example_data_c.html"] = "../example_data.c";
	this.html2Root["example_data_c.html"] = "example_data_c.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.html2SrcPath["rtmodel_h.html"] = "../rtmodel.h";
	this.html2Root["rtmodel_h.html"] = "rtmodel_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"example_c.html","example_h.html","example_private_h.html","example_types_h.html","example_data_c.html","rtwtypes_h.html","rtmodel_h.html"];
