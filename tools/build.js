var fs = require('fs');
var cp = require('child_process');
var Path = require('path');
var mkdirp = require('mkdirp');
var wrench = require('wrench');
var glob = require('glob');

var inputDir="./input/";
var outputDir="./output/";

var head="head.js";
var preDef="pre-definitions.js";
var bodys=[
   "body-0.js",
   "body-1.js",
   "body-2.js",
   "body-3.js",
   "body-4.js",
   "body-5.js",
   "body-6.js",
   "body-7.js",
   "body-8.js",
]
var tail="tail.js";

var dirMap=[
   "0-Collision",
   "1-Shape",
   "2-Common",
   "3-Common",
   "4-Dynamics",
   "5-Contact",
   "6-Controller",
   "7-Joints",
   "8-Dynamics",

]


var headStr=fs.readFileSync(inputDir+head,'utf8');
var preDefStr=fs.readFileSync(inputDir+preDef,'utf8');
var bodysStr=[];
bodys.forEach(function(f){
   bodysStr.push( fs.readFileSync(inputDir+f,'utf8') );
})
var tailStr=fs.readFileSync(inputDir+tail,'utf8');

var defMap={}

var files=preDefStr.split("function b2");
files.forEach(function(file){
   file=file.trim();
   if (!file){
      return;
   }
   var idx=file.indexOf("(");
   var name=file.substring(0,idx);
   name="b2"+name;
   file="function b2"+file+"\n";
   defMap[name]={
      def : file
   };
})

var bodySpInfo=[]
var globalIdx=0;

var jsFileMap=[];

bodysStr.forEach(function(bodyStr,fileIdx){
   bodyStr=bodyStr.trim();
   if (!bodyStr){
      return;
   }
   var sp=bodySpInfo[fileIdx]=bodySpInfo[fileIdx]||[];
   var spMap={}
   for (var name in defMap){
      var inheritIdx= bodyStr.indexOf("Box2D.inherit("+name+",");
      var nameNameIdx=bodyStr.indexOf(name+"."+name+" =");
      if (inheritIdx>=0){
         sp.push([globalIdx++,inheritIdx,name,fileIdx]);
      }else if(nameNameIdx>=0){
         sp.push([globalIdx++,nameNameIdx,name,fileIdx]);
      }
   }
   sp.sort(function(a,b){
      return a[0]-b[0]
   });

   for (var i=0,len=sp.length;i<len;i++){
      var startIdx=sp[i][1];
      var name=sp[i][2];
      var fileIdx=sp[i][3];
      if (i+1<len){
         var endIdx=sp[i+1][1];
      }else{
         endIdx=bodyStr.length;
      }
      var core=bodyStr.substring(startIdx,endIdx);
      jsFileMap.push({
         name : name,
         def : defMap[name].def,
         core : core,
         fileIdx : fileIdx
      });
      defMap[name].core=core;
   }

})

var maked={}
var lastDir=-1;
jsFileMap.forEach(function(js){
   var jsName=js.name;
   if (lastDir!=js.fileIdx){
      lastDir=js.fileIdx;
      console.log( "<!-- "+dirMap[lastDir]+" -->");
   }
   var i=0;
   while (maked[jsName]){
      jsName=jsName+"-"+(++i)
   }
   maked[jsName]=true;
   var code=js.def+js.core;
   var oDir=outputDir+dirMap[lastDir];

    if (!fs.existsSync(oDir+"/")) {
       mkdirp.sync(oDir+"/");
    }

   var fileName=oDir+"/"+jsName+".js";
   fs.writeFileSync(fileName,code);
   console.log("<script src=\""+oDir+"/"+jsName+".js\"></script>")
})

for (var name in defMap){
   if (!defMap[name].core){
      console.log("bad : ",name)
   }
}
