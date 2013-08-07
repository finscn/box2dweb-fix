var fs = require('fs');
var cp = require('child_process');
var Path = require('path');
var mkdirp = require('mkdirp');
var wrench = require('wrench');
var glob = require('glob');


var inputDir = Path.normalize("./lib/**/*.js");
var files = glob.sync(inputDir, {});

var startFlag="(function(";
var endFlag1="}())";
var endFlag2="})()";

function isHasScope(code){
   var idx1=code.indexOf(startFlag);
   var idx2=code.indexOf(endFlag1);
   var idx3=code.indexOf(endFlag2);
   var len=code.length;

   if (idx1<3 && (idx2>len-endFlag1.length-2 || idx3>len-endFlag2.length-2)){
      return true
   }
   return false;

}

// files.forEach(function(file){
//    // return;
//    var code=fs.readFileSync(file,'utf8');

//    if (!isHasScope(code.trim())){
//       code=";(function(){\n"+code+"\n}());";
//       fs.writeFileSync(file,code);
//    }else{
      
//       console.log(file)
//    }
   
// })


