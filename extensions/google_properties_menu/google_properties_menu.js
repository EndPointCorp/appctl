
var initializeMoreFun = function() {

  var oncl = function() {alert('aaa');};

  var s = document.createElement('div');
	s.id = "morefun";
  s.innerText = "More fun --ICON HERE--";
	s.onclick = oncl;
  document.body.appendChild(s);
};

initializeMoreFun();
