
// Elements are shown in this order
var data = [
  {name: "youtube", desc: "YouTube", action="switch_display", url="youtube"}, 
	{name: "earth",   desc: "Earth",   action="switch_display", url="youtube"},
  {name: "doodles", desc: "Doodles", action="showDoodlesPage"}
];

var sendSwitchROSMessage = function(message) {
  
};

var showDoodlesPage = function() {

}

var createElementsList = function() {
	
	var ul = document.createElement('ul');
	for(var i = 0 ; i < data.length ; i++ ) {
		
		var name = data[i].name;
		var desc = data[i].desc;

		var li = document.createElement('li');
		
		var img = document.createElement('img');
		img.setAttribute('src', chrome.extension.getURL("images/" + name + '.jpg'));

    var span = document.createElement('span');
    span.innerText = desc;

		li.appendChild(img);
		li.appendChild(span);

		li.onclick = sendSwitchROSMessage(name);

		ul.appendChild(li);
	}

	return ul;

};

var initializeMoreFun = function() {

  var onclick = function() {
		document.getElementById("morefun_items").style.visibility='visible';
	};

  var s = document.createElement('div');
	s.id = "morefun";
  s.innerText = "More fun --ICON HERE--";
	s.onclick = onclick;
  document.body.appendChild(s);

	var d = document.createElement('div');
	d.id = "morefun_items";
	
	d.appendChild(createElementsList());
	d.style.visibility='hidden';
	console.log(d);
	document.body.appendChild(d);
};

initializeMoreFun();
