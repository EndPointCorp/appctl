
// Elements are shown in this order
// name 	- used as a key, image name
// desc 	- shown below the image
// action - (switch_display|showDoodlesPage)
// url 		- url to switch by the action switch_display
var data = [
	{name: "earth",   desc: "Earth",   action:"switch_display", url:"earth URL"},
  {name: "doodles", desc: "Doodles", action:"showDoodlesPage"}
];

// Sends ROS message to display to change the browser's URL
var sendSwitchROSMessage = function(url) {
	// TODO invent some protocol message with url inside
  // TODO send proper message
	// TODO add support at display's side
	alert("Dear display, please switch to " + url);
};


var showDoodlesPage = function() {
  // TODO: implement the doodles page
	// TODO: add support for sending the ROS message to switch URL on display
	document.location = chrome.extension.getURL("pages/doodles.html');
};

var createElementsList = function() {
	
	var ul = document.createElement('ul');
	for(var i = 0 ; i < data.length ; i++ ) {
		
		var name   = data[i].name;
		var desc   = data[i].desc;
		var action = data[i].action;

		var li = document.createElement('li');
		
		var img = document.createElement('img');
		img.setAttribute('src', chrome.extension.getURL("images/" + name + '.jpg'));

    var span = document.createElement('span');
    span.innerText = desc;

		li.appendChild(img);
		li.appendChild(span);

		if (action == "switch_display")	li.onclick = sendSwitchROSMessage(url);
		if (action == "showDoodlesPage") li.onclick = showDoodlesPage();

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
