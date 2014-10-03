var portalRos = new ROSLIB.Ros({
  url: 'wss://42-b:9090'
});

var displaySwitchTopic = new ROSLIB.Topic({
  ros: portalRos,
  name: '/display/switch',
  messageType: 'std_msgs/String'
});

displaySwitchTopic.advertise();

// Elements are shown in this order
// name   - used as a key
// desc   - shown below the image
// icon   - the image used for an icon, must be in the 'images' directory
// action - (switch_display|showDoodlesPage)
// url     - url to switch by the action switch_display
var earthURL = 'http://lg-head/portal-loader.html';

var data = [
  {
    name: 'earth',
    desc: 'Earth',
    icon: 'icon_earth.png',
    action: 'switch_display',
    url: earthURL
  },
  {
    name: 'doodles',
    desc: 'Arcade',
    icon: 'doodles.png',
    action: 'showDoodlesPage'
  }
];

// Sends ROS message to display to change the browser's URL
var sendSwitchROSMessage = function(e) {
  var url = e.target.getAttribute('switch_url');
  sendSwitchROSMessageURL(url);
};

var sendSwitchROSMessageURL = function(url) {
  console.log('Trying to switch display to', url);
  var msg = new ROSLIB.Message({data: url});
  displaySwitchTopic.publish(msg);
};

var showDoodlesPage = function() {
  document.location = chrome.extension.getURL('pages/doodles.html');
};

var goBackToEarthPage = function() {
  sendSwitchROSMessageURL(earthURL);
  document.location = earthURL;
};

var createElementsList = function() {

  var ul = document.createElement('ul');
  for (var i = 0; i < data.length; i++) {

    var name = data[i].name;
    var desc = data[i].desc;
    var action = data[i].action;
    var url = data[i].url;
    var icon = data[i].icon;

    var li = document.createElement('li');
    li.setAttribute('switch_url', url);

    var img = document.createElement('img');
    img.setAttribute('src', chrome.extension.getURL('images/' + icon));
    img.setAttribute('switch_url', url);

    var span = document.createElement('span');
    span.innerText = desc;
    span.setAttribute('switch_url', url);

    li.appendChild(img);
    li.appendChild(span);

    if (action == 'switch_display') {
      li.onclick = goBackToEarthPage;
      li.addEventListener(
        'touchstart',
        goBackToEarthPage,
        true
      );
    }
    if (action == 'showDoodlesPage') {
      li.onclick = showDoodlesPage;

      li.addEventListener(
        'touchstart',
        showDoodlesPage,
        true
      );
    }

    ul.appendChild(li);
  }

  return ul;

};

var initializeMoreFun = function() {

  var onclick = function() {
    document.getElementById('morefun_items').style.visibility = 'visible';
  };

  var i = document.createElement('img');
  i.setAttribute('src', chrome.extension.getURL('images/icon_grid_grey.png'));

  var s = document.createElement('div');
  s.id = 'morefun';
  var span = document.createElement('span');
  span.innerText = 'More fun';
  s.onclick = onclick;
  s.appendChild(span);
  s.appendChild(i);

  document.body.appendChild(s);

  var d = document.createElement('div');
  d.id = 'morefun_items';

  var t = document.createElement('img');
  t.setAttribute('src', chrome.extension.getURL("images/arrow.png"));
  t.setAttribute('id', "arrow");

  d.appendChild(t);

  d.appendChild(createElementsList());
  d.style.visibility = 'hidden';

  document.body.appendChild(d);

  // and we should close the morefun_items window when clicked anywhere else...
  document.body.addEventListener(
      'click',
      function(e) {
        if (e.target.id != 'morefun_items')
          document.getElementById('morefun_items').style.visibility = 'hidden';
      },
      true
  );
  document.body.addEventListener(
      'touchstart',
      function(e) {
        if (e.target.id != 'morefun_items')
          document.getElementById('morefun_items').style.visibility = 'hidden';
      },
      true
  );
};

initializeMoreFun();
