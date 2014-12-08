/*
  The configuration is read from the file taken from the dataURL variable.

  A couple of remarks:
  1 - the elements in the list are shown in the order
      they are defined in the config file

  2 - if there are no (display|kiosk)_url variables,
      then tapping the icon does nothing

  The example config looks like this:

{
    "portal": {
        "morefun_items": [
            {
                "display_url": "http://lg-head/portal-loader.html",
                "icon":        "https://lg-head/portal/morefun/icon_earth.png",
                "kiosk_url":   "http://lg-head/portal-loader.html",
                "name":        "Maps"
            },
            {
                "display_url": "",
                "icon":        "https://lg-head/portal/morefun/doodles.png",
                "kiosk_url":   "",
                "name":        "Arcade"
            },
            {
                "icon":        "https://lg-head/portal/morefun/timelapse.png",
                "name":        "Timelapse"
            }

        ]
    }
}

*/

// Elements are shown in this order
// name        - shown below the image
// icon        - the image used for an icon, must be in the 'images' directory
// display_url - url to switch the display browser
// kiosk_url   - url to switch the kiosk browser

//TODO: remove two below urls later
var earthURL = 'http://lg-head/portal-loader.html';
var timelapseURL = 'file:///mnt/earthtime/data-visualization-tools/examples/webgl-timemachine/landsat.html';

/* URL to download the config file from, must be HTTPS */
var dataURL = "https://lg-head/portal/config.json";
var data = null;

/* Downloads configuration file from lg-head, it must be done using https,
   as http requests are blocked by chrome.
   After downloading the file is parsed, and the section
   ['portal']['morefun_items'] is copied to the data variable.
*/
var downloadConfigData = function(url) {
  $.get( url )
    .done(function(d){ data = d.portal.morefun_items; initializeMoreFunMenu(); })
    .fail(function(){ console.error("Cannot download config file from: " + dataURL )});
}


/*
  Changes the display URL.
  Internally there is a ROS message sent with the URL.
  The display extension listens to that and redirects the browser.
*/
var changeDisplayURL = function(e) {
  var i = e.target.getAttribute('index');
  var url = data[i].display_url;

  if (url == null || url == "") {
    console.log("Cannot change display url to null");
    return;
  }

  console.log("Changing display url to " + url);
  chrome.runtime.sendMessage({url: url});
}

/*
  Changes the kiosk URL - currently this means changing the url
                          of the browser this extension runs at.
*/
var changeKioskURL = function(e) {
  var i = e.target.getAttribute('index');
  var url = data[i].kiosk_url;

  if (url == null || url == "") {
    console.log("Cannot change kiosk url to null");
    return;
  }

  console.log("Changing kiosk url to " + url);
  document.location = url;
}

/*
  Creates the element list for morefun_items element.
  This is made from the configuration read from lg-head
*/
var createElementsList = function() {

  var ul = document.createElement('ul');
  for (var i = 0; i < data.length; i++) {

    var name        = data[i].name;
    var display_url = data[i].display_url;
    var kiosk_url   = data[i].kiosk_url;
    var icon        = data[i].icon;

    var li = document.createElement('li');
    li.setAttribute('index', i);

    var img = document.createElement('img');
    img.setAttribute('src', icon);
    img.setAttribute('index', i);

    var span = document.createElement('span');
    span.innerText = name;
    span.setAttribute('index', i);

    li.appendChild(img);
    li.appendChild(span);

    li.addEventListener(
      'touchstart',
      changeKioskURL,
      true
    );

    li.addEventListener(
      'touchstart',
      changeDisplayURL,
      true
    );

    ul.appendChild(li);
  }

  return ul;

};

/*
  Creates all the more fun items according to the configuration
  from the data variable.
*/
var initializeMoreFunMenu = function() {

  var onclick = function() {
    document.getElementById('morefun_items').style.visibility = 'visible';
  };

  var i = document.createElement('img');
  i.setAttribute('src', chrome.extension.getURL('images/icon_grid_grey.png'));

  var s = document.createElement('div');
  s.id = 'morefun';
  var span = document.createElement('span');
  span.innerText = 'More fun';
  s.addEventListener('click', onclick, true);
  s.addEventListener('touchstart', onclick, true);
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
  var closeHandler = function(e) {
    if (e.target.id != 'morefun_items')
      document.getElementById('morefun_items').style.visibility = 'hidden';
  };
  document.body.addEventListener('click', closeHandler, true);
  document.body.addEventListener('touchstart', closeHandler, true);
};

downloadConfigData(dataURL);
