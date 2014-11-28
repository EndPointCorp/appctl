---
title: 'Portal Selenium Tests 0.1 documentation'
...

### Navigation

-   [Portal Selenium Tests 0.1 documentation](index.html#document-index)
    »

Portal Selenium Tests’s documentation[¶](#portal-selenium-tests-s-documentation "Permalink to this headline")
=============================================================================================================

Description of the test modules and classes grouping and particular test
cases in the **tests/extensions/functional/tests** portal project
package.

The test cases have **test\_** prefix in their names.

tests/extensions/functional/tests package[¶](#tests-extensions-functional-tests-package "Permalink to this headline")
---------------------------------------------------------------------------------------------------------------------

### test\_display\_kiosk module[¶](#module-test_display_kiosk "Permalink to this headline")

The display extension tests.

The kiosk extension tests.

Cross-verification of DOM elements between the display and kiosk
extensions.

 *class*`test_display_kiosk.`{.descclassname}`TestBaseDisplay`{.descname}[¶](#test_display_kiosk.TestBaseDisplay "Permalink to this definition")
:   Tests related to the display extension.

     `test_widgets_not_displayed`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_display_kiosk.TestBaseDisplay.test_widgets_not_displayed "Permalink to this definition")
    :   Test that the following graphical widgets (elements) are not displayed:
        :   -   zoom in/out buttons,
            -   compass,
            -   google More Fun menu,
            -   search field,
            -   search button,
            -   runway panel (Points of Interest, Famous Places).

        After the browser loads the initial URL, it takes some time
        until also the extensions are fully loaded and until the DOM is
        modified accordingly.

 *class*`test_display_kiosk.`{.descclassname}`TestBaseKioskExtension`{.descname}[¶](#test_display_kiosk.TestBaseKioskExtension "Permalink to this definition")
:   Tests related to the kiosk extension.

     `test_widgets_displayed`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_display_kiosk.TestBaseKioskExtension.test_widgets_displayed "Permalink to this definition")
    :   Test that the following graphical widgets (elements) are displayed:
        :   -   zoom in/out buttons,
            -   compass,
            -   google More Fun menu,
            -   search field,
            -   search button,
            -   runway panel (Points of Interest, Famous Places).

        If elements from the elements list are renamed, the test
        **test\_widgets\_not\_displayed()** would just test that they
        are not present, however they might be, but under a different
        name. Test that exactly the same are present when the display
        extension is not loaded, i.e. with kiosk extension loaded.

### test\_general module[¶](#module-test_general "Permalink to this headline")

General Portal selenium tests.

 *class*`test_general.`{.descclassname}`TestMiscellaneous`{.descname}[¶](#test_general.TestMiscellaneous "Permalink to this definition")
:   Other test cases not fitting any other current category.

     `test_eu_cookies_info_bar_is_hidden`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_general.TestMiscellaneous.test_eu_cookies_info_bar_is_hidden "Permalink to this definition")
    :   Test that the EU cookies info bar is invisible.

        And the map canvas has set top=0px, because the cookie container
        is 30 px higher, and the canvas is moved 30 px down.

 *class*`test_general.`{.descclassname}`TestSearch`{.descname}[¶](#test_general.TestSearch "Permalink to this definition")
:   Test class related to the search box, search button. Various
    interaction scenarios.

     `test_no_searchbox_on_other_planets`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_general.TestSearch.test_no_searchbox_on_other_planets "Permalink to this definition")
    :   The test loads the initial MAPS\_URL and zooms out. After this
        zoom out, the universe objects appear (Earth, Moon, Mars).

        The search box should not be visible when Moon, Mars are
        clicked.

        First Moon is clicked, disappearance of search box is verified.
        Second, the Earth is clicked, verify search box appeared. Last,
        Mars is clicked, disappearance of search box is verified.

     `test_search_clicking_search_button`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_general.TestSearch.test_search_clicking_search_button "Permalink to this definition")
    :   Test that camera coordinates were changed from the initial
        position to something different after performing the search.

        Interacts with search box and clicks on the search button.

     `test_search_hitting_return_on_search_box`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_general.TestSearch.test_search_hitting_return_on_search_box "Permalink to this definition")
    :   Test that camera coordinates were changed from the initial
        position to something different after performing the search.

        Interacts with search box and hits the return key on on the
        search box.

     `test_search_hitting_return_on_search_button`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_general.TestSearch.test_search_hitting_return_on_search_button "Permalink to this definition")
    :   Test that camera coordinates were changed from the initial
        position to something different after performing the search.

        Interacts with search box and sends return key event on the
        search button.

### test\_google\_menu module[¶](#module-test_google_menu "Permalink to this headline")

Google Menu related tests.

 *class*`test_google_menu.`{.descclassname}`TestGoogleMenu`{.descname}[¶](#test_google_menu.TestGoogleMenu "Permalink to this definition")
:   Google Menu tests.

     `test_clicking_doodle_item`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_google_menu.TestGoogleMenu.test_clicking_doodle_item "Permalink to this definition")
    :   Test that clicking on the doodle item changes the URL to the
        doodles page.

     `test_google_items_are_visible_on_click`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_google_menu.TestGoogleMenu.test_google_items_are_visible_on_click "Permalink to this definition")
    :   Test that Google Menu (More fun) items are visible after
        clicking it.

     `test_google_menu_is_visible`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_google_menu.TestGoogleMenu.test_google_menu_is_visible "Permalink to this definition")
    :   Test that Google Menu (More fun) is displayed along with some
        items.

### test\_platform module[¶](#module-test_platform "Permalink to this headline")

Browser platform tests (webgl, drivers etc).

 *class*`test_platform.`{.descclassname}`TestPlatform`{.descname}[¶](#test_platform.TestPlatform "Permalink to this definition")
:   Following tests will check whether:
    :   -   version of the browser matches the version in config.json

    Check:
    :   -   gpu\_data.clientInfo.version =\> Chrome/37.0.2062.120
        -   gpu\_data.gpuInfo.basic\_info[n] =\> description : ‘Direct
            Rendering’

    Log:
    :   -   gpu\_data.gpuInfo.basic\_info[n] =\> description :
            GL\_VERSION

     `test_get_chrome_gpu_data`{.descname}()[¶](#test_platform.TestPlatform.test_get_chrome_gpu_data "Permalink to this definition")
    :   Check if the chrome supports GPU.

### test\_ros module[¶](#module-test_ros "Permalink to this headline")

Tests involving ROS communication between browsers.

 *class*`test_ros.`{.descclassname}`TestBaseSingleBrowserROS`{.descname}[¶](#test_ros.TestBaseSingleBrowserROS "Permalink to this definition")
:   Tests involving single browser.

    Listening and asserting ROS traffic based on the actions performed
    in the (kiosk extension) browser. Browser runs in one process,
    another process is ROS topic listener.

     `test_ros_position_after_search`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_ros.TestBaseSingleBrowserROS.test_ros_position_after_search "Permalink to this definition")
    :   Run browser and type something in the search box, a place with a
        known position.

        Helper background process listening on
        **/portal\_kiosk/current\_pose** ROS topic and checks ROS
        position messages until the one expected arrives (within certain
        timeout).

        Pass/fail flag is set accordingly by the listener process via
        shared memory value which this test cases evaluates eventually.

 *class*`test_ros.`{.descclassname}`TestBaseTwoBrowsersROS`{.descname}[¶](#test_ros.TestBaseTwoBrowsersROS "Permalink to this definition")
:   Tests involving two browsers.

    General idea is to selenium-manipulate one browser (with kiosk
    extension) and assert accordingly adjusted state (as propagated
    through ROS) in the other browser (display extension).

     `test_ros_positions_in_browsers_aligned_after_kiosk_search`{.descname}()[¶](#test_ros.TestBaseTwoBrowsersROS.test_ros_positions_in_browsers_aligned_after_kiosk_search "Permalink to this definition")
    :   Perform search in the kiosk browser and assert on the
        automatically synchronized final position in the display
        browser.

### test\_runway module[¶](#module-test_runway "Permalink to this headline")

Tests related to Runway elements.

The Runway is the element at to bottom of the touchscreen browser, it
contains a list of Points of Interests (if the camera is zoomed in to
some level, and there are some interesting places around) or the list
consisting of the Earth, Moon, Mars.

There is also a list of Famous Places (Earth tours), which is always
filled (loaded by a static list of entries read from a file).

 *class*`test_runway.`{.descclassname}`TestRunway`{.descname}[¶](#test_runway.TestRunway "Permalink to this definition")
:   Runway element interaction tests.

     `prepare_poi`{.descname}()[¶](#test_runway.TestRunway.prepare_poi "Permalink to this definition")
    :   Prepare for Points of Interest tests.

        Load browser, search for a location with POIs and wait some time
        for the runway tray to populate.

     `test_runway_buttons_basic`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_runway.TestRunway.test_runway_buttons_basic "Permalink to this definition")
    :   Test that Point of Interest and Famous Places are displayed.

     `test_runway_check_earth_icon_click`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_runway.TestRunway.test_runway_check_earth_icon_click "Permalink to this definition")
    :   The Earth icon (most left picture), clicking it should bring the
        view to a considerably zoomed out position.

        NB: position object values differ between subsequent runs.

     `test_runway_planets_on_max_zoom_out`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_runway.TestRunway.test_runway_planets_on_max_zoom_out "Permalink to this definition")
    :   Test there is Mars, Earth, Moon loaded in Points of Interest
        tray on maximal zoom out.

     `test_runway_points_of_interest`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_runway.TestRunway.test_runway_points_of_interest "Permalink to this definition")
    :   Test Points of Interest is loaded, we can load one, exit it and
        and load another Point of Interest.

### test\_zoom module[¶](#module-test_zoom "Permalink to this headline")

Tests related to Zoom buttons, zoom operation.

 *class*`test_zoom.`{.descclassname}`TestZoomButtons`{.descname}[¶](#test_zoom.TestZoomButtons "Permalink to this definition")
:   Tests for checking the zoom buttons are functional.

     `test_zoom_buttons`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_zoom.TestZoomButtons.test_zoom_buttons "Permalink to this definition")
    :   Test that the zoom in and out buttons are displayed.

     `test_zoom_in_button_change`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_zoom.TestZoomButtons.test_zoom_in_button_change "Permalink to this definition")
    :   Test clicks on the zoom out button and checks the pose object
        coordinates (altitude, latitude, longitude) according change.

     `test_zoom_out_button_change`{.descname}(*\*args*, *\*\*kwargs*)[¶](#test_zoom.TestZoomButtons.test_zoom_out_button_change "Permalink to this definition")
    :   Test clicks on the zoom in button and checks the pose object
        coordinates (altitude, latitude, longitude) according change.

### [Table Of Contents](index.html#document-index)

-   [tests/extensions/functional/tests
    package](index.html#document-modules)
    -   [test\_display\_kiosk
        module](index.html#document-test_display_kiosk)
    -   [test\_general module](index.html#document-test_general)
    -   [test\_google\_menu
        module](index.html#document-test_google_menu)
    -   [test\_platform module](index.html#document-test_platform)
    -   [test\_ros module](index.html#document-test_ros)
    -   [test\_runway module](index.html#document-test_runway)
    -   [test\_zoom module](index.html#document-test_zoom)

### Navigation

-   [Portal Selenium Tests 0.1 documentation](index.html#document-index)
    »

© Copyright 2014, End Point. Created using
[Sphinx](http://sphinx-doc.org/) 1.3b1.
