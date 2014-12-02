mirror
------

Full-screen desktop mirroring with `appctl` mode activation.

### capture.py

A node for capturing a desktop and providing a TCP server for streaming the video to a `playback.py` node.

##### Paramaters

* `modes` : Comma-separated list of activation modes.  See `appctl` README.md for more info.

* `port` : The port to run the server on.  Default `4953`.

* `display` : The Xorg display to capture.  Default `:0`.

##### TODO / Limitations

The video must be scaled to the playback device's screen resolution for best results.  This is presently hard-coded at 1920x1080 but could easily be a parameter.  Quality settings are based on this target resolution and may need tweaking for best results.

### playback.py

A node for connecting to a `capture.py` server and showing the captured desktop of its host.

##### Parameters

* `modes` : Comma-separated list of activation modes.  See `appctl` README.md for more info.

* `host` : Hostname of the capture server.

* `port` : The port of the capture server.  Default `4953`.

* `display` : The Xorg display to capture.  Default `:0`.

##### TODO / Limitations

Like the capture node, the resolution is assumed to be 1920x1080.  Also, the video capabilities must match the capture instance's since no RTP payloading is used.
