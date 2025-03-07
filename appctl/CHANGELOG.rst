^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package appctl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.1 (2025-01-24)
------------------
* Fix Callable Python 3.10 compatibility
* Contributors: Matt Vollrath

3.1.0 (2023-04-17)
------------------
* Merge pull request `#12 <https://github.com/endpointcorp/appctl/issues/12>`_ from EndPointCorp/std_redirect
  adding stdout/stderr as options for appctl
* Merge branch 'master' of github.com:endpointcorp/appctl into std_redirect
* adding stdout/stderr as options for appctl
* Contributors: Jacob Minshall

3.0.2 (2020-04-23)
------------------

3.0.1 (2019-12-11)
------------------
* Merge pull request `#11 <https://github.com/EndPointCorp/appctl/issues/11>`_ from EndPointCorp/python3_change
  Python3 msg/srv changes
* package.xml dep fixes and remove some files
* Merge branch 'master' of github.com:EndPointCorp/appctl into python3_change
  Conflicts:
  Dockerfile
  appctl/src/appctl_support/awesome.py
* Merge branch 'python3_change' of github.com:EndPointCorp/appctl into python3_change
* appctl_msg_defs: update msg/srv imports
* make seperate appctl_msg_defs
* pep8 > pycodestyle
* Contributors: Will Plaut

3.0.0 (2019-10-31)
------------------

2.0.2 (2019-10-31)
------------------

2.0.1 (2019-06-28)
------------------

2.0.0 (2019-06-14)
------------------
* skipping test
* pycodestyle'd
* Contributors: Jacob Minshall

1.2.2 (2019-06-13)
------------------

1.2.1 (2017-10-10)
------------------
* some applications in the stopped state don't have a watcher
* Contributors: Jacob Minshall

1.2.0 (2017-08-17)
------------------
* Add env arg to ProcController and ProcRunner
* Contributors: Matt Vollrath

1.1.1 (2016-08-29)
------------------
* Expand ProcController docstrings and tests
* Add atexit hooks to guarantee ProcRunner closure
  Switch to daemon thread to facilitate this.
* Contributors: Matt Vollrath

1.1.0 (2016-08-23)
------------------
* Fix ProcController gc behavior
  * Do not register a shutdown hook.
  * Add close() method which is called from __del_\_().
  * Add cleanup tests.
* Add destructor to ProcRunner, refine gc test
* Contributors: Matt Vollrath

1.0.14 (2016-07-25)
-------------------
* Revert "Disable appctl mode abuse test"
  This reverts commit 0f041a80cfc289f6b158eaf15ec7dfc219e7d3b5.
  We actually want this.
* Revert "Remove unnecessary locking from ProcRunner"
  This reverts commit 44f3ede3fa2af64df2d217717affa8d16833ddc4.
  We actually need this.
* Disable appctl mode abuse test
  Gremlins are preventing messages from being received.
* Remove unnecessary locking from ProcRunner
  Prevent deadlock on concurrent ProcController operations.
* Re-introduce ProcController locking
* Add ProcController concurrency test
* Add start/stop validation to ProcController
* Revise ProcController docstring
* Contributors: Matt Vollrath

1.0.13 (2016-07-21)
-------------------
* remove lock from proc controller
* respawn flag tests, related to lg_media mplayer work
* Contributors: Wojciech Zieniewicz, Zdenek Maxa

1.0.12 (2016-06-10)
-------------------
* lg_media respawn mplayer behaviour to control,
   touch: `EndPointCorp/lg_ros_nodes#193 <https://github.com/EndPointCorp/lg_ros_nodes/issues/193>`_
* Contributors: Zdenek Maxa

1.0.11 (2016-04-18)
-------------------
* soft relaunch support for appctl
  Processes will be killed on soft relaunches.
* get_pid function to grab proc_runner's pid
* Contributors: Jacob Minshall

1.0.10 (2016-01-28)
-------------------
* Rename ProcRunner._spawn() to _run_spawn_hooks()
* One less logging statement in ProcRunner
* Lock critical sections of ProcRunner
* Remove unneeded DEVNULL from ProcRunner
* Remove ProcRunner._proc_is_alive()
* Lock critical sections of ProcController
  Also remove some logging.
* Add Mode Spin test to repro race condition
* Contributors: Matt Vollrath

1.0.9 (2016-01-08)
------------------
* Comment cleanup in ProcRunner
* Remove psutil dependency
* Simplify ProcRunner
  * Grace delay in cleanup test
  * Remove zombie detection, always wait()
* tests: increase delay and change default test_cmd
  The grace delay of 0.1 was not long enough for some machines, especially
  when testing within a docker container. After increasing the delay > 1
  second I realized that using '/usr/bin/python' as the test command was
  not working as desired. It would respawn every second.
* pep8 fixes
* Contributors: Jacob Minshall, Matt Vollrath

1.0.8 (2015-12-17)
------------------
* proc_runner logic
  - added logging of zombie children and zombification of main process
  - added respawn limits with default of -1
* Don't kill zombie processes by default.
  https://github.com/EndPointCorp/lg_sv_nonfree/issues/6#issuecomment-165166855
* Contributors: Adam Vollrath, Matt Vollrath, Wojciech Ziniewicz

1.0.7 (2015-12-01)
------------------
* Improve+test ProcRunner cleanup
* Better nomenclature for spawn hooks
* De-nest proc life logic
* Guard against post-mortem respawn
* Wait after killing proc
* Remove "cheating" wait() from proc_runner test
* Contributors: Matt Vollrath

1.0.6 (2015-11-17)
------------------
* Bumped changelog
* Contributors: Wojciech Ziniewicz

1.0.5 (2015-11-17)
------------------
