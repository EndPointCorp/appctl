^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package appctl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
