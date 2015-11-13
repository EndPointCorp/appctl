# Liquid Galaxy

Appctl is a ROS node that lets you run processes on your system on the
basis of "Modes". It manages processes running on [Liquid
Galaxy](https://github.com/EndPointCorp/lg_ros_nodes).

# General requirements

NOTE: all requirements specific to ros_nodes are in their respective
README.md files

- [Ubuntu 14.04 LTS](http://releases.ubuntu.com/14.04/)
- [ros-indigo](http://wiki.ros.org/indigo)

## Development

LINT is configured, run `pep8` in the root of this repo to check Python
and use `catkin_lint` to check for errors in `package.xml` and
`CMakeLists.txt`.

## Making new release

- To make new release you need to:

```shell
$ catkin_generate_changelog
```

- Then edit all your `.rst` changelogs - remove unwanted or bogus messages
and make them look pretty. Use `catkin_generate_changelog --all` to
create `CHANGELOG.rst` for a new package.

- Once that's done, prepare release:

```shell
$ catkin_prepare_release
```
