# Liquid Galaxy

`appctl` is rospy library for running processes while the graph is in a particular state. It is intended to be a distributed init system for the [Liquid Galaxy](https://github.com/EndPointCorp/lg_ros_nodes) project.

See the README.md in `appctl/` for details.

# General requirements

- [ros-indigo](http://wiki.ros.org/indigo)

## Development

LINT is configured, run `pep8` in the root of this repo to check Python
and use `catkin_lint` to check for errors in `package.xml` and
`CMakeLists.txt`.

### Running tests

Use the `dev_tests` script to run the tests quickly.

Use the `ci_tests` script to run the tests cleanly.

### Building the package

Use the `ci_build` script to run the tests and produce a debian package in the `.build/` directory.

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
