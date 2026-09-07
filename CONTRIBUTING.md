# Contributing to object_with_region

Thanks for your interest in contributing!

## Reporting issues

Please open an issue describing the problem, the ROS 2 distribution and
platform you're using, and steps to reproduce it.

## Submitting changes

1. Fork the repository and create a branch for your change.
2. Keep commits focused and follow [Conventional Commits](https://www.conventionalcommits.org/)
   for commit messages.
3. Make sure the linters pass before opening a pull request:

       colcon test --packages-select object_with_region
       colcon test-result --verbose

4. Update `CHANGELOG.rst` with a summary of your change.
5. Open a pull request describing the change and its motivation.

## License

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~
