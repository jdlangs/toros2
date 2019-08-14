Tool for basic automatic porting of ROS1 C++ code to ROS2 equivalents

### Installation

This tool is configured to be installed through pip rather than run directly. After cloning the
repository, run:

    pip3 install --user .

This will install a `toros2` executable script in `~/.local/bin` that can be run from any location.
The `--editable` flag can also be added to `pip3 install` to allow you to change the source code and
have the changes take effect without reinstalling the package.
