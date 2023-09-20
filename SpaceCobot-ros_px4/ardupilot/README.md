# Space Cobot Build Instructions:
- sudo apt-get install python-pip
- sudo pip install future
- sudo apt-get install -y pkg-config-arm-linux-gnueabihf
- git clone https://github.com/ardupilot/ardupilot
- ardupilot/Tools/scripts/install-prereqs-ubuntu.sh -y
- cd ardupilot/
- git submodule update --init
- ./waf configure --board=erlebrain2
- ./waf scobot

To allow GDB debug of the executables:
- ./waf configure --board=erlebrain2 --debug

Build examples from libraries:
- waf build --target examples/name_of_example_folder

More ErleBrain documentation [here](http://docs.erlerobotics.com/brains/erle-brain-3/sofware/apm) and [here](http://ardupilot.org/dev/docs/building-for-erle-brain-2.html).