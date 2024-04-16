<img src="https://github.com/zauberzeug/field_friend/raw/main/assets/field_friend.webp"  alt="Field Friend rendering" width="40%" align="right" />

# Zauberzeug Field Friend

This is the full source code of the [Field Friend](http://feldfreund.de) (aka Feldfreund) weeding robot.
The software is based on [RoSys](https://rosys.io) and fully Open Source.
The hardware is build by [Zauberzeug](http://zauberzeug.com) and intended as a platform to advance organic and regenerative agriculture.

## Features

- full control via web interface
- manual steering with touch-joystick and keyboard
- camera/motor calibration for real world coordinate system (unit: meters)
- ...

## Getting Started

Run these commands on your local machine to start the Field Friend simulation:

```bash
git clone git@github.com:zauberzeug/field_friend.git
cd field_friend
python3 -m pip install -r requirements.txt
./main.py
```

This will open the user interface with a simulated robot in your browser.

NOTE: The software is intended to run on Linux and Unix systems.
If you are using windows please consider running in a Docker container or virtual machine.

## On Real Hardware

### Development

The following instructions will only work if you have a real "Zauberzeug Field Friend" at your disposal.
Contact [sales@zauberzeug.com](mailto:sales@zauberzeug.com) if you are interested in a non-profit purchase of this development hardware.

#### Setup

1. ensure you can login via ssh without providing a password (via `ssh-copy-id` command)
2. ensure you have [LiveSync](https://github.com/zauberzeug/livesync) installed with <br> `python3 -m pip install livesync`
3. ensure the latest version of the docker image is installed on the Field Friend by syncing the code as described below and then running <br> `./docker.sh uppull`
4. Optional: ensure the correct docker containers are loaded on startup by running <br> `./docker.sh stopall && ./docker.sh uppull && ./docker.sh install`
5. Optional: update the [Lizard](https://lizard.dev) microcontroller firmware on your Robot Brain by accessing the Field Friend web interface and navigating to the "Developer" options

#### Deploy and Change Code

1. go to your local `field_friend` folder and start the [LiveSync](https://github.com/zauberzeug/livesync) script: <br>
   `./sync.py <ssh-host-name-of-field-friend>`
2. this will deploy your local code to the Field Friend
3. as long as [LiveSync](https://github.com/zauberzeug/livesync) is active, all code change are automatically pushed to the machine
4. any code changes will automatically trigger a reload on the Field Friend

### Update RoSys and NiceGUI

To utilize personal versions of RoSys and NiceGUI instead of the default ones provided in the docker image,
modify the `sync.py` file by uncommenting the specific folders.

### Debugging

You can see the current log with

```bash
./docker.sh l rosys
```
