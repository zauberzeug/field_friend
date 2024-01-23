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

```bash
git clone git@github.com:zauberzeug/field_friend.git
cd field_friend
python3 -m pip install -r requirements.txt
./main.py
```

This will start the simulated robot and open the user interface in your browser.

NOTE: The software is intended to run on Linux and Unix systems. If you are using windows please consider running in a Docker container or virtual machine.

## On Real Hardware

### Development

The following instructions will only work if you have a real "Zauberzeug Field Friend" at your disposal.
Contact [sales@zauberzeug.com](mailto:sales@zauberzeug.com) if you are interested in a non-profit purchase of this development hardware.

1. make sure you can login via ssh without providing a password (via `ssh-copy-id` command)
2. go to your local `field_friend` folder and start the LiveSync script: <br>
   `./sync.py <ssh-host-name-of-field-friend>`
3. this will deploy your local code to the Field Friend
4. as long as [LiveSync](https://github.com/zauberzeug/livesync) is active, all code change are automatically pushed to the machine
5. the new code will automatically trigger a reload on the Field Friend

### Update RoSys and NiceGUI

To utilize personal versions of RoSys and NiceGUI instead of the default ones provided in the docker image,
modify the `sync.py` file by uncommenting the specific folders.

### Debugging

You can see the current log with

```bash
./docker.sh l rosys
```
