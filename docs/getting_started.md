# Getting Started

Test

## Run in Simulation

We suggest you begin with simulating the Field Friend on your local development machine.
The software is meant to run on Linux and Unix systems so if you are using Windows, consider running in a Docker container or virtual machine.

Just execute the following commands:

```bash
git clone git@github.com:zauberzeug/field_friend.git
cd field_friend
python3 -m pip install -r requirements.txt
./main.py
```

This will open the user interface of a simulated robot in your browser.
If you change some code, the simulation will automatically reload.
The Field Friend code is based on [RoSys](https://rosys.io) which itself uses [NiceGUI](https://nicegui.io),
both having a very gentle learning curve and are designed to boost your rapid development and testing.

## Run on Real Hardware

The following instructions will only work if you have a real Zauberzeug Field Friend at your disposal.
Contact [sales@zauberzeug.com](mailto:sales@zauberzeug.com) if you are interested in purchasing this robot.

### Setup

1. ensure you can login via ssh without providing a password (via `ssh-copy-id` command)
2. ensure you have [LiveSync](https://github.com/zauberzeug/livesync) installed with <br> `python3 -m pip install livesync`
3. ensure the latest version of the docker image is installed on the Field Friend by syncing the code as described below and then running <br> `./docker.sh uppull`
4. Optional: ensure the correct docker containers are loaded on startup by running <br> `./docker.sh stopall && ./docker.sh uppull && ./docker.sh install`
5. Optional: update the [Lizard](https://lizard.dev) microcontroller firmware on your Robot Brain by accessing the Field Friend web interface and navigating to the "Developer" options

### Deploy and Change Code

1. go to your local `field_friend` folder and start the [LiveSync](https://github.com/zauberzeug/livesync) script: <br>
   `./sync.py <ssh-host-name-of-field-friend>`
2. this will deploy your local code to the Field Friend
3. as long as [LiveSync](https://github.com/zauberzeug/livesync) is active, all code change are automatically pushed to the machine
4. any code changes will automatically trigger a reload on the Field Friend

### Update RoSys and NiceGUI

To utilize personal versions of RoSys and NiceGUI instead of the default ones provided in the docker image,
modify the `sync.py` file by uncommenting the specific folders.

### Logs

You can see the current log with

```bash
./docker.sh l rosys
```

The history of logs can be seen with

```bash
less -r ~/.rosys/debug.log
```
