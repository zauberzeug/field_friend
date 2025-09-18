# Getting Started

## Run in Simulation

We suggest you begin with simulating the Field Friend on your local development machine.
The software is meant to run on Linux and Unix systems so if you are using Windows, consider running in a Docker container or virtual machine.

The Field Friend requires either Linux or Mac system with Python {{ python_version }} to run. <br>Windows is currently not supported, but may work by using Docker or a virtual machine.

1. Clone the repository <br> `git clone git@github.com:zauberzeug/field_friend.git && cd field_friend`

2. Create a file with your environment variables and replace `U4` with the name of your robot <br> `echo "ROBOT_ID=U4" > .env`

Now you have two options, either you run the code in Docker, like it does on the robot, or you setup your local python environment to run it.
Should you face any problems during the setup, please check the [troubleshooting page](troubleshooting.md) or submit a [GitHub issue](https://github.com/zauberzeug/field_friend/issues).

### Docker Setup (recommended)

1. Install [Docker](https://docs.docker.com/get-started/get-docker/)

2. Build and start the container. <br> `./docker.sh U` <br> Our `docker.sh` script will automatically use the correct settings for your system.

### Local Setup

1. Optional: Setup a virtual environment with [venv](https://docs.python.org/3/library/venv.html) <br> `python3 -m venv .venv && source .venv/bin/activate`

2. Install the python requirements <br> `python3 -m pip install -r requirements-dev.txt`

3. Now you can run the program <br> `python3 main.py`

In both cases, this will open the user interface of a simulated robot in your browser (if not browse to [http://localhost/](http://localhost/)).
The simulation will automatically hot reload upon code changes.
The Field Friend code is based on [RoSys](https://rosys.io) which itself uses [NiceGUI](https://nicegui.io),
both having a very gentle learning curve and are designed to boost your rapid development and testing.

## Run on Real Hardware

The following instructions will only work if you have a real Zauberzeug Field Friend at your disposal.
Contact [sales@zauberzeug.com](mailto:sales@zauberzeug.com) if you are interested in purchasing this robot.

### Setup

1. Ensure you can login via ssh without providing a password (via `ssh-copy-id` command)
2. Ensure you have [LiveSync](https://github.com/zauberzeug/livesync) installed with <br> `python3 -m pip install livesync`
3. Ensure the latest version of the docker image is installed on the Field Friend by syncing the code as described below and then running <br> `./docker.sh U`
4. Optional: ensure the correct docker containers are loaded on startup by running <br> `./docker.sh stopall && ./docker.sh U && ./docker.sh install`
   <!-- TODO -->
   <!-- 5. Optional: update the [Lizard](https://lizard.dev) microcontroller firmware on your Robot Brain by accessing the Field Friend web interface and navigating to the "Developer" options -->

### Deploy and Change Code

1. Go to your local `field_friend` folder and start the [LiveSync](https://github.com/zauberzeug/livesync) script: <br>
   `./sync.py <ssh-host-name-of-field-friend>`
2. This will deploy your local code to the Field Friend
3. As long as [LiveSync](https://github.com/zauberzeug/livesync) is active, all code change are automatically pushed to the machine
4. Any code changes will automatically trigger a hot reload on the Field Friend.

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
