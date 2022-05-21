# Docker for Tendon Experiments

This docker container is a minimal development environment.  It is intended to
be used as your user, with your same home directory, and using the same X11
server so that it feels like it's fully your computer.

* `Dockerfile`: A script for generating the docker image.  You can refer to
  this to understand the installation requirements.  This file takes a few
  arguments which are provided by `build.sh`.
  * `user`: your username
  * `user_id`: the id of your user
  * `user_group_id`: the id of your user's main group
  * `user_shell`: the shell your user uses (e.g., bash)
  * `user_home`: your home directory
* `build.sh`: Build the Docker image and name it `tendon-dev`, giving values
  for the arguments expected by Dockerfile.
* `run.sh`: Create an run a container based on the
  `tendon-dev` image and also call the container `tendon-dev`
  * Disables many docker security features since this container is not intended
    to be exposed over the internet.
  * Connect to the container with SSH over port 3322 to localhost (you can edit
    this script to change that port).  Instructions are printed from the
    script.
  * Mounts your home directory and X11 to be shared.  Access X11 by setting the
    `DISPLAY` environment variable to the host's value after connecting.


## Building

Within the Dockerfile, there are some sections commented out that are optional.
Feel free to browse the Dockerfile and uncomment some of the extra wanted
pieces.  Otherwise, you can always install more software in the container after
building the image.


## Running

There is a script called `run.sh`.  You are not required to run the image this
way.  You can look inside and use it as an example in order to have access to
your home directory and be able to launch X11 windows from inside the
container.

The `run.sh` script creates the docker container and runs it in the background,
then prints example commands for connecting to it via SSH.  Note, for this to
work, you will need to make sure your public ssh key is included in your list
of authorized keys as passwords are disabled.  For example, that can be done
with

```bash
cat ~/.ssh/id_rsa.pub >> ~/.ssh/authorized_keys
```

The container does not have its own X11 server running.  Instead, it uses the
host X11 server.  However, you will need to manually set the DISPLAY
environment variable AFTER connecting via SSH locally on port 3322.

Also, every teriminal launched within the container will automatically source
the `/opt/ros-linux/setup.bash` setup file for convenience.


## Bash Logic

It may be helpful to distinguish host terminal shells from those terminal
shells launched in the Docker container, there is a file created in the image
called `/etc/docker-name`.

Here is an example of a prompt that can be specified in `~/.bashrc`:

```bash
# ANSI color codes
RS="\[\033[0m\]"    # reset
HC="\[\033[1m\]"    # hicolor
FGRN="\[\033[32m\]" # foreground green
FYEL="\[\033[33m\]" # foreground yellow

# Set the prompt variable
PS1="${HC}${FGRN} \! \W \$${RS} "
if [ -f "/etc/docker-name" ]; then
  PS1="${HC}${FYEL} ($(cat /etc/docker-name))${RS}${PS1}"
fi
```

When inside of a docker container, it will put the name of the docker image
at the beginning of the prompt in yellow if there is a file caled
`/etc/docker-name`.
