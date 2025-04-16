# macOS Instructions:
We will use a virtualization software called [Docker](https://www.docker.com/) to run Ubuntu 22.04 on MacOS.

## Docker setup
First, install docker:
If you have brew, you can run `brew install docker`. Otherwise, follow the [installation instructions here](https://docs.docker.com/desktop/setup/install/mac-install/#:~:text=GB%20of%20RAM.-,Install%20and%20run%20Docker%20Desktop%20on%20Mac,-Tip).

Next, pull the Ubuntu 22.04 image: `docker image pull ubuntu:22.04`

Run `docker image ls` to verify that it worked. Output should resemble this:
```
REPOSITORY                TAG       IMAGE ID       CREATED        SIZE
ubuntu                    22.04     560582227a09   2 months ago   69.2MB
```

Now, create container from the image: `docker run -it --name cav_container -p 2222:22 -p 8765:8765 -d [image_id]`. So, in the example above, you would run 
```
docker run -it --name cav_container -p 2222:22 -p 8765:8765 -d 560582227a09
```

Now, running `docker container ls -a` should show the newly created container:

```
CONTAINER ID   IMAGE          COMMAND       CREATED          STATUS          PORTS     NAMES
dac3ebc42682   560582227a09   "/bin/bash"   10 minutes ago   Up 10 minutes             cav_container
```
As the last step in setup, let's set up a user so we don't run everything as root. Run the following commands with the following output, using your desired username and password.
```
(dev) akashpamal@Akashs-MacBook-Air Sem6 % docker exec -it cav_container /bin/bash
root@1e66c14f256e:/# useradd -m akash
root@1e66c14f256e:/# passwd akash
New password:
Retype new password:
passwd: password updated successfully
root@1e66c14f256e:/#
root@1e66c14f256e:/# apt-get update
[Output ommitted]
root@1e66c14f256e:/# apt-get install sudo
[Output ommitted]
root@1e66c14f256e:/# sudo adduser akash sudo
Adding user `akash' to group `sudo' ...
Adding user akash to group sudo
Done.
root@9be6f749e534:/# exit
```

Now, run the following commands to enter the container as the user you just created and set up SSH. [SSH](https://en.wikipedia.org/wiki/Secure_Shell) is a way that you can access the Docker container using its IP address. These commands enable you to ssh into your container (which you will need later for some visualization steps).

```
docker exec -it --user [your_username] cav_container /bin/bash
sudo apt update
sudo apt install -y openssh-server
sudo apt install net-tools
sudo service ssh start
```
Now, our setup is complete!

## Using the container
Each time you want to use the container run: `docker exec -it --user [your_username] cav_container /bin/bash`. Exit the docker container by running `exit`.

Notice that sometimes, when you try to exec into the container, you might get an error message like: `Error response from daemon: container 6261ae1954290684bd80a728a092972408695c5062960583aaa8785c01708dbd is not running`. If that happens, just run `docker container start cav_container` and you should be able to exec into it again.


Recap:
We now have a container that we can use for all of our development. MAKE SURE NOT TO DELETE THE CONTAINER UNLESS YOU HAVE BACKED IT UP TO AN IMAGE OTHERWISE YOU WILL LOSE ALL YOUR WORK. Stopping the container is ok. You can save your container to an image by running a command like `docker commit --pause cav_container cav_save/cav_save:snapshot1`. See [the docs for more details](https://docs.docker.com/reference/cli/docker/container/commit/).


Now, follow the instructions in the docs for [installing ROS](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and [colcon](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) and continue with the following tasks. Reach out on the Slack if you have any questions. Make sure to do these instructions inside the container (by running `docker exec`).

## Writing Code

Ensure that ssh is still running and find the IP address.
```
sudo service ssh restart
ifconfig
```

The `ifconfig` command outputs some networking information about your container. Look for the word "inet" and look for the IP next to it. If you see multiple entries, use one that isn't `127.0.0.1`. For example, the IP address for my Docker container is `172.17.0.2`. Use this IP address for later steps.

In later steps you will need to write code. Use [VSCode's SSH](https://code.visualstudio.com/docs/remote/ssh) extension to access it. Install the VSCode extension and use "[username]@<ip_address>:2222" as the host to connect to. Use the IP address you found from the `ifconfig` command. From here, you can open a folder, create and edit files, and even use the integrated terminal to access your docker container. (Or just use Vim if you're built different). 

## Visualization with Foxglove
In later steps, you will see "RViz" or "Foxglove". Use Foxglove for visualization (RViz does not work nicely on Mac).

Click "Open connection..." on Foxglove and use "ws://<ip_address>:8765" as the websocket URL. Use the IP address that you found earlier by running `ifconfig`. For more detailed instructions on using Foxglove, see the main README file. This will show you how to set up the Foxglove bridge to use Foxglove.

## Getting Files into Docker
After you install git into your Docker container, you can clone the repository directly into your docker container.

You can move files from your local computer to the docker container using the [scp] (https://stackoverflow.com/questions/19945881/copy-file-folder-using-scp-command) utility. For example, if you downloaded the `cavalier_take_home.mcap` file to `~/Downloads/` on your local computer and want to move it to your Docker container, you can run the following command from your computer:
```
scp -P 2222 ~/Downloads/cavalier_take_home.mcap akash@<ip_address>:~/
```