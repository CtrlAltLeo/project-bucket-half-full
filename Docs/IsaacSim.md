# IsaacSim
Isaac Sim is a professional grade software for physics simulations on 3D Models. During our time with Bobcat, we had access to 3D models of
loaders and the Bobcat Arena at the Accelleration Center. We used Isaac Sim primarily to generate training data to give to our Vision team. 

Isaac Sim **IS**:
- a professional physics simulator
- accessed through an Web RTC Client
- spun up in a Docker Container

Isaac Sim **IS NOT**:
- something that should be installed locally unless you have an extremely beefy PC

## Setup

These steps assume the following:
1. You have VPN Access from Bobcat to a development server
2. You are able to SSH into their server
3. You are planning on using Isaac Sim 5.1.0. (Your Bobcat Team will know this if you don't)

You must follow those steps in order to continue.

These steps come from the [Official NVidia Docs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html#isaac-sim-app-install-container)
but are presented here for clarity.


### Initial Setup

You can assume Docker is installed. Verify using `docker ps`.

> Note: If you can't use `docker` without `sudo`, you can run
> `sudo usermod -aG docker $USER` to add your user to the usegroup
> and not need to run sudo each time. You might need an admin to 
> help you with this.


Depending on the setup, you might need to install the nvidia container toolkit (Step 3 in the docs).

### Container Setup

First, pull the container with `docker pull nvcr.io/nvidia/isaac-sim:5.1.0`.

Next, create the caches in your user home folder. You might need an admin to help you with this as well.

```bash
mkdir -p ~/docker/isaac-sim/cache/main/ov
mkdir -p ~/docker/isaac-sim/cache/main/warp
mkdir -p ~/docker/isaac-sim/cache/computecache
mkdir -p ~/docker/isaac-sim/config
mkdir -p ~/docker/isaac-sim/data/documents
mkdir -p ~/docker/isaac-sim/data/Kit
mkdir -p ~/docker/isaac-sim/logs
mkdir -p ~/docker/isaac-sim/pkg
sudo chown -R 1234:1234 ~/docker/isaac-sim
```

Finally, start the container using this docker command: 

```bash
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -u 1234:1234 \
    nvcr.io/nvidia/isaac-sim:5.1.0
```

### Starting Isaac Sim

You will be in a new shell environment, isaacsim@ros4090. Inside this directory, you should see `./runheadless.sh`. This is the only
shell script you'll need to run. It will spew a lot of logs, but the final message you're looking for is
`Application Started Successfully, Ready to Stream`.


### Setting up the Container for Continious Operation

This is good, but you do not want to have to run this every time you want to work on Isaac Sim. Instead of running the container with 
`docker run`, use this command:

```bash
docker create --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
    -u 1234:1234 \
    nvcr.io/nvidia/isaac-sim:5.1.0
```

```bash
docker start isaac-sim
```

Now, inside the Isaac Sim container:
```bash
./runheadless &
```

You can safely exit the Isaac Sim shell, and disconnect from the server and it will still run. To stop the container, simply use
`docker stop isaac-sim`.

## Usage
Assuming that you have access to the VPN and are continously running the container, the following steps will be use to download and run the WebRTC Isaac Sim Streaming Client.

### Installing the Streaming Client

First, navigate to [Download Isaac Sim, Version 5.1.0](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

Next, scroll down until you see the `Isaac Sim WebRTC Streaming Client`. It should be version 1.1.5.

> The next steps assume that you are downloading the Windows link. This is the version our team used for the project.

Next, download the Windows version. You should now see an executable in your downloaded files called `isaacsim-webrtc-streaming-client-1.1.5-windows-x64.exe`.

Run the executable. After it is finished installing, you should be brought to the following window:

<img width="1915" height="1014" alt="Screenshot 2026-03-30 132025" src="https://github.com/user-attachments/assets/866fe2b5-8244-4fa1-bebb-a665d941019a" />

### Order of Operation

To properly run the WebRTC Streaming Client, do the following in the specified order:

1. Connect to Bobcat's GlobalProtect VPN.
2. Open the Streaming Client.
3. In the "Server" text box, insert the appropriate IP address that Bobcat provides you.
4. Press "Connect".

If the VPN is operating properly, you should see the following viewport after a few moments of loading:

![isaac-sim-viewport](https://github.com/user-attachments/assets/c37554fd-fda0-4b63-9649-5bc1bdb8fdfd)

You can then use Isaac Sim as long as the continuous server is running and you are connect to the VPN.
> Note: Since this is a remote streaming client, it is impossible to save your progress when you close the client. Even if you try to "save" a file inside the client, you will not be able to find the file again, and your progress will be lost. There is a possibility of using the "docker cp" command to copy and extract local files to and from the container, but our team did not discover how to accomplish this in our time with the client.
> Going forward, the implementation of the "docker cp" command should be investigated in order to prevent the loss of progress.
