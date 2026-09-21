# Installing tmux on the robot (offline, arm64/Ubuntu 22.04 "jammy")

The robot has no wifi/internet access, so `apt install tmux` won't work directly on
it. Instead, download the `.deb` packages on your PC (which does have internet)
using Docker to emulate the robot's arm64 architecture, then copy them over and
install with `dpkg`.

Run everything below from PowerShell on your PC. Replace `192.168.149.1` with the
robot's IP if different.

## 1. Register QEMU emulation with Docker (one-time per PC)

```powershell
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

## 2. Download tmux + dependencies for arm64/jammy

```powershell
docker run --rm --platform linux/arm64 -v "${PWD}/debs:/debs" ubuntu:22.04 bash -c "apt-get update && apt-get install -y --download-only -o Dir::Cache::Archives=/debs tmux"
```

This drops `tmux`, `libevent-core-2.1-7`, and `libutempter0` `.deb` files into a
local `debs/` folder.

## 3. Copy the packages to the robot

```powershell
ssh ubuntu@192.168.149.1 "mkdir -p ~/tmux_debs"
scp .\debs\*.deb ubuntu@192.168.149.1:~/tmux_debs/
```

## 4. Install on the robot

```powershell
ssh ubuntu@192.168.149.1 "sudo dpkg -i ~/tmux_debs/*.deb && tmux -V"
```

If `tmux -V` prints a version, the install succeeded.

## Notes

- Step 1 only needs to be run once per PC (registers arm64 emulation with Docker
  Desktop). Skip it on subsequent installs unless Docker Desktop was reinstalled.
- If a robot uses a different architecture or Ubuntu release, check first with:
  ```powershell
  ssh ubuntu@<robot-ip> "dpkg --print-architecture && lsb_release -a"
  ```
  and adjust `--platform` (e.g. `linux/arm/v7` for armhf) and the base image tag
  (e.g. `ubuntu:20.04`) in step 2 accordingly.
