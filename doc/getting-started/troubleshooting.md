# Troubleshooting

A collection of quick fixes for the most common hiccups when running **curobo_ros**.

---

## Docker Issues

### 1. Not Enough Disk Space

**Symptom**
Docker build fails with:
```
no space left on device
ERROR: failed to solve: write /var/lib/docker/...
```

**Cause**
The Docker image requires approximately **30 GB** of disk space for the complete build (base images, ROS 2, CUDA, PyTorch, cuRobo, and dependencies).

**Check Available Space**
```bash
# Check disk usage
df -h

# Check Docker's disk usage
docker system df
```

**Solutions**

**Option 1: Free up Docker space**
```bash
# Remove unused containers, images, and volumes
docker system prune -a --volumes

# This can free up several GB!
```

**Option 2: Change Docker's data location** (if your main disk is small)
```bash
# Stop Docker
sudo systemctl stop docker

# Edit Docker daemon config
sudo nano /etc/docker/daemon.json

# Add data-root to a larger drive:
{
  "data-root": "/path/to/large/drive/docker"
}

# Move existing data
sudo rsync -aP /var/lib/docker/ /path/to/large/drive/docker/

# Restart Docker
sudo systemctl start docker
```

**Option 3: Clean system space**
```bash
# Remove old kernels (Ubuntu)
sudo apt autoremove

# Clean apt cache
sudo apt clean

# Remove snap cache (if using snap)
sudo rm -rf /var/lib/snapd/cache/*
```

---

### 2. Container Name Already Exists

**Symptom**
```
docker: Error response from daemon: Conflict. The container name "/x86docker" is already in use
```

**Cause**
You're trying to create a container, but one with that name already exists (possibly stopped).

**Solution**

**Option 1: Start the existing container** (recommended)
```bash
# Check if container exists
docker ps -a | grep x86docker

# Start it
docker start x86docker

# Attach to it
docker exec -it x86docker bash
```

**Option 2: Remove and recreate** (you'll lose container-specific changes)
```bash
# Stop the container
docker stop x86docker

# Remove it
docker rm x86docker

# Now run start_docker_x86.sh again
bash start_docker_x86.sh
```

See also: [Docker Workflow Guide](concepts/docker_workflow.md#troubleshooting-docker-issues)

---

### 3. Docker Daemon Not Running

**Symptom**
```
Cannot connect to the Docker daemon at unix:///var/run/docker.sock
```

**Cause**
Docker service is not running.

**Solution**
```bash
# Start Docker
sudo systemctl start docker

# Check status
sudo systemctl status docker

# Enable Docker to start on boot
sudo systemctl enable docker

# Add your user to docker group (to avoid sudo)
sudo usermod -aG docker $USER
# Log out and back in for this to take effect
```

---

### 4. Permission Denied

**Symptom**
```
permission denied while trying to connect to the Docker daemon socket
```

**Cause**
Your user is not in the `docker` group.

**Solution**
```bash
# Add user to docker group
sudo usermod -aG docker $USER

# Apply changes (log out and back in, OR)
newgrp docker

# Verify
docker ps
```

---

### 5. NVIDIA GPU Not Accessible in Container

**Symptom**
Inside the container, `nvidia-smi` fails or shows no GPUs.

**Cause**
NVIDIA Container Toolkit not installed or not configured correctly.

**Check**
```bash
# On host
nvidia-smi  # Should work

# Test GPU access in Docker
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
# Should show your GPU
```

**Solution**

Install NVIDIA Container Toolkit:
```bash
# Add repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker
sudo systemctl restart docker
```

---

### 6. Docker Build is Extremely Slow

**Symptom**
`build_docker.sh` takes hours instead of 20-30 minutes.

**Possible Causes & Solutions**

**1. Slow internet connection**
- The build downloads several GB of data
- Use wired connection if possible
- Consider building during off-peak hours

**2. Limited CPU resources**
```bash
# Check CPU usage during build
htop
```
- Close other applications
- Ensure Docker has enough CPU allocated (Docker Desktop settings)

**3. Limited RAM**
- Recommend at least 16 GB RAM
- Close memory-heavy applications
- Adjust Docker memory limit (Docker Desktop → Settings → Resources)

**4. Disk I/O bottleneck**
- Use SSD instead of HDD if possible
- Check disk usage: `iotop`

---

## ROS / Launch Issues

### 7. OpenCV - `AttributeError: module 'cv2.dnn' has no attribute 'DictValue'`

**Symptom**  
You launch the Realsense D405 (or any vision node) and get:

```
AttributeError: module 'cv2.dnn' has no attribute 'DictValue'
```

**Cause**  
Some OpenCV Python wheels ship an outdated `typing` stub that still references
the deprecated `DictValue` symbol.

**Fix**

1. Open the offending file **inside the container**:

```bash
nano /usr/local/lib/python3.10/dist-packages/cv2/typing/__init__.py
```

2. Comment out **line 171** (the import of `DictValue`) or remove the entire
   line.

3. Save, exit, and re-run your command.

---

### 8. Missing symbol `ucm_set_global_opts`

**Symptom**
ROS nodes fail at startup with an error similar to:

```
undefined symbol: ucm_set_global_opts
```

**Cause**
A broken or partially removed `mpich`/`hwloc` dependency inside the image.

**Fix**

Run the following **inside the Docker container** to reinstall the packages:

```bash
sudo apt-get update && sudo apt-get install --reinstall -y \
  libmpich-dev hwloc-nox libmpich12 mpich
```

---

## Windows-Specific Issues

### 9. Docker Containers Will Not Start

**Symptom**
`docker run` fails immediately, or the Docker Desktop dashboard shows
**“Insufficient virtual memory”**.

**Fix**

1. Reboot into your BIOS/UEFI setup.
2. Enable the CPU feature usually labelled **Intel VT-x / AMD-V /
   Virtualization**.
3. Save & reboot. Docker Desktop should now start containers normally.

*(Names and menus differ between motherboards; consult your vendor manual.)*

---

### 10. RViz / GUI Windows Do Not Appear (Windows)

Linux GUI apps inside the container need an X-server running on the host.

**Solution (recommended for Windows)**

1. Install **VcXsrv / XLaunch** from
   [https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/).

2. Launch **XLaunch** and keep the default settings **except**:
   *Tick “Disable access control”* on the last page.

3. In the PowerShell or WSL terminal where you will run `start_docker_x86.sh`,
   set the display variable, replacing `<HOST_IP>` with the address shown by
   `ipconfig`:

   ```bash
   export DISPLAY=<HOST_IP>:0
   ```

4. Start the container and run RViz as usual; the window should pop up on your
   desktop.

---
