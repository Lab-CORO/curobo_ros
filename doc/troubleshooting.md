# Troubleshooting

A collection of quick fixes for the most common hiccups when running **curobo_ros**.

---

## 1. OpenCV - `AttributeError: module 'cv2.dnn' has no attribute 'DictValue'`

**Symptom**  
You launch the Realsense D405 (or any vision node) and get:

```

AttributeError: module 'cv2.dnn' has no attribute 'DictValue'

````

**Cause**  
Some OpenCV Python wheels ship an outdated `typing` stub that still references
the deprecated `DictValue` symbol.

**Fix**

1. Open the offending file **inside the container**:

```bash
   nano /usr/local/lib/python3.10/dist-packages/cv2/typing/__init__.py
````

2. Comment out **line 171** (the import of `DictValue`) or remove the entire
   line.

3. Save, exit, and re-run your command.

---

## 2. Missing symbol `ucm_set_global_opts`

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

## 3. Windows-specific issues

### 3.1 Docker containers will not start

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

### 3.2 RViz / GUI windows do not appear

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
