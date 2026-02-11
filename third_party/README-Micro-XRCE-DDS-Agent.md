# Micro-XRCE-DDS-Agent (submodule)

This submodule is pinned to **v2.4.3** (tag). Used for PX4–ROS 2 / XRCE-DDS communication (e.g. Holybro Pixhawk Jetson Baseboard).

## Build and install

After cloning/updating the repo (e.g. `git submodule update --init third_party/Micro-XRCE-DDS-Agent`):

```bash
cd third_party/Micro-XRCE-DDS-Agent
mkdir -p build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

The agent binary is installed to `/usr/local/bin/MicroXRCEAgent`.
