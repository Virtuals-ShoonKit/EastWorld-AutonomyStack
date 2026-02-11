# systemd services

## Micro XRCE-DDS Agent

Runs the Micro XRCE-DDS agent on startup for PX4–ROS 2 / XRCE-DDS over serial (e.g. Holybro Pixhawk Jetson Baseboard on `/dev/ttyTHS1` at 921600 baud).

### Prerequisites

- Micro-XRCE-DDS-Agent built and installed (binary at `/usr/local/bin/MicroXRCEAgent`). From repo root:

  ```bash
  cd third_party/Micro-XRCE-DDS-Agent
  mkdir -p build && cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
  ```

### Install and enable on boot

From the **EastWorld-AutonomyStack** repo root:

```bash
sudo cp systemd/microxrce-dds-agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable microxrce-dds-agent.service
sudo systemctl start microxrce-dds-agent.service
```

### Useful commands

- Status: `sudo systemctl status microxrce-dds-agent`
- Stop: `sudo systemctl stop microxrce-dds-agent`
- Disable on boot: `sudo systemctl disable microxrce-dds-agent`
- View logs: `journalctl -u microxrce-dds-agent -f`
