# rosbagsaver
This node listens to a given Zenoh topic and starts/stops a rosbag save process in Python. It assumes you have container with the Zenoh package and the ros2-dds Zenoh plugin.

After building and sourcing the package you can run it with:

```
ros2 run rosbagsaver my_rosbagsaver <topic> <bag_name> <zenoh_router_url> <prefix>

```
Where <topic> is the topic it will listen to for triggering messages (start/stop), <bag_name> is the name of the bag to store, <zenoh_router_url> is the zenoh router url in the format tcp/ip_address:port,  <prefix> is the prefix of topics to store in the bag (e.g., /tb2 namespace).


