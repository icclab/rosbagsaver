# rosbagsaver

After building and sourcing the package you can run it with:

```
ros2 run rosbagsaver my_rosbagsaver
```

Then to start/stop the rosbagsaver publish a topic as follows:

```
ros2 topic pub /process_trigger std_msgs/msg/String "data: start/stop" --once
```
