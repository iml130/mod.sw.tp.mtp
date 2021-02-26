# Setup for ROS Noetic

This error can appaer in Ubuntu 20.04 with ROS noetic:

```
  /usr/bin/env: ‘python’: No such file or directory
```

To fix this problem add a Symlink (Softlink) for python 3:
```
  sudo ln -s /usr/bin/python3 /usr/bin/python
```

This adds a Symlink in /usr/bin to python 3. With 

```
  ll /usr/bin | grep python
```

this line should appear:

```
  lrwxrwxrwx 1 root root       16 Jun 19 13:27  python -> /usr/bin/python3
```
