# ros2web

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

The ros2web is software that forwards HTTP requests to ROS2 packages.
It facilitates the integration of web applications with 
ROS2 by allowing HTTP requests to be processed by ROS2 packages.


```text
+-------------+            +-----------+               +--------------+
|             | <--------- |           | <------------ |              |
| Web Browser |    HTTP    |  ros2web  |  ROS2 Service | ROS2 package |
|             | ---------> |           | ------------> |              |
+-------------+            +-----------+               +--------------+
```

There is also [ros2web_app](https:://github.com/ros2web/ros2web_app) for the creation of web applications with ros2web.

### Packages

- ros2web: ros2cli extension to start HTTP server.
- ros2web_interface: Message and service data structures for ros2web.


### Related Packages

Related packages for creating web applications to manipulate ROS2 packages using ros2web.

- [ros2web_app](https:://github.com/ros2web/ros2web_app): A web application framework designed for the easy creation of
  web applications that manipulate ROS2 packages.
- [launch_api](https:://github.com/ros2web/ros_run): Library for managing ROS2 package launches in python.


## Installation

The installation procedure is the same as for the general ROS2 package.

```bash
# Install dependencies
python3 -m pip install -r requirements.txt 

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https:://github.com/ros2web/ros2web.git
cd ~/ros2_ws
colcon build
. ./install/local_setup.bash
```

## Usage

### 1. Create ROS2 Service.
Create a service in the ROS2 package that accepts `HTTP` of the ROS2 service provided by ros2web.
The service name should be in the format `/http/<method>/<path>`so that ros2web can accept HTTP requests.
For example, if you create a service named `/http/get/add_two_ints`, then when you access `/add_two_ints` with the `get`
method, the service of the package will be called.

The following is an example of a service that accepts a
GET request and returns the sum of the two numbers specified in the query.

```python
import urllib.parse
import json
import rclpy
from rclpy.node import Node
from ros2web_interface.srv import HTTP

class WebHandler(Node):
    def __init__(self):
        super().__init__('example')
        self.get_srv = self.create_service(HTTP, 'http/get/add_two_ints', self.callback)

    def callback(self, request: HTTP.Request, response: HTTP.Response):
        query = dict(urllib.parse.parse_qsl(request.query))
        
        a = int(query.get('a', 0))
        b = int(query.get('b', 0))
        self.get_logger().info('Incoming request\na: %d b: %d' % (a, b))
        
        sum_num = a + b
        response.text = json.dumps({'sum': sum_num})

        return response
    
def main(args=None):
    rclpy.init(args=args)
    web_handler = WebHandler()
    try:
        rclpy.spin(web_handler)
    except KeyboardInterrupt:
        pass
```

### 2. Start ros2web server and ros2 package.
Start ros2web server with `ros2 web server` command.
ros2web is implemented as an extension of ros2cli. It can be used in the same way as the `ros2` command

Then run the ROS2 package you created.
The name of the created ROS package is `ros2web_example_py` and the executable is `api`.

```bash
ros2 web server --no-auth
ros2 run ros2web_example_py api
```

### 3. Access the address of the ros2web server.

Accesses the path specified in the service name of the ROS2 package.

http://localhost:8080/add_two_ints?a=1&b=2

Accessing the above will return the following values.
```json
{"sum": 3}
```

A complete example is available at [ros2web_example_py]().

## License

Distributed under the Apache 2.0 License. See `LICENSE` for more information.

## Acknowledgements

