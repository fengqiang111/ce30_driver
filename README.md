This page is set up for CE30-D SDK technical support. For product information about CE30 series and other Benewake products, please visit our [offical website](http://www.benewake.com/en/index.html).

---

#### Get Started

CE30-D adopts the User Datagram Protocol (UDP) to interchange data messages with host machines. We highly recommend users coding their own program for interacting with CE30-D. In such a case, you can proceed to our [offical website](http://www.benewake.com/en/index.html) for the documentation of our UDP communication stipulations. However, if you wish to quickly deploy CE30-D on your application platforms and test its functionalities, then this repository will help you. Here, we provide a C++ library that helps you to retrieve CE30-D measurement data at ease. The library is able to be compiled and run on both Windows and Linux platforms with few dependencies (mainly the Boost Library and the C++11 Standard Library). However, we develop and test this library on 64-bit Windows 10 and Ubuntu 16.04 x86_64 with Intel Core i7 and 16G memory for most of the time. Thus, we expect your computer to match this configuration as much as possible. Also, we have developed a set of visualization tools to help you obtain the perceptibility towards the measurement data. These visualization tools are a part of CE30-D SDK, see [ce30_viz](https://codincodee.github.io/ce30_viz) for details. Furthermore, we provided [ce30_driver_ros](https://codincodee.github.io/ce30_driver_ros/) for those who work with [ROS](http://www.ros.org/).

If this is your first time visiting the site, we recommend you going through this [tutorial](tutorials/write_a_simple_server.md). In the tutorial, we demonstrate how to obtain point cloud data in a super easy way. Also, we step further to show how to manipulate data packet sending, receiving and parsing, which is for users with advanced purposes. If you have already done that and have been seeking for API documentations, click [here](api_doc/html/index.html).



As `ce30_driver` is an open-source project, we are calling for contributions. Please fork the project if you want to be a part of it. We appreciate any effort you make to perfect this project.

#### Quick Access

- [Official Website](http://www.benewake.com/en/index.html)
- [Tutorial](tutorials/write_a_simple_server.md)
- [API Documentation](api_doc/html/index.html)
- [ce30_viz](https://github.com/fengqiang111/ce30_viz/tree/develop)
- [ce30_driver_ros](https://github.com/fengqiang111/ce30_driver_ros/tree/develop/)

#### Version

- v2.0.0
