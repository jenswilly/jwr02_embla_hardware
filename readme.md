# JWR-01 Embla Hardware Controller

This code is based heavily on the hardware controller of the [Clearpath Husky robot](https://github.com/husky/husky). 🙇‍♂️ many thanks and heaps of streetcred to the [Clearpath team](https://www.clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/).

Likewise, 🙇‍♂️ many thanks to Zach Allen of [Slate Robotics](https://slaterobots.com/) for the [excellent article on ros_control](https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot) which also helped me a lot to get a grip on how controllers work in ROS.

Thirdly, 🙇‍♂️ many thanks to Carroll Vance for the [C++ version of the RoboClaw driver](https://github.com/csvance/roboclaw) which is used in this project.

### Programming Style Guide

This project (largely) follows the guidelines at http://geosoft.no/development/cppstyle.html.

## Controllers
...

## Diagnostics
The `embla_hardware` contains a diagnostics updater that is updated from the `embla_hardware_node`'s `diagnosticsLoop()` function.

The diagnostics updater is a `diagnostic_updater::DiagnosticTask` sublass and uses a `EmblaEMCUStatus` message to pass status.
