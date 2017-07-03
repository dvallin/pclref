# PCLref #

is a framework for point cloud registration based on the PCL.

The main.cc gives a complete example application, showcasing nearly all the functionality of the PCLref Library. Standard parameters should be looked up at the config folder. Command-line arguments are only used to get the app running, the Parameters are what governs the actual registration and are fed via parameter file to the app. The locations of the point clouds are supplied by a session file to the app. It also contains parsing information for ground truth. Usage examples (although based on my local file structure) are in the tests folder, calling help gives a comprehensive list of arguments.

For more info build the doxygen from the project folder.