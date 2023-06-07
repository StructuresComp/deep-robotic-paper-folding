## Simulation Framework
Discrete differentiable geometry (DDG)-based simulations for generating training data to construct the mapping from the robot's grasp to the ratio of the forces applied on the manipulated paper from the substrate.

All code tested and developed on **Ubuntu 20.04.4 LTS** using **Python 3.9.12** and **C++11**.
***
#### How to use
First, create a "build" folder under the "simulation" folder, then go to that folder
```bash
mkdir build && cd build # go to the build folder
```
Then, use the cmake command to compile the configuration of the simulation programs
```bash
cmake ..
```
Finally, compile the programs, the simulation programs "simF" and "simBC" will appear under the "simulation" folder.
```bash
make -j4
```

The optionB.txt and optionF.txt contains all simulation settings. Here describes the details of each parameter (unitless):
- ```tol``` and ```stol``` - Small numbers used in solving the linear system. Fraction of a percent, e.g. 1.0e-3, is often a good choice.
- ```maxIter``` - Maximum number of iterations allowed before the solver quits. 
- ```viscosity``` - Viscosity for applying damping forces.
- ```render (0 or 1) ```- Flag indicating whether OpenGL visualization should be rendered.
- ```saveData (0 or 1)``` - Flag indicating whether pull forces and rod end positions should be reocrded.
- ```deltaTime``` - Time step size.
- ```filename``` - File name for sampling points. This is the fixed name when using the python file.


### Dependencies
Install the following C++ dependencies:
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
  - Eigen is used for various linear algebra operations.
  - IMC is built with Eigen version 3.4.0 which can be downloaded [here](https://gitlab.com/libeigen/eigen/-/releases/3.4.0). After downloading the source code, install through cmake as follows.
    ```bash
    cd eigen-3.4.0 && mkdir build && cd build
    cmake ..
    sudo make install
    ```
- [OpenGL / GLUT](https://www.opengl.org/)
  - OpenGL / GLUT is used for rendering the knot through a simple graphic.
  - Simply install through apt package manager:
      ```bash
    sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
    ```
- Lapack (*usually preinstalled on your computer*)

***
#### Dependencies for Python
Run the following to download all dependencies.
```bash
pip3 install numpy matplotlib
```
***
