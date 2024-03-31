## [Learning Neural Force Manifolds for Sim2Real Robotic Symmetrical Paper Folding](https://ieeexplore.ieee.org/abstract/document/10445527)


This repo contains the source code for the manuscript "*Learning Neural Force Manifolds for Sim2Real Robotic Symmetrical Paper Folding*". This paper introduced a methodology that can use only one robotic arm to fold a paper to form a prescribed crease when minimizing the sliding of the paper during the manipulation. The method combines numerical simulation of the deformed paper, machine learning to create a neural force manifold, and optimal algorithm (uniform cost search) to generate the globally optimal path.

The numerical simulation framework in the `simulation` folder generates the training data.

By running `main.py`, the optimal folding trajectories can be generated in the pre-trained neural force manifold mode.

<p align="center">
<img src="images/knot_tying.png" alt>
<br>
<em> Figure 1. Comparison between an intuitive baseline and the designed optimal manipulations for paper folding. </em>
</p>

## How to Use

### Dependencies
Install the following python dependencies:
```bash
numpy
scipy
shapely
torch
matplotlib
```

### Setting Parameters

Some parameters can be changed directly in ```main.py``` to generate different types of trajectories.


Specifiable parameters are as follows:
- ```Lgb_b``` - The material parameter encapsulates the deformations of the paper under gravity. (See paper for the details)
- ```delta_r``` - Discretization of the trained neural force manifold
- ```start``` - The start point in the workspace of the trajectory
- ```goal``` - The goal point in the workspace of the trajectory


### Running the code for trajectory generation
Once parameters are set to your liking, the code can be executed from the terminal by running the provided script:
```bash
python3 main.py
```

### Citation
If our work has helped your research, please cite the following paper.
```
@article{choi2024learning,
  title={Learning Neural Force Manifolds for Sim2Real Robotic Symmetrical Paper Folding},
  author={Choi, Andrew and Tong, Dezhong and Terzopoulos, Demetri and Joo, Jungseock and Jawed, Mohammad Khalid},
  journal={IEEE Transactions on Automation Science and Engineering},
  year={2024},
  publisher={IEEE}
}
```
