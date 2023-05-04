# Robotics Toolbox 

This  repository contains some useful materials for the Robotics1 course at DIAG Sapienza. The course covers fundamental concepts in robotics, including direct and inverse kinematics, differential kinematics, trajectory planning, and kinematic controls.

## Table of Contents
- [Course Materials](#course-materials)
  - [Slides](#slides)
  - [Scripts](#scripts)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Usage](#usage)
- [Contributing](#contributing)
- [Credits](#credits)

## Course Materials

### Slides
The **slides** directory contains lecture slides in PDF format with some additional notes on. The following subdirectories are available:

| Subdirectory              | Description                                                                       |
|---------------------------|-----------------------------------------------------------------------------------|
| 01_direct_kinematics      | Slides on direct kinematics, including position and orientation representation.  |
| 02_inverse_kinematics     | Slides on inverse kinematics, including analytical solutions and numerical methods.|
| 03_direct_differential_kinematics | Slides on differential kinematics, including the geometric and analytical Jacobians. |
| 04_inverse_differential_kinematics| Slides on inverse differential kinematics and static force analysis.                |
| 05_trajectory_planning    | Slides on trajectory planning, including polynomial trajectories and the trapezoidal timing law.|
| 06_kinematic_control      | Slides on kinematic control, including the Jacobian-based control.                     |
| 07_sensors_actuators      | Slides on sensors and actuators used in robotics.                                    |

### Scripts
The **scripts** directory contains MATLAB scripts that demonstrate various concepts covered in the course. The following subdirectories are available:

| Subdirectory              | Description                                                       |
|---------------------------|-------------------------------------------------------------------|
| differential_kinematics   | Scripts for computing the geometric and analytic Jacobians, as well as the time derivative of the Jacobian.|
| direct_kinematics         | Scripts for computing the direct kinematics of robotic manipulators using the Denavit-Hartenberg convention.|
| inverse_kinematic         | Scripts for computing the inverse kinematics of robotic manipulators.|
| position_orientation      | Scripts for computing the orientation of a rigid body given a rotation matrix or Euler angles.|
| trajectory_planning       | Scripts for trajectory planning using cubic and quintic polynomial trajectories, as well as the trapezoidal timing law.|

## Getting Started

### Prerequisites
To use the MATLAB scripts, you will need to have MATLAB installed on your computer. If you are a Sapienza student, you can access MATLAB through the university's license [here](https://it.mathworks.com/academia/tah-portal/sapienza-universita-di-roma-40576534.html).

### Usage
1. Clone this repository to your local machine using: ``git clone git@github.com:leeoos/AIRO-Robotics1.git``
2. Open MATLAB and navigate to the directory where you cloned the repository.
3. Start practicing with the MATLAB scripts!

## Contributing
If you would like to contribute to this repository just submit a pull request as follow:
1. Fork this repository: Click on the "Fork" button on the top right corner of the repository page. This will create a copy of the repository in your own GitHub account.
2. Clone the forked repository to your local machine: Use the git clone command to clone the repository to your local machine.
3. Create a new branch: Use the git checkout -b command to create a new branch for your changes. It's good practice to name the branch something descriptive of the changes you'll be making.
4. Make your changes: Make the changes you want to make to the repository.
5. Commit your changes: Use the git add and git commit commands to commit your changes to the new branch.
6. Push your changes to your forked repository: Use the git push command to push your changes to your forked repository.
7. Create a pull request: Go to this repository, and click on the "New pull request" button. Select the branch you created in step 3 as the compare branch, and the original repository's main branch as the base branch.
8. Submit the pull request: Provide a brief description of the changes you made, and submit the pull request.

## Credits
We want to thank our classmates and all the other fellow students who contributed to the realization of the project.
