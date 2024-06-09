# Diamond (Aras Eye Surgical Robot) Kinematic Calibration

This repository contains the code for calibrating the hand-eye transformation of a robot arm using a diamond calibration pattern. The calibration is performed using the Symforce library for symbolic computation and code generation.

### Project Structure

The project is organized into the following folders:

- **cpp/symforce/sym**: Contains the Symforce-generated C++ code for the error model function and its Jacobians.
- **data**: Contains the ground truth data and joint values used for calibration.
- **lib**: Contains the C++ header files for the GTSAM factors used in the calibration process.
- **system_model**: Contains the Symforce Python code for defining the error model function and generating the C++ code.
- **verification**: Contains the Python code for verifying the calibration results.

### Calibration Process

The calibration process involves the following steps:

1. **Data Acquisition**: Collect data of the robot arm's joint values and the corresponding ground truth poses of the camera.
2. **Error Model Definition**: Define the error model function that calculates the difference between the predicted camera pose based on the robot's kinematics and the ground truth camera pose.
3. **Code Generation**: Use Symforce to generate C++ code for the error model function and its Jacobians.
4. **Factor Graph Construction**: Construct a GTSAM factor graph using the generated C++ code and the collected data.
5. **Optimization**: Optimize the factor graph to estimate the hand-eye transformation.
6. **Verification**: Verify the calibration results by comparing the predicted camera poses with the ground truth poses.

### Usage

To use the code, follow these steps:

1. **Install Dependencies**: Install the required dependencies, including Symforce, GTSAM, and Eigen.
2. **Generate C++ Code**: Run the `Diamon_model_symforce.py` script in the `system_model` folder to generate the C++ code for the error model function and its Jacobians.
3. **Compile C++ Code**: Compile the generated C++ code and link it with the GTSAM library.
4. **Run Calibration**: Run the `DiamondFactor.cpp` file to perform the calibration process.
5. **Verify Results**: Run the `Hand_Eye_Verification.py` script to verify the calibration results.

### Example

The `data` folder contains two CSV files:

- **GT_data_and_joint_values_for_factor_graph_base_on_symforce.csv**: Contains the ground truth data and joint values for a specific calibration experiment.
- **GT_data_and_joint_values_for_factor_graph_base_on_symforce_20.csv**: Contains the ground truth data and joint values for a different calibration experiment.

The `DiamondFactor.cpp` file uses the data from the `GT_data_and_joint_values_for_factor_graph_base_on_symforce_20.csv` file to perform the calibration. The `Hand_Eye_Verification.py` script then verifies the calibration results using the data from both CSV files.

### Notes

- The calibration process assumes that the diamond calibration pattern is known and that the camera's intrinsic parameters are calibrated.
- The `DiamondCalibrationFactor` class in the `lib` folder implements the GTSAM factor for the diamond calibration error model.
- The `Hand_Eye_Verification.py` script uses the Symforce library to symbolically calculate the error between the predicted and ground truth camera poses.

### Future Work

- Implement a more robust calibration algorithm that can handle noisy data and outliers.
- Extend the calibration process to include other robot arm parameters, such as joint offsets and link lengths.
- Develop a user-friendly interface for data acquisition and calibration.

### Acknowledgements

This project was inspired by the work of [insert relevant references here].

### License

This project is licensed under the [insert license name here] license.
# Paper
[Link](https://drive.google.com/file/d/1xQ3zcgaOW9Up-gMPBI9qiqMDW6Fq9o3l/view?usp=sharing)
