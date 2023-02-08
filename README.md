# Visual-odometry
To build this project:
- create build directory with `mkdir build` and navigate to it with `cd build`
- then do `cmake ..` and `make`

A folder `exec/` will be created inside the build folders, with the following executables:
- `init`: syntethic data and measurements are generated, and the pose between two images is estimated using epipolar geometry
- `picp_test`: synthetic data and measurements from 2 poses are generated, the relative pose is estimated using picp
- `whole_test`: synthetic data and 3 sets of measurements are generated: estimation of the relative pose between the first two cameras is done using epipolar geometry. Given this transformation, we triangulate to estimate the position of the points in the world, and then use this estimate to perform picp
- `read_data_test`: a test to see if the measurement files are retrieved correctly
- `real_init`: the relative position between the first 2 set of real measurements is estimated using epipolar geometry
- `picp_known_real`: picp is performed on the given data, assuming that the position of the points in the world is known and also data association in known
- `vo_daKnown`: the relative position between the first 2 set of measurements is estimated using epipolar geometry, then triangulation is performed to find a set of world points, and then iteratively picp is executed between subsequent poses, each time triangulating to find a new set of world points. Data association is given
- `vo_complete`: same as before, but data association is not given. Correspondances are retrieved using a kd tree leveraging on the appearances of the points
- `evaluation`: comparison between ground truth and predicted trajectory previously saved. Given two estimated absolute poses $T_0,T_1$ and the corresponding ground truth poses $T_{0,GT},T_{1,GT}$ represented as homogenous matrices $\in \mathbb{R}^{4\times4}$, we can compute the relative poses as
```math
\textbf{T}_{rel}=\textbf{T}^{-1}_{0}\textbf{T}_1
```
```math
\textbf{T}_{rel,GT}=\textbf{T}^{-1}_{0,GT}\textbf{T}_{1,GT}
```
and then the orientation error $e_{\theta}$ and position ratio $r_t$ are computed as
```math
e_{\theta}=trace(\textbf{I}_{3}-\textbf{T}_{err}(1:3,1:3))
```
```math
r_t=\frac{||\textbf{T}_{rel}(1:3,4)||}{||\textbf{T}_{rel,GT}(1:3,4)||}
```
where matlab-like indexing is used. These last two quantities are used for evaluation.
