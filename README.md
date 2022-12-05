# Dynamic Kelvinlets: Secondary Motions based on Fundamental Solutions of Elastodynamics
A naive C++ implementaion of the SIGGRAPH 2018 paper Dynamic Kelvinlets (de Goes et al. 2018) with a viewer and time integration. This is a CMake project.

---
### Author
Zhecheng Wang

---
### Implemented Brushes
- [ ] Grab
- [X] Twist
- [X] Scale
- [X] Pinch

---
### Working Brushes
- [ ] Grab
- [X] Twist
- [X] Scale
- [X] Pinch

---
### Prerequisites
[polyscope](https://polyscope.run/)

[libigl](https://libigl.github.io/)

---
### Installation
Clone from this repo

    git clone https://github.com/Zhecheng-Wang/Dynamic-Kelvinlets.git

Initialize build folder and compile the code

    mkdir build
    cd build
    cmake ..
    make

To run the program, run ``kelvinlets`` in the ``build`` folder with a mesh in the ``data`` folder.
For example,

    ./kelvinlets ../data/spot_triangulated.obj

### Documentation
Note that all parameters are tuned for demonstration purpose. Kelvinlet algorithms are essentially a force field applied in the space, they are not aware of boundaries.

Here are the three force fields I implemented and tested.

Twisting
$$\boldsymbol{t}(\boldsymbol{r},t) = \left[\frac{1}{r}\partial_r \mathcal{A}(r,t)-\mathcal{B}(r,t)\right]\boldsymbol{r} \times \boldsymbol{r}$$

Scaling
$$\boldsymbol{s}(\boldsymbol{r},t) = \left[4\mathcal{B}(r,t) + \frac{1}{r}\partial_r \mathcal{A}(r,t)+r\partial_r\mathcal{B}(r,t)\right]s\boldsymbol{r}$$

Pinching
$$\boldsymbol{p}(\boldsymbol{r},t) = \left[\frac{1}{r}\partial_r \mathcal{A}(r,t)+\mathcal{B}(r,t)\right]\boldsymbol{F}\boldsymbol{r} + \frac{1}{r}\partial_r\mathcal{B}(r,t)(\boldsymbol{r}^T\boldsymbol{F}\boldsymbol{r})\boldsymbol{r}$$

### How To Use?
Click the brush (Twist, Scale, Pinch) buttons on GUI to "time stepping" the mesh! After using every brush, please hit the "Reset" button to reset mesh state.

By default, the force field center is always around the neck. So you will see deformation there.

---
### References
[[de Goes *et al.*, 2017]](https://graphics.pixar.com/library/Kelvinlets/paper.pdf)

[[de Goes *et al.*, 2018]](https://graphics.pixar.com/library/DynaKelvinlets/paper.pdf)

