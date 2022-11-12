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
- [ ] Scale
- [ ] Pinch

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

---
### References
[[de Goes *et al.*, 2017]](https://graphics.pixar.com/library/Kelvinlets/paper.pdf)

[[de Goes *et al.*, 2018]](https://graphics.pixar.com/library/DynaKelvinlets/paper.pdf)

