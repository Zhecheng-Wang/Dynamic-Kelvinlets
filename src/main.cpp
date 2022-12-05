#include <vector>
#include <string>
#include <unordered_map>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <igl/readOBJ.h>
#include "dynamic_kelvinlets_driver.h"

int main(int argc, char** argv) {
    // parsing
    if (argc < 2) {
        std::cout << "Usage: [file name]" << std::endl;
        return 1;
    }
    std::string file_name = argv[1];
    Eigen::MatrixX3d OV;
    Eigen::MatrixXi OF;
    if (!igl::readOBJ(file_name, OV, OF)) {
        std::cout << "Cannot Read " << file_name << "." << std::endl;
        return 1;
    }
    if (OF.cols() > 3) {
        std::cout << "Does Not Support Quad Mesh." << std::endl;
        return 1;
    }

    // polyscope settings
    polyscope::options::programName = "Dynamic Kelvinlets";
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneEnabled = false;
    polyscope::view::bgColor = { 0.0f, 0.0f, 0.0f, 0.0f };
    polyscope::init();

    // add original mesh
    polyscope::registerSurfaceMesh("Original", OV, OF)->setEnabled(false);

    // add morphed mesh
    DynamicKelvinletsDriver driver(OV, OF);
    polyscope::state::userCallback = std::bind(&DynamicKelvinletsDriver::drawGUI, &driver);
    
    // display
    polyscope::show();
    
    return 0;
}
