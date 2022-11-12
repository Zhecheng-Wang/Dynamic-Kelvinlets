#include "dynamic_kelvinlets.h"
#include <polyscope/surface_mesh.h>

class DynamicKelvinletsDriver {
    public:
    const Eigen::MatrixX3d OV;
    const Eigen::MatrixXi OF;
    DynamicKelvinlets kelvinlets;
    DynamicKelvinletsDriver(const Eigen::MatrixX3d& OV_, const Eigen::MatrixXi& OF_) : OV(OV_), OF(OF_), kelvinlets(OV) {}
    void drawGUI() {
        if (ImGui::Button("Grab")) {
            kelvinlets.compute(BrushType::GRAB);
            polyscope::registerSurfaceMesh("Kelvinlets", kelvinlets.V, OF);
        }
        ImGui::SameLine();
        if (ImGui::Button("Twist")) {
            Eigen::Matrix3d twist_matrix;
            twist_matrix << 0, 1, -1, -1, 0, 1, 1, -1, 0;
            kelvinlets.compute(BrushType::TWIST, Eigen::Vector3d::Zero(), twist_matrix);
            polyscope::registerSurfaceMesh("Kelvinlets", kelvinlets.V, OF);
        }
        ImGui::SameLine();
        if (ImGui::Button("Scale")) {
            kelvinlets.compute(BrushType::SCALE, Eigen::Vector3d(-0.2,0.5,0.5), Eigen::Matrix3d::Identity()*5);
            polyscope::registerSurfaceMesh("Kelvinlets", kelvinlets.V, OF);
        }
        ImGui::SameLine();
        if (ImGui::Button("Pinch")) {
            Eigen::Matrix3d pinch_matrix;
            pinch_matrix << 0, 1, 1, 1, 0, 1, 1, 1, 0;
            kelvinlets.compute(BrushType::PINCH, Eigen::Vector3d::Zero(), pinch_matrix*1e-5);
            polyscope::registerSurfaceMesh("Kelvinlets", kelvinlets.V, OF);
        }
        if (ImGui::Button("Reset")) {
            kelvinlets.reset(OV);
            polyscope::registerSurfaceMesh("Kelvinlets", kelvinlets.V, OF);
        }
    }
};
