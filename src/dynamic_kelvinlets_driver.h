#include "dynamic_kelvinlets.h"
#include <polyscope/surface_mesh.h>

class DynamicKelvinletsDriver {
    public:
    const int steps = 10000;
    const Eigen::MatrixX3d OV;
    const Eigen::MatrixXi OF;
    DynamicKelvinlets kelvinlets;
    BrushType brush_type = BrushType::GRAB;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    polyscope::SurfaceMesh* mesh = nullptr;

    DynamicKelvinletsDriver(const Eigen::MatrixX3d& OV_, const Eigen::MatrixXi& OF_) : OV(OV_), OF(OF_), kelvinlets(OV) {
        mesh = polyscope::registerSurfaceMesh("Kelvinlets", OV, OF);
    }
    void drawGUI() {
        if (ImGui::Button("Grab")) {
            brush_type = BrushType::GRAB;
            kelvinlets.displace(brush_type, center);
            mesh->updateVertexPositions(kelvinlets.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Twist")) {
            brush_type = BrushType::TWIST;
            Eigen::Matrix3d twist_matrix;
            twist_matrix << 0, 1, -1, -1, 0, 1, 1, -1, 0;
            kelvinlets.dt = 0.01;
            kelvinlets.Ym = 100000;
            kelvinlets.epsilon = 1.5;
            twist_matrix *= 0.25;
            kelvinlets.displace(brush_type, center, twist_matrix);
            mesh->updateVertexPositions(kelvinlets.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Scale")) {
            brush_type = BrushType::SCALE;
            Eigen::Matrix3d scale_matrix = Eigen::Matrix3d::Identity();
            kelvinlets.dt = 0.0075;
            kelvinlets.Ym = 50000;
            kelvinlets.epsilon = 1;
            scale_matrix *= 1;
            kelvinlets.displace(brush_type, center, scale_matrix);
            mesh->updateVertexPositions(kelvinlets.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Pinch")) {
            brush_type = BrushType::PINCH;
            Eigen::Matrix3d pinch_matrix;
            pinch_matrix << 0, 1, 1, 1, 0, 1, 1, 1, 0;
            kelvinlets.dt = 0.01;
            kelvinlets.Ym = 250000;
            kelvinlets.epsilon = 1.5;
            pinch_matrix *= 0.25;
            kelvinlets.displace(brush_type, center, pinch_matrix);
            mesh->updateVertexPositions(kelvinlets.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            kelvinlets.time = 0;
            mesh->updateVertexPositions(kelvinlets.OV);
        }
    }
};
