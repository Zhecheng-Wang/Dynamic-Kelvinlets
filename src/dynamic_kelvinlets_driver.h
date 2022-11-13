#include "dynamic_kelvinlets.h"
#include <polyscope/surface_mesh.h>

class DynamicKelvinletsDriver {
    public:
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
            kelvinlets.step(brush_type, center);
            mesh->updateVertexPositions(kelvinlets.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Twist")) {
            brush_type = BrushType::TWIST;
            Eigen::Matrix3d twist_matrix;
            twist_matrix << 0, 1, -1, -1, 0, 1, 1, -1, 0;
            kelvinlets.step(brush_type, center, twist_matrix);
            mesh->updateVertexPositions(kelvinlets.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Scale")) {
            brush_type = BrushType::SCALE;
            kelvinlets.step(brush_type, center, Eigen::Matrix3d::Identity());
            mesh->updateVertexPositions(kelvinlets.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Pinch")) {
            brush_type = BrushType::GRAB;
            Eigen::Matrix3d pinch_matrix;
            pinch_matrix << 0, 1, 1, 1, 0, 1, 1, 1, 0;
            kelvinlets.step(brush_type, center, pinch_matrix);
            mesh->updateVertexPositions(kelvinlets.V);
        }
        if (ImGui::Button("Step")) {
            kelvinlets.step(brush_type, center);
            mesh->updateVertexPositions(kelvinlets.V);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            kelvinlets.reset(OV);
            mesh->updateVertexPositions(kelvinlets.V);
        }
    }
};
