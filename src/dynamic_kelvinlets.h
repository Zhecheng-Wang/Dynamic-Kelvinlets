#include <igl/PI.h>
#include <Eigen/Dense>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

enum class BrushType : int
{
  GRAB,
  SCALE,
  TWIST,
  PINCH
};

class DynamicKelvinlets {
    public:
    double dt = 0.005;
    double Ps = 0.45;
    double Ym = 100;
    double beta = std::sqrt(Ym);
    double alpha = beta*std::sqrt(1.+1./(1.-2.*Ps));
    double epsilon = 1;

    const Eigen::MatrixX3d OV;
    Eigen::MatrixX3d V;

    double time = 0.0;

    DynamicKelvinlets(const Eigen::MatrixX3d& OV_): OV(OV_), V(OV_) {}

    inline Eigen::ArrayXd W(const Eigen::ArrayXd r, const Eigen::ArrayXd s) {
        const Eigen::ArrayXd s_epsilon = (s.matrix().rowwise().squaredNorm().array() + std::pow(epsilon, 2)).sqrt();
        const Eigen::ArrayXd s_epsilon_inv = s_epsilon.inverse();
        const Eigen::ArrayXd s_epsilon_3_inv = s_epsilon.pow(3).inverse();
        return s_epsilon_inv*(2*s.pow(2)+std::pow(epsilon, 2)-3*r*s) + s_epsilon_3_inv*r*s.pow(3);
    }

    inline Eigen::Array3Xd U(const Eigen::ArrayXd r, const double t, const double gamma) {
        return (16.*igl::PI*(r.pow(3))).inverse() * (W(r, r+gamma*t) - W(r, r-gamma*t));
    }

    inline Eigen::ArrayXd dWdr(const Eigen::ArrayXd r, const Eigen::ArrayXd s) {
        const Eigen::ArrayXd s_epsilon = (s.matrix().rowwise().squaredNorm().array() + std::pow(epsilon, 2)).sqrt();
        const Eigen::ArrayXd s_epsilon_5_inv = s_epsilon.pow(5).inverse();
        return -3 * std::pow(epsilon, 4) * (r * s_epsilon_5_inv);
    }

    inline Eigen::ArrayXd dUdr(const Eigen::ArrayXd r, const double t, const double gamma) {
        return (16.*igl::PI*(r.pow(3))).inverse() * 
            ((dWdr(r, r+gamma*t) - dWdr(r, r-gamma*t))
            -3*r.inverse() * (W(r, r+gamma*t) - W(r, r-gamma*t)));
    }

    inline Eigen::ArrayXd dWdrr(const Eigen::ArrayXd r, const Eigen::ArrayXd s) {
        const Eigen::ArrayXd s_epsilon = (s.matrix().rowwise().squaredNorm().array() + std::pow(epsilon, 2)).sqrt();
        const Eigen::ArrayXd s_epsilon_7_inv = s_epsilon.pow(7).inverse();
        return -3 * std::pow(epsilon, 4) * (s_epsilon.pow(2)-5*r*s)*s_epsilon_7_inv;
    }

    inline Eigen::ArrayXd dUdrr(const Eigen::ArrayXd r, const double t, const double gamma) {
        return (16.*igl::PI*(r.pow(3))).inverse() * 
            ((dWdrr(r, r+gamma*t) - dWdrr(r, r-gamma*t))
            -6*r.inverse()*(dWdr(r, r+gamma*t) - dWdr(r, r-gamma*t))
            -12*r.inverse()*(dWdrr(r, r+gamma*t) - dWdrr(r, r-gamma*t)));
    }

    inline Eigen::ArrayXd A(const Eigen::ArrayXd r, const double t) {
        return U(r, t, alpha) + 2*U(r, t, beta) + r*dUdr(r, t, beta);
    }

    inline Eigen::ArrayXd B(const Eigen::ArrayXd r, const double t) {
        return (dUdr(r, t, alpha) - dUdr(r, t, beta)) * r.inverse();
    }

    inline Eigen::ArrayXd dAdr(const Eigen::ArrayXd r, const double t) {
        return dUdr(r, t, alpha) + 3*dUdr(r, t, beta) + r*dUdrr(r, t, beta);
    }

    inline Eigen::ArrayXd dBdr(const Eigen::ArrayXd r, const double t) {
        return (dUdrr(r, t, alpha) - dUdrr(r, t, beta) - B(r, t)) * r.inverse();
    }

    void displace(BrushType brush_type = BrushType::GRAB, Eigen::Vector3d x0 = Eigen::Vector3d::Zero(), Eigen::Matrix3d force = Eigen::Matrix3d::Zero()) {
        beta = std::sqrt(Ym);
        alpha = beta*std::sqrt(1.+1./(1.-2.*Ps));
        V = OV;
        Eigen::MatrixX3d u0 = compute(V, brush_type, x0, force, time);
        Eigen::MatrixX3d u1 = compute(V+0.5*u0, brush_type, x0, force, time);
        Eigen::MatrixX3d u2 = compute(V+0.5*u1, brush_type, x0, force, time);
        Eigen::MatrixX3d u3 = compute(V+u2, brush_type, x0, force, time);
        V += (u0 + 2.*u1 + 2.*u2 + u3) / 6.;
        time += dt;
    }

    Eigen::MatrixX3d compute(const Eigen::MatrixX3d V, const BrushType brush_type, const Eigen::Vector3d x0, const Eigen::Matrix3d force, const double t) {
        const Eigen::MatrixX3d r = (V.rowwise()-x0.transpose()).array();
        const Eigen::ArrayXd r_norm = r.rowwise().norm().array();
        const Eigen::ArrayXd r_inv = r_norm.inverse();
        Eigen::ArrayX3d u = Eigen::ArrayX3d::Zero(V.rows(), 3);
        switch (brush_type) {
            case BrushType::GRAB: {

            } break;
            case BrushType::TWIST: {
                u = (r*force).array().colwise() * (r_inv*dAdr(r_norm, t)-B(r_norm, t));
            } break;
            case BrushType::SCALE: {
                const double s = -50.;
                u = (s*r).array().colwise() * (4.*B(r_norm, t)+r_inv*dAdr(r_norm, t)+r_norm*dBdr(r_norm, t));
            } break;
            case BrushType::PINCH: {
                u = (r*force).array().colwise() * (r_inv*dAdr(r_norm, t)+B(r_norm, t)) +
                (r*(r.transpose()*(r*force).transpose())).array()*r_inv*dBdr(r_norm, t);
            } break;
            default:
            break;
        }

        return u.matrix();
    }
};
