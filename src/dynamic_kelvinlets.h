#include <igl/PI.h>
#include <Eigen/Dense>

enum class BrushType : int
{
  GRAB,
  SCALE,
  TWIST,
  PINCH
};

class DynamicKelvinlets {
    public:
    const double dt = 0.1;
    const double Ps = 0.5;
    const double Ym = 1.0;
    const double a = 1/(4*3.1415926*Ym);
    const double b = a/(4*(1-Ps));
    const double c = 2/(3*a-2*b);
    Eigen::MatrixX3d V;
    const double epsilon = 1e-5;

    double t = 0.0;

    DynamicKelvinlets(const Eigen::MatrixX3d& OV_): V(OV_) {}

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
        return U(r, t, a) + 2*U(r, t, b) + r*dUdr(r, t, b);
    }

    inline Eigen::ArrayXd B(const Eigen::ArrayXd r, const double t) {
        return (dUdr(r, t, a) - dUdr(r, t, b)) * r.inverse();
    }

    inline Eigen::ArrayXd dAdr(const Eigen::ArrayXd r, const double t) {
        return dUdr(r, t, a) + 3*dUdr(r, t, b) + r*dUdrr(r, t, b);
    }

    inline Eigen::ArrayXd dBdr(const Eigen::ArrayXd r, const double t) {
        return (dUdrr(r, t, a) - dUdrr(r, t, b) - B(r, t)) * r.inverse();
    }

    void compute(BrushType brush_type = BrushType::GRAB, Eigen::Vector3d x0 = Eigen::Vector3d::Zero(), Eigen::Matrix3d force = Eigen::Matrix3d::Identity()) {
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
                const double s = -5.;
                u = (s*r).array().colwise() * (4.*B(r_norm, t)+r_inv*dAdr(r_norm, t)+r_norm*dBdr(r_norm, t));
            } break;
            case BrushType::PINCH: {
                u = (r*force).array().colwise() * (r_inv*dAdr(r_norm, t)+B(r_norm, t)) +
                (r*(r.transpose()*(r*force).transpose())).array()*r_inv*dBdr(r_norm, t);
            } break;
            default:
            break;
        }
        V += dt * u.matrix();
        t += dt;
    }

    void reset(const Eigen::MatrixX3d& V_) {
        V = V_;
    }
};
