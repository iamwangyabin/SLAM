#include "core/base_vertex.h"
#include "core/base_binary_edge.h"

#include <Eigen/Core>
#include "ceres/autodiff.h"

#include "tools/rotation.h"
#include "common/projection.h"


class VertexCameraBAL:public g2o::BaseVertex<9,Eigen::VectorXd>
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL(){};
    virtual bool read(std::istream& is){return false};
    virtual bool write(std::ostream& os) const{return false};
    virtual void setToOriginImpl(){};

    virtual void oplusImpl(const double* update)override{
        Eigen::VectorXd::ConstMapType v(update,VertexCameraBAL::Dimension);
        _estimate += v;
    }
};

class VertexPointBAL:public g2o::BaseVertex<3, Eigen::Vector3d>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexPointBAL(){};
        virtual bool read(std::istream& is){return false};
        virtual bool write(std::ostream& os) const{return false};
        virtual void setToOriginImpl(){};

        virtual void oplusImpl(const double* update)override{
            Eigen::Vector3d::ConstMapType v(update);
            _estimate += v;
        }
};

