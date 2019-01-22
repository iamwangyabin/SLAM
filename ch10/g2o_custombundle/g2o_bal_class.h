#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"

#include <Eigen/Core>
#include "ceres/autodiff.h"

#include "common/projection.h"
#include "tools/rotation.h"


/**
 *  图优化里面的相机点
 *  自定义了相机顶点和路标顶点（名字自己定），继承自g2o::BaseVertex.一个9维，
 * */
class VertexCameraBAL:public g2o::BaseVertex<9,Eigen::VectorXd>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL(){}
    virtual bool read(std::istream& is){ return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void setToOriginImpl(){}
    /**
     * 增量函数，增量为传进的参数update，这里是9个double值，所以就是double类型指针了(其实也就是数组)
     * */
    virtual void oplusImpl(const double* update) {
        Eigen::VectorXd::ConstMapType v(update,VertexCameraBAL::Dimension);
        _estimate += v;
    }
};

/**
 *  图优化里面的路标点
 *  Eigen::VectorXd，一个3维，Eigen::Vector3d.
 * */
class VertexPointBAL:public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL(){}
    virtual bool read(std::istream& is){ return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void setToOriginImpl(){}

    /**
     * 对update进行操作，变成v，之前都是 _estimate += Eigen::Vector3d(update);
     * update不再是Eigen::Vector3d的形式，而是变成了地图类型
     * */
    virtual void oplusImpl(const double* update) {
        Eigen::VectorXd::ConstMapType v(update);
        _estimate += v;
    }
};

/**
 * BAL观测边，边即误差，继承自基础二元边。这里误差应该是重投影的像素误差
 * 参数为：误差维度2维，误差类型为Eigen::Vector2d，连接两个顶点：VertexCameraBAL和VertexPointBAL(也就是说误差和这两个优化变量有关)
 * */
class EdgeObservationBAL:public g2o::BaseBinaryEdge<2,Eigen::Vector2d, VertexCameraBAL,VertexPointBAL>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL(){}

    virtual bool read(std::istream& is){ return false; }
    virtual bool write(std::ostream& os) const { return false; }

    /**
     * 观测值在当前图片中可以看到这个路标，通过计算描述子可以在当前图片中找到和路标匹配的特征点，这个匹配的特征点的像素坐标就是观测值。
     * 把vertex(0)赋值给顶点camera,vertex(1)赋值给顶点point.
     * 这里的计算误差的函数被重写了。
     * */
    virtual void computeError() override {
        const VertexCameraBAL* cam= static_cast<const VertexCameraBAL*>(vertex(0));
        const VertexPointBAL* point= static_cast<const VertexPointBAL*>(vertex(1));
        (*this)(cam->estimate().data(),point->estimate().data(),_error.data());
    }

    template <typename T>
    bool operator()(const T* camera,const T* point, T* residuals) const {
        T predictions[2];
        CamProjectionWithDistortion(camera,point,predictions);
        residuals[0]=predictions[0]-T(measurement()(0));
        residuals[1]=predictions[1]-T(measurement()(1));
        return true;
    }

    virtual void linearizeOplus() override{
        const VertexCameraBAL* cam= static_cast<const VertexCameraBAL*>(vertex(0));
        const VertexPointBAL* point= static_cast<const VertexPointBAL*>(vertex(1));
        /**
         * 四个参数分别是边类，数据类型，两个顶点类。
         * */
        typedef ceres::internal::AutoDiff<EdgeObservationBAL,double,VertexCameraBAL::Dimension,VertexPointBAL::Dimension> BalAutoDiff;
        /**
         * 定义对Camera顶点求导后的矩阵，dError_dCamera,维度Dimension*顶点的维度，应该是9.
         * */
        Eigen::Matrix<double,Dimension,VertexCameraBAL::Dimension,Eigen::RowMajor> dError_dCamera;
        /**
         * 对Point顶点求导后的矩阵dError_dPoint.
         * */
        Eigen::Matrix<double,Dimension,VertexCameraBAL::Dimension,Eigen::RowMajor> dError_dPoint;
        double *parameters[]={const_cast<double *>(cam->estimate().data()), const_cast<double *>(point->estimate().data())};
        /**
         * 雅克比矩阵是2*6形式。每个误差都有一个雅可比矩阵，这里应该是每个雅可比矩阵的形式。
         * 由camera的导数和point的导数组成。
         * */
        double *jacobians[]={const_cast<double *>(dError_dCamera.data()), dError_dPoint.data()};
        double value[Dimension];

        /**
         * 放入边，参数，维度Dimension,value(它的值就是Dimension),还有要得到雅可比矩阵jacobian.
         * */
        bool diffState=BalAutoDiff::Differentiate(*this,parameters,Dimension,value,jacobians);

        if(diffState){
            _jacobianOplusXi=dError_dCamera;
            _jacobianOplusXj=dError_dPoint;
        } else{
            assert(0&&"rooro");
            _jacobianOplusXi.setZero();
            _jacobianOplusXj.setZero();
        }

    }
};