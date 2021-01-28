#include <iostream>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "bal_parser.h"

using namespace std;

struct PoseAndIntrins
{
    Sophus::SO3d rotation;
    Eigen::Vector3d translation;
    double focal;
    double k1, k2;
};

// the dim of update is 9
// the type of _estimate is PoseAndIntrins
class VertexCamera: public g2o::BaseVertex<9, PoseAndIntrins>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override
    {
        _estimate = PoseAndIntrins();
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate.rotation = Sophus::SO3d::exp(Eigen::Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.translation += Eigen::Vector3d(update[3], update[4], update[5]);
        _estimate.focal += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    }

    Eigen::Vector2d Project(const Eigen::Vector3d& pt_3d)
    {
        Eigen::Vector3d pt_3d_in_cam = _estimate.rotation.matrix() * pt_3d + _estimate.translation;
        pt_3d_in_cam /= pt_3d_in_cam.z();
        double r2 = pt_3d_in_cam.squaredNorm();
        double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2); 
        Eigen::Vector2d pt_2d;
        pt_2d.x() = - _estimate.focal * distortion * pt_3d.x();
        pt_2d.y() = - _estimate.focal * distortion * pt_3d.y();
        return pt_2d;
    }

    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
};

class VertexPoint: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override
    {
        _estimate = Eigen::Vector3d();
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate += Eigen::Vector3d(update[0], update[1], update[2]);
    }
    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
};

class EdgeProjection: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCamera, VertexPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjection(Eigen::Vector2d& pt): pt_(pt){};

    virtual void computeError() override
    {
        VertexCamera* vCam = static_cast<VertexCamera*>(_vertices[0]);
        VertexPoint* vP = static_cast<VertexPoint*>(_vertices[1]);
        Eigen::Vector2d reproj_pt = vCam->Project(vP->estimate());
        _error = pt_ - reproj_pt;
    }

    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
public:
    Eigen::Vector2d pt_;
};

int main()
{
    BALParser paser("dataset/problem-16-22106-pre.txt");
    return 0;
}