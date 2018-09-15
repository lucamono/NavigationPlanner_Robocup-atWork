#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <iostream>
#include <eigen3/Eigen/Geometry>

typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, 2, 4> Coefficients;
typedef Eigen::Matrix<double, 1, 2> Row2;

class CubicSpline {
  private:
    int _num_points;
    int _A_size;
    double _interval;
    Row2 _vi;
    Row2 _vf;
    
    Eigen::VectorXd _t; // time_intervals
    std::vector<Vector2> _p; //points
    
    Eigen::MatrixXd _A; // A matrix used in order to compute velocities NxN
    Eigen::MatrixXd _c; // c matrix used in order to compute velocities Nx2
    Eigen::MatrixXd _v; // v that represents the velocities Nx2

    std::vector<Coefficients> _coefficients;
    
    std::vector<Vector2> _spline_p;
    std::vector<Vector2> _spline_v;
    std::vector<Vector2> _spline_a;
    
  //wv private methods
  private:
    void lineSpace();
    void createMatrixA();
    void createMatrixC();
    void computeVelocities();
    void computeCoefficients();
    void computeCubicSpline();
    
  public:
    CubicSpline(const std::vector<Vector2>& points_, const Row2& vi_ = Row2(0.0,0.0), const Row2& vf_ = Row2(0.0,0.0), double interval_=0.01);
    
    inline void setInitVelocity(Row2& vi_){_vi = vi_;}
    inline void setEndVelocity( Row2& vf_){_vf = vf_;}
    inline void setTimeInterval(double interval_){_interval = interval_;}
    
    inline const std::vector<Vector2>& getPositions() const{return _spline_p;}
    inline const std::vector<Vector2>& getVelocities() const {return _spline_v;}
    inline const std::vector<Vector2>& getAccelerations() const {return _spline_a;}
    
    void compute();

};
#endif /* CUBIC_SPLINE_H */
