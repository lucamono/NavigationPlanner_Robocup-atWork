#include "cubic_spline.h"

CubicSpline::CubicSpline(const std::vector< Vector2 >& points_, const Row2& vi_, const Row2& vf_, double interval_){
  _num_points = points_.size();
  _A_size = _num_points-2;
  _interval = interval_;
  
  _p = points_;
  _t = Eigen::VectorXd(_num_points);
  _A = Eigen::MatrixXd(_A_size, _A_size);
  _c = Eigen::MatrixXd(_A_size, 2);
  _v = Eigen::MatrixXd(_num_points, 2);

  
  _vi = vi_;
  _vf = vf_;
   
}

double getDistance(Vector2 p1, Vector2 p2){
  return sqrt(pow(p2.x() - p1.x(),2) + pow(p2.y() - p1.y(),2));
}


void CubicSpline::lineSpace(){
  
  double total_norm = 0.0;
  std::vector<double> norms;
  for(int k=0; k < _num_points-1; k++){
    double dist_k =  getDistance(_p[k], _p[k+1]);
    total_norm += dist_k;
    norms.push_back(dist_k*_num_points);
  }
  
  for(int k = 0; k < _num_points; k ++){
    double tt = norms[k]/total_norm;
    if(tt>=0.5){
      tt=1;
      
    }else if(tt<0.5){
      tt=1;
    }
    _t(k) =tt;  
  }
//   std::cout << "Total norm:" << total_norm << std::endl;
//   std::cout << "lineSpace:" << std::endl;
//   std::cout << _t << std::endl;
}


void CubicSpline::createMatrixA(){

  Eigen::MatrixXd  A_tmp(_A_size, _num_points);
  A_tmp.setZero();
    
  for(int i=0; i< _A_size; i++){
    A_tmp(i, i + 0) = _t(i+1);
    A_tmp(i, i + 1) = 2.0*( _t(i) + _t(i+1) );
    A_tmp(i, i + 2) = _t(i); 
  }
  _A = A_tmp.block(0, 1, _A_size, _A_size);
//   std::cout << "A_tmp" << std::endl;
//   std::cout << A_tmp << std::endl;
//   std::cout << "A" << std::endl;
//   std::cout << _A << std::endl;
}


void CubicSpline::createMatrixC(){

  for(int i=0; i<_A_size; i++){
    _c.row(i) = ((3.0/(_t(i)*_t(i+1)) * ( pow(_t(i),2)*( _p[i+2] - _p[i+1] ) +  pow(_t(i+1),2)*( _p[i+1] - _p[i])))).transpose();
    if(i == 0){ 
      _c.row(i) -= _t(i+1)*_vi;
    }else if(i == _A_size-1){
      _c.row(i) -= _t(i)*_vf;
    }
  }
//   std::cout << "C" << std::endl;
//   std::cout << _c << std::endl;
}


void CubicSpline::computeVelocities(){
  
  Eigen::MatrixXd partial_v = _A.inverse()*_c;
  _v << _vi, partial_v, _vf; 
  
//   std::cout << "V" << std::endl;
//   std::cout << _v << std::endl;
}


void CubicSpline::computeCoefficients(){

  Eigen::MatrixXd vt(2,_num_points);
  vt = _v.transpose();
  
//   std::cout << "coefficients" << std::endl;
  for(int i=0; i < _num_points - 1; i++){
    Coefficients a;

    a.col(0) = _p[i].transpose();
    a.col(1) = vt.col(i);
    a.col(2) = (3.0*( _p[i+1] - _p[i] )/_t(i) - 2.0*vt.col(i) - vt.col(i+1) )/_t(i);
    a.col(3) = (2.0*( _p[i] - _p[i+1] )/_t(i) +   vt.col(i) + vt.col(i+1) )/_t(i);
    
    _coefficients.push_back(a);
    
//     std::cout << a.transpose() << std::endl;
  }
}


void CubicSpline::computeCubicSpline(){
  
//   std::cout << "positions:" << std::endl;
  for(int k=0; k < _num_points-1; k++){
    for(double t = _t(k); t < _t(k)+_t(k+1)+_interval; t +=_interval){
      if(k!=0 && t== _t(k)){ 
	continue;
      }else{
	Coefficients ak = _coefficients[k];
	Vector2 p = ak.col(0) + ak.col(1)*(t- _t(k)) + ak.col(2)*pow(t- _t(k), 2) + ak.col(3)*pow(t- _t(k), 3);
	Vector2 v = ak.col(1) + 2.0*ak.col(2)*(t- _t(k)) + 3.0*ak.col(3)*pow(t- _t(k), 2);
	Vector2 a = 2.0*ak.col(2) + 6.0*ak.col(3)*(t- _t(k));
	
	_spline_p.push_back(p);
	_spline_v.push_back(v);
	_spline_a.push_back(a);
      }
      
//       std::cout << p.transpose() << std::endl;
    }  
  }
}

void CubicSpline::compute(){
  std::cout << "Computing CubicSplines ..."  << std::endl;
//   std::cout << "_interval:" << _interval << std::endl;
  lineSpace();
//   std::cout << "ok lineSpace:"  << std::endl;
  createMatrixA();
//   std::cout << "ok createMatrixA:"  << std::endl;
  createMatrixC();
//   std::cout << "ok createMatrixC:"  << std::endl;
  computeVelocities();
//   std::cout << "ok computeVelocities:"  << std::endl;
  computeCoefficients();
//   std::cout << "ok computeCoefficients:"  << std::endl;
  computeCubicSpline();
  std::cout << "CubicSpline computed"  << std::endl;

}
