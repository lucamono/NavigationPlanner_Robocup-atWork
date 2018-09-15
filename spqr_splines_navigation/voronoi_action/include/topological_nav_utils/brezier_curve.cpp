#include "brezier_curve.h"

BrezierCurve::BrezierCurve(const std::vector< Vector2 >& waypoints_p_,const std::vector<Vector2>& cubic_spline_p_,  const std::vector<Vector2>& cubic_spline_a_, const Vector2& vi_, const Vector2& vf_, double max_vel_, double interval_){
  _waypoints_p = waypoints_p_;
  _num_points = _waypoints_p.size();

  _cubic_spline_p = cubic_spline_p_;
  _cubic_spline_a = cubic_spline_a_;
  _interval = interval_;
  _max_vel = max_vel_;

}


void BrezierCurve::compute(){
   std::cout << "Computing Brezier Curves ..." << std::endl;
//   std::cout << "Computing Path Segments ..." << std::endl;
  computePathSegments();
//   std::cout << "Path Segments Computed" << std::endl << std::endl;

//   std::cout << "Computing Velocities At Waypoints ..." << std::endl;
  computeVelocitiesAtWaypoints();
//   std::cout << "Velocities At Waypoints Computed" << std::endl << std::endl;

//   std::cout << "Computing Accelerations At Waypoints ..." << std::endl;
  computeAccelerationsAtWaypoints();
//   std::cout << "Accelerations At Waypoints Computed" << std::endl << std::endl;

  computeBrezierCurves();
  std::cout << "Brezier Curves Computed" << std::endl;

}


void BrezierCurve::computePathSegments(){

  for(int i=0; i < _waypoints_p.size()-1; i++){
    Segment s_i(_waypoints_p[i], _waypoints_p[i+1]);
    _path_segments.push_back(s_i);
  }

}

void BrezierCurve::computeVelocitiesAtWaypoints(){

  _waypoints_v.push_back(_vi);
  for(int i=0; i < _path_segments.size() - 1; i++){

    Vector2 a(_path_segments[i].first.x()   , _path_segments[i].first.y());
    Vector2 b(_path_segments[i].second.x()  , _path_segments[i].second.y());
    Vector2 c(_path_segments[i+1].second.x(), _path_segments[i+1].second.y());

    Vector2 ab = b-a;
    Vector2 bc = c-b;

    ab /= ab.norm();
    bc /= bc.norm();

//     double theta = ab.transpose()*bc;
    Vector2 tangent = ab+bc;
    tangent *= 0.18;

    _waypoints_v.push_back(tangent);

  }
  _waypoints_v.push_back(_vf);
}

double BrezierCurve::getDistance(Vector2& g1, Vector2& g2){
  return sqrt( pow(g1.x()-g2.x(),2) + pow(g1.y()-g2.y(),2));
}

void BrezierCurve::computeAccelerationsAtWaypoints(){

  int asd = ((_cubic_spline_a.size()-1)/(_num_points-1)) -1;
  for(int i=0; i<_cubic_spline_a.size();i++){
    if(i%asd == 0){
      if(i==0 || i==_cubic_spline_a.size()-1){
	_waypoints_a.push_back(_cubic_spline_a[i]);
      }else{
	Vector2 curr_pos = _cubic_spline_p[i];
	Vector2 prev_pos = _cubic_spline_p[i-asd];
	Vector2 next_pos = _cubic_spline_p[i+asd];

	double prev_curr_dist = getDistance(prev_pos, curr_pos);
	double curr_next_dist = getDistance(curr_pos, next_pos);

	Vector2 a_prev = _cubic_spline_a[i-1];
	Vector2 a_next = _cubic_spline_a[i+1];

	Vector2 acc = curr_next_dist/(prev_curr_dist + curr_next_dist )*a_prev +  prev_curr_dist/(prev_curr_dist + curr_next_dist )*a_next;
	_waypoints_a.push_back(acc);

      }
    }
  }
}

void BrezierCurve::computeBrezierCurves(){
  for(int i=0; i < _path_segments.size(); i++){
    Vector2 p0 = _path_segments[i].first;
    Vector2 p1 = _waypoints_v[i]/5.0 + p0;
    Vector2 p2 = _waypoints_a[i]/20.0 + 2.0*p1 - p0;

    Vector2 p5 = _path_segments[i].second;
    Vector2 p4 = -_waypoints_v[i+1]/5.0 + p5;
    Vector2 p3 = _waypoints_a[i+1]/20.0 + 2.0*p4 - p5;


    for(double u=0; u <= 1; u +=_interval ){
      Vector2 s_p = pow((1-u),5)*p0 + 5*pow((1-u),4)*u*p1 + 10*pow((1-u),3)*pow(u,2)*p2 + 10*pow((1-u),2)*pow(u,3)*p3 + 5*(1-u)*pow(u,4)*p4  + pow(u,5)*p5;
      Vector2 s_v = 5*pow((u-1),4)*(p1-p0) +  20*u*pow((u-1),3)*(p1-p2) + 30*pow(u,2)*pow((u-1),2)*(p3-p2) + 20*pow(u,3)*(u-1)*(p3-p4) + 5*pow(u,4)*(p5-p4);
      Vector2 s_a = 20*pow(u,3)*(p3 - 2*p4 + p5) + 60*pow(u,2)*(u-1)*(2*p3 - p2 - p4) + 60*u*pow((u-1),2)*(p1 - 2*p2 + p3) + 20*pow((u-1),3)*(2*p1 - p0 -p2);

      _brezier_p.push_back(s_p);
      _brezier_v.push_back(s_v);
      _brezier_v_norm.push_back(_max_vel/s_v.norm());
      _brezier_a.push_back(s_a);
    	_brezier_theta.push_back(atan2(s_v(1), s_v(0)));
    }
  }
}

int BrezierCurve::getIndexFromTime(double t){
  int index = round(t/_interval);

  if(index < 0){
    index=0;
  }else if(index >= _brezier_p.size()){
    index = _brezier_p.size()-1;
  }
  return index ;
}



Vector2 BrezierCurve::getPositionAtTime(double t){
  return _brezier_p[getIndexFromTime(t)];
}
double BrezierCurve::getOrientationAtTime(double t){
  return _brezier_theta[getIndexFromTime(t)];
}

Vector2 BrezierCurve::getVelocityAtTime(double t){
  return _brezier_v[getIndexFromTime(t)];
}

double BrezierCurve::getVelocityNormAtTime(double t){
  return _brezier_v_norm[getIndexFromTime(t)];
}

Vector2 BrezierCurve::getAccelerationAtTime(double t){
  return _brezier_a[getIndexFromTime(t)];
}

bool BrezierCurve::trajectoryFinished (double t) {
  int index = round(t/_interval);
  if(index >= _brezier_p.size()){
    return true;
  }
  else
    return false;
}