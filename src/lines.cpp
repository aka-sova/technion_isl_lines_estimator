#include <aligned_def.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <algorithm>
#include <memory>
#include <iomanip>
#include <numeric>
#include <random.h>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <utils.h>
#include <lines2.h>
#include <imgutils.h>
#include <ba/optimization.h>

template<class II>
void correct_points_for_pitch_roll(II b, II e, double roll, double pitch, const Matrix3& K, const Matrix3& Ki)
{
  Matrix3 rz=get_z_rotation_matrix(roll);
  Matrix3 rx=get_x_rotation_matrix(pitch);
  Matrix3 R=rz.transpose()*rx.transpose();
  Matrix3 T=K*R*Ki;
  for (; b != e; ++b)
  {
    Vector2 p= slice(T*extend(*b));
    b->x() = p.x();
    b->y() = p.y();
  }
}


LinesEstimator::LinesEstimator(const cv::Mat& image, const Matrix3& K, cv::Mat& visualization)
: m_Pitch(0)
, m_Roll(0)
, m_Yaw(0)
, m_TranslationDirection(0,0,0)
, m_K(K)
, m_Ki(K.inverse())
{
  int step = 2;
  cxx::add_task([&](){
    find_horz_lines(image,step,m_HLines);
  }, "lines");
  cxx::add_task([&](){
    find_vert_lines(image,step,m_VLines);
  }, "lines");
  cxx::wait_group("lines");
  find_intersections(m_HLines,m_VLines,m_IntersectionPoints);
  m_OrigPoints=m_IntersectionPoints;
  if (visualization.data)
  {
    //std::cout << "Verticals:\n";
    int i=0;
    for(auto& l : m_VLines)
    {
      //std::cout << l.p1.transpose() << "  ->  " << l.p2.transpose() << std::endl;
      cv::line(visualization,cvt(l.p1),cvt(l.p2),cv::Scalar(0,0,255,255),2);
      //cv::putText(visualization,xstring(i),cvt(l.p1),CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0, 255), 2);
      ++i;
    }
    for(auto& l : m_HLines)
    {
      cv::line(visualization,cvt(l.p1),cvt(l.p2),cv::Scalar(255,255,0,255),2);
    }
    
    //show(visualization);
    //cv::imwrite("vlines.png", visualization);
    //std::cout << "-------------------\n";
  }
  estimate_pitch_roll();
  if (visualization.data)
  {
    std::cout << "Pitch=" << m_Pitch << "   Roll=" << m_Roll << std::endl;
    for (auto i : m_VInliers)
    {
      const auto& l = m_VLines[i];
      cv::line(visualization, cvt(l.p1), cvt(l.p2), cv::Scalar(0, 255, 0, 255), 2);
    }
    show(visualization);
  }
  correct_lines_for_pitch_roll(m_VLines);
  correct_lines_for_pitch_roll(m_HLines);
  ::correct_points_for_pitch_roll(m_IntersectionPoints.begin(),m_IntersectionPoints.end(),m_Roll,m_Pitch,m_K,m_Ki);
  estimate_yaw();
  if (visualization.data)
  {
//    for(auto& l : m_VLines)
//      cv::line(visualization,cvt(l.p1),cvt(l.p2),random_bright_color(0),2);
    for(auto& l : m_HLines) 
      cv::line(visualization,cvt(l.p1),cvt(l.p2),random_bright_color(2),2);
    for(auto& p : m_IntersectionPoints)
    {
      cv::circle(visualization,cvt(p),5,cv::Scalar(0,255,0,255),2);
      xstring s="{}, {}";
      s << int(p.theta*180/PI) << p.descriptor;
      text(visualization,p+Vector2(10,10),s,2);
    }
    show(visualization);
  }
}

bool LinesEstimator::estimate_pitch_roll()
{
  int min_line_length = 100;
  m_VInliers.clear();
  for(size_t i=0;i<m_VLines.size();++i)
  {
    const auto& l=m_VLines[i];
    if ((l.p1-l.p2).squaredNorm() > sqr(min_line_length))
      m_VInliers.push_back(int(i));
  }
  return estimate_pitch_roll_ransac();
}

void LinesEstimator::fill_vlines_matrix(Matrix& A, const int_vec& indices)
{
  int row=0;
  for(int i : indices)
  {
    const auto& l=m_VLines[i];
    Vector3 v0=m_Ki*extend(l.p1);
    Vector3 v1=m_Ki*extend(l.p2);
    double a=v0.x()-v1.x();
    double b=v0.x()*v1.y()-v0.y()*v1.x();
    double c=v0.y()-v1.y();
    A.block<1,3>(row++,0)=Vector3(a,b,-c).transpose();
  }
}

bool LinesEstimator::estimate_pitch_roll_ransac()
{
  bool found=false;
  int n = int(m_VInliers.size());
  int_vec alt_inliers,best_inliers;
  Matrix FA(n,3);
  fill_vlines_matrix(FA,m_VInliers);
  int max_count = 0;
  double best_pitch = 0, best_roll = 0;
  for(int iter=0;iter<100;++iter)
  {
    int_vec cand(2);
    select_random_pair(cand[0],cand[1],m_VInliers.begin(),m_VInliers.end());
    Matrix A(2,3);
    fill_vlines_matrix(A,cand);
    Matrix3 P=A.transpose()*A;
    if (!solve_pitch_roll_analytic(P)) continue;
    Vector3 x(cos(m_Pitch),sin(m_Pitch),tan(m_Roll));
    Vector scores=A*x;
    scores = FA*x;
    int count = 0;
    alt_inliers.clear();
    for (int i = 0; i < n; ++i)
    {
      if (fabs(scores(i)) < 0.004)
      {
        ++count;
        alt_inliers.push_back(m_VInliers[i]);
      }
    }
    if (count > max_count)
    {
      A = Matrix(alt_inliers.size(), 3);
      fill_vlines_matrix(A, alt_inliers);
      P = A.transpose()*A;
      if (solve_pitch_roll_analytic(P))
      {
        max_count = count;
        best_pitch = m_Pitch;
        best_roll = m_Roll;
        best_inliers.swap(alt_inliers);
      }
    }
  }
  found = (max_count > 4);
  if (!found) { m_Pitch = m_Roll = 0; }
  else 
  { 
    m_Pitch = best_pitch;
    m_Roll = best_roll; 
    m_VInliers.swap(best_inliers);
  }
  return found;
}

bool LinesEstimator::estimate_pitch_roll_all()
{
  int n=int(m_VInliers.size());
  Matrix A(n,3);
  fill_vlines_matrix(A,m_VInliers);
  Matrix3 P=A.transpose()*A;
  if (!solve_pitch_roll_analytic(P)) return false;
  Vector3 x(cos(m_Pitch),sin(m_Pitch),tan(m_Roll));
  m_Cost=x.transpose()*P*x;
  return true;
}

bool LinesEstimator::solve_pitch_roll_analytic(const Matrix3& P)
{
  double p11=P(0,0),p22=P(1,1),p33=P(2,2),p2=P(0,1),p3=P(0,2),p4=P(1,2);
  double ip33=1.0/p33;
  Matrix2 M; M << p11-sqr(p3)*ip33, p2-p3*p4*ip33, p2-p3*p4*ip33, p22-sqr(p4)*ip33;
  Eigen::EigenSolver<Matrix2> solver(M);
  if (solver.info() == Eigen::ComputationInfo::Success)
  {
    Vector2 vals=solver.eigenvalues().real();
    Matrix2 V=solver.eigenvectors().real();
    Vector2 v=V.block<2,1>(0,vals.x()<vals.y()?0:1);
    m_Pitch=-asin(v.y());
    m_Roll=-atan(-ip33*v.transpose()*Vector2(p3,p4));
    return true;
  }
  m_Pitch=m_Roll=0;
  return false;
}

void LinesEstimator::correct_lines_for_pitch_roll(lines_2d_vec& lines)
{
  Matrix3 rz=get_z_rotation_matrix(m_Roll);
  Matrix3 rx=get_x_rotation_matrix(m_Pitch);
  Matrix3 R=rz.transpose()*rx.transpose();
  Matrix3 T=m_K*R*m_Ki;
  for(auto& l : lines)
  {
    l.p1=slice(T*extend(l.p1));
    l.p2=slice(T*extend(l.p2));
  }
}

void LinesEstimator::correct_point_for_pitch_roll(Vector2& p) const
{
  Vector2* ptr=&p;
  ::correct_points_for_pitch_roll(ptr,ptr+1,m_Roll,m_Pitch,m_K,m_Ki);
}

void LinesEstimator::correct_points_for_pitch_roll(frame_point_vec& points)
{
  ::correct_points_for_pitch_roll(points.begin(),points.end(),m_Roll,m_Pitch,m_K,m_Ki);
}

bool LinesEstimator::estimate_yaw()
{
  for(auto& p : m_IntersectionPoints)
  {
    const auto& horz=m_HLines[p.line_indices[0]];
    Vector2 dp=horz.p2-horz.p1;
    double slope=dp.y() / dp.x();
    Vector3 D=m_Ki * extend(p);
    p.theta = atan((slope * D.z()) / (D.y() - slope * D.x()));
  }
  return true;
}

bool LinesEstimator::estimate_translation_direction()
{
  return false;
}
