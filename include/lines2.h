#pragma once

#include <types.h>
#include <lines.h>

class LinesEstimator
{
  lines_2d_vec m_VLines,m_HLines;
  int_vec      m_VInliers;
  ip_vec       m_IntersectionPoints;
  ip_vec       m_OrigPoints;
  double m_Pitch,m_Roll,m_Yaw,m_Cost;
  Vector3 m_TranslationDirection;
  Matrix3 m_K,m_Ki;

  void fill_vlines_matrix(Matrix& A, const int_vec& indices);

  bool estimate_pitch_roll_all();
  bool estimate_pitch_roll_ransac();

  bool estimate_pitch_roll();
  bool estimate_yaw();
  bool estimate_translation_direction();
  
  bool solve_pitch_roll_analytic(const Matrix3& P);
  void solve_pitch_roll_opt(const Matrix3& P);
public:
  LinesEstimator(const cv::Mat& image, const Matrix3& K, cv::Mat& visualization);
  
  void correct_point_for_pitch_roll(Vector2& p) const;
  void correct_points_for_pitch_roll(frame_point_vec& points);
  void correct_lines_for_pitch_roll(lines_2d_vec& lines);
  
  typedef typename ip_vec::const_iterator const_iterator;
  const_iterator begin() const { return m_IntersectionPoints.begin(); }
  const_iterator end()   const { return m_IntersectionPoints.end();   }
  const_iterator orig_begin() const { return m_OrigPoints.begin(); }
  const_iterator orig_end()   const { return m_OrigPoints.end();   }

  double  get_cost()  const { return m_Cost; }
  double  get_pitch() const { return m_Pitch; }
  double  get_roll()  const { return m_Roll; }
  double  get_yaw()   const { return m_Yaw; }
  Vector3 get_direction() const { return m_TranslationDirection; }
};


