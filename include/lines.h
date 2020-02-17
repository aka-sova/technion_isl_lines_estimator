#ifndef lines_h__
#define lines_h__

#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <types.h>

template<class POINT>
struct EndPointsLine
{
  POINT p1, p2;
  POINT dir;
  POINT normal;
  double contrast;
  double origin_distance;
  double aligned_distance;

  EndPointsLine(const POINT& _p1, const POINT& _p2, double c=0)
  : p1(_p1)
  , p2(_p2)
  , normal(POINT::Zero())
  , contrast(c)
  , origin_distance(0)
  {
    dir = p2 - p1;
    // Normalize line so that in the longest axis, p1 < p2
    int idx;
    dir.cwiseAbs().maxCoeff(&idx);
    if (dir(idx) < 0)
    {
      std::swap(p1, p2);
      dir = -dir;
    }
    dir.normalize();
    normal=POINT(-dir.y(),dir.x());
    origin_distance=p1.dot(normal);
    aligned_distance = 0.5*(p1.x() + p2.x());
    if (fabs(normal.y()) > fabs(normal.x()))
      aligned_distance = 0.5*(p1.y() + p2.y());
  }

  double length() const { return (p2 - p1).norm(); }
  double dot(const EndPointsLine& rhs) const { return dir.dot(rhs.dir); }
  POINT center() const { return 0.5*(p1 + p2); }
  bool is_in_front(const POINT& p) const
  {
    double d1 = (p - p1).norm();
    double d2 = (p - p2).norm();
    double len = length();
    return d1 < len && d2 < len;
  }
  double distance_from_endpoint(const POINT& p) const
  {
    return Min((p - p1).norm(), (p - p2).norm());
  }

  double cos_angle(const EndPointsLine<POINT>& other) const
  {
    return dir.dot(other.dir);
  }
};

typedef EndPointsLine<Vector2> Line2D;
typedef EndPointsLine<Vector3> Line3D;
typedef std::vector<Line2D,Eigen::aligned_allocator<Line2D>> lines_2d_vec;

struct IntersectionPoint : public Vector2
{
  IntersectionPoint(const Vector2& p = Vector2(0, 0), int l1 = -1, int l2 = -1)
  : Vector2(p)
  , theta(0)
  , descriptor(0)
  {
    line_indices[0] = l1;
    line_indices[1] = l2;
  }
  
  void calculate_descriptor(const lines_2d_vec& hlines, const lines_2d_vec& vlines)
  {
    const auto& hl=hlines[line_indices[0]];
    const auto& vl=vlines[line_indices[1]];
    descriptor=0;
    double h1=(*this - hl.p1).squaredNorm(), h2=(*this - hl.p2).squaredNorm();
    double v1=(*this - vl.p1).squaredNorm(), v2=(*this - vl.p2).squaredNorm();
    if (h2 < h1) descriptor |= 1;
    if (v2 < v1) descriptor |= 2;
  }
  
  int     line_indices[2];
  double  theta;
  int     descriptor;
};

inline bool ip_by_xy(const IntersectionPoint& a, const IntersectionPoint& b)
{
  double dx = b.x() - a.x();
  if (fabs(dx) > 10) return dx > 0;
  return a.y() < b.y();
  //return a.x()<b.x();
}

typedef std::unordered_map<int, int> matches_mapping;
typedef std::pair<Vector2, Vector2> pt_pair;
typedef std::vector<pt_pair,Eigen::aligned_allocator<Vector2>> pt_pair_vec;

typedef std::vector<IntersectionPoint, Eigen::aligned_allocator<Vector2>> ip_vec;

typedef std::vector<cv::Point> path;
typedef vector2_vec dpath;

int  find_vert_lines(const cv::Mat& image, int step, lines_2d_vec& lines);
int  find_horz_lines(const cv::Mat& image, int step, lines_2d_vec& lines);
void find_intersections(const lines_2d_vec& hlines, const lines_2d_vec& vlines, ip_vec& points);

void gaussian_blur(const cv::Mat& src, cv::Mat& dst);
cv::Mat find_image_lines(const cv::Mat& image, lines_2d_vec& lines, int min_len);
void match_lines(lines_2d_vec& lines1, lines_2d_vec& lines2, matches_mapping& matches);
void match_lines_intersection(lines_2d_vec& lines1, lines_2d_vec& lines2, 
                              matches_mapping& matches, pt_pair_vec& pts);
void find_line_intersections(const lines_2d_vec& lines, ip_vec& pts, const Matrix3& Ki);
void remove_ambiguous_intersections(ip_vec& pts);

void match_intersection_points(const lines_2d_vec& lines1, const ip_vec& pts1,
                               const lines_2d_vec& lines2, const ip_vec& pts2,
                               matches_mapping& matches);

void correct_pitch_roll(const Matrix& K, lines_2d_vec& lines, double* pitch=0, double* roll=0);

Line2D estimate_line(const path& points);
Line2D estimate_line(const dpath& points);

inline void filter_vertical_lines(lines_2d_vec& lines)
{
  lines.erase(std::remove_if(lines.begin(),lines.end(),[](const Line2D& l){
    Vector2 d=l.p2-l.p1;
    double ady=fabs(d.y());
    double adx=fabs(d.x());
    return ady<200 || adx>=ady;
  }),lines.end());
}

inline void filter_vertical_lines(lines_2d_vec& lines, lines_2d_vec& vert_lines, lines_2d_vec& horiz_lines)
{
  auto it=std::remove_if(lines.begin(),lines.end(),[](const Line2D& l){
    Vector2 d=l.p2-l.p1;
    double ady=fabs(d.y());
    double adx=fabs(d.x());
    return ady<200 || adx>=ady;
  });
  vert_lines.assign(lines.begin(),it);
  horiz_lines.assign(it,lines.end());
  horiz_lines.erase(std::remove_if(horiz_lines.begin(),horiz_lines.end(),[](const Line2D& l){
    Vector2 d=l.p2-l.p1;
    return fabs(d.x())<100;
  }),horiz_lines.end());
}

#endif // lines_h__
