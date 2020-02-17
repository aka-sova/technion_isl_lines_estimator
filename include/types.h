#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <random>
#include <memory>
#include <string>
#include <sstream>
#include <prims.h>
#include <cxx/xstring.h>
using cxx::xstring;
using cxx::xstring_tokenizer;
#include <aligned_def.h>
#include <aligned.h>

#include <opencv2/opencv.hpp>
#ifdef MKL
#define EIGEN_USE_MKL_ALL
#endif
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>
#include <Eigen/StdVector>
#include <Eigen/QR>
#include <Eigen/LU>
#ifdef MKL
#include <Eigen/PardisoSupport>
#endif
#include <opencv2/core/eigen.hpp>

typedef Eigen::Vector2d FramePoint;
typedef Eigen::Vector3d WorldPoint;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Matrix2d Matrix2;
typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector2i iVector2;
typedef Eigen::VectorXcd cVector;

struct Transpose {};
const Transpose T;

inline Matrix3 operator^ (const Matrix3& m, const Transpose& t)
{
  return m.transpose();
}

typedef Eigen::MatrixXd Matrix;
typedef Eigen::SparseMatrix<double> SparseMatrix;
typedef Eigen::VectorXd Vector;
//typedef cv::Scalar_<double> Scalar;
typedef std::vector<FramePoint,Eigen::aligned_allocator<FramePoint>> frame_point_vec;
typedef std::vector<WorldPoint,Eigen::aligned_allocator<WorldPoint>> world_point_vec;
typedef std::vector<Matrix, Eigen::aligned_allocator<Matrix>> matrix_vec;
typedef std::vector<Matrix3, Eigen::aligned_allocator<Matrix3>> matrix3_vec;
typedef std::vector<Vector3, Eigen::aligned_allocator<Vector3>> vector3_vec;
typedef std::vector<Vector2, Eigen::aligned_allocator<Vector2>> vector2_vec;
typedef std::vector<int> int_vec;
typedef std::set<int> int_set;
typedef std::map<int,int> int_map;
typedef std::set<xstring> str_set;
typedef std::vector<int_vec> int_vec_vec;
typedef std::vector<bool> bool_vec;
typedef std::vector<double> dvec;
typedef std::vector<float> fvec;
typedef std::vector<float, aligned_allocator<float, 32>> aligned_fvec;
typedef std::vector<xstring> str_vec;
typedef std::vector<uint8_t> status_vec;

typedef std::vector<cv::Point2f> pt_vec;

typedef std::map<xstring,int> frame_index_mapping;

/// A 2D point in a frame, and the camera index from which it is taken.
struct FramePointIndex
{
  FramePointIndex(const FramePoint& fp=FramePoint(), int cam_idx=0)
    : point(fp)
    , camera_index(cam_idx)
  {}

  FramePointIndex(const cv::Point2f& p, int cam_idx)
    : point(p.x, p.y)
    , camera_index(cam_idx)
  {}

  FramePoint point;
  int        camera_index;
  double     conf;
};

struct TrackPointIndex
{
  TrackPointIndex(const FramePoint& fp=FramePoint(), int wp_idx=0)
    : point(fp)
    , point_index(wp_idx)
  {}

  TrackPointIndex(const cv::Point2f& p, int wp_idx)
    : point(p.x, p.y)
    , point_index(wp_idx)
  {}

  FramePoint point;
  int        point_index;
};

typedef std::vector<TrackPointIndex,Eigen::aligned_allocator<TrackPointIndex>> track_point_vec;

/// A sequence of frame points that make a track of the movement of a world point in the
/// various cameras views.

struct Track : public std::vector<FramePointIndex,Eigen::aligned_allocator<FramePointIndex>>
{
  //int world_point_index; // Used for ground truth comparison
  xstring cams_str;

  void calculate_cams_str()
  {
    cams_str = "";
    for (const FramePointIndex& fpi : *this)
      cams_str << ' ' << fpi.camera_index;
  }
};

typedef std::vector<Track, Eigen::aligned_allocator<Track>> track_vec;
typedef std::vector<track_point_vec> cam_points;

inline bool load_track_file(const xstring& filename, track_vec& tracks, frame_index_mapping& frames)
{
  std::ifstream f(filename);
  if (f.fail()) return false;
  tracks.clear();
  xstring line;
  while (line.read_line(f))
  {
    line.trim();
    if (line.empty()) continue;
    if (line.startswith("*"))
    {
      tracks.push_back(Track());
      continue;
    }
    Track& t = tracks.back();
    xstring_tokenizer st(line, ",");
    xstring name = st.get_next_token();
    auto it = frames.find(name);
    if (it == frames.end())
    {
      int index=int(frames.size());
      frames[name]=index;
      it = frames.find(name);
    }
    FramePointIndex fpi;
    fpi.camera_index = it->second;
    fpi.point.x() = st.get_next_token().as_double();
    fpi.point.y() = st.get_next_token().as_double();
    t.push_back(fpi);
  }
  for(auto& track : tracks)
    std::stable_sort(track.begin(),track.end(),[](const FramePointIndex& a, const FramePointIndex& b){
      return a.camera_index < b.camera_index;
    });
  std::stable_sort(tracks.begin(),tracks.end(),[](const Track& a, const Track& b) {
    return a.front().camera_index < b.front().camera_index;
  });
  return true;
}

inline void invert_tracks(const track_vec& tracks, cam_points& ctracks)
{
  int max_cam_index=0;
  for(const auto& track : tracks)
    for(const auto& fpi : track)
      max_cam_index=Max(max_cam_index,fpi.camera_index);
  ctracks.assign(max_cam_index+1,track_point_vec());
  int wp_idx=0;
  for(const auto& track : tracks)
  {
    for(const auto& fpi : track)
      ctracks[fpi.camera_index].push_back(TrackPointIndex(fpi.point,wp_idx));
    ++wp_idx;
  }
}

Matrix3 get_rotation_matrix(double rx, double ry, double rz);
Matrix3 get_rotation_matrix(const Vector3& angles);

struct Camera
{
  Camera()
    : R(3, 3)
    , t(3, 1)
  {
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    t << 0, 0, 0;
  }
  
  Camera(const Vector3& pos, const Matrix3& rot=Matrix3::Identity())
  : t(pos)
  , R(rot)
  {}

  /// R - is the 3x3 Rotation matrix
  /// t - is the 3x1 translation matrix
  Matrix3 R;
  Vector3 t;

  void apply_delta_rotation(const Matrix& delta)
  {
    R = ::get_rotation_matrix(delta) * R;
  }

  const Matrix3& get_rotation_matrix() const
  {
    //return ::get_rotation_matrix(EulerAngs(0), EulerAngs(1), EulerAngs(2));
    return R;
  }

  Matrix calculate_projection_matrix(const Matrix& K) const
  {
    Matrix P(3, 4);
    P.block<3, 3>(0, 0) = K*R;
    P.block<3, 1>(0, 3) = -K*R*t;
    return P;
  }

  Vector3 project_homogeneous(const Vector3& X, const Matrix3& W2C) const 
  {
    Vector3 v = R*W2C*(X - t);
    return (1.0 / v.z())*v;
  }

  Vector3 project_homogeneous(const Vector3& X) const 
  {
    Vector3 v = R*(X - t);
    return (1.0 / v.z())*v;
  }

  Vector3 project(const Matrix& K, const Vector3& X, const Matrix3& W2C) const
  {
    return K*project_homogeneous(X,W2C);
  }

  Vector3 project(const Matrix& K, const Vector3& X) const
  {
    return K*project_homogeneous(X);
  }

  Vector3 unproject(const Matrix3& Ki, const FramePoint& fp) const
  {
    Vector3 p(fp.x(), fp.y(), 1.0);
    return (R.transpose()*Ki*p).normalized();
  }

  Vector3 unproject(const Matrix3& W2C, const Matrix3& Ki, const FramePoint& fp) const
  {
    Vector3 p(fp.x(), fp.y(), 1.0);
    return (W2C.transpose()*R.transpose()*Ki*p).normalized();
  }

};

typedef std::vector<Camera,Eigen::aligned_allocator<Camera>> camera_vec;

struct ResultPoint
{
  int world_point_index; // Used for comparison with ground truth
  WorldPoint estimated_point;
};

typedef std::vector<ResultPoint, Eigen::aligned_allocator<ResultPoint>> results_vec;


class SparseMatrixBuilder
{
  typedef Eigen::Triplet<double> triplet;
  typedef std::vector<triplet> triplets_seq;
  triplets_seq m_Data;
  unsigned m_Width, m_Height;
public:
  typedef SparseMatrixBuilder self;
  typedef triplets_seq::const_iterator const_iterator;
  const_iterator begin() const { return m_Data.begin(); }
  const_iterator end()   const { return m_Data.end(); }

  SparseMatrixBuilder(unsigned initial_capacity=0)
    : m_Width(0), m_Height(0)
  {
    if (initial_capacity > 0)
      m_Data.reserve(initial_capacity);
  }

  void set_minimum_size(unsigned width, unsigned height)
  {
    m_Width = Max(m_Width, width);
    m_Height = Max(m_Height, height);
  }
  
  unsigned get_width() const { return m_Width; }
  unsigned get_height() const { return m_Height; }

  self& operator()(unsigned row, unsigned col, double v)
  {
    add(row, col, v);
    return *this;
  }

  self& operator() (unsigned row, unsigned col, const Matrix& values)
  {
    for (unsigned i = 0; i < unsigned(values.rows()); ++i)
    {
      const auto& row_values = values.row(i);
      for (unsigned j = 0; j < unsigned(values.cols()); ++j)
      {
        m_Height = Max(m_Height, i + row + 1);
        m_Width = Max(m_Width, j + col + 1);
        m_Data.push_back(triplet(i + row, j + col, row_values(j)));
      }
    }
    return *this;
  }

  void add(unsigned row, unsigned col, double value)
  {
    m_Height = Max(m_Height, row + 1);
    m_Width = Max(m_Width, col + 1);
    m_Data.push_back(triplet(row, col, value));
  }

  void generate(SparseMatrix& matrix, bool size_to_fit)
  {
    if (size_to_fit)
      matrix = SparseMatrix(m_Height, m_Width);
    matrix.setFromTriplets(m_Data.begin(), m_Data.end());
  }
};

class SparseMatrixBuilderStream
{
  SparseMatrixBuilder& m_Builder;
  unsigned             m_Row, m_Col, m_Width, m_Current;

  void append(double value)
  {
    m_Builder.add(m_Row, m_Col + m_Current, value);
    if (++m_Current == m_Width)
    {
      m_Current = 0;
      ++m_Row;
    }
  }
public:
  SparseMatrixBuilderStream(SparseMatrixBuilder& smb, unsigned row, unsigned col, unsigned width)
  : m_Builder(smb) 
  , m_Row(row)
  , m_Col(col)
  , m_Width(width)
  , m_Current(0)
  {}

  SparseMatrixBuilderStream& operator<< (double value)
  {
    append(value);
    return *this;
  }

  SparseMatrixBuilderStream& operator, (double value)
  {
    append(value);
    return *this;
  }
};

class Ray
{
  Vector3 m_Origin,m_Direction,m_InverseDirection;
  bool    m_Degenerate[3];
public:
  Ray(const Vector3& O, const Vector3& D)
  : m_Origin(O)
  , m_Direction(D)
  {
    for(int i=0;i<3;++i)
    {
      m_Degenerate[i]=false;
      if (fabs(D(i))<1e-14) 
      {
        m_InverseDirection(i)=D(i)>0?1e300:-1e300;
        m_Degenerate[i]=true;
      }
      else
        m_InverseDirection(i)=1.0/D(i);
    }
  }
  
  const Vector3& get_origin() const { return m_Origin; }
  const Vector3& get_direction() const { return m_Direction; }
  
  double origin(int axe) const { return m_Origin(axe); }
  double project(const Vector3& P) const { return P.dot(m_Direction)-m_Origin.dot(m_Direction); }
  
  Vector3 calc_point(double t) const
  {
    return m_Origin+t*m_Direction;
  }
  
  Vector3 calculate(double t) const { return calc_point(t); }
  
  bool calct(int axe, double value, double& res) const
  {
    if (m_Degenerate[axe]) return false;
    res=(value-m_Origin(axe))*m_InverseDirection(axe);
    return true;
  }
};

/*
struct Ray
{
  Ray(const Vector3& origin=Vector3(0,0,0), const Vector3& dir=Vector3(1,0,0))
  : O(origin), D(dir) {}
  Vector3 O,D;
  
  Vector3 calculate(double t) const { return O+t*D; }
  double  project(const Vector3& P) const { return P.dot(D)-O.dot(D); }

  bool calct(int axe, double value, double& res) const
  {
    if (m_Degenerate[axe]) return false;
    res=(value-m_Origin(axe))*m_InverseDirection(axe);
    return true;
  }
};
*/

typedef std::vector<Ray,Eigen::aligned_allocator<Ray>> ray_vec;

struct Plane
{
  Plane(const Vector3& n=Vector3(0,0,0), double d=0) : N(n), d(d) {}
  Plane(const Vector3& a, const Vector3& b, const Vector3& c)
  {
    N = (b - a).cross(c - b);
    N.normalize();
    d = a.dot(N);
  }
  Vector3 N;
  double  d;

  Plane& invert()
  {
    N = -N;
    d = -d;
    return *this;
  }
  
  double distance(const Vector3& p) const
  {
    return p.dot(N)-d;
  }
};

typedef std::vector<Plane,Eigen::aligned_allocator<Plane>> plane_vec;

struct Wall
{
  Plane   plane;
  Vector3 center;
  Vector3 size;
  
  double distance(const Vector3& P) const
  {
    double d1 = P.transpose()*plane.N-plane.d;
    Vector3 P1=P-d1*plane.N;
    d1=sqr(d1);
    Vector3 d=(P1-center).cwiseAbs() - size;
    for(int i=0;i<3;++i)
      if (d(i)>0) d1+=sqr(d(i));
    return sqrt(d1);
  }
};

typedef std::vector<Wall,Eigen::aligned_allocator<Wall>> wall_vec;


class Model3D
{
public:
  virtual ~Model3D() {}

  /** Find the intersection of a ray from camera 'c' towards point 'P'
      and set the intersection point 'Pg' and the plane's N,d
      Return the plane index (0..n-1)
  */
  virtual int intersect(const Camera& c, const Vector3& P,
                        Vector3& Pg, Vector3& N, double& d) const = 0;

  virtual void intersect_plane(int index, const Camera& c, const Vector3& P,
                               Vector3& Pg, Vector3& N, double& d) const = 0;

  /** Find the nearest plane and return its index */
  virtual int nearest(const Vector3& P) const = 0;

  /** Query a plane's parameters.  Returns true if successful */
  virtual bool get_plane(int index, Vector3& N, double& d) const = 0;

  virtual int get_number_of_planes() const = 0;
};

inline std::string format_vector(const Vector& v)
{
  size_t n = v.size();
  std::ostringstream os;
  for (size_t i = 0; i < n; ++i)
    os << v(i) << " ";
  return os.str();
}

inline std::string format_matrix(const SparseMatrix& smat)
{
  Matrix mat(smat);
  int n = int(mat.rows()), m = int(mat.cols());
  std::ostringstream os;
  for (int i = 0; i < n;++i)
  {
    for (int j = 0; j < m;++j)
    {
      double d=mat(i, j);
//       std::ostringstream dos;
//       dos.precision(2);
//       dos << std::fixed << d;
//       std::string s = dos.str();
//       if (s.length()>8)
//       {
//         int p = int(s.find('.'));
//         s = s.substr(0, p);
//       }
//       while (s.length() < 9) s = ' ' + s;
//       os << s;
      os << d << ' ';
    }
    os << ";" << std::endl;
  }
  return os.str();
}

inline void compact_tracks(track_vec& tracks)
{
  track_vec alt;
  for (auto& t : tracks)
  {
    if (!t.empty())
    {
      //int wpi = alt.size();
      alt.push_back(Track());
      alt.back().swap(t);
      //alt.back().world_point_index = wpi;
      alt.back().calculate_cams_str();
    }
  }
  tracks.swap(alt);
}

class Line
{
  Vector3 m_Coefs;

  void normalize()
  {
    double denom = 1.0 / sqrt(sqr(m_Coefs(0)) + sqr(m_Coefs(1)));
    m_Coefs = denom*m_Coefs;
  }
public:
  Line() {}
  Line(const Vector3& v)
    : m_Coefs(v)
  {
    normalize();
  }
  
  Line(const cv::Point& p1, const cv::Point& p2)
  {
    m_Coefs=Vector3(p1.x,p1.y,1).cross(Vector3(p2.x,p2.y,1));
    normalize();
  }

  Line(const Vector2& p1, const Vector2& p2)
  {
    m_Coefs = Vector3(p1.x(), p1.y(), 1).cross(Vector3(p2.x(), p2.y(), 1));
    normalize();
  }

  Line(const Vector3& p1, const Vector3& p2)
    : m_Coefs(p1.cross(p2))
  {
    normalize();
  }

  Vector3 intersection(const Line& other) const
  {
    Vector3 v= m_Coefs.cross(other.m_Coefs);
    return (1.0 / v(2))*v;
  }

  double distance(const cv::Point& p) const
  {
    return fabs(p.x*m_Coefs.x() + p.y*m_Coefs.y() + m_Coefs.z());
  }

  double distance(const Vector2& p) const
  {
    return fabs(p.x()*m_Coefs.x() + p.y()*m_Coefs.y() + m_Coefs.z());
  }
};

typedef std::vector<Line, Eigen::aligned_allocator<Vector3>> line_vec;

struct null_output_iterator :
  std::iterator< std::output_iterator_tag,
  null_output_iterator > {
  /* no-op assignment */
  template<typename T>
  void operator=(T const&) { }

  null_output_iterator & operator++() {
    return *this;
  }

  null_output_iterator operator++(int) {
    return *this;
  }

  null_output_iterator & operator*() { return *this; }
};

template<class T, unsigned N>
class HistoryCyclicBuffer
{
  T        m_Buffer[N];
  unsigned m_Current;
  
  unsigned index(unsigned i) { return ((i+m_Current) % N); }
public:
  HistoryCyclicBuffer() : m_Current(0) {}
  T& operator[] (unsigned i) { return m_Buffer[index(i)]; }
  const T& operator[] (unsigned i) const { return m_Buffer[index(i)]; }
  T& front() { return m_Buffer[m_Current]; }
  const T& front() const { return m_Buffer[m_Current]; }
  void advance() { m_Current=(m_Current==0?N-1:m_Current-1); }
};


//template<>
inline Vector3 Min(const Vector3& a, const Vector3& b)
{
  return Vector3(Min(a.x(),b.x()),Min(a.y(),b.y()),Min(a.z(),b.z()));
}

//template<>
inline Vector3 Max(const Vector3& a, const Vector3& b)
{
  return Vector3(Max(a.x(),b.x()),Max(a.y(),b.y()),Max(a.z(),b.z()));
}


template<class T>
struct MinMax
{
  bool init;
  T mn,mx;
  MinMax() 
  : init(true)
  , mn(std::numeric_limits<T>::max())
  , mx(std::numeric_limits<T>::min())
  {}
  MinMax<T>& operator() (const T& t)
  {
    if (init) { init=false; mn=mx=t; }
    else
    {
      mn=Min(mn,t);
      mx=Max(mx,t);
    }
    return *this;
  }
  T diff() const { return mx - mn; }
};


/// Axis aligned bounding box
struct AABB
{
  Vector3 mn,mx;

  template<class Archive>
  void serialize(Archive& ar, unsigned int version)
  {
    ar & mn;
    ar & mx;
  }

  AABB() : mn(9e9,9e9,9e9), mx(-9e9,-9e9,-9e9) {}
  AABB(const Vector3& min_coords, const Vector3& max_coords)
  : mn(min_coords)
  , mx(max_coords)
  {}
  
  void contain_point(const Vector3& p)
  {
    for(int i=0;i<3;++i)
    {
      mn(i)=Min(mn(i),p(i));
      mx(i)=Max(mx(i),p(i));
    }
  }
  
  void unite(const AABB& box)
  {
    contain_point(box.mn);
    contain_point(box.mx);
  }
  
  bool does_intersect(const Ray& r) const
  {
    Vector3 entry_point,exit_point;
    double tmin,tmax;
    return find_intersection(r,entry_point,tmin,exit_point,tmax);
  }
  
  bool find_intersection(const Ray& r, 
                         Vector3& entry_point, double& tmin,
                         Vector3& exit_point, double& tmax) const
  {
    tmin = -9e9;
    tmax = 9e9;
    vector2_vec intervals(3,Vector2(0,0));
    //Vector3 entry_t,exit_t;
    for(int axe=0;axe<3;++axe)
    {
      double v1=0,v2=0;
      if (!r.calct(axe,mn(axe),v1))
      {
        if (r.origin(axe)<mn(axe) || r.origin(axe)>mx(axe)) return false;
      }
      else
      {
        r.calct(axe, mx(axe), v2);
        if (v1 > v2) std::swap(v1, v2);
        intervals[axe] = Vector2(v1, v2);
        
        //if (v1 > 0)
          tmin = Max(tmin, v1);
        //if (v2 > 0)
          tmax = Min(tmax, v2);
        
        //if (exit_t(axe)<entry_t(axe))
        //  std::swap(exit_t(axe),entry_t(axe));
      }
    }
    //tmin=Max(entry_t(0),Max(entry_t(1),entry_t(2)));
    //tmax=Min(exit_t(0),Min(exit_t(1),exit_t(2)));
    if (tmax<=tmin || tmax<0) return false;
    entry_point=r.calc_point(tmin);
    exit_point=r.calc_point(tmax);
    return true;
  }
  
  /// Verify whether a ray intersects the box
  bool check_intersection(const Ray& r)
  {
    Vector3 v1,v2;
    double tmin,tmax;
    return find_intersection(r,v1,tmin,v2,tmax);
  }
};

inline std::ostream& operator<<(std::ostream& os, const AABB& box)
{
  return os << box.mn.transpose() << " : " << box.mx.transpose();
}

