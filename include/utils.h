#ifndef utils_h__
#define utils_h__

#include <types.h>
#include <serialization.h>
#include <chrono>
#include "config.h"
#include <cxx/xstring.h>
#include <cxx/threading.h>
#include <statistics.h>

class RandomGenerator
{
  SYNC_MUTEX;
public:
  static RandomGenerator* instance()
  {
    static std::unique_ptr<RandomGenerator> ptr(new RandomGenerator);
    return ptr.get();
  }

  double gen() 
  {
    SYNCHRONIZED;
    return uniform(engine); 
  }

  int    geni(int range) 
  { 
    return int(gen()*range);
  }

  double gen_gaussian(double sigma)
  {
    SYNCHRONIZED;
    return gaussian(engine)*sigma;
  }

  void set_seed(unsigned seed)
  {
    SYNCHRONIZED;
    engine.seed(seed);
  }

  void set_seed()
  {
    SYNCHRONIZED;
    auto t=std::chrono::system_clock::now();
    unsigned s=std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count();
    set_seed(unsigned(s));
  }

private:
  friend struct std::default_delete<RandomGenerator>;
  RandomGenerator() 
  : engine(unsigned(std::chrono::high_resolution_clock::now().time_since_epoch().count())) 
  {}
  ~RandomGenerator() {}
  RandomGenerator(const RandomGenerator&) {}
  RandomGenerator& operator= (const RandomGenerator&) { return *this; }

  std::mt19937 engine;
  std::uniform_real_distribution<double> uniform;
  std::normal_distribution<double> gaussian;
};


inline double R(double s = 2.0f)
{
  return s*(RandomGenerator::instance()->gen() - 0.5f);
}

inline double G(double sigma)
{
  return RandomGenerator::instance()->gen_gaussian(sigma);
}

inline double U(double mx=1.0) { return mx*RandomGenerator::instance()->gen(); }

inline double UH(double range = 1.0) { return U(range) - range * 0.5; }

inline int    UI(int v) { return int(RandomGenerator::instance()->gen()*v); }

inline void   select_random_pair(int& i, int& j, int v)
{
  i=j=0;
  while (i==j || i>=v || j>=v)
  {
    i=UI(v);
    j=UI(v);
  }
}

template<class II>
inline void   select_random_pair(int& i, int& j, II b, II e)
{
  int v=std::distance(b,e);
  select_random_pair(i,j,v);
  i=*(b+i);
  j=*(b+j);
}

inline void SEED(unsigned seed) { RandomGenerator::instance()->set_seed(seed); }

inline cv::Rect cv_rect(const Rect& R) { return cv::Rect(R.l,R.t,R.r-R.l,R.b-R.t); }

inline void show(const cv::Mat image, const xstring& name="vis", double scale=1)
{
  if (scale != 1)
  {
    cv::Mat tmp;
    cv::resize(image, tmp, cv::Size(0, 0), scale, scale,cv::INTER_NEAREST);
    cv::imshow(name, tmp);
  }
  else
  if (image.rows>1000)
  {
    cv::Mat tmp;
    cv::resize(image,tmp,cv::Size(0,0),0.5,0.5);
    cv::imshow(name, tmp);
  }
  else
    cv::imshow(name, image);
  int key=cv::waitKey()&255;
  std::cout << "Key=" << key << std::endl;
  if (key==27) throw xstring("ESCAPE");
}

inline cv::Scalar random_color()
{
  int base = 0, m = 255;
  return cv::Scalar(base + RandomGenerator::instance()->geni(m),
                    base + RandomGenerator::instance()->geni(m),
                    base + RandomGenerator::instance()->geni(m),
                    255);
}

inline cv::Scalar random_bright_color(int main_comp)
{
  int bases[] = {0,0,0};
  if (main_comp>=0 && main_comp<3) bases[main_comp]=255;
  for(int i=0;i<3;++i)
    if (i!=main_comp)
      bases[i]=UI(160);
  return cv::Scalar(bases[0],bases[1],bases[2],255);
}

inline cv::Scalar random_bright_color()
{
  int bases[] = {0,0,0};
  bases[UI(3)]=155;
  int m = 100;
  return cv::Scalar(bases[0] + RandomGenerator::instance()->geni(m),
                    bases[1] + RandomGenerator::instance()->geni(m),
                    bases[2] + RandomGenerator::instance()->geni(m),
                    255);
}

const cv::Scalar CV_BLACK(0, 0, 0, 255);
const cv::Scalar CV_BLUE(255, 0, 0, 255);
const cv::Scalar CV_GREEN(0, 255, 0, 255);
const cv::Scalar CV_RED(0, 0, 255, 255);
const cv::Scalar CV_CYAN(255, 255, 0, 255);
const cv::Scalar CV_WHITE(255, 255, 255, 255);
const cv::Scalar CV_YELLOW(0, 255, 255, 255);

inline cv::Point2f cvt(const Vector2& v) { return cv::Point2f(float(v.x()), float(v.y())); }
inline cv::Point3f cvt(const Vector3& v) { return cv::Point3f(float(v.x()), float(v.y()), float(v.z())); }
inline cv::Point2d CV(const Vector2& v) { return cv::Point2d(v.x(), v.y()); }
inline cv::Point3d CV(const Vector3& v) { return cv::Point3d(v.x(), v.y(), v.z()); }
inline Vector2 cvt(const cv::Point2f& p) { return Vector2(p.x, p.y); }
inline Vector2 slice(const Vector3& v) { return Vector2(v.x()/v.z(), v.y()/v.z()); }
inline Vector2 sliceun(const Vector3& v) { return Vector2(v.x(), v.y()); }
inline Vector3 extend(const Vector2& v) { return Vector3(v.x(), v.y(), 1); }

//template<class D>
//inline D cubed(D d)
//{
//  return d*d*d;
//}

inline void load_matrix(std::istream& is, Matrix3& m)
{
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      is >> m(i, j);
}

inline void load_matrix(std::istream& is, Matrix& m)
{
  for (int i = 0; i < m.rows(); ++i)
  {
    for (int j = 0; j < m.cols(); ++j)
    {
      is >> m(i, j);
    }
  }
}

inline void parse_vector(const xstring& values, std::vector<double>& v)
{
  xstring_tokenizer st(values," ");
  while (st.has_more_tokens()) v.push_back(st.get_next_token().as_double());
}

inline Vector3 parse_vector3(const xstring& values)
{
  std::istringstream is(values);
  Vector3 v;
  for(int i=0;i<3;++i) is >> v(i);
  return v;
}

inline Vector2 parse_vector2(const xstring& values)
{
  std::istringstream is(values);
  Vector2 v;
  for(int i=0;i<2;++i) is >> v(i);
  return v;
}

inline void parse_matrix(const xstring& values, Matrix& m)
{
  std::istringstream is(values);
  load_matrix(is,m);
}

inline void parse_matrix(const xstring& values, Matrix3& m)
{
  std::istringstream is(values);
  load_matrix(is,m);
}

inline void load_matrix(const xstring& filename, Matrix& m)
{
  std::ifstream fin(filename);
  if (!fin)
  {
    std::cout << "Failed to open " << filename << std::endl;
    return;
  }
  load_matrix(fin,m);
}

inline void load_matrix(const xstring& filename, Matrix3& m)
{
  std::ifstream fin(filename);
  if (!fin)
  {
    std::cout << "Failed to open " << filename << std::endl;
    return;
  }
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      fin >> m(i, j);
}

inline void load_strings(std::istream& is, str_vec& v)
{
  xstring line;
  while (line.read_line(is))
  {
    if (line.trim().empty()) continue;
    v.push_back(line);
  }
}

inline void load_strings(const xstring& filename, str_vec& v)
{
  std::ifstream fin(filename);
  if (!fin.fail())
    load_strings(fin,v);
}

inline void write_tracks(const xstring& filename, const str_vec& frame_names, const track_vec& tracks, int min_track_len)
{
  int index = 0;
  std::ofstream fout(filename);
  int track_index = 0;
  for (const auto& t : tracks)
  {
    if (t.size() >= min_track_len)
    {
      fout << "***************************************" << track_index++ << " : " << index << "\n";
      for (const auto& fpi : t)
      {
        ++index;
        fout << frame_names[fpi.camera_index] << ", " << fpi.point.x() << ", " << fpi.point.y() << std::endl;
      }
    }
  }
}

template<class DST, class SRC>
inline void invert_mapping(DST& dst, const SRC& src)
{
  for(auto it=src.begin();it!=src.end();++it)
    dst[it->second]=it->first;
}

class Logger
{
  std::ofstream m_FileOutput;
  std::ostream* m_Output;
public:
  Logger(std::ostream& os) : m_Output(&os) {}
  Logger(const std::string& filename) : m_FileOutput(filename.c_str())
  {
    m_Output = &m_FileOutput;
  }
  Logger() : m_Output(nullptr) {}

  template<class T>
  Logger& operator<< (const T& t)
  {
    if (m_Output)
      (*m_Output) << t;
    return *this;
  }
};

typedef std::shared_ptr<Logger> logger_ptr;


inline bool read_string_sequence(const xstring& filename, std::vector<xstring>& v)
{
  std::ifstream fin(filename);
  if (fin.fail()) return false;
  xstring line;
  while (line.read_line(fin))
    v.push_back(line.trim());
  return true;
}

inline Matrix3 wedge(const Vector3& v)
{
  Matrix3 res;
  res << 0.0,-v.z(),v.y(),
         v.z(),0.0,-v.x(),
         -v.y(),v.x(),0.0;
  return res;
}

inline Matrix3 get_x_derivative_matrix(double a)
{
  Matrix3 R;
  R <<
    0, 0, 0,
    0, -sin(a), cos(a),
    0, -cos(a), -sin(a);
  return R;
}

inline Matrix3 get_x_rotation_matrix(double a)
{
  Matrix3 R;
  R <<
    1, 0, 0,
    0, cos(a), sin(a),
    0, -sin(a), cos(a);
  return R;
}

inline Matrix3 get_y_derivative_matrix(double a)
{
  Matrix3 R;
  R <<
    -sin(a), 0, -cos(a),
    0, 0, 0,
    cos(a), 0, -sin(a);
  return R;
}

inline Matrix3 get_y_rotation_matrix(double a)
{
  Matrix3 R;
  R <<
    cos(a), 0, -sin(a),
    0, 1, 0,
    sin(a), 0, cos(a);
  return R;
}

inline Matrix3 get_z_derivative_matrix(double a)
{
  Matrix3 R;
  R <<
    -sin(a), cos(a), 0,
    -cos(a), -sin(a), 0,
    0, 0, 0;
  return R;
}

inline Matrix3 get_z_rotation_matrix(double a)
{
  Matrix3 R;
  R <<
    cos(a), sin(a), 0,
    -sin(a), cos(a), 0,
    0, 0, 1;
  return R;
}

inline Matrix3 get_axis_derivative_matrix(int axis, const Vector3& a)
{
  switch (axis) {
  case 0: return get_x_derivative_matrix(a(axis));
  case 1: return get_y_derivative_matrix(a(axis));
  case 2: return get_z_derivative_matrix(a(axis));
  }
  return Matrix3::Identity();
}

inline Matrix3 get_axis_rotation_matrix(int axis, const Vector3& a)
{
  switch (axis) {
  case 0: return get_x_rotation_matrix(a(axis));
  case 1: return get_y_rotation_matrix(a(axis));
  case 2: return get_z_rotation_matrix(a(axis));
  }
  return Matrix3::Identity();
}

inline Matrix3 get_rotation_matrix(double rx, double ry, double rz)
{
  Matrix3 RX = get_x_rotation_matrix(rx);
  Matrix3 RY = get_y_rotation_matrix(ry);
  Matrix3 RZ = get_z_rotation_matrix(rz);
  // Composed rotation matrix with (RX, RY, RZ)
  Matrix3 R = RX * RY * RZ;
  return R;
}

inline Matrix3 get_rotation_matrix(const Vector3& a)
{
  return get_rotation_matrix(a(0), a(1), a(2));
}

inline Vector3 extract_euler_angles(const Matrix& R)
{
  const double pi = 3.1415926535897932384626433832795;
  double x = atan2(R(1, 2), R(2, 2));
  double cy = sqrt(sqr(R(0, 0)) + sqr(R(0, 1)));
  double y = atan2(-R(0, 2), cy);
  double sx = sin(x), cx = cos(x);
  double z = atan2(sx*R(2, 0) - cx * R(1, 0), cx*R(1, 1) - sx * R(2, 1));
  return Vector3(x, y, z);
}

inline Matrix extract_euler_angles_old(const Matrix& R)
{
  const double pi = 3.1415926535897932384626433832795;
  Matrix res(3, 1);
  double threshold = 1.0e-5;
  if ((1-abs(R(0,2))) < threshold)
  {
    res(1) = R(0, 2) > 0 ? -0.5*pi : 0.5*pi;
    res(2) = 0;
    res(0) = atan2(-R(2, 1), R(1, 1));
  }
  else
  {
    res(0) = atan2(R(1, 2), R(2, 2));
    double c2 = sqrt(sqr(R(0, 0)) + sqr(R(0, 1)));
    res(1) = atan2(-R(0, 2), c2);
    res(2) = atan2(R(0, 1), R(0, 0));
  }
  return res;
}

inline Matrix get_lookat_rotation_matrix(const Vector3& at, const Vector3& from, const Vector3& up)
{
  Vector3 x,z;
  Matrix rot(3, 3);
  //rot.setIdentity();
  //auto& z=rot.block<3, 1>(0, 2);
  z = (at - from).normalized();
  //auto& x = rot.block<3, 1>(0, 0);
  x = (up.cross(z)).normalized();
  rot.block<3, 1>(0, 0) = x;
  rot.block<3, 1>(0, 1) = (z.cross(x)).normalized();
  rot.block<3, 1>(0, 2) = z;
  return rot.transpose();
}

inline Matrix matrix_3x1(double x, double y, double z)
{
  Matrix t(3, 1);
  t << x, y, z;
  return t;
}

inline Matrix get_translation_matrix(double x, double y, double z)
{
  return matrix_3x1(x, y, z);
}

inline Matrix point_to_matrix(const WorldPoint& p)
{
  return Matrix(p);
  //return matrix_3x1(p.x, p.y, p.z);
}

inline Vector3 matrix_to_point(const Matrix& m)
{
  return Vector3(m(0), m(1), m(2));
}

inline bool valid_frame_point(const FramePoint& fp)
{
  return fp(0) >= 0 && fp(1) >= 0;
}

inline double calc_points_sigma(const world_point_vec& wps, WorldPoint& centroid)
{
  WorldPoint Pc(0, 0, 0);
  for (const auto& P : wps)
    Pc += P;
  Pc = (1.0 / wps.size())*Pc;
  double sum=0;
  for (const auto& P : wps)
  {
    sum += (P - Pc).squaredNorm();
  }
  centroid = Pc;
  return sqrt((1.0 / wps.size())*sum);
}


inline Matrix get_Rt_matrix(Matrix euler, Matrix t)
{
  Matrix R = get_rotation_matrix(euler(0), euler(1), euler(2));
  Matrix Rt(3,4);
  Rt << R , t;
  return Rt;
}

inline void scaled_eye(SparseMatrix& mat, unsigned n, double scale)
{
  SparseMatrixBuilder spb;
  for (unsigned i = 0; i < n; ++i)
    spb.add(i, i, scale);
  spb.generate(mat,true);
}

inline void generate_tracks(
  track_vec& tracks,
  const world_point_vec& world_points,
  const camera_vec& cameras,
  const Matrix& K,
  double fp_noise_sigma)
{
  int empty_tracks = 0;
  int world_point_index = -1;
  Vector vk(3);
  for (const WorldPoint& wp : world_points)
  {
    ++world_point_index;
    Matrix world_point = point_to_matrix(wp); // world_points[world_point_index]
    Track current_track;
    //current_track.world_point_index = world_point_index;
    for (unsigned i = 0; i < cameras.size(); ++i)
    {
      const Camera& c = cameras[i];
      Matrix R = c.get_rotation_matrix();
      Matrix v = R*(world_point - c.t);
      vk <<
        v(0) / v(2),
        v(1) / v(2),
        1;
      Vector projected = K*vk;
      FramePoint fp;
      fp(0) = projected(0);
      fp(1) = projected(1);
      fp += FramePoint(G(fp_noise_sigma), G(fp_noise_sigma));
      if (fp(0) >= 0 && fp(0) < FRAME_WIDTH && fp(1) >= 0 && fp(1) < FRAME_HEIGHT)
      {
        FramePointIndex fpi;
        fpi.camera_index = i;
        fpi.point = fp;
        current_track.push_back(fpi);
      }
    }
    if (!current_track.empty())
    {
      tracks.push_back(current_track);
    }
    else empty_tracks++;
  }
}

inline void extract_positions(const camera_vec& cameras, vector3_vec& positions)
{
  positions.clear();
  for (const auto& c : cameras) positions.push_back(matrix_to_point(c.t));
}

inline void extract_angles(const camera_vec& cameras, vector3_vec& angles)
{
  angles.clear();
  for (const auto& c : cameras)
    angles.push_back(matrix_to_point(extract_euler_angles(c.R)));
}

template<class II>
inline typename II::value_type find_median(II b, II e)
{
  size_t n = std::distance(b, e);
  size_t n2 = n / 2;
  std::vector<typename II::value_type> v(b, e);
  std::nth_element(v.begin(), v.begin() + n2, v.end());
  return v[n2];
}

template<class II>
inline typename II::value_type find_median_inplace(II b, II e)
{
  size_t n = std::distance(b, e);
  size_t n2 = n / 2;
  std::nth_element(b, b + n2, e);
  return *(b + n2);
}



struct Errors
{
  Errors() : avg(0), x(0), y(0), z(0) {}
  double avg, x, y, z;
};

template<class C>
inline Errors analyze_errors(const char* name, const C& truth, const C& result)
{
  unsigned n = result.size();
  double sum_square_error = 0;
  Vector3 axis_sqr_error(0, 0, 0);
  for (unsigned i = 0; i < n; ++i)
  {
    Vector3 diff = result[i] - truth[i];
    sum_square_error += diff.dot(diff);
    for (int j = 0; j < 3; ++j)
      axis_sqr_error(j) += sqr(diff(j));
  }
  Errors e;
  if (n > 0)
  {
    axis_sqr_error = (1.0 / n)*axis_sqr_error;
    //double xe, ye, ze;
    e.x = sqrt(axis_sqr_error(0));
    e.y = sqrt(axis_sqr_error(1));
    e.z = sqrt(axis_sqr_error(2));
    e.avg = sqrt(sum_square_error / n);
    std::cout << name << " average error: " << e.avg << "   " << e.x << ',' << e.y << ',' << e.z << ',' << std::endl;
  }
  return e;
}

void write_scenario(const xstring& filename, const xstring& dtmname, const camera_vec& cams, const track_vec& tracks, const world_point_vec& wps, const Matrix& K);
void load_scenario(const xstring& filename, xstring& dtmname, camera_vec& cams, track_vec& tracks, world_point_vec& wps, Matrix3& K);

inline void write_csv(std::ostream& os, const Matrix& mat)
{
  unsigned w = int(mat.cols()), h = int(mat.rows());
  for (unsigned y = 0; y < h; ++y)
  {
    for (unsigned x = 0; x < w; ++x)
    {
      os << mat(y, x);
      if (x < (w - 1)) os << ',';
    }
    os << std::endl;
  }
}

inline void write_csv(std::ostream& os, const SparseMatrix& smat)
{
  write_csv(os, Matrix(smat));
}

inline double norm(const Vector2& v) { return v.norm(); }

template<class FI>
inline typename FI::value_type geometric_median(FI begin, FI end)
{
  //typedef cv::Point2f P;
  typedef typename FI::value_type P;
  P pt(0, 0);
  int n = 0;
  for (FI it = begin; it != end; ++it, ++n)
    pt += *it;
  if (n == 0) return pt;
  pt *= 1.0f / n;
  bool done = false;
  while (!done)
  {
    P s1(0, 0);
    float s2 = 0;
    for (FI it = begin; it != end; ++it, ++n)
    {
      const auto& p = *it;
      float denom = float(1 / norm(p - pt));
      s1 += denom*p;
      s2 += denom;
    }
    P new_pt = (1.0f / s2)*s1;
    if (norm(new_pt - pt) < 1) done = true;
    pt = new_pt;
  }
  return pt;
}

inline double get_tick_count()
{
  auto tse=std::chrono::system_clock::now().time_since_epoch();
  auto d = std::chrono::duration_cast<std::chrono::milliseconds>(tse);
  return 0.001 * d.count();
}

inline bool exists(const xstring& filename)
{
  std::ifstream f(filename);
  return !f.fail();
}

template<class M>
inline bool read_matrix(const xstring& filename, M& m)
{
  std::ifstream f(filename);
  if (f.fail()) return false;
  int rows = m.rows(), cols = m.cols();
  double v;
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      f >> v;
      m(i, j) = v;
    }
  }
  return true;
}

inline double rcond(const SparseMatrix& m)
{
//   Eigen::EigenSolver<SparseMatrix> es(m);
//   const auto& ev = es.eigenvalues();
  return 0.0;
  //return ev.minCoeff().real() / ev.maxCoeff().real();
//   Eigen::JacobiSVD<SparseMatrix> svd(m);
//   const auto& sv = svd.singularValues();
//   return sv(sv.size() - 1) / sv(0);
}

template<class C>
inline void erase_if(C& container, std::function<bool(typename C::value_type)> pred)
{
  container.erase(std::remove_if(container.begin(),container.end(),pred),container.end());
}

inline void text(cv::Mat& image, int x, int y, const xstring& s, 
                 double size=1, cv::Scalar color=cv::Scalar(0,255,0,255),
                 int thickness=2)
{
  cv::putText(image,s,cv::Point(x,y),cv::FONT_HERSHEY_PLAIN,size,color,thickness);
}

inline void text(cv::Mat& image, const Vector2& p, const xstring& s, 
                 double size=1, cv::Scalar color=cv::Scalar(0,255,0,255),
                 int thickness=2)
{
  text(image,p.x(),p.y(),s,size,color,thickness);
}

inline std::string format_matlab_matrix(const Matrix& mat)
{
  int n = int(mat.rows()), m = int(mat.cols());
  std::ostringstream os;
  os << "[";
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < m; ++j)
    {
      double d = mat(i, j);
      os << d;
      if (j<(m-1)) os << ',';
    }
    if (i < (n - 1))
      os << ";";
  }
  os << "]";
  return os.str();
}


#endif // utils_h__
