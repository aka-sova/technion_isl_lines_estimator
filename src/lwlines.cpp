#include <aligned_def.h>
#include <utils.h>
#include <imgutils.h>
#include <lines.h>
#include <lines2.h>
#include <cxx/equiv.h>

static inline double calc_score(const dvec& errs)
{
  double sum = 0;
  for (const auto& e : errs)
    sum += 1.0 / (1.0 + e);
  return sum;
}

static Vector3 estimate_random_line(const vector3_vec& pts)
{
  int i=0,j=0;
  double x1, y1, x2, y2;
  for (int iter = 0; iter < 10; ++iter)
  {
    select_random_pair(i, j, int(pts.size()));
    x1 = pts[i].x();
    y1 = pts[i].y();
    x2 = pts[j].x();
    y2 = pts[j].y();
    if (fabs(x1 - x2) > 50 || fabs(y1 - y2) > 50) break;
  }
  double b=(1-x2/x1)/(y2-y1*(x2/x1));
  double a=(1-b*y1)/x1;
  double nrm=sqrt(sqr(a)+sqr(b));
  return (1.0/nrm)*Vector3(a,b,-1);
}

static Vector3 estimate_ls_line(const vector3_vec& pts, const bool_vec& mask)
{
  int n=std::count(mask.begin(),mask.end(),true);
  if (n<3)
  {
    static Vector3 def(0.5*sqrt(2.0),0.5*sqrt(2.0),0);
    return def;
  }
  Matrix A(n,3);
  int i=0,j=0;
  for(const auto& p : pts)
  {
    if (mask[i++])
    {
      A(j,0)=p.x();
      A(j,1)=p.y();
      A(j,2)=1;
      ++j;
    }
  }
  auto svd=A.jacobiSvd(Eigen::ComputeThinV);
  Vector3 L=svd.matrixV().block<3,1>(0,2);
  double nrm=sqrt(sqr(L.x())+sqr(L.y()));
  return (1.0/nrm)*L;
}

static double estimate_line(const vector3_vec& pts, Vector2& p1, Vector2& p2, double estimated_inlier_ratio=0)
{
  size_t n=pts.size();
  bool_vec mask(n,true);
  dvec errs(n),serrs(n);
  
  double max_score=0;
  Vector3 best;
  for(int iter=0;iter<100;++iter)
  {
    Vector3 L=estimate_random_line(pts);
    for (int i = 0; i<n; ++i)
      serrs[i] = errs[i] = fabs(pts[i].dot(L));
    double score=calc_score(errs);
    std::nth_element(serrs.begin(),serrs.begin()+n/2,serrs.end());
    double thres=Max(1.0,serrs[n/2]);
    int inliers = 0;
    for (int i = 0; i < n; ++i)
    {
      mask[i] = (errs[i] < thres);
      if (mask[i]) inliers++;
    }
    L=estimate_ls_line(pts,mask);
    for (int i = 0; i<n; ++i)
      serrs[i] = errs[i] = fabs(pts[i].dot(L));
    score=calc_score(errs);
    inliers = 0;
    for (int i = 0; i < n; ++i)
    {
      mask[i] = (errs[i] < thres);
      if (mask[i]) inliers++;
    }
    if (score>max_score)
    {
      max_score=score;
      best=L;
      Vector3 D(-L.y(),L.x(),0);
      double mn=9e9,mx=-9e9;
      size_t mni=0,mxi=0;
      for(size_t i=0;i<n;++i)
      {
        if (mask[i])
        {
          double d=pts[i].transpose()*D;
          if (d<mn) { mn=d; mni=i; }
          if (d>mx) { mx=d; mxi=i; }
        }
      }
      p1=sliceun(pts[mni] - (pts[mni].dot(L))*L);
      p2=sliceun(pts[mxi] - (pts[mxi].dot(L))*L);
    }
  }
  return max_score;
}

void filter_outlier_lines(lines_2d_vec& lines)
{
  dvec angles;
  for (auto& l : lines)
  {
    if (l.p1.y() > l.p2.y()) std::swap(l.p1, l.p2);
    angles.push_back(atan2(l.p2.y() - l.p1.y(), l.p2.x() - l.p1.x()));
    //std::cout << angles.back() << std::endl;
  }
  size_t n = angles.size();
  double med = find_median(angles.begin(), angles.end());
  dvec errs(n);
  for (size_t i = 0; i < n; ++i) errs[i] = fabs(angles[i] - med);
  double thres=calculate_mean(errs.begin(), errs.end());
  lines_2d_vec alt;
  for (size_t i = 0; i < n; ++i)
    if (errs[i] < thres) alt.push_back(lines[i]);
  lines.swap(alt);
}

void filter_parallel_lines(lines_2d_vec& lines)
{
  size_t n=lines.size();
  int_set bad;
  typedef std::pair<double, double> projection;
  typedef std::vector<projection> proj_vec;
  proj_vec lines_proj;
  for (size_t i = 0; i < n; ++i)
  {
    const auto& l = lines[i];
    if (fabs(l.normal.y()) > fabs(l.normal.x()))
      lines_proj.push_back(std::make_pair(l.p1.x(), l.p2.x()));
    else
      lines_proj.push_back(std::make_pair(l.p1.y(), l.p2.y()));
  }
  for(size_t i=0;i<n;++i)
  {
    auto& l1=lines[i];
    auto& pr1 = lines_proj[i];
    for(size_t j=i+1;j<n;++j)
    {
      auto& l2=lines[j];
      auto& pr2 = lines_proj[j];
      double overlap = Min(pr1.second,pr2.second) - Max(pr1.first, pr2.first);
      double range = Max(pr1.second, pr2.second) - Min(pr1.first, pr2.first);
      double overlap_ratio = overlap / range;
      if (udiff(l1.aligned_distance,l2.aligned_distance)<32 && 
          fabs(l1.normal.dot(l2.normal))>0.95 &&
          overlap_ratio>0)
      {
        double start=Max(l1.p1.dot(l1.dir), l2.p1.dot(l1.dir));
        double stop=Min(l1.p2.dot(l1.dir), l2.p2.dot(l1.dir));
        if (stop > start)
        {
          if (l1.contrast > l2.contrast) bad.insert(int(j));
          else
          {
            bad.insert(int(i));
            break;
          }
        }
      }
    }
  }
//  std::sort(bad.begin(),bad.end());
//  for(size_t i=0;i<(bad.size()-1);++i)
//    if (bad[i]==bad[i+1]) throw xstring("Duplicate bad");
  int offset=0;
  for(auto i : bad)
  {
    lines.erase(lines.begin()+(i+offset));
    offset--;
  }
}


struct PIndex
{
  PIndex() : coord(0), index(-1) {}
  double coord;
  int    index;
};

struct ContrastPoint : public Vector2
{
  ContrastPoint(double x=0, double y=0, double c=0)
  : Vector2(x,y)
  , contrast(c)
  {}
  double contrast;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int find_horz_lines(const cv::Mat& image, int step, lines_2d_vec& lines)
{
  int border=step;
  int s2=step/2;
  cv::Mat vis;
  //cv::cvtColor(image, vis, CV_GRAY2BGR);
  cxx::EquivalenceClassifier<ContrastPoint> horz_eqv;
  std::vector<PIndex> y_indices(image.cols);
  for(int y=border;y<(image.rows-border);y+=step)
  {
    int last_index = -1;
    int track_back=0;
    const uint8_t* crow=image.ptr(y);
    const uint8_t* prowf = image.ptr(y - step);
    const uint8_t* prow=image.ptr(y-s2);
    const uint8_t* nrow=image.ptr(y+s2);
    const uint8_t* nrowf = image.ptr(y + step);
    for(int x=1;x<(image.cols-1);++x)
    {
      bool edge=false;
      if (udiff(prow[x],nrow[x])>32)
      {
        if (track_back<0)
        {
          track_back=50;
          x-=51;
          continue;
        }
        if (track_back>0) --track_back;
        double range=udiff(prow[x],nrow[x]);
        int mx = Max(prow[x], Max(nrow[x], Max(prowf[x], nrowf[x])));
        int mn = Min(prow[x], Min(nrow[x], Min(prowf[x], nrowf[x])));
        double center=0.5*(mn+mx);
        double contrast = mx - mn;
        {
          edge=true;
          const uint8_t* drow=image.ptr(y+step);
          bool rel=(drow[x]>center);
          uint8_t prev=drow[x];
          for(int dy=step-1;dy>-step;--dy)
          {
            drow=image.ptr(y+dy);
            bool crel=(drow[x]>center);
            if (crel != rel)
            {
              double iy=y+dy+1-(center-prev)/(double(drow[x])-double(prev));
              ContrastPoint p(x,iy,contrast);
              int idx=horz_eqv.add_item(p);
              if (vis.data)
                vis(cv::Rect(x, iy, 1, 1)) = cv::Scalar(0, 255, 0, 255);
              if (last_index>=0) horz_eqv.add_equivalence(idx,last_index);
              PIndex& prevy=y_indices[x];
              if (prevy.index>=0 && udiff(iy,prevy.coord)<2)
                horz_eqv.add_equivalence(idx,prevy.index);
              prevy.coord=iy;
              prevy.index=idx;
              last_index=idx;
              break;
            }
            prev=drow[x];
          }
        }
      }
      if (!edge)
      {
        last_index=-1;
        y_indices[x].index=-1;
        if (track_back<=0)
        {
          track_back=-1;
          x+=50;
        }
      }
    }
  }
  if (vis.data)
  {
    //cv::imwrite("hlines.png", vis);
    show(vis);
  }
  int gi=0;
  for(const int_vec& group : horz_eqv)
  {
    size_t n=group.size();
    if (n > 60)
    {
      vector3_vec pts(n);
      double sum_contrast=0;
      for (size_t i = 0; i < n; ++i)
      {
        const ContrastPoint& p = horz_eqv.get(group[i]);
        double x = p.x(), y = p.y();
        sum_contrast += p.contrast;
        pts[i] = extend(p);
      }
      std::sort(pts.begin(), pts.end(), [](const Vector3& a, const Vector3& b) { return a.x() < b.x(); });
      Vector2 p1,p2;
      estimate_line(pts,p1,p2,0.9);
      lines.push_back(Line2D(p1,p2,sum_contrast/n));
    }
  }
  filter_parallel_lines(lines);
  return 0;
}

inline int pmod(int a, int m)
{
  return (a + m) % m;
}

int find_vert_lines(const cv::Mat& image, int step, lines_2d_vec& lines)
{
  int n=0;
  int border=step;
  int s2=step/2;
  cxx::EquivalenceClassifier<ContrastPoint> vert_eqv;
  typedef std::vector<PIndex> row_indices_vec;
  const int HISTORY = 4;
  std::vector<row_indices_vec> row_indices(HISTORY, row_indices_vec(image.cols / step + 3));
  int row_idx = 0;
  for(int y=0;y<image.rows;++y)
  {
    const uint8_t* row=image.ptr(y);
    int j=2;
    for(int x=border;x<(image.cols-border);x+=step,++j)
    {
      if (udiff(row[x+s2],row[x-s2])>32)
      {
        double center=0.5*(int(row[x-step])+int(row[x+step]));
        double contrast = fabs(int(row[x+step]) - int(row[x-step]));
        bool rel=(row[x-step]<center);
        for(int dx=-step;dx<step;++dx)
        {
          int cur=row[x+dx+1];
          bool crel=(cur<center);
          if (rel!=crel)
          {
            double prev=row[x+dx];
            double xd=x+dx+(center-prev)/(cur-prev);
            int idx=vert_eqv.add_item(ContrastPoint(xd,y,contrast));
            bool found = false;
            for (int dy = 1; dy<HISTORY && !found; ++dy)
            {
              const row_indices_vec& pr = row_indices[pmod(row_idx - dy,HISTORY)];
              for (int dj = -2; dj <= 2 && !found; ++dj)
              {
                if (pr[j + dj].index >= 0 && udiff(xd, pr[j + dj].coord) < 3)
                {
                  vert_eqv.add_equivalence(pr[j + dj].index, idx);
                  found = true;
                }
              }
            }
            row_indices[row_idx][j].index = idx;
            row_indices[row_idx][j].coord = xd;
            break;
          }
        }
        ++n;
        x+=step;
        ++j;
      }
      else row_indices[row_idx][j].index=-1;
    }
    row_idx = pmod(row_idx + 1,HISTORY);
  }

  for(const int_vec& group : vert_eqv)
  {
    size_t n=group.size();
    if (n > 60)
    {
      vector3_vec pts(n);
      double sum_contrast=0;
      for(size_t i=0;i<n;++i)
      {
        const ContrastPoint& p=vert_eqv.get(group[i]);
        sum_contrast += p.contrast;
        pts[i]=extend(p);
      }
      Vector2 p1,p2;
      estimate_line(pts,p1,p2,0.9);
      lines.push_back(Line2D(p1,p2,sum_contrast/n));
    }
  }
  filter_parallel_lines(lines);
  filter_outlier_lines(lines);
  return n;
}

void find_intersections(const lines_2d_vec& hlines, const lines_2d_vec& vlines, ip_vec& points)
{
  for(size_t hi=0;hi<hlines.size();++hi)
  {
    const auto& hl=hlines[hi];
    for(size_t vi=0;vi<vlines.size();++vi)
    {
      const auto& vl=vlines[vi];
      for(int i=0;i<4;++i)
      {
        const Vector2& ph=((i&1)==1)?hl.p1:hl.p2;
        const Vector2& pv=((i&2)==2)?vl.p1:vl.p2;
        if ((ph-pv).norm() < 32) 
        {
          Vector3 p1=extend(hl.p1);
          Vector3 p2=extend(hl.p2);
          Vector3 l1=p1.cross(p2);
          l1=(1.0/sqrt(sqr(l1.x())+sqr(l1.y()))) * l1;
          p1=extend(vl.p1);
          p2=extend(vl.p2);
          Vector3 l2=p1.cross(p2);
          l2=(1.0/sqrt(sqr(l2.x())+sqr(l2.y()))) * l2;
          Vector3 p=l1.cross(l2);
          points.push_back(IntersectionPoint(slice(p),int(hi),int(vi)));
          points.back().calculate_descriptor(hlines,vlines);
        }
      }
    }
  }
}
