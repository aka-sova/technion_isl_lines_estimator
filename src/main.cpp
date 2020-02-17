#include <aligned_def.h>
#include <utils.h>
#include <cxx/properties.h>
#include <cxx/cmdline.h>
#include <lines.h>
#include <lines2.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <filesystem>
#include <stdio.h> 
#include <direct.h>

#define GetCurrentDir _getcwd


cxx::xstring g_Output;
bool         g_Vis=true;

COMMAND_LINE_OPTION(o,true,"Output to file")
{
  g_Output=param;
}

COMMAND_LINE_OPTION(v,false,"Enable visualization")
{
  g_Vis=true;
}

void visualize(cv::Mat image, const lines_2d_vec& vert_lines, 
               const lines_2d_vec& horiz_lines, const std::vector<double>& yaw_angles)
{
  cv::Mat vis;
  cv::cvtColor(image,vis,CV_GRAY2BGR);
  for(const auto& l : vert_lines)
    cv::line(vis,cvt(l.p1),cvt(l.p2),cv::Scalar(0,255,0,255));
  auto yaw_it=yaw_angles.begin();
  double oy=-30;
  for(const auto& l : horiz_lines)
  {
    cv::line(vis,cvt(l.p1),cvt(l.p2),cv::Scalar(0,0,255,255));
    double yaw=*yaw_it++;
    // cv::putText(vis,xstring(yaw),cvt(l.p1+Vector2(50,oy)),
    //             cv::FONT_HERSHEY_PLAIN,2.0,cv::Scalar(255,0,0,255),2);
    oy+=30;
    if (oy>61) oy=-30;
  }
  show(vis);
}

void calculate_yaw_angles(const Matrix3& K, const lines_2d_vec& horiz_lines, std::vector<double>& yaw_angles)
{
  yaw_angles.clear();
  Matrix3 Ki=K.inverse();
  for(const auto& l : horiz_lines)
  {
    Vector2 dir=l.p2-l.p1;
    double s=dir.y()/dir.x();
    Vector3 D=Ki*Vector3(l.p1.x(),l.p1.y(),1.0);
    double tan_yaw=s*D.z()/(D.y()-s*D.x());
    yaw_angles.push_back(atan(tan_yaw));
  }
}




int main(int argc, char* argv[])
{
  try
  {
    //PROCESS_COMMAND_LINE_P("<cfgfile> [image]",1,2);
    //cxx::xstring cfgfile,image_filename;
    //*cmd >> cfgfile;
    //if (cmd->get_parameter_count()>1)
    //  *cmd >> image_filename;

    cxx::xstring image_filename;

    //cfgfile = "config.txt";

    // calibration 
    double focal_x = 3000.0;
    double focal_y = 3000.0;
    double skew = 0;
    double principal_x = 1980;
    double principal_y = 1500;

    Matrix3 K(3, 3);
    K(0, 0) = focal_x;
    K(0, 1) = 0;
    K(0, 2) = principal_x;
    K(1, 0) = 0;
    K(1, 1) = focal_y;
    K(1, 2) = principal_y;
    K(2, 0) = 0; // assume no skew
    K(2, 1) = 0; // assume no skew
    K(2, 2) = 1;


    std::unique_ptr<std::ofstream> fout;
    std::ostream* os=&std::cout;
    if (!g_Output.empty())
    {
      fout.reset(new std::ofstream(g_Output));
      os=fout.get();
    }
    //cxx::Properties props(cfgfile);
    //cxx::xstring frames_list=props.get("FRAME_LIST");

    char cCurrentPath[FILENAME_MAX];

    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
    {
        return errno;
    }
    printf("The current working directory is %s", cCurrentPath);

    cxx::xstring frames_list = "images.txt";
    std::vector<cxx::xstring> frames;
    load_strings(frames_list,frames);

    //Matrix3 K;
    //parse_matrix(props.get("K"),K);

    if (!image_filename.empty())
      frames.assign(1,image_filename);
    for(auto& filename : frames)
    {
      cv::Mat image_color = cv::imread(filename);
      cv::Mat image;
      cv::cvtColor(image_color, image, cv::COLOR_BGR2GRAY);

      // cv::Mat image=cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat vis;
      if (g_Vis)
        cv::cvtColor(image,vis,CV_GRAY2BGR);
      if (!image.data) continue;

      LinesEstimator le(image,K,vis);
      if (g_Vis) show(vis);
      double pitch=le.get_pitch();
      double roll=le.get_roll();
      std::vector<double> yaw_angles;
      *os << filename << " " << pitch << " " << roll << std::endl;
      std::cerr << filename << std::endl;
    }
  } catch (const cxx::xstring& msg)
  {
    std::cerr << msg << std::endl;
  }
  return 0;
}
