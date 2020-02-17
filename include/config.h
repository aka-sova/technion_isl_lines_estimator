#ifndef config_h__
#define config_h__

#include <utils.h>
#include <cxx/xstring.h>
#include <consts.h>
#include <properties.h>

inline double G(double sigma);
inline Matrix matrix_3x1(double x, double y, double z);

struct Config
{
  double world_point_noise_sigma;
  double frame_point_noise_sigma;
  double camera_position_noise_sigma;
  double camera_angles_noise_sigma;

  double camera_motion_sigma;
  double camera_rotation_sigma;

  unsigned world_points_count;
  unsigned camera_count;

  int  verbose_level;
  bool round_test_case;
  xstring test_case_file;

  Config()
    : world_point_noise_sigma(0)
    , frame_point_noise_sigma(0)
    , camera_position_noise_sigma(0)
    , camera_angles_noise_sigma(0)
    , camera_motion_sigma(0.1)
    , camera_rotation_sigma(0.05)
    , world_points_count(N_WORLD_POINTS)
    , camera_count(M_CAMERAS)
    , verbose_level(1)
    , round_test_case(false)
  {}

  void set_sample_noise()
  {
    world_point_noise_sigma = 0.01; // meters
    frame_point_noise_sigma = 0.5; // pixels
    camera_position_noise_sigma = 0.01; // meters
    camera_angles_noise_sigma = 0.01; // radians   (about 1/2 degree)
  }

  bool load(const char* filename = "ba.cfg")
  {
    try
    {
      Properties P(filename);
      world_point_noise_sigma = P.get("WORLD_POINT_SIGMA").as_double();
      frame_point_noise_sigma = P.get("FRAME_POINT_SIGMA").as_double();
      camera_position_noise_sigma = P.get("CAMERA_POSITION_SIGMA").as_double();
      camera_angles_noise_sigma = P.get("CAMERA_ANGLES_SIGMA").as_double();
      camera_motion_sigma = P.get("CAMERA_MOTION_SIGMA").as_double();
      camera_rotation_sigma = P.get("CAMERA_ROTATION_SIGMA").as_double();

      test_case_file = P.get("TEST_CASE");

      if (P.has("WORLD_POINT_COUNT")) world_points_count = P.get("WORLD_POINT_COUNT").as_int();
      if (P.has("CAMERA_COUNT")) camera_count = P.get("CAMERA_COUNT").as_int();

      verbose_level = P.get("VERBOSE").as_int();
      round_test_case = (P.get("ROUND_TEST_CASE") == "1");
    }
    catch (const xstring& msg)
    {
      std::cout << "Config load failed, " << msg << std::endl;
      return false;
    }
    return true;
  }

  FramePoint gen_frame_point_noise()
  {
    return FramePoint(G(frame_point_noise_sigma), G(frame_point_noise_sigma));
  }

  WorldPoint gen_world_point_noise()
  {
    return WorldPoint(G(world_point_noise_sigma), G(world_point_noise_sigma), G(world_point_noise_sigma));
  }

  Matrix gen_camera_position_noise()
  {
    //return matrix_3x1(G(camera_position_noise_sigma), 0, 0);
    return matrix_3x1(G(camera_position_noise_sigma), G(camera_position_noise_sigma), G(camera_position_noise_sigma));
  }

  Matrix gen_camera_angles_noise()
  {
    return matrix_3x1(G(camera_angles_noise_sigma), G(camera_angles_noise_sigma), G(camera_angles_noise_sigma));
  }

};



#endif // config_h__
