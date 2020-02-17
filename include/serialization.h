#pragma once

#include <boost/serialization/unique_ptr.hpp>

namespace boost
{
  namespace serialization
  {
    template<class Archive>
    void serialize(Archive& ar, Vector3& v, unsigned int version)
    {
      ar & v.x();
      ar & v.y();
      ar & v.z();
    }

    template<class Archive>
    void serialize(Archive& ar, Matrix3& m, unsigned int version)
    {
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          ar & m(i, j);
    }


  }
}