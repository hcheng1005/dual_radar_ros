// Generated by gencpp from file radarExp/ArbeRadarPoint.msg
// DO NOT EDIT!


#ifndef RADAREXP_MESSAGE_ARBERADARPOINT_H
#define RADAREXP_MESSAGE_ARBERADARPOINT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace radarExp
{
template <class ContainerAllocator>
struct ArbeRadarPoint_
{
  typedef ArbeRadarPoint_<ContainerAllocator> Type;

  ArbeRadarPoint_()
    : id(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , rcs(0)
    , doppler(0)  {
    }
  ArbeRadarPoint_(const ContainerAllocator& _alloc)
    : id(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , rcs(0)
    , doppler(0)  {
  (void)_alloc;
    }



   typedef uint32_t _id_type;
  _id_type id;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef uint8_t _rcs_type;
  _rcs_type rcs;

   typedef uint8_t _doppler_type;
  _doppler_type doppler;





  typedef boost::shared_ptr< ::radarExp::ArbeRadarPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::radarExp::ArbeRadarPoint_<ContainerAllocator> const> ConstPtr;

}; // struct ArbeRadarPoint_

typedef ::radarExp::ArbeRadarPoint_<std::allocator<void> > ArbeRadarPoint;

typedef boost::shared_ptr< ::radarExp::ArbeRadarPoint > ArbeRadarPointPtr;
typedef boost::shared_ptr< ::radarExp::ArbeRadarPoint const> ArbeRadarPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::radarExp::ArbeRadarPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::radarExp::ArbeRadarPoint_<ContainerAllocator1> & lhs, const ::radarExp::ArbeRadarPoint_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.rcs == rhs.rcs &&
    lhs.doppler == rhs.doppler;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::radarExp::ArbeRadarPoint_<ContainerAllocator1> & lhs, const ::radarExp::ArbeRadarPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace radarExp

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::radarExp::ArbeRadarPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::radarExp::ArbeRadarPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::radarExp::ArbeRadarPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "acef2277b91a0014d044f54e5ef5d5ff";
  }

  static const char* value(const ::radarExp::ArbeRadarPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xacef2277b91a0014ULL;
  static const uint64_t static_value2 = 0xd044f54e5ef5d5ffULL;
};

template<class ContainerAllocator>
struct DataType< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "radarExp/ArbeRadarPoint";
  }

  static const char* value(const ::radarExp::ArbeRadarPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ARBE pointcloud format.\n"
"\n"
"uint32 id               # ID\n"
"float32 x               # X axis, unit:m\n"
"float32 y               # Y axis, unit:m\n"
"float32 z               # Z axis, unit:m\n"
"uint8 rcs               # reflectivity, 0~255\n"
"uint8 doppler           # livox tag\n"
"\n"
"\n"
;
  }

  static const char* value(const ::radarExp::ArbeRadarPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.rcs);
      stream.next(m.doppler);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArbeRadarPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::radarExp::ArbeRadarPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::radarExp::ArbeRadarPoint_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "rcs: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rcs);
    s << indent << "doppler: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.doppler);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RADAREXP_MESSAGE_ARBERADARPOINT_H
