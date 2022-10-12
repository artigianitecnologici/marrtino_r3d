// Generated by gencpp from file face_tracker_pkg/centroid.msg
// DO NOT EDIT!


#ifndef FACE_TRACKER_PKG_MESSAGE_CENTROID_H
#define FACE_TRACKER_PKG_MESSAGE_CENTROID_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace face_tracker_pkg
{
template <class ContainerAllocator>
struct centroid_
{
  typedef centroid_<ContainerAllocator> Type;

  centroid_()
    : x(0)
    , y(0)  {
    }
  centroid_(const ContainerAllocator& _alloc)
    : x(0)
    , y(0)  {
  (void)_alloc;
    }



   typedef int32_t _x_type;
  _x_type x;

   typedef int32_t _y_type;
  _y_type y;




  typedef boost::shared_ptr< ::face_tracker_pkg::centroid_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::face_tracker_pkg::centroid_<ContainerAllocator> const> ConstPtr;

}; // struct centroid_

typedef ::face_tracker_pkg::centroid_<std::allocator<void> > centroid;

typedef boost::shared_ptr< ::face_tracker_pkg::centroid > centroidPtr;
typedef boost::shared_ptr< ::face_tracker_pkg::centroid const> centroidConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::face_tracker_pkg::centroid_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::face_tracker_pkg::centroid_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace face_tracker_pkg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'face_tracker_pkg': ['/home/lentin/social_robot_ws/src/face_tracker_pkg/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::face_tracker_pkg::centroid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::face_tracker_pkg::centroid_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::face_tracker_pkg::centroid_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::face_tracker_pkg::centroid_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::face_tracker_pkg::centroid_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::face_tracker_pkg::centroid_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::face_tracker_pkg::centroid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd7b43fd41d4c47bf5c703cc7d016709";
  }

  static const char* value(const ::face_tracker_pkg::centroid_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd7b43fd41d4c47bULL;
  static const uint64_t static_value2 = 0xf5c703cc7d016709ULL;
};

template<class ContainerAllocator>
struct DataType< ::face_tracker_pkg::centroid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "face_tracker_pkg/centroid";
  }

  static const char* value(const ::face_tracker_pkg::centroid_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::face_tracker_pkg::centroid_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 x\n\
int32 y\n\
\n\
";
  }

  static const char* value(const ::face_tracker_pkg::centroid_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::face_tracker_pkg::centroid_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct centroid_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::face_tracker_pkg::centroid_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::face_tracker_pkg::centroid_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FACE_TRACKER_PKG_MESSAGE_CENTROID_H
