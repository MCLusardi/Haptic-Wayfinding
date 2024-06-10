// Generated by gencpp from file stretch_demos/VisualServoGoal.msg
// DO NOT EDIT!


#ifndef STRETCH_DEMOS_MESSAGE_VISUALSERVOGOAL_H
#define STRETCH_DEMOS_MESSAGE_VISUALSERVOGOAL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace stretch_demos
{
template <class ContainerAllocator>
struct VisualServoGoal_
{
  typedef VisualServoGoal_<ContainerAllocator> Type;

  VisualServoGoal_()
    : target_frame()
    , source_frame()  {
    }
  VisualServoGoal_(const ContainerAllocator& _alloc)
    : target_frame(_alloc)
    , source_frame(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _target_frame_type;
  _target_frame_type target_frame;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _source_frame_type;
  _source_frame_type source_frame;





  typedef boost::shared_ptr< ::stretch_demos::VisualServoGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stretch_demos::VisualServoGoal_<ContainerAllocator> const> ConstPtr;

}; // struct VisualServoGoal_

typedef ::stretch_demos::VisualServoGoal_<std::allocator<void> > VisualServoGoal;

typedef boost::shared_ptr< ::stretch_demos::VisualServoGoal > VisualServoGoalPtr;
typedef boost::shared_ptr< ::stretch_demos::VisualServoGoal const> VisualServoGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stretch_demos::VisualServoGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::stretch_demos::VisualServoGoal_<ContainerAllocator1> & lhs, const ::stretch_demos::VisualServoGoal_<ContainerAllocator2> & rhs)
{
  return lhs.target_frame == rhs.target_frame &&
    lhs.source_frame == rhs.source_frame;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::stretch_demos::VisualServoGoal_<ContainerAllocator1> & lhs, const ::stretch_demos::VisualServoGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace stretch_demos

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stretch_demos::VisualServoGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stretch_demos::VisualServoGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stretch_demos::VisualServoGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1cca159d7671f770d19e4e288854c223";
  }

  static const char* value(const ::stretch_demos::VisualServoGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1cca159d7671f770ULL;
  static const uint64_t static_value2 = 0xd19e4e288854c223ULL;
};

template<class ContainerAllocator>
struct DataType< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stretch_demos/VisualServoGoal";
  }

  static const char* value(const ::stretch_demos::VisualServoGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# goal\n"
"# target frame defines the transform frame that needs to be compared with source_frame\n"
"string target_frame\n"
"\n"
"# target frame defines the transform frame that needs to be compared with target_frame\n"
"string source_frame\n"
"\n"
;
  }

  static const char* value(const ::stretch_demos::VisualServoGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_frame);
      stream.next(m.source_frame);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VisualServoGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stretch_demos::VisualServoGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stretch_demos::VisualServoGoal_<ContainerAllocator>& v)
  {
    s << indent << "target_frame: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.target_frame);
    s << indent << "source_frame: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.source_frame);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STRETCH_DEMOS_MESSAGE_VISUALSERVOGOAL_H
