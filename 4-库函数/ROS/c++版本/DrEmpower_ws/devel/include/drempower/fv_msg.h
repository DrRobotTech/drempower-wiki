// Generated by gencpp from file drempower/fv_msg.msg
// DO NOT EDIT!


#ifndef DREMPOWER_MESSAGE_FV_MSG_H
#define DREMPOWER_MESSAGE_FV_MSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace drempower
{
template <class ContainerAllocator>
struct fv_msg_
{
  typedef fv_msg_<ContainerAllocator> Type;

  fv_msg_()
    : id_list()
    , input_vel_list()
    , input_torque_list()  {
    }
  fv_msg_(const ContainerAllocator& _alloc)
    : id_list(_alloc)
    , input_vel_list(_alloc)
    , input_torque_list(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> _id_list_type;
  _id_list_type id_list;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _input_vel_list_type;
  _input_vel_list_type input_vel_list;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _input_torque_list_type;
  _input_torque_list_type input_torque_list;





  typedef boost::shared_ptr< ::drempower::fv_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::drempower::fv_msg_<ContainerAllocator> const> ConstPtr;

}; // struct fv_msg_

typedef ::drempower::fv_msg_<std::allocator<void> > fv_msg;

typedef boost::shared_ptr< ::drempower::fv_msg > fv_msgPtr;
typedef boost::shared_ptr< ::drempower::fv_msg const> fv_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::drempower::fv_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::drempower::fv_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::drempower::fv_msg_<ContainerAllocator1> & lhs, const ::drempower::fv_msg_<ContainerAllocator2> & rhs)
{
  return lhs.id_list == rhs.id_list &&
    lhs.input_vel_list == rhs.input_vel_list &&
    lhs.input_torque_list == rhs.input_torque_list;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::drempower::fv_msg_<ContainerAllocator1> & lhs, const ::drempower::fv_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace drempower

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::drempower::fv_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::drempower::fv_msg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drempower::fv_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drempower::fv_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drempower::fv_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drempower::fv_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::drempower::fv_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3c98505ee3c30564cc662aa205b0a079";
  }

  static const char* value(const ::drempower::fv_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3c98505ee3c30564ULL;
  static const uint64_t static_value2 = 0xcc662aa205b0a079ULL;
};

template<class ContainerAllocator>
struct DataType< ::drempower::fv_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "drempower/fv_msg";
  }

  static const char* value(const ::drempower::fv_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::drempower::fv_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16[] id_list\n"
"float32[] input_vel_list\n"
"float32[] input_torque_list\n"
"\n"
;
  }

  static const char* value(const ::drempower::fv_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::drempower::fv_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id_list);
      stream.next(m.input_vel_list);
      stream.next(m.input_torque_list);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct fv_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::drempower::fv_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::drempower::fv_msg_<ContainerAllocator>& v)
  {
    s << indent << "id_list[]" << std::endl;
    for (size_t i = 0; i < v.id_list.size(); ++i)
    {
      s << indent << "  id_list[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.id_list[i]);
    }
    s << indent << "input_vel_list[]" << std::endl;
    for (size_t i = 0; i < v.input_vel_list.size(); ++i)
    {
      s << indent << "  input_vel_list[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.input_vel_list[i]);
    }
    s << indent << "input_torque_list[]" << std::endl;
    for (size_t i = 0; i < v.input_torque_list.size(); ++i)
    {
      s << indent << "  input_torque_list[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.input_torque_list[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DREMPOWER_MESSAGE_FV_MSG_H
