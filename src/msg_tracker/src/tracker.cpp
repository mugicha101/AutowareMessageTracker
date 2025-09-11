#include "tracker.hpp"

#include <iomanip>
#include <sstream>
#include <thread>

std::string gen_uid(const rmw_gid_t &gid, builtin_interfaces::msg::Time src_stamp) {
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; i++) {
      ss << std::setw(2) << static_cast<unsigned>(gid.data[i]);
  }
  ss << std::dec;
  ss << "|" << src_stamp.sec << ":" << src_stamp.nanosec << "|" << std::this_thread::get_id();
  return ss.str();
}