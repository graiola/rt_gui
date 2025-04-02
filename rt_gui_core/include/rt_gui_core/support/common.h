#ifndef RT_GUI_CORE_SUPPORT_COMMON_H
#define RT_GUI_CORE_SUPPORT_COMMON_H

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <tuple>
#include <map>
#include <string>
#include <functional>

namespace rt_gui
{

using key_t = std::pair<std::string, std::string>;

template <class data_t>
class CallbackBuffer {

public:

  using callback_t = std::function<void(data_t)>;
  using value_t = std::tuple<data_t*, data_t, callback_t, bool, bool>;
  using buffer_t = std::map<key_t, value_t>;

  CallbackBuffer() = default;

  // Method to add data with callback
  bool add(const std::string& key1, const std::string& key2, data_t data, callback_t callback, bool sync = true)
  {
    if (callback) {
      key_t key{key1, key2};
      buffer_.emplace(key, value_t(nullptr, data, std::move(callback), sync, false));
      old_values_[key] = data;
      return true;
    }
    return false;
  }

  // Method to add data by pointer
  bool add(const std::string& key1, const std::string& key2, data_t* data_ptr, bool sync = true, bool read_only = false)
  {
    if (data_ptr) {
      key_t key{key1, key2};
      buffer_.emplace(key, value_t(data_ptr, *data_ptr, nullptr, sync, read_only));
      old_values_[key] = *data_ptr;
      return true;
    }
    return false;
  }

  // Method to update buffer data
  data_t update(const std::string& key1, const std::string& key2, const data_t& value)
  {
    key_t key{key1, key2};
    auto it = buffer_.find(key);
    if (it == buffer_.end()) {
      throw std::runtime_error("Key not found in buffer!");
    }

    data_t actual_value = std::get<0>(it->second) ? *std::get<0>(it->second) : value;

    if (!std::get<4>(it->second)) { // if not read-only
      if (std::get<3>(it->second)) { // if sync is true
        std::get<1>(it->second) = value;
      } else {
        auto* data_ptr = std::get<0>(it->second);
        if (data_ptr) {
          *data_ptr = value;
        } else if (auto& callback = std::get<2>(it->second)) {
          callback(value);
        } else {
          throw std::runtime_error("Missing pointer and callback in buffer!");
        }
      }
    }
    return actual_value;
  }

  // Sync buffer data
  bool sync()
  {
    for (auto& [key, val] : buffer_) {
      if (std::get<3>(val)) { // sync
        auto* data_ptr = std::get<0>(val);
        if (data_ptr) {
          *data_ptr = std::get<1>(val);
        } else if (auto& callback = std::get<2>(val)) {
          callback(std::get<1>(val));
        } else {
          return false;
        }
      }
    }
    return true;
  }

  // Retrieve value from buffer
  bool getValue(const std::string& key1, const std::string& key2, data_t& value)
  {
    key_t key{key1, key2};
    auto it = buffer_.find(key);
    if (it != buffer_.end() && std::get<0>(it->second)) {
      value = *std::get<0>(it->second);
      return true;
    }
    return false;
  }

  // Check if data has changed
  bool isDataChanged(const std::string& key1, const std::string& key2)
  {
    key_t key{key1, key2};
    auto it = buffer_.find(key);
    if (it != buffer_.end() && std::get<0>(it->second)) {
      if (old_values_[key] != *std::get<0>(it->second)) {
        old_values_[key] = *std::get<0>(it->second);
        return true;
      }
    }
    return false;
  }

  // Accessor for the buffer
  const buffer_t& getBuffer() const
  {
    return buffer_;
  }

private:
  buffer_t buffer_;
  std::map<key_t, data_t> old_values_;
};

#define RT_GUI_SERVER_NAME "rt_gui_server"
#define RT_GUI_CLIENT_NAME "rt_gui_client"

// Define ros service names using a struct
struct
{
  struct ServiceGroup {
    std::string add;
    std::string update;
    std::string feedback;
  };

  ServiceGroup double_srvs{"add_double", "update_double", "feedback_double"};
  ServiceGroup int_srvs{"add_int", "update_int", "feedback_int"};
  ServiceGroup bool_srvs{"add_bool", "update_bool", "feedback_bool"};
  ServiceGroup list_srvs{"add_list", "update_list", "feedback_list"};
  ServiceGroup check_srvs{"add_check", "update_check", "feedback_check"};
  ServiceGroup trigger_srvs{"add_trigger", "update_trigger", "feedback_trigger"};
  ServiceGroup text_srvs{"add_text", "update_text", "feedback_text"};
  ServiceGroup label_srvs{"add_label", "update_label", "feedback_label"};

  unsigned int n_threads = 3;
  double wait_service_secs = 60.0;
  std::string remove_service = "remove_widget";
  std::string add_client = "add_client";

} _ros_services;

} // namespace rt_gui

#endif
