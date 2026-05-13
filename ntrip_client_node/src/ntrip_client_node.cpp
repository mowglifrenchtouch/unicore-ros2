// Copyright 2023 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ntrip_client_node/visibility_control.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include <curl/curl.h>

namespace ublox_dgnss
{

struct CurlHandle
{
  CURL *handle;
  CurlHandle() : handle(curl_easy_init())
  {
  }
  ~CurlHandle()
  {
    curl_easy_cleanup(handle);
  }
};

// Format a NavSatFix as an NMEA $GPGGA sentence suitable for the
// Ntrip-GGA HTTP header (used by VRS / NEAR mountpoints to route to
// the closest physical base station). Returns an empty string if the
// fix is invalid (status STATUS_NO_FIX).
inline std::string format_gga(const sensor_msgs::msg::NavSatFix &fix)
{
  if (fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
  {
    return "";
  }

  std::time_t t = std::time(nullptr);
  std::tm tm_utc{};
  gmtime_r(&t, &tm_utc);

  const double lat_abs = std::fabs(fix.latitude);
  const int lat_deg = static_cast<int>(lat_abs);
  const double lat_min = (lat_abs - lat_deg) * 60.0;
  const double lon_abs = std::fabs(fix.longitude);
  const int lon_deg = static_cast<int>(lon_abs);
  const double lon_min = (lon_abs - lon_deg) * 60.0;
  const char ns = fix.latitude >= 0.0 ? 'N' : 'S';
  const char ew = fix.longitude >= 0.0 ? 'E' : 'W';

  // Map NavSatStatus.status -> NMEA GGA quality. STATUS_GBAS_FIX is
  // reported as quality 4 (RTK Fixed); STATUS_SBAS_FIX as 5 (RTK Float
  // / DGPS — the caster doesn't care about the distinction between 2
  // and 5 for routing); STATUS_FIX as 1 (SPS).
  int quality = 1;
  if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
  {
    quality = 4;
  }
  else if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX)
  {
    quality = 5;
  }

  char body[192];
  const int n = std::snprintf(
      body, sizeof(body),
      "GPGGA,%02d%02d%02d.00,%02d%07.4f,%c,%03d%07.4f,%c,%d,10,1.0,%.1f,M,0.0,M,,",
      tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec,
      lat_deg, lat_min, ns,
      lon_deg, lon_min, ew,
      quality,
      fix.altitude);
  if (n <= 0)
  {
    return "";
  }

  std::uint8_t cs = 0;
  for (int i = 0; i < n; ++i)
  {
    cs ^= static_cast<std::uint8_t>(body[i]);
  }
  char out[224];
  std::snprintf(out, sizeof(out), "$%s*%02X", body, cs);
  return std::string(out);
}

class NTRIPClientNode : public rclcpp::Node
{
public:
  NTRIP_CLIENT_NODE_PUBLIC
  explicit NTRIPClientNode(const rclcpp::NodeOptions &options)
      : Node("ntrip_client", rclcpp::NodeOptions(options)),
        curl_handle_(std::make_shared<CurlHandle>())
  {
    RCLCPP_INFO(get_logger(), "starting %s", get_name());

    declare_parameter("use_https", true);
    declare_parameter("host", "ntrip.data.gnss.ga.gov.au");
    declare_parameter("port", 443);
    declare_parameter("mountpoint", "MBCH00AUS0");
    declare_parameter("username", "noname");
    declare_parameter("password", "password");
    declare_parameter("log_level", "INFO");
    declare_parameter("maxage_conn", 30);
    // VRS / NEAR-style mountpoints route to the closest physical base
    // station based on the rover's NMEA GGA position. We subscribe to
    // a NavSatFix topic and forward the latest fix as the Ntrip-GGA HTTP
    // header on every connection. Set gga_fix_topic="" to disable.
    declare_parameter("gga_fix_topic", "/gps/fix");

    use_https_ = get_parameter("use_https").as_bool();
    host_ = get_parameter("host").as_string();
    port_ = get_parameter("port").as_int();
    mountpoint_ = get_parameter("mountpoint").as_string();
    username_ = get_parameter("username").as_string();
    password_ = get_parameter("password").as_string();
    log_level_ = get_parameter("log_level").as_string();
    maxage_conn_ = get_parameter("maxage_conn").as_int();
    gga_fix_topic_ = get_parameter("gga_fix_topic").as_string();

    if (!gga_fix_topic_.empty())
    {
      fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          gga_fix_topic_, rclcpp::SensorDataQoS(),
          std::bind(&NTRIPClientNode::on_fix, this, std::placeholders::_1));
      // Refresh the connection every 5 min so VRS / NEAR mountpoints
      // re-pick the closest base after the rover has moved. For static
      // installs this just keeps the connection alive — cheap.
      gga_refresh_timer_ = create_wall_timer(
          std::chrono::minutes(5),
          [this]() {
            std::lock_guard<std::mutex> lock(gga_mutex_);
            if (!latest_gga_.empty())
            {
              reconfigure_needed_.store(true);
            }
          });
      RCLCPP_INFO(get_logger(),
                  "GGA forwarding enabled — subscribing to %s",
                  gga_fix_topic_.c_str());
    }

    pending_use_https_ = use_https_;
    pending_host_ = host_;
    pending_port_ = port_;
    pending_mountpoint_ = mountpoint_;
    pending_username_ = username_;
    pending_password_ = password_;
    pending_log_level_ = log_level_;
    pending_maxage_conn_ = maxage_conn_;

    parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&NTRIPClientNode::on_set_parameters_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ntrip connection url: '%s'", connection_url().c_str());

    rtcm_pub_ = create_publisher<rtcm_msgs::msg::Message>("/ntrip_client/rtcm", 10);

    curl_global_init(CURL_GLOBAL_DEFAULT);

    constexpr int desired_count = 10;
    auto handle = curl_handle_->handle;
    if (handle != nullptr)
    {
      curl_easy_setopt(handle, CURLOPT_HTTP09_ALLOWED, true);
      curl_easy_setopt(handle, CURLOPT_USERAGENT, "NTRIP ros2/ublox_dgnss");
      curl_easy_setopt(handle, CURLOPT_FAILONERROR, true);
      curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, &NTRIPClientNode::write_callback);
      curl_easy_setopt(handle, CURLOPT_WRITEDATA, this);
      curl_easy_setopt(handle, CURLOPT_PRIVATE, reinterpret_cast<void *>(desired_count));

      apply_curl_options();

      streaming_exit_.store(false);
      streaming_thread_ = std::thread(&NTRIPClientNode::do_streaming, this);
    }
  }

  NTRIP_CLIENT_NODE_LOCAL
  ~NTRIPClientNode() override
  {
    streaming_exit_.store(true);
    if (streaming_thread_.joinable())
    {
      streaming_thread_.join();
    }
    if (gga_header_list_ != nullptr)
    {
      curl_slist_free_all(gga_header_list_);
      gga_header_list_ = nullptr;
    }
    curl_handle_.reset();
    curl_global_cleanup();
    RCLCPP_INFO(get_logger(), "finished");
  }

  void on_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    const std::string gga = format_gga(*msg);
    if (gga.empty())
    {
      return;
    }
    bool first_fix = false;
    {
      std::lock_guard<std::mutex> lock(gga_mutex_);
      first_fix = latest_gga_.empty();
      latest_gga_ = gga;
    }
    if (first_fix)
    {
      // The connection opened in the constructor with an empty GGA
      // (no fix yet at startup), so VRS / NEAR mountpoints couldn't
      // pick a base. Trigger reconfigure so the next streaming cycle
      // re-runs apply_curl_options() with the freshly populated
      // Ntrip-GGA header.
      RCLCPP_INFO(get_logger(),
                  "First GGA available — forcing NTRIP reconnect to send Ntrip-GGA header");
      reconfigure_needed_.store(true);
    }
  }

private:
  std::string connection_url() const
  {
    const std::string scheme = use_https_ ? "https://" : "http://";
    return scheme + host_ + ":" + std::to_string(port_) + "/" + mountpoint_;
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<std::mutex> lock(params_mutex_);
    for (const auto &param : parameters)
    {
      const auto &name = param.get_name();
      if (name == "use_https")
      {
        pending_use_https_ = param.as_bool();
      }
      else if (name == "host")
      {
        if (param.as_string().empty())
        {
          result.successful = false;
          result.reason = "host cannot be empty";
          return result;
        }
        pending_host_ = param.as_string();
      }
      else if (name == "port")
      {
        const auto port = param.as_int();
        if (port < 1 || port > 65535)
        {
          result.successful = false;
          result.reason = "port must be between 1 and 65535";
          return result;
        }
        pending_port_ = static_cast<int>(port);
      }
      else if (name == "mountpoint")
      {
        if (param.as_string().empty())
        {
          result.successful = false;
          result.reason = "mountpoint cannot be empty";
          return result;
        }
        pending_mountpoint_ = param.as_string();
      }
      else if (name == "username")
      {
        pending_username_ = param.as_string();
      }
      else if (name == "password")
      {
        pending_password_ = param.as_string();
      }
      else if (name == "log_level")
      {
        pending_log_level_ = param.as_string();
      }
      else if (name == "maxage_conn")
      {
        pending_maxage_conn_ = param.as_int();
      }
    }

    reconfigure_needed_.store(true);
    RCLCPP_INFO(get_logger(), "Parameter change queued for next streaming cycle");
    return result;
  }

  void apply_curl_options()
  {
    auto handle = curl_handle_->handle;
    const std::string url = connection_url();
    const std::string userpwd = username_ + ":" + password_;

    curl_easy_setopt(handle, CURLOPT_URL, url.c_str());
    curl_easy_setopt(handle, CURLOPT_USERPWD, userpwd.c_str());
    curl_easy_setopt(handle, CURLOPT_VERBOSE, log_level_ != "INFO" ? 1L : 0L);
    curl_easy_setopt(handle, CURLOPT_MAXAGE_CONN, maxage_conn_);

    // (Re)build the Ntrip-* HTTP header list. Ntrip-Version: Ntrip/2.0
    // is required for full NTRIP 2.0 compliance with crtk.net and most
    // modern casters. Ntrip-GGA carries the rover's NMEA GGA so VRS /
    // NEAR mountpoints can route to the closest physical base station.
    if (gga_header_list_ != nullptr)
    {
      curl_slist_free_all(gga_header_list_);
      gga_header_list_ = nullptr;
    }
    gga_header_list_ = curl_slist_append(gga_header_list_, "Ntrip-Version: Ntrip/2.0");
    {
      std::lock_guard<std::mutex> lock(gga_mutex_);
      if (!latest_gga_.empty())
      {
        const std::string h = "Ntrip-GGA: " + latest_gga_;
        gga_header_list_ = curl_slist_append(gga_header_list_, h.c_str());
      }
    }
    curl_easy_setopt(handle, CURLOPT_HTTPHEADER, gga_header_list_);
  }

  static size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata)
  {
    auto *node = reinterpret_cast<NTRIPClientNode *>(userdata);

    std::stringstream hex_stream;
    hex_stream << std::hex << std::setfill('0');
    for (size_t i = 0; i < size * nmemb; ++i)
    {
      hex_stream << std::setw(2) << static_cast<int>(static_cast<unsigned char>(ptr[i]));
    }
    RCLCPP_DEBUG(node->get_logger(),
                 "Received size: %ld nmemb: %ld data: %s",
                 size,
                 nmemb,
                 hex_stream.str().c_str());

    auto message = std::make_unique<rtcm_msgs::msg::Message>();
    message->header.stamp = node->get_clock()->now();
    message->header.frame_id = node->mountpoint_;
    message->message.assign(ptr, ptr + size * nmemb);
    node->rtcm_pub_->publish(std::move(message));

    node->record_count_++;

    int desired_count;
    curl_easy_getinfo(node->curl_handle_->handle, CURLINFO_PRIVATE, &desired_count);
    if (node->record_count_ >= desired_count)
    {
      node->record_count_ = 0;
      node->desired_count_reached_ = true;
      return size * nmemb - 1;
    }

    return size * nmemb;
  }

  void do_streaming()
  {
    while (!streaming_exit_.load())
    {
      desired_count_reached_ = false;
      const CURLcode res = curl_easy_perform(curl_handle_->handle);

      if (reconfigure_needed_.load())
      {
        std::lock_guard<std::mutex> lock(params_mutex_);
        use_https_ = pending_use_https_;
        host_ = pending_host_;
        port_ = pending_port_;
        mountpoint_ = pending_mountpoint_;
        username_ = pending_username_;
        password_ = pending_password_;
        log_level_ = pending_log_level_;
        maxage_conn_ = pending_maxage_conn_;
        apply_curl_options();
        reconfigure_needed_.store(false);
        RCLCPP_INFO(get_logger(), "Reconfigured NTRIP connection: %s", connection_url().c_str());
        continue;
      }

      if (res == CURLE_OK)
      {
        continue;
      }

      if (desired_count_reached_)
      {
        RCLCPP_DEBUG(get_logger(), "Processed desired count...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      char *effective_url = nullptr;
      curl_easy_getinfo(curl_handle_->handle, CURLINFO_EFFECTIVE_URL, &effective_url);
      long response_code = 0;
      curl_easy_getinfo(curl_handle_->handle, CURLINFO_RESPONSE_CODE, &response_code);

      RCLCPP_ERROR(get_logger(),
                   "Failed to perform streaming request for URL: %s",
                   effective_url != nullptr ? effective_url : "(unknown)");
      RCLCPP_ERROR(get_logger(), "Response code: %ld", response_code);
      RCLCPP_ERROR(get_logger(),
                   "Failed to perform streaming request: %s",
                   curl_easy_strerror(res));
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  std::shared_ptr<CurlHandle> curl_handle_;
  std::thread streaming_thread_;

  std::atomic<bool> streaming_exit_{false};
  bool desired_count_reached_{false};
  std::mutex params_mutex_;
  std::atomic<bool> reconfigure_needed_{false};
  int record_count_{0};

  bool pending_use_https_{true};
  std::string pending_host_;
  int pending_port_{443};
  std::string pending_mountpoint_;
  std::string pending_username_;
  std::string pending_password_;
  std::string pending_log_level_;
  long pending_maxage_conn_{30};

  bool use_https_{true};
  std::string host_;
  int port_{443};
  std::string mountpoint_;
  std::string username_;
  std::string password_;
  std::string log_level_;
  long maxage_conn_{30};

  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;

  // GGA forwarding for VRS / NEAR mountpoints.
  std::string gga_fix_topic_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::TimerBase::SharedPtr gga_refresh_timer_;
  std::mutex gga_mutex_;
  std::string latest_gga_;            // protected by gga_mutex_
  curl_slist *gga_header_list_{nullptr};
};

}  // namespace ublox_dgnss

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_dgnss::NTRIPClientNode)
