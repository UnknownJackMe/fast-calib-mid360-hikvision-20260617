#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::string g_current_ip;
std::string g_new_ip;
std::string g_netmask;
std::string g_gateway;
std::atomic<bool> g_command_sent{false};
std::atomic<bool> g_done{false};
std::atomic<bool> g_reboot_after{false};
std::atomic<int> g_status{-1};
std::atomic<uint32_t> g_handle{0};

void CopyIp(char *dst, const std::string &src) {
  std::memset(dst, 0, 16);
  std::strncpy(dst, src.c_str(), 15);
}

void SetIpCallback(livox_status status, uint32_t handle,
                   LivoxLidarAsyncControlResponse *response, void *) {
  g_status = status;
  std::cout << "SetLivoxLidarIp callback: status=" << status
            << " handle=" << handle;
  if (response != nullptr) {
    std::cout << " ret_code=" << static_cast<int>(response->ret_code)
              << " error_key=" << response->error_key;
  }
  std::cout << std::endl;
  if (status == 0 && g_reboot_after) {
    livox_status reboot_status = LivoxLidarRequestReboot(
        handle,
        [](livox_status reboot_cb_status, uint32_t reboot_handle,
           LivoxLidarRebootResponse *reboot_response, void *) {
          g_status = reboot_cb_status;
          std::cout << "LivoxLidarRequestReboot callback: status=" << reboot_cb_status
                    << " handle=" << reboot_handle;
          if (reboot_response != nullptr) {
            std::cout << " ret_code=" << static_cast<int>(reboot_response->ret_code);
          }
          std::cout << std::endl;
          g_done = true;
        },
        nullptr);
    std::cout << "LivoxLidarRequestReboot requested: immediate_status="
              << reboot_status << std::endl;
    if (reboot_status != 0) {
      g_status = reboot_status;
      g_done = true;
    }
    return;
  }
  g_done = true;
}

void InfoChangeCallback(const uint32_t handle, const LivoxLidarInfo *info, void *) {
  if (info == nullptr) {
    return;
  }
  std::cout << "discovered lidar handle=" << handle
            << " type=" << static_cast<int>(info->dev_type)
            << " sn=" << info->sn
            << " ip=" << info->lidar_ip << std::endl;

  if (g_command_sent || g_done) {
    return;
  }
  if (g_current_ip != "*" && g_current_ip != info->lidar_ip) {
    return;
  }

  LivoxLidarIpInfo ip_config;
  CopyIp(ip_config.ip_addr, g_new_ip);
  CopyIp(ip_config.net_mask, g_netmask);
  CopyIp(ip_config.gw_addr, g_gateway);

  g_handle = handle;
  g_command_sent = true;
  livox_status status = SetLivoxLidarIp(handle, &ip_config, SetIpCallback, nullptr);
  std::cout << "SetLivoxLidarIp requested: handle=" << handle
            << " new_ip=" << g_new_ip
            << " netmask=" << g_netmask
            << " gateway=" << g_gateway
            << " immediate_status=" << status << std::endl;
  if (status != 0) {
    g_status = status;
    g_done = true;
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 3 || argc > 8) {
    std::cerr << "usage: " << argv[0]
              << " CURRENT_IP_OR_* NEW_IP [NETMASK] [GATEWAY] [HOST_IP] [SDK_CONFIG_PATH] [--reboot]\n";
    return 64;
  }

  g_current_ip = argv[1];
  g_new_ip = argv[2];
  g_netmask = argc > 3 ? argv[3] : "255.255.255.0";
  g_gateway = argc > 4 ? argv[4] : "192.168.1.1";
  const std::string host_ip = argc > 5 ? argv[5] : "192.168.1.50";
  const std::string sdk_config_path = argc > 6 ? argv[6] : "/home/vision/FAST-Calib/config/livox_mid360_fast_calib.json";
  g_reboot_after = argc > 7 && std::string(argv[7]) == "--reboot";

  DisableLivoxSdkConsoleLogger();

  if (!LivoxLidarSdkInit(sdk_config_path.c_str(), host_ip.c_str())) {
    std::cerr << "LivoxLidarSdkInit failed for host_ip=" << host_ip << std::endl;
    return 2;
  }
  SetLivoxLidarInfoChangeCallback(InfoChangeCallback, nullptr);
  if (!LivoxLidarSdkStart()) {
    std::cerr << "LivoxLidarSdkStart failed" << std::endl;
    LivoxLidarSdkUninit();
    return 3;
  }

  const auto start = std::chrono::steady_clock::now();
  while (!g_done) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed > std::chrono::seconds(20)) {
      std::cerr << "Timed out waiting for LiDAR discovery/IP-set callback" << std::endl;
      LivoxLidarSdkUninit();
      return g_command_sent ? 5 : 4;
    }
  }

  LivoxLidarSdkUninit();
  if (g_status == 0) {
    std::cout << "LiDAR IP change command succeeded. Power-cycle or restart the LiDAR if it does not reappear immediately." << std::endl;
    return 0;
  }
  std::cerr << "LiDAR IP change command failed with status=" << g_status << std::endl;
  return 6;
}
