#include "rebocap_ws_sdk_cpp.h"
#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include "nlohmann/json.hpp"

void parseVertexData(const std::string& filePath,
               std::vector<std::array<float, 3>>& all_left_vertex,
               std::vector<std::array<float, 3>>& all_left_normals,
               std::vector<std::array<float, 3>>& all_right_vertex,
               std::vector<std::array<float, 3>>& all_right_normals,
               std::vector<std::array<float, 3>>& skeletons) {
    // 读取json文件
    std::ifstream file(filePath);
    nlohmann::json j;
    file >> j;

    // 解析left_foot_vertex
    for (const auto& item : j["left_foot_vertex"].items()) {
        auto temp_v = item.value().get<std::vector<std::array<float, 3>>>();
        all_left_vertex.push_back(temp_v[0]);
        all_left_normals.push_back(temp_v[1]);
    }

    // 解析right_foot_vertex
    for (const auto& item : j["right_foot_vertex"].items()) {
        auto temp_v = item.value().get<std::vector<std::array<float, 3>>>();
        all_right_vertex.push_back(temp_v[0]);
        all_right_normals.push_back(temp_v[1]);
    }

    // 解析skeleton
    for (const auto& array : j["skeleton"]) {
        skeletons.push_back({array[0], array[1], array[2]});
    }
}

int main() {
  rebocap::RebocapWsSdk sdk(DefaultCoordinate, true);
  static int counter = 0;
  sdk.SetPoseMsgCallback([](const QuatMsg *msg, rebocap::RebocapWsSdk *self) {
      if (counter % 60 == 0) {
          std::cout << "trans=[" << msg->trans[0] << "," << msg->trans[1] << "," << msg->trans[2] << "]" << std::endl;
          std::cout << "quat:\n";
          for (size_t i = 0; i < 24; i++) {
            auto s_id = i * 4;
            std::cout << "id[" << i << "]--" << "w:" << msg->quat[s_id+3]
                      << ", x:" << msg->quat[s_id] << ", y:" << msg->quat[s_id + 1] << ", z:" << msg->quat[s_id + 2] << "\n";
          }
          std::cout << std::endl;
          std::cout << "static_index=" << static_cast<int>(msg->static_index) << std::endl;
          std::cout << "tp=" << msg->tp << std::endl;
      }
    counter++;

  });
  sdk.SetExceptionCloseCallback([](rebocap::RebocapWsSdk *self) { std::cerr << "exception_close_callback" << std::endl; });
  int open_res = sdk.Open(7690);
  if (open_res == 0) {
    std::cout << "连接成功" << std::endl;
    // register skeleton data to rebocap
      std::vector<std::array<float, 3>> left_vertex;
      std::vector<std::array<float, 3>> left_normals;
      std::vector<std::array<float, 3>> right_vertex;
      std::vector<std::array<float, 3>> right_normals;
      std::vector<std::array<float, 3>> skeletons;
      parseVertexData("sample_data\\avatar_parsed_data.json", left_vertex, left_normals, right_vertex, right_normals, skeletons);

      std::vector<std::array<float, 3>> foot_vertices;
      // load data axis is:  x: right  y: up  z: forward   target axis is:  x: left  y: up  z: backward
      // axis is relative to avatar, for example, if avatar in t-pose, it's right hand x is 1.0, then x is right
      int res = sdk.CalculateAndRegisterToRebocap(left_vertex, left_normals, right_vertex, right_normals,
                                        skeletons, "-xy-z", foot_vertices);
      if (res == 0) {
          std::cout << "register to rebocap suc!, please check rebocap console or information on ui\n";
      } else if (res == -9) {
          std::cerr << "register failed, coordinate parse error!";
      } else {
          std::cerr << "register failed, other error!";
      }
  } else {
    std::cerr << "连接失败" << open_res << std::endl;
    if (open_res == 1) {
      std::cerr << "连接状态错误" << std::endl;
    } else if (open_res == 2) {
      std::cerr << "连接失败" << std::endl;
    } else if (open_res == 3) {
      std::cerr << "认证失败" << std::endl;
    } else {
      std::cerr << "未知错误" << open_res << std::endl;
    }
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  QuatMsg msg{};
  int get_last_msg_ret = sdk.GetLastMsg(&msg);
  if (get_last_msg_ret == 0) {
    std::cerr << "GetLastMsg 成功" << std::endl;
  } else {
    std::cerr << "GetLastMsg 失败" << std::endl;
  }

  std::this_thread::sleep_for(std::chrono::seconds(10));

  sdk.Close();
  std::cerr << "Close" << std::endl;
  return 0;
}
