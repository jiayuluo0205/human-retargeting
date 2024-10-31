#include "rebocap_ws_sdk_cpp.h"
#include <array>
#include <vector>
#include <fstream>

namespace rebocap {
RebocapWsSdk::RebocapWsSdk(CoordinateSpaceType coordinateSpaceType, bool globalQuat) : handle_(rebocap_ws_sdk_new(coordinateSpaceType, globalQuat), [](void *ptr) { rebocap_ws_sdk_release(ptr); }) {
  rebocap_ws_sdk_set_pose_msg_callback(handle_.get(), this, RebocapWsSdk::pose_msg_callback);
  rebocap_ws_sdk_set_exception_close_callback(handle_.get(), this, RebocapWsSdk::exception_close_callback);
}

int RebocapWsSdk::Open(uint32_t port, std::string_view name, int64_t uid) {
  return rebocap_ws_sdk_open(handle_.get(), port, name.data(), name.size(), uid);
}

void RebocapWsSdk::Close() { rebocap_ws_sdk_close(handle_.get()); }

void RebocapWsSdk::SetPoseMsgCallback(std::function<void(const struct QuatMsg *, RebocapWsSdk *)> &&func) {
  pose_msg_callback_ = std::move(func);
}

void RebocapWsSdk::SetExceptionCloseCallback(std::function<void(RebocapWsSdk *)> &&func) { exception_close_callback_ = std::move(func); }

void RebocapWsSdk::pose_msg_callback(const struct QuatMsg *msg, void *extra) {
  auto *that = reinterpret_cast<RebocapWsSdk *>(extra);
  that->PoseMsgCallback(msg);
  if (that->pose_msg_callback_) { that->pose_msg_callback_(msg, that); }
}

void RebocapWsSdk::exception_close_callback(void *extra) {
  auto *that = reinterpret_cast<RebocapWsSdk *>(extra);
  that->ExceptionCloseCallback();
  if (that->exception_close_callback_) { that->exception_close_callback_(that); }
}

int RebocapWsSdk::GetLastMsg(QuatMsg *msg) { return rebocap_ws_sdk_get_last_msg(handle_.get(), msg); }


inline std::vector<UV3f> convertToContinuous(const std::vector<std::array<float, 3>>& input) {
    std::vector<UV3f> output;
    output.reserve(input.size() * 3);
    for (const auto& arr : input) output.emplace_back(UV3f{.x=arr[0], .y=arr[1], .z=arr[2]});
    return output;
}

int RebocapWsSdk::CalculateAndRegisterToRebocap(std::vector<std::array<float, 3>> &left_vertex,
                                                std::vector<std::array<float, 3>> &left_normals,
                                                std::vector<std::array<float, 3>> &right_vertex,
                                                std::vector<std::array<float, 3>> &right_normals,
                                                std::vector<std::array<float, 3>> &skeletons,
                                                const std::string& coordinate,
                                                std::vector<std::array<float, 3>> &foot_vertices) {
    std::vector<UV3f> temp_left_vertex = convertToContinuous(left_vertex);
    std::vector<UV3f> temp_left_normals = convertToContinuous(left_normals);
    std::vector<UV3f> temp_right_vertex = convertToContinuous(right_vertex);
    std::vector<UV3f> temp_right_normals = convertToContinuous(right_normals);
    std::vector<UV3f> temp_skeletons = convertToContinuous(skeletons);

    int calculate_foot_vertices = foot_vertices.size() == 12 ? 0 : 1;
    std::vector<UV3f> out_foot_vertices = convertToContinuous(foot_vertices);
    if (calculate_foot_vertices) {
        foot_vertices.clear();
        out_foot_vertices.resize(12);
    }

    int res = rebocap_ws_sdk_calculate_foot_vertex(
            handle_.get(),
            temp_left_vertex.data(), temp_left_vertex.size(),
            temp_right_vertex.data(), temp_right_vertex.size(),
            temp_left_normals.data(), temp_left_normals.size(),
            temp_right_normals.data(), temp_right_normals.size(),
            temp_skeletons.data(), coordinate.c_str(),
            out_foot_vertices.data(), calculate_foot_vertices
            );
    if (calculate_foot_vertices) {
        for (int i = 0; i < 12; i++) {
            foot_vertices.emplace_back(std::array{out_foot_vertices[i].x, out_foot_vertices[i].y, out_foot_vertices[i].z});
        }
    }
    return res;

}
}// namespace rebocap