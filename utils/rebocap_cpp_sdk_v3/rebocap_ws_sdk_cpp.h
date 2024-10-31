#pragma once

#ifndef REBOCAP_EXPORT
#define REBOCAP_EXPORT
#endif
#include "rebocap_ws_sdk/rebocap_ws_sdk.h"
#include <functional>
#include <memory>
#include <string_view>

namespace rebocap {

class RebocapWsSdk {
 public:
  explicit RebocapWsSdk(CoordinateSpaceType coordinateSpaceType = DefaultCoordinate, bool globalQuat = false);

  int Open(uint32_t port, std::string_view name = "reborn_app", int64_t uid = 0);

  void Close();

  int GetLastMsg(QuatMsg *msg);

  void SetPoseMsgCallback(std::function<void(const struct QuatMsg *, RebocapWsSdk *)> &&func);
  void SetExceptionCloseCallback(std::function<void(RebocapWsSdk *)> &&func);

  int CalculateAndRegisterToRebocap(std::vector<std::array<float, 3>> &left_vertex,
                                    std::vector<std::array<float, 3>> &left_normals,
                                    std::vector<std::array<float, 3>> &right_vertex,
                                    std::vector<std::array<float, 3>> &right_normals,
                                    std::vector<std::array<float, 3>> &skeletons,
                                    const std::string& coordinate,
                                    std::vector<std::array<float, 3>> &foot_vertices);

 private:
  virtual void PoseMsgCallback(const struct QuatMsg *msg) {}
  virtual void ExceptionCloseCallback() {}

  std::shared_ptr<void> handle_;
  std::function<void(const struct QuatMsg *, RebocapWsSdk *)> pose_msg_callback_;
  std::function<void(RebocapWsSdk *)> exception_close_callback_;

  static void pose_msg_callback(const struct QuatMsg *, void *extra);
  static void exception_close_callback(void *extra);
};
}// namespace rebocap
