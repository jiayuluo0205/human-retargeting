#pragma once

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

#if (defined REBOCAP_EXPORT && defined __cplusplus)
extern "C" {
#endif

#ifdef REBOCAP_EXPORT
#ifdef WIN32
#define REBROCAP_DLL_EXPORT __declspec(dllexport)
#else
#define REBROCAP_DLL_EXPORT
#endif
#else
#define REBROCAP_DLL_EXPORT
#endif

typedef enum {
  DefaultCoordinate = 0,
  UnityCoordinate = 1,
  BlenderCoordinate = 2,
  MayaCoordinate = 3,
  MaxCoordinate = 4,
  UECoordinate = 5,
} CoordinateSpaceType;


struct QuatMsg {
  float trans[3];
  float quat[96];
  int8_t static_index;
  uint64_t tp;
};

struct UV3f {
  float x;
  float y;
  float z;
};

REBROCAP_DLL_EXPORT void *rebocap_ws_sdk_new(CoordinateSpaceType coordinate = DefaultCoordinate, uint8_t globalQuat = 0);

REBROCAP_DLL_EXPORT void rebocap_ws_sdk_release(void *handle);

REBROCAP_DLL_EXPORT int rebocap_ws_sdk_open(void *handle, uint32_t port, const char *name, size_t name_len, int64_t uid);

REBROCAP_DLL_EXPORT void rebocap_ws_sdk_close(void *handle);

REBROCAP_DLL_EXPORT void rebocap_ws_sdk_set_pose_msg_callback(void *handle, void *extra, void (*func)(const struct QuatMsg *, void *extra));

REBROCAP_DLL_EXPORT void rebocap_ws_sdk_set_exception_close_callback(void *handle, void *extra, void (*func)(void *extra));

REBROCAP_DLL_EXPORT int rebocap_ws_sdk_get_last_msg(void *handle, QuatMsg *msg);

REBROCAP_DLL_EXPORT int rebocap_ws_sdk_calculate_foot_vertex(void *handle,
                                                             UV3f* leftFootV, size_t sleftFootV,
                                                             UV3f* rightFootV, size_t srightFootV,
                                                             UV3f* leftFootNormV, size_t sleftFootNormV,
                                                             UV3f* rightFootNormV, size_t srightFootNormV,
                                                             UV3f* skeleton, const char* coordinate_change,
                                                             UV3f* foot_vertex, int calculate_vertex
                                                             );
REBROCAP_DLL_EXPORT int rebocap_ws_sdk_calculate_foot_vertex_il2cpp(void *handle,
                                                             const char* leftFootV,
                                                             const char* rightFootV,
                                                             const char* leftFootNormV,
                                                             const char* rightFootNormV,
                                                             const char* skeleton, const char* coordinate_change,
                                                             const char* foot_vertex, int calculate_vertex
);

#if (defined REBOCAP_EXPORT && defined __cplusplus)
}
#endif