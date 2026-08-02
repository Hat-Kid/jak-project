#pragma once

#include <string>
#include <vector>

#include "common/common_types.h"
#include "common/math/Vector.h"

#include "third-party/tiny_gltf/tiny_gltf.h"

namespace anim {

struct UncompressedSingleJointAnim {
  std::vector<math::Vector3f> trans_frames;
  std::vector<math::Vector3f> scale_frames;
  std::vector<math::Vector4f> quat_frames;
};

struct UncompressedJointAnim {
  std::string name;
  std::string master_art_group_name;
  int master_art_group_index = -1;
  std::vector<UncompressedSingleJointAnim> joints;
  float framerate = 60;
  float artist_base = 0;
  float artist_frame_rate = 60;
  int frames = 0;
};

struct CompressedMatrixMetadata {
  bool is_animated[2];
};

struct CompressedFrame {
  std::vector<u16> data16;
  std::vector<u32> data32;
  std::vector<u64> data64;

  int size_bytes() const { return data16.size() * 2 + data32.size() * 4 + data64.size() * 8; }
};

struct CompressedJointMetadata {
  bool animated_trans = false;
  bool animated_quat = false;
  bool animated_scale = false;
  bool big_trans_mode = false;
};

struct CompressedAnim {
  std::string name;
  std::string master_art_group_name;
  int master_art_group_index = -1;
  CompressedFrame fixed;
  std::vector<CompressedFrame> frames;
  bool matrix_animated[2] = {false, false};
  std::vector<CompressedJointMetadata> joint_metadata;
  float framerate = 60;
  float artist_base = 0;
  float artist_frame_rate = 60;
};

/*!
 * The num-funcs step frame-num by (requested speed * the art-joint-anim's speed) once per tick, so
 * an animation sampled at 15 fps has to advance a quarter of a frame per tick to last as long as it
 * did in the source file.
 */
inline float playback_speed_for_framerate(float framerate) {
  return framerate > 0.f ? framerate / 60.f : 1.f;
}

/*!
 * ja-aframe divides an artist frame number by artist-step to get a stored frame, so artist-step is
 * how many artist frames one stored frame covers. The original animations were authored at 15, 30 or
 * 60 fps depending on the animation; 60 is the sane guess when we have nothing better.
 */
inline float artist_step_for_framerate(float framerate, float artist_frame_rate) {
  return (framerate > 0.f && artist_frame_rate > 0.f) ? artist_frame_rate / framerate : 1.f;
}

/*!
 * Load animation data from GLTF file.
 * @param model The GLTF model containing the animation
 * @param anim The animation to convert
 * @param node_to_joint Mapping from GLTF node index to the joint index
 * @param master_art_group The master art group to link this animation to, if set.
 * @param master_art_group_index The index of the slot in the master art group to link this
 * animation to.
 * @param framerate Number of key-frames per second. (this doesn't have to match frame rate, the
 * game will interpolate between keyframes as needed.)
 */
UncompressedJointAnim extract_anim_from_gltf(const tinygltf::Model& model,
                                             const tinygltf::Animation& anim,
                                             const std::map<int, int>& node_to_joint,
                                             const std::string& master_art_group,
                                             int master_art_group_index,
                                             float framerate);

CompressedAnim compress_animation(const UncompressedJointAnim& in);

}  // namespace anim