#pragma once

#include "extract_anim.h"

#include "common_formats.h"

#include "decompiler/util/goal_data_reader.h"

namespace decompiler {
void extract_animations(const ObjectFileData& ag_data,
                        const DecompilerTypeSystem& dts,
                        GameVersion /*version*/,
                        std::map<std::string, level_tools::ArtData>& out) {
  auto locations = find_objects_with_type(ag_data.linked_data, "art-joint-anim");
  // dump all anims into the first model that is found
  TypedRef mdl(Ref{&ag_data.linked_data, 0,
                   find_objects_with_type(ag_data.linked_data, "art-joint-geo")[0] * 4},
               dts.ts.lookup_type("art-joint-geo"));
  auto mdl_name = read_string_field(mdl, "name", dts, false);
  auto& data = out[mdl_name];
  data.art_name = mdl_name;
  data.art_group_name = ag_data.name_in_dgo;
  for (auto loc : locations) {
    TypedRef ref(Ref{&ag_data.linked_data, 0, loc * 4}, dts.ts.lookup_type("art-joint-anim"));
    auto& anim = data.anims.emplace_back();
    anim.name = read_string_field(ref, "name", dts, false);
    anim.speed = read_plain_data_field<float>(ref, "speed", dts);
    anim.artist_base = read_plain_data_field<float>(ref, "artist-base", dts);
    anim.artist_step = read_plain_data_field<float>(ref, "artist-step", dts);
    Ref jacc = deref_label(get_field_ref(ref, "frames", dts));
    int jacc_word_off = 0;
    anim.frames.num_frames = deref_u32(jacc, jacc_word_off++);
    anim.frames.fixed_qwc = deref_u32(jacc, jacc_word_off++);
    anim.frames.frame_qwc = deref_u32(jacc, jacc_word_off++);
    // fixed start
    Ref fixed_ptr = jacc;
    fixed_ptr.byte_offset += jacc_word_off * 4;
    Ref fixed_ref = deref_label(fixed_ptr);
    int fixed_word_off = 0;
    // hdr start
    memcpy_from_plain_data((u8*)anim.frames.fixed.hdr.control_bits, fixed_ref, 4 * 14);
    fixed_word_off += 14;
    anim.frames.fixed.hdr.num_joints = deref_u32(fixed_ref, fixed_word_off++);
    anim.frames.fixed.hdr.matrix_bits = deref_u32(fixed_ref, fixed_word_off++);
    // hdr end
    anim.frames.fixed.offset_64 = deref_u32(fixed_ref, fixed_word_off++);
    anim.frames.fixed.offset_32 = deref_u32(fixed_ref, fixed_word_off++);
    anim.frames.fixed.offset_16 = deref_u32(fixed_ref, fixed_word_off++);
    anim.frames.fixed.reserved = deref_u32(fixed_ref, fixed_word_off++);
    auto qwc = 5;
    anim.frames.fixed.data64_size = anim.frames.fixed.offset_32 - anim.frames.fixed.offset_64;
    qwc += anim.frames.fixed.data64_size / 16;
    anim.frames.fixed.data32_size = anim.frames.fixed.offset_16 - anim.frames.fixed.offset_32;
    qwc += anim.frames.fixed.data32_size / 16;
    anim.frames.fixed.data16_size = (anim.frames.fixed_qwc - qwc) * 16;
    qwc += anim.frames.fixed.data16_size / 16;
    ASSERT(qwc == anim.frames.fixed_qwc);
    fixed_ref.byte_offset += fixed_word_off * 4;
    auto has_fixed_align = (anim.frames.fixed.hdr.matrix_bits & 1) == 0;
    auto has_fixed_prejoint = (anim.frames.fixed.hdr.matrix_bits & 2) == 0;
    int bytes_copied = 0;
    if (has_fixed_align) {
      anim.frames.fixed.mat[0] = true;
      memcpy_from_plain_data((u8*)anim.frames.fixed.mats[0].data(), fixed_ref, sizeof(float) * 16);
      bytes_copied += 64;
      fixed_ref.byte_offset += 64;
    }
    if (has_fixed_prejoint) {
      anim.frames.fixed.mat[1] = true;
      memcpy_from_plain_data((u8*)anim.frames.fixed.mats[1].data() + bytes_copied, fixed_ref,
                             sizeof(float) * 16);
      bytes_copied += 64;
      fixed_ref.byte_offset += 64;
    }
    // data64
    memcpy_from_plain_data((u8*)anim.frames.fixed.data64.data() + bytes_copied, fixed_ref,
                           anim.frames.fixed.data64_size - bytes_copied);
    bytes_copied += anim.frames.fixed.offset_32 - bytes_copied;
    fixed_ref.byte_offset += anim.frames.fixed.offset_32 - bytes_copied;
    // data32
    memcpy_from_plain_data((u8*)anim.frames.fixed.data32.data() + bytes_copied, fixed_ref,
                           anim.frames.fixed.data32_size);
    bytes_copied += anim.frames.fixed.data32_size;
    fixed_ref.byte_offset += anim.frames.fixed.data32_size;
    // data16
    memcpy_from_plain_data((u8*)anim.frames.fixed.data16.data() + bytes_copied, fixed_ref,
                           anim.frames.fixed.data16_size);
    bytes_copied += anim.frames.fixed.data16_size;
    fixed_ref.byte_offset += anim.frames.fixed.data16_size;
    // fixed end
    ASSERT(bytes_copied == (anim.frames.fixed_qwc - 5) * 16);
    Ref frames_ref = jacc;
    frames_ref.byte_offset += 16;
    for (int i = 0; i < anim.frames.num_frames; i++) {
      Ref frame_ref = deref_label(frames_ref);
      int frame_off = 0;
      auto& frame = anim.frames.frame.emplace_back();
      frame.offset_64 = deref_u32(frame_ref, frame_off++);
      frame.offset_32 = deref_u32(frame_ref, frame_off++);
      frame.offset_16 = deref_u32(frame_ref, frame_off++);
      frame.reserved = deref_u32(frame_ref, frame_off++);
      auto frame_qwc = 1;
      frame.data64_size = frame.offset_32 - frame.offset_64;
      frame_qwc += frame.data64_size / 16;
      frame.data32_size = frame.offset_16 - frame.offset_32;
      frame_qwc += frame.data32_size / 16;
      frame.data16_size = (anim.frames.frame_qwc - frame_qwc) * 16;
      frame_qwc += frame.data16_size / 16;
      ASSERT(frame_qwc == anim.frames.frame_qwc);
      int frame_bytes_copied = 0;
      if (!has_fixed_align) {
        memcpy_from_plain_data((u8*)frame.mats[0].data(), frame_ref, sizeof(float) * 16);
        frame_bytes_copied += 64;
        frame_ref.byte_offset += 64;
      }
      if (!has_fixed_prejoint) {
        memcpy_from_plain_data((u8*)frame.mats[1].data() + frame_bytes_copied, frame_ref,
                               sizeof(float) * 16);
        frame_bytes_copied += 64;
        frame_ref.byte_offset += 64;
      }
      // data64
      memcpy_from_plain_data((u8*)frame.data64.data() + frame_bytes_copied, frame_ref,
                             frame.data64_size - frame_bytes_copied);
      frame_bytes_copied += frame.offset_32 - frame_bytes_copied;
      frame_ref.byte_offset += frame.offset_32 - frame_bytes_copied;
      // data32
      memcpy_from_plain_data((u8*)frame.data32.data() + frame_bytes_copied, frame_ref,
                             frame.data32_size);
      frame_bytes_copied += frame.data32_size;
      frame_ref.byte_offset += frame.data32_size;
      // data16
      memcpy_from_plain_data((u8*)frame.data16.data() + frame_bytes_copied, frame_ref,
                             frame.data16_size);
      frame_bytes_copied += frame.data16_size;
      frame_ref.byte_offset += frame.data16_size;
      // frame end
      ASSERT(frame_bytes_copied == anim.frames.frame_qwc * 16);
      frames_ref.byte_offset += 4;
    }
  }
}
}  // namespace decompiler