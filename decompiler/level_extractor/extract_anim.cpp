#include "extract_anim.h"

#include "common_formats.h"

#include "decompiler/ObjectFile/LinkedObjectFile.h"
#include "decompiler/util/goal_data_reader.h"

#include "third-party/lzokay/lzokay.hpp"

namespace decompiler {

static std::vector<u8> get_plain_data_bytes_up_to_label(const Ref& ref) {
  const auto& words = ref.data->words_by_seg.at(ref.seg);
  int start_word = ref.byte_offset / 4;
  std::vector<u8> result;
  for (int w = start_word; w < (int)words.size(); w++) {
    if (words[w].kind() != LinkedWord::PLAIN_DATA)
      break;
    for (int b = 0; b < 4; b++)
      result.push_back(words[w].get_byte(b));
  }
  return result;
}

static u32 read_u32(const u8* p) {
  u32 v;
  memcpy(&v, p, 4);
  return v;
}

static void parse_fixed_from_buf(const u8* p,
                                 level_tools::JointAnimCompressedFixed& fixed,
                                 u32 fixed_qwc) {
  memcpy(fixed.hdr.control_bits, p, sizeof(u32) * 14);
  fixed.hdr.num_joints = read_u32(p + 56);
  fixed.hdr.matrix_bits = read_u32(p + 60);
  fixed.offset_64 = read_u32(p + 64);
  fixed.offset_32 = read_u32(p + 68);
  fixed.offset_16 = read_u32(p + 72);
  fixed.reserved = read_u32(p + 76);

  fixed.data64_size = fixed.offset_32 - fixed.offset_64;
  fixed.data32_size = fixed.offset_16 - fixed.offset_32;
  int data16 = (int)((fixed_qwc - 5) * 16) - (int)fixed.data64_size - (int)fixed.data32_size;
  ASSERT(data16 >= 0);
  fixed.data16_size = (u16)data16;

  fixed.mat[0] = (fixed.hdr.matrix_bits & 1) == 0;
  fixed.mat[1] = (fixed.hdr.matrix_bits & 2) == 0;

  const u8* data = p + 80;  // data section starts after the 20-word (80-byte) header
  int d64 = (int)fixed.data64_size;
  int d32 = (int)fixed.data32_size;
  int d16 = (int)fixed.data16_size;

  fixed.data64.resize((d64 + 7) / 8);
  if (d64 > 0)
    memcpy(fixed.data64.data(), data + fixed.offset_64, d64);
  fixed.data32.resize((d32 + 3) / 4);
  if (d32 > 0)
    memcpy(fixed.data32.data(), data + fixed.offset_32, d32);
  fixed.data16.resize((d16 + 1) / 2);
  if (d16 > 0)
    memcpy(fixed.data16.data(), data + fixed.offset_16, d16);
}

static void parse_frame_from_buf(const u8* p,
                                 level_tools::JointAnimCompressedFrame& frame,
                                 u32 frame_qwc) {
  frame.offset_64 = read_u32(p + 0);
  frame.offset_32 = read_u32(p + 4);
  frame.offset_16 = read_u32(p + 8);
  frame.reserved = read_u32(p + 12);

  frame.data64_size = frame.offset_32 - frame.offset_64;
  frame.data32_size = frame.offset_16 - frame.offset_32;
  int data16 = (int)((frame_qwc - 1) * 16) - (int)frame.data64_size - (int)frame.data32_size;
  ASSERT(data16 >= 0);
  frame.data16_size = (u16)data16;

  const u8* data = p + 16;
  int fd64 = (int)frame.data64_size;
  int fd32 = (int)frame.data32_size;
  int fd16 = (int)frame.data16_size;

  frame.data64.resize((fd64 + 7) / 8);
  if (fd64 > 0)
    memcpy(frame.data64.data(), data + frame.offset_64, fd64);
  frame.data32.resize((fd32 + 3) / 4);
  if (fd32 > 0)
    memcpy(frame.data32.data(), data + frame.offset_32, fd32);
  frame.data16.resize((fd16 + 1) / 2);
  if (fd16 > 0)
    memcpy(frame.data16.data(), data + frame.offset_16, fd16);
}

void extract_animations(const ObjectFileData& ag_data,
                        const DecompilerTypeSystem& dts,
                        GameVersion version,
                        std::map<std::string, level_tools::ArtData>& out) {
  lg::info("extracting anims for ag {}", ag_data.name_in_dgo);
  auto locations = find_objects_with_type(ag_data.linked_data, "art-joint-anim");
  auto geos = find_objects_with_type(ag_data.linked_data, "art-joint-geo");
  TypedRef art;
  std::string mdl_name;
  // try to get the master art group name
  // some art groups have no geos (only animations), some have no animations
  if (!geos.empty()) {
    art = {Ref{&ag_data.linked_data, 0, geos[0] * 4}, dts.ts.lookup_type("art-joint-geo")};
    mdl_name = read_string_field(art, "name", dts, false);
  } else {
    art = {Ref{&ag_data.linked_data, 0, locations[0] * 4}, dts.ts.lookup_type("art-joint-anim")};
    // should work for most cases...
    mdl_name = read_string_field(art, "master-art-group-name", dts, false) + "-lod0";
  }
  if (!art.type) {
    lg::warn("extract_animations: art group {} has no geos or anims, skipping\n",
             ag_data.name_in_dgo);
    return;
  }
  auto& data = out[mdl_name];
  data.art_name = mdl_name;
  data.art_group_name = ag_data.name_in_dgo;
  // jak 2/3 split the first word into num-frames + flags.
  const bool has_flags = version != GameVersion::Jak1;
  for (auto loc : locations) {
    TypedRef ref(Ref{&ag_data.linked_data, 0, loc * 4}, dts.ts.lookup_type("art-joint-anim"));
    auto& anim = data.anims.emplace_back();
    anim.name = read_string_field(ref, "name", dts, false);
    lg::info("{}: extracting anim {}", ag_data.name_in_dgo, anim.name);
    anim.speed = read_plain_data_field<float>(ref, "speed", dts);
    anim.artist_base = read_plain_data_field<float>(ref, "artist-base", dts);
    anim.artist_step = read_plain_data_field<float>(ref, "artist-step", dts);
    Ref jacc = deref_label(get_field_ref(ref, "frames", dts));
    int jacc_word_off = 0;
    u32 first_word = deref_u32(jacc, jacc_word_off++);
    anim.frames.num_frames = has_flags ? (first_word & 0xFFFF) : first_word;
    anim.frames.fixed_qwc = deref_u32(jacc, jacc_word_off++);
    anim.frames.frame_qwc = deref_u32(jacc, jacc_word_off++);

    // jak 2/3 may lzo compress the animation, check the flag bit
    const bool lzo_compressed = has_flags && ((first_word >> 16) & 1) != 0;

    Ref fixed_ptr = jacc;
    fixed_ptr.byte_offset += jacc_word_off * 4;
    Ref fixed_ref = deref_label(fixed_ptr);

    if (lzo_compressed) {
      size_t decompressed_size =
          ((size_t)anim.frames.fixed_qwc + (size_t)anim.frames.num_frames * anim.frames.frame_qwc) *
          16;

      auto compressed = get_plain_data_bytes_up_to_label(fixed_ref);
      ASSERT(!compressed.empty());

      std::vector<u8> decompressed(decompressed_size);
      size_t out_size = 0;
      auto lzo_result = lzokay::decompress(compressed.data(), compressed.size(),
                                           decompressed.data(), decompressed.size(), out_size);
      ASSERT(lzo_result == lzokay::EResult::Success ||
             lzo_result == lzokay::EResult::InputNotConsumed);
      // ASSERT(out_size == decompressed_size);

      parse_fixed_from_buf(decompressed.data(), anim.frames.fixed, anim.frames.fixed_qwc);

      size_t frame_base = (size_t)anim.frames.fixed_qwc * 16;
      for (int i = 0; i < (int)anim.frames.num_frames; i++) {
        auto& frame = anim.frames.frame.emplace_back();
        parse_frame_from_buf(
            decompressed.data() + frame_base + (size_t)i * anim.frames.frame_qwc * 16, frame,
            anim.frames.frame_qwc);
      }
    } else {
      int fixed_word_off = 0;

      // fixed hdr
      memcpy_from_plain_data((u8*)anim.frames.fixed.hdr.control_bits, fixed_ref, 4 * 14);
      fixed_word_off += 14;
      anim.frames.fixed.hdr.num_joints = deref_u32(fixed_ref, fixed_word_off++);
      anim.frames.fixed.hdr.matrix_bits = deref_u32(fixed_ref, fixed_word_off++);
      anim.frames.fixed.offset_64 = deref_u32(fixed_ref, fixed_word_off++);
      anim.frames.fixed.offset_32 = deref_u32(fixed_ref, fixed_word_off++);
      anim.frames.fixed.offset_16 = deref_u32(fixed_ref, fixed_word_off++);
      anim.frames.fixed.reserved = deref_u32(fixed_ref, fixed_word_off++);

      anim.frames.fixed.data64_size = anim.frames.fixed.offset_32 - anim.frames.fixed.offset_64;
      anim.frames.fixed.data32_size = anim.frames.fixed.offset_16 - anim.frames.fixed.offset_32;
      {
        int data16 = (int)((anim.frames.fixed_qwc - 5) * 16) - (int)anim.frames.fixed.data64_size -
                     (int)anim.frames.fixed.data32_size;
        ASSERT(data16 >= 0);
        anim.frames.fixed.data16_size = (u16)data16;
      }

      // matrix flags
      anim.frames.fixed.mat[0] = (anim.frames.fixed.hdr.matrix_bits & 1) == 0;
      anim.frames.fixed.mat[1] = (anim.frames.fixed.hdr.matrix_bits & 2) == 0;

      fixed_ref.byte_offset += fixed_word_off * 4;

      int d64_bytes = (int)anim.frames.fixed.data64_size;
      int d32_bytes = (int)anim.frames.fixed.data32_size;
      int d16_bytes = (int)anim.frames.fixed.data16_size;

      anim.frames.fixed.data64.resize((d64_bytes + 7) / 8);
      if (d64_bytes > 0) {
        Ref d64 = fixed_ref;
        d64.byte_offset += anim.frames.fixed.offset_64;
        memcpy_from_plain_data((u8*)anim.frames.fixed.data64.data(), d64, d64_bytes);
      }
      anim.frames.fixed.data32.resize((d32_bytes + 3) / 4);
      if (d32_bytes > 0) {
        Ref d32 = fixed_ref;
        d32.byte_offset += anim.frames.fixed.offset_32;
        memcpy_from_plain_data((u8*)anim.frames.fixed.data32.data(), d32, d32_bytes);
      }
      anim.frames.fixed.data16.resize((d16_bytes + 1) / 2);
      if (d16_bytes > 0) {
        Ref d16 = fixed_ref;
        d16.byte_offset += anim.frames.fixed.offset_16;
        memcpy_from_plain_data((u8*)anim.frames.fixed.data16.data(), d16, d16_bytes);
      }

      Ref frames_ref = jacc;
      frames_ref.byte_offset += 16;
      for (int i = 0; i < (int)anim.frames.num_frames; i++) {
        Ref frame_ref = deref_label(frames_ref);
        int frame_off = 0;
        auto& frame = anim.frames.frame.emplace_back();

        frame.offset_64 = deref_u32(frame_ref, frame_off++);
        frame.offset_32 = deref_u32(frame_ref, frame_off++);
        frame.offset_16 = deref_u32(frame_ref, frame_off++);
        frame.reserved = deref_u32(frame_ref, frame_off++);

        frame.data64_size = frame.offset_32 - frame.offset_64;
        frame.data32_size = frame.offset_16 - frame.offset_32;
        {
          int data16 = (int)((anim.frames.frame_qwc - 1) * 16) - (int)frame.data64_size -
                       (int)frame.data32_size;
          ASSERT(data16 >= 0);
          frame.data16_size = (u16)data16;
        }

        Ref frame_data = frame_ref;
        frame_data.byte_offset += frame_off * 4;

        int fd64_bytes = (int)frame.data64_size;
        int fd32_bytes = (int)frame.data32_size;
        int fd16_bytes = (int)frame.data16_size;

        frame.data64.resize((fd64_bytes + 7) / 8);
        if (fd64_bytes > 0) {
          Ref fd64 = frame_data;
          fd64.byte_offset += frame.offset_64;
          memcpy_from_plain_data((u8*)frame.data64.data(), fd64, fd64_bytes);
        }
        frame.data32.resize((fd32_bytes + 3) / 4);
        if (fd32_bytes > 0) {
          Ref fd32 = frame_data;
          fd32.byte_offset += frame.offset_32;
          memcpy_from_plain_data((u8*)frame.data32.data(), fd32, fd32_bytes);
        }
        frame.data16.resize((fd16_bytes + 1) / 2);
        if (fd16_bytes > 0) {
          Ref fd16 = frame_data;
          fd16.byte_offset += frame.offset_16;
          memcpy_from_plain_data((u8*)frame.data16.data(), fd16, fd16_bytes);
        }

        frames_ref.byte_offset += 4;
      }
    }
  }
}
}  // namespace decompiler
