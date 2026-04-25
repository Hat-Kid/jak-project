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

  const u8* data = p + 80;
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
  // lg::info("extracting anims for ag {}", ag_data.name_in_dgo);
  // ban animations that have very weird data layouts due to being compressed
  PerGameVersion<std::vector<std::string>> banned_anims = {{},
                                                           {"jakb-walk",
                                                            "jakb-edge-grab-stance0",
                                                            "jakb-deatha",
                                                            "jakb-slide-right",
                                                            "jakb-slide-left",
                                                            "jakb-wall-hide-scared-return",
                                                            "jakb-wall-hide-head",
                                                            "jakb-wall-hide-head-left",
                                                            "jakb-wall-hide-head-right",
                                                            "jakb-wall-hide-body",
                                                            "jakb-fuel-cell-victory-9",
                                                            "daxter-edge-grab-stance0",
                                                            "daxter-slide-right",
                                                            "daxter-slide-left",
                                                            "daxter-wall-hide-head",
                                                            "daxter-wall-hide-head-right",
                                                            "daxter-fuel-cell-victory-9",
                                                            "daxter-attack-from-jump-loop",
                                                            "daxter-flop-down-loop",
                                                            "daxter-moving-flop-down-loop",
                                                            "daxter-hit-up",
                                                            "daxter-gun-jump-land",
                                                            "daxter-gun-jump-land-side",
                                                            "daxter-gun-flop-down",
                                                            "daxter-gun-flop-down-loop",
                                                            "jakb-gun-stance-blue",
                                                            "jakb-gun-duck-roll",
                                                            "jakb-gun-flop-down",
                                                            "daxter-board-stance",
                                                            "daxter-board-flip-forward-loop",
                                                            "daxter-board-kickflip-c",
                                                            "jakb-board-stance",
                                                            "jakb-board-kickflip-c",
                                                            "daxter-darkjak-attack-combo3-end",
                                                            "daxter-darkjak-attack-ice-loop2"},
                                                           {},
                                                           {}};
  auto ja_locs = find_objects_with_type(ag_data.linked_data, "art-joint-anim");
  if (ja_locs.empty()) {
    lg::warn("extract_animations: art group {} has no anims, skipping\n", ag_data.name_in_dgo);
    return;
  }
  // jak 2/3 split the first word into num-frames + flags
  const bool has_flags = version != GameVersion::Jak1;
  for (auto loc : ja_locs) {
    TypedRef ref(Ref{&ag_data.linked_data, 0, loc * 4}, dts.ts.lookup_type("art-joint-anim"));
    auto master_art_name = read_string_field(ref, "master-art-group-name", dts, false);
    level_tools::ArtJointAnim ja;
    ja.name = read_string_field(ref, "name", dts, false);
    if (std::ranges::find(banned_anims[version], ja.name) != banned_anims[version].end()) {
      lg::info("skipping banned anim {} for {}", ja.name, ag_data.name_in_dgo);
      continue;
    }
    ja.speed = read_plain_data_field<float>(ref, "speed", dts);
    ja.artist_base = read_plain_data_field<float>(ref, "artist-base", dts);
    ja.artist_step = read_plain_data_field<float>(ref, "artist-step", dts);
    Ref jacc = deref_label(get_field_ref(ref, "frames", dts));
    int jacc_word_off = 0;
    u32 first_word = deref_u32(jacc, jacc_word_off++);
    ja.frames.num_frames = has_flags ? (first_word & 0xffff) : first_word;
    ja.frames.fixed_qwc = deref_u32(jacc, jacc_word_off++);
    ja.frames.frame_qwc = deref_u32(jacc, jacc_word_off++);

    // jak 2/3 may lzo compress the animation, check the flag bit
    const bool lzo_compressed = has_flags && ((first_word >> 16) & 1) != 0;
    lg::info("{}: extracting anim {} (compressed {})", ag_data.name_in_dgo, ja.name,
             lzo_compressed);

    Ref fixed_ptr = jacc;
    fixed_ptr.byte_offset += jacc_word_off * 4;
    Ref fixed_ref = deref_label(fixed_ptr);

    if (lzo_compressed) {
      size_t decompressed_size =
          ((size_t)ja.frames.fixed_qwc + (size_t)ja.frames.num_frames * ja.frames.frame_qwc) * 16;

      // the lzo stream may be split across non-contiguous labels in the art groups so we have to go
      // label by label and can't just memcpy the assumed size
      // on top of that, there is not really a way to know when and where the stream ends until we
      // encounter the lzo terminating sequence 0x11 0x00 0x00
      // on top of THAT, some of the labels can just have a wrong size or point to a
      // completely different random thing
      std::vector<u8> compressed = {};
      size_t read_bytes_total = 0;
      compressed.reserve(decompressed_size);
      // std::vector<u8> fixed_bytes = bytes_from_plain_data(fixed_ref, ja.frames.fixed_qwc * 16);
      // compressed.insert(compressed.end(), fixed_bytes.begin(), fixed_bytes.end());
      // read_bytes_total += ja.frames.fixed_qwc * 16;
      {
        auto first = fixed_ref.data->words_by_seg.at(fixed_ref.seg).at((fixed_ref.byte_offset) / 4);
        lg::info("fixed first word: 0x{:x}", first.data);
        auto fixed_label = ag_data.linked_data.get_label_at(fixed_ref.seg, fixed_ref.byte_offset);
        lg::info("fixed label {}", fixed_ref.data->labels.at(fixed_label).name);
        size_t read_bytes_fixed = 0;
        for (int b = 0; b < ja.frames.fixed_qwc * 16; b++) {
          u8 byte = deref_u8(fixed_ref, b);
          read_bytes_total += sizeof(u8);
          read_bytes_fixed += sizeof(u8);
          compressed.push_back(byte);
          auto next_word = b;
          while (next_word % 4) {
            next_word += 1;
          }
          // auto label_at =
          //     ag_data.linked_data.get_label_at(fixed_ref.seg, fixed_ref.byte_offset + next_word);
          // if (label_at != -1 && label_at != fixed_label && b < ja.frames.fixed_qwc * 16 - 4) {
          //   lg::info("hit an unexpected label {} in fixed, breaking...",
          //            fixed_ref.data->labels.at(label_at).name);
          //   break;
          // }
        }
        if (read_bytes_fixed != ja.frames.fixed_qwc * 16) {
          lg::warn("read {} bytes for fixed (total should be {} qw/{} bytes)", read_bytes_fixed,
                   ja.frames.fixed_qwc, ja.frames.fixed_qwc * 16);
        } else {
          lg::info("read {} bytes for fixed", read_bytes_fixed);
        }
      }

      Ref frame_ptr_ref = jacc;
      frame_ptr_ref.byte_offset += 16;
      bool hit_end_of_compressed_stream = false;
      for (int i = 0; i < (int)ja.frames.num_frames; i++) {
        Ref frame_ref = deref_label(frame_ptr_ref);
        const auto& frame_label = frame_ptr_ref.data->words_by_seg.at(frame_ptr_ref.seg)
                                      .at(frame_ptr_ref.byte_offset / 4)
                                      .label_id();
        lg::info("frame {} label {}", i, frame_ptr_ref.data->labels.at(frame_label).name);
        lg::info(
            "first word: 0x{:x}",
            frame_ref.data->words_by_seg.at(frame_ref.seg).at((frame_ref.byte_offset) / 4).data);
        std::vector<u8> chunk;
        std::deque<u8> last_three;
        size_t read_bytes_frame = 0;
        for (int b = 0; b < ja.frames.frame_qwc * 16; b++) {
          if (hit_end_of_compressed_stream) {
            break;
          }
          auto label_at =
              ag_data.linked_data.get_label_at(frame_ref.seg, frame_ref.byte_offset + (b + 4) & ~3);
          if (frame_ref.byte_offset % 4 != 0 && label_at != -1 && label_at != frame_label) {
            lg::info("hit an unexpected label {}, breaking...",
                     frame_ref.data->labels.at(label_at).name);
            break;
          }
          u8 byte = deref_u8(frame_ref, b);
          read_bytes_total += sizeof(u8);
          read_bytes_frame += sizeof(u8);
          last_three.push_back(byte);
          if (last_three.size() >= 3) {
            last_three.pop_front();
            last_three.resize(3);
            // lg::info("last three: 0x{:x} 0x{:x} 0x{:x}", last_three[0], last_three[1],
            //          last_three[2]);
          }
          chunk.push_back(byte);
          // if we hit the lzo terminating sequence, go forward until we hit the next label
          if (last_three.size() >= 3 && last_three[0] == 0x11 && last_three[1] == 0x00 &&
              last_three[2] == 0x00) {
            lg::info("found lzo terminating sequence");
            auto last = b;
            while (last % 4) {
              last -= 1;
            }
            lg::info("last word: 0x{:x}", frame_ref.data->words_by_seg.at(frame_ref.seg)
                                              .at((frame_ref.byte_offset + last) / 4)
                                              .data);
            auto next = b;
            while (next % 4) {
              next += 1;
            }
            auto next_word = frame_ref.data->words_by_seg.at(frame_ref.seg)
                                 .at((frame_ref.byte_offset + next) / 4);
            lg::info("next word: 0x{:x} kind {}", next_word.data, (int)next_word.kind());
            if (next_word.kind() != LinkedWord::PLAIN_DATA) {
              hit_end_of_compressed_stream = true;
              break;
            }
            while (next_word.data == 0x0) {
              lg::info("checking for padding...");
              next += 4;
              next_word = frame_ref.data->words_by_seg.at(frame_ref.seg)
                              .at((frame_ref.byte_offset + next) / 4);
              lg::info("next+1 word: 0x{:x} kind {}", next_word.data, (int)next_word.kind());
              if (next_word.kind() != LinkedWord::PLAIN_DATA) {
                hit_end_of_compressed_stream = true;
                break;
              }
              auto label_next =
                  ag_data.linked_data.get_label_at(frame_ref.seg, frame_ref.byte_offset + next);
              if ((frame_ref.byte_offset + next) % 4 != 0 && label_next != -1 &&
                  label_next != frame_label) {
                lg::info("hit next label {}, breaking...",
                         frame_ref.data->labels.at(label_next).name);
                hit_end_of_compressed_stream = true;
                break;
              }
            }
          }
        }
        compressed.insert(compressed.end(), chunk.begin(), chunk.end());
        lg::info("read {} bytes for frame {}", read_bytes_frame, i);
        frame_ptr_ref.byte_offset += 4;
        if (hit_end_of_compressed_stream) {
          break;
        }
      }

      std::vector<u8> decompressed(decompressed_size);
      size_t out_size = 0;
      auto lzo_result = lzokay::decompress(compressed.data(), compressed.size(),
                                           decompressed.data(), decompressed.size(), out_size);
      ASSERT(lzo_result == lzokay::EResult::Success ||
             lzo_result == lzokay::EResult::InputNotConsumed);
      if (out_size != decompressed_size) {
        lg::warn("lzo decomp size mismatch for '{}' in '{}': got {} bytes, expected {}", ja.name,
                 ag_data.name_in_dgo, out_size, decompressed_size);
      }

      parse_fixed_from_buf(decompressed.data(), ja.frames.fixed, ja.frames.fixed_qwc);

      size_t frame_base = (size_t)ja.frames.fixed_qwc * 16;
      for (int i = 0; i < (int)ja.frames.num_frames; i++) {
        auto& frame = ja.frames.frame.emplace_back();
        parse_frame_from_buf(
            decompressed.data() + frame_base + (size_t)i * ja.frames.frame_qwc * 16, frame,
            ja.frames.frame_qwc);
      }
    } else {
      int fixed_word_off = 0;

      // fixed hdr
      memcpy_from_plain_data((u8*)ja.frames.fixed.hdr.control_bits, fixed_ref, sizeof(u32) * 14);
      fixed_word_off += 14;
      ja.frames.fixed.hdr.num_joints = deref_u32(fixed_ref, fixed_word_off++);
      ja.frames.fixed.hdr.matrix_bits = deref_u32(fixed_ref, fixed_word_off++);
      ja.frames.fixed.offset_64 = deref_u32(fixed_ref, fixed_word_off++);
      ja.frames.fixed.offset_32 = deref_u32(fixed_ref, fixed_word_off++);
      ja.frames.fixed.offset_16 = deref_u32(fixed_ref, fixed_word_off++);
      ja.frames.fixed.reserved = deref_u32(fixed_ref, fixed_word_off++);

      ja.frames.fixed.data64_size = ja.frames.fixed.offset_32 - ja.frames.fixed.offset_64;
      ja.frames.fixed.data32_size = ja.frames.fixed.offset_16 - ja.frames.fixed.offset_32;
      {
        int data16 = (int)((ja.frames.fixed_qwc - 5) * 16) - (int)ja.frames.fixed.data64_size -
                     (int)ja.frames.fixed.data32_size;
        ASSERT(data16 >= 0);
        ja.frames.fixed.data16_size = (u16)data16;
      }

      // matrix flags
      ja.frames.fixed.mat[0] = (ja.frames.fixed.hdr.matrix_bits & 1) == 0;
      ja.frames.fixed.mat[1] = (ja.frames.fixed.hdr.matrix_bits & 2) == 0;

      fixed_ref.byte_offset += fixed_word_off * 4;

      int d64_bytes = (int)ja.frames.fixed.data64_size;
      int d32_bytes = (int)ja.frames.fixed.data32_size;
      int d16_bytes = (int)ja.frames.fixed.data16_size;

      ja.frames.fixed.data64.resize((d64_bytes + 7) / 8);
      if (d64_bytes > 0) {
        Ref d64 = fixed_ref;
        d64.byte_offset += ja.frames.fixed.offset_64;
        memcpy_from_plain_data((u8*)ja.frames.fixed.data64.data(), d64, d64_bytes);
      }
      ja.frames.fixed.data32.resize((d32_bytes + 3) / 4);
      if (d32_bytes > 0) {
        Ref d32 = fixed_ref;
        d32.byte_offset += ja.frames.fixed.offset_32;
        memcpy_from_plain_data((u8*)ja.frames.fixed.data32.data(), d32, d32_bytes);
      }
      ja.frames.fixed.data16.resize((d16_bytes + 1) / 2);
      if (d16_bytes > 0) {
        Ref d16 = fixed_ref;
        d16.byte_offset += ja.frames.fixed.offset_16;
        memcpy_from_plain_data((u8*)ja.frames.fixed.data16.data(), d16, d16_bytes);
      }

      Ref frames_ref = jacc;
      frames_ref.byte_offset += 16;
      for (int i = 0; i < (int)ja.frames.num_frames; i++) {
        Ref frame_ref = deref_label(frames_ref);
        int frame_off = 0;
        auto& frame = ja.frames.frame.emplace_back();

        frame.offset_64 = deref_u32(frame_ref, frame_off++);
        frame.offset_32 = deref_u32(frame_ref, frame_off++);
        frame.offset_16 = deref_u32(frame_ref, frame_off++);
        frame.reserved = deref_u32(frame_ref, frame_off++);

        frame.data64_size = frame.offset_32 - frame.offset_64;
        frame.data32_size = frame.offset_16 - frame.offset_32;
        {
          int data16 = (int)((ja.frames.frame_qwc - 1) * 16) - (int)frame.data64_size -
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
    // this should catch 99% of cases, but there could be mismatches between
    // master art names and model names
    out[master_art_name + "-lod0"].anims.push_back(ja);
    // out[master_art_name + "-lod1"].anims.push_back(ja);
    // out[master_art_name + "-lod2"].anims.push_back(ja);
  }
}
}  // namespace decompiler
