#include "debugger_thread.h"

#include <utility>

#include <common/util/FileUtil.h>
#include <common/util/string_util.h>

#include <game/runtime.h>

#include "third-party/imgui/imgui.h"
#include "third-party/imgui/imgui_stdlib.h"

namespace debugger_thread {
DebuggerPacket::DebuggerPacket(std::string compiler_in, DebuggerSignal signal)
    : m_compiler_in(std::move(compiler_in)), m_signal(signal) {}

void DebuggerThread::connect() const {
  if (m_compiler.get()) {
    lg::info("[DebuggerThread] connecting compiler...");
    m_compiler->connect_to_target();
  }
}

void DebuggerThread::run_code(const std::string& in) const {
  m_compiler->run_front_end_on_string(in);
}

void DebuggerThread::set_packet(DebuggerPacket* packet) {
  m_packet = packet;
}

DebuggerPacket* DebuggerThread::get_packet() const {
  return m_packet;
}

void DebuggerThread::delete_packet() {
  delete m_packet;
  m_packet = nullptr;
}

std::shared_ptr<Compiler> DebuggerThread::get_compiler() {
  return g_debugger_thread.m_compiler;
}

void DebuggerThread::init_compiler(GameVersion version) {
  m_compiler = std::make_shared<Compiler>(version);
}

void DebuggerThread::print_strace() {
  auto strace = file_util::get_jak_project_dir() / "log" / "goal-strace";
  auto filename = fmt::format("strace-{}.txt", str_util::current_local_timestamp_no_colons());
  g_debugger_thread.m_compiler->run_front_end_on_string(
      fmt::format("(:di \"{}/{}\")", strace.c_str(), filename));
  lg::info("[DebuggerThread] strace generated ({})!", filename);
}

void DebuggerThread::handle_signal() const {
  switch (m_packet->m_signal) {
    case DebuggerSignal::NONE:
      lg::info("[DebuggerThread] got packet NONE");
      break;
    case DebuggerSignal::CONNECT:
      lg::info("[DebuggerThread] got packet CONNECT");
      connect();
      break;
    case DebuggerSignal::SEND_CODE:
      lg::info("[DebuggerThread] got packet SEND_CODE");
      if (!m_packet->m_compiler_in.empty()) {
        lg::info("[DebuggerThread] SEND_CODE \"{}\"", m_packet->m_compiler_in);
        run_code(m_packet->m_compiler_in);
      } else {
        lg::warn("[DebuggerThread] got SEND_CODE but input was empty!");
      }
      break;
    case DebuggerSignal::STRACE:
      lg::info("[DebuggerThread] got packet STRACE");
      print_strace();
    default:
      break;
  }
}

void DebuggerWindow::draw_window() {
  ImGui::Begin("Debugger");
  ImGui::InputText("Compiler input", &m_compiler_in, ImGuiInputTextFlags_AutoSelectAll);
  if (ImGui::Button("Send Code")) {
    g_debugger_thread.set_packet(new DebuggerPacket(m_compiler_in, DebuggerSignal::SEND_CODE));
  }
  ImGui::End();
}
}  // namespace debugger_thread