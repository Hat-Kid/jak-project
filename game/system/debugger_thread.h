#pragma once
#include <optional>

#include <goalc/compiler/Compiler.h>

namespace debugger_thread {
enum class DebuggerSignal {
  NONE,
  CONNECT,
  SEND_CODE,
  STRACE,
};

struct DebuggerPacket {
  DebuggerPacket(std::string compiler_in, DebuggerSignal signal);
  std::string m_compiler_in;
  DebuggerSignal m_signal = DebuggerSignal::NONE;
};
class DebuggerThread {
 public:
  void set_packet(DebuggerPacket* packet);
  DebuggerPacket* get_packet() const;
  void delete_packet();
  void handle_signal() const;
  std::shared_ptr<Compiler> get_compiler();
  void init_compiler(GameVersion version);

 private:
  std::shared_ptr<Compiler> m_compiler = nullptr;
  DebuggerPacket* m_packet = nullptr;

  void connect() const;
  void run_code(const std::string& in) const;
  static void print_strace();
};

class DebuggerWindow {
 public:
  void draw_window();

 private:
  void draw_repl_options();

  std::string m_compiler_in;
};
}  // namespace debugger_thread