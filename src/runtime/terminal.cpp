// SPDX-License-Identifier: Apache-2.0
#include "terminal.hpp"
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>
namespace boltrt {
TerminalControl::TerminalControl(bool enabled) {
  if (!enabled || !isatty(STDIN_FILENO))
    return;
  if (tcgetattr(STDIN_FILENO, &old_) != 0)
    throw std::runtime_error("cannot read terminal settings");
  auto raw = old_;
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
    throw std::runtime_error("cannot enable terminal controls");
  active_ = true;
  display_ = isatty(STDOUT_FILENO);
  if (display_)
    std::cout << "\033[?1049h\033[?25l" << std::flush;
}
TerminalControl::~TerminalControl() { close(); }
void TerminalControl::close() noexcept {
  if (active_)
    tcsetattr(STDIN_FILENO, TCSANOW, &old_);
  if (display_)
    std::cout << "\033[?25h\033[?1049l" << std::flush;
  active_ = display_ = false;
}
std::vector<Event> TerminalControl::poll() {
  std::vector<Event> result;
  if (!active_)
    return result;
  pollfd input{STDIN_FILENO, POLLIN, 0};
  while (::poll(&input, 1, 0) > 0) {
    unsigned char key;
    if (read(STDIN_FILENO, &key, 1) != 1)
      break;
    if (key >= 'A' && key <= 'Z')
      key = std::tolower(key);
    last_key_ = key == ' '                    ? "Space"
                : (key == 127 || key == '\b') ? "Backspace"
                : std::isprint(key)           ? std::string(1, char(key))
                                              : "control key";
    switch (key) {
    case 'i':
      result.push_back(Event::Init);
      break;
    case 'o':
      result.push_back(Event::Hold);
      break;
    case ']':
      result.push_back(Event::Policy);
      break;
    case ' ':
      result.push_back(Event::TogglePause);
      break;
    case 'q':
      result.push_back(Event::Stop);
      break;
    default:
      break;
    }
  }
  return result;
}
void TerminalControl::render(const RuntimeStatus &s) {
  if (!display_)
    return;
  const auto now = std::chrono::steady_clock::now();
  if (s.last_event == previous_event_ && now - last_draw_ < std::chrono::milliseconds(100))
    return;
  previous_event_ = s.last_event;
  last_draw_ = now;
  winsize window{};
  const int width =
      ioctl(STDOUT_FILENO, TIOCGWINSZ, &window) == 0 && window.ws_col ? window.ws_col : 100;
  auto line = [&](std::string value) {
    if (int(value.size()) >= width)
      value = value.substr(0, std::max(0, width - 4)) + "...";
    std::cout << "\033[2K" << value << '\n';
  };
  auto number = [](double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << value;
    return out.str();
  };
  const char *mode = s.mode == Mode::Policy ? "POLICY"
                     : s.mode == Mode::Init ? "INITIALIZING"
                                            : "HOLD";
  std::cout << "\033[H";
  line("boltrt | " + s.robot + " | " + mode);
  line("Policy : " + s.policy_path);
  for (const auto &value : s.task_status)
    line(value);
  line("Backend: " + number(s.backend_time) + " s    Policy: " + number(s.policy_ms) + " ms");
  line("");
  for (const auto &value : s.task_controls)
    line(value);
  line("Policy : [I] Initialize   [O] Hold pose   []] Enter policy");
  line("[Q] Quit   [Ctrl-C] Quit");
  line("");
  line("Last key: " + last_key_);
  line("Status  : " + s.last_event);
  line("Focus this terminal to use runtime controls.");
  std::cout << "\033[J" << std::flush;
}
} // namespace boltrt
