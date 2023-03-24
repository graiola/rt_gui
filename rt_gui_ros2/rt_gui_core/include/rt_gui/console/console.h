#ifndef RT_GUI_CONSOLE_H
#define RT_GUI_CONSOLE_H

#include <rt_gui/console/console_core.h>
#include <atomic>
#include <thread>
#include <functional>

namespace rt_gui {

class Console
{

public:

  typedef std::function<void ()> funct_t;

  struct ConsoleCommand {
    std::string command;
    std::string comment;
    funct_t function;
  };

  Console();

  Console(const std::string& prompt_name);

  ~Console();

  void runConsole();

  void stopConsole();

  bool addConsoleFunction(const std::string command,
                          const std::string comment,
                          const funct_t function);

  bool addConsoleFunction(const ConsoleCommand cmd);

private:

  bool loop();

  void updateConsoleOptions();

  void help();

  std::vector<ConsoleCommand> menu_console_;
  std::shared_ptr<std::thread> prompt_thread_;
  std::atomic<bool> stop_console_;
  std::string prompt_name_;
};

} // namespace

#endif
