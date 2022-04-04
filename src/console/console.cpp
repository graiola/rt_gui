#include <rt_gui/console/console.h>
#include <algorithm>
#include <string.h>

namespace rt_gui
{

Console::Console()
  :  Console("rt_gui_console")
{
}

Console::Console(const std::string& prompt_name)
{
  stop_console_ = false;
  prompt_name_ = prompt_name;

  ConsoleCommand helpCmd;
  helpCmd.command = "help";
  helpCmd.comment = "print all available commands";
  helpCmd.function = std::bind(&Console::help,this);
  addConsoleFunction(helpCmd);
}

Console::~Console()
{
  prompt_thread_->join();
}

//Add local prompt commands
bool Console::addConsoleFunction(const std::string command,
                                 const std::string comment,
                                 const funct_t function)
{
  ConsoleCommand tmp = {command,comment,function};
  menu_console_.push_back(tmp);
  return true;
}

//Add local prompt commands
bool Console::addConsoleFunction(const ConsoleCommand cmd)
{
  menu_console_.push_back(cmd);
  return true;
}

// FIXME add help function
void Console::updateConsoleOptions()
{
  // Resetting the pointer with the different console options
  newline::options_ptr.reset(new std::vector<std::string>);

  // Copy function names of loaded controller(local) to console
  for (unsigned int i = 0; i < menu_console_.size(); i++)
    newline::options_ptr->push_back(menu_console_[i].command);

  //Sort options in alphabetical order:
  std::sort(newline::options_ptr->begin(), newline::options_ptr->end());
}

void Console::help()
{
  std::cout << "****************" << std::endl;
  for (unsigned int i = 0; i < menu_console_.size(); i++)
    std::cout << menu_console_[i].command << " - " << menu_console_[i].comment << std::endl;
  std::cout << "****************" << std::endl;
}

void Console::stopConsole()
{
  stop_console_ = true;
}

void Console::runConsole()
{
  stop_console_ = false;
  updateConsoleOptions();
  prompt_thread_.reset(new std::thread(&Console::loop,this));
}

bool Console::loop()
{
  char *buf;
  //Change the newline autocomplete function to our own implementation
  rl_attempted_completion_function = newline::consoleAutoComplete;
  //Change getc function so stdio's getc passes interrupt
  //rl_getc_function = getc;
  std::stringstream prompt;
  prompt << "["<<prompt_name_ << "]>> ";
  while ((buf = readline(prompt.str().c_str())) != NULL)
  {
    if (strcmp(buf, "exit") == 0 || stop_console_)
    {
      break;	//Exits the loop
    }
    else if (buf[0] == '\0')
    {
      free(buf);
      continue;
    }
    else
    {
      //std::cout << buf << std::endl; // This is the echo
      add_history(buf);
      std::stringstream ss;
      std::string input;
      ss << buf;
      ss >> input;

      // Menu choices:
      for (unsigned int i = 0; i < menu_console_.size(); i++)
      {
        if (input == menu_console_[i].command)
        {
          if(menu_console_[i].function)
          {
            menu_console_[i].function();
            input.clear();
          }
        }
      }
    }
    free(buf);
    buf = NULL;
    prompt.str("");
    prompt.clear();
    prompt << "["<<prompt_name_ << "]>> ";

  }
  std::cout << "Breaking console thread" << std::endl;
  return true;
}

} // namespace
