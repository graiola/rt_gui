#include <console/console.h>

using namespace rt_gui;

void printFunction()
{
  std::cout << "hello world!" << std::endl;
}

int main(int argc, char *argv[])
{

  Console console;

  Console::ConsoleCommand printFunctionCmd;
  printFunctionCmd.command = "print";
  printFunctionCmd.comment = "print some useful stuff";
  printFunctionCmd.function = printFunction;

  console.addConsoleFunction(printFunctionCmd);

  console.runConsole();

  return 0;
}
