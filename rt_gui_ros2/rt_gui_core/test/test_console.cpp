#include <rt_gui/console/console.h>

using namespace rt_gui;

void printFunction()
{
  std::cout << "hello world!" << std::endl;
}

void sumFunction()
{
  double a = 0.0;
  double b = 0.0;
  std::cout << "Sum two numbers" << std::endl;
  newline::getDouble("First number: ",a,a);
  newline::getDouble("Second number: ",b,b);
  std::cout << a + b << std::endl;
}

int main(int argc, char *argv[])
{

  Console console;

  console.addConsoleFunction("print","print some useful stuff",printFunction);
  console.addConsoleFunction("sum","sum two numbers",sumFunction);

  console.runConsole();

  return 0;
}
