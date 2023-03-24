#ifndef RT_GUI_CORE_CONSOLE_CORE_H
#define RT_GUI_CORE_CONSOLE_CORE_H

#include <iostream>
#include <memory>
#include <vector>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <readline/readline.h>
#include <readline/history.h>

namespace newline {

extern std::shared_ptr<std::vector<std::string> > options_ptr;


char** consoleAutoComplete(const char*, int ,int);
char* consoleAutoGenerator(const char*,int);


void getDouble(std::string comment, double defaultvalue, double& value);

void getInt(std::string comment, int defaultvalue, int& value);

void consoleCleanUp();

}

#endif
