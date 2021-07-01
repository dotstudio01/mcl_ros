#ifndef __TOOLS_H
#define __TOOLS_H
#include "types.h"
#include <fstream>
#include <string>

double randDouble();
float randFloat();
double randGaussian(double sigma);
PoseVector load_pose(const std::string &file_name);
void write_pose(const std::string &file_name, PoseVector &poses);
bool file_exists(const std::string &name);
#endif