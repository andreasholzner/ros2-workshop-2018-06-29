/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include <memory>
#include <string>

#include "rcutils/cmdline_parser.h"

int main(int argc, char **argv)
{
  std::string param_a = "Aaaaaaaaaaaaa";
  if (rcutils_cli_option_exist(argv, argv + argc, "-a"))
  {
    param_a = std::string(rcutils_cli_get_option(argv, argv + argc, "-a"));
  }
  std::cout << "Parameter a=" << param_a << std::endl;

  auto foo = [](std::shared_ptr<int> v) { ++*v; };
  auto num = std::make_shared<int>(7);
  std::cout << "Number=" << *num << std::endl;
  foo(num);
  std::cout << "Number=" << *num << std::endl;

  return 0;
}