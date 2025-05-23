/**
 * @file scoped_timer.h
 * @brief Defines the inline scoped timer
 *
 * A handy utility that uses RAII scoped timing for a function
 */

#pragma once

#include <chrono>
#include <iostream>

struct ScopedTimer
{
  const char*                           name;
  std::chrono::steady_clock::time_point start;

  ScopedTimer(const char* name) : name(name), start(std::chrono::steady_clock::now()) {}
  ~ScopedTimer()
  {
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - start)
                  .count();
    std::cout << name << " took " << dt << "microseconds" << std::endl;
  }
};
