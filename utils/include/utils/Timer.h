// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <deque>
#include <iomanip>
#include <map>
#include <stack>
#include <string>
#include <utility>
#include <vector>
#include <HAL/Utils/TicToc.h>

class Timer {
 public:
  struct TFunction {
    TFunction() : level(-1),
                  init_time(0.0),
                  children_time(0.0),
                  is_active(false),
                  parent_name("") {}

    int                 level;
    Scalar              init_time;
    Scalar              children_time;    // Time used by internal functions
    bool                is_active;
    std::string         parent_name;      // Name of the parent function
    std::deque<Scalar>  process_time;     // Overall function time
    std::deque<Scalar>  additional_time;  // Time not accounted by dChildrenTime
  };

  Timer() : max_name_length_(0),
            window_size_(40) {
    Reset();
  }

  void Reset() {
    tracked_functions_.clear();
    display_order_.clear();
    while (!function_stack_.empty()) {
      function_stack_.pop();
    }
  }

  void set_window_size(size_t window_size) {
    window_size_ = window_size;
  }

  size_t window_size() const {
    return window_size_;
  }

  void Tic(std::string func_name) {
    // check if is main program
    if (func_name.empty()) {
      // reset all values
      func_name = "Total";
    }

    // Look if this function is already registered
    std::map< std::string, TFunction >::iterator it;
    it = tracked_functions_.find(func_name);

    if (it == tracked_functions_.end()) {
      TFunction f;
      f.level  = static_cast<int>(function_stack_.size());
      f.parent_name = (f.level == 0)?"root":function_stack_.top();
      f.is_active = false;
      tracked_functions_.insert(
          std::pair<std::string, TFunction>(func_name, f));
      display_order_.insert(std::pair<int, std::string>(
          static_cast<int>(display_order_.size()), func_name));
      if (func_name.length() > max_name_length_) {
        max_name_length_ = func_name.length();
      }
    } else if (it->second.is_active) {
      std::cerr << "WARNING -- Timer::Tic() called with the name " <<
          func_name << " twice" << std::endl;
      return;
    }

    // Save current time
    tracked_functions_[func_name].is_active = true;
    tracked_functions_[func_name].init_time = hal::Tic();
    tracked_functions_[func_name].children_time = 0.0;

    // Add function to stack
    function_stack_.push(func_name);
  }

  void Toc(std::string func_name) {
    // check if is main program
    if (func_name.empty()) {
      func_name = "Total";
    }

    // check if there is a function in the stack
    if (function_stack_.empty()) {
      std::cerr << "ERROR: [Toc] Inbalance of Tic-Toc calls" <<
          " for [" << func_name << "]" << std::endl;
      return;
    }

    // Pop last function in the stack
    function_stack_.pop();

    // Register time -- smoothed a bit (smooth_factor=0 := no smoothing)
    Scalar smooth_factor = 0.8;
    Scalar cur  = hal::TocMS(tracked_functions_[func_name].init_time);
    Scalar prev = tracked_functions_[func_name].process_time.empty() ?
        cur : tracked_functions_[func_name].process_time.back();
    Scalar process_time = (1-smooth_factor)*cur + smooth_factor*prev;
    Scalar children_time = tracked_functions_[func_name].children_time;
    Scalar additional_time = process_time - children_time;
    std::string parent_name  = tracked_functions_[func_name].parent_name;

    tracked_functions_[func_name].process_time.push_back(process_time);
    tracked_functions_[func_name].additional_time.push_back(additional_time);
    tracked_functions_[func_name].is_active = false;
    if (parent_name.compare("root")) {
      tracked_functions_[parent_name].children_time += process_time;
    }

    // Delete oldest time from the vector if we have more than needed
    if (tracked_functions_[func_name].process_time.size() > window_size_) {
      tracked_functions_[func_name].process_time.pop_front();
      tracked_functions_[func_name].additional_time.pop_front();
    }
  }

  // return number of registered functions with level <= nLevel
  int GetNumFunctions(int level) {
    int num_func = 0;
    std::map< std::string, TFunction >::iterator it;

    for (it = tracked_functions_.begin();
         it != tracked_functions_.end(); ++it) {
      if ((*it).second.level <= level) {
        ++num_func;
      }
    }

    return num_func;
  }

  void PrintToTerminal(int levels) {
    std::map< int, std::string >::iterator it;

    std::cout << "-----------------------------------------------" << std::endl;
    for (it = display_order_.begin(); it != display_order_.end(); ++it) {
      std::string func_name = (*it).second;
      int level    = tracked_functions_[func_name].level;
      if (level <= levels) {
        Scalar process_time = tracked_functions_[func_name].process_time.back();
        if (level > 1) {
          std::cout << std::setw(5*(tracked_functions_[func_name].level-1))
                    << "";
        }
        std::cout << std::setw(max_name_length_+5) << std::left
                  << (*it).second;
        std::cout << std::setw(10) << std::setprecision(3) <<
            std::fixed << std::right;
        std::cout << process_time << std::endl;
      }
    }
  }

  std::vector<std::string> GetNames(const int levels) {
    std::vector<std::string> names;
    names.resize(GetNumFunctions(levels));
    std::map< int, std::string>::iterator it;

    int kk = 0;
    for (it = display_order_.begin(); it != display_order_.end(); ++it) {
      std::string func_name = (*it).second;
      std::string level_mark = "";
      int level = tracked_functions_[func_name].level;

      for (int ii = 1; ii < level; ++ii) {
        level_mark += "  ";
      }

      if (level > 0) level_mark += "|-";

      if (level <= levels) {
        if ((level < levels) &&
            (tracked_functions_[func_name].children_time > 0.0)) {
          // This is a parent function with children in the last level
          names[kk] = level_mark + func_name + " [T]";
        } else {
          // This is either a function in the last requested level or
          // a parent function with no children
          names[kk] = level_mark + func_name;
        }
        ++kk;
      }
    }
    return names;
  }

  std::vector<std::pair<Scalar, Scalar> > GetTimes(const int levels) {
    std::vector< std::pair<Scalar, Scalar> > times;
    times.resize(GetNumFunctions(levels));

    Scalar process_time;
    Scalar additional_time;
    std::map< int, std::string>::iterator it;

    int kk = 0;
    for (it = display_order_.begin(); it != display_order_.end(); ++it) {
      std::string func_name = (*it).second;

      std::map< std::string, TFunction >::iterator searchit;
      searchit = tracked_functions_.find(func_name);
      if (searchit == tracked_functions_.end()) {
        continue;
      }

      if (tracked_functions_[func_name].process_time.empty() ||
          tracked_functions_[func_name].additional_time.empty()) {
        continue;
      }

      int level = tracked_functions_[func_name].level;
      if (level <= levels) {
        process_time    = tracked_functions_[func_name].process_time.back();
        additional_time = tracked_functions_[func_name].additional_time.back();

        if ((level < levels) &&
            (tracked_functions_[func_name].children_time > 0.0)) {
          // This is a parent function with children in the last level
          times[kk] = std::pair<Scalar, Scalar>(process_time, additional_time);
        } else {
          // This is either a function in the last requested level or
          // a parent function with no children
          times[kk] = std::pair<Scalar, Scalar>(process_time, process_time);
        }
        ++kk;
      }
    }
    return times;
  }

 private:
  unsigned int max_name_length_;
  size_t window_size_;
  std::stack<std::string> function_stack_;
  std::map<std::string, TFunction> tracked_functions_;
  std::map<int, std::string> display_order_;
};
