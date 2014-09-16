/*	
 * File: Profiler.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: September 14, 2010
 * Description: class for profiling code
 *
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once
#ifndef __D_PROFILER__
#define __D_PROFILER__

#include <map>
#include <vector>
#include <string>

#include "Timestamp.h"

namespace DUtils {

/**
 * Measures the execution time of the given command and prints it with
 * a title
 * @param cmd command
 * @param s title
 */
#define PROFILE_S(cmd, s) \
  { DUtils::Timestamp t_begin, t_end; \
    t_begin.setToCurrentTime(); \
    cmd; \
    t_end.setToCurrentTime(); \
    std::cout << s << " - elapsed time: " \
      << DUtils::Timestamp::Format(t_end - t_begin) \
      << std::endl; \
  }

/**
 * Measures the execution time of the given command and prints it
 * @param cmd command
 */
#define PROFILE(cmd) PROFILE_S(cmd, "")

/** 
 * Profiles a command with a profiler
 * @param P Profiler object
 * @param name name of profiling item
 * @param scale scaling factor of execution time
 * @param cmd command
 */
#define PROFILE_CODE(P, name, scale, cmd) \
  { (P).profile(name); \
    cmd; \
    (P).stopAndScale((scale), (name)); \
  }  

/// Measures execution time of code
class Profiler
{
public:

  // scales of time
  static const float MS; // milliseconds
  static const float SECONDS; // seconds

public:

  /// Item iterator
  class const_iterator:
      protected std::map<std::string, std::vector<double> >::const_iterator
  {
  public:

    using std::map<std::string, std::vector<double> >::const_iterator::operator++;
    using std::map<std::string, std::vector<double> >::const_iterator::operator--;

    /**
     * @brief const_iterator
     */
    const_iterator(){}

    /**
     * @brief Creates a profiler iterator from a map iterator
     * @param it map iterator
     */
    const_iterator(
        const std::map<std::string, std::vector<double> >::const_iterator &it);

    /**
     * @brief operator ==
     * @param b
     * @return true iif this == b
     */
    bool operator==(const const_iterator& b) const;

    /**
     * @brief operator !=
     * @param b
     * @return true iif this != b
     */
    inline bool operator!=(const const_iterator& b) const
      { return !(*this == b); }

    /**
     * @brief Deference operator. It returns the same iterator object
     * @return *this
     */
    const_iterator operator*();

    /**
     * @brief Returns the name of the profiled item
     * @return name
     */
    std::string name() const;

    /**
     * @brief Returns the mean of the profiled times
     * @return mean
     */
    double mean() const;

    /**
     * @brief Returns the standard deviation of the profiled times
     * @return standard deviation
     */
    double stdev() const;

    /**
     * @brief Returns the median of the profiled times
     * @return median
     */
    double median() const;

    /**
     * @brief Returns the min. of the profiled times
     * @return min
     */
    double min() const;

    /**
     * @brief Returns the max. of the profiled times
     * @return max
     */
    double max() const;

    /**
     * @brief Returs the number of times the item has been counted
     * @return number of entries of the item
     */
    size_t count() const;
  };

public:

  Profiler(const float scale = Profiler::SECONDS): 
    m_last_profile(""), m_scale(scale), m_reserve(0){}
  virtual ~Profiler(){}
  
  /**
   * Starts profiling the given item. If a profile on this item is already 
   * active, that previous call with this item is overriden.
   * @param name name of item to profile. If not given, an empty string is used.
   */
  void profile(const std::string &name = "");
  
  /**
   * Does the same as Profiler::stop, but overrides the default scale of time
   * (i.e it multiplies the elapsed time in seconds by
   * the given scale factor)
   * @param scale stored_duration = actual_duration (s) * scale
   * @param name item name. If not given, last entry used in ::profile is used.
   * @note use scale 1e3 to store the time in milliseconds
   */
  void stopAndScale(double scale, const std::string &name = "");
  
  /**
   * Stops profiling the given item or the last one if this is not provided.
   * Adds the elapsed time (in the default scale of time) to the sum of this 
   * item profile time
   * @param name item name. If not given, last entry used in ::profile is used.
   */
  inline void stop(const std::string &name = "")
  {
    stopAndScale(m_scale, name);
  }

  /**
   * For each item in the profile, n blocks (double) are reserved in memory.
   * The new items that are created after calling this function will also
   * reserve n blocks of memory
   */
  void reserve(size_t n);

  /**
   * Adds a value to the given entry
   * @param v value (already scaled) to add
   * @param name entry name. If not given, an empty string is used.
   */
  void add(double v, const std::string &name = "");

  /**
   * Returns the last measured time for an item
   * @param name item name. If not given, an empty string is used.
   */
  double back(const std::string &name = "") const;
 
  /**
   * Removes all the measurements of the given item
   * @param name item name. If not given, an empty string is used.
   */
  void reset(const std::string &name = "");
  
  /**
   * Returns the number of elements profiled of a given item
   * @param name item name. If not given, an empty string is used.
   * @return number of entries with the given name
   */
  size_t size(const std::string &name = "") const;
  
  /**
   * Removes the measurements of all the items
   */
  inline void resetAll()
  {
    m_profiles.clear();
  }

  /**
   * Returns the names of all the entries in the profiler
   * @param names (out) names
   */
  void getEntryNames(std::vector<std::string> &names) const;

  /**
   * Returns the default scale of time to use with ::stop
   * @return scale
   */
  inline float getDefaultScale() const { return m_scale; }
  
  /**
   * Sets the default scale of time to use with ::stop
   * @param scale
   */
  inline void setDefaultScale(float scale) {  m_scale = scale; }

  /**
   * Returns the mean of the time of the given entry
   * @param name entry name
   */
  double getMeanTime(const std::string &name = "") const ;
  
  /**
   * Returns the standard deviation of the time of the given entry
   * @param name entry name
   */
  double getStdevTime(const std::string &name = "") const ;
  
  /**
   * Returns the min time of the given entry
   * @param name entry name
   */
  double getMinTime(const std::string &name = "") const ;
  
  /**
   * Returns the max time of the given entry
   * @param name entry name
   */
  double getMaxTime(const std::string &name = "") const ;
  
  /**
   * Returns the sum of the times of the given entry
   * @param name entry name
   */
  double getTotalTime(const std::string &name = "") const;
  
  /**
   * Returns all the time measurements of the given entry
   * @param time (out) measurements
   * @param name entry name
   */
  void getTime(std::vector<double> &time, const std::string &name = "") const;
  
  /**
   * Returns all the statistics of the given entry
   * @param mean (out) mean
   * @param stdev (out) standard deviation
   * @param min (out) min value
   * @param max (out) max value
   * @param name entry name
   */
  void getStatistics(double &mean, double &stdev,
    double &min, double &max, const std::string &name = "") const;
  
  /**
   * Prints all the statistics of the given entry
   * @param name entry name
   * @param suffix unit suffix to print with the measurements
   * @param scale scale to multiply the measurements before printing
   * @param out stream to print to
   */
  void showStatistics(const std::string &name = "", 
    const std::string &suffix = "s", double scale = 1.,
    std::ostream &out = std::cout) const;

  /**
   * @brief Returns a const iterator to the first item in the profiler
   * @return begin iterator
   */
  inline const_iterator begin() const { return m_profiles.begin(); }

  /**
   * @brief Returns a const iterator to the first item in the profiler
   * @return begin iterator
   */
  inline const_iterator cbegin() const { return m_profiles.begin(); }

  /**
   * @brief Returns a const iterator to the last+1 item in the profiler
   * @return end iterator
   */
  inline const_iterator end() const { return m_profiles.end(); }

  /**
   * @brief Returns a const iterator to the last+1 item in the profiler
   * @return end iterator
   */
  inline const_iterator cend() const { return m_profiles.end(); }

protected:
  
  /// Profile data
  std::map<std::string, std::vector<double> > m_profiles;
  
  /// Starting points
  std::map<std::string, Timestamp> m_start_points;
  
  /// Last used entry
  std::string m_last_profile;
  
  /// Default scale
  float m_scale;
  
  /// Blocks to reserve when an entry is added 
  size_t m_reserve;
  
};

}

#endif
