/*	
 * File: Profile.cpp
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

#include <map>
#include <vector>
#include <string>
#include <algorithm>

#include "Timestamp.h"
#include "Profiler.h"
#include "Math.hpp"

using namespace DUtils;
using namespace std;

// ---------------------------------------------------------------------------

const float Profiler::MS = 1e3; // milliseconds
const float Profiler::SECONDS = 1; // seconds

// ---------------------------------------------------------------------------

void Profiler::reserve(size_t n)
{
  std::map<std::string, std::vector<double> >::iterator it;
  for(it = m_profiles.begin(); it != m_profiles.end(); ++it)
  {
    it->second.reserve(n);
  }
  m_reserve = n;
}

// ---------------------------------------------------------------------------

void Profiler::profile(const std::string &name)
{
  Timestamp t;
  m_last_profile = name;
  pair<std::map<std::string, Timestamp>::iterator, bool> res =
    m_start_points.insert(make_pair(name, t));
  res.first->second.setToCurrentTime();
}

// ---------------------------------------------------------------------------

void Profiler::stopAndScale(double scale, const std::string &name)
{
  Timestamp t;
  t.setToCurrentTime();
  
  string s = (name.empty() ? m_last_profile : name);
  
  std::map<std::string, Timestamp>::iterator it = m_start_points.find(s);
  
  if(it != m_start_points.end())
  {
    double duration = (t - it->second) * scale;
    m_start_points.erase(it);
    
    pair<std::map<std::string, std::vector<double> >::iterator,bool> pdit;
    pdit.first = m_profiles.find(s);
    if(pdit.first == m_profiles.end())
    {
      pdit = m_profiles.insert(make_pair(s, vector<double>()));
      if(m_reserve != 0) pdit.first->second.reserve(m_reserve);
    }
    pdit.first->second.push_back(duration);
  }
}

// ---------------------------------------------------------------------------

void Profiler::add(double v, const std::string &name)
{
  std::map<std::string, vector<double> >::iterator it = m_profiles.find(name);
  
  if(it == m_profiles.end())
  {
    pair<std::map<std::string, std::vector<double> >::iterator,bool> pdit =
      m_profiles.insert(make_pair(name, vector<double>(1, v)));
    if(m_reserve != 0) pdit.first->second.reserve(m_reserve);
  }
  else
  {
    it->second.push_back(v);
  }
  
}

// ---------------------------------------------------------------------------

size_t Profiler::size(const std::string &name) const
{
  std::map<std::string, vector<double> >::const_iterator it = 
    m_profiles.find(name);
  
  if(it == m_profiles.end())
  {
    return 0;
  }
  else
  {
    return it->second.size();
  }
}

// ---------------------------------------------------------------------------

double Profiler::getMeanTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    return Math::Mean<double>(it->second);
  }
  else return 0;
}

// ---------------------------------------------------------------------------

double Profiler::getStdevTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);

  if(it != m_profiles.end())
  {
    return Math::Stdev<double>(it->second);
  }
  else return 0;
}
  
// ---------------------------------------------------------------------------
  
double Profiler::getMinTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    return *std::min_element(it->second.begin(), it->second.end());
  }
  else return 0;
}
  
// ---------------------------------------------------------------------------
  
double Profiler::getMaxTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    return *std::max_element(it->second.begin(), it->second.end());
  }
  else return 0;
}

// ---------------------------------------------------------------------------

double Profiler::getTotalTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    return std::accumulate(it->second.begin(), it->second.end(), 0.);
  }
  else return 0;
}

// ---------------------------------------------------------------------------

void Profiler::getStatistics(double &mean, double &stdev, double &min, 
  double &max, const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    if(it->second.empty())
    {
      mean = stdev = min = max = 0;
    }
    else
    {
      vector<double>::const_iterator dit = it->second.begin();
      mean = min = max = *dit;
      
      for(++dit; dit != it->second.end(); ++dit)
      {
        mean += *dit;
        if(*dit < min) min = *dit;
        else if(*dit > max) max = *dit;
      }
      mean /= it->second.size();
      stdev = Math::Stdev<double>(it->second, mean);
    }
  }
}

// ---------------------------------------------------------------------------

void Profiler::showStatistics(const std::string &name, 
  const std::string &suffix, double scale, ostream &out) const
{
  double mean, std, min, max;
  this->getStatistics(mean, std, min, max, name);
  
  if(!name.empty())
    out << name << ": ";
  
  out << mean * scale
    << " +/- " << std * scale << " " << suffix
    << " (" << min * scale << " .. " << max * scale << ")" << endl;
}

// ---------------------------------------------------------------------------
  
void
Profiler::getTime(std::vector<double> &time, const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    time = it->second;
  }
  else time.clear();
}
  
// ---------------------------------------------------------------------------

double Profiler::back(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    return it->second.back();
  }
  else return 0;
}

// ---------------------------------------------------------------------------

void Profiler::reset(const std::string &name)
{
  std::map<std::string, std::vector<double> >::iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    it->second.clear();
  }
}

// ---------------------------------------------------------------------------

void Profiler::getEntryNames(std::vector<std::string> &names) const
{
  names.clear();
  names.reserve(m_profiles.size());
  
  map<string, vector<double> >::const_iterator pit;
  for(pit = m_profiles.begin(); pit != m_profiles.end(); ++pit)
  {
    names.push_back(pit->first);
  }
}

// ---------------------------------------------------------------------------

Profiler::const_iterator::const_iterator(
    const std::map<std::string, std::vector<double> >::const_iterator &it)
{
  std::map<std::string, std::vector<double> >::const_iterator::operator=(it);
}

Profiler::const_iterator Profiler::const_iterator::operator*()
{
  return *this;
}

bool Profiler::const_iterator::operator==(const const_iterator& b) const
{
  return
    static_cast
      <const std::map<std::string, std::vector<double> >::const_iterator&>
      (*this)
    ==
    static_cast
      <const std::map<std::string, std::vector<double> >::const_iterator&>(b);
}

std::string Profiler::const_iterator::name() const
{
  return (*this)->first;
}

double Profiler::const_iterator::mean() const
{
  return Math::Mean<double>((*this)->second);
}

double Profiler::const_iterator::stdev() const
{
  return Math::Stdev((*this)->second);
}

double Profiler::const_iterator::median() const
{
  return Math::Median((*this)->second);
}

double Profiler::const_iterator::min() const
{
  return Math::Min((*this)->second);
}

double Profiler::const_iterator::max() const
{
  return Math::Max((*this)->second);
}

size_t Profiler::const_iterator::count() const
{
  return (*this)->second.size();
}
