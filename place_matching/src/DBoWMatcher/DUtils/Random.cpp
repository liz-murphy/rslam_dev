/*	
 * File: Random.cpp
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010
 * Description: manages pseudo-random numbers
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

#include <place_matching/DBoWMatcher/DUtils/Random.h>
#include <place_matching/DBoWMatcher/DUtils/Timestamp.h>
#include <cstdlib>
using namespace std;

bool DUtils::Random::m_already_seeded = false;

void DUtils::Random::SeedRand(){
	Timestamp time;
	time.setToCurrentTime();
	srand((unsigned)time.getFloatTime()); 
}

void DUtils::Random::SeedRandOnce()
{
  if(!m_already_seeded)
  {
    DUtils::Random::SeedRand();
    m_already_seeded = true;
  }
}

void DUtils::Random::SeedRand(int seed)
{
	srand(seed); 
}

void DUtils::Random::SeedRandOnce(int seed)
{
  if(!m_already_seeded)
  {
    DUtils::Random::SeedRand(seed);
    m_already_seeded = true;
  }
}

int DUtils::Random::RandomInt(int min, int max){
	int d = max - min + 1;
	return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

DUtils::Random::UnrepeatedRandomizer::UnrepeatedRandomizer(int min, int max)
{
  if(min <= max)
  {
    m_min = min;
    m_max = max;
  }
  else
  {
    m_min = max;
    m_max = min;
  }

  createValues();
}

// ---------------------------------------------------------------------------

DUtils::Random::UnrepeatedRandomizer::UnrepeatedRandomizer
  (const DUtils::Random::UnrepeatedRandomizer& rnd)
{
  *this = rnd;
}

// ---------------------------------------------------------------------------

int DUtils::Random::UnrepeatedRandomizer::get()
{
  if(empty()) createValues();
  
  DUtils::Random::SeedRandOnce();
  
  int k = DUtils::Random::RandomInt(0, m_values.size()-1);
  int ret = m_values[k];
  m_values[k] = m_values.back();
  m_values.pop_back();
  
  return ret;
}

// ---------------------------------------------------------------------------

void DUtils::Random::UnrepeatedRandomizer::createValues()
{
  int n = m_max - m_min + 1;
  
  m_values.resize(n);
  for(int i = 0; i < n; ++i) m_values[i] = m_min + i;
}

// ---------------------------------------------------------------------------

void DUtils::Random::UnrepeatedRandomizer::reset()
{
  if((int)m_values.size() != m_max - m_min + 1) createValues();
}

// ---------------------------------------------------------------------------

DUtils::Random::UnrepeatedRandomizer& 
DUtils::Random::UnrepeatedRandomizer::operator=
  (const DUtils::Random::UnrepeatedRandomizer& rnd)
{
  if(this != &rnd)
  {
    this->m_min = rnd.m_min;
    this->m_max = rnd.m_max;
    this->m_values = rnd.m_values;
  }
  return *this;
}

// ---------------------------------------------------------------------------


