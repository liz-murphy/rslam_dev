/*
 * File: FileFunctions.cpp
 * Author: Dorian Galvez-Lopez
 * Date: June 2009
 * Description: file system functions
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

#include <place_matching/DBoWMatcher/DUtils/FileFunctions.h>
#include <vector>
#include <string>
#include <fstream>
#include <cstdio>
#include <algorithm>

#ifdef _WIN32
#ifndef WIN32
#define WIN32
#endif
#endif

#ifdef WIN32
	#include <direct.h>
	#include "dirent_win.h"
	#define mkdir(a) _mkdir(a)
#else
    #ifdef __APPLE__
        #include <sys/stat.h>
        #include <unistd.h>
        #include <dirent.h>
        #define mkdir(a) mkdir(a, 0755)
    #else
        #include <dirent.h>
        #include <sys/stat.h>
        #include <unistd.h>
        #define mkdir(a) mkdir(a, 0755)
    #endif
#endif

using namespace std;
using namespace DUtils;

void FileFunctions::MkDir(const char *path)
{
	mkdir(path);
}

void FileFunctions::RmDir(const char *path)
{
	// empty path
	vector<string> files = FileFunctions::Dir(path, "");
	for(vector<string>::iterator it = files.begin(); it != files.end(); it++){
		remove(it->c_str());
	}
	rmdir(path);
}

void FileFunctions::RmFile(const char *path)
{
	remove(path);
}

bool FileFunctions::FileExists(const char *filename)
{
	std::fstream f(filename, ios::in);

	if(f.is_open()){
		f.close();
		return true;
	}else
		return false;
}

bool FileFunctions::DirExists(const char *path)
{
    DIR *dirp;
	if((dirp = opendir(path)) != NULL){
		closedir(dirp);
		return true;
	}else
		return false;
}

std::vector<std::string> FileFunctions::Dir(const char *path, const char *right,
  bool sorted, bool case_sensitive)
{
  DIR *dirp;
  struct dirent *entry;
  vector<string> ret;
  const string r(right);

  dirp = opendir(path);
  if(dirp != NULL)
  {
    if(case_sensitive)
    {
      while((entry = readdir(dirp)) != NULL){
        string name(entry->d_name);
        if((name.length() >= r.length()) &&
          (name.substr(name.length() - r.length()).compare(r) == 0))
        {
          ret.push_back(string(path) + "/" + entry->d_name);
        }
      }
    }
    else
    {
      // case insensitive
      while((entry = readdir(dirp)) != NULL)
      {
        string name(entry->d_name);

        if(name.length() >= r.length())
        {
          const string endpart = name.substr(name.length() - r.length());

          bool equal = true;
          string::const_iterator ait, bit;
          ait = endpart.begin();
          bit = r.begin();

          for(; ait != endpart.end(); ++ait, ++bit)
          {
            if(tolower(*ait) != tolower(*bit))
            {
              equal = false;
              break;
            }
          }

          if(equal)
          {
            ret.push_back(string(path) + "/" + entry->d_name);
          }
        } // if name.length >= r.length
      } // for each file
    } // if case_sensitive

    closedir(dirp);
  } // if dir opened

  if(sorted) sort(ret.begin(), ret.end());

  return ret;
}
std::string FileFunctions::FileName(const std::string filepath)
{
	string::size_type p = filepath.find_last_of('/');
	string::size_type p2 = filepath.find_last_of('\\');
	if(p2 != string::npos && p2 > p) p = p2;
	return filepath.substr(p+1);
}

void FileFunctions::FileParts(const std::string filepath, std::string &path,
						   std::string &filename, std::string &ext)
{
	string::size_type p = filepath.find_last_of('/');
	string::size_type p2 = filepath.find_last_of('\\');
	if(p == string::npos || (p2 != string::npos && p2 > p)) p = p2;

	std::string filext;

	if(p == string::npos){
		path = "";
		filext = filepath;
	}else{
		path = filepath.substr(0, p);
		filext = filepath.substr(p+1);
	}

	p = filext.find_last_of('.');
	if(p == string::npos){
		filename = filext;
		ext = "";
	}else{
		filename = filext.substr(0, p);
		ext = filext.substr(p+1);
	}
}

