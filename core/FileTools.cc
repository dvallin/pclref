/*
 * file_tools.h
 *
 *  Created on: June 27, 2014
 *      Author: max
 */

#include <FileTools.h>
using namespace pclref;

namespace fs = ::boost::filesystem;

void FileTools::findFiles(const std::string& root, std::vector<std::string>& ret)
{
  findFiles(root, "", ret);
}
void FileTools::findFilesRecursive(const std::string& root, std::vector<std::string>& ret)
{
  findFilesRecursive(root, "", ret);
}
void FileTools::findFiles(const std::string& root, const std::string& ext,
                          std::vector<std::string>& ret)
{
  if (!fs::exists(root)) return;

  if (fs::is_directory(root))
  {
    fs::directory_iterator it(root);
    fs::directory_iterator endit;
    while(it != endit)
    {
      if (fs::is_regular_file(it->status()) && (ext.empty() || it->path().extension() == ext))
      {
        ret.push_back(it->path().string());
      }
      ++it;
    }
  }
}
void FileTools::findFilesRecursive(const std::string& root, const std::string& ext,
                                   std::vector<std::string>& ret)
{
  if (!fs::exists(root)) return;

  if (fs::is_directory(root))
  {
    fs::recursive_directory_iterator it(root);
    fs::recursive_directory_iterator endit;
    while(it != endit)
    {
      if (fs::is_regular_file(*it) && (ext.empty() || it->path().extension() == ext))
      {
        ret.push_back(it->path().string());
      }
      ++it;
    }
  }
}
void FileTools::createFolder(const std::string &root)
{
  boost::filesystem::path dir(root);
  if(!boost::filesystem::exists(root))
    boost::filesystem::create_directory(dir);
}

