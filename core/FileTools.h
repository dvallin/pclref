/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2014/09/11 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: May 19 2014
 *
 */

#ifndef PCLREF_FILE_TOOLS_H_
#define PCLREF_FILE_TOOLS_H_

#include <PclRefLIB.h>
#include <boost/filesystem.hpp>

namespace pclref
{
  /**
  * \brief simple file utils class.
  */
  class FileTools
  {
  public:
    /// helper fuction to search for files
    static void findFiles(const std::string& root, std::vector<std::string>& ret);
    /// helper fuction to search for files recursively
    static void findFilesRecursive(const std::string& root, std::vector<std::string>& ret);

    /// helper fuction to search for files with specific extension
    static void findFiles(const std::string& root, const std::string& ext,
                          std::vector<std::string>& ret);
    /// helper fuction to search for files with specific extension recursively
    static void findFilesRecursive(const std::string& root, const std::string& ext,
                                   std::vector<std::string>& ret);

    /// helper fuction to create folder if it doesn't exist
    static void createFolder(const std::string& root);

  private:
    DISALLOW_COPY_AND_ASSIGN (FileTools);
  };
}


#endif /* ARRAYS_H_ */
