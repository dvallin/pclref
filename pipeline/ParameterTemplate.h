/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/07/24 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: July 28 2014
 *
 */

#ifndef PARAMETER_TEMPLATE_H_
#define PARAMETER_TEMPLATE_H_

#include <ParameterContext.h>

namespace pclref
{
  /**
   * \class ParameterTemplate
   *
   * \brief Class to generate sets of ParameterContexts
   */
  class ParameterTemplate : public ParameterContext
  {
  public:
    typedef boost::shared_ptr<ParameterTemplate> Ptr; ///< type of smart pointer of this class

    struct interval
    {
      double min, max, step; ///< interval borders
      bool real; ///< is it real or integral
      bool boolean; ///< should it be converted to boolean
    };
    typedef std::pair<std::string, interval> NamedInterval; ///< type of a name interval pair

    /// constructs a ParameterTemplate
    ParameterTemplate();

    /// loads a parameter file by filename
    void load(const std::string& filename);
    /// loads a parameter file by stl stream
    void load(std::ifstream& file);

    /// generate all possible ParameterCotexts based on the current intervals
    void generate(std::vector<ParameterContext::Ptr>& contexts);

  private:
    bool generate(std::vector<ParameterContext::Ptr> &contexts, std::string& name,
                  std::vector<double>& values, int level);

    bool parseInterval(const std::string& key, const std::string& value);
    bool parseValue(const std::string& key, const std::string& value);

  protected:
    std::vector<NamedInterval> m_intervals; ///< currently registered intervals
  };
}

#endif
