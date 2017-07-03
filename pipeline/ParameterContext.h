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
 * Created on: June 4 2014
 *
 */

#ifndef PARAMETER_CONTEXT_H_
#define PARAMETER_CONTEXT_H_

#include <PclRefLIB.h>
#include <DataAccessor.h>

namespace pclref
{
  /**
   * \class ParameterContext
   *
   * \brief Holding parameters for ProcessingSteps
   *
   * The ParameterContext encapsulates parameters in a convinient way. Every
   * ProcessingStep uses it
   *
   * \note For parameter names, look up documentation of ProcessingSteps you use.
   */
  class ParameterContext
  {
  public:
    typedef boost::shared_ptr<ParameterContext> Ptr; ///< type of smart pointer of this class
    /// Param holding all types a parameter can take
    typedef boost::any Param;
    typedef std::pair<std::string, Param> NamedParam; ///< type of a name parameter pair

    /// constructs a ParameterContext
    ParameterContext()
    {
    }
    /** \param params name parameter pairs
      *
      * constructs a ParameterContext from name parameter pairs.
      */
    ParameterContext(const std::vector<NamedParam>& params)
    {
      BOOST_FOREACH(NamedParam param, params)
      {
        m_params.insert(param);
      }
    }

    /// count of parameters. stl style size function
    size_t size() const
    {
      return m_params.size();
    }

    /** \brief Checks if a parameters by this name exists.
      * \param name name of the parameter to query for
      * \return true if parameter exists
      *
      * This method checks if an object by this name exists.
      */
    bool exists(const std::string& name) const
    {
      return m_params.find(name) != m_params.end();
    }
    /** \brief Gets a parameter by its name.
      * \param name name of the parameter to get
      * \return the parameter
      *
      * This method gets a variant containing the parameter by its name.
      *
      * \note a parameter by this name must exist
      */
    Param getParam(const std::string& name)
    {
      return m_params[name];
    }
    /** \brief Gets a parameter by its name.
      * \param name name of the parameter to get
      * \return the parameter
      *
      * This method gets a parameter by its name.
      */
    template<typename T>
    T get(const std::string& name)
    {
      std::map<std::string, Param>::iterator iter = m_params.find(name);
      if(iter == m_params.end())
        return T();
      return boost::any_cast<T>(iter->second);
    }
    /** \brief Gets a parameter by its name.
      * \param name name of the parameter to get
      * \param def the default value to return
      * \return the parameter
      *
      * This method gets an parameter by its name. If it is not present,
      * this method returns a default value.
      */
    template<typename T>
    T get(const std::string& name, T def)
    {
      std::map<std::string, Param>::iterator iter = m_params.find(name);
      if(iter == m_params.end())
        return def;
      return boost::any_cast<T>(iter->second);
    }
    /** \brief Set a named parameter.
      * \param name name of the parameter to be set
      * \param param the parameter to be set
      *
      * This method sets a name parameter pair.
      */
    template<typename T>
    void set(const std::string& name, const T& param)
    {
      m_params[name] = param;
    }

    /// sets name of this parameter context
    void setName(const std::string& name)
    {
      m_name = name;
    }
    /// gets name of this parameter context
    std::string getName() const
    {
      return m_name;
    }


  protected:
    std::map<std::string, Param> m_params; ///< a map of name parameter pairs
    std::string m_name; ///< the name
  };
}

#endif
