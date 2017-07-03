#include <ParameterTemplate.h>

namespace pclref
{
  ParameterTemplate::ParameterTemplate()
  {
  }

  void ParameterTemplate::load(const std::string& filename)
  {
    std::ifstream file(filename.c_str());
    load(file);
    file.close();
  }

  void ParameterTemplate::load(std::ifstream& file)
  {
    std::string line;
    while( std::getline(file, line) )
    {
      if(line[0] == '#')
        continue;

      std::istringstream is_line(line);
      std::string key;
      if(std::getline(is_line, key, '='))
      {
        std::string value;
        if(std::getline(is_line, value))
        {
          if(value[0] == '[')
          {
            parseInterval(key, value);
          }
          else if(value[0] == 'f')
          {
            set(key, false);
          }
          else if(value[0] == 't')
          {
            set(key, true);
          }
          else
          {
            parseValue(key, value);
          }
        }
      }
    }
  }

  void ParameterTemplate::generate(std::vector<ParameterContext::Ptr>& contexts)
  {
    std::vector<double> values;
    BOOST_FOREACH(NamedInterval& i, m_intervals)
    {
      values.push_back(i.second.min);
    }

    std::vector<NamedParam> params;
    std::copy(this->m_params.begin(), this->m_params.end(), std::back_inserter(params));

    while(true)
    {
      contexts.push_back(ParameterContext::Ptr(new ParameterContext(params)));

      if(m_intervals.empty())
      {
        contexts.back()->setName(m_name);
        break;
      }

      std::string name = m_name;
      if(generate(contexts, name, values, m_intervals.size()-1))
        break;
    }
  }

  bool ParameterTemplate::generate(std::vector<ParameterContext::Ptr> &contexts, std::string& name,
                                   std::vector<double>& values, int level)
  {
    size_t idx = m_intervals.size()-1 - level;
    NamedInterval interval = m_intervals[idx];
    ParameterContext::Ptr p = contexts.back();
    if(interval.second.boolean)
      p->set(interval.first, (bool)(values[idx] > 0.0));
    else if(interval.second.real)
      p->set(interval.first, values[idx]);
    else
      p->set(interval.first, (int)values[idx]);

    name += "_" + interval.first + "_" + precision_cast(values[idx]);
    if(level == 0)
    {
      p->setName(name);
    }

    bool overflow = false;
    if(level == 0 || generate(contexts, name, values, level-1))
    {
      values[idx] += interval.second.step;
      if(values[idx] > interval.second.max + std::numeric_limits<double>::epsilon())
      {
        values[idx] = interval.second.min;
        overflow = true;
      }
    }
    return overflow;
  }

  bool ParameterTemplate::parseInterval(const std::string& key, const std::string& value)
  {
    interval i;
    std::string token;

    std::string v = value;
    v.erase(std::remove(v.begin(), v.end(), ' '), v.end());

    if(v[v.size()-1] == 'b')
    {
      i.boolean = true;
      i.real = false;
      i.min = 0.0;
      i.max = 1.0;
      i.step = 1.0;
    }
    else
    {
      i.boolean = false;
      i.real = v[v.size()-1] != 'i';

      std::istringstream is_line(v.substr(1, v.size()-3));
      // i know many people say these should be individual try catch blocks
      // but fuck those people
      try
      {
        if(std::getline(is_line, token, ','))
        {
          i.min = boost::lexical_cast<double>(token);
        }
        if(std::getline(is_line, token, ','))
        {
          i.max = boost::lexical_cast<double>(token);
        }
        if(std::getline(is_line, token))
        {
          i.step = boost::lexical_cast<double>(token);
        }
      }
      catch (...)
      {
        return false;
      }
    }

    m_intervals.push_back(NamedInterval(key, i));
    return true;
  }

  bool ParameterTemplate::parseValue(const std::string& key, const std::string& value)
  {
    bool done = false;
    try
    {
      int v = boost::lexical_cast<int>(value);
      set(key, v);
      done = true;
    }
    catch (boost::bad_lexical_cast)
    {
    }
    if(!done)
    {
      try
      {
        double v = boost::lexical_cast<double>(value);
        set(key, v);
        done = true;
      }
      catch (boost::bad_lexical_cast)
      {
        set(key, value);
        done = true;
      }
    }
    return done;
  }
}

