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

#ifndef PROCESSING_CONTEXT_H_
#define PROCESSING_CONTEXT_H_

#include <PclRefLIB.h>
#include <DataAccessor.h>

#include <ParameterContext.h>

namespace pclref
{
  /**
  * \class ProcessingContext
  *
  * \brief Holding useful objects and typedefs for ProcessingStep
  *
  * The ProcessingContext encapsulates typedefintions and processed objects.
  * Any ProcessingStep relies on this class, as this class provides all objects
  * in a coherent manner.
  *
  * \note Note that each objects needs to be uniquely named to avoid overriding.
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class ProcessingContext
  {
  public:
    typedef boost::shared_ptr<ProcessingContext<PointType, KeypointType, FeatureType> > Ptr; ///< type of smart pointer of this class

    //typedef pcl::PointXYZI KeypointType; ///< type of keypoints to use

    typedef typename pcl::PointCloud<PointType> PointCloud; ///< type of point cloud
    typedef typename pcl::RangeImage RangeImage; ///< type of the range image to use
    typedef pcl::PointCloud<pcl::Normal> NormalCloud; ///< type of a cloud of normals
    typedef pcl::PointCloud<KeypointType> KeypointCloud; ///< type of a cloud of keypoints
    typedef pcl::PointCloud<int> IndexCloud; ///< type of a cloud of indices
    typedef pcl::PointCloud<typename FeatureType::Type> FeatureCloud; ///< type of a cloud of features
    typedef Eigen::Matrix4f Transformation; ///< transformation
    typedef std::vector<pcl::PointIndices> Clusters; ///< type of clusters of a point cloud, may overlap

    /// a boost any to hold all types used by this framework
    typedef boost::any Datum;

    /// stl style iterator
    typedef typename std::vector<Datum>::iterator iterator;

    /** \brief Set a named object.
      * \param name name of the object to be set
      * \param value the object to be set
      *
      * This method sets a name object pair.
      */
    template<class T>
    void set(const std::string& name, T value)
    {
      typename std::map<std::string, identifier>::iterator iter = m_indices.find(name);
      if(iter == m_indices.end())
      {
        m_indices.insert(std::make_pair(name, m_data.size()));
        m_data.push_back(Datum(value));
      }
      else
      {
        m_data[iter->second] = value;
      }
    }

    /** \brief Add a named object.
      * \param name name of the object to be added
      * \param datum the object to be added
      *
      * This method adds a name object pair.
      *
      * \note An object by this name must not be present!
      */
    void add(const std::string& name, const Datum& datum)
    {
      m_indices.insert(std::make_pair(name, m_data.size()));
      m_data.push_back(datum);
    }

    /// Clears the content of this context
    void clear()
    {
      m_data.clear();
      m_indices.clear();
    }

    /** \brief Gets an object by its name.
      * \param name name of the parameter to get
      * \return the object
      *
      * This method gets a variant containing the object by its name.
      *
      * \note The object must be present!
      */
    Datum getDatum(const std::string& name)
    {
      return m_data[m_indices[name]];
    }
    /** \brief Gets an object by its name.
      * \param name name of the object to get
      * \return the object
      *
      * This method gets an object by its name.
      *
      * \note The object must be present!
      */
    template<typename T>
    T get(const std::string& name)
    {
      return boost::any_cast<T>(m_data[m_indices[name]]);
    }
    /** \brief Gets an object by its name, if it does not exist gets alternative object.
      * \param name name of the object to get
      * \param name_alt alternative name of the (or another) object to get
      * \return the object
      *
      * This method gets an object by its name. If no such object can be found it returns
      * the object by the alternative name given.
      *
      * \note At least the alternative object must be present!
      */
    template<typename T>
    T get(const std::string& name, const std::string& name_alt)
    {
      std::string s = exists(name) ? name : name_alt;
      return boost::any_cast<T>(m_data[m_indices[s]]);
    }
    /** \brief Checks if an object by this name exists.
      * \param name name of the object to query for
      * \return true if object exists
      *
      * This method checks if an object by this name exists.
      */
    bool exists(const std::string& name) const
    {
      return m_indices.find(name) != m_indices.end();
    }
    /** \brief Removes an object if one with this name is present
      * \param name name of the object to be removed
      *
      * Removes an object if one with this name is present.
      */
    void remove(const std::string& name)
    {
      std::map<std::string, identifier>::iterator iter = m_indices.find(name);
      if(iter != m_indices.end())
      {
        m_data[iter->second] = 0;
        m_indices.erase(iter);
      }

    }

    /// stl style iterator begin
    iterator begin()
    {
      return m_data.begin();
    }
    /// stl style iterator end
    iterator end()
    {
      return m_data.end();
    }
    /// stl style size
    size_t size()
    {
      return m_data.size();
    }
    iterator operator[] (const int idx)
    {
      assert(idx >=0 && idx < m_data.size());
      return (m_data.begin() + idx);
    }

  protected:
    std::map<std::string, identifier> m_indices; ///< maps names to indices
    std::vector<Datum> m_data; ///< stores objects
  };

}

#endif
