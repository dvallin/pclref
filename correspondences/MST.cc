#include <MST.h>
#include <DataTable.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

namespace pclref
{
  template<typename P, typename K, typename F>
  MST<P, K, F>::MST()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void MST<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("MST_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int MST<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    // this step is run accumulated.
    if(context->exists("accu") && context->template get<bool>("accu"))
    {
      typename ContextType::Ptr accu = context->template get<typename ContextType::Ptr>("ceContexts");

      using namespace boost;
      typedef property<edge_weight_t, int> EdgeWeightProperty;
      typedef boost::adjacency_list<listS, vecS, directedS, no_property, EdgeWeightProperty > mygraph;
      typedef mygraph::edge_descriptor Edge;

      mygraph g;
      int num_nodes;
      typename ContextType::iterator iterI = accu->begin();
      typename ContextType::iterator iend = accu->end();
      /* Normalization seems to be a bad idea! Huh, who knew?
      float min_score = 1.0f;
      float max_score = 0.0f;
      for(; iterI != iend; ++iterI)
      {
          typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iterI);
          int i = sourceContext->template get<identifier>("source_id");
          num_nodes = num_nodes > i ? num_nodes : i;
          float score = sourceContext->template get<float>("ransac_score");
          if(score < 1.0)
          {
              min_score = score < min_score ? score : min_score;
              max_score = score > max_score ? score : max_score;
          }
      }
      */
      std::vector<typename ContextType::Ptr> context_map;
      context_map.resize(num_nodes * num_nodes);
      iterI = accu->begin();
      for(; iterI != iend; ++iterI)
      {
        typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iterI);
        int i = sourceContext->template get<identifier>("source_id") - 1;
        int j = sourceContext->template get<identifier>("target_id") - 1;
        float score = sourceContext->template get<float>("ransac_score");
        assert(i+j*num_nodes < num_nodes*num_nodes);
        assert(i != j);
        context_map[i + j * num_nodes] = sourceContext;

        score = (score == 1) ? 0.0 : score;
        float weight = 1 - score;
        add_edge(i, j, weight, g);
      }
      std::list < Edge > s;
      kruskal_minimum_spanning_tree (g, std::back_inserter(s));

      typename ContextType::Ptr mstContext(new ContextType);
      int k = 0;
      for (std::list < Edge >::iterator ei = s.begin(); ei != s.end(); ++ei)
      {
        int i = source(*ei, g);
        int j = target(*ei, g);
        assert(i+j*num_nodes < num_nodes*num_nodes);
        assert(i != j);
        mstContext->add("context_" + precision_cast(k++, 1), context_map[i + j * num_nodes]);
      }

      context->set("mstContext", mstContext);
    }
#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string MST<P, K, F>::getName() const
  {
    return "MST";
  }
}

PCLREF_INSTANTIATE_PRODUCT(MST,PCLREF_TYPES_PRODUCT)

