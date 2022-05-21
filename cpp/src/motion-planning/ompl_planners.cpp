#include "ompl_planners.h"

#include "StraightLinePlanner.h"
#include "VoxelCachedLazyPRM.h"
#include "LazyPRMFixed.h"

#include <ompl/config.h>

// load all planners


#include <ompl/base/spaces/RealVectorStateSpace.h>

#if OMPL_VERSION_VALUE < 1005000
# include <ompl/geometric/planners/bitstar/BITstar.h>
#else
# include <ompl/geometric/planners/informedtrees/ABITstar.h>
# include <ompl/geometric/planners/informedtrees/AITstar.h>
# include <ompl/geometric/planners/informedtrees/BITstar.h>
//# include <ompl/geometric/planners/quotientspace/QRRT.h>
# include <ompl/geometric/planners/rlrt/BiRLRT.h>
# include <ompl/geometric/planners/rlrt/RLRT.h>
# include <ompl/geometric/planners/rrt/TSRRT.h>
# include <ompl/geometric/planners/xxl/XXL.h>
#endif

#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/Discretization.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <functional>
#include <map>

namespace motion_planning {

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {
using plan_maker = std::function<ob::PlannerPtr (ob::SpaceInformationPtr)>;

//using plan_maker = decltype(std::make_shared<ob::Planner>);
template <typename T>
ob::PlannerPtr make_shared_planner(ob::SpaceInformationPtr si) {
  return std::make_shared<T>(si);
}

std::map<std::string, plan_maker> plan_maker_map {
  // commented out ones require more arguments

  // My custom planners
  {"StraightLinePlanner", make_shared_planner<StraightLinePlanner>},
  {"VoxelCachedLazyPRM",  make_shared_planner<VoxelCachedLazyPRM>},
  {"LazyPRMFixed",        make_shared_planner<og::LazyPRMFixed>},

  // BITstar variants
  {"BITstar",         make_shared_planner<og::BITstar        >},
#if OMPL_VERSION_VALUE >= 1005000
  {"ABITstar",        make_shared_planner<og::ABITstar       >},
  {"AITstar",         make_shared_planner<og::AITstar        >},

  // Quotient Space (doesn't compile)
  //{"QRRT",            make_shared_planner<og::QRRT           >},

  // RLRT
  {"RLRT",            make_shared_planner<og::RLRT           >},
  {"BiRLRT",          make_shared_planner<og::BiRLRT         >},

  // RRT (doesn't compile as-is, needs one more constructor argument)
  //{"TSRRT",           make_shared_planner<og::TSRRT          >},

  // XXL
  {"XXL",             make_shared_planner<og::XXL            >},
#endif

  // CForest variants
  {"CForest",         make_shared_planner<og::CForest        >},

  // EST variants
  {"EST",             make_shared_planner<og::EST            >},
  {"BiEST",           make_shared_planner<og::BiEST          >},
  {"ProjEST",         make_shared_planner<og::ProjEST        >},

  // FMT variants
  {"FMT",             make_shared_planner<og::FMT            >},
  {"BFMT",            make_shared_planner<og::BFMT           >},

  // KPIECE variants
  {"KPIECE1",         make_shared_planner<og::KPIECE1        >},
  {"BKPIECE1",        make_shared_planner<og::BKPIECE1       >},
  {"LBKPIECE1",       make_shared_planner<og::LBKPIECE1      >},

  // PDST variants
  {"PDST",            make_shared_planner<og::PDST           >},

  // PRM variants
  {"PRM",             make_shared_planner<og::PRM            >},
  {"PRMstar",         make_shared_planner<og::PRMstar        >},
  {"LazyPRM",         make_shared_planner<og::LazyPRM        >},
  {"LazyPRMstar",     make_shared_planner<og::LazyPRMstar    >},
  {"SPARS",           make_shared_planner<og::SPARS          >},
  {"SPARStwo",        make_shared_planner<og::SPARStwo       >},

  // RRT variants
  //{"VFRRT",           make_shared_planner<og::VFRRT          >},
  {"InformedRRTstar", make_shared_planner<og::InformedRRTstar>},
  {"RRTConnect",      make_shared_planner<og::RRTConnect     >},
  {"RRTstar",         make_shared_planner<og::RRTstar        >},
  {"LBTRRT",          make_shared_planner<og::LBTRRT         >},
  {"LazyLBTRRT",      make_shared_planner<og::LazyLBTRRT     >},
  {"BiTRRT",          make_shared_planner<og::BiTRRT         >},
  {"RRT",             make_shared_planner<og::RRT            >},
  {"pRRT",            make_shared_planner<og::pRRT           >},
  {"LazyRRT",         make_shared_planner<og::LazyRRT        >},
  {"RRTsharp",        make_shared_planner<og::RRTsharp       >},
  {"TRRT",            make_shared_planner<og::TRRT           >},
  {"SORRTstar",       make_shared_planner<og::SORRTstar      >},
  {"RRTXstatic",      make_shared_planner<og::RRTXstatic     >},

  // SST variants
  {"SST",             make_shared_planner<og::SST            >},

  // SBL variants
  {"SBL",             make_shared_planner<og::SBL            >},
  {"pSBL",            make_shared_planner<og::pSBL           >},

  // STRIDE variants
  {"STRIDE",          make_shared_planner<og::STRIDE         >},

};

} // end of unnamed namespace

std::vector<std::string> available_planners() {
  std::vector<std::string> names;
  for (auto &kv : plan_maker_map) {
    names.push_back(kv.first);
  }
  return names;
}

ob::PlannerPtr make_planner(ob::SpaceInformationPtr si,
                            const std::string &plannerName)
{
  try {
    return plan_maker_map.at(plannerName)(si);
  } catch (std::out_of_range&) {
    throw std::logic_error("unrecognized planner name: '" + plannerName + "'");
  }
}

std::map<std::string, std::string> planner_options(const std::string &plannerName) {
  std::map<std::string, std::string> options;

  auto space(std::make_shared<ob::RealVectorStateSpace>(2));
  auto si(std::make_shared<ob::SpaceInformation>(space));
  auto planner = make_planner(si, plannerName);
  auto params = planner->params().getParams();

  for (auto &kv : params) {
    options[kv.first] = kv.second->getRangeSuggestion();
  }

  return options;
}

} // end of namespace motion_planning
