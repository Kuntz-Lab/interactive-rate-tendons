#ifndef OMPL_PLANNERS_H
#define OMPL_PLANNERS_H

#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>

#include <map>
#include <string>

namespace motion_planning {

std::vector<std::string> available_planners();

ompl::base::PlannerPtr make_planner(ompl::base::SpaceInformationPtr si,
                                    const std::string &plannerName);

/** Returns a map of {name -> range suggestion} */
std::map<std::string, std::string> planner_options(const std::string &plannerName);

} // end of namespace motion_planning

#endif // PLANNERS_H

