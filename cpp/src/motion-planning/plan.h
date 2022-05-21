#ifndef PLAN_H
#define PLAN_H

#include "motion-planning/Problem.h"

#include <ompl/base/Cost.h>
#include <ompl/base/Planner.h>

#include <functional>
#include <string>
#include <vector>

namespace motion_planning {

using WritePlanFunc = std::function<void(const std::string&,
                                         const Problem::PlanType&)>;

/** Extract the best found solution from the planner */
Problem::PlanType get_solution(ompl::base::PlannerPtr planner);

ompl::base::Cost solution_cost(ompl::base::PlannerPtr planner);

/** Plan the problem
 *
 * Creates a planner from the problem and runs the given planner, and outputs
 * to the given file.
 *
 * If the problem has rotation and/or retraction enabled, then the returned
 * control values will be augmented with those controls.
 *
 * If optimize is set to true, multiple results will be generated and saved to
 * a file.  Only the best plan is returned.
 *
 * @param problem: problem to plan
 * @param outfile_base: basename of the filename to output the results.  If
 *     optimize is false, then it will simply add ".csv" to the end.  If
 *     optimize is true, then potentially multiple files will be generated and
 *     numbered.
 * @param planner_name: name of the planner to use
 * @param timeout: timeout for planning in seconds.  This function will block
 *     at most this amount of time.  If optimize is true, then the full timeout
 *     time will be used to try to improve the plan.
 * @param planner_options: planner-specific options to set.  Use the
 *     query_planner application to see what options are available for a
 *     particular planner name
 * @param optimize: only applies to planners that can optimize.  Will cause the
 *     planner to find better paths after finding the first solution.
 *
 * @return the best found plan.  A single configuration consists of the tendon
 *     tensions in order followed by the rotation and/or retraction values (if
 *     those controls are turned on)
 */
Problem::PlanType plan(const Problem &problem,
                       const std::string &outfile_base,
                       const std::string &planner_name = "RRTConnect",
                       double timeout = 10.0,
                       const Options &planner_options = {},
                       bool optimize = true);

/** Plan the problem
 *
 * @param planner: Planner to run
 * @param outfile_base: basename of the filename to output the results.  If
 *     optimize is false, then it will simply add ".csv" to the end.  If
 *     optimize is true, then potentially multiple files will be generated and
 *     numbered.
 * @param write_func: function to write the plan to a file given the filename
 *     and the plan.
 * @param timeout: timeout for planning in seconds.  This function will block
 *     at most this amount of time.  If optimize is true, then the full timeout
 *     time will be used to try to improve the plan.
 * @param optimize: only applies to planners that can optimize.  Will cause the
 *     planner to find better paths after finding the first solution.
 *
 * @return the best found plan.  A single configuration consists of the tendon
 *     tensions in order followed by the rotation and/or retraction values (if
 *     those controls are turned on)
 */
Problem::PlanType plan(
    ompl::base::PlannerPtr planner,
    const std::string &outfile_base,
    WritePlanFunc &write_func,
    double timeout = 10.0,
    bool optimize = true);

} // end of namespace motion_planning

#endif // PLAN_H
