/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/planner/planner_dispatcher.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/open_space/open_space_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"

namespace apollo {
namespace planning {
// 注册5个planner, 但配置文件中使用的是EMplanner
void PlannerDispatcher::RegisterPlanners() {
  planner_factory_.Register(    // 注册RTK延时的Planner
      PlanningConfig::RTK, []() -> Planner* { return new RTKReplayPlanner(); });
  planner_factory_.Register(PlanningConfig::EM, // 注册EM的Planner
                            []() -> Planner* { return new EMPlanner(); });
  planner_factory_.Register(PlanningConfig::LATTICE, // 注册LatticePlanner
                            []() -> Planner* { return new LatticePlanner(); });   // 网格的planner
  planner_factory_.Register(PlanningConfig::OPENSPACE, []() -> Planner* {
    return new OpenSpacePlanner();   // 注册开放道路的Planner
  });
  planner_factory_.Register(PlanningConfig::NAVI,   //注册导航地图的Planner
                            []() -> Planner* { return new NaviPlanner(); });
}

}  // namespace planning
}  // namespace apollo
