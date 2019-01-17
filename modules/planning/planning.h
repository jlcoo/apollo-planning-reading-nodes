/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PLANNING_PLANNING_H_
#define MODULES_PLANNING_PLANNING_H_

#include <memory>
#include <string>

#include "modules/common/apollo_app.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/thread_pool.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/std_planning.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class Planning : public apollo::common::ApolloApp {
 public:
  Planning() {    // 在modules/common/configs/config_gflags.cc中定义为false
    if (FLAGS_use_navigation_mode) {  // 构造函数中根据参数项选择性构造那个函数
      planning_base_ = std::unique_ptr<PlanningBase>(new NaviPlanning()); //转换为基类指针
    } else {                                                              // 在3.0的代码中没有加开放道路
      planning_base_ = std::unique_ptr<PlanningBase>(new StdPlanning());  // 所以使用的是std_planning
    }
  }
  virtual ~Planning() = default;                                                   // planning的析构函数
  /**
   * @brief module name
   * @return module name
   */
  std::string Name() const override { return planning_base_->Name(); }             // 重写

  virtual void RunOnce() { planning_base_->RunOnce(); }                            // 所有的逻辑在run once这个函数中运行

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override { return planning_base_->Init(); }        // 初始化

  /**
   * @brief module start function
   * @return start status
   */
  // 在int ApolloApp::Spin()函数中调用，
  apollo::common::Status Start() override { return planning_base_->Start(); }      // 启动模块

  /**
   * @brief module stop function
   */
  void Stop() override { return planning_base_->Stop(); }                          // 停止模块

 private:
  std::unique_ptr<PlanningBase> planning_base_;                                    // planning的基类
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_H_ */
