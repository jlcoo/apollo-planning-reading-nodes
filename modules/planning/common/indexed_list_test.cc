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

/**
 * @file
 **/

#include <memory>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/util/util.h"
#include "modules/planning/common/indexed_list.h"

namespace apollo {
namespace planning {

using StringIndexedList = IndexedList<int, std::string>;
TEST(IndexedList, Add_ConstRef) {
  StringIndexedList object;   // 一个对象
  {
    ASSERT_NE(nullptr, object.Add(1, "one"));
    ASSERT_NE(nullptr, object.Find(1));
    ASSERT_NE(nullptr, object.Add(1, "one"));
    const auto& items = object.Items();
    ASSERT_EQ(nullptr, object.Find(2));
    ASSERT_EQ(1, items.size());
    ASSERT_EQ("one", *items[0]);   // 直接对迭代器进行索引操作[]
  }
  {
    ASSERT_NE(nullptr, object.Add(2, "two"));
    ASSERT_NE(nullptr, object.Add(2, "two"));
    ASSERT_NE(nullptr, object.Find(1));
    ASSERT_NE(nullptr, object.Find(2));
    const auto& items = object.Items();
    ASSERT_EQ(2, items.size());
    ASSERT_EQ("one", *items[0]);
    ASSERT_EQ("two", *items[1]);
  }
}

TEST(IndexedList, Find) {
  StringIndexedList object;
  object.Add(1, "one");
  auto* one = object.Find(1);    // Find返回的是迭代器
  ASSERT_EQ(*one, "one");
  ASSERT_NE(nullptr, one);
  *one = "one_again";            // 可以对查找的值进行修改
  const auto* one_again = object.Find(1);
  ASSERT_NE(nullptr, one_again);
  ASSERT_EQ("one_again", *one_again);
  ASSERT_EQ(nullptr, object.Find(2));
}

}  // namespace planning
}  // namespace apollo
