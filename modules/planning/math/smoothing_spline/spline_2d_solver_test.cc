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
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"

#include "gtest/gtest.h"

#include "modules/planning/math/curve_math.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;
using Eigen::MatrixXd;

TEST(constraint_test, test_suit_one) {
  std::vector<double> t_knots{0, 1, 2, 3, 4, 5};    // knots是什么意思？
  std::size_t order = 5;
  Spline2dSolver spline_solver(t_knots, order);

  Spline2dConstraint* constraint = spline_solver.mutable_constraint();
  Spline2dKernel* kernel = spline_solver.mutable_kernel();

  std::vector<double> et{0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};
  std::vector<double> bound(11, 0.2);
  std::vector<std::vector<double>> constraint_data{
      {-1.211566924, 434592.7844, 4437011.568},
      {-1.211572116, 434594.6884, 4437006.498},
      {-1.21157766, 434596.5923, 4437001.428},
      {-1.211571616, 434598.4962, 4436996.358},
      {-1.21155227, 434600.4002, 4436991.288},
      {-1.211532017, 434602.3043, 4436986.218},
      {-1.21155775, 434604.2083, 4436981.148},
      {-1.211634014, 434606.1122, 4436976.077},
      {-1.211698593, 434608.0156, 4436971.007},
      {-1.211576177, 434609.9191, 4436965.937},
      {-1.211256197, 434611.8237, 4436960.867}};
  std::vector<double> angle;
  std::vector<Vec2d> ref_point;

  for (std::size_t i = 0; i < 11; ++i) {
    angle.push_back(constraint_data[i][0]);
    Vec2d prev_point(constraint_data[i][1], constraint_data[i][2]);

    Vec2d new_point = prev_point;
    ref_point.emplace_back(new_point.x(), new_point.y());
  }

  EXPECT_TRUE(constraint->Add2dBoundary(et, angle, ref_point, bound, bound));
  EXPECT_TRUE(constraint->AddThirdDerivativeSmoothConstraint());
  kernel->AddThirdOrderDerivativeMatrix(100);
  // kernel->add_second_order_derivative_matrix(100);
  // kernel->add_derivative_kernel_matrix(100);

  kernel->AddRegularization(0.1);
  // constraint->add_point_angle_constraint(0, -1.21);
  EXPECT_TRUE(spline_solver.Solve());

  MatrixXd gold_res(51, 6);
  // clang-format off
  gold_res <<
-1.211536842077514109 ,  434592.66747698455583,   4437011.3104558130726,   3.8080526249093837876,   -10.139728378837974176,  -4.9702012767283664302e-06, // NOLINT
-1.2115424151423492827,  434593.04828098160215,   4437010.2964778374881,   3.8080266324837173109,   -10.139830860402472723,  -5.250894216084755994e-06, // NOLINT
-1.211548079937599498 ,  434593.42908218270168,   4437009.282489891164 ,   3.8079967985589404655,   -10.139925941668183285,  -5.1522704816467339639e-06, // NOLINT
-1.2115534670910343973,  434593.80988023628015,   4437008.2684932174161,   3.807963824342671888 ,   -10.140004109433711221,  -4.7509311852720327338e-06, // NOLINT
-1.2115582901792760762,  434594.19067487574648,   4437007.2544898428023,   3.8079287063501028321,   -10.140059191754115631,  -4.1234548052899087269e-06, // NOLINT
-1.2115623457107336236,  434594.57146594882943,   4437006.2404822465032,   3.8078927364039967252,   -10.140088357940907571,  -3.346398718521420416e-06, // NOLINT
-1.2115655131115372622,  434594.95225344720529,   4437005.226473021321 ,   3.807857501634689612 ,   -10.140092118562053614,  -2.4963055868708661977e-06, // NOLINT
-1.2115677547171821438,  434595.33303753606742,   4437004.2124645449221,   3.807824884480089267 ,   -10.140074325441972292,  -1.6497122000467622075e-06, // NOLINT
-1.2115691157708639025,  434595.71381858363748,   4437003.1984586389735,   3.8077970626856774139,   -10.140042171661530546,  -8.8315837695925047957e-07, // NOLINT
-1.2115697244277590094,  434596.09459719056031,   4437002.1844562413171,   3.807776509304506618 ,   -10.140006191558054383,  -2.7319352839184158338e-07, // NOLINT
-1.2115697917627634705,  434596.47537421970628,   4437001.1704570697621,   3.8077659926972025062,   -10.139980260725319994,  1.0362151721092292462e-07, // NOLINT
-1.2115695060626494595,  434596.85615080740536,   4437000.1564594125375,   3.8077678515443436069,   -10.139976408292163512,  4.6408745318981610703e-07, // NOLINT
-1.2115687107848465143,  434597.23692818894051,   4436999.1424612496048,   3.8077816733989728881,   -10.139988712761946132,  1.0340515817845881855e-06, // NOLINT
-1.2115672131285435409,  434597.61770749953575,   4436998.1284614419565,   3.8078062133014105584,   -10.140007918433125766,  1.7505174784501674035e-06, // NOLINT
-1.2115648885226146803,  434597.99848974274937,   4436997.1144596887752,   3.8078400821457005776,   -10.140026487871562466,  2.550493252490749351e-06, // NOLINT
-1.2115616806216027435,  434598.3792757759802 ,   4436996.1004563607275,   3.8078817466796182067,   -10.140038601910511318,  3.3709929708283110962e-06, // NOLINT
-1.2115576013019537793,  434598.76006629614858,   4436995.0864523220807,   3.8079295295046677872,   -10.140040159650634877,  4.1490353176471159136e-06, // NOLINT
-1.2115527306601105995,  434599.1408618252608 ,   4436994.0724487621337,   3.8079816090760862934,   -10.140028778459988956,  4.8216404409474350213e-06, // NOLINT
-1.2115472170133783081,  434599.52166269585723,   4436993.058447021991 ,   3.8080360197028402247,   -10.140003793974035062,  5.3258259372985455144e-06, // NOLINT
-1.2115412769037934293,  434599.90246903675143,   4436992.0444484241307,   3.8080906515476260488,   -10.139966260095631512,  5.598602925861600625e-06, // NOLINT
-1.2115351951045365553,  434600.28328075853642,   4436991.030454098247 ,   3.808143250626879972 ,   -10.139918948995036985,  5.576973162632266028e-06, // NOLINT
-1.2115294702539036731,  434600.66409751231549,   4436990.0164647698402,   3.8081903561870884545,   -10.139868008034298441,  4.8077597313767370503e-06, // NOLINT
-1.2115251375632734021,  434601.04491840628907,   4436989.0024801855907,   3.808224824160084232 ,   -10.139826306862863348,  3.0486030083292842975e-06, // NOLINT
-1.2115231321195760739,  434601.42574184050318,   4436987.9884988041595,   3.8082402598548479311,   -10.13980562520879225 ,  5.5275025457330651254e-07, // NOLINT
-1.2115241146872521849,  434601.80656567483675,   4436986.9745180681348,   3.8082322584127257237,   -10.139814590034983866,  -2.4265100165787592887e-06, // NOLINT
-1.2115284716799898934,  434602.18738742824644,   4436985.9605347122997,   3.8081984048074284388,   -10.139858675539189292,  -5.6358560536309203708e-06, // NOLINT
-1.2115363151431126632,  434602.56820447766222,   4436984.9465450821444,   3.8081382738450333392,   -10.139940203154003129,  -8.8219528535762411261e-06, // NOLINT
-1.2115474827532255464,  434602.94901425705757,   4436983.9325454477221,   3.8080534301639805683,   -10.140058341546867027,  -1.1731478591964790206e-05, // NOLINT
-1.2115615378377246891,  434603.32981445622863,   4436982.9185323221609,   3.8079474282350775916,   -10.140209106620064361,  -1.4111151966075817204e-05, // NOLINT
-1.2115777694127920494,  434603.71060322003905,   4436981.9045027736574,   3.8078258123614956432,   -10.140385361510730888,  -1.5707754920828727438e-05, // NOLINT
-1.2115951922344931901,  434604.09137934719911,   4436980.8904547393322,   3.8076961166787692825,   -10.140576816590844089,  -1.6268145228448618606e-05, // NOLINT
-1.2116125889487479039,  434604.47214246902149,   4436979.8763873716816,   3.8075670315926619658,   -10.140769106614820672,  -1.5673636657669147182e-05, // NOLINT
-1.2116287906335816427,  434604.85289298970019,   4436978.8623015228659,   3.8074450081129960211,   -10.140943390519536749,  -1.4090310598988598536e-05, // NOLINT
-1.2116428203719258327,  434605.23363187833456,   4436977.8481998452917,   3.8073351923255067675,   -10.141083264606631786,  -1.1692162504206557436e-05, // NOLINT
-1.21165388963954479  ,  434605.61436058417894,   4436976.834086435847 ,   3.8072420352090459161,   -10.141176277484222013,  -8.6530843040166213536e-06, // NOLINT
-1.2116613982288511053,  434605.99508096667705,   4436975.8199664391577,   3.8071692926355793496,   -10.141213930066893312,  -5.1468717174906078468e-06, // NOLINT
-1.2116649341851788435,  434606.37579522602027,   4436974.8058456517756,   3.8071200253701888983,   -10.141191675575703002,  -1.3472491624807740227e-06, // NOLINT
-1.2116642737662484119,  434606.75650583388051,   4436973.7917301254347,   3.8070965990710723403,   -10.141108919538183386,  2.5720935559557779256e-06, // NOLINT
-1.2116593814313549871,  434607.13721546367742,   4436972.7776257768273,   3.8071006842895429578,   -10.140969019788338201,  6.4374585958675587059e-06, // NOLINT
-1.2116504098626179609,  434607.51792692107847,   4436971.7635379871354,   3.8071332564700290924,   -10.140779286466644393,  1.0075097281741009924e-05, // NOLINT
-1.2116377000164157973,  434607.89864307461539,   4436970.7494712099433,   3.8071945959500750334,   -10.140550982020052118,  1.3311172977170112426e-05, // NOLINT
-1.2116217746240613984,  434608.27936676860554,   4436969.7354286238551,   3.8072835982384574116,   -10.140297281596426515,  1.5999703511677472223e-05, // NOLINT
-1.2116032340269256018,  434608.66010057670064,   4436968.7214123532176,   3.8073960888063611563,   -10.140025573140695414,  1.8150697288770883026e-05, // NOLINT
-1.2115826280259958114,  434609.04084661870729,   4436967.7074239440262,   3.8075275641774672941,   -10.139740809308877445,  1.9824220542444219705e-05, // NOLINT
-1.2115604413276630513,  434609.42160658695502,   4436966.6934644868597,   3.8076739776390393644,   -10.139447151801833868,  2.1080345198949189623e-05, // NOLINT
-1.2115370935378786399,  434609.80238179198932,   4436965.6795346960425,   3.8078317392419229748,   -10.139147971365266798,  2.1979154386414284563e-05, // NOLINT
-1.211512939152885826 ,  434610.18317320814822,   4436964.6656349897385,   3.8079977158005493543,   -10.138845847789720978,  2.2580746160597712412e-05, // NOLINT
-1.2114882675477143259,  434610.56398151925532,   4436963.6517655644566,   3.8081692308929300239,   -10.138542569910585556,  2.2945235784308586307e-05, // NOLINT
-1.2114633029633761208,  434610.94480716442922,   4436962.6379264798015,   3.8083440648606621259,   -10.138239135608092312,  2.3132756898251399347e-05, // NOLINT
-1.211438204493459958 ,  434611.3256503836019 ,   4436961.624117734842 ,   3.8085204548089248711,   -10.137935751807310325,  2.3203461921265504272e-05, // NOLINT
-1.211413066070579303 ,  434611.70651126321172,   4436960.6103393463418,   3.8086970946064799826,   -10.13763183447815841 ,  2.3217522018132972405e-05; // NOLINT
  // clang-format on

  double t = 0;
  for (int i = 0; i < 51; ++i) {
    auto xy = spline_solver.spline()(t);
    const double heading = std::atan2(spline_solver.spline().DerivativeY(t),
                                      spline_solver.spline().DerivativeX(t));
    const double kappa = CurveMath::ComputeCurvature(
        spline_solver.spline().DerivativeX(t),
        spline_solver.spline().SecondDerivativeX(t),
        spline_solver.spline().DerivativeY(t),
        spline_solver.spline().SecondDerivativeY(t));
    EXPECT_NEAR(heading, gold_res(i, 0), 1e-4);
    EXPECT_NEAR(xy.first, gold_res(i, 1), 1e-4);
    EXPECT_NEAR(xy.second, gold_res(i, 2), 1e-4);
    EXPECT_NEAR(spline_solver.spline().DerivativeX(t), gold_res(i, 3), 1e-4);
    EXPECT_NEAR(spline_solver.spline().DerivativeY(t), gold_res(i, 4), 1e-4);
    EXPECT_NEAR(kappa, gold_res(i, 5), 1e-4);
    t += 0.1;
  }
}

}  // namespace planning
}  // namespace apollo
