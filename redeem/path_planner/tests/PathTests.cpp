//#define _SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "Path.h"
#include "TestUtils.h"

struct PathTests : ::testing::Test
{
    PathBuilder builder = PathBuilder::CartesianBuilder();
};

TEST_F(PathTests, NoPressureAdvanceByDefault)
{
    // XYE move
    auto path = builder.makePath(VectorN(0.1, 0.1, 0, 0.01), 0.1);
    ASSERT_FALSE(path.willUsePressureAdvance());
}

TEST_F(PathTests, NoPressureAdvanceByDefaultInExtruderMove)
{
    // E-only move
    auto path = builder.makePath(VectorN(0, 0, 0, 0.01), 0.1);
    ASSERT_FALSE(path.willUsePressureAdvance());
}

TEST_F(PathTests, NoPressureAdvanceInExtruderMoveEvenWhenEnabled)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0, 0, 0, 0.01), 0.01);
    ASSERT_FALSE(path.willUsePressureAdvance());
}

TEST_F(PathTests, PressureAdvanceEnabledInMixedMove)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0.1, 0, 0.01), 0.1);
    ASSERT_TRUE(path.willUsePressureAdvance());
}

TEST_F(PathTests, NoPressureAdvanceInNonExtruderMove)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0.1, 0, 0), 0.1);
    ASSERT_FALSE(path.willUsePressureAdvance());
}

TEST_F(PathTests, StepGenerationDeferredForPressureAdvancedAxes)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0.1, 0, 0.01), 0.1);
    ASSERT_FALSE(path.getSteps()[X_AXIS].empty());
    ASSERT_FALSE(path.getSteps()[Y_AXIS].empty());
    ASSERT_TRUE(path.getSteps()[E_AXIS].empty());
}

TEST_F(PathTests, NormalStepCountWithoutPressureAdvance)
{
    auto path = builder.makePath(VectorN(0.1, 0.1, 0, 0.01), 0.1);
    // make sure this is a typical move that can hit full speed
    ASSERT_TRUE(path.willMoveReachFullSpeed());
    path.runFinalStepCalculations();
    ASSERT_EQ(path.getSteps()[E_AXIS].size(), 100);
}

/*
Some simple calculation checks here - when pressure advance is enabled and working,
extruder acceleration gets more steps than normal to "compress" the filament path and
extruder deceleration gets fewer steps than normal to "decompress" the filament path.

When accel and decel match up (as in a move that starts from rest and ends at rest),
the total distance traveled will be the same as without pressure advance but there will
be more steps because the accel phase will move more and the decel phase will "retract"
with a few negative steps.
*/

TEST_F(PathTests, NormalStepCountWhenNoAccelOrDecel)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0.1, 0, 0.01), 0.1);
    path.setStartSpeed(0.1);
    path.setEndSpeed(0.1);
    path.runFinalStepCalculations();
    ASSERT_EQ(path.getSteps()[E_AXIS].size(), 100);
}

TEST_F(PathTests, MoreStepsWhenNoDecel)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0, 0, 0.01), 0.1);
    // path XYZ speed will be 0.1m/s, which will also make the extruder 0.01m/s
    path.setStartSpeed(0.05);
    path.setEndSpeed(0.1);
    path.runFinalStepCalculations();
    ASSERT_EQ(path.getSteps()[E_AXIS].size(), 105); // 0.1 mm/(mm/s) * 5 mm/s = 0.5mm = 5 steps extra
}

TEST_F(PathTests, FewerStepsWhenNoAccel)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0, 0, 0.01), 0.1);
    path.setStartSpeed(0.1);
    path.setEndSpeed(0.05);
    path.runFinalStepCalculations();
    ASSERT_EQ(path.getSteps()[E_AXIS].size(), 95); // 0.1 mm/(mm/s) * 5mm/s = 0.5mm = 5 steps fewer
}

TEST_F(PathTests, NormalStepCountWhenAccelAndDecelMatchButNoRetract)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0, 0, 0.01), 0.1);
    path.setStartSpeed(0.05);
    path.setEndSpeed(0.05);
    path.runFinalStepCalculations();
    ASSERT_EQ(path.getSteps()[E_AXIS].size(), 100); // everything cancels out
}

TEST_F(PathTests, MoreStepsWhenAccelAndDecelMatchAndDecelHasRetract)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.2, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0, 0, 0.01), 0.1);
    ASSERT_TRUE(path.willMoveReachFullSpeed());
    path.runFinalStepCalculations();
    ASSERT_EQ(path.getSteps()[E_AXIS].size(), 104);
}

TEST_F(PathTests, NormalStepsWhenMoveWontReachFullSpeed)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.1, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.1, 0, 0, 0.01), 0.5);
    ASSERT_FALSE(path.willMoveReachFullSpeed());
    path.runFinalStepCalculations();
    ASSERT_EQ(path.getSteps()[E_AXIS].size(), 100);
}

TEST_F(PathTests, PathAccelIsNormalWhenItWontViolateMaxSpeedJump)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.05, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.01, 0, 0, 0.01), 0.1);
    // between accel phase and cruise phase, path will jump factor * accel m/s
    // here, that's 0.05 * 0.1 = 0.005 m/s, which is well below the max speed jump for E
    ASSERT_DOUBLE_EQ(path.getAcceleration(), vabs(VectorN(0.1, 0, 0, 0.1))); // accel is across X and E
}

TEST_F(PathTests, PathAccelIsReducedWhenItWillViolateJerk)
{
    builder.pressureAdvanceFactors = { 0, 0, 0, 0.2, 0, 0, 0, 0 };
    auto path = builder.makePath(VectorN(0.01, 0, 0, 0.01), 0.1);
    // between accel phase and cruise phase, path will jump factor * accel m/s
    // here, that's 0.2 * 0.1 = 0.02 m/s, which exceeds the max speed jump for E
    // and will cause E's accel to be reduced to maxJump / factor = 0.01 / 0.2 = 0.05
    ASSERT_DOUBLE_EQ(path.getAcceleration(), vabs(VectorN(0.05, 0, 0, 0.05))); // accel is across X and E
}