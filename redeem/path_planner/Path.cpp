/*
  This file is part of Redeem - 3D Printer control software

  Author: Mathieu Monney
  License: GNU GPLv3 http://www.gnu.org/copyleft/gpl.html


  This file is based on Repetier-Firmware licensed under GNU GPL v3 and
  available at https://github.com/repetier/Repetier-Firmware

  Redeem is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Redeem is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Redeem.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "Path.h"
#include "Delta.h"
#include "Logger.h"
#include <algorithm>
#include <cmath>
#include <numeric>

void calculateLinearMove(const int axis, const int startStep, const int endStep, const double time, std::vector<Step>& steps)
{
    const double distance = endStep - startStep;

    const bool direction = startStep < endStep;
    const int stepIncrement = direction ? 1 : -1;
    const double stepOffset = stepIncrement / 2.0;

    int step = startStep;

    steps.reserve(std::abs(endStep - startStep));

    while (step != endStep)
    {
        const double position = step + stepOffset;
        const double stepTime = (position - startStep) / distance * time;

        assert(stepTime > 0 && stepTime < time);
        assert(steps.empty() || stepTime > steps.back().time);

        steps.emplace_back(Step(stepTime, axis, direction));

        step += stepIncrement;
    }
}

void calculateXYMove(const IntVector3& start, const IntVector3& end, const Vector3& stepsPerM, double time, std::array<std::vector<Step>, NUM_AXES>& steps)
{
    for (int i = 0; i < NUM_MOVING_AXES; i++)
    {
        calculateLinearMove(i, start[i], end[i], time, steps[i]);
    }
}

void calculateExtruderMove(const IntVectorN& start, const IntVectorN& end, double time, bool pressureAdvanceEnabled, const VectorN& pressureAdvanceFactors, std::array<std::vector<Step>, NUM_AXES>& steps)
{
    for (int i = NUM_MOVING_AXES; i < NUM_AXES; i++)
    {
        if (pressureAdvanceEnabled && pressureAdvanceFactors[i] != 0)
        {
            continue;
        }

        calculateLinearMove(i, start[i], end[i], time, steps[i]);
    }
}

/*
 * Calculate a move with a pressure advance factor. Note that all distances must be in steps.
 */
void calculatePressureAdvancedExtruderMove(const int axis, const long long endStep,
    const double startSpeed, const double cruiseSpeed, const double endSpeed,
    const double accel, const double pressureAdvanceFactor, const double time,
    std::vector<Step>& steps)
{
    const double& Vi = startSpeed;
    const double& Vc = cruiseSpeed;
    const double& Vf = endSpeed;
    const double& A = accel;
    const double& P = pressureAdvanceFactor;
    const double D = endStep;

    const double Vi2 = Vi * Vi;
    const double Vc2 = Vc * Vc;
    const double Vf2 = Vf * Vf;
    const double A2 = A * A;
    const double P2 = P * P;

    /*
	Pressure advance makes a move trapezoid that starts like this:
	   cruise
	   ______
	  /      \
	 /<accel  \<decel
	/          \

	look like this:
	     cruise
	  /______
	 /<accel 
	/         \
	           \<decel
			    \

    This adds a few interesting characteristics:
	- the accel phase starts and ends faster, so it will cover more steps in the same time
	- the decel phase starts and ends slower, so it will cover fewer steps in the same time
	- if there's enough decel and enough pressure advance, the decel phase may have a "turnaround" where it actually starts retracting
	*/

    // turnaround refers to the time when the decel phase actually reverses direction
    const double turnaroundTime = (Vi2 - 2 * Vc * Vi + Vf2 + 2 * Vc2 - 2 * A * P * Vc + 2 * A * D) / (2 * A * Vc);
    const double turnaroundStep = -(2 * A * P * Vi - Vf2 - A2 * P2 - 2 * A * D) / (2 * A);

    const double accelEndStep = -(Vi2 + 2 * A * P * Vi - Vc2 - 2 * A * P * Vc) / (2 * A);
    const double cruiseEndStep = -(2 * A * P * Vi - Vf2 + Vc2 - 2 * A * P * Vc - 2 * A * D) / (2 * A);
    const double cruiseEndTime = (Vi2 - 2 * Vc * Vi + Vf2 + 2 * A * D) / (2 * A * Vc);
    const double moveEndStep = -P * Vi + P * Vf + D;

    const long long realEndStep = turnaroundTime < time ? turnaroundStep : endStep;
    const long long delta = realEndStep > 0 ? 1 : -1;
    const double stepDelta = delta * 0.5;

    assert(A > 0);
    assert(!std::isnan(A));
    assert(Vc > 0);
    assert(!std::isnan(Vc));
    assert(!std::isnan(turnaroundTime));
    assert(!std::isnan(turnaroundStep));
    assert(!std::isnan(accelEndStep));
    assert(!std::isnan(cruiseEndStep));
    assert(accelEndStep >= 0 || std::abs(accelEndStep) < NEGLIGIBLE_ERROR);
    assert(cruiseEndStep >= 0 || std::abs(cruiseEndStep) < NEGLIGIBLE_ERROR);

    /*
    There are a couple interesting cases here:
    - In the "normal" case, cruiseEndTime < turnaroundTime < time, and we decelerate, stop, and retract a bit
      - the forward move should run until turnaroundStep
      - the backward move should run until moveEndStep
    - In the "shortened" case, cruiseEndTime <= time <= turnaroundTime, and we decelerate but don't turnaround and retract
      - the forward move should run until moveEndStep
      - there will be no backward move
    - In the "abrupt" case, turnaroundTime <= cruiseEndTime < time, and at the end of cruise we immediately turnaround and retract
      - the forward move should run until cruiseEndStep
      - the backward move should run until moveEndStep
    */

    const double forwardMoveEndStep = (cruiseEndTime < turnaroundTime && turnaroundTime < time)
        ? turnaroundStep
        : (cruiseEndTime <= time && time <= turnaroundTime)
            ? moveEndStep
            : (turnaroundTime <= cruiseEndTime)
                ? cruiseEndStep
                : std::numeric_limits<double>::quiet_NaN();

    assert(!std::isnan(forwardMoveEndStep));
    assert(forwardMoveEndStep >= 0 || std::abs(forwardMoveEndStep) < NEGLIGIBLE_ERROR);

    for (long long step = 0; step < std::llround(forwardMoveEndStep); step += delta)
    {
        const double position = step + stepDelta;
        double stepTime = 0;

        if (position < accelEndStep) // acceleration phase
        {
            stepTime = (sqrt(2 * A * position + Vi2 + 2 * A * P * Vi + A2 * P2) - Vi - A * P) / A;
        }
        else if (position < cruiseEndStep) // cruise phase
        {
            stepTime = (2 * A * position + Vi2 + (2 * A * P - 2 * Vc) * Vi + Vc2 - 2 * A * P * Vc) / (2 * A * Vc);
        }
        else if (cruiseEndTime < turnaroundTime) // only decelerate if we're in the "normal" case or the "shortened" case
        {
            stepTime = -(2 * Vc * sqrt(-2 * A * position - 2 * A * P * Vi + Vf2 + A2 * P2 + 2 * A * D) - Vi2 + 2 * Vc * Vi - Vf2 - 2 * Vc2 + 2 * A * P * Vc - 2 * A * D) / (2 * A * Vc);
        }

        assert(!std::isnan(stepTime) && stepTime > 0 && stepTime < time);
        assert(steps.empty() || stepTime > steps.back().time);

        steps.emplace_back(Step(stepTime, axis, delta == 1));
    }

    if (turnaroundTime < time) // only retract in the "normal" and "abrupt" cases
    {
        // the velocity had a zero-crossing where the extruder started retracting - calculate these steps as well
        for (long long step = std::llround(forwardMoveEndStep); step > std::llround(moveEndStep); step -= delta)
        {
            const double position = step - stepDelta;
            const double stepTime = (2 * Vc * sqrt(-2 * A * position - 2 * A * P * Vi + Vf2 + A2 * P2 + 2 * A * D) + Vi2 - 2 * Vc * Vi + Vf2 + 2 * Vc2 - 2 * A * P * Vc + 2 * A * D) / (2 * A * Vc);

            assert(!std::isnan(stepTime) && stepTime > 0 && stepTime < time);
            assert(steps.empty() || stepTime >= steps.back().time);

            // If our forward deceleration move ended at a position ending in 0.5,  the math can work out that we need to take a forward and backward step at the same time
            // resolve this by removing both
            if (stepTime == steps.back().time)
            {
                assert(std::abs(std::abs(forwardMoveEndStep - std::llround(forwardMoveEndStep)) - 0.5) < NEGLIGIBLE_ERROR);
                steps.pop_back();
                continue;
            }

            steps.emplace_back(Step(stepTime, axis, delta != 1));
        }
    }
}

VectorN calculateMaxAccelDueToPressureAdvance(const VectorN& pressureAdvanceFactors, const VectorN& maxAccels,
    const VectorN& maxSpeedJumps)
{
    VectorN result;

    for (int axis = 0; axis < NUM_AXES; axis++)
    {
        result[axis] = maxAccels[axis];

        if (pressureAdvanceFactors[axis] == 0)
        {
            continue;
        }

        double speedJump = maxAccels[axis] * pressureAdvanceFactors[axis];

        if (speedJump > maxSpeedJumps[axis])
        {
            result[axis] = maxSpeedJumps[axis] / pressureAdvanceFactors[axis];
        }
    }

    return result;
}

void Path::zero()
{
    joinFlags = 0;
    flags = 0;
    maxStartSpeed = 0;
    maxEndSpeed = 0;

    distance = 0;
    moveMask = 0;
    estimatedTime = 0;
    speeds.zero();
    worldMove.zero();
    fullSpeed = 0;
    startSpeed = 0;
    endSpeed = 0;
    accel = 0;
    startMachinePos.zero();
    machineMove.zero();
    baseSpeeds.zero();
    pressureAdvanceFactors.zero();

    stepperPath.zero();

    for (auto& stepVector : steps)
    {
        std::vector<Step> empty;
        stepVector.swap(empty);
    }

    syncCallback = nullptr;
    waitEvent = std::optional<std::future<void>>();
    probeResult = std::optional<std::promise<IntVectorN>>();
}

Path::Path()
{
    zero();
}

Path::Path(Path&& path)
{
    *this = std::move(path);
}

Path& Path::operator=(Path&& path)
{
    // move assignment operator
    joinFlags = path.joinFlags;
    flags = path.flags.load();
    maxStartSpeed = path.maxStartSpeed;
    maxEndSpeed = path.maxEndSpeed;

    distance = path.distance;
    moveMask = path.moveMask;
    estimatedTime = path.estimatedTime;
    speeds = path.speeds;
    worldMove = path.worldMove;
    fullSpeed = path.fullSpeed;
    startSpeed = path.startSpeed;
    endSpeed = path.endSpeed;
    accel = path.accel;
    startMachinePos = path.startMachinePos;
    machineMove = path.machineMove;
    baseSpeeds = path.baseSpeeds;
    pressureAdvanceFactors = path.pressureAdvanceFactors;

    stepperPath = path.stepperPath;
    steps = std::move(path.steps);

    syncCallback = path.syncCallback;
    waitEvent = std::move(path.waitEvent);
    probeResult = std::move(path.probeResult);

    return *this;
}

enum class CalculatedValue
{
    Speed,
    Acceleration
};

inline static double calculateMaximumSpeedInternal(const VectorN& worldMove, const VectorN& maxSpeeds, const double distance, CalculatedValue value)
{
    // First we need to figure out the minimum time for the move.
    // We determine this by calculating how long each axis would take to complete its move
    // at its maximum speed.
    double minimumTimeForMove = 0;

    for (int i = 0; i < NUM_AXES; i++)
    {
        if (worldMove[i])
        {
            double minimumAxisTimeForMove = fabs(worldMove[i]) / maxSpeeds[i]; // m / (m/s) = s

            if (value == CalculatedValue::Speed)
            {
                LOG("axis " << i << " needs to travel " << worldMove[i] << " at a maximum of " << maxSpeeds[i] << " which would take " << minimumAxisTimeForMove << std::endl);
            }
            else
            {
                assert(value == CalculatedValue::Acceleration);
                LOG("axis " << i << " needs to accelerate to " << worldMove[i] << " at a maximum rate of " << maxSpeeds[i] << " which would take " << minimumAxisTimeForMove << std::endl);
            }

            minimumTimeForMove = std::max(minimumTimeForMove, minimumAxisTimeForMove);
        }
    }

    return distance / minimumTimeForMove;
}

inline static double calculateMaximumSpeed(const VectorN& worldMove, const VectorN& maxSpeeds, const double distance, int axisConfig, CalculatedValue value)
{
    if (axisConfig == AXIS_CONFIG_DELTA)
    {
        // Fold the entire XYZ distance into X
        VectorN fakeWorldMove(worldMove);
        fakeWorldMove[0] = vabs(fakeWorldMove.toVector3());
        fakeWorldMove[1] = 0;
        fakeWorldMove[2] = 0;

        return calculateMaximumSpeedInternal(fakeWorldMove, maxSpeeds, distance, value);
    }
    else if (axisConfig == AXIS_CONFIG_CORE_XY || axisConfig == AXIS_CONFIG_H_BELT)
    {
        // Fold the XY distance into X
        VectorN fakeWorldMove(worldMove);
        fakeWorldMove[0] = vabs(Vector3(fakeWorldMove[0], fakeWorldMove[1], 0));
        fakeWorldMove[1] = 0;

        return calculateMaximumSpeedInternal(fakeWorldMove, maxSpeeds, distance, value);
    }
    else
    {
        return calculateMaximumSpeedInternal(worldMove, maxSpeeds, distance, value);
    }
}

void Path::initialize(const IntVectorN& machineStart,
    const IntVectorN& machineEnd,
    const VectorN& worldStart,
    const VectorN& worldEnd,
    const VectorN& stepsPerM,
    const VectorN& maxSpeeds, /// Maximum allowable speeds in m/s
    const VectorN& maxAccelMPerSquareSecond,
    const VectorN& maxSpeedJumps,
    const VectorN& pressureAdvanceFactors,
    double requestedSpeed,
    double requestedAccel,
    int axisConfig,
    const Delta& delta,
    bool cancelable,
    bool is_probe)
{
    this->zero();

    machineMove = machineEnd - machineStart;
    worldMove = worldEnd - worldStart;
    distance = vabs(worldMove);
    startMachinePos = machineStart;

    joinFlags = 0;
    flags = (cancelable ? FLAG_CANCELABLE : 0) | (is_probe ? FLAG_PROBE : 0);

    assert(!std::isnan(distance));

    for (int axis = 0; axis < NUM_AXES; axis++)
    {
        if (machineMove[axis] != 0)
        {
            moveMask |= (1 << axis);
        }
    }

    for (int axis = E_AXIS; axis <= C_AXIS; axis++)
    {
        flags |= (isAxisMove(axis) && !isAxisOnlyMove(axis) && pressureAdvanceFactors[axis] != 0) ? FLAG_USE_PRESSURE_ADVANCE : 0;
    }

    // Now figure out if we can honor the user's requested speed.
    fullSpeed = std::min(requestedSpeed, calculateMaximumSpeed(worldMove, maxSpeeds, distance, axisConfig, CalculatedValue::Speed));
    assert(!std::isnan(fullSpeed));

    const double idealTimeForMove = distance / fullSpeed; // m / (m/s) = s
    estimatedTime = (unsigned long long)(F_CPU * idealTimeForMove); // ticks / s * s = ticks

    for (int i = 0; i < NUM_AXES; i++)
    {
        if (worldMove[i])
        {
            speeds[i] = worldMove[i] / idealTimeForMove;
            if (std::abs(speeds[i]) < NEGLIGIBLE_ERROR) // TODO I suspect this could be improved by moving to integer steps?
            {
                speeds[i] = 0;
            }
        }
        else
        {
            speeds[i] = 0;
        }
    }

    baseSpeeds = machineMove.toVectorN() / idealTimeForMove;

    VectorN localMaxAccel;

    if (flags & FLAG_USE_PRESSURE_ADVANCE)
    {
        LOG("move will use pressure advance" << std::endl);
        this->pressureAdvanceFactors = pressureAdvanceFactors;

        localMaxAccel = calculateMaxAccelDueToPressureAdvance(pressureAdvanceFactors, maxAccelMPerSquareSecond, maxSpeedJumps);
    }
    else
    {
        localMaxAccel = maxAccelMPerSquareSecond;
    }

    accel = std::min(requestedAccel, calculateMaximumSpeed(speeds, localMaxAccel, fullSpeed, axisConfig, CalculatedValue::Acceleration));

    // Calculate whether we're guaranteed to reach cruising speed.
    double maximumAccelTime = fullSpeed / accel; // (m/s) / (m/s^2) = s
    double maximumAccelDistance = maximumAccelTime * (fullSpeed / 2.0);
    if (2.0 * maximumAccelDistance < distance)
    {
        // This move has enough distance that we can accelerate from 0 to fullSpeed and back to 0.
        // We'll definitely hit cruising speed.
        flags |= FLAG_WILL_REACH_FULL_SPEED;
    }

    LOG("ideal move should be " << fullSpeed << " m/s and cover " << distance << " m in " << idealTimeForMove << " seconds" << std::endl);

    switch (axisConfig)
    {
    case AXIS_CONFIG_DELTA:
        delta.calculateMove(machineStart.toIntVector3(), machineEnd.toIntVector3(), stepsPerM.toVector3(), idealTimeForMove, steps);
        break;
    case AXIS_CONFIG_XY:
    case AXIS_CONFIG_H_BELT:
    case AXIS_CONFIG_CORE_XY:
        calculateXYMove(machineStart.toIntVector3(), machineEnd.toIntVector3(), stepsPerM.toVector3(), idealTimeForMove, steps);
        break;
    default:
        assert(0);
    }

    calculateExtruderMove(machineStart, machineEnd, idealTimeForMove, !!(flags & FLAG_USE_PRESSURE_ADVANCE), pressureAdvanceFactors, steps);

    LOG("Distance in m:     " << distance << std::endl);
    LOG("Speed in m/s:      " << fullSpeed << " requested: " << requestedSpeed << std::endl);
    LOG("Accel in m/s:     " << accel << " requested: " << requestedAccel << std::endl);
    LOG("Ticks :            " << estimatedTime << std::endl);

    invalidateStepperPathParameters();
}

double Path::runFinalStepCalculations()
{
    updateStepperPathParameters();

    for (auto& axisSteps : steps)
    {
        for (auto& step : axisSteps)
        {
            step.time = stepperPath.dilateTime(step.time);
        }
    }

    if (flags & FLAG_USE_PRESSURE_ADVANCE)
    {
        const double accelTime = (stepperPath.cruiseSpeed - stepperPath.startSpeed) / stepperPath.accel;
        const double decelTime = (stepperPath.cruiseSpeed - stepperPath.endSpeed) / stepperPath.accel;

        assert(accelTime >= 0 && decelTime >= 0);

        const double startFactor = stepperPath.startSpeed / stepperPath.baseSpeed;
        const double cruiseFactor = stepperPath.cruiseSpeed / stepperPath.baseSpeed;
        const double endFactor = stepperPath.endSpeed / stepperPath.baseSpeed;

        for (int axis = E_AXIS; axis <= C_AXIS; axis++)
        {
            if (machineMove[axis] == 0 || pressureAdvanceFactors[axis] == 0)
            {
                assert(machineMove[axis] == 0 || !steps[axis].empty());
                continue;
            }
            else if (accelTime == 0 && decelTime == 0)
            {
                calculateLinearMove(axis, 0, machineMove[axis], stepperPath.moveEnd, steps[axis]);
                continue;
            }

            assert(steps[axis].empty());

            const double startSpeed = startFactor * baseSpeeds[axis];
            const double cruiseSpeed = cruiseFactor * baseSpeeds[axis];
            const double endSpeed = endFactor * baseSpeeds[axis];
            const double accel = accelTime != 0 ? (cruiseSpeed - startSpeed) / accelTime : (cruiseSpeed - endSpeed) / decelTime;

            calculatePressureAdvancedExtruderMove(axis, machineMove[axis],
                startSpeed, cruiseSpeed, endSpeed,
                accel, pressureAdvanceFactors[axis], stepperPath.moveEnd,
                steps[axis]);
        }
    }

    LOG("accelSteps: " << stepperPath.accelSteps
                       << " cruiseSteps: " << stepperPath.cruiseSteps
                       << " decelSteps: " << stepperPath.decelSteps << std::endl);

    return stepperPath.finalTime();
}

/** Update parameter used by updateTrapezoids

Computes the acceleration/decelleration steps and advanced parameter associated.
*/
void Path::updateStepperPathParameters()
{
    if (areParameterUpToDate() || fullSpeed == 0)
        return;

    double cruiseSpeed = fullSpeed;

    double accelTime = (fullSpeed - startSpeed) / accel;
    double decelTime = (fullSpeed - endSpeed) / accel;

    assert(startSpeed <= cruiseSpeed);
    assert(endSpeed <= cruiseSpeed);

    double accelDistance = (cruiseSpeed * cruiseSpeed - startSpeed * startSpeed) / (2.0 * accel);
    double decelDistance = (cruiseSpeed * cruiseSpeed - endSpeed * endSpeed) / (2.0 * accel);
    assert(accelDistance >= 0);
    assert(decelDistance >= 0);

    double cruiseDistance = distance - accelDistance - decelDistance;
    double cruiseTime = cruiseDistance / cruiseSpeed;

    if (accelDistance + decelDistance > distance)
    {
        //           cruiseSpeed                  // (no, C++, this isn't a multiline comment)
        //               /\                       //
        //              /  \                      //
        //             /    \ endSpeed            //
        //            /   ---d2                   //
        //startSpeed /                            //
        //           -----d1                      //
        //
        // vf^2 = vi^2 + 2*a*d
        // -> d = (vf^2 - vi^2) / (2*a)
        // distance = d1 + d2
        // -> distance = (cruiseSpeed^2 - startSpeed^2) / (2*accel) + (endSpeed^2 - cruiseSpeed^2) / (2*-accel) // note the negative accel
        // -> distance = (cruiseSpeed^2 - startSpeed^2) / (2*accel) - (endSpeed^2 - cruiseSpeed^2) / (2*accel)
        // -> distance = (cruiseSpeed^2 - startSpeed^2 - endSpeed^2 + cruiseSpeed^2) / (2*accel)
        // -> 2*accel*distance = 2*cruiseSpeed^2 - startSpeed^2 - endSpeed^2
        // -> 2*accel*distance + startSpeed^2 + endSpeed^2 = 2*cruiseSpeed^2
        // -> cruiseSpeed = sqrt(accel*distance + (startSpeed^2 + endSpeed^2) / 2)
        cruiseSpeed = std::sqrt(accel * distance + (startSpeed * startSpeed + endSpeed * endSpeed) / 2.0);

        // Try to reduce cruiseSpeed 1% so we get just a bit of cruising time. This makes the calculations
        // below more stable.
        cruiseSpeed = std::max(startSpeed, std::max(endSpeed, cruiseSpeed * 0.99));

        accelTime = (cruiseSpeed - startSpeed) / accel;
        decelTime = (cruiseSpeed - endSpeed) / accel;

        accelDistance = (cruiseSpeed * cruiseSpeed - startSpeed * startSpeed) / (2.0 * accel);
        decelDistance = (cruiseSpeed * cruiseSpeed - endSpeed * endSpeed) / (2.0 * accel);

        cruiseDistance = distance - accelDistance - decelDistance;

        if (cruiseDistance < 0)
        {
            // As it turns out, the optimizer can over-accelerate on very short moves because it
            // doesn't check that the end speed of a move is reachable from the start speed within
            // limits. This hasn't been a problem because it only affects moves that are only a few
            // steps in the first place. However, when it occurs, this assert will fire.
            assert(std::abs(cruiseDistance) < NEGLIGIBLE_ERROR);

            if (accelDistance == 0)
            {
                assert(accelTime == 0);

                cruiseDistance = 0;
                cruiseTime = 0;
                cruiseSpeed = startSpeed;

                decelDistance = distance;
                decelTime = distance / ((cruiseSpeed + endSpeed) / 2.0);
            }
            else if (decelDistance == 0)
            {
                assert(decelTime == 0);

                cruiseDistance = 0;
                cruiseTime = 0;
                cruiseSpeed = endSpeed;

                accelDistance = distance;
                accelTime = distance / ((cruiseSpeed + startSpeed) / 2.0);
            }
            else
            {
                assert(0);
            }
        }

        cruiseTime = cruiseDistance / cruiseSpeed;

        LOG("Move will not reach full speed" << std::endl);
    }

    LOG("accelTime: " << accelTime << " cruiseTime: " << cruiseTime << " decelTime: " << decelTime << std::endl);
    assert(accelTime >= 0 && cruiseTime >= 0 && decelTime >= 0);
    assert(accelDistance >= 0 && cruiseDistance >= 0 && decelDistance >= 0);
    assert(std::abs(distance - (accelDistance + cruiseDistance + decelDistance)) < NEGLIGIBLE_ERROR);

    stepperPath.baseSpeed = fullSpeed;
    stepperPath.startSpeed = startSpeed;
    stepperPath.cruiseSpeed = cruiseSpeed;
    stepperPath.endSpeed = endSpeed;
    stepperPath.accel = accel;
    stepperPath.distance = distance;

    const double& Vi = startSpeed;
    const double& Vc = cruiseSpeed;
    const double& Vf = endSpeed;
    const double& A = accel;
    const double& D = distance;

    const double Vi2 = Vi * Vi;
    const double Vc2 = Vc * Vc;
    const double Vf2 = Vf * Vf;

    stepperPath.baseAccelEnd = (-(Vi2 - Vc2)) / (2 * A * Vc);
    stepperPath.baseCruiseEnd = (Vf2 - Vc2 + 2 * A * D) / (2 * A * Vc);
    stepperPath.baseMoveEnd = D / Vc;

    stepperPath.moveEnd = (Vi2 - 2 * Vc * Vi + Vf2 - 2 * Vc * Vf + 2 * Vc2 + 2 * A * D) / (2 * A * Vc);

    joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED;

    assert(areParameterUpToDate());
}

double StepperPathParameters::dilateTime(double t) const
{
    const double& Vi = startSpeed;
    const double& Vc = cruiseSpeed;
    const double& Vf = endSpeed;
    const double& A = accel;
    const double& D = distance;

    const double Vi2 = Vi * Vi;
    const double Vc2 = Vc * Vc;
    const double Vf2 = Vf * Vf;

    assert(t >= 0);

    t *= baseSpeed / cruiseSpeed;

    double result = NAN;

    if (t < baseAccelEnd)
    {
        accelSteps++;
        result = (sqrt(2 * A * Vc * t + Vi2) - Vi) / A;
    }
    else if (t < baseCruiseEnd)
    {
        cruiseSteps++;
        result = (2 * A * Vc * t + Vi2 + (-2) * Vc * Vi + Vc2) / (2 * A * Vc);
    }
    else if (t < baseMoveEnd)
    {
        decelSteps++;
        result = (-(2 * Vc * sqrt((-2) * A * Vc * t + Vf2 + 2 * A * D) - Vi2 + 2 * Vc * Vi - Vf2 + (-2) * Vc2 + (-2) * A * D)) / (2 * A * Vc);
    }
    else
    {
        assert(0);
    }

    assert(!std::isnan(result));

    return result;
}

double StepperPathParameters::finalTime() const
{
    return moveEnd;
}

void SyncCallback::syncComplete()
{
    // this should never be called - this class will be extended in python
    assert(0);
}