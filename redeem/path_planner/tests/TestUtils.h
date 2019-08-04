#pragma once

#define _USE_MATH_DEFINES
#include <future>
#include <math.h>
#include <thread>
#undef _USE_MATH_DEFINES

#include "Path.h"
#include "vectorN.h"

struct PathBuilder
{
    VectorN currentPosition;
    VectorN stepsPerM;
    VectorN maxSpeedJumps;
    VectorN maxSpeeds;
    VectorN maxAccelMPerSquareSecond;
    VectorN pressureAdvanceFactors;
    double layerHeight = 0.0002;
    double trackWidth = 0.0005;
    double filamentDiameter = 0.00175;

    static PathBuilder CartesianBuilder()
    {
        return PathBuilder(VectorN(10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000),
            VectorN(0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01),
            VectorN(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
            VectorN(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
    }

    static PathBuilder FullSpeedBuilder()
    {
        return PathBuilder(VectorN(10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000),
            VectorN(1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000),
            VectorN(1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000),
            VectorN(1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000));
    }

    PathBuilder()
    {
    }

    PathBuilder(const VectorN& stepsPerM, const VectorN& maxSpeedJumps, const VectorN& maxSpeeds, const VectorN& maxAccelMPerSquareSecond)
        : stepsPerM(stepsPerM)
        , maxSpeedJumps(maxSpeedJumps)
        , maxSpeeds(maxSpeeds)
        , maxAccelMPerSquareSecond(maxAccelMPerSquareSecond)
    {
    }

    void setPressureAdvanceFactors(const VectorN& factors)
    {
        pressureAdvanceFactors = factors;
    }

    Path makePath(const VectorN& move, double speed)
    {
        Path result;

        VectorN endPosition = currentPosition + move;

        result.initialize(
            (currentPosition * stepsPerM).round(), // machineStart
            (endPosition * stepsPerM).round(), // machineEnd
            currentPosition, // worldStart,
            endPosition, // worldEnd
            stepsPerM,
            maxSpeeds,
            maxAccelMPerSquareSecond,
            maxSpeedJumps,
            pressureAdvanceFactors,
            speed,
            std::numeric_limits<double>::infinity(),
            AXIS_CONFIG_XY,
            *(Delta*)nullptr,
            false,
            false);

        currentPosition += move;

        return result;
    }

    Path makePath(double x, double y, double z, double speed)
    {
        return makePath(VectorN(x, y, z), speed);
    }

    enum class SpeedType
    {
        PrintHeadSpeed,
        CompleteSpeed
    };

    Path makeExtrudedPath(double x, double y, double z, double speed, SpeedType speedType = SpeedType::PrintHeadSpeed)
    {
        const double filamentRadius = filamentDiameter / 2;
        // volume we need to extrude to fill a 1mm by <trackWidth> by <layerheight> box
        const double extruderVolume = trackWidth * layerHeight;
        // distance we need to extrude to generate that volume
        // cylinder volume is pi*r^2*h, so we want volume/(pi*r^2)
        const double extruderRatio = extruderVolume / (M_PI * filamentRadius * filamentRadius);

        const VectorN printHeadMove { x, y, z };
        

        const VectorN completeMove { x, y, z, vabs(printHeadMove) * extruderRatio };

        const double scaledSpeed = [&]() {
            if (speedType == SpeedType::CompleteSpeed)
            {
                return speed;
            }
            else
            {
                const double printHeadMoveTime = vabs(printHeadMove) / speed;
                return vabs(completeMove) / printHeadMoveTime;
			}
        }();

        return makePath(completeMove, scaledSpeed);
    }
};

template <typename T>
bool is_ready(std::future<T> const& future)
{
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

struct WorkerThread
{
    std::promise<void> running;
    std::future<void> runningFuture;
    std::promise<void> finished;
    std::future<void> finishedFuture;

    std::thread thread;

    WorkerThread(std::function<void(void)> func)
        : running()
        , runningFuture(running.get_future())
        , finished()
        , finishedFuture(finished.get_future())
        , thread([this, func]() {
            running.set_value();
            func();
            finished.set_value();
        })
    {
        runningFuture.wait();
    }

    bool isFinished()
    {
        return is_ready(finishedFuture);
    }

    void waitAndJoin()
    {
        finishedFuture.wait();
        thread.join();
    }
};
