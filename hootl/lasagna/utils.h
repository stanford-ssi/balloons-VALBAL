/**
 * This file is a shortened version of the utils.h file from the valbal-trajectory repo.
 * It is here so that we can simulate a differentiable lasagna controller in this repo
 * without relying on all the other trajectory simulation stuff.
 */

#ifndef TRAJ_UTILS_H
#define TRAJ_UTILS_H
#include <time.h>
#include <cmath>
#include <thread>
#include <utility>
#include <mutex>
#include <queue>
#include <functional>
#include <condition_variable>
#include <adept.h>

using namespace std;
using adept::adouble;

inline float VAL(float f) { return f; }
inline float VAL(adouble f) { return f.value(); }
inline adouble cosf(adouble f) { return cos(f); }
inline float fastcos(float f) { return __builtin_cos(f); }
inline adouble fastcos(adouble f) { return cos(f); }

#endif
