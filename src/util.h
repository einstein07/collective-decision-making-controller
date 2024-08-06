/*
 * control.h
 *
 *  Created on: 05 Aug 2024
 *  Author: Sindiso Mkhatshwa
 * 	Email: sindiso.mkhatshwa@uni-konstanz.de
 */

#ifndef UTIL_H_
#define UTIL_H_

/************************************************************************
 * INCLUDES
 ***********************************************************************/

#include <cstdlib> // RAND_MAX
#include <random>
/**
 * \brief GLOBAL VARIABLES
 */

/**
 *\brief Random seed. Default value (="-1") means time based.
 *
 * "Note that boost/c++11 random is slower (by one order of magnitude), but more precise,
 * than old classic rand()"
 *
 */
extern int gRandomSeed;

extern std::random_device rnd;
/**
 * \brief Returns an int with value drawn uniformly in [0,max)
 */
extern std::minstd_rand randint;
/**
 * \brief Engine
 */
extern std::mt19937 engine;
/**
 * \brief Uniform real distribution
 */
extern std::uniform_real_distribution<double> disRandom;
/**
 * \brief Normal distribution
 */
extern std::normal_distribution<> disNormal;
/**
 * \brief Iniform, unbiased
 */
extern std::uniform_int_distribution<int> randomSensor;
/**
 * Uniform in [0,1), return double
 */
#define random() disRandom(engine)
/**
 * \brief Normal distribution mean=0 and stddev=1 (use: mean+rand*stddev)
 */
#define randgaussian() disNormal(engine)

#endif /* UTIL_H_ */