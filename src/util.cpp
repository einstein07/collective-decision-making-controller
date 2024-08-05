/*
 * control.h
 *
 *  Created on: 05 Aug 2024
 *  Author: Sindiso Mkhatshwa
 * 	Email: sindiso.mkhatshwa@uni-konstanz.de
 */
#include "util.h"


// random generator functions, header declaration in common.h (general scope)
std::random_device rnd;
std::minstd_rand randint;
std::mt19937 engine(rnd());
std::uniform_real_distribution<double> disRandom(0.0, 1.0);
std::normal_distribution<> disNormal(0,1);