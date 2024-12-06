/*
 * SPDX-FileCopyrightText: 2011-2013 Gerhard Reitmayr, TU Graz
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_TIMINGS_HPP
#define SE_TIMINGS_HPP

#include "perfstats.hpp"

#define TICK(str) se::perfstats.sampleDurationStart(str);
#define TICKD(str) se::perfstats.sampleDurationStart(str, true);
#define TOCK(str) se::perfstats.sampleDurationEnd(str);

#endif // SE_TIMINGS_HPP
