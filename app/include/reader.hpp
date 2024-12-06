/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef __READER_HPP
#define __READER_HPP

#include "reader_base.hpp"


namespace se {

/** Create the appropriate reader instance based on the configuration.
   *
   * \param[in] config The pipeline configuration.
   * \return A pointer to an instance of a class derived from Reader.
   */
Reader* create_reader(const se::ReaderConfig& config);

} // namespace se

#endif
