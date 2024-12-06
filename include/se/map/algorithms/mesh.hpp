/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MESH_FACE_HPP
#define SE_MESH_FACE_HPP

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <vector>

#include "se/map/utils/setup_util.hpp"
#include "se/map/utils/type_util.hpp"

namespace se {

template<size_t NumVertexes, Colour ColB>
struct MeshFaceColourData {
};

template<size_t NumVertexes>
struct MeshFaceColourData<NumVertexes, Colour::On> {
    std::array<rgb_t, NumVertexes> vertex_colours;
};

template<size_t NumVertexes, Semantics SemB>
struct MeshFaceSemanticData {
    semantics_t class_id = 0;
};

template<size_t NumVertexes>
struct MeshFaceSemanticData<NumVertexes, Semantics::Off> {
};


template<size_t NumVertexes, Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
struct MeshFace : MeshFaceColourData<NumVertexes, ColB>, MeshFaceSemanticData<NumVertexes, SemB> {
    static constexpr bool colour = ColB == Colour::On;
    static constexpr bool semantics = SemB != Semantics::Off;
    static constexpr size_t num_vertexes = NumVertexes;

    std::array<Eigen::Vector3f, NumVertexes> vertexes;
    int8_t max_vertex_scale = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/** \brief Meshes are represented as lists of faces.
 *
 * \bug This representation has the inherent problem that there is vertex duplication. A more
 * advanced representation would be needed to alleviate this, e.g. a list of vertices and a list of
 * faces with indices to the list of faces.
 */
template<typename FaceT>
using Mesh = std::vector<FaceT>;

} // namespace se

#endif // SE_MESH_FACE_HPP
