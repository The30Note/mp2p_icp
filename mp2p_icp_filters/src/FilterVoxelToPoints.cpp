/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   FilterVoxelToPoints.cpp
 * @brief  Converts any MetricMap to a Point Cloud layer.
 * @author Jose Luis Blanco Claraco
 * @date   May 24, 2024
 */

#include <mp2p_icp_filters/FilterVoxelToPoints.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterVoxelToPoints, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterVoxelToPoints::FilterVoxelToPoints()
{
    mrpt::system::COutputLogger::setLoggerName("FilterVoxelToPoints");
}

void FilterVoxelToPoints::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_layer);
    MCP_LOAD_REQ(c, output_layer);
}

void FilterVoxelToPoints::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START
    params.load_from_yaml(c);
    MRPT_END
}

void FilterVoxelToPoints::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    auto itLy = inOut.layers.find(params.input_layer);
    if (itLy == inOut.layers.end()) return;

    const auto& map = itLy->second;
    if (!map) return;

    // Use the virtual method in CMetricMap to get a point cloud view.
    // This is a generic way that works for MOLA HashedVoxelPointCloud
    // if the MOLA plugin is loaded, or for any other map class.
    const mrpt::maps::CSimplePointsMap* pts = map->getAsSimplePointsMap();

    if (!pts)
    {
        MRPT_LOG_THROTTLE_WARN_FMT(
            5.0, "Layer '%s' does not support conversion to point cloud.",
            params.input_layer.c_str());
        return;
    }

    // Create or get the output layer
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        inOut, params.output_layer,
        /*do not allow empty*/
        false,
        /* create cloud of the same type as the view */
        pts->GetRuntimeClass()->className);

    outPc->copyFrom(*pts);
    outPc->mark_as_modified();

    MRPT_END
}
