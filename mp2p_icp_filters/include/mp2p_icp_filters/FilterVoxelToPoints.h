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
#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Converts any MetricMap (including MOLA Voxel Maps) to a Point Cloud layer.
 * This is useful for tools like icp-run that require a point cloud for the
 * moving (local) map.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterVoxelToPoints : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterVoxelToPoints, mp2p_icp_filters)
   public:
    FilterVoxelToPoints();

    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        /** Input layer to extract points from (e.g., "localmap") */
        std::string input_layer;

        /** Output point cloud layer name (e.g., "pts") */
        std::string output_layer;
    };

    Parameters params;

   protected:
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

}  // namespace mp2p_icp_filters
