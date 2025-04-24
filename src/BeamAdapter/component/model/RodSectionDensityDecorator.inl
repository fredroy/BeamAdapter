#pragma once

#include <BeamAdapter/component/model/RodSectionDensityDecorator.h>

namespace beamadapter
{

template <typename DataTypes>
RodSectionDensityDecorator<DataTypes>::RodSectionDensityDecorator()
    : Inherit()
    , d_density(initData(&d_density, "density", "Defines the density on the material of this section in HounsField Unit (HU)"))
{
    
}

} // namespace beamadapter
