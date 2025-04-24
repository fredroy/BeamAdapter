#pragma once

#include <BeamAdapter/component/model/RodSectionColorDecorator.h>

namespace beamadapter
{

template <typename DataTypes>
RodSectionColorDecorator<DataTypes>::RodSectionColorDecorator()
    : Inherit()
    , d_color(initData(&d_color, "color", "Defines the color on the material of this section"))
{
    
}

} // namespace beamadapter
