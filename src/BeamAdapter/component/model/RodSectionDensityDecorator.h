#pragma once

#include <BeamAdapter/config.h>

#include <BeamAdapter/component/model/BaseRodSectionDecorator.h>
#include <sofa/core/objectmodel/BaseObject.h>


namespace beamadapter
{

template <typename DataTypes>
class RodSectionDensityDecorator : public BaseRodSectionDecorator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(BaseRodSectionDecorator, DataTypes), SOFA_TEMPLATE(BaseRodSectionMaterial, DataTypes));
    
    RodSectionDensityDecorator();
    virtual ~RodSectionDensityDecorator() = default;
    
};

} // namespace beamadapter

