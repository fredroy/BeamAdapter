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
    
    using Inherit = BaseRodSectionDecorator<DataTypes>;
    
    RodSectionDensityDecorator();
    virtual ~RodSectionDensityDecorator() = default;
    
//    const char* getFieldName() override { return d_density.getName().c_str(); };
    
    
private:
    Data<SReal> d_density; // in HounsField Unit (HU)
};

#ifndef SOFA_PLUGIN_BEAMADAPTER_RODSECTIONDENSITYDECORATOR_CPP
extern template class SOFA_BEAMADAPTER_API RodSectionDensityDecorator<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace beamadapter

