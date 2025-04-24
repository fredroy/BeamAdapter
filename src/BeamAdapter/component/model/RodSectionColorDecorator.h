#pragma once

#include <BeamAdapter/config.h>

#include <BeamAdapter/component/model/BaseRodSectionDecorator.h>
#include <sofa/core/objectmodel/BaseObject.h>


namespace beamadapter
{

template <typename DataTypes>
class RodSectionColorDecorator : public BaseRodSectionDecorator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(RodSectionColorDecorator, DataTypes), SOFA_TEMPLATE(BaseRodSectionMaterial, DataTypes));
    
    using Inherit = BaseRodSectionDecorator<DataTypes>;
    
    RodSectionColorDecorator();
    virtual ~RodSectionColorDecorator() = default;
    
private:
    Data<sofa::type::RGBAColor> d_color;
};

#ifndef SOFA_PLUGIN_BEAMADAPTER_RODSECTIONCOLORDECORATOR_CPP
extern template class SOFA_BEAMADAPTER_API RodSectionColorDecorator<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace beamadapter

