#define SOFA_PLUGIN_BEAMADAPTER_RODSECTIONCOLORDECORATOR_CPP

#include <BeamAdapter/component/model/RodSectionColorDecorator.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace beamadapter
{

template class SOFA_BEAMADAPTER_API RodSectionColorDecorator<sofa::defaulttype::Rigid3Types>;

void registerRodSectionColorDecorator(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Adding color parameter to a RodSection.")
                             .add< RodSectionColorDecorator<sofa::defaulttype::Rigid3Types> >());
}

} // namespace beamadapter
