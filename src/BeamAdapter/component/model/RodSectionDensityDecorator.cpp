#define SOFA_PLUGIN_BEAMADAPTER_RODSECTIONDENSITYDECORATOR_CPP

#include <BeamAdapter/component/model/RodSectionDensityDecorator.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace beamadapter
{

template class SOFA_BEAMADAPTER_API RodSectionDensityDecorator<sofa::defaulttype::Rigid3Types>;

void registerRodSectionDensityDecorator(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Adding density parameter to a RodSection.")
                             .add< RodSectionDensityDecorator<sofa::defaulttype::Rigid3Types> >());
}

} // namespace beamadapter
