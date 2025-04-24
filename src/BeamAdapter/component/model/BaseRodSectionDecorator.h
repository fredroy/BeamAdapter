#pragma once

#include <BeamAdapter/config.h>

#include <BeamAdapter/component/model/BaseRodSectionMaterial.h>
#include <sofa/core/objectmodel/BaseObject.h>


namespace beamadapter
{

template <typename DataTypes>
class BaseRodSectionDecorator : public BaseRodSectionMaterial<DataTypes>
{
public:
    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE(BaseRodSectionDecorator, DataTypes), SOFA_TEMPLATE(BaseRodSectionMaterial, DataTypes));
    
    using Inherit = BaseRodSectionMaterial<DataTypes>;
    using Real = typename Inherit::Real;
    using Transform = typename Inherit::Transform;
    using Size = typename Inherit::Size;
    
    BaseRodSectionDecorator();
    virtual ~BaseRodSectionDecorator() = default;
    
    void init() override;
    [[nodiscard]] Size getNbVisualEdges() const override { return l_decoratedSection->getNbVisualEdges(); }
    [[nodiscard]] Size getNbCollisionEdges() const override { return l_decoratedSection->getNbCollisionEdges(); }
    [[nodiscard]] Real getLength() const override { return l_decoratedSection->getLength(); }
    [[nodiscard]] const BeamSection& getBeamSection() const override { return l_decoratedSection->getBeamSection(); }
    void getInterpolationParameters(Real& _A, Real& _Iy, Real& _Iz, Real& _Asy, Real& _Asz, Real& _J) const override
    {
        return l_decoratedSection->getInterpolationParameters(_A, _Iy, _Iz, _Asy, _Asz, _J);
    }
    void getMechanicalParameters(Real& youngModulus, Real& cPoisson, Real& massDensity) const override
    {
        return l_decoratedSection->getMechanicalParameters(youngModulus, cPoisson, massDensity);
    }
    void getRestTransformOnX(Transform& global_H_local, const Real x_used, const Real x_start) override
    {
        return l_decoratedSection->getRestTransformOnX(global_H_local, x_used, x_start);
    }
        
public:
    SingleLink<BaseRodSectionDecorator<DataTypes>, BaseRodSectionMaterial<DataTypes>, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_decoratedSection;
    
};

} // namespace beamadapter

