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
    SOFA_CLASS(SOFA_TEMPLATE(BaseRodSectionDecorator, DataTypes), SOFA_TEMPLATE(BaseRodSectionMaterial, DataTypes));
    
    using Inherit = BaseRodSectionMaterial<DataTypes>;
    using Real = typename Inherit::Real;
    using Transform = typename Inherit::Transform;
    
    BaseRodSectionDecorator();
    virtual ~BaseRodSectionDecorator() = default;
    
    void init() override;
    [[nodiscard]] int getNbVisualEdges() const { return l_decoratedSection->getNbVisualEdges(); }
    [[nodiscard]] int getNbCollisionEdges() const { return l_decoratedSection->getNbCollisionEdges(); }
    [[nodiscard]] Real getLength() const { return l_decoratedSection->getLength(); }


    /// Returns the BeamSection @sa m_beamSection corresponding to this section
    [[nodiscard]] const BeamSection& getBeamSection() const { return l_decoratedSection->getBeamSection(); }

    /// Returns the mass density and the BeamSection of this section
    void getInterpolationParameters(Real& _A, Real& _Iy, Real& _Iz, Real& _Asy, Real& _Asz, Real& _J) const
    {
        return l_decoratedSection->getInterpolationParameters(_A, _Iy, _Iz, _Asy, _Asz, _J);
    }

    /// Returns the Young modulus, Poisson's and massDensity coefficient of this section
    void getMechanicalParameters(Real& youngModulus, Real& cPoisson, Real& massDensity)
    {
        return l_decoratedSection->getMechanicalParameters(youngModulus, cPoisson, massDensity);
    }

    /// This function is called to get the rest position of the beam depending on the current curved abscisse given in parameter
    virtual void getRestTransformOnX(Transform& global_H_local, const Real x_used, const Real x_start) override
    {
        return l_decoratedSection->getRestTransformOnX(global_H_local, x_used, x_start);
    }
    
public:
    SingleLink<BaseRodSectionDecorator<DataTypes>, BaseRodSectionMaterial<DataTypes>, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_decoratedSection;
    
};

} // namespace beamadapter

