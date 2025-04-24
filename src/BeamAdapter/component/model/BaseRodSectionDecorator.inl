#pragma once

#include <BeamAdapter/component/model/BaseRodSectionDecorator.h>

namespace beamadapter
{

template <typename DataTypes>
BaseRodSectionDecorator<DataTypes>::BaseRodSectionDecorator()
    : Inherit()
    , l_decoratedSection(initLink("section", "Link to the Section to decorate"))
{
}


template <typename DataTypes>
void BaseRodSectionDecorator<DataTypes>::init()
{
    if(!l_decoratedSection)
    {
        msg_error() << "decoratedSection is not set.";
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }
    
    this->d_length.setParent(&l_decoratedSection->d_length);
    this->d_radius.setParent(&l_decoratedSection->d_radius);
    this->d_innerRadius.setParent(&l_decoratedSection->d_innerRadius);
    this->d_massDensity.setParent(&l_decoratedSection->d_massDensity);
    this->d_poissonRatio.setParent(&l_decoratedSection->d_poissonRatio);
    this->d_youngModulus.setParent(&l_decoratedSection->d_youngModulus);
    this->d_nbEdgesVisu.setParent(&l_decoratedSection->d_nbEdgesVisu);
    this->d_nbEdgesCollis.setParent(&l_decoratedSection->d_nbEdgesCollis);
    
    l_decoratedSection->init();
    
}

template <typename DataTypes>
auto BaseRodSectionDecorator<DataTypes>::getNbVisualEdges() const -> Size
{
    if(this->isComponentStateValid())
    {
        return l_decoratedSection->getNbVisualEdges();
    }
    else
    {
        return 0;
    }
}
template <typename DataTypes>
auto BaseRodSectionDecorator<DataTypes>::getNbCollisionEdges() const -> Size
{
    if(this->isComponentStateValid())
    {
        return l_decoratedSection->getNbCollisionEdges();
    }
    else
    {
        return 0;
    }
}

template <typename DataTypes>
auto BaseRodSectionDecorator<DataTypes>::getLength() const -> Real
{
    if(this->isComponentStateValid())
    {
        return l_decoratedSection->getLength();
    }
    else
    {
        return 0;
    }
}

template <typename DataTypes>
auto BaseRodSectionDecorator<DataTypes>::getBeamSection() const -> const BeamSection&
{
    if(this->isComponentStateValid())
    {
        return l_decoratedSection->getBeamSection();
    }
    else
    {
        static BeamSection bs {};
        return bs;
    }
}

template <typename DataTypes>
void BaseRodSectionDecorator<DataTypes>::getInterpolationParameters(Real& _A, Real& _Iy, Real& _Iz, Real& _Asy, Real& _Asz, Real& _J) const
{
    if(this->isComponentStateValid())
    {
        return l_decoratedSection->getInterpolationParameters(_A, _Iy, _Iz, _Asy, _Asz, _J);
    }
    else
    {
        return;
    }
}

template <typename DataTypes>
void BaseRodSectionDecorator<DataTypes>::getMechanicalParameters(Real& youngModulus, Real& cPoisson, Real& massDensity) const
{
    if(this->isComponentStateValid())
    {
        return l_decoratedSection->getMechanicalParameters(youngModulus, cPoisson, massDensity);
    }
    else
    {
        return;
    }
    
}
template <typename DataTypes>
void BaseRodSectionDecorator<DataTypes>::getRestTransformOnX(Transform& global_H_local, const Real x_used, const Real x_start)
{
    if(this->isComponentStateValid())
    {
        return l_decoratedSection->getRestTransformOnX(global_H_local, x_used, x_start);
    }
    else
    {
        return;
    }
}

} // namespace beamadapter
