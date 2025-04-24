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
    l_decoratedSection->init();
    
}

} // namespace beamadapter
