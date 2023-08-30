/******************************************************************************
*                              BeamAdapter plugin                             *
*                  (c) 2006 Inria, University of Lille, CNRS                  *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: see Authors.md                                                     *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <BeamAdapter/component/AdaptiveBeamContactMapper.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/DeleteVisitor.h>
#include <iostream>

namespace sofa::component::collision::response::mapper
{


template < class TCollisionModel, class DataTypes >
void AdaptiveBeamContactMapper<TCollisionModel,DataTypes>::cleanup()
{
    if (mapping!=NULL)
    {
        simulation::Node* parent = dynamic_cast<simulation::Node*>(model->getContext());
        if (parent!=NULL)
        {
            simulation::Node::SPtr child = dynamic_cast<simulation::Node*>(mapping->getContext());
            child->detachFromGraph();
            child->execute<simulation::DeleteVisitor>(sofa::core::ExecParams::defaultInstance());
            child.reset();
            mapping.reset();
        }
    }



}

template < class TCollisionModel, class DataTypes >
typename AdaptiveBeamContactMapper<TCollisionModel,DataTypes>::MMechanicalState* AdaptiveBeamContactMapper<TCollisionModel,DataTypes>::createMapping(const char* name)
{
    if (model==NULL) return NULL;
    InMechanicalState* instate = model->getMechanicalState();
    if (instate!=NULL)
    {

        auto* instateContext= instate->getContext();
        simulation::Node* parent = dynamic_cast<simulation::Node*>(instateContext);
        sofa::component::fem::BeamInterpolation<InDataTypes>* _interpolation;
		instate->getContext()->get(_interpolation);
        if (parent==NULL )
        {
            std::cerr << "ERROR: AdaptiveBeamContactMapper only works for scenegraph scenes.\n";
            return NULL;
        }
        if (_interpolation==NULL)
        {
            std::cerr << "ERROR: AdaptiveBeamContactMapper only works if having BeamInterpolation .\n";
            return NULL;
        }
        simulation::Node::SPtr childPtr= parent->createChild(name);
        child = childPtr.get();
        parent->addChild(child); child->updateSimulationContext();
        typename MMechanicalObject::SPtr outmodel = sofa::core::objectmodel::New<MMechanicalObject>();
        child->addObject(outmodel);
        //outmodel->useMask.setValue(true);

        mapping =  sofa::core::objectmodel::New<MMapping>(instate, outmodel.get(),_interpolation);
        child->addObject(mapping);
    }
    else
    {
        simulation::Node* parent = dynamic_cast<simulation::Node*>(model->getContext());
        if (parent==NULL)
        {
            std::cerr << "ERROR: AdaptiveBeamContactMapper only works for scenegraph scenes.\n";
            return NULL;
        }
        simulation::Node::SPtr childPtr=  parent->createChild(name);
        child =childPtr.get();
        parent->addChild(child); child->updateSimulationContext();
        typename MMechanicalObject::SPtr outmodel = sofa::core::objectmodel::New<MMechanicalObject>();

        child->addObject(outmodel);
        //outmodel->useMask.setValue(true);
        mapping = NULL;
    }
    return outmodel;
}


} // namespace sofa::component::collision::response::mapper
