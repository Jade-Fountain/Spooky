/*  This file is part of Spooky, a sensor fusion plugin for VR in the Unreal Engine

    Copyright 2017 Jake Fountain

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "Spooky.h"
#include "ArticulatedModel.h"
#include "sophus/so3.hpp"

namespace spooky{

    Vector3f ArticulatedModel::getPositionFlex(){

        for(int i = 0; i < articulations.size(); i++){
            variance = state->articulation[i].variance;
        }
    }

    void ArticulatedModel::fusePositionMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){
        //Calculate error
        Eigen::Vector3f error = toFusionSpace * m->getPosition() - getGlobalPose().translation();
        
        //Iterate through parents until enough flex is found
        Node::Ptr n = std::make_shared<Node>(this);
        Eigen::Vector3f flex = Eigen::Vector3f::Zero();
        while(flex < error.norm()){
            flex += n->getPositionFlex();
        }
    }

    void ArticulatedModel::fuseRotationMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }

    void ArticulatedModel::fuseRigidMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }

    void ArticulatedModel::fuseScaleMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }

}