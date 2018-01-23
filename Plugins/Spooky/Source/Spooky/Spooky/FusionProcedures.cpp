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

    //TODO: try this?
    // Node::getRequiredParentsCts(const Measurement::Ptr& m, const Transform3D& toFusionSpace){
    //         Eigen::Vector<float,6> error = toFusionSpace * m->getPosRot() - getGlobalPose().toPosRot();
            
    //         //Iterate through parents until enough flex is found
    //         std::vector<Node::Ptr> parents;
    //         parents.push_back(std::make_shared<Node>(this));
    //         Eigen::Matrix<float,6> varTotal = Eigen::Vector3f::Zero();
    //         Transform3D pose = Transform3D::Identity();
    //         float flex = 0;
    //         const float flex_threshold = 0.9;
    //         while(true){
    //             //Assume decoupling between nodes
    //             pose = parents.back()->getLocalPose() * pose;
    //             varTotal = parents.back()->getLocalPoseVariance(pose,varTotal);
    //             flex = std::exp(-error.transpose()*varTotal*error)
    //             if(flex<flex_threshold){
    //                 //We dont need anymore nodes to fuse data
    //                 break;
    //             } else {
    //                 //We need more nodes
    //                 parents.push_back(parents.back()->parent);
    //             }
    //         }
    //         return parents;
    // }

    std::vector<Node::Ptr> Node::getRequiredParents(const Measurement::Ptr& m){
        //Iterate through parents until enough flex is found
        std::vector<Node::Ptr> parents;
        parents.push_back(std::make_shared<Node>(this));
        //Positional degrees of freedom
        int p_dof = 0;
        //rotational degrees of freedom
        int r_dof = 0;
        //Required dof:
        int p_dof_req = m->getRequiredPDoF();
        int r_dof_req = m->getRequiredRDoF();

        const float flex_threshold = 0.9;
        bool hasLeverChild = false;
        while(true){
            //Assume decoupling between nodes
            p_dof += parent.back()->getPDoF(hasLeverChild);
            r_dof += parent.back()->getRDoF();
            
            if(p_dof>=p_dof_req && r_dof>=r_dof_req || parents.back()->parent == NULL){
                //We dont need anymore nodes to fuse data, or we are out of nodes
                break;
            } else {
                //We need more nodes
                parents.push_back(parents.back()->parent);
            }
        }
        return parents;
    }

    void Node::fusePositionMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){
        //Calculate error
        if(m->isGlobal){
            std::vector<Node::Ptr> fusion_chain = getRequiredParents(m);
        }
        //TODO: fuse chain
    }

    void Node::fuseRotationMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }

    void Node::fuseRigidMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }

    void Node::fuseScaleMeasurement(const Measurement::Ptr& m, const Transform3D& toFusionSpace){

    }

}