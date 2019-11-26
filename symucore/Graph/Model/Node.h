/* ----------------------------------------------------------------------
* OPEN-SYMUVIA (http://open-symuvia.ifsttar.fr). This file is part of OPEN-SYMUVIA.
*
* OPEN-SYMUVIA is an open-source traffic simulator.
*
* Copyright (C) - IFSTTAR, ENTPE - Ludovic Leclercq, Cécile Bécarie
*
* Open-SymuVia is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
* 
* OPEN-SYMUVIA is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lessed General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with this program; if not, write to the Free Software Foundation,
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA or 
* see <http://ww.gnu.org/licenses/>
*
* For more information, please consult: <http://open-symuvia.ifsttar.fr> or 
* send an email to cecile.becarie@ifsttar.fr
*
* ----------------------------------------------------------------------*/

﻿#pragma once 

#ifndef SYMUCORE_NODE_H
#define SYMUCORE_NODE_H

#include "SymuCoreExports.h"

#include "PatternsSwitch.h"
#include "Pattern.h"

#include "Utils/SymuCoreConstants.h"

#include <string>
#include <vector>
#include <map>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Point;
class VehicleType;
class Link;
class AbstractPenalty;
class Graph;
class Cost;
class SubPopulation;

struct ComparePattern {
    bool operator()(Pattern * p1, Pattern * p2) const { return p1->getUniqueID() < p2->getUniqueID(); }
};

class SYMUCORE_DLL_DEF Node {

public:

    Node();
    Node(const std::string& strName, NodeType eNodeType, const Point* ptCoordinates);
    virtual ~Node();

    //getters
    Graph * getParent() const;
    std::string getStrName() const;
    NodeType getNodeType() const;
    Point* getCoordinates() const;
    const std::vector<Link*>& getUpstreamLinks() const;
    const std::vector<Link*>& getDownstreamLinks() const;
    const std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern > & getMapPenalties() const;
    std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern > & getMapPenalties();

    //setters
    void setParent(Graph * pGraph);
    void setStrName(const std::string &strName);
    void setNodeType(const NodeType &eNodeType);
    void setCoordinates(Point* ptCoordinate);
    void addUpstreamLink(Link * pUpstreamLink);
    void addDownstreamLink(Link * pDownstreamLink);

    Cost * getPenaltyCost(int iSimulationInstance, Pattern* upstreamPattern, Pattern*downstreamPattern, double t, SymuCore::SubPopulation *pSubPopulation);

private:

    Graph*                                                                      m_pParent;

    std::string                                                                 m_strName; //name of the Node
    NodeType                                                                    m_eNodeType; //type of the Node
    Point*                                                                      m_pCoordinates; //coordinates of the point
    std::vector<Link*>                                                          m_listUpstreamLinks; //list of pointers to the upstream links
    std::vector<Link*>                                                          m_listDownstreamLinks; //list of pointers to the downstream links
    std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern >   m_mapPenalties; //map of the Penalties for upstream/downstream patterns

};
}

#pragma warning(pop)

#endif // SIMUCORE_NODE_H
