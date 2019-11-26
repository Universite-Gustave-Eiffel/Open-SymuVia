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

﻿#include "Node.h"

#include "AbstractPenalty.h"
#include "Utils/Point.h"
#include "Graph.h"

using namespace SymuCore;

Node::Node() : m_pParent(NULL), m_strName(), m_eNodeType(NT_Undefined), m_pCoordinates(NULL)
{

}

Node::Node(const std::string &strName, NodeType eNodeType, const Point *ptCoordinates)
: m_pParent(NULL)
{
    m_strName = strName;
    m_eNodeType = eNodeType;
    if (ptCoordinates != NULL)
    {
        m_pCoordinates = new Point(*ptCoordinates);
    }
    else
    {
        m_pCoordinates = NULL;
    }
}

Node::~Node()
{
    for (std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iter = m_mapPenalties.begin(); iter != m_mapPenalties.end(); ++iter)
    {
        for (std::map<Pattern*, AbstractPenalty*, ComparePattern>::iterator iterDown = iter->second.begin(); iterDown != iter->second.end(); ++iterDown)
        {
            delete iterDown->second;
        }
    }

    if (m_pCoordinates)
    {
        delete m_pCoordinates;
    }
}

Graph * Node::getParent() const
{
    return m_pParent;
}

void Node::setStrName(const std::string &strName)
{
    m_strName = strName;
}

std::string Node::getStrName() const
{
    return m_strName;
}

void Node::setNodeType(const NodeType &eNodeType)
{
    m_eNodeType = eNodeType;
}

NodeType Node::getNodeType() const
{
    return m_eNodeType;

}

void Node::setCoordinates(Point* ptCoordinate)
{
    if (m_pCoordinates)
    {
        delete m_pCoordinates;
        m_pCoordinates = NULL;
    }
    if (ptCoordinate)
    {
        m_pCoordinates = new Point(*ptCoordinate);
    }
}

void Node::setParent(Graph * pGraph)
{
    m_pParent = pGraph;
}

Point* Node::getCoordinates() const
{
    return m_pCoordinates;
}

const std::vector<Link*>& Node::getUpstreamLinks() const
{
    return m_listUpstreamLinks;
}

const std::vector<Link*>& Node::getDownstreamLinks() const
{
    return m_listDownstreamLinks;
}

const std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern > & Node::getMapPenalties() const
{
    return m_mapPenalties;
}

std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern > & Node::getMapPenalties()
{
    return m_mapPenalties;
}

void Node::addUpstreamLink(Link * pUpstreamLink)
{
    m_listUpstreamLinks.push_back(pUpstreamLink);
}

void Node::addDownstreamLink(Link * pDownstreamLink)
{
    m_listDownstreamLinks.push_back(pDownstreamLink);
}

Cost * Node::getPenaltyCost(int iSimulationInstance, Pattern* upstreamPattern, Pattern* downstreamPattern, double t, SubPopulation* pSubPopulation)
{
    std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iter = m_mapPenalties.find(upstreamPattern);
    if (iter != m_mapPenalties.end())
    {
        std::map<Pattern*, AbstractPenalty*, ComparePattern>::iterator iterDown = iter->second.find(downstreamPattern);
        if (iterDown != iter->second.end())
        {
            return iterDown->second->getPenaltyCost(iSimulationInstance, t, pSubPopulation);
        }
    }
    
    return NULL;
}





