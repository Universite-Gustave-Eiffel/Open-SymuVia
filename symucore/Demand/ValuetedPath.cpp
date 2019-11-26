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

﻿#include "ValuetedPath.h"

#include "Graph/Algorithms/ShortestPathsComputer.h"
#include "Demand/Path.h"
#include "Graph/Model/Link.h"
#include "Graph/Model/Node.h"

using namespace SymuCore;

#include <vector>

ValuetedPath::ValuetedPath()
{
    m_bIsPredefined = false;
    m_pPredefinedJunctionNode = NULL;
    m_dbPenalizedCost = 0;
    m_dbCommonalityFactor = 0;
}

ValuetedPath::ValuetedPath(const Path &pPath, double dTotalCost, double dTotalTime)
{
    m_Path = Path(pPath);
    m_dTotalCost = dTotalCost;
    m_dTotalTime = dTotalTime;
    m_bIsPredefined = false;
    m_pPredefinedJunctionNode = NULL;
    m_dbPenalizedCost = dTotalCost;
    m_dbCommonalityFactor = 0;
}

ValuetedPath::ValuetedPath(ShortestPathsComputer::TimedNode *tnPathNode, bool hasToReverse)
{
    std::vector<Pattern*> listPattern = std::vector<Pattern*>();
    if (tnPathNode != NULL)
    {
        ShortestPathsComputer::TimedNode* currentNode = tnPathNode;
        while (currentNode->m_pPredecossorNode != NULL)
        {
            listPattern.push_back(currentNode->m_pPredecossorPattern);
            currentNode = currentNode->m_pPredecossorNode;
        }
        if(hasToReverse)
            reverse(listPattern.begin(), listPattern.end());
    }

    m_Path = Path(listPattern);
    m_dTotalCost = tnPathNode->m_dTotalCost;
    m_dTotalTime = tnPathNode->m_dTotalTime;
    m_bIsPredefined = false;
    m_pPredefinedJunctionNode = NULL;
    m_dbPenalizedCost = m_dTotalCost;
    m_dbCommonalityFactor = 0;
}

ValuetedPath::~ValuetedPath()
{
}

void ValuetedPath::setTime(double dbTime)
{
    m_dTotalTime = dbTime;
}

void ValuetedPath::setCost(double dbCost)
{
    m_dTotalCost = dbCost;
}

void ValuetedPath::setStrName(const std::string & strName)
{
    m_strName = strName;
}

void ValuetedPath::setIsPredefined(bool bPredefined)
{
    m_bIsPredefined = bPredefined;
}

void ValuetedPath::setPredefinedJunctionNode(Node * pJunction)
{
    m_pPredefinedJunctionNode = pJunction;
}

void ValuetedPath::setPenalizedCost(double dbPenalizedCost)
{
    m_dbPenalizedCost = dbPenalizedCost;
}

void ValuetedPath::setCommonalityFactor(double dbCommonalityFactor)
{
    m_dbCommonalityFactor = dbCommonalityFactor;
}

bool ValuetedPath::operator<(const ValuetedPath &otherPath) const
{
    // rmk. to ensure no identical paths, due to numerical precision limit on path cost,
    // we check the equality of paths first, and compare path costs only for different paths
    if (m_Path == otherPath.m_Path)
    {
        return false;
    }
    else
    {
        if (m_dTotalCost != otherPath.m_dTotalCost)
            return m_dTotalCost < otherPath.m_dTotalCost;

        return m_Path < otherPath.m_Path;
    }
}

bool ValuetedPath::operator==(const ValuetedPath &otherPath) const
{
    return (!this->operator<(otherPath) && !otherPath.operator <(*this));
}

double ValuetedPath::GetCost() const
{
    return m_dTotalCost;
}

double ValuetedPath::GetTime() const
{
    return m_dTotalTime;
}

Path& ValuetedPath::GetPath()
{
    return m_Path;
}

const Path& ValuetedPath::GetPath() const
{
    return m_Path;
}

bool ValuetedPath::IsPredefined() const
{
    return m_bIsPredefined;
}

const std::string & ValuetedPath::getStrName() const
{
    return m_strName;
}

Node * ValuetedPath::getPredefinedJunctionNode() const
{
    return m_pPredefinedJunctionNode;
}

double ValuetedPath::getPenalizedCost() const
{
    return m_dbPenalizedCost;
}

double ValuetedPath::getCommonalityFactor() const
{
    return m_dbCommonalityFactor;
}


