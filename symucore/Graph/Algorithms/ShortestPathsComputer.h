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

#ifndef SYMUCORE_SHORTESTPATHSCOMPUTER_H
#define SYMUCORE_SHORTESTPATHSCOMPUTER_H

#include "SymuCoreExports.h"

#include "Demand/Path.h"

#include <map>
#include <vector>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Destination;
class Origin;
class SubPopulation;
class ValuetedPath;

class SYMUCORE_DLL_DEF ShortestPathsComputer {

public:

    // Structure used to keep information about Nodes during the travel of the graph
    struct TimedNode {

        TimedNode()
        {
        }

        ~TimedNode()
        {
        }

        TimedNode(Node* pNode, TimedNode* pPredecossorNode, Pattern* pPredecossorPattern, double dTotalCost, double dTotalTime)
        {
            m_pNode = pNode;
            m_pPredecossorNode = pPredecossorNode;
            m_pPredecossorPattern = pPredecossorPattern;
            m_dTotalCost = dTotalCost;
            m_dTotalTime = dTotalTime;
        }

        inline bool operator==(TimedNode* otherTimedNode)
        {
            return otherTimedNode->m_pNode == m_pNode;
        }

        Node *							m_pNode; //pointer to the node
        TimedNode *						m_pPredecossorNode; //direct predecessor Node
        Pattern *						m_pPredecossorPattern; //direct predecessor Pattern
        double							m_dTotalCost; //total cost to reach this Node
        double                  		m_dTotalTime; // total time to reach this Node
    };

    ShortestPathsComputer();
	virtual ~ShortestPathsComputer();

    virtual std::map<Origin*, std::map<Destination*, std::vector<ValuetedPath*> > > getShortestPaths(int iSimulationInstance, const std::vector<Origin*>& originNodes, const std::vector<Destination*>& destinationNodes, SubPopulation* pSubPopulation, double dbArrivalTime, bool bUseCostsForTemporalAlgorithm = false) =0;
};
}

#pragma warning(pop)

#endif // SYMUCORE_SHORTESTPATHSCOMPUTER_H
