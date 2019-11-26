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

#ifndef SYMUCORE_DIJKSTRA_H
#define SYMUCORE_DIJKSTRA_H

#include "SymuCoreExports.h"
#include "ShortestPathsComputer.h"
#include "Utils/SymuCoreConstants.h"


#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

    class SubPopulation;
    class Path;
    class Origin;
    class Destination;
    class ValuetedPath;

class SYMUCORE_DLL_DEF Dijkstra : public ShortestPathsComputer {

public:

    Dijkstra();
    Dijkstra(bool bFromOriginToDestination);
    virtual ~Dijkstra();

    virtual std::map<Origin*, std::map<Destination*, std::vector<ValuetedPath*> > >  getShortestPaths(int iSimulationInstance, const std::vector<Origin*>& originNodes, const std::vector<Destination*>& destinationNodes, SubPopulation* pSubPopulation, double dbStartingPointTime, bool bUseCostsForTemporalAlgorithm = false);

    std::map<Pattern *, double> &GetMapPenalizedPatterns();

    void SetHeuristic(ShortestPathHeuristic eHeuristic, double dbAStarBeta, double dbHeuristicGamma);

private:

    template<class T>
    std::vector<std::vector<ValuetedPath*> > getShortestPaths(int iSimulationInstance, SubPopulation* pSubPopulation, double dbStartingPointTime, const T & graphTraverser, bool bUseCostsForTemporalAlgorithm);

    Cost GetTotalPatternCost(int iSimulationInstance, Pattern *pPattern, double t, SubPopulation *pSubPopulation);

    std::map<Pattern*, double>  m_mapPenalizedPatterns;

    ShortestPathHeuristic       m_eHeuristic;
    double                      m_dbAStarBeta;
    double                      m_dbHeuristicGamma;

    bool                        m_bFromOriginToDestination;
};
}

#pragma warning(pop)

#endif // SYMUCORE_DIJKSTRA_H
