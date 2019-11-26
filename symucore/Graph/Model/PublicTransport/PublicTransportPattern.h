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

#pragma once

#ifndef SYMUCORE_PUBLICTRANSPORTPATTERN_H
#define SYMUCORE_PUBLICTRANSPORTPATTERN_H

#include "SymuCoreExports.h"

#include "Graph/Model/Pattern.h"

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class PublicTransportLine;
class SYMUCORE_DLL_DEF PublicTransportPattern : public Pattern {

public:

    PublicTransportPattern();
    PublicTransportPattern(Link* pLink, PatternType ePatternType, PublicTransportLine* pLine);
    virtual ~PublicTransportPattern();

    //getters
    PublicTransportLine* getLine() const;
    const std::vector<ListTimeFrame<Cost> > & getTemporalCosts() const;
    std::vector<ListTimeFrame<Cost> > & getTemporalCosts();


    //setters
    void setLine(PublicTransportLine* pLine);

    virtual void addTimeFrame(double tBeginTime, double tEndTime);
    virtual void prepareTimeFrames(double startPeriodTime, double endPeriodTime, double travelTimesUpdatePeriod, const std::vector<SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions, int nbSimulationInstances, int iInstance);
    virtual Cost* getPatternCost(int iSimulationInstance, double t, SubPopulation* pSubPopulation);
    virtual std::string toString() const;

    virtual void fillFromSecondaryInstance(Pattern * pFrom, int iInstance);

protected:

    PublicTransportLine*                m_pLine; //the public transport line use for this pattern
    std::vector<ListTimeFrame<Cost> >   m_TemporalCosts; //cost to use this pattern for each simulation instance, depends on time

};
}

#pragma warning(pop)

#endif // SYMUCORE_PUBLICTRANSPORTPATTERN_H
