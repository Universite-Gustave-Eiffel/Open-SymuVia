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

#ifndef ZoneDrivingPattern_H
#define ZoneDrivingPattern_H

#include "SymuViaDrivingPattern.h"

#include <Utils/SpatialMarginalsHelper.h>

class Tuyau;
class ZoneDeTerminaison;
class Connexion;
class TypeVehicule;

class ZoneDrivingPattern : public SymuViaDrivingPattern
{
public:

    ZoneDrivingPattern();
    ZoneDrivingPattern(SymuCore::Link* pLink, ZoneDeTerminaison * pZone, Connexion * pJunction, bool bIsOrigin);

    virtual ~ZoneDrivingPattern();

    virtual std::string toString() const;

    virtual void fillMeasuredCostsForTravelTimesUpdatePeriod(int iTravelTimesUpdatePeriodIndex, const std::vector<SymuCore::SubPopulation *> &listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions);

    virtual void postProcessCosts(const std::vector<SymuCore::SubPopulation*>& listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions);

    virtual void fillCost(const std::map<TypeVehicule*, SymuCore::SubPopulation *> &listSubPopulations);

public:
    ZoneDeTerminaison * getZone();

private:
    ZoneDeTerminaison * m_pZone;
    Connexion * m_pJunction;

    // true si le pattern est un pattern de dÃ©part de zone, false s'il s'agit d'un pattern d'arrivÃ©e en zone
    bool      m_bIsOrigin;

    SymuCore::SpatialMarginalsHelper m_marginalsHelper;
};

#endif // ZoneDrivingPattern_H


