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

#ifndef SYMUCORE_COST_H
#define SYMUCORE_COST_H

#include "SymuCoreExports.h"

#include "Utils/SymuCoreConstants.h"

#include <map>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class SYMUCORE_DLL_DEF Cost {

public:

    Cost();
    Cost(double dCostValue, double dTravelTime);
    virtual ~Cost();

    //getters
	double getCostValue() const;
	double getTravelTime() const;
    double getOtherCostValue(CostFunction eCostFunction) const;

    double getCostValue(CostFunction eCostFunction) const;

    //setters
    void setUsedCostValue(double dCostValue);
	void setTravelTime(double dTravelTime);
    void setOtherCostValue(CostFunction eCostFunction, double dbValue);


    void plus(Cost *otherCost);

private:

    double m_dbCostValue;
	double m_dbTravelTime; // in seconds
    std::map<CostFunction, double> m_mapOtherCosts;
};
}

#pragma warning(pop)

#endif // SYMUCORE_PATTERNSSWITCH_H
