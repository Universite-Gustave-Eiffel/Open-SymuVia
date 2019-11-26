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

#ifndef SYMUCORE_POPULATIONS_H
#define SYMUCORE_POPULATIONS_H

#include "SymuCoreExports.h"

#include <vector>
#include <string>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class Population;
class SubPopulation;
class VehicleType;
class MacroType;
class MultiLayersGraph;

class SYMUCORE_DLL_DEF Populations {

public:

    Populations();
    virtual ~Populations();

    void Clear();
    void DeepCopy(const Populations & source, MultiLayersGraph * pGraph);

    const std::vector<Population*> & getListPopulations() const;
    std::vector<Population*> & getListPopulations();

    void getPopulationAndVehicleType(Population ** pPopulation, SubPopulation ** pSubPopulation, VehicleType ** pVehType, std::string strVehicleTypeName = std::string(), std::string strMotive = std::string()) const;
    Population * getPopulation(const std::string & strPopulationName) const;

    double getMaxInitialWalkRadius() const;
    double getMaxIntermediateWalkRadius() const;

private:
    std::vector<Population*> m_listPopulations;
};
}

#pragma warning(pop)

#endif // SYMUCORE_POPULATIONS_H
