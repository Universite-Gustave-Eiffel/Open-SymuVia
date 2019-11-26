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

#ifndef SYMUCORE_MACROTYPE_H
#define SYMUCORE_MACROTYPE_H

#include "SymuCoreExports.h"

#include "Utils/SymuCoreConstants.h"

#include <vector>
#include <string>

#pragma warning(push)
#pragma warning(disable : 4251)

namespace SymuCore {

class VehicleType;

class SYMUCORE_DLL_DEF MacroType {

public:

    MacroType();
    virtual ~MacroType();

    void Clear();

    void DeepCopy(const MacroType & source);

    //getters
    const std::vector<VehicleType*>& getListVehicleTypes() const;
    virtual VehicleType * getVehicleType(const std::string &strName);

    //adders
    void addVehicleType(VehicleType* vehType);

    bool operator ==(MacroType& macro);

    // to avoid creation of the vehicle type when using getVehicleType and a DefaultMacroType.
    bool hasVehicleType(const std::string &strName) const;

protected:

    std::vector<VehicleType*>       m_listVehicleTypes; //the list of all vehicle that compose this macro

};
}

#pragma warning(pop)

#endif // SYMUCORE_MACROTYPE_H
