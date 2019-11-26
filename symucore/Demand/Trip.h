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

#ifndef _SYMUCORE_TRIP_
#define _SYMUCORE_TRIP_

#include "SymuCoreExports.h"

#include <Demand/Path.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#pragma warning( push )  
#pragma warning( disable : 4251 )  

namespace SymuCore {

class Origin;
class Destination;
class Population;
class VehicleType;
class SubPopulation;

class SYMUCORE_DLL_DEF Trip {

public:

    Trip();
    Trip(int nID, const boost::posix_time::ptime & departureTime, Origin * pOrigin, Destination * pDest, SubPopulation* pSubPopulation, VehicleType * pVehicleType, double iSigmaConvenience = -1);

    virtual ~Trip();

    static bool DepartureTimeSorter(const Trip& lhs, const Trip& rhs);
    static bool DepartureTimePtrSorter(const Trip* lhs, const Trip* rhs);

    int GetID() const;
    const boost::posix_time::ptime & GetDepartureTime() const;
    Origin * GetOrigin() const;
    Destination * GetDestination() const;
    Population* GetPopulation() const;
    SubPopulation *GetSubPopulation() const;
    VehicleType * GetVehicleType() const;
	const Path& GetPath(int iInstance) const;
    Path& GetPath(int iInstance);
    bool isPredefinedPath() const;
    double GetSigmaConvenience(double dbDefaultValue) const;

    void SetOrigin(Origin * pOrigin);
    void SetDestination(Destination * pDestination);
    void SetPath(int iInstance, const SymuCore::Path& path);
    void SetIsPredefinedPath(bool isPredefinedPath);

protected:

    int                         m_nID;
    boost::posix_time::ptime    m_DepartureTime;
    Origin*                     m_pOrigin;
    Destination*                m_pDestination;
    Population*                 m_pPopulation;
    SubPopulation*              m_pSubPopulation;
    VehicleType*                m_pVehicleType;
    std::vector<Path>           m_Path;
    bool                        m_bIsPredefinedPath;
    double                      m_dbSigmaConvenience;
};

}

#pragma warning( pop )  

#endif // _SYMUCORE_TRIP_
