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

ï»¿#pragma once

#ifndef SymuCoreDrivingTripsBuilder_H
#define SymuCoreDrivingTripsBuilder_H

class SymuCoreGraphBuilder;
class SymuCoreDrivingGraphBuilder;
class Reseau;
class SymuViaTripNode;
class TypeVehicule;

namespace SymuCore {
    class Populations;
    class SubPopulation;
    class MultiLayersGraph;
    class Trip;
    class VehicleType;
    class Population;

}

namespace boost {
    namespace posix_time {
        class ptime;
    }
}

#include <vector>

class SymuCoreDrivingTripsBuilder
{
public:

    SymuCoreDrivingTripsBuilder();

	//! Constructeur
    SymuCoreDrivingTripsBuilder(Reseau* pReseau);

	//! Destructeur
    virtual ~SymuCoreDrivingTripsBuilder();

    virtual bool FillPopulations(SymuCore::MultiLayersGraph * pGraph, SymuCore::Populations & populations,
        const boost::posix_time::ptime & startSimulationTime, const boost::posix_time::ptime & endSimulationTime,
        bool bIgnoreSubAreas, std::vector<SymuCore::Trip> & lstTrips);

protected:

    void AddTrip(std::vector<SymuCore::Trip> & symuViaTrips, SymuCore::MultiLayersGraph * pGraph, const boost::posix_time::ptime & simulationStartTime, double dbCreationTime, SymuViaTripNode * pOrigin, SymuViaTripNode * pDestination, TypeVehicule * pTypeVeh, SymuCore::VehicleType * pVehType, SymuCore::SubPopulation * pSubPopulationForTrip,
        bool bIgnoreSubAreas);

protected:

    Reseau*                         m_pNetwork;

};

#endif // SymuCoreDrivingTripsBuilder_H


