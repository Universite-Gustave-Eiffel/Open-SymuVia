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

#ifndef SYMUVIAPUBLICTRANSPORTPATTERN_H
#define SYMUVIAPUBLICTRANSPORTPATTERN_H

#include <Graph/Model/PublicTransport/PublicTransportPattern.h>

class Reseau;
class Tuyau;
class Arret;
class Trip;
class TypeVehicule;
class PublicTransportLine;

class SymuViaPublicTransportPattern : public SymuCore::PublicTransportPattern
{
public:

    SymuViaPublicTransportPattern(SymuCore::Link* pLink, SymuCore::PublicTransportLine* pLine, Reseau * pNetwork, Trip * pSymuViaLine,
                                  const std::vector<Tuyau*> &LstTuyau, Arret * pUpstreamStop, Arret * pDownstreamStop);

    virtual ~SymuViaPublicTransportPattern();

    PublicTransportLine * GetSymuViaLine();
    const std::vector<Tuyau*> & GetLstTuyau() const;
    Arret* GetUpstreamStop() const;
    Arret* GetDownstreamStop() const;

    virtual void fillMeasuredCostsForTravelTimesUpdatePeriod(int iTravelTimesUpdatePeriodIndex, const std::vector<SymuCore::SubPopulation *> &listSubPopulations, const std::map<SymuCore::SubPopulation*, SymuCore::CostFunction> & mapCostFunctions);

protected:

    Reseau *                    m_pNetwork;
    PublicTransportLine*        m_pSymuViaBusLine;
    std::vector<Tuyau*>         m_LstTuyau; //liste des tuyaux composant le pattern
    Arret*                      m_pUpstreamStop; //arrêt amont
    Arret*                      m_pDownstreamStop; //arrêt aval

    TypeVehicule*               m_pTypeVehicleForLine; // TYpe de véhicule associé à la ligne de transport public
};

#endif // SYMUVIAPUBLICTRANSPORTPATTERN_H


