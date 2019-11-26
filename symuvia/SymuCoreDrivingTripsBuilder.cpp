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

ï»¿#include "stdafx.h"
#include "SymuCoreDrivingTripsBuilder.h"

#include "SymuCoreDrivingGraphBuilder.h"
#include "SymuCoreGraphBuilder.h"
#include "reseau.h"
#include "RandManager.h"
#include "ZoneDeTerminaison.h"
#include "Plaque.h"
#include "usage/SymuViaTripNode.h"


#include <Graph/Model/MultiLayersGraph.h>

#include <Demand/Trip.h>
#include <Demand/Population.h>
#include <Demand/MacroType.h>
#include <Demand/Populations.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <boost/log/trivial.hpp>

#include <boost/math/special_functions/round.hpp>

#include <limits>


using namespace SymuCore;

SymuCoreDrivingTripsBuilder::SymuCoreDrivingTripsBuilder()
: m_pNetwork(NULL)
{
}

SymuCoreDrivingTripsBuilder::SymuCoreDrivingTripsBuilder(Reseau * pNetwork)
: m_pNetwork(pNetwork)
{
}


SymuCoreDrivingTripsBuilder::~SymuCoreDrivingTripsBuilder()
{
}

void SymuCoreDrivingTripsBuilder::AddTrip(std::vector<SymuCore::Trip> & symuViaTrips, MultiLayersGraph * pGraph, const boost::posix_time::ptime & simulationStartTime, double dbCreationTime, SymuViaTripNode * pOrigin, SymuViaTripNode * pDestination, TypeVehicule * pTypeVeh, VehicleType * pVehType, SubPopulation * pSubPopulationForTrip,
    bool bIgnoreSubAreas)
{
    double dbPlaceHolder;
    bool bPlaceHolder;
    Tuyau * pPlaceHolder, * pPlaceHolder2;

    // Tirage du motif pour l'utilisateur :
    CMotifCoeff * pMotif = pOrigin->GetMotif(dbCreationTime, pTypeVeh, pDestination);

    // Cas particulier des zones : on tire une plaque si possible
    ZoneDeTerminaison * pZoneOrigin = dynamic_cast<ZoneDeTerminaison*>(pOrigin);
    SymuCore::Origin * pSymuCoreOrigin = NULL;
    // on Ã©vite l'Ã©tape trÃ¨s couteuse du tirage de la plaque si de toute faÃ§on on ne s'en sert pas aprÃ¨s
    if (pZoneOrigin && !bIgnoreSubAreas)
    {
        CPlaque * pPlaqueOrigin;
        Parking * pParkingOrigin;
        pZoneOrigin->TiragePlaqueOrigine(dbCreationTime, NULL, pTypeVeh, pMotif, pDestination, NULL, NULL, false, dbPlaceHolder, pPlaqueOrigin, bPlaceHolder, pParkingOrigin, bPlaceHolder);
        if (pPlaqueOrigin)
        {
            pSymuCoreOrigin = pGraph->GetOrCreateOrigin(pPlaqueOrigin->GetID());
        }
    }
    if (pSymuCoreOrigin == NULL)
    {
        pSymuCoreOrigin = pGraph->GetOrCreateOrigin(pOrigin->GetID());
    }

    ZoneDeTerminaison * pZoneDestination = dynamic_cast<ZoneDeTerminaison*>(pDestination);
    SymuCore::Destination * pSymuCoreDestination = NULL;
    // on Ã©vite l'Ã©tape trÃ¨s couteuse du tirage de la plaque si de toute faÃ§on on ne s'en sert pas aprÃ¨s
    if (pZoneDestination && !bIgnoreSubAreas)
    {
        CPlaque * pPlaqueDestination;
        pZoneDestination->TiragePlaqueDestination(dbCreationTime, pMotif, NULL, pOrigin, pTypeVeh, pPlaqueDestination, pPlaceHolder, pPlaceHolder2);
        if (pPlaqueDestination)
        {
            pSymuCoreDestination = pGraph->GetOrCreateDestination(pPlaqueDestination->GetID());
        }
    }
    if (pSymuCoreDestination == NULL)
    {
        pSymuCoreDestination = pGraph->GetOrCreateDestination(pDestination->GetID());
    }

    SymuCore::Trip newTrip(
        -1,
        simulationStartTime + boost::posix_time::microseconds((int64_t)boost::math::round(dbCreationTime * 1000000)),    // departure time
        pSymuCoreOrigin,                                                                                    // origin
        pSymuCoreDestination,                                                                               // destination
        pSubPopulationForTrip,                                                                              // population
        pVehType);                                                                                          // vehicle type

    symuViaTrips.push_back(newTrip);
}

bool SymuCoreDrivingTripsBuilder::FillPopulations(MultiLayersGraph * pGraph, Populations & populations,
    const boost::posix_time::ptime & startSymuMasterSimulationTime, const boost::posix_time::ptime & endSymuMasterSimulationTime,
    bool bIgnoreSubAreas, std::vector<SymuCore::Trip> & lstTrips)
{
    // on fait en sorte que cette opÃ©ration soit transparente d'un point de vue de la gÃ©nÃ©ration des nombre alÃ©atoires
    // afin qu'une simulation SymuMaster qui rÃ©utilise un fichier CSV dÃ©jÃ  Ã©crit produise les mÃªmes rÃ©sultats :
    unsigned int nRandCountSvg = m_pNetwork->GetRandManager()->getCount();

    boost::posix_time::ptime simulationStartTime = m_pNetwork->GetSimulationStartTime();

    double dbStartSymuMasterInstant = (double)((startSymuMasterSimulationTime - simulationStartTime).total_microseconds()) / 1000000.0;
    double dbEndSymuMasterInstant = (double)((endSymuMasterSimulationTime - simulationStartTime).total_microseconds()) / 1000000.0;
    double dbEndSimulationInstant = std::min<double>(m_pNetwork->GetDureeSimu(), dbEndSymuMasterInstant);

    // StratÃ©gie : on traite indÃ©pendamment chaque origine, puis on merge les Trips en les ordonnant par instant de dÃ©part.
    for (size_t iOrigin = 0; iOrigin < m_pNetwork->Liste_origines.size(); iOrigin++)
    {
        SymuViaTripNode * pOrigin = m_pNetwork->Liste_origines[iOrigin];

        if (pOrigin->IsTypeDemande())
        {
            // On commence par dÃ©terminer les instants de crÃ©ation :
            std::vector<double> tripInstants;

            double dbCurrentTime = 0.0;
            double dbCriterion = 0;
            double dbNextCreationTime;
            double dbDemandValue = DBL_MAX;
            double dbEndDemandVariationTime = DBL_MAX;
            double dbEndTime = DBL_MAX;
            bool bEnd = false;
            while (!bEnd)
            {
                // rmk : +1 microsec to take care of overlapping instants between two demand variations
                dbDemandValue = pOrigin->GetDemandeValue(dbCurrentTime + 0.000001, dbEndDemandVariationTime);

                // Calcul de la durÃ©e avec cette valeur de demande pour atteindre dbCriterion == 1 :
                if (dbDemandValue > 0)
                {
                    if (!pOrigin->IsTypeDistribution())
                    {
                        dbNextCreationTime = dbCurrentTime + (1.0 - dbCriterion) / dbDemandValue;
                    }
                    else
                    {
                        dbNextCreationTime = dbCurrentTime + (1.0 - dbCriterion) * m_pNetwork->GetRandManager()->myExponentialRand(dbDemandValue);
                    }
                }
                else
                {
                    dbNextCreationTime = DBL_MAX;
                }

                dbEndTime = std::min<double>(dbEndDemandVariationTime, dbEndSimulationInstant);

                if (dbNextCreationTime <= dbEndTime)
                {
                    // crÃ©ation d'un vÃ©hicule (sauf si avant le dÃ©but de la simulation SYmuMaster
                    if (dbNextCreationTime >= dbStartSymuMasterInstant)
                    {
                        tripInstants.push_back(dbNextCreationTime);
                    }
                    dbCriterion = 0;
                    dbCurrentTime = dbNextCreationTime;
                }
                else
                {
                    dbCriterion += (dbEndTime - dbCurrentTime) * dbDemandValue;
                    dbCurrentTime = dbEndTime;
                }

                if (dbCurrentTime >= dbEndSimulationInstant)
                {
                    bEnd = true;
                }
            }

            // Pour chaque instant de crÃ©ation, dÃ©finition des autres Ã©lÃ©ments du Trip (type de vÃ©hicule, origine, destination,...) :
            for (size_t iTripInstant = 0; iTripInstant < tripInstants.size(); iTripInstant++)
            {
                double dbCreationTime = tripInstants[iTripInstant];

                TypeVehicule * pTypeVeh = pOrigin->CalculTypeNewVehicule(dbCreationTime, pOrigin->CalculNumVoie(dbCreationTime));

                Population * pPopulationForTrip;
                SubPopulation * pSubPopulationForTrip;
                VehicleType * pVehType;
                populations.getPopulationAndVehicleType(&pPopulationForTrip, &pSubPopulationForTrip, &pVehType, pTypeVeh->GetLabel());

                if (pPopulationForTrip == NULL)
                {
                    BOOST_LOG_TRIVIAL(error) << "No population for vehicle type " << pTypeVeh->GetLabel();
                    return false;
                }

                if (!pSubPopulationForTrip)
                {
                    if (!pSubPopulationForTrip && pPopulationForTrip->GetListSubPopulations().size() != 1)
                    {
                        BOOST_LOG_TRIVIAL(error) << "Too much possibilities to select SubPopulation for Population " << pPopulationForTrip->GetStrName();
                        return false;
                    }
                    else
                    {
                        pSubPopulationForTrip = pPopulationForTrip->GetListSubPopulations()[0];
                    }
                }
                

                // voie 0 en dur mais pas utilisÃ©e par la fonction GetDestination donc OK
                SymuViaTripNode * pDestination = pOrigin->GetDestination(dbCreationTime, 0, pTypeVeh);

                AddTrip(lstTrips, pGraph, simulationStartTime, dbCreationTime, pOrigin, pDestination, pTypeVeh, pVehType, pSubPopulationForTrip, bIgnoreSubAreas);
            }
        }
        else
        {
            // Cas d'une liste de vÃ©hicules (facile !)
            const std::deque<CreationVehicule> & lstCreations = pOrigin->GetLstCreationsVehicule();
            for (size_t iCreation = 0; iCreation < lstCreations.size(); iCreation++)
            {
                const CreationVehicule & creation = lstCreations[iCreation];

                if (creation.dbInstantCreation < dbStartSymuMasterInstant)
                {
                    continue;
                }

                // creation is time ordered. we stop the loop as soon as the endPeriodTime is reached
                if (creation.dbInstantCreation >= dbEndSimulationInstant)
                {
                    break;
                }

                Population * pPopulationForTrip;
                SubPopulation * pSubPopulationForTrip;
                VehicleType * pVehType;
                populations.getPopulationAndVehicleType(&pPopulationForTrip, &pSubPopulationForTrip, &pVehType, creation.pTypeVehicule->GetLabel());
                if (pPopulationForTrip == NULL)
                {
                    BOOST_LOG_TRIVIAL(error) << "No population for vehicle type " << creation.pTypeVehicule->GetLabel();
                    return false;
                }

                if (!pSubPopulationForTrip)
                {
                    if (pPopulationForTrip->GetListSubPopulations().size() != 1)
                    {
                        BOOST_LOG_TRIVIAL(error) << "Too much possibilities to select SubPopulation for Population " << pPopulationForTrip->GetStrName();
                        return false;
                    }
                    else
                    {
                        pSubPopulationForTrip = pPopulationForTrip->GetListSubPopulations()[0];
                    }
                }

                SymuCore::Destination * pSymuCoreDestination = pGraph->GetOrCreateDestination(creation.pDest->GetID());

                AddTrip(lstTrips, pGraph, simulationStartTime, creation.dbInstantCreation, pOrigin, creation.pDest, creation.pTypeVehicule, pVehType, pSubPopulationForTrip, bIgnoreSubAreas);
            }
        }
    }

    m_pNetwork->RestoreSeed(nRandCountSvg);

    return true;
}
