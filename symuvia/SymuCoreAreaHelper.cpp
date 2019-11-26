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

#include "stdafx.h"
#include "SymuCoreAreaHelper.h"

#include "TronconDestination.h"
#include "TronconOrigine.h"
#include "tuyau.h"
#include "reseau.h"
#include "SymuCoreManager.h"
#include "ZoneDeTerminaison.h"
#include "sensors/EdieSensor.h"

#include <Demand/MacroType.h>
#include <Utils/TravelTimeUtils.h>

std::map<Tuyau*, std::pair<double, double> > &  SymuCoreAreaHelper::GetLstLinks()
{
    return m_LstLinks;
}

void SymuCoreAreaHelper::ComputeIncomingPaths(ZoneDeTerminaison * pZone, Connexion* pFromJunction, const std::vector<Tuyau*> & lstJunctionLinks, const std::vector<SymuCore::MacroType*> & listMacroTypes)
{
    Tuyau * pItiLink, *pPrevLink;
    for (size_t iMacroType = 0; iMacroType < listMacroTypes.size(); iMacroType++)
    {
        SymuCore::MacroType * pMacroType = listMacroTypes[iMacroType];

        TypeVehicule * pTypeVeh = pFromJunction->GetReseau()->GetVehiculeTypeFromMacro(pMacroType);

        if (pTypeVeh)
        {
            // Pour chaque tuyau amont possible :
            for (std::vector<Tuyau*>::const_iterator iterJunctionLink = lstJunctionLinks.begin(); iterJunctionLink != lstJunctionLinks.end(); ++iterJunctionLink)
            {
                if (!(*iterJunctionLink) || !(*iterJunctionLink)->IsInterdit(pTypeVeh))
                {
                    std::vector<TripNode*> lstDestinations;
                    std::vector<double> lstDestinationPositions;
                    // Calcul du plus court chemin entre pFromJunction et chaque tronÃ§on de la zone :
                    for (std::map<Tuyau*, std::pair<double, double> >::const_iterator iterLink = m_LstLinks.begin(); iterLink != m_LstLinks.end(); ++iterLink)
                    {
                        Tuyau * pLink = iterLink->first;
                        if (!pLink->IsInterdit(pTypeVeh, 0))
                        {
                            TronconDestination tuyDest(pLink);
                            lstDestinations.push_back(new TronconDestination(pLink));
                            lstDestinationPositions.push_back(iterLink->second.second);
                        }
                    }

                    if (!lstDestinations.empty())
                    {
                        std::vector<std::vector<PathResult> > pathsForAllDestinations;
                        std::vector<std::map<std::vector<Tuyau*>, double> > MapFilteredItis;
                        pFromJunction->GetReseau()->GetSymuScript()->ComputePaths(pFromJunction, lstDestinations, pTypeVeh, 0, 1, *iterJunctionLink, pathsForAllDestinations, MapFilteredItis, false);

                        for (size_t iDest = 0; iDest < lstDestinations.size(); iDest++)
                        {
                            std::vector<PathResult> & paths = pathsForAllDestinations.at(iDest);
                            if (paths.size() == 1)
                            {
                                // Calcul de la longueur associÃÂ©e ÃÂ  l'itinÃÂ©raire :
                                double dbLength = 0;
                                pPrevLink = NULL;
                                for (size_t iLink = 0; iLink < paths.front().links.size(); iLink++)
                                {
                                    pItiLink = paths.front().links[iLink];

                                    if (pPrevLink && pPrevLink->GetBriqueAval())
                                    {
                                        // Prise en compte de la longueur dans la brique
                                        std::vector<Tuyau*> lstTuyauxInternes;
                                        pPrevLink->GetBriqueAval()->GetTuyauxInternes(pPrevLink, pItiLink, lstTuyauxInternes);
                                        for (size_t iTuyInt = 0; iTuyInt < lstTuyauxInternes.size(); iTuyInt++)
                                        {
                                            dbLength += lstTuyauxInternes[iTuyInt]->GetLength();
                                        }
                                    }

                                    if (iLink == paths.front().links.size() - 1)
                                    {
                                        // Position moyenne d'arrivÃÂ©e ÃÂ  destination sur le tuyau cible
                                        dbLength += lstDestinationPositions.at(iDest);
                                    }
                                    else
                                    {
                                        dbLength += pItiLink->GetLength();
                                    }

                                    // prÃÂ©paration itÃÂ©ration suivante
                                    pPrevLink = pItiLink;
                                }

                                // rmq. : par cohÃÂ©rence avec le tirage de l'origine dans la zone, on ne prend pas en compte la longueur sur le dernier tronÃÂ§on ici.
                                if (!pZone || dbLength - lstDestinationPositions.at(iDest) <= pZone->GetMaxDistanceToJunction())
                                {
                                    m_IncomingPaths[pFromJunction][*iterJunctionLink][lstDestinations.at(iDest)->GetInputPosition().GetLink()][pMacroType] = std::make_pair(paths.front().links, dbLength);
                                }
                            }
                            delete lstDestinations.at(iDest);
                        }
                    }
                }
            }
        }
    }
}

void SymuCoreAreaHelper::ComputeOutgoingPaths(ZoneDeTerminaison * pZone, Connexion* pToJunction, const std::vector<Tuyau*> & lstJunctionLinks, const std::vector<SymuCore::MacroType*> & listMacroTypes)
{
    Tuyau * pItiLink, *pPrevLink;
    for (size_t iMacroType = 0; iMacroType < listMacroTypes.size(); iMacroType++)
    {
        SymuCore::MacroType * pMacroType = listMacroTypes[iMacroType];

        TypeVehicule * pTypeVeh = pToJunction->GetReseau()->GetVehiculeTypeFromMacro(pMacroType);

        if (pTypeVeh)
        {
            // Pour chaque tuyau aval possible :
            for (std::vector<Tuyau*>::const_iterator iterJunctionLink = lstJunctionLinks.begin(); iterJunctionLink != lstJunctionLinks.end(); ++iterJunctionLink)
            {

                if (!(*iterJunctionLink) || !(*iterJunctionLink)->IsInterdit(pTypeVeh))
                {
                    std::vector<TripNode*> lstOrigins;
                    std::vector<double> lstOriginPositions;
                    for (std::map<Tuyau*, std::pair<double, double> >::const_iterator iterLink = m_LstLinks.begin(); iterLink != m_LstLinks.end(); ++iterLink)
                    {
                        Tuyau * pLink = iterLink->first;
                        if (!pLink->IsInterdit(pTypeVeh, 0))
                        {
                            lstOrigins.push_back(new TronconOrigine(pLink, NULL));
                            lstOriginPositions.push_back(iterLink->second.first);
                        }
                    }

                    if (!lstOrigins.empty())
                    {
                        std::vector<std::vector<PathResult> > pathsForAllorigins;
                        std::vector<std::map<std::vector<Tuyau*>, double> > MapFilteredItis;
                        if (*iterJunctionLink)
                        {
                            TronconDestination pTDest(*iterJunctionLink);
							
                            pToJunction->GetReseau()->GetSymuScript()->ComputePaths(lstOrigins, &pTDest, pTypeVeh, 0, 1, pathsForAllorigins, MapFilteredItis, false);
                        }
                        else
                        {
                            pToJunction->GetReseau()->GetSymuScript()->ComputePaths(lstOrigins, pToJunction, pTypeVeh, 0, 1, pathsForAllorigins, MapFilteredItis, false);
                        }

                        for (size_t iOrigin = 0; iOrigin < lstOrigins.size(); iOrigin++)
                        {
						
                            std::vector<PathResult> & paths = pathsForAllorigins.at(iOrigin);
                            if (paths.size() == 1)
                            {
                                if (*iterJunctionLink)
                                {
                                    // on enlÃ¨ve le tuyau suivant si on l'a ajoutÃ© pour vÃ©rifier les mouvements autorisÃ©s sur le noeud de jonction
									if (paths.front().links.size() > 0)
										paths.front().links.pop_back();
									
                                }
                                // Calcul de la longueur associÃ©e Ã  l'itinÃ©raire :
                                double dbLength = 0;
                                pPrevLink = NULL;

						
								for (size_t iLink = 0; iLink < paths.front().links.size(); iLink++)
								{
									
									pItiLink = paths.front().links[iLink];

								
									if (pPrevLink && pPrevLink->GetBriqueAval())
									{
										// Prise en compte de la longueur dans la brique
										std::vector<Tuyau*> lstTuyauxInternes;
										pPrevLink->GetBriqueAval()->GetTuyauxInternes(pPrevLink, pItiLink, lstTuyauxInternes);
										for (size_t iTuyInt = 0; iTuyInt < lstTuyauxInternes.size(); iTuyInt++)
										{
											dbLength += lstTuyauxInternes[iTuyInt]->GetLength();
										}
									}

									if (iLink == 0)
									{
										// Position moyenne de depart sur le tuyau origine
										dbLength += (pItiLink->GetLength() - lstOriginPositions.at(iOrigin));
									}
									else
									{
										dbLength += pItiLink->GetLength();
									}

									// prÃ©paration itÃ©ration suivante
									pPrevLink = pItiLink;
								}

                                // rmq. : par cohÃ©rence avec le tirage de l'origine dans la zone, on ne prend pas en compte la longueur sur le premier tronÃ§on ici.	
								if (!pZone || dbLength - (pItiLink->GetLength() - lstOriginPositions.at(iOrigin)) <= pZone->GetMaxDistanceToJunction())
								{
									
									m_OutgoingPaths[pToJunction][*iterJunctionLink][lstOrigins.at(iOrigin)->GetOutputPosition().GetLink()][pMacroType] = std::make_pair(paths.front().links, dbLength);
								}
								
                            }
                            delete lstOrigins.at(iOrigin);
                        }
                    }
                }
            }
        }
    }
}

double SymuCoreAreaHelper::GetTravelTime(bool bIsOrigin, Connexion * pJunction, SymuCore::MacroType * pMacroType, ZoneDeTerminaison * pZone, double & dbMarginal, double & dbNbOfVehiclesForAllMacroTypes, double & dbTravelTimeForAllMacroTypes)
{
    bool bIsAllowed = HasPath(pJunction, pMacroType, bIsOrigin);
    double dbMeanLengthForPaths = GetMeanLengthForPaths(pJunction, pMacroType, bIsOrigin);

    // RÃ©cupÃ©ration du temps de parcours
    double dbTravelTime;
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & pTravelTimes = bIsOrigin ? m_LstDepartingTravelTimes : m_LstArrivingTravelTimes;
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > >::const_iterator iterJunction = pTravelTimes.find(pJunction);

    bool bFound = false;

    if (iterJunction != pTravelTimes.end())
    {
        std::map<SymuCore::MacroType*, double >::const_iterator iterMT = iterJunction->second.find(pMacroType);
        if (iterMT != iterJunction->second.end())
        {
            bFound = true;
            dbTravelTime = iterMT->second;
        }
    }

    if (!bFound)
    {
        dbTravelTime = GetEmptyTravelTime(pJunction, bIsOrigin, pMacroType);
    }

    // Application de la vitesse minimum le cas Ã©chÃ©ant :
    if (pJunction->GetReseau()->GetMinSpeedForTravelTime() > 0 && bIsAllowed)
    {
        dbTravelTime = std::min<double>(dbTravelTime, dbMeanLengthForPaths / pJunction->GetReseau()->GetMinSpeedForTravelTime());
    }

    // RÃ©cupÃ©ration du marginal
    if (pJunction->GetReseau()->UseTravelTimesAsMarginalsInAreas())
    {
        dbMarginal = dbTravelTime;
    }
    else
    {
        if (pJunction->GetReseau()->UseSpatialTTMethod())
        {
            std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & pNbVehicles = bIsOrigin ? m_dbDepartingMeanNbVehicles : m_dbArrivingMeanNbVehicles;
            std::map<Connexion*, double> & pTTForAllmacroTypes = bIsOrigin ? m_dbDepartingTTForAllMacroTypes : m_dbArrivingTTForAllMacroTypes;

            bool bFound = false;
            std::map<Connexion*, std::map<SymuCore::MacroType*, double > >::const_iterator iterJunction = pNbVehicles.find(pJunction);
            if (iterJunction != pNbVehicles.end())
            {
                std::map<SymuCore::MacroType*, double >::const_iterator iterMT = iterJunction->second.find(pMacroType);
                if (iterMT != iterJunction->second.end())
                {
                    dbMarginal = iterMT->second;
                    dbNbOfVehiclesForAllMacroTypes = iterJunction->second.at(NULL);
                    dbTravelTimeForAllMacroTypes = pTTForAllmacroTypes.at(pJunction);
                    bFound = true;
                }
            }
            if (!bFound)
            {
                // Nombre de vÃ©hicule nul dans le cas non encore calculÃ© (Ã  vide)
                dbMarginal = 0;
                dbNbOfVehiclesForAllMacroTypes = 0;
                dbTravelTimeForAllMacroTypes = dbTravelTime;
            }
        }
        else
        {
            std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & pMarginals = bIsOrigin ? m_LstDepartingMarginals : m_LstArrivingMarginals;
            std::map<Connexion*, std::map<SymuCore::MacroType*, double > >::const_iterator iterJunctionForMarginal = pMarginals.find(pJunction);

            bool bFound = false;
            if (iterJunctionForMarginal != pMarginals.end())
            {
                std::map<SymuCore::MacroType*, double >::const_iterator iterMT = iterJunctionForMarginal->second.find(pMacroType);
                if (iterMT != iterJunctionForMarginal->second.end())
                {
                    bFound = true;
                    dbMarginal = iterMT->second;
                }
            }

            if (!bFound)
            {
                dbMarginal = dbTravelTime;
            }
        }
    }

    // Gestion du marginals maximum
    if (bIsAllowed)
    {
        dbMarginal = std::min<double>(dbMarginal, pJunction->GetReseau()->GetMaxMarginalsValue());
    }

    dbMarginal = std::max<double>(0, dbMarginal); // Pas de marginals nÃ©gatifs

    return dbTravelTime;
}

bool SymuCoreAreaHelper::isAcessibleFromConnexion(Connexion* pFromJunction) const
{
    return m_IncomingPaths.find(pFromJunction) != m_IncomingPaths.end();
}

bool SymuCoreAreaHelper::isAcessibleToConnexion(Connexion* pToJunction) const
{
    return m_OutgoingPaths.find(pToJunction) != m_OutgoingPaths.end();
}

bool SymuCoreAreaHelper::isAcessibleFromConnexion(Connexion* pFromJunction, Tuyau * pUpstreamLink, bool bForSymuMaster, std::set<TypeVehicule*> & lstAllowedVehicleTypes) const
{
    if (bForSymuMaster)
    {
        std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterCon = m_IncomingPaths.find(pFromJunction);
        if (iterCon != m_IncomingPaths.end())
        {
            return iterCon->second.find(pUpstreamLink) != iterCon->second.end();
        }
        return false;
    }
    else
    {
        bool bIsAccessible = false;
        // Dans ce cas, on vÃ©rifie qu'on a un mouvement autorisÃ© depuis pUpstreamLink vers la zone
        for (size_t iTV = 0; iTV < pFromJunction->GetReseau()->m_LstTypesVehicule.size(); iTV++)
        {
            TypeVehicule * pTypeVeh = pFromJunction->GetReseau()->m_LstTypesVehicule.at(iTV);

            if (pUpstreamLink->IsInterdit(pTypeVeh))
                continue;

            for (size_t iLink = 0; iLink < pFromJunction->m_LstTuyAssAv.size(); iLink++)
            {
                Tuyau * pDownstreamLink = pFromJunction->m_LstTuyAssAv.at(iLink);

                if (pDownstreamLink->IsInterdit(pTypeVeh))
                    continue;

                if (pFromJunction->IsMouvementAutorise(pUpstreamLink, pDownstreamLink, pTypeVeh, NULL))
                {
                    if (m_LstLinks.find(pDownstreamLink) != m_LstLinks.end())
                    {
                        lstAllowedVehicleTypes.insert(pTypeVeh);
                        bIsAccessible = true;
                        break;
                    }
                }
            }
        }
        return bIsAccessible;
    }
}

bool SymuCoreAreaHelper::isAcessibleToConnexion(Connexion* pToJunction, Tuyau * pDownstreamLink, bool bForSymuMaster, std::set<TypeVehicule*> & lstAllowedVehicleTypes) const
{
    if (bForSymuMaster)
    {
        std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterCon = m_OutgoingPaths.find(pToJunction);
        if (iterCon != m_OutgoingPaths.end())
        {
            return iterCon->second.find(pDownstreamLink) != iterCon->second.end();
        }
        return false;
    }
    else
    {
        // Dans ce cas, on vÃ©rifie qu'on a un mouvement autorisÃ© vers pDownstreamLink depuis la zone,
        // et ce depuis et vers un tronÃ§on ou le type de vÃ©hicule peut circuler :
        bool bIsAccessible = false;
        for (size_t iTV = 0; iTV < pToJunction->GetReseau()->m_LstTypesVehicule.size(); iTV++)
        {
            TypeVehicule * pTypeVeh = pToJunction->GetReseau()->m_LstTypesVehicule.at(iTV);

            if (pDownstreamLink->IsInterdit(pTypeVeh))
                continue;

            for (size_t iLink = 0; iLink < pToJunction->m_LstTuyAssAm.size(); iLink++)
            {
                Tuyau * pUpstreamLink = pToJunction->m_LstTuyAssAm.at(iLink);
                if (pUpstreamLink->IsInterdit(pTypeVeh))
                    continue;

                if (pToJunction->IsMouvementAutorise(pUpstreamLink, pDownstreamLink, pTypeVeh, NULL))
                {
                    if (m_LstLinks.find(pUpstreamLink) != m_LstLinks.end())
                    {
                        lstAllowedVehicleTypes.insert(pTypeVeh);
                        bIsAccessible = true;
                        break;
                    }
                }
            }
        }
        return bIsAccessible;
    }
}

bool SymuCoreAreaHelper::isAcessibleFromConnexionWithDownstreamLink(Connexion* pFromJunction, Tuyau * pDownstreamLink) const
{
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterCon = m_IncomingPaths.find(pFromJunction);
    if (iterCon != m_IncomingPaths.end())
    {
        for (std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > >::const_iterator iterUpstreamLink
            = iterCon->second.begin(); iterUpstreamLink != iterCon->second.end(); ++iterUpstreamLink)
        {
            for (std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > >::const_iterator iterDestinationLink
                = iterUpstreamLink->second.begin(); iterDestinationLink != iterUpstreamLink->second.end(); ++iterDestinationLink)
            {
                for (std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> >::const_iterator iterMacroType
                    = iterDestinationLink->second.begin(); iterMacroType != iterDestinationLink->second.end(); ++iterMacroType)
                {
					if (iterMacroType->second.first.size() > 0)
					{
						if (iterMacroType->second.first.front() == pDownstreamLink)
						{							
							return true;
						}
					}
                }
            }
        }
    }
    return false;
}

bool SymuCoreAreaHelper::isAcessibleToConnexionWithUpstreamLink(Connexion* pToJunction, Tuyau * pUpstreamLink) const
{
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterCon = m_OutgoingPaths.find(pToJunction);
    if (iterCon != m_OutgoingPaths.end())
    {
        for (std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > >::const_iterator iterDownstreamLink
            = iterCon->second.begin(); iterDownstreamLink != iterCon->second.end(); ++iterDownstreamLink)
        {
            for (std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > >::const_iterator iterOriginLink
                = iterDownstreamLink->second.begin(); iterOriginLink != iterDownstreamLink->second.end(); ++iterOriginLink)
            {
                for (std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> >::const_iterator iterMacroType
                    = iterOriginLink->second.begin(); iterMacroType != iterOriginLink->second.end(); ++iterMacroType)
                {
					if (iterMacroType->second.first.size() > 0)
					{
						if (iterMacroType->second.first.back() == pUpstreamLink)
						{						
							return true;
						}
					}
                }
            }
        }
    }
    return false;
}

bool SymuCoreAreaHelper::isAccessibleToArea(Connexion* pToJunction, SymuCoreAreaHelper * pToArea, bool bForSymuMaster, std::set<TypeVehicule*> & lstAllowedVehicleTypes) const
{
    if (bForSymuMaster)
    {
        std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterCon = m_OutgoingPaths.find(pToJunction);
        if (iterCon != m_OutgoingPaths.end())
        {
            // le tuyau aval Ã  la zone doit Ãªtre dans un des itinÃ©raires d'entrÃ©e dans la zone cible :
            for (std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > >::const_iterator iterDownstreamLink
                = iterCon->second.begin(); iterDownstreamLink != iterCon->second.end(); ++iterDownstreamLink)
            {
                // 20/12/2018 : ajout d'un test d'appartenance du tronÃ§on aval Ã  la zone cible (cette mÃ©thode est appelÃ©e pour vÃ©rifier si 
                // deux zones sont jointives)
                if (pToArea->GetLstLinks().find(iterDownstreamLink->first) != pToArea->GetLstLinks().end()
                    && pToArea->isAcessibleFromConnexionWithDownstreamLink(pToJunction, iterDownstreamLink->first))
                {
                    return true;
                }
            }
        }
        return false;
    }
    else
    {
        // Dans ce cas, on vÃ©rifie qu'on a un mouvement autorisÃ© vers pToArea depuis la zone
        bool bIsAccessible = false;
        for (std::map<Tuyau*, std::pair<double, double> >::const_iterator iterLink = pToArea->GetLstLinks().begin(); iterLink != pToArea->GetLstLinks().end(); ++iterLink)
        {
            Tuyau * pDownstreamLink = iterLink->first;
            if (pDownstreamLink->GetCnxAssAm() == pToJunction)
            {
                for (size_t iTV = 0; iTV < pToJunction->GetReseau()->m_LstTypesVehicule.size(); iTV++)
                {
                    TypeVehicule * pTypeVeh = pToJunction->GetReseau()->m_LstTypesVehicule.at(iTV);
                    if (pDownstreamLink->IsInterdit(pTypeVeh))
                        continue;

                    for (size_t iLink = 0; iLink < pToJunction->m_LstTuyAssAm.size(); iLink++)
                    {
                        Tuyau * pUpstreamLink = pToJunction->m_LstTuyAssAm.at(iLink);

                        if (pUpstreamLink->IsInterdit(pTypeVeh))
                            continue;

                        if (pToJunction->IsMouvementAutorise(pUpstreamLink, pDownstreamLink, pTypeVeh, NULL))
                        {
                            if (m_LstLinks.find(pUpstreamLink) != m_LstLinks.end())
                            {
                                bIsAccessible = true;
                            }
                        }
                    }
                }
            }
        }
        return bIsAccessible;
    }
}

bool SymuCoreAreaHelper::isAccessibleFromArea(Connexion* pFromJunction, SymuCoreAreaHelper * pFromArea, bool bForSymuMaster, std::set<TypeVehicule*> & lstAllowedVehicleTypes) const
{
    if (bForSymuMaster)
    {
        std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterCon = m_IncomingPaths.find(pFromJunction);
        if (iterCon != m_IncomingPaths.end())
        {
            // le tuyau amont Ã  la zone cible doit Ãªtre le dernier tronÃ§on d'un des itinÃ©raires d'entrÃ©e dans la zone cible :
            for (std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > >::const_iterator iterUpstreamLink
                = iterCon->second.begin(); iterUpstreamLink != iterCon->second.end(); ++iterUpstreamLink)
            {
                // 20/12/2018 : ajout d'un test d'appartenance du tronÃ§on amont Ã  la zone d'origine (cette mÃ©thode est appelÃ©e pour vÃ©rifier si 
                // deux zones sont jointives)
                if (pFromArea->GetLstLinks().find(iterUpstreamLink->first) != pFromArea->GetLstLinks().end()
                    && pFromArea->isAcessibleToConnexionWithUpstreamLink(pFromJunction, iterUpstreamLink->first))
                {
                    return true;
                }
            }
        }
        return false;
    }
    else
    {
        bool bIsAccessible = false;
        // Dans ce cas, on vÃ©rifie qu'on a un mouvement autorisÃ© depuis pFromArea vers la zone
        for (std::map<Tuyau*, std::pair<double, double> >::const_iterator iterLink = pFromArea->GetLstLinks().begin(); iterLink != pFromArea->GetLstLinks().end(); ++iterLink)
        {
            Tuyau * pUpstreamLink = iterLink->first;

            if (pFromJunction == pUpstreamLink->GetCnxAssAv())
            {
                for (size_t iTV = 0; iTV < pFromJunction->GetReseau()->m_LstTypesVehicule.size(); iTV++)
                {
                    TypeVehicule * pTypeVeh = pFromJunction->GetReseau()->m_LstTypesVehicule.at(iTV);

                    if (pUpstreamLink->IsInterdit(pTypeVeh))
                        continue;

                    for (size_t iLink = 0; iLink < pFromJunction->m_LstTuyAssAv.size(); iLink++)
                    {
                        Tuyau * pDownstreamLink = pFromJunction->m_LstTuyAssAv.at(iLink);

                        if (pDownstreamLink->IsInterdit(pTypeVeh))
                            continue;

                        if (pFromJunction->IsMouvementAutorise(pUpstreamLink, pDownstreamLink, pTypeVeh, NULL))
                        {
                            if (m_LstLinks.find(pDownstreamLink) != m_LstLinks.end())
                            {
                                lstAllowedVehicleTypes.insert(pTypeVeh);
                                bIsAccessible = true;
                                break;
                            }
                        }
                    }
                }
            }
        }
        return bIsAccessible;
    }
}

const std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > & SymuCoreAreaHelper::GetPrecomputedPaths(bool bPlaqueOrigine)
{
    if (bPlaqueOrigine)
    {
        return m_OutgoingPaths;
    }
    else
    {
        return m_IncomingPaths;
    }
}

void SymuCoreAreaHelper::CalculTempsParcours(Reseau * pNetwork, ZoneDeTerminaison * pZone, double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration, bool bDeparting)
{
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & pLstTravelTimes = bDeparting ? m_LstDepartingTravelTimes : m_LstArrivingTravelTimes;
    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & pLstMarginals = bDeparting ? m_LstDepartingMarginals : m_LstArrivingMarginals;

    std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & pMapMeanVehicleNb = bDeparting ? m_dbDepartingMeanNbVehicles : m_dbArrivingMeanNbVehicles;
    std::map<Connexion*, double > & pMapTTForAllMacroTypes = bDeparting ? m_dbDepartingTTForAllMacroTypes : m_dbArrivingTTForAllMacroTypes;

    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > & pPaths = 
        bDeparting ? m_OutgoingPaths : m_IncomingPaths;

    std::map<Connexion*, std::map<int, std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > > > & pMapVeh = bDeparting ? m_mapDepartingVeh : m_mapArrivingVeh;

    // Pour chaque point de jonction :
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterJunction;
    for (iterJunction = pPaths.begin(); iterJunction != pPaths.end(); ++iterJunction)
    {
        bool bHasPath = HasPath(iterJunction->first, pMacroType, bDeparting);
        double dbMeanLengthForPaths = GetMeanLengthForPaths(iterJunction->first, pMacroType, bDeparting);
        double dbMeanVehicleNbForMacroType = 0;
        double dbMeanVehicleNbForAllMacroTypes = 0;
        if (!pNetwork->UseTravelTimesAsMarginalsInAreas()) // Pas la peine de faire le calcul sinon.
        {
            dbMeanVehicleNbForMacroType = GetMeanVehicleNumber(iterJunction->first, pMacroType, bDeparting, dbPeriodDuration);
            dbMeanVehicleNbForAllMacroTypes = GetMeanVehicleNumber(iterJunction->first, NULL, bDeparting, dbPeriodDuration);
        }

        double & TTForMacroType = pLstTravelTimes[iterJunction->first][pMacroType];
        double & dbMarginalForMacroType = pLstMarginals[iterJunction->first][pMacroType];

        // Calcul du temps de parcours
        if (pNetwork->UseSpatialTTMethod())
        {
            double dbMeanSpeed = pZone->m_dbMeanSpeed.at(pMacroType);
            double dbEmptyMeanSpeed = pZone->GetEmptyMeanSpeed(pNetwork->m_dbInstSimu, pMacroType);

            TTForMacroType = SymuCore::TravelTimeUtils::ComputeSpatialTravelTime(dbMeanSpeed, dbEmptyMeanSpeed, dbMeanLengthForPaths, pZone->m_dbConcentration, !bHasPath, pNetwork->GetConcentrationRatioForFreeFlowCriterion(), pNetwork->GetMainKx());

            // Calcul du marginal spatialisÃ© : dans ce mode, on stocke le nombre moyen de vÃ©hicules et le temps de parcours, et on fait le calcul Ã  la fin de la pÃ©riode de prÃ©diction
            pMapMeanVehicleNb[iterJunction->first][pMacroType] = dbMeanVehicleNbForMacroType;
            pMapMeanVehicleNb[iterJunction->first][NULL] = dbMeanVehicleNbForAllMacroTypes;
            double dbMeanSpeedForAllMacroTypes = pZone->m_dbMeanSpeed.at(NULL);
            pMapTTForAllMacroTypes[iterJunction->first] = SymuCore::TravelTimeUtils::ComputeSpatialTravelTime(dbMeanSpeedForAllMacroTypes, dbEmptyMeanSpeed, dbMeanLengthForPaths, pZone->m_dbConcentration, !bHasPath, pNetwork->GetConcentrationRatioForFreeFlowCriterion(), pNetwork->GetMainKx());
        }
        else
        {
            std::map<int, std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > > & departingVehs = pMapVeh[iterJunction->first];

            // calcul du marginal
            std::map<double, std::map<int, std::pair<bool, std::pair<double, double> > > > mapUsefuleVehicles;
            for (std::map<int, std::pair<TypeVehicule*, std::pair<std::pair<double, double>, double> > >::const_iterator itVeh = departingVehs.begin(); itVeh != departingVehs.end(); ++itVeh)
            {
                // contrairement aux cas tronÃ§on et connexion, pour les zones, il faut calculer un ratio de longueurs permettant de se ramener Ã  la longueur moyenne des chemins correspondants
                // pour la zone/plaque. Le ratio doit donc Ãªtre longueur moyenne / longueur utilisateur (comme ca on multiplie le temps passÃ© par le ratio pour chaque utilisateur pour se ramener Ã  un temps relatif Ã  la longueur moyenne parcourue en zone)
                double dbLengthRatioToMeanLength;
                if (dbMeanLengthForPaths == 0)
                {
                    // cas de temps de parcours indÃ©finis si on a un 0 / 0 ici. pour Ã©viter Ã§a, on prend un ratio de longueur 1 dÃ¨s que dbMeanLengthForPaths vaut 0
                    dbLengthRatioToMeanLength = 1;
                }
                else
                {
                    dbLengthRatioToMeanLength = dbMeanLengthForPaths / itVeh->second.second.second;
                }
                mapUsefuleVehicles[itVeh->second.second.first.first][itVeh->first] = std::make_pair(pMacroType->hasVehicleType(itVeh->second.first->GetLabel()), std::make_pair(itVeh->second.second.first.second, dbLengthRatioToMeanLength));
            }

            double dbITT;
            std::vector<std::pair<std::pair<double, double>, double> > exitInstantAndTravelTimesWithLengthsForMacroType, exitInstantAndTravelTimesWithLengthsForAllMacroTypes;
            std::vector<int> vehicleIDsToClean;
            SymuCore::TravelTimeUtils::SelectVehicleForTravelTimes(mapUsefuleVehicles, dbInstFinPeriode, dbPeriodDuration, pNetwork->GetNbPeriodsForTravelTimes(), pNetwork->GetNbVehiclesForTravelTimes(), pNetwork->GetMarginalsDeltaN(), pNetwork->UseMarginalsMeanMethod(), pNetwork->UseMarginalsMedianMethod(), dbITT, exitInstantAndTravelTimesWithLengthsForMacroType, exitInstantAndTravelTimesWithLengthsForAllMacroTypes, vehicleIDsToClean);

            // Nettoyage des vÃ©hicules devenus trop anciens :
            for (size_t iVeh = 0; iVeh < vehicleIDsToClean.size(); iVeh++)
            {
                departingVehs.erase(vehicleIDsToClean[iVeh]);
            }

            double dbEmptyTravelTime = GetEmptyTravelTime(iterJunction->first, bDeparting, pMacroType);
            TTForMacroType = SymuCore::TravelTimeUtils::ComputeTravelTime(exitInstantAndTravelTimesWithLengthsForMacroType, pZone->m_dbConcentration, dbEmptyTravelTime, !bHasPath, pNetwork->GetNbVehiclesForTravelTimes(), pNetwork->GetConcentrationRatioForFreeFlowCriterion(), pNetwork->GetMainKx());

            // calcul du marginal
            // Pour les marginals, on prend tous les derniÃ¨rs vÃ©hicules sortis du tronÃ§on quelquesoit leur macrotype : 
            dbMarginalForMacroType = SymuCore::TravelTimeUtils::ComputeTravelTime(exitInstantAndTravelTimesWithLengthsForAllMacroTypes, pZone->m_dbConcentration, dbEmptyTravelTime, !bHasPath, pNetwork->GetNbVehiclesForTravelTimes(), pNetwork->GetConcentrationRatioForFreeFlowCriterion(), pNetwork->GetMainKx());
            // Le dbITT est aussi pour tous les macrotypes : seul le nombre de vÃ©hicules sur le tronÃ§on est pris pour le macro-type considÃ©rÃ©.
            dbMarginalForMacroType += dbITT * dbMeanVehicleNbForMacroType;
        }
    }
}

bool SymuCoreAreaHelper::HasPath(Connexion * pJunction, SymuCore::MacroType * pMacroType, bool bIsOrigin)
{
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > & pPaths =
        bIsOrigin ? m_OutgoingPaths : m_IncomingPaths;

    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterJunction = pPaths.find(pJunction);

    // rmq. possibilitÃ© d'optimisation si ca prend du temps puisque tout Ã§a ne devrait pas varier au cours de la simulation (ne le calculer qu'une fois et mettre en cache la rÃ©sultat?)
    if (iterJunction != pPaths.end())
    {
        for (std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > >::const_iterator iterPaths = iterJunction->second.begin();
            iterPaths != iterJunction->second.end(); ++iterPaths)
        {
            for (std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > >::const_iterator iterPathTuy = iterPaths->second.begin();
                iterPathTuy != iterPaths->second.end(); ++iterPathTuy)
            {
                std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> >::const_iterator iterPathMacroType = iterPathTuy->second.find(pMacroType);
                if (iterPathMacroType != iterPathTuy->second.end())
                {
                    return true;
                }
            }
        }
    }
    return false;
}

// Pour traiter les itinÃ©raires toujours dans le mÃªme ordre pour avoir des simus reproductibles (sinon
// on peut avoir une erreur d'arrondi sur la somme si faite dans un ordre diffÃ©rent
class PathPairSorter {
public:
    bool operator()(const std::pair<std::vector<Tuyau*>, double> & a, const std::pair<std::vector<Tuyau*>, double> & b) const
    {
        if (a.second != b.second)
        {
            return a.second < b.second;
        }
        else
        {
            if (a.first.size() != b.first.size())
            {
                return a.first.size() < b.first.size();
            }
            else
            {
                for (size_t iLink = 0; iLink < a.first.size(); iLink++)
                {
                    const std::string & aLabel = a.first.at(iLink)->GetLabel();
                    const std::string & bLabel = b.first.at(iLink)->GetLabel();
                    if (aLabel != bLabel)
                    {
                        return aLabel < bLabel;
                    }
                }
                return false;
            }
        }
    }
};

double SymuCoreAreaHelper::GetMeanLengthForPaths(Connexion * pJunction, SymuCore::MacroType * pMacroType, bool bIsOrigin)
{
    double dbMeanLength = 0;

    // rmq. possibilitÃ© d'optimisation si ca prend du temps puisque tout Ã§a ne devrait pas varier au cours de la simulation (ne le calculer qu'une fois et mettre en cache la rÃ©sultat?)
    
    // Ensemble unique des itinÃ©raires sur lesquels moyenner le temps de parcours (il peut y avoir des doublons car on stocke les itinÃ©raires par tuyau de jonction et il peut y en avoir
    // plusieurs pour une mÃªme connexion et un mÃªme itinÃ©raire) :
    std::set<std::pair<std::vector<Tuyau*>, double>, PathPairSorter> setPaths;

    const std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > & paths = GetPrecomputedPaths(bIsOrigin);
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterJunction = paths.find(pJunction);
    if (iterJunction != paths.end())
    {
        for (std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > >::const_iterator iterLinkJunction = iterJunction->second.begin(); iterLinkJunction != iterJunction->second.end(); ++iterLinkJunction)
        {
            for (std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > >::const_iterator iterLink = iterLinkJunction->second.begin(); iterLink != iterLinkJunction->second.end(); ++iterLink)
            {
                std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> >::const_iterator iterMacroType = iterLink->second.find(pMacroType);
                if (iterMacroType != iterLink->second.end())
                {
                    setPaths.insert(iterMacroType->second);
                }
            }
        }
    }

    for (std::set<std::pair<std::vector<Tuyau*>, double>, PathPairSorter>::const_iterator iterPath = setPaths.begin(); iterPath != setPaths.end(); ++iterPath)
    {
        const std::pair<std::vector<Tuyau*>, double> & path = *iterPath;
        dbMeanLength += path.second;
    }
    dbMeanLength /= setPaths.size();

    return dbMeanLength;
}

double SymuCoreAreaHelper::GetMeanVehicleNumber(Connexion * pJunction, SymuCore::MacroType * pMacroType, bool bIsOrigin, double dbPeriodDuration)
{
    // Ensemble unique des tronÃ§ons prÃ©sents sur les itinÃ©raires Ã  prendre en compte pour le calcul
    // plusieurs pour une mÃªme connexion et un mÃªme itinÃ©raire) :
    std::set<Tuyau*, LessPtr<Tuyau> > links;

    const std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > & paths = GetPrecomputedPaths(bIsOrigin);
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterJunction = paths.find(pJunction);
    if (iterJunction != paths.end())
    {
        for (std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > >::const_iterator iterLinkJunction = iterJunction->second.begin(); iterLinkJunction != iterJunction->second.end(); ++iterLinkJunction)
        {
            for (std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > >::const_iterator iterLink = iterLinkJunction->second.begin(); iterLink != iterLinkJunction->second.end(); ++iterLink)
            {
                std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> >::const_iterator iterMacroType = iterLink->second.find(pMacroType);
                if (iterMacroType != iterLink->second.end())
                {
                    links.insert(iterMacroType->second.first.begin(), iterMacroType->second.first.end());
                }
            }
        }
    }

    double dbTravelledTimeForMacroType = 0;
    double dbTotalLength = 0;
    for (std::set<Tuyau*, LessPtr<Tuyau>>::const_iterator iterLink = links.begin(); iterLink != links.end(); ++iterLink)
    {
        TuyauMicro * pLink = (TuyauMicro*)*iterLink;

        // Calcul de la concentration et du nombre moyen de vÃ©hicules sur la cellule :
        const std::map<TypeVehicule*, double> & mapTT = pLink->m_pEdieSensor->GetData().dbTotalTravelledTime.back();
        for (std::map<TypeVehicule*, double>::const_iterator iterTT = mapTT.begin(); iterTT != mapTT.end(); ++iterTT)
        {
            if (pMacroType->hasVehicleType(iterTT->first->GetLabel()))
            {
                dbTravelledTimeForMacroType += iterTT->second;
            }
        }
        
        dbTotalLength += pLink->GetLength();
    }

    double dbMeanVehicleNbForMacroType = dbTravelledTimeForMacroType / dbPeriodDuration;

    // On se ramÃ¨ne Ã  un nombre de vÃ©hicule pour la longueur moyenne de l'itinÃ©raire du pattern :
    double dbMeanLength = GetMeanLengthForPaths(pJunction, pMacroType, bIsOrigin);
    if (dbTotalLength > 0)
    {
        dbMeanVehicleNbForMacroType = dbMeanVehicleNbForMacroType * dbMeanLength / dbTotalLength;
    }

    return dbMeanVehicleNbForMacroType;
}

// Pour traiter les itinÃ©raires toujours dans le mÃªme ordre pour avoir des simus reproductibles (sinon
// on peut avoir une erreur d'arrondi sur la somme si faite dans un ordre diffÃ©rent
class PathSorter {
public:
    bool operator()(const std::vector<Tuyau*> & a, const std::vector<Tuyau*> & b) const
    {
        if (a.size() != b.size())
        {
            return a.size() < b.size();
        }
        else
        {
            for (size_t iLink = 0; iLink < a.size(); iLink++)
            {
                const std::string & aLabel = a.at(iLink)->GetLabel();
                const std::string & bLabel = b.at(iLink)->GetLabel();
                if (aLabel != bLabel)
                {
                    return aLabel < bLabel;
                }
            }
            return false;
        }
    }
};


double SymuCoreAreaHelper::GetEmptyTravelTime(Connexion * pJunction, bool bIsOrigin, SymuCore::MacroType * pMacroType)
{
    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > > & pPaths =
        bIsOrigin ? m_OutgoingPaths : m_IncomingPaths;

    std::map<Connexion*, std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > > >::const_iterator iterJunction = pPaths.find(pJunction);
    std::set<std::vector<Tuyau*>, PathSorter> lstPaths;

    // Calcul du temps de parcours Ã  vide de rÃ©fÃ©rence : on dÃ©termine la liste des itinÃ©raires uniques correspondants,
    // puis on moyenne le temps de parcours Ã  vide.
    // rmq. possibilitÃ© d'optimisation si ca prend du temps puisque tout Ã§a ne devrait pas varier au cours de la simulation (ne le calcul qu'une fois ?)
    if (iterJunction != pPaths.end())
    {
        for (std::map<Tuyau*, std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > > >::const_iterator iterPaths = iterJunction->second.begin();
            iterPaths != iterJunction->second.end(); ++iterPaths)
        {
            for (std::map<Tuyau*, std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> > >::const_iterator iterPathTuy = iterPaths->second.begin();
                iterPathTuy != iterPaths->second.end(); ++iterPathTuy)
            {
                std::map<SymuCore::MacroType*, std::pair<std::vector<Tuyau*>, double> >::const_iterator iterPathMacroType = iterPathTuy->second.find(pMacroType);
                if (iterPathMacroType != iterPathTuy->second.end())
                {
                    lstPaths.insert(iterPathMacroType->second.first);
                }
            }
        }
    }

    // Temps de parcours de rÃ©fÃ©rence au cas oÃ¹ aucun vÃ©hicule n'ait rÃ©alisÃ© le pattern :
    double dbTpsVitReg = 0;
    for (std::set<std::vector<Tuyau*>, PathSorter> ::const_iterator iterPath = lstPaths.begin(); iterPath != lstPaths.end(); ++iterPath)
    {
        const std::vector<Tuyau*> & path = *iterPath;
        double dbTpsVitRegForPath = 0;
        TuyauMicro * pPrevLink = NULL;
        for (size_t iLink = 0; iLink < path.size(); iLink++)
        {
            TuyauMicro * pLink = (TuyauMicro*)path[iLink];

            if (pPrevLink)
            {
                // Mouvement amont :
                dbTpsVitRegForPath += pPrevLink->GetCnxAssAv()->ComputeCost(pMacroType, pPrevLink, pLink);
            }

            // Tuyau :
            double dbTravelTimeInLink = pLink->GetTravelTime(pMacroType);

            if (bIsOrigin && iLink == 0)
            {
                // Prise en compte de la position de dpart moyenne sur le premier lien
                if (pLink->GetLength() > 0)
                {
                    dbTravelTimeInLink = dbTravelTimeInLink * (pLink->GetLength() - m_LstLinks.at(pLink).first) / pLink->GetLength();
                }
            }
            else if (!bIsOrigin && iLink == path.size() - 1)
            {
                // Prise en compte de la position de fin moyenne sur le dernier lien
                if (pLink->GetLength() > 0)
                {
                    dbTravelTimeInLink = dbTravelTimeInLink * m_LstLinks.at(pLink).second / pLink->GetLength();
                }
            }

            dbTpsVitRegForPath += dbTravelTimeInLink;

            // PrÃ©paration itÃ©ration suivante
            pPrevLink = pLink;
        }

        dbTpsVitReg += dbTpsVitRegForPath;
    }

    if (lstPaths.empty())
    {
        dbTpsVitReg = std::numeric_limits<double>::infinity();
    }
    else
    {
        dbTpsVitReg /= lstPaths.size();
    }

    return dbTpsVitReg;
}

void SymuCoreAreaHelper::CalculTempsParcoursDepart(Reseau * pNetwork, ZoneDeTerminaison * pZone, double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration)
{
    CalculTempsParcours(pNetwork, pZone, dbInstFinPeriode, pMacroType, dbPeriodDuration, true);
}

void SymuCoreAreaHelper::CalculTempsParcoursArrivee(Reseau * pNetwork, ZoneDeTerminaison * pZone, double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration)
{
    CalculTempsParcours(pNetwork, pZone, dbInstFinPeriode, pMacroType, dbPeriodDuration, false);
}

std::map<Connexion*, std::map<int, std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > > > & SymuCoreAreaHelper::GetMapDepartingVehicles()
{
    return m_mapDepartingVeh;
}

std::map<Connexion*, std::map<int, std::pair< TypeVehicule*, std::pair<std::pair<double, double>, double> > > > & SymuCoreAreaHelper::GetMapArrivingVehicles()
{
    return m_mapArrivingVeh;
}

std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & SymuCoreAreaHelper::GetMapDepartingTT()
{
    return m_LstDepartingTravelTimes;
}

std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & SymuCoreAreaHelper::GetMapDepartingMarginals()
{
    return m_LstDepartingMarginals;
}

std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & SymuCoreAreaHelper::GetMapArrivingTT()
{
    return m_LstArrivingTravelTimes;
}

std::map<Connexion*, std::map<SymuCore::MacroType*, double > > & SymuCoreAreaHelper::GetMapArrivingMarginals()
{
    return m_LstArrivingMarginals;
}

std::map<Connexion*, std::map<SymuCore::MacroType*, double> > & SymuCoreAreaHelper::GetMapDepartingMeanNbVehicles()
{
    return m_dbDepartingMeanNbVehicles;
}
std::map<Connexion*, std::map<SymuCore::MacroType*, double> > & SymuCoreAreaHelper::GetMapArrivingMeanNbVehicles()
{
    return m_dbArrivingMeanNbVehicles;
}

std::map<Connexion*, double> & SymuCoreAreaHelper::GetMapDepartingTTForAllMacroTypes()
{
    return m_dbDepartingTTForAllMacroTypes;
}
std::map<Connexion*, double> & SymuCoreAreaHelper::GetMapArrivingTTForAllMacroTypes()
{
    return m_dbArrivingTTForAllMacroTypes;
}
