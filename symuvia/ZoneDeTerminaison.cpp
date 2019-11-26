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
#include "ZoneDeTerminaison.h"

#include "Connection.h"
#include "tuyau.h"
#include "Parking.h"
#include "TronconDestination.h"
#include "RandManager.h"
#include "Affectation.h"
#include "BriqueDeConnexion.h"
#include "reseau.h"
#include "vehicule.h"
#include "voie.h"
#include "SystemUtil.h"
#include "Logger.h"
#include "SymuCoreManager.h"
#include "Plaque.h"
#include "usage/TripLeg.h"
#include "usage/Trip.h"
#include "usage/SymuViaFleetParameters.h"
#include "RepMotif.h"
#include "Plaque.h"
#include "Motif.h"
#include "ParkingParameters.h"
#include "sensors/MFDSensor.h"

#ifdef USE_SYMUCORE
#include <Demand/MacroType.h>
#include "sensors/EdieSensor.h"
#endif // USE_SYMUCORE

#include <boost/serialization/vector.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/shared_ptr.hpp>

using namespace std;

// *************************************
// Classe ZoneDeTerminaison
// *************************************

ZoneDeTerminaison::ZoneDeTerminaison() :
    SymuViaTripNode()
{
    m_dbMaxDistanceToJunction = std::numeric_limits<double>::infinity();

    m_pParkingParameters = NULL;
    m_bOwnsParkingParams = false;

#ifdef USE_SYMUCORE
    m_pMFDSensor = NULL;
    m_dbConcentration = 0;
#endif // USESYMUCORE
}

ZoneDeTerminaison::ZoneDeTerminaison(string sID, Reseau * pReseau) :
    SymuViaTripNode(sID, pReseau)
{
    m_dbMaxDistanceToJunction = std::numeric_limits<double>::infinity();

    m_pParkingParameters = NULL;
    m_bOwnsParkingParams = false;

#ifdef USE_SYMUCORE
    m_pMFDSensor = NULL;
    m_dbConcentration = 0;
#endif // USESYMUCORE
}


ZoneDeTerminaison::~ZoneDeTerminaison(void)
{
    if (m_pParkingParameters && m_bOwnsParkingParams)
    {
        delete m_pParkingParameters;
    }

    for (size_t i = 0; i < m_LstPlaques.size(); i++)
    {
        delete m_LstPlaques[i];
    }

    std::map<std::pair<Tuyau*,double>, TronconDestination*>::iterator iter;
    for(iter = m_CacheTronconsDestination.begin(); iter != m_CacheTronconsDestination.end(); ++iter)
    {
        delete iter->second;
    }

#ifdef USE_SYMUCORE
    if (m_pMFDSensor)
    {
        delete m_pMFDSensor;
    }
#endif // USESYMUCORE
}

void ZoneDeTerminaison::VehiculeEnter(boost::shared_ptr<Vehicule> pVehicle, VoieMicro * pDestinationEnterLane, double dbInstDestinationReached, double dbInstant, double dbTimeStep, bool bForcedOutside)
{
	//m_pNetwork->log() << std::endl << "ZoneDeTerminaison::VehiculeEnter - id " << pVehicle.get()->GetID() << std::endl;
}

// Initialisation de la zone (dÃ©tection des extrÃ©mitÃ©s de la zone,
// crÃ©ation des origines et destinations associÃ©es)
void ZoneDeTerminaison::SetZonePosition(const std::vector<Tuyau*> & tuyaux)
{
    m_InputPosition.SetZone(tuyaux, GetID(), m_pNetwork);
    m_OutputPosition.SetZone(tuyaux, GetID(), m_pNetwork);
#ifdef USE_SYMUCORE
    for (size_t iLink = 0; iLink < tuyaux.size(); iLink++)
    {
        Tuyau * pLink = tuyaux[iLink];
        // position moyenne de dÃ©part : premier tiers du tronÃ§on (suite Ã  modif Ludo pour Charlotte, Ã  valider et reprendre si ca rechange).
        // position moyenne d'arrivÃ©e : 0 car la destination est le dÃ©but du tronÃ§on tel que codÃ© actuellement dans ReaffecteVehiculeTronconDistribuÃ© si pas de plaque
        m_AreaHelper.GetLstLinks()[pLink] = std::make_pair(pLink->GetLength() / 3, 0.0);
    }
#endif // USE_SYMUCORE
}

std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > ZoneDeTerminaison::GetInputJunctions()
{
    std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > result;
    Tuyau * pTuyau;
    Connexion * pCon;
    for (size_t iTuy = 0; iTuy < GetInputConnexion()->m_LstTuyAssAm.size(); iTuy++)
    {
        pCon = GetInputConnexion()->m_LstTuyAssAm[iTuy]->GetCnxAssAv();
        std::vector<Tuyau*> setLinks;
        for (size_t iLink = 0; iLink < pCon->m_LstTuyAssAm.size(); iLink++)
        {
            pTuyau = pCon->m_LstTuyAssAm[iLink];

            // Le tuyau doit Ãªtre un tuyau d'accÃ¨s Ã  la zone :
            if (std::find(GetInputConnexion()->m_LstTuyAssAm.begin(), GetInputConnexion()->m_LstTuyAssAm.end(), pTuyau) != GetInputConnexion()->m_LstTuyAssAm.end())
            {
                setLinks.push_back(pTuyau);
            }
        }
        if (!setLinks.empty())
        {
            result.insert(std::make_pair(pCon, setLinks));
        }
    }

    // il faut Ã©galement compter les noeud sans tuyau amont qui ne sont pas dÃ©tectÃ©s par la boucle prÃ©cÃ©dente (cas des entrÃ©es ponctuelles, typiquement)
    for (size_t iTuy = 0; iTuy < GetInputPosition().GetZone().size(); iTuy++)
    {
        pTuyau = GetInputPosition().GetZone()[iTuy];
        if (pTuyau->GetCnxAssAm()->m_LstTuyAssAm.empty())
        {
            result.insert(std::make_pair(pTuyau->GetCnxAssAm(), std::vector<Tuyau*>(1, NULL)));
        }
    }
    return result;
}

std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > ZoneDeTerminaison::GetOutputJunctions()
{
    std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > result;
    Tuyau * pTuyau;
    Connexion * pCon;
    for (size_t iTuy = 0; iTuy < GetOutputConnexion()->m_LstTuyAssAv.size(); iTuy++)
    {
        pCon = GetOutputConnexion()->m_LstTuyAssAv[iTuy]->GetCnxAssAm();
        std::vector<Tuyau*> setLinks;
        for (size_t iLink = 0; iLink < pCon->m_LstTuyAssAv.size(); iLink++)
        {
            pTuyau = pCon->m_LstTuyAssAv[iLink];

            // Le tuyau doit Ãªtre un tuyau de sortie de la zone :
            if (std::find(GetOutputConnexion()->m_LstTuyAssAv.begin(), GetOutputConnexion()->m_LstTuyAssAv.end(), pTuyau) != GetOutputConnexion()->m_LstTuyAssAv.end())
            {
                setLinks.push_back(pTuyau);
            }
        }
        if (!setLinks.empty())
        {
            result.insert(std::make_pair(pCon, setLinks));
        }
    }

    // il faut Ã©galement compter les noeud sans tuyau aval qui ne sont pas dÃ©tectÃ©s par la boucle prÃ©cÃ©dente (cas des sorties ponctuelles, typiquement)
    for (size_t iTuy = 0; iTuy < GetOutputPosition().GetZone().size(); iTuy++)
    {
        pTuyau = GetOutputPosition().GetZone()[iTuy];
        if (pTuyau->GetCnxAssAv()->m_LstTuyAssAv.empty())
        {
            result.insert(std::make_pair(pTuyau->GetCnxAssAv(), std::vector<Tuyau*>(1, NULL)));
        }
    }
    return result;
}

// Renvoie la liste des tuyaux avec capacitÃ© de stationnement non nulle
std::vector<Tuyau*> ZoneDeTerminaison::GetTuyauxStationnement()
{
    std::vector<Tuyau*> result;

    const std::vector<Tuyau*> & lstTuyaux = m_InputPosition.GetZone();

    for(size_t i = 0; i < lstTuyaux.size(); i++)
	{
        if(lstTuyaux[i]->GetCapaciteStationnement() > 0 || lstTuyaux[i]->GetCapaciteStationnement() == -1)
        {
            result.push_back(lstTuyaux[i]);
        }
    }

    return result;
}

bool ZoneDeTerminaison::TirageTypeDestinationResidentielle(CPlaque * pPlaque, double dbInstant)
{
    bool bResult = true;
    double dbRand = (double)m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;
    if (pPlaque)
    {
        bResult = dbRand < pPlaque->GetParkingParameters()->GetVariation(dbInstant)->GetResidentialRatioDestination();
    }
    else
    {
        bResult = dbRand < GetParkingParameters()->GetVariation(dbInstant)->GetResidentialRatioDestination();
    }
    return bResult;
}

bool ZoneDeTerminaison::CheckRadiusForOriginOK(const std::vector<Tuyau*> & path, Tuyau * pPreviousLink)
{
    // Calcul de la longueur de l'itinÃ©raire dans la brique pour respect du radius
    double dbLength = 0;
    if (!path.empty())
    {
        Tuyau * pPrevLinkFromPath = pPreviousLink;
        for (size_t iLink = 0; iLink < path.size(); iLink++)
        {
            Tuyau * pLinkFromPath = path[iLink];

            // On s'arrÃªte au premier tronÃ§on qui n'est pas dans la zone
            if (!GetOutputPosition().IsInZone(pLinkFromPath))
            {
                break;
            }

            dbLength += pLinkFromPath->GetLength();
            if (pPrevLinkFromPath && pLinkFromPath->GetBriqueAmont())
            {
                // On compte la longueur du mouvement interne le cas Ã©chÃ©ant
                std::vector<Tuyau*> tuyauxInternes;
                pLinkFromPath->GetBriqueAmont()->GetTuyauxInternes(pPrevLinkFromPath, pLinkFromPath, tuyauxInternes);
                for (size_t iInternalLink = 0; iInternalLink < tuyauxInternes.size(); iInternalLink++)
                {
                    dbLength += tuyauxInternes[iInternalLink]->GetLength();
                }
            }
            pPrevLinkFromPath = pLinkFromPath;
        }
    }
    return dbLength <= m_dbMaxDistanceToJunction;
}

bool ZoneDeTerminaison::CheckRadiusForDestinationOK(const std::vector<Tuyau*> & path)
{
    double dbLength = 0;
    if (!path.empty())
    {
        Tuyau * pPrevLinkFromPath = NULL;
        // rmq. : on ne prend pas en compte le dernier tronÃ§on dans le radius (son dÃ©but compte)
        for (std::vector<Tuyau*>::const_reverse_iterator iterLink = path.rbegin()+1; iterLink != path.rend(); ++iterLink)
        {
            Tuyau * pLinkFromPath = *iterLink;

            // On s'arrÃªte au premier tronÃ§on qui n'est pas dans la zone
            if (!GetInputPosition().IsInZone(pLinkFromPath))
            {
                break;
            }

            dbLength += pLinkFromPath->GetLength();
            if (pPrevLinkFromPath && pLinkFromPath->GetBriqueAval())
            {
                // On compte la longueur du mouvement interne le cas Ã©chÃ©ant
                std::vector<Tuyau*> tuyauxInternes;
                pLinkFromPath->GetBriqueAval()->GetTuyauxInternes(pLinkFromPath, pPrevLinkFromPath, tuyauxInternes);
                for (size_t iInternalLink = 0; iInternalLink < tuyauxInternes.size(); iInternalLink++)
                {
                    dbLength += tuyauxInternes[iInternalLink]->GetLength();
                }
            }
            pPrevLinkFromPath = pLinkFromPath;
        }
    }
    return dbLength <= m_dbMaxDistanceToJunction;
}

std::vector<Tuyau*> ZoneDeTerminaison::CheckRadiusForDestinationOK(TypeVehicule * pTypeVeh, Connexion * pJunction, const std::map<Tuyau*, std::vector<Tuyau*>, LessPtr<Tuyau> > & pCandidates)
{
    std::vector<Tuyau*> result;

    std::map<Tuyau*, bool> & linksInRadius = m_CacheLinksInRadius[pTypeVeh][pJunction];

    std::vector<Tuyau*> linksToCheck;
    // Pour chaque tuyau candidat :
    for (std::map<Tuyau*, std::vector<Tuyau*>, LessPtr<Tuyau> >::const_iterator iterCandidate = pCandidates.begin(); iterCandidate != pCandidates.end(); ++iterCandidate)
    {
        Tuyau * pCandidate = iterCandidate->first;

        std::map<Tuyau*, bool>::const_iterator iter = linksInRadius.find(pCandidate);
        if (iter != linksInRadius.end())
        {
            if (iter->second)
            {
                result.push_back(pCandidate);
            }
        }
        else
        {
            // Il va falloir faire le calcul pour celui-ci
            linksToCheck.push_back(pCandidate);
        }
    }

    // Calcul des itinÃ©raires d'un seul coup :
    std::vector<TripNode*> lstDestinations;
    for (size_t iLinkToCheck = 0; iLinkToCheck < linksToCheck.size(); iLinkToCheck++)
    {
        Tuyau * pLinkToCheck = linksToCheck.at(iLinkToCheck);
        TronconDestination * pTDest;
        std::map<std::pair<Tuyau*, double>, TronconDestination*>::iterator iter = m_CacheTronconsDestination.find(make_pair(pLinkToCheck, UNDEF_LINK_POSITION));
        if (iter != m_CacheTronconsDestination.end())
        {
            pTDest = iter->second;
        }
        else
        {
            pTDest = new TronconDestination(pLinkToCheck);
            m_CacheTronconsDestination[make_pair(pLinkToCheck, UNDEF_LINK_POSITION)] = pTDest;
        }
        lstDestinations.push_back(pTDest);
    }

    if (!lstDestinations.empty())
    {
        std::vector<std::vector<PathResult> > newPathsForDestinations;
        std::vector<std::map<std::vector<Tuyau*>, double> > placeHolder;

        m_pNetwork->GetSymuScript()->ComputePaths(pJunction, lstDestinations, pTypeVeh, m_pNetwork->m_dbInstSimu, 1, NULL, newPathsForDestinations, placeHolder, false);

        for (size_t iLinkToCheck = 0; iLinkToCheck < linksToCheck.size(); iLinkToCheck++)
        {
            const std::vector<PathResult> & newPaths = newPathsForDestinations.at(iLinkToCheck);

            bool bOk = false;
            if (newPaths.size() == 1)
            {
                const PathResult & path = newPaths.front();
                Tuyau * pPrevLinkFromPath = NULL;
                double dbLength = 0;
                if (!path.links.empty())
                {
                    // rmq. : -1 car on ne considÃ¨re pas le tronÃ§on de destination par cohÃ©rence avec les autres tests de radius
                    for (size_t iLink = 0; iLink < path.links.size() - 1; iLink++)
                    {
                        Tuyau * pLinkFromPath = path.links[iLink];

                        dbLength += pLinkFromPath->GetLength();
                        if (pPrevLinkFromPath && pLinkFromPath->GetBriqueAmont())
                        {
                            // On compte la longueur du mouvement interne le cas Ã©chÃ©ant
                            std::vector<Tuyau*> tuyauxInternes;
                            pLinkFromPath->GetBriqueAmont()->GetTuyauxInternes(pPrevLinkFromPath, pLinkFromPath, tuyauxInternes);
                            for (size_t iInternalLink = 0; iInternalLink < tuyauxInternes.size(); iInternalLink++)
                            {
                                dbLength += tuyauxInternes[iInternalLink]->GetLength();
                            }
                        }
                        pPrevLinkFromPath = pLinkFromPath;
                    }
                }

                bOk = dbLength <= m_dbMaxDistanceToJunction;
            }

            Tuyau * pCandidate = linksToCheck.at(iLinkToCheck);
            if (bOk)
            {
                result.push_back(pCandidate);
            }

            // Mise dans le cache pour Ã©viter le recalcul plus tard (ce cache est-il vraiment encore utile depuis qu'on travaille d'un coup pour tous les candidats ???)
            linksInRadius[pCandidate] = bOk;
        }
    }

    return result;
}

const std::vector<Tuyau*> & ZoneDeTerminaison::GetAccessibleLinksToStart(CPlaque * pPlaque, bool bPerimetreElargi, TypeVehicule * pTypeVeh, SymuViaTripNode * pJunctionLinkOrDestination)
{
    std::map<SymuViaTripNode*, std::vector<Tuyau*> >::const_iterator iter = m_CacheAvailableOriginLinks[pTypeVeh][pPlaque][bPerimetreElargi].find(pJunctionLinkOrDestination);
    if (iter != m_CacheAvailableOriginLinks[pTypeVeh][pPlaque][bPerimetreElargi].end())
    {
        // le calcul est dÃ©jÃ  fait :
        return iter->second;
    }
    else
    {
        // Calcul si on n'a pas dÃ©jÃ  la rÃ©ponse dans le cache :
        std::vector<Tuyau*> accessibleLinks;

        const std::vector<Tuyau*> & candidateLinks = GetOutputPosition().GetZone();

        TronconDestination * pLinkDest = dynamic_cast<TronconDestination*>(pJunctionLinkOrDestination);

        // Liste des origines possibles :
        std::vector<std::pair<Connexion*, Tuyau*> > lstCandidateOrigins;
        std::map<Tuyau*, std::vector<Tuyau*>, LessPtr<Tuyau> > lstPotentialResult;

        for (size_t iCandidate = 0; iCandidate < candidateLinks.size(); iCandidate++)
        {
            Tuyau * pCandidate = candidateLinks.at(iCandidate);

            if (pCandidate->IsInterdit(pTypeVeh))
            {
                continue;
            }

            // mode plaque non Ã©largi : on ignore les tuyaux pas dans la plaque
            if (pPlaque && !pPlaque->HasLink(pCandidate) && !bPerimetreElargi)
            {
                continue;
            }

            if (pLinkDest && pCandidate == pLinkDest->GetTuyau())
            {
                lstPotentialResult[pCandidate] = std::vector<Tuyau*>();
            }
            else
            {
                lstCandidateOrigins.push_back(std::make_pair(pCandidate->GetCnxAssAv(), pCandidate));
            }
        }

        if (!lstCandidateOrigins.empty())
        {
            if (pJunctionLinkOrDestination == NULL)
            {
                // cas particulier origine = destination = zone (chemin interne Ã  la zone)
                // Pas besoin de calcul un itinÃ©raire, donc :
                for (size_t iCandidate = 0; iCandidate < lstCandidateOrigins.size(); iCandidate++)
                {
                    const std::pair<Connexion*, Tuyau*> & myOrigin = lstCandidateOrigins.at(iCandidate);
                    lstPotentialResult[myOrigin.second];
                }
            }
            else
            {
                std::vector<std::vector<PathResult> > newPaths;
                std::vector<std::map<std::vector<Tuyau*>, double> > placeHolder;
                m_pNetwork->GetSymuScript()->ComputePaths(lstCandidateOrigins, pJunctionLinkOrDestination, pTypeVeh, m_pNetwork->m_dbInstSimu, 1, newPaths, placeHolder, false);

                // Test pour les itinÃ©raires non Ã©vidents
                for (size_t iCandidate = 0; iCandidate < lstCandidateOrigins.size(); iCandidate++)
                {
                    const std::vector<PathResult> & paths = newPaths.at(iCandidate);

                    if (paths.size() == 1)
                    {
                        // le tuyau est accessible 
                        const std::pair<Connexion*, Tuyau*> & myOrigin = lstCandidateOrigins.at(iCandidate);
                        lstPotentialResult[myOrigin.second] = paths.front().links;
                    }
                }
            }
        }

        // DerniÃ¨res vÃ©rifications
        for (std::map<Tuyau*, std::vector<Tuyau*>, LessPtr<Tuyau> >::const_iterator iterPotential = lstPotentialResult.begin(); iterPotential != lstPotentialResult.end(); ++iterPotential)
        {
            Tuyau * pCandidate = iterPotential->first;

            // vÃ©rification du respect du radius si on n'est pas en mode de recherche Ã©largie pour une zone :
            if (!bPerimetreElargi && !pPlaque)
            {
                if (!CheckRadiusForOriginOK(iterPotential->second, pCandidate))
                {
                    continue;
                }
            }

            // en mode plaque Ã©largi, on vÃ©rifie que le tuyau est dans la plaque ou dans un radius autour du centre de la plaque
            if (pPlaque && bPerimetreElargi)
            {
                if (!pPlaque->CheckRadiusOK(pCandidate))
                {
                    continue;
                }
            }

            accessibleLinks.push_back(pCandidate);
        }
        std::pair<std::map<SymuViaTripNode*, std::vector<Tuyau*> >::iterator, bool> iterNewValue = m_CacheAvailableOriginLinks[pTypeVeh][pPlaque][bPerimetreElargi].insert(std::make_pair(pJunctionLinkOrDestination, accessibleLinks));
        assert(iterNewValue.second == true); // la valeur ne peut pas Ãªtre dÃ©jÃ  prÃ©sente...
        return iterNewValue.first->second;
    }
}

// RÃ©cupÃ¨re la liste des tronÃ§ons accessibles depuis l'origine passÃ© en paramÃ¨tre (soit origine, soit couple de tuyau courant + prÃ©cÃ©dent)
const std::vector<Tuyau*> & ZoneDeTerminaison::GetAccessibleLinksToEnd(CPlaque * pPlaque, bool bPerimetreElargi, TypeVehicule * pTypeVeh, Connexion * pJunction, SymuViaTripNode * pOrigin, Tuyau * pCurrentTuy, Tuyau * pPrevTuyau)
{
    if (pOrigin)
    {
        std::map<SymuViaTripNode*, std::vector<Tuyau*> > & cacheAvailablelinks = m_CacheAvailableDestinationLinksFromOrigin[pTypeVeh][pPlaque][bPerimetreElargi];

        std::map<SymuViaTripNode*, std::vector<Tuyau*> >::const_iterator iter = cacheAvailablelinks.find(pOrigin);
        if (iter != cacheAvailablelinks.end())
        {
            // le calcul est dÃ©jÃ  fait :
            return iter->second;
        }
    }
    else
    {
        if (!bPerimetreElargi)
        {
            std::map<std::pair<Tuyau*, Tuyau*>, std::vector<Tuyau*> > & cacheAvailablelinks = m_CacheAvailableDestinationLinksFromLink[pTypeVeh][pPlaque][pJunction];
            std::map<std::pair<Tuyau*, Tuyau*>, std::vector<Tuyau*> >::const_iterator iter = cacheAvailablelinks.find(std::make_pair(pCurrentTuy, pPrevTuyau));
            if (iter != cacheAvailablelinks.end())
            {
                // le calcul est dÃ©jÃ  fait :
                return iter->second;
            }
        }
        else
        {
            std::map<std::pair<Tuyau*, Tuyau*>, std::vector<Tuyau*> > & cacheAvailablelinks = m_CacheExtendedAvailableDestinationLinksFromLink[pTypeVeh][pPlaque];
            std::map<std::pair<Tuyau*, Tuyau*>, std::vector<Tuyau*> >::const_iterator iter = cacheAvailablelinks.find(std::make_pair(pCurrentTuy, pPrevTuyau));
            if (iter != cacheAvailablelinks.end())
            {
                // le calcul est dÃ©jÃ  fait :
                return iter->second;
            }
        }
    }

    // Calcul si on n'a pas dÃ©jÃ  la rÃ©ponse dans le cache :
    std::vector<Tuyau*> accessibleLinks;

    const std::vector<Tuyau*> & candidateLinks = GetInputPosition().GetZone();

    // Liste des destinations possibles :
    std::vector<TripNode*> lstCandidateDestinations;
    std::map<Tuyau*, std::vector<Tuyau*>, LessPtr<Tuyau> > lstPotentialResult;

    for (size_t iCandidate = 0; iCandidate < candidateLinks.size(); iCandidate++)
    {
        Tuyau * pCandidate = candidateLinks.at(iCandidate);

        if (pCandidate->IsInterdit(pTypeVeh))
        {
            continue;
        }

        // mode plaque non Ã©largi : on ignore les tuyaux pas dans la plaque
        if (pPlaque && !pPlaque->HasLink(pCandidate) && !bPerimetreElargi)
        {
            continue;
        }

        TronconDestination * pTDest;
        std::map<std::pair<Tuyau*, double>, TronconDestination*>::iterator iter = m_CacheTronconsDestination.find(make_pair(pCandidate, UNDEF_LINK_POSITION));
        if (iter != m_CacheTronconsDestination.end())
        {
            pTDest = iter->second;
        }
        else
        {
            pTDest = new TronconDestination(pCandidate);
            m_CacheTronconsDestination[make_pair(pCandidate, UNDEF_LINK_POSITION)] = pTDest;
        }

        if (!pOrigin && pCandidate == pCurrentTuy)
        {
            lstPotentialResult[pCandidate] = std::vector<Tuyau*>();
        }
        else
        {
            lstCandidateDestinations.push_back(pTDest);
        }
    }

    if (!lstCandidateDestinations.empty())
    {
        std::vector<std::vector<PathResult> > newPaths;
        std::vector<std::map<std::vector<Tuyau*>, double> > placeHolder;

        if (pOrigin)
        {
            m_pNetwork->GetSymuScript()->ComputePaths(pOrigin, lstCandidateDestinations, pTypeVeh, m_pNetwork->m_dbInstSimu, 1, newPaths, placeHolder, false);
        }
        else
        {
            m_pNetwork->GetSymuScript()->ComputePaths(pCurrentTuy->GetBriqueParente() ? pCurrentTuy->GetBriqueParente() : pCurrentTuy->GetCnxAssAv(), lstCandidateDestinations, pTypeVeh, m_pNetwork->m_dbInstSimu, 1, pCurrentTuy->GetBriqueParente() ? pPrevTuyau : pCurrentTuy, newPaths, placeHolder, false);
        }

        // Test pour les itinÃ©raires non Ã©vidents
        for (size_t iCandidate = 0; iCandidate < lstCandidateDestinations.size(); iCandidate++)
        {
            const std::vector<PathResult> & paths = newPaths.at(iCandidate);

            if (paths.size() == 1)
            {
                // le tuyau est accessible 
                lstPotentialResult[lstCandidateDestinations.at(iCandidate)->GetInputPosition().GetLink()] = paths.front().links;
            }
        }
    }

    // DerniÃ¨res vÃ©rifications :

    // en mode plaque Ã©largi, on vÃ©rifie que le tuyau est dans la plaque ou dans un radius autour du centre de la plaque
    if (pPlaque && bPerimetreElargi)
    {
        for (std::map<Tuyau*, std::vector<Tuyau*>, LessPtr<Tuyau> >::const_iterator iterPotential = lstPotentialResult.begin(); iterPotential != lstPotentialResult.end(); ++iterPotential)
        {
            Tuyau * pCandidate = iterPotential->first;

            if (pPlaque->CheckRadiusOK(pCandidate))
            {
                accessibleLinks.push_back(pCandidate);
            }
        }
    }
    // vÃ©rification du respect du radius si on n'est pas en mode de recherche Ã©largie pour une zone :
    else if (!bPerimetreElargi && !pPlaque)
    {
        if (pOrigin)
        {
            for (std::map<Tuyau*, std::vector<Tuyau*>, LessPtr<Tuyau> >::const_iterator iterPotential = lstPotentialResult.begin(); iterPotential != lstPotentialResult.end(); ++iterPotential)
            {
                if (CheckRadiusForDestinationOK(iterPotential->second))
                {
                    accessibleLinks.push_back(iterPotential->first);
                }
            }
        }
        else
        {
            // Dans ce cas, on teste l'appartenance du tronÃ§on candidat aux tronÃ§ons dans le radius du point de jonction utilisÃ© par le vÃ©hicule
            // qui peut Ãªtre diffÃ©rent du couple de tronÃ§on courant du vÃ©hicule !!!
            accessibleLinks = CheckRadiusForDestinationOK(pTypeVeh, pJunction, lstPotentialResult);
        }
    }
    else
    {
        for (std::map<Tuyau*, std::vector<Tuyau*>, LessPtr<Tuyau> >::const_iterator iterPotential = lstPotentialResult.begin(); iterPotential != lstPotentialResult.end(); ++iterPotential)
        {
            accessibleLinks.push_back(iterPotential->first);
        }
    }

    if (pOrigin)
    {
        std::map<SymuViaTripNode*, std::vector<Tuyau*> > & cacheAvailablelinks = m_CacheAvailableDestinationLinksFromOrigin[pTypeVeh][pPlaque][bPerimetreElargi];
        std::pair<std::map<SymuViaTripNode*, std::vector<Tuyau*> >::iterator, bool> iterNewValue = cacheAvailablelinks.insert(std::make_pair(pOrigin, accessibleLinks));
        assert(iterNewValue.second == true); // la valeur ne peut pas Ãªtre dÃ©jÃ  prÃ©sente...
        return iterNewValue.first->second;
    }
    else
    {
        if (bPerimetreElargi)
        {
            std::map<std::pair<Tuyau*, Tuyau*>, std::vector<Tuyau*> > & cacheAvailablelinks = m_CacheExtendedAvailableDestinationLinksFromLink[pTypeVeh][pPlaque];
            std::pair<std::map<std::pair<Tuyau*, Tuyau*>, std::vector<Tuyau*> >::iterator, bool> iterNewValue = cacheAvailablelinks.insert(std::make_pair(std::make_pair(pCurrentTuy, pPrevTuyau), accessibleLinks));
            assert(iterNewValue.second == true); // la valeur ne peut pas Ãªtre dÃ©jÃ  prÃ©sente...
            return iterNewValue.first->second;
        }
        else
        {
            std::map<std::pair<Tuyau*, Tuyau*>, std::vector<Tuyau*> > & cacheAvailablelinks = m_CacheAvailableDestinationLinksFromLink[pTypeVeh][pPlaque][pJunction];
            std::pair<std::map<std::pair<Tuyau*, Tuyau*>, std::vector<Tuyau*> >::iterator, bool> iterNewValue = cacheAvailablelinks.insert(std::make_pair(std::make_pair(pCurrentTuy, pPrevTuyau), accessibleLinks));
            assert(iterNewValue.second == true); // la valeur ne peut pas Ãªtre dÃ©jÃ  prÃ©sente...
            return iterNewValue.first->second;
        }
    }
}


bool ZoneDeTerminaison::TirageTypeParkingSurfaciqueOrigin(double dbInstant, CPlaque * pPlaque, bool bPerimetreElargi, TypeVehicule * pTypeVeh, SymuViaTripNode * pJunctionLink, bool & bFound)
{
    const std::vector<Tuyau*> & accessibleLinks = GetAccessibleLinksToStart(pPlaque, bPerimetreElargi, pTypeVeh, pJunctionLink);

    bool bHasSurfacicStock, bHasHSStock, bHasSurfacicRoom, bHasHSRoom;
    double dbTauxSurfacique = GetTauxOccupationSurfacique(accessibleLinks, bHasSurfacicStock, bHasSurfacicRoom);
    double dbTauxHS = GetTauxOccupationHorsSol(accessibleLinks, pTypeVeh, bHasHSStock, bHasHSRoom);

    double dbProbaSurfacique;
    double dbDefaultProbaSurfacique;
    if (pPlaque)
    {
        dbDefaultProbaSurfacique = pPlaque->GetParkingParameters()->GetVariation(dbInstant)->GetSurfacicProportionOrigin();
    }
    else
    {
        dbDefaultProbaSurfacique = GetParkingParameters()->GetVariation(dbInstant)->GetSurfacicProportionOrigin();
    }
    
    if (!bHasHSStock)
    {
        // Si pas de parking hors sol, on fait du surfacique (mÃªme s'il n'y en a pas, on pourra au pire prendre un tuyau alÃ©atoirement mÃªme sans stock si l'option est activÃ©e pour respecter la demande)
        dbProbaSurfacique = 1;
    }
    else if (!bHasSurfacicStock)
    {
        dbProbaSurfacique = 0;
    }
    else
    {
        dbProbaSurfacique = dbDefaultProbaSurfacique;
    }

    double dbRand = (double)m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

    bool bIsSurfacique = dbRand < dbProbaSurfacique;

    if (bIsSurfacique)
    {
        bFound = bHasSurfacicStock;
    }
    else
    {
        bFound = bHasHSStock;
    }

    return bIsSurfacique;
}

// Tire le type de stationnement entre surfacique et hors-sol en fonction des taux d'occupation et des couts pour l'arrivÃ©e en zone.
bool ZoneDeTerminaison::TirageTypeParkingSurfaciqueDestination(double dbInstant, CPlaque * pPlaque, bool bPerimetreElargi, TypeVehicule * pTypeVeh, Connexion * pJunction, Tuyau * pCurrentTuy, Tuyau * pPrevTuyau)
{
    const std::vector<Tuyau*> & accessibleLinks = GetAccessibleLinksToEnd(pPlaque, bPerimetreElargi, pTypeVeh, pJunction, NULL, pCurrentTuy, pPrevTuyau);

    bool bHasSurfacicStock, bHasHSStock, bHasSurfacicRoom, bHasHSRoom;
    double dbTauxSurfacique = GetTauxOccupationSurfacique(accessibleLinks, bHasSurfacicStock, bHasSurfacicRoom);
    double dbTauxHS = GetTauxOccupationHorsSol(accessibleLinks, pTypeVeh, bHasHSStock, bHasHSRoom);

    double dbProbaSurfacique;
    
    if (!bHasHSRoom)
    {
        // Si pas de parking hors sol, on fait du surfacique (mÃªme s'il n'y en a pas, on pourra au pire prendre un tuyau alÃ©atoirement mÃªme sans stock si l'option est activÃ©e pour respecter la demande)
        dbProbaSurfacique = 1;
    }
    else if (!bHasSurfacicRoom)
    {
        dbProbaSurfacique = 0;
    }
    else
    {
        ParkingParameters * pParkingParams = pPlaque ? pPlaque->GetParkingParameters() : GetParkingParameters();
        ParkingParametersVariation * pVariation = pParkingParams->GetVariation(dbInstant);
        if (!pVariation->UseCosts())
        {
            dbProbaSurfacique = pVariation->GetSurfacicProportionDestination();
        }
        else
        {
            double dbRelativeParkingCostRatio = pVariation->GetCostRatio();
            double dbCostSurfacique = pVariation->GetSurfacicCost();
            double dbCostHS = pVariation->GetHSCost();


            double dbPriceCoeffSurfacique, dbPriceCoeffHS;

            // gestion de la division par 0 : si cout nul pour le hors sol, on ne fait pas de surfacique...
            if (dbRelativeParkingCostRatio == 0 || dbCostHS == 0)
            {
                if (dbCostSurfacique != 0)
                {
                    // parking HS gratuit et surfacique payant : tout va vers le hors sol tant qu'on y a de la place
                    return false;
                }
                else
                {
                    // coÃ»ts nuls pour les deux types de parking : le cout n'intervient plus
                    dbPriceCoeffSurfacique = 1;
                    dbPriceCoeffHS = 1;
                }
            }
            else
            {
                dbPriceCoeffHS = dbCostSurfacique / (dbRelativeParkingCostRatio * dbCostHS);
                dbPriceCoeffSurfacique = 1.0 - dbPriceCoeffHS;
            }

            double dbNumerator = dbPriceCoeffSurfacique * (1.0 - dbTauxSurfacique);
            double dbDenom = dbNumerator + dbPriceCoeffHS* (1.0 - dbTauxHS);

            if (dbDenom == 0)
            {
                if (dbNumerator == 0)
                {
                    // je ne sais pas arbitrer : 50% de chance pour chaque type de parking
                    dbProbaSurfacique = 0.5;
                }
                else
                {
                    dbProbaSurfacique = dbNumerator > 0 ? 1 : 0;
                }
            }
            else
            {
                dbProbaSurfacique = dbNumerator / dbDenom;
            }
        }
    }

    double dbRand = (double)m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

    bool bIsSurfacique = dbRand < dbProbaSurfacique;

    return bIsSurfacique;
}

double ZoneDeTerminaison::GetTauxOccupationSurfacique(const std::vector<Tuyau*> & links, bool & bhasSurfacicStock, bool & bHasSurfacicRoom)
{
    double dbCurrentStock = 0;
    double dbMaxStock = 0;
    bool bHasInfiniteCapacity = false;
    for (size_t iLink = 0; iLink < links.size(); iLink++)
    {
        Tuyau * pLink = links[iLink];

        // capacitÃ© infinie : taux d'occupation nul
        if (pLink->GetCapaciteStationnement() == -1)
        {
            bHasInfiniteCapacity = true;
        }

        dbCurrentStock += pLink->GetStockStationnement();
        dbMaxStock += pLink->GetCapaciteStationnement();
    }

    bhasSurfacicStock = dbCurrentStock > 0;
    bHasSurfacicRoom = bHasInfiniteCapacity || dbCurrentStock < dbMaxStock;

    // On ne peut pas exploiter le calcul car stock infini = capacitÃ© Ã  -1 : taux de remplissage nul dans ce cas particulier
    if (bHasInfiniteCapacity)
    {
        return 0;
    }

    double dbTauxSurfacique;
    if (dbMaxStock > 0)
    {
        dbTauxSurfacique = dbCurrentStock / dbMaxStock;
    }
    else
    {
        // pour Ãªtre sÃ»r que cette valeur ne soit pas utilisÃ©e, on positionne bhasSurfacicStock Ã  false
        bhasSurfacicStock = false;
        bHasSurfacicRoom = false;
        dbTauxSurfacique = 0;
    }
    return dbTauxSurfacique;
}

double ZoneDeTerminaison::GetTauxOccupationHorsSol(const std::vector<Tuyau*> & links, TypeVehicule * pTypeVeh, bool & bHasHSStock, bool & bHasHSRoom)
{
    int iCurrentStock = 0;
    int iMaxStock = 0;
    const std::set<Parking*> & setParkings = GetParkingsFromLinks(links, pTypeVeh);

    bool bHasInfiniteCapacity = false;
    for (std::set<Parking*>::const_iterator iterPark = setParkings.begin(); iterPark != setParkings.end(); ++iterPark)
    {
        // capacitÃ© infinie : taux d'occupation nul
        if ((*iterPark)->GetStockMax() == -1)
        {
            bHasInfiniteCapacity = true;
        }

        iCurrentStock += (*iterPark)->GetStock();
        iMaxStock += (*iterPark)->GetStockMax();
    }

    bHasHSStock = iCurrentStock > 0;
    bHasHSRoom = bHasInfiniteCapacity || iCurrentStock < iMaxStock;

    // On ne peut pas exploiter le calcul car stock infini = capacitÃ© Ã  -1 : taux de remplissage nul dans ce cas particulier
    if (bHasInfiniteCapacity)
    {
        return 0;
    }

    double dbTauxHorsSol;
    if (iMaxStock > 0)
    {
        dbTauxHorsSol = (double)iCurrentStock / iMaxStock;
    }
    else
    {
        // pour Ãªtre sÃ»r que cette valeur ne soit pas utilisÃ©e, on positionne bhasSurfacicStock Ã  false
        bHasHSStock = false;
        bHasHSRoom = false;
        dbTauxHorsSol = 0;
    }
    return dbTauxHorsSol;
}

std::set<Parking*> ZoneDeTerminaison::GetParkingsFromLinks(const std::vector<Tuyau*> & links, TypeVehicule * pTypeVehicule)
{
    std::set<Parking*> result;
    for (size_t iPark = 0; iPark < m_LstParkings.size(); iPark++)
    {
        Parking * pPark = m_LstParkings.at(iPark);
        if (pPark->IsAllowedVehiculeType(pTypeVehicule))
        {
            for (size_t iLink = 0; iLink < links.size(); iLink++)
            {
                Tuyau * pLink = links[iLink];
                if (pLink->GetCnxAssAm() == pPark->GetOutputConnexion() || pLink->GetCnxAssAv() == pPark->GetInputConnexion())
                {
                    result.insert(pPark);
                    break;
                }
            }
        }
    }

    return result;
}


bool ZoneDeTerminaison::TirageTypeOrigineResidentielle(CPlaque * pPlaque, double dbInstant)
{
    bool bResult = true;
    double dbRand = (double)m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;
    if (pPlaque)
    {
       bResult = dbRand <  pPlaque->GetParkingParameters()->GetVariation(dbInstant)->GetResidentialRatioOrigin();
    }
    else
    {
        bResult = dbRand < GetParkingParameters()->GetVariation(dbInstant)->GetResidentialRatioOrigin();
    }
    return bResult;
}

// Traite le cas du vÃ©hicule qui entre dans sa zone de destination
void ZoneDeTerminaison::ReaffecteVehicule(Vehicule * pVehicule, double dbInstant)
{
    SymuViaFleetParameters * pParams = (SymuViaFleetParameters*)pVehicule->GetFleetParameters();

    // Passage du vÃ©hicule en mode terminaison de trajet
    pParams->SetIsTerminaison(true);

    // Tirage de la plaque de destination
    CPlaque * pPlaque = NULL;
    Tuyau * pCurrentTuy = NULL;
    Tuyau * pPrevTuyau = NULL;
    TiragePlaqueDestination(dbInstant, pParams->GetMotif(), pVehicule, NULL, pVehicule->GetType(), pPlaque, pCurrentTuy, pPrevTuyau);

    // Pas de plaque : on traite le cas de la zone :
    if (!pPlaque)
    {
        bool bIsResidentiel = true;
        bool bIsParkingSurfacique = true;
        TiragesStationnementDestination(dbInstant, pPlaque, pVehicule->GetType(), pParams->GetZoneDestinationInputJunction(), pCurrentTuy, pPrevTuyau, false, bIsResidentiel, bIsParkingSurfacique);
        pParams->SetIsTerminaisonResidentielle(bIsResidentiel);
        pParams->SetIsTerminaisonSurfacique(bIsParkingSurfacique);

        if (!bIsParkingSurfacique)
        {
            // Cas du tirage d'un parking hors sol. Si on a pu tirer le cas parking non surfacique, c'est qu'on a forcÃ©ment de la palce libre dedans, donc ne peut Ã©chouer
            Parking * pParkingDestination = TirageParkingDestination(NULL, pVehicule->GetType(), false, pParams->GetZoneDestinationInputJunction(), NULL, pCurrentTuy, pPrevTuyau);

            if (!AddFinalPathToParking(pVehicule, pParkingDestination, pParams->GetZoneDestinationInputJunction(), pCurrentTuy, pPrevTuyau, pVehicule->GetType(), dbInstant))
            {
                m_pNetwork->log() << Logger::Error << "Unable to find a path to the supposedly accessible parking " << pParkingDestination->GetID() << " as a destination in the zone " << GetID() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
        else
        {
            if (!AddFinalPathToSection(pVehicule, dbInstant, NULL, pVehicule->GetType(), NULL, pCurrentTuy, pPrevTuyau, false))
            {
                m_pNetwork->log() << Logger::Error << "Unable to find a path to a supposeldy accessible section as a destination in the zone " << GetID() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
    }

    // Si mode simplifiÃ©, calcul de la distance Ã  parcourir avant de pouvoir se garer :
    if (IsModeSurfaciqueSimplifie(pPlaque, dbInstant) && !pParams->IsTerminaisonResidentielle() && pParams->IsTerminaisonSurfacique())
    {
        UpdateGoalDistance(dbInstant, pParams, pVehicule->GetType(), pCurrentTuy, pPrevTuyau);
    }
}

bool isBriqueAmontZone(Tuyau * pTuyau, void * pArg)
{
    BriqueDeConnexion * pBrique = (BriqueDeConnexion*)pArg;
    return pTuyau->GetBriqueAmont() == pBrique;
}

// Traite le rÃ©arbitrage d'un vÃ©hicule vers un stationnement de destination
void ZoneDeTerminaison::ReaffecteVehiculeStationnementDestination(Vehicule * pVehicule, double dbInstant, bool bRearbitrage)
{
    SymuViaFleetParameters * pParams = (SymuViaFleetParameters*)pVehicule->GetFleetParameters();

    Tuyau * pCurrentTuy = NULL;
    Tuyau * pPrevTuyau = NULL;

    // DÃ©termination du tuyau initial du vÃ©hicule
    pCurrentTuy = pVehicule->GetLink(0) != NULL ? pVehicule->GetLink(0) : pVehicule->GetLink(1); // pour avoir une fonction valable en dÃ©but et fin de pas de temps
    if (pCurrentTuy->GetBriqueParente())
    {
        // On prend le tuyau qui suit la brique dans l'itinÃ©raire
        int iTuyIdx = pVehicule->GetItineraireIndex(isBriqueAmontZone, pCurrentTuy->GetBriqueParente());
        if (iTuyIdx != -1)
        {
            pCurrentTuy = pVehicule->GetItineraire()->at(iTuyIdx);
            if (iTuyIdx > 0)
            {
                pPrevTuyau = pVehicule->GetItineraire()->at(iTuyIdx - 1);
            }
        }
        else
        {
            // Le vÃ©hicule est dÃ©jÃ  en bout d'itinÃ©raire dans la brique aval (peut se produire si tronÃ§on court) :
            pPrevTuyau = pVehicule->GetItineraire()->back();
        }
    }

    if (bRearbitrage)
    {
        bool bIsParkingSurfacique = TirageTypeParkingSurfaciqueDestination(dbInstant, pParams->GetPlaqueDestination(), pParams->IsPerimetreElargi(dbInstant, this), pVehicule->GetType(), pParams->GetZoneDestinationInputJunction(), pCurrentTuy, pPrevTuyau);
        pParams->SetIsTerminaisonSurfacique(bIsParkingSurfacique);

        // Si mode simplifiÃ©, calcul de la distance Ã  parcourir avant de pouvoir se garer :
        if (IsModeSurfaciqueSimplifie(pParams->GetPlaqueDestination(), dbInstant) && pParams->IsTerminaisonSurfacique())
        {
            assert(!pParams->IsTerminaisonResidentielle());
            UpdateGoalDistance(dbInstant, pParams, pVehicule->GetType(), pCurrentTuy, pPrevTuyau);
        }
    }

    if (!pParams->IsTerminaisonSurfacique())
    {
        // Cas du tirage d'un parking hors sol. Si on a pu tirer le cas parking non surfacique, c'est qu'on a forcÃ©ment de la palce libre dedans, donc ne peut Ã©chouer
        Parking * pParkingDestination = TirageParkingDestination(pParams->GetPlaqueDestination(), pVehicule->GetType(), pParams->IsPerimetreElargi(dbInstant, this), pParams->GetZoneDestinationInputJunction(), NULL, pCurrentTuy, pPrevTuyau);

        if (!AddFinalPathToParking(pVehicule, pParkingDestination, pParams->GetZoneDestinationInputJunction(), pCurrentTuy, pPrevTuyau, pVehicule->GetType(), dbInstant))
        {
            m_pNetwork->log() << Logger::Error << "Unable to find a path to the supposedly accessible parking " << pParkingDestination->GetID() << " ..." << std::endl;
            m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
        }
    }
    else
    {
        if (!AddFinalPathToSection(pVehicule, dbInstant, pParams->GetPlaqueDestination(), pVehicule->GetType(), NULL, pCurrentTuy, pPrevTuyau, pParams->IsPerimetreElargi(dbInstant, this)))
        {
            m_pNetwork->log() << Logger::Error << "Unable to find a path to a supposedly accessible section ..." << std::endl;
            m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
        }
    }
}


// Sorties Ã  produire pour la zone
void ZoneDeTerminaison::SortieTrafic(DocTrafic *pXMLDocTrafic)
{
    std::vector<Tuyau*> lstTuyauxStationnement = GetTuyauxStationnement();
	for(size_t nTuy = 0; nTuy < lstTuyauxStationnement.size(); nTuy++)	
	{
        pXMLDocTrafic->AddSimuTronconStationnement(lstTuyauxStationnement[nTuy]->GetLabel(), lstTuyauxStationnement[nTuy]->GetStockStationnement());
	}
}

// GÃ¨re l'incrÃ©mentation du stock de vÃ©hicules stationnÃ©s en mode simplifiÃ©
void ZoneDeTerminaison::IncStockStationnement(Vehicule * pVehicle)
{
    SymuViaFleetParameters * pParams = (SymuViaFleetParameters*)pVehicle->GetFleetParameters();

    const std::vector<Tuyau*> & accessibleLinks = pParams->GetAccessibleLinksForParking();

    bool bFound = false;
    for (size_t iLink = 0; iLink < accessibleLinks.size() && !bFound; iLink++)
    {
        Tuyau * pLink = accessibleLinks.at(iLink);
        if (pLink->GetCapaciteStationnement() == -1 || pLink->GetStockStationnement() < pLink->GetCapaciteStationnement())
        {
            pLink->IncStockStationnement(pVehicle->GetLength());
            bFound = true;
        }
    }

    if (!bFound)
    {
        m_pNetwork->log() << Logger::Warning << "No accessible link found for vehicle " << pVehicle->GetID() <<
            " in simplified surfacic parking mode : the zone must have become full since the distance to travel before parking computation. The stock won't be incremented and the vehicle destroyed" << std::endl;
        m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }
}


// OpÃ©rateurs de tri pour ces objets dans les maps utilisÃ©es ci-dessous afin d'avoir
// un ordre fixe non basÃ© sur l'adresse des objets et ainsi avoir un comportement reproductible Ã  l'identique
// d'une exÃ©cution sur l'autre
class LessPlaquePtr
{
public:
    bool operator()(const CPlaque *a, const CPlaque *b) const
    {
        return (a->GetID() < b->GetID());
    }
};

class LessSection
{
public:
    bool operator()(const CSection & a, const CSection & b) const
    {
        return (a.getTuyau()->GetLabel() < b.getTuyau()->GetLabel()
            || ((a.getTuyau()->GetLabel() == b.getTuyau()->GetLabel()) &&
            ((a.getStartPosition() < b.getStartPosition()) || ((a.getStartPosition() == b.getStartPosition()) && (a.getEndPosition() < b.getEndPosition())))));
    }
};

Tuyau * ZoneDeTerminaison::TiragePlaqueOrigine
(	double dbInstant, 
	CPlaque * pPlaqueOrigin, 
	TypeVehicule * pTypeVehicule, 
	CMotifCoeff * pMotifCoeff, 
	SymuViaTripNode * pDestination, // input: next symuvia trip node
	Tuyau * pTuy,					// downstream link
	TripLeg * pTripLeg, 
	bool bForceNonResidential, 
	double & dbCreationPosition, 
	CPlaque* & pPlaqueResult,		// output
	bool & bDontCreateVehicle, 
	Parking* & pParkingOrigin, 
	bool & bSortieStationnement
)
{
    bSortieStationnement = false;
    Tuyau * pResult = pTuy;

    pParkingOrigin = NULL;
    bDontCreateVehicle = false;
    pPlaqueResult = NULL;
    bool bDone = false;
    bool bPlaquesDefined = false;

    SymuViaTripNode * pMultiClassDestination = NULL;
    if (pDestination)
    {
        pMultiClassDestination = pDestination;
    }
    else if (pTuy)
    {
        TronconDestination * pTDest;
        std::map<std::pair<Tuyau*, double>, TronconDestination*>::iterator iter = m_CacheTronconsDestination.find(make_pair(pTuy, UNDEF_LINK_POSITION));
        if (iter != m_CacheTronconsDestination.end())
        {
            pTDest = iter->second;
        }
        else
        {
            pTDest = new TronconDestination(pTuy);
            m_CacheTronconsDestination[make_pair(pTuy, UNDEF_LINK_POSITION)] = pTDest;
        }
        pMultiClassDestination = pTDest;
    }
    // remarque : pMultiClassDestination peut rester NULL dans le cas particulier origine = destination = zone. Dans ce cas on va tirer un tuyau
    // sans contrainte sur l'itinÃ©raire pour aller au point de jonction dÃ©fini par pTuy.

    // Constitution de l'ensemble des plaques candidates (pour lesquelles on a un motif) avec le coefficient associÃ©
    std::map<CPlaque*, double, LessPlaquePtr> plaquesCandidates;
    if (pPlaqueOrigin)
    {
        // Cas d'une plaque de dÃ©part imposÃ©e par SymuMaster
        plaquesCandidates[pPlaqueOrigin] = 1;
        bPlaquesDefined = true;
    }
    else
    {
        // Cas classique de recherche d'une plaque
        if (pMotifCoeff)
        {
            double dbSumPlaquesCoeffs = 0;
            for (size_t i = 0; i < m_LstPlaques.size(); i++)
            {
                CPlaque * pPlaque = m_LstPlaques[i];
                double dbCoeff = pPlaque->GetMotifCoeff(pMotifCoeff->GetMotifOrigine());
                if (dbCoeff > 0)
                {
                    bPlaquesDefined = true;
                    if (!GetAccessibleLinksToStart(pPlaque, false, pTypeVehicule, pMultiClassDestination).empty())
                    {
                        plaquesCandidates[pPlaque] = dbCoeff;
                        dbSumPlaquesCoeffs += dbCoeff;
                    }
                }
            }
            // Ã©largissement du pÃ©rimÃ¨tre des plaques
            if (plaquesCandidates.empty())
            {
                dbSumPlaquesCoeffs = 0;
                for (size_t i = 0; i < m_LstPlaques.size(); i++)
                {
                    CPlaque * pPlaque = m_LstPlaques[i];
                    double dbCoeff = pPlaque->GetMotifCoeff(pMotifCoeff->GetMotifOrigine());
                    if (dbCoeff > 0)
                    {
                        if (!GetAccessibleLinksToStart(pPlaque, true, pTypeVehicule, pMultiClassDestination).empty())
                        {
                            plaquesCandidates[pPlaque] = dbCoeff;
                            dbSumPlaquesCoeffs += dbCoeff;
                        }
                    }
                }
            }
            // renormalisation des coefficients :
            for (std::map<CPlaque*, double, LessPlaquePtr>::iterator iterPlaque = plaquesCandidates.begin(); iterPlaque != plaquesCandidates.end(); ++iterPlaque)
            {
                iterPlaque->second /= dbSumPlaquesCoeffs;
            }
        }
    }

    if (!plaquesCandidates.empty())
    {
        // Tirage d'une plaque parmis les candidates
        ///////////////////////////////////////////////////////////////////////////
        CPlaque * pPlaque = NULL;
        double dbSum = 0;
        double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

        for (std::map<CPlaque*, double, LessPlaquePtr>::iterator iterPlaque = plaquesCandidates.begin(); iterPlaque != plaquesCandidates.end() && !pPlaque; ++iterPlaque)
        {
            dbSum += iterPlaque->second;

            if (dbRand < dbSum)
            {
                // on tient notre plaque
                pPlaque = iterPlaque->first;
            }
        }
        // protection contre les erreurs d'arrondi liÃ©s Ã  le renormalisation
        if (!pPlaque)
        {
            pPlaque = plaquesCandidates.rbegin()->first;
        }

        // Si on est en mode gÃ©nÃ©ration de demande pour SymuMaster, pTripLeg est nul, et on ne fait pas de tirage entre
        // stationnement surfacique / parking : ce sera fait pas SymuVia au moment de l'activation du vÃ©hicule
        bool bIsResidentiel = true;
        bool bIsParkingSurfacique = true;
        bool bPerimetreElargi = false;
        if (pTripLeg)
        {
            TiragesStationnementOrigin(dbInstant, pPlaque, pTypeVehicule, pMultiClassDestination, bForceNonResidential, bIsResidentiel, bIsParkingSurfacique, bPerimetreElargi, bDontCreateVehicle);
            if (bDontCreateVehicle)
            {
                return NULL;
            }
        }

        
        if (!bIsParkingSurfacique)
        {
            // Cas du tirage d'un parking hors sol. Si on a pu tirer le cas parking non surfacique, c'est qu'on a forcÃ©ment du stock dedans, donc ne peut Ã©chouer
            pParkingOrigin = TirageParkingOrigin(pPlaque, pTypeVehicule, bPerimetreElargi, pMultiClassDestination);

            if (AddInitialPathFromParking(pTripLeg, pParkingOrigin, pMultiClassDestination, pTypeVehicule, dbInstant))
            {
                bDone = true;
                pPlaqueResult = pPlaque;
                pResult = pTripLeg->GetPath().front();
            }
            else
            {
                m_pNetwork->log() << Logger::Error << "Unable to find a path from the supposedly accessible parking " << pParkingOrigin->GetID() << " departing from plaque " << pPlaque->GetID() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

                pParkingOrigin = NULL;
            }
        }
        else
        {
            Tuyau * pSectionLink = AddInitialPathFromSection(pTripLeg, dbInstant, pPlaque, pTypeVehicule, bPerimetreElargi, pMultiClassDestination, bIsResidentiel);
            if (pSectionLink)
            {
                bSortieStationnement = bIsResidentiel;
                bDone = true;
                pPlaqueResult = pPlaque;
                pResult = pSectionLink;

                // Tirage de la position sur la section
                //dbRand = RandManager::getInstance()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;
                //dbCreationPosition = dbRand * pSection->getLength() + pSection->getStartPosition();

                // Finalement on prend 1/3 du tronÃ§on pour que tout le monde s'insÃ¨re au mÃªme endroit et respecter la FIFO des vÃ©hicules en attente d'insertion
                // remarque - pas bon dans le cas des sections puisqu'on ne respecte pas la position des plaques sur le tronÃ§on... Revenir lÃ -dessus une foie la modif validÃ©e ou non.
                dbCreationPosition = pSectionLink->GetLength() / 3;
            }
            else
            {
                m_pNetwork->log() << Logger::Error << "Unable to find a path from a supposedly accessible section departing from plaque " << pPlaque->GetID() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
    }
    else
    {
        if (bPlaquesDefined)
        {
            m_pNetwork->log() << Logger::Warning << "No departing candidate plaque to reach destination " << pMultiClassDestination->GetID() << " : the origin link will be randomly chosen in the parent zone" << std::endl;
            m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
        }
    }

    return pResult;
}

//================================================================
Tuyau * ZoneDeTerminaison::AddInitialPathToLeg(
	double dbInstant, 
	SymuViaFleetParameters * pFleetParams, 
	CPlaque * pPlaqueOrigin, 
	CMotifCoeff * pMotifCoeff, 
	Tuyau * pTuy, 
	TypeVehicule * pTypeVehicule, 
	TripLeg * pTripLeg, 
	bool bForceNonResidential, 
	double & dbCreationPosition, 
	Parking* & pParkingOrigin, 
	bool & bSortieStationnement)
//----------------------------------------------------------------
// Fonction  :  Construit le dÃ©but du parcours pour sortir de la
// zone d'origine en fonction du motif et des plaques
//================================================================
{
    bSortieStationnement = false;
    CPlaque * pPlaque = NULL;
    bool bDontCreateVehicle = false;
    Tuyau * pResult = TiragePlaqueOrigine(dbInstant, pPlaqueOrigin, pTypeVehicule, pMotifCoeff, NULL, pTuy, pTripLeg, bForceNonResidential, dbCreationPosition, pPlaque, bDontCreateVehicle, pParkingOrigin, bSortieStationnement);
	
	//if(pPlaqueOrigin)
		//m_pNetwork->log() << std::endl << "ZoneDeTerminaison::AddInitialPathToLeg pPlaqueOrigin " << pPlaqueOrigin->GetID() << std::endl;

    if (bDontCreateVehicle)
    {
        return NULL;
    }

    if (pPlaque != NULL)
    {
        pFleetParams->SetPlaqueOrigin(pPlaque);
    }
    else
    {
        TronconDestination * pTDest = NULL;
        if (pTuy)
        {
            std::map<std::pair<Tuyau*, double>, TronconDestination*>::iterator iter = m_CacheTronconsDestination.find(make_pair(pTuy, UNDEF_LINK_POSITION));
            if (iter != m_CacheTronconsDestination.end())
            {
                pTDest = iter->second;
            }
            else
            {
                pTDest = new TronconDestination(pTuy);
                m_CacheTronconsDestination[make_pair(pTuy, UNDEF_LINK_POSITION)] = pTDest;
            }
        }
        // rmq. : pTDest reste nul si itinÃ©raire interne Ã  une zone Ã  la fois origine et destination. La suite du code n'appliqura donc pas de contrainte d'accessibilitÃ©
        // vers un point de jonction puisqu'il est non dÃ©fini dans ce cas.

        // Si on est en mode gÃ©nÃ©ration de demande pour SymuMaster, pTripLeg est nul, et on ne fait pas de tirage entre
        // stationnement surfacique / parking : ce sera fait pas SymuVia au moment de l'activation du vÃ©hicule
        bool bIsResidentiel = true;
        bool bIsParkingSurfacique = true;
        bool bPerimetreElargi = false;
        if (pTripLeg)
        {
            TiragesStationnementOrigin(dbInstant, NULL, pTypeVehicule, pTDest, bForceNonResidential, bIsResidentiel, bIsParkingSurfacique, bPerimetreElargi, bDontCreateVehicle);
            if (bDontCreateVehicle)
            {
                return NULL;
            }
        }

        if (!bIsParkingSurfacique)
        {
            // Cas du tirage d'un parking hors sol. Si on a pu tirer le cas parking non surfacique, c'est qu'on a forcÃ©ment du stock dedans, donc ne peut Ã©chouer
            pParkingOrigin = TirageParkingOrigin(NULL, pTypeVehicule, bPerimetreElargi, pTDest);

            if (AddInitialPathFromParking(pTripLeg, pParkingOrigin, pTDest, pTypeVehicule, dbInstant))
            {
                return pTripLeg->GetPath().front();
            }
            else
            {
                m_pNetwork->log() << Logger::Error << "Unable to find a path from the supposedly accessible parking " << pParkingOrigin->GetID() << " departing from zone " << GetID() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

                pParkingOrigin = NULL;

                return NULL;
            }
        }
        else
        {
			Tuyau * pSectionLink = AddInitialPathFromSection(pTripLeg, dbInstant, pPlaque, pTypeVehicule, bPerimetreElargi, pTDest, bIsResidentiel);
            if (pSectionLink)
            {
                pResult = pSectionLink;
                bSortieStationnement = !bIsResidentiel;

                // Tirage de la position sur la section
                //dbRand = RandManager::getInstance()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;
                //dbCreationPosition = dbRand * pRandomLink->GetLength();

                // Finalement on prend 1/3 du tronÃ§on pour que tout le monde s'insÃ¨re au mÃªme endroit et respecter la FIFO des vÃ©hicules en attente d'insertion
                dbCreationPosition = pSectionLink->GetLength() / 3;
            }
            else
            {
                m_pNetwork->log() << Logger::Warning << "The zone " << GetOutputID() << " doesn't allow to reach the link " << pTuy->GetLabel() << ". The vehicle will leave directly from the link " << pTuy->GetLabel() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
    }

    return pResult;
}

void ZoneDeTerminaison::TiragePlaqueDestination(double dbInstant, CMotifCoeff * pMotifCoeff, Vehicule * pVehicule, SymuViaTripNode * pOrigin, TypeVehicule * pVehiculeType, CPlaque * & pPlaqueResult, Tuyau * & pCurrentTuy, Tuyau * & pPrevTuyau)
{
    bool bPlaquesDefined = false;
    bool bDone = false;
    pPlaqueResult = NULL;

    CPlaque * pForcedPlaqueDest = NULL;

    Connexion * inputJunction = NULL;

    pPrevTuyau = NULL;
    if (pVehicule)
    {
        SymuViaFleetParameters* pFleetParams = (SymuViaFleetParameters*)pVehicule->GetFleetParameters();
        pForcedPlaqueDest = pFleetParams->GetPlaqueDestination();

        // DÃ©termination du tuyau initial du vÃ©hicule
        pCurrentTuy = pVehicule->GetLink(0) != NULL ? pVehicule->GetLink(0) : pVehicule->GetLink(1); // pour avoir une fonction valable en dÃ©but et fin de pas de temps

        // Pour avoir le point de jonction alors qu'on ne l'a pas encore ranchi et positionnÃ© forcÃ©ment :
        if (!pFleetParams->GetZoneDestinationInputJunction())
        {
            if (pCurrentTuy->GetBriqueParente())
            {
                pFleetParams->SetZoneDestinationInputJunction(pCurrentTuy->GetBriqueParente());
            }
            else
            {
                // le cas classique ici serait de prendre le tuyau aval (cas du vÃ©hicule qui arrive sur le tronÃ§on amont au point de jonction),
                // sauf que on peut aussi avoir le cas particulier du vÃ©hicule qui rentre sur le rÃ©seau dÃ©jÃ  dans la zone : il faut alors prendre le noeud amont !
                // du coup, on regarde d'abord le noeud amont, et s'il n'est pas un point de jonction de la zone, le noeud aval.
                const std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > & inputJunctions = GetInputJunctions();
                if (inputJunctions.find(pCurrentTuy->GetCnxAssAm()) != inputJunctions.end())
                {
                    pFleetParams->SetZoneDestinationInputJunction(pCurrentTuy->GetCnxAssAm());
                }
                else
                {
                    pFleetParams->SetZoneDestinationInputJunction(pCurrentTuy->GetCnxAssAv());
                }
            }
        }

        if (pCurrentTuy->GetBriqueParente())
        {
            // On prend le tuyau qui suit la brique dans l'itinÃ©raire
            int iTuyIdx = pVehicule->GetItineraireIndex(isBriqueAmontZone, pCurrentTuy->GetBriqueParente());
            if (iTuyIdx != -1)
            {
                pCurrentTuy = pVehicule->GetItineraire()->at(iTuyIdx);
                if (iTuyIdx > 0)
                {
                    pPrevTuyau = pVehicule->GetItineraire()->at(iTuyIdx - 1);
                }
            }
            else
            {
                // Le vÃ©hicule est dÃ©jÃ  en bout d'itinÃ©raire dans la brique aval (peut se produire si tronÃ§on court) :
                pPrevTuyau = pVehicule->GetItineraire()->back();
            }
        }

        inputJunction = ((SymuViaFleetParameters*)pVehicule->GetFleetParameters())->GetZoneDestinationInputJunction();
    }

    

    // Constitution de l'ensemble des plaques candidates (pour lesquelles on a un motif) avec le coefficient associÃ©
    std::map<CPlaque*, double, LessPlaquePtr> plaquesCandidates;
    if (pForcedPlaqueDest)
    {
        // Moude plaque de destination imposÃ©e par SymuMaster :
        plaquesCandidates[pForcedPlaqueDest] = 1;
    }
    else
    {
        // Mode classique : tirage d'une plaque accessible
        if (pMotifCoeff)
        {
            double dbSumPlaquesCoeffs = 0;
            for (size_t i = 0; i < m_LstPlaques.size(); i++)
            {
                CPlaque * pPlaque = m_LstPlaques[i];
                double dbCoeff = pPlaque->GetMotifCoeff(pMotifCoeff->GetMotifDestination());
                if (dbCoeff > 0)
                {
                    bPlaquesDefined = true;
                    if (!GetAccessibleLinksToEnd(pPlaque, false, pVehiculeType, inputJunction, pOrigin, pCurrentTuy, pPrevTuyau).empty())
                    {
                        plaquesCandidates[pPlaque] = dbCoeff;
                        dbSumPlaquesCoeffs += dbCoeff;
                    }
                }
            }
            // Ã©largissement du pÃ©rimÃ¨tre des plaques si aucune plaque n'est accessible...
            if (plaquesCandidates.empty())
            {
                dbSumPlaquesCoeffs = 0;
                for (size_t i = 0; i < m_LstPlaques.size(); i++)
                {
                    CPlaque * pPlaque = m_LstPlaques[i];
                    double dbCoeff = pPlaque->GetMotifCoeff(pMotifCoeff->GetMotifDestination());
                    if (dbCoeff > 0)
                    {
                        bPlaquesDefined = true;
                        if (!GetAccessibleLinksToEnd(pPlaque, true, pVehiculeType, inputJunction, pOrigin, pCurrentTuy, pPrevTuyau).empty())
                        {
                            plaquesCandidates[pPlaque] = dbCoeff;
                            dbSumPlaquesCoeffs += dbCoeff;
                        }
                    }
                }
            }
            // renormalisation des coefficients :
            for (std::map<CPlaque*, double, LessPlaquePtr>::iterator iterPlaque = plaquesCandidates.begin(); iterPlaque != plaquesCandidates.end(); ++iterPlaque)
            {
                iterPlaque->second /= dbSumPlaquesCoeffs;
            }
        }
    }

    if (!plaquesCandidates.empty())
    {
        // Tirage d'une plaque parmis les candidates
        ///////////////////////////////////////////////////////////////////////////
        CPlaque * pPlaque = NULL;
        double dbSum = 0;
        double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

        for (std::map<CPlaque*, double, LessPlaquePtr>::iterator iterPlaque = plaquesCandidates.begin(); iterPlaque != plaquesCandidates.end() && !pPlaque; ++iterPlaque)
        {
            dbSum += iterPlaque->second;

            if (dbRand < dbSum)
            {
                // on tient notre plaque
                pPlaque = iterPlaque->first;
            }
        }

        // protection contre les erreurs d'arrondi liÃ©s Ã  le renormalisation
        if (!pPlaque)
        {
            pPlaque = plaquesCandidates.rbegin()->first;
        }

        // Si on est en mode gÃ©nÃ©ration de demande pour SymuMaster, pTripLeg est nul, et on ne fait pas de tirage entre
        // stationnement surfacique / parking : ce sera fait pas SymuVia au moment de l'activation du vÃ©hicule
        bool bIsResidentiel = true;
        bool bIsParkingSurfacique = true;
        if (pVehicule)
        {
            TiragesStationnementDestination(dbInstant, pPlaque, pVehiculeType, inputJunction, pCurrentTuy, pPrevTuyau, false, bIsResidentiel, bIsParkingSurfacique);
            ((SymuViaFleetParameters*)pVehicule->GetFleetParameters())->SetIsTerminaisonResidentielle(bIsResidentiel);
            ((SymuViaFleetParameters*)pVehicule->GetFleetParameters())->SetIsTerminaisonSurfacique(bIsParkingSurfacique);
        }

      
        if (!bIsParkingSurfacique)
        {
            // Cas du tirage d'un parking hors sol. Si on a pu tirer le cas parking non surfacique, c'est qu'on a forcÃ©ment de la palce libre dedans, donc ne peut Ã©chouer
            Parking * pParkingDestination = TirageParkingDestination(pPlaque, pVehiculeType, false, inputJunction, pOrigin, pCurrentTuy, pPrevTuyau);

            if (AddFinalPathToParking(pVehicule, pParkingDestination, inputJunction, pCurrentTuy, pPrevTuyau, pVehiculeType, dbInstant))
            {
                bDone = true;
                pPlaqueResult = pPlaque;
            }
            else
            {
                m_pNetwork->log() << Logger::Error << "Unable to find a path to the supposedly accessible parking " << pParkingDestination->GetID() << " departing from plaque " << pPlaque->GetID() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
        else
        {
            if (AddFinalPathToSection(pVehicule, dbInstant, pPlaque, pVehiculeType, pOrigin, pCurrentTuy, pPrevTuyau, false))
            {
                bDone = true;
                pPlaqueResult = pPlaque;
            }
            else
            {
                m_pNetwork->log() << Logger::Error << "Unable to find a path to a supposedly accessible section as destination in plaque " << pPlaque->GetID() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }
    }
    else
    {
        if (bPlaquesDefined)
        {
            m_pNetwork->log() << Logger::Warning << "The chosen plaque for the destination of the vehicle is not accessible : the vehicle will use the parent zone " << GetID() << " as destination" << std::endl;
            m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
        }
    }

    if (pVehicule && bDone && !pForcedPlaqueDest)
    {
        ((SymuViaFleetParameters*)pVehicule->GetFleetParameters())->SetPlaqueDestination(pPlaqueResult);
    }

    if (!bDone && pForcedPlaqueDest)
    {
        // On force une plaque mais on ne la tire pas : problÃ¨me, on ne respecte pas la  consigne...
        m_pNetwork->log() << Logger::Warning << "The requested plaque " << pForcedPlaqueDest->GetID() << " as destination is not accessible from " << (pVehicule ? pCurrentTuy->GetLabel() : pOrigin->GetOutputID()) << " !" << std::endl;
        m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }
}

//================================================================
void ZoneDeTerminaison::ReaffecteVehiculeTroncon(Vehicule * pVehicule, double dbInstant)
//----------------------------------------------------------------
// Fonction  :  Traite la terminaison en zone en mode distribuÃ©
// (utilisation du motif et tirage d'une plaque puis d'une section)
//================================================================
{
    SymuViaFleetParameters * pFleetParams = (SymuViaFleetParameters*)pVehicule->GetFleetParameters();

    Tuyau * pCurrentTuy = NULL;
    Tuyau * pPrevTuyau = NULL;

    // DÃ©termination du tuyau initial du vÃ©hicule
    pCurrentTuy = pVehicule->GetLink(0) != NULL ? pVehicule->GetLink(0) : pVehicule->GetLink(1); // pour avoir une fonction valable en dÃ©but et fin de pas de temps
    if (pCurrentTuy->GetBriqueParente())
    {
        // On prend le tuyau qui suit la brique dans l'itinÃ©raire
        int iTuyIdx = pVehicule->GetItineraireIndex(isBriqueAmontZone, pCurrentTuy->GetBriqueParente());
        if (iTuyIdx != -1)
        {
            pCurrentTuy = pVehicule->GetItineraire()->at(iTuyIdx);
            if (iTuyIdx > 0)
            {
                pPrevTuyau = pVehicule->GetItineraire()->at(iTuyIdx - 1);
            }
        }
        else
        {
            // Le vÃ©hicule est dÃ©jÃ  en bout d'itinÃ©raire dans la brique aval (peut se produire si tronÃ§on court) :
            pPrevTuyau = pVehicule->GetItineraire()->back();
        }
    }

    const std::vector<Tuyau*> & accessibleLinks = GetAccessibleLinksToEnd(pFleetParams->GetPlaqueDestination(), pFleetParams->IsPerimetreElargi(dbInstant, this), pVehicule->GetType(), pFleetParams->GetZoneDestinationInputJunction(), NULL, pCurrentTuy, pPrevTuyau);

    // On enlÃ¨ve les tuyaux dÃ©jÃ  parcourus :
    std::map<Tuyau*, double, LessPtr<Tuyau> > candidateLinks;
    std::set<Tuyau*> outputLinks;
    double dbSumLength = 0;
    for (size_t iLink = 0; iLink < accessibleLinks.size(); iLink++)
    {
        Tuyau * pCandidate = accessibleLinks.at(iLink);

        if (std::find(pFleetParams->GetLstTuyauxDejaParcourus().begin(), pFleetParams->GetLstTuyauxDejaParcourus().end(), pCandidate) == pFleetParams->GetLstTuyauxDejaParcourus().end())
        {
            candidateLinks[pCandidate] = pCandidate->GetLength();
            dbSumLength += pCandidate->GetLength();

            if (pCandidate->get_Type_aval() == 'S' || pCandidate->get_Type_aval() == 'P')
            {
                outputLinks.insert(pCandidate);
            }
        }
    }

    if (candidateLinks.empty())
    {
        // On fera Ã  partir de maintenant du mode Ã©tendu pour ce vÃ©hicule.
        pFleetParams->SetForcePerimetreEtendu(true);

        const std::vector<Tuyau*> & accessibleLinksExtended = GetAccessibleLinksToEnd(pFleetParams->GetPlaqueDestination(), true, pVehicule->GetType(), pFleetParams->GetZoneDestinationInputJunction(), NULL, pCurrentTuy, pPrevTuyau);

        // On enlÃ¨ve les tuyaux dÃ©jÃ  parcourus :
        dbSumLength = 0;
        for (size_t iLink = 0; iLink < accessibleLinksExtended.size(); iLink++)
        {
            Tuyau * pCandidate = accessibleLinksExtended.at(iLink);

            if (std::find(pFleetParams->GetLstTuyauxDejaParcourus().begin(), pFleetParams->GetLstTuyauxDejaParcourus().end(), pCandidate) == pFleetParams->GetLstTuyauxDejaParcourus().end())
            {
                candidateLinks[pCandidate] = pCandidate->GetLength();
                dbSumLength += pCandidate->GetLength();

                if (pCandidate->get_Type_aval() == 'S' || pCandidate->get_Type_aval() == 'P')
                {
                    outputLinks.insert(pCandidate);
                }
            }
        }
    }

    // on Ã©vite dans un premier temps les tuyaux qui mÃ¨nent directement Ã  une sortie ponctuelle :
    if (outputLinks.size() < candidateLinks.size())
    {
        for (std::map<Tuyau*, double, LessPtr<Tuyau> >::const_iterator iterTuyau = candidateLinks.begin(); iterTuyau != candidateLinks.end();)
        {
            Tuyau * pCandidate = iterTuyau->first;
            if (pCandidate->get_Type_aval() == 'S' || pCandidate->get_Type_aval() == 'P')
            {
                iterTuyau = candidateLinks.erase(iterTuyau);
                dbSumLength -= pCandidate->GetLength();
            }
            else
            {
                ++iterTuyau;
            }
        }
    }

    Tuyau * pRandomTuyauRetenu = NULL;

    if (!candidateLinks.empty())
    {
        // Tirage d'une section candidate
        double dbSum = 0;
        double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

        Tuyau * pRandomLink = NULL;

        for (std::map<Tuyau*, double, LessPtr<Tuyau> >::const_iterator iterTuyau = candidateLinks.begin(); iterTuyau != candidateLinks.end() && !pRandomLink; ++iterTuyau)
        {
            dbSum += iterTuyau->second / dbSumLength;

            if (dbRand < dbSum)
            {
                // on tient notre tuyau
                pRandomLink = iterTuyau->first;
            }
        }
        // protection contre les erreurs de limite de prÃ©cision numÃ©rique
        if (!pRandomLink)
        {
            pRandomLink = candidateLinks.rbegin()->first;
        }

        // Calcul de l'itinÃ©raire vers le tronÃ§on cible pRandomTuyau si diffÃ©rent du tuyau actuel
        std::vector<PathResult> newPaths;

        TronconDestination * pTDest;
        std::map<std::pair<Tuyau*, double>, TronconDestination*>::iterator iter = m_CacheTronconsDestination.find(make_pair(pRandomLink, UNDEF_LINK_POSITION));
        if (iter != m_CacheTronconsDestination.end())
        {
            pTDest = iter->second;
        }
        else
        {
            pTDest = new TronconDestination(pRandomLink, UNDEF_LINK_POSITION);
            m_CacheTronconsDestination[make_pair(pRandomLink, UNDEF_LINK_POSITION)] = pTDest;
        }

        if (pRandomLink == pCurrentTuy)
        {
            PathResult res;
            newPaths.push_back(res);
        }
        else
        {
            std::map<std::vector<Tuyau*>, double> placeHolder;
            m_pNetwork->GetSymuScript()->ComputePaths(pCurrentTuy->GetBriqueParente() ? pCurrentTuy->GetBriqueParente() : pCurrentTuy->GetCnxAssAv(), pTDest, pVehicule->GetType(), dbInstant, 1, pCurrentTuy->GetBriqueParente() ? pPrevTuyau : pCurrentTuy, newPaths, placeHolder, false);
        }

        if (newPaths.size() == 1)
        {
            // Reconstruction de l'itinÃ©raire
            std::vector<Tuyau*> newIti;
            if (!pCurrentTuy->GetBriqueParente())
            {
                newIti.push_back(pCurrentTuy);
            }
            newIti.insert(newIti.end(), newPaths.front().links.begin(), newPaths.front().links.end());
            TripLeg * pNewLeg = new TripLeg();
            pNewLeg->SetDestination(pTDest);
            pNewLeg->SetPath(newIti);
            pVehicule->GetTrip()->AddTripLeg(pNewLeg);
            pVehicule->MoveToNextLeg();
        }
        else
        {
            assert(false); // en principe on constitue une liste de sections accessibles...
        }
    }
    else
    {
        // Aucun tuyau atteignable et non dÃ©jÃ  parcouru.
        m_pNetwork->log() << Logger::Warning << "The vehicle " << pVehicule->GetID() << " did not find any available parking spot after trying every accessible links. The vehicle is destroyed." << std::endl;
        m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
        pVehicule->SetTuyau(NULL, dbInstant);
        pVehicule->SetVoie(NULL);
        pVehicule->SetDejaCalcule(true);
        pVehicule->SetInstantSortie(dbInstant);

        assert(!pFleetParams->IsTerminaisonResidentielle());
        GestionDemandeStationnement(pFleetParams->GetPlaqueDestination(), dbInstant);
    }
}

bool ZoneDeTerminaison::CheckCoeffs(Logger & logger)
{
    bool bOk = true;

    // Constitution de la liste des motifs dÃ©finis dans les noeuds REP_MOTIFS_PLAQUES de la zone
    std::set<CMotif*> setMotifs;
    for (size_t iPlaque = 0; iPlaque < m_LstPlaques.size(); iPlaque++)
    {
        CPlaque * pPlaque = m_LstPlaques[iPlaque];
        const std::map<CMotif*, double> & mapMotifs = pPlaque->GetMapMotifCoeff();
        std::map<CMotif*, double>::const_iterator iter;
        for (iter = mapMotifs.begin(); iter != mapMotifs.end(); ++iter)
        {
            if (iter->second > 0)
            {
                setMotifs.insert(iter->first);
            }
        }
    }

    // on rajoute les motifs d'origine dÃ©finis dans les noeuds MOTIF_COEFF (ils doivent apparaÃ®tre dans les plaques)
    std::map<TypeVehicule*, std::deque<TimeVariation<CRepMotif> > >::iterator iter;
    for (iter = m_lstRepMotifInit.begin(); iter != m_lstRepMotifInit.end(); ++iter)
    {
        for (size_t i = 0; i < iter->second.size(); i++)
        {
            std::map<SymuViaTripNode*, CRepMotifDest> & mapRepMotifs = iter->second[i].m_pData->getMapRepMotifs();
            std::map<SymuViaTripNode*, CRepMotifDest>::iterator iter2;
            for (iter2 = mapRepMotifs.begin(); iter2 != mapRepMotifs.end(); ++iter2)
            {
                for (size_t j = 0; j < iter2->second.getCoeffs().size(); j++)
                {
                    const CMotifCoeff & motifCoeff = iter2->second.getCoeffs()[j];
                    if (motifCoeff.getCoeff() > 0)
                    {
                        setMotifs.insert(motifCoeff.GetMotifOrigine());
                    }
                }
            }
        }
    }

    // Pour chaque motif, on regarde si la somme des coefficients au travers de chaque plaque donne bien Ã  peu prÃ¨s 1
    std::set<CMotif*>::const_iterator iterMotifs;
    for (iterMotifs = setMotifs.begin(); iterMotifs != setMotifs.end(); ++iterMotifs)
    {
        double dbSum = 0;
        std::set<CPlaque*> setPlaques;
        for (size_t iPlaque = 0; iPlaque < m_LstPlaques.size(); iPlaque++)
        {
            CPlaque * pPlaque = m_LstPlaques[iPlaque];
            double dbCoeff = pPlaque->GetMotifCoeff(*iterMotifs);
            if (dbCoeff > 0)
            {
                dbSum += dbCoeff;
                setPlaques.insert(pPlaque);
            }
        }

        // VÃ©rification de la somme des coefficients

        // rmq. : l'Ã©cart sur la somme peut dÃ©passer epsilon (la perte de precision se cumule)
        double dbTolerance = std::numeric_limits<double>::epsilon() * setPlaques.size();

        // Pour pouvoir faire fonctionner les anciens fichiers invalides sans effort, on normalise automatiquement
        // les coefficients si la somme est proche de 1 au sens de l'ancienne tolÃ©rance (0.0001)
        if (fabs(dbSum - 1.0) > dbTolerance && fabs(dbSum - 1.0) < 0.0001)
        {
            // logger << Logger::Warning << "WARNING : la somme des coefficents pour le motif " << (*iterMotifs)->GetID() << " pour l'ensemble des plaques de la zone " << GetID() << " est proche mais diffÃ©rente de 1 : normalisation automatique des coefficients..." << std::endl;
            // logger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            // Normalisation automatique des coefficients
            for (std::set<CPlaque*>::iterator iterPlaque = setPlaques.begin(); iterPlaque != setPlaques.begin(); ++iterPlaque)
            {
                (*iterPlaque)->SetMotifCoeff(*iterMotifs, (*iterPlaque)->GetMotifCoeff(*iterMotifs) / dbSum);
            }
            dbSum = 1.0;
        }

        if (fabs(dbSum - 1) > dbTolerance)
        {
            logger << Logger::Error << "ERROR : the sum of coefficients for motive " << (*iterMotifs)->GetID() << " for all plaques in zone " << GetID() << " must be equal to 1 " << std::endl;
            logger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            bOk = false;
        }
    }

    return bOk;
}

void ZoneDeTerminaison::ClearPathCaches()
{
    m_CacheAvailableOriginLinks.clear();
    m_CacheAvailableDestinationLinksFromOrigin.clear();
    m_CacheAvailableDestinationLinksFromLink.clear();
    m_CacheExtendedAvailableDestinationLinksFromLink.clear();
}

// fonction de construction de la map Ã  partir d'un ensemble de longueurs et de sections
void ZoneDeTerminaison::FillCandidateSectionsFromLength(const std::vector<CSection> & sections, TypeVehicule * pTypeVehicule, std::map<CSection, double, LessSection> & sectionsCandidates)
{
    double dbLengthSum = 0.0;
    for (size_t iSection = 0; iSection < sections.size(); iSection++)
    {
        const CSection & section = sections[iSection];
        sectionsCandidates[section] = section.getLength();
        dbLengthSum += section.getLength();
    }
    for (std::map<CSection, double, LessSection>::iterator iterSection = sectionsCandidates.begin(); iterSection != sectionsCandidates.end(); ++iterSection)
    {
        iterSection->second /= dbLengthSum;
    }
}

// Construit la map pour le tirage d'une section de la zone ou plaque de dÃ©part en fonction du type de stationnement initial
void ZoneDeTerminaison::FillCandidateSectionsMapOrigin(CPlaque * pPlaque, TypeVehicule * pTypeVehicule, bool bPerimetreElargi, SymuViaTripNode * pJunctionLink, bool bIsResidentiel, std::map<CSection, double, LessSection> & sectionsCandidates)
{
    // construction de la liste des sections accessibles + Ã©tendues
    const std::vector<Tuyau*> & accessibleLinks = GetAccessibleLinksToStart(pPlaque, bPerimetreElargi, pTypeVehicule, pJunctionLink);
    std::vector<CSection> accessibleSections;

    if (pPlaque)
    {
        for (size_t iSection = 0; iSection < pPlaque->getSections().size(); iSection++)
        {
            const CSection & section = pPlaque->getSections().at(iSection);
            if (std::find(accessibleLinks.begin(), accessibleLinks.end(), section.getTuyau()) != accessibleLinks.end())
            {
                accessibleSections.push_back(section);
            }
        }
        if (bPerimetreElargi)
        {
            // Il faut dans ce cas ajouter des sections virtuelles pour les liens accessible mais qui n'apparaisent pas dans la plaque
            for (size_t iLink = 0; iLink < accessibleLinks.size(); iLink++)
            {
                Tuyau * pLink = accessibleLinks.at(iLink);
                if (!pPlaque->HasLink(pLink))
                {
                    accessibleSections.push_back(CSection(pLink, 0, pLink->GetLength()));
                }
            }
        }
    }
    else
    {
        for (size_t iLink = 0; iLink < accessibleLinks.size(); iLink++)
        {
            Tuyau * pLink = accessibleLinks.at(iLink);
            accessibleSections.push_back(CSection(pLink, 0, pLink->GetLength()));
        }
    }

    if (bIsResidentiel)
    {
        FillCandidateSectionsFromLength(accessibleSections, pTypeVehicule, sectionsCandidates);
    }
    else
    {
        // cas du stationnement surfacique : on tire au prorata des places de stationnement occupÃ©es
        double dbStockSum = 0.0;
        for (size_t iSection = 0; iSection < accessibleSections.size(); iSection++)
        {
            const CSection & section = accessibleSections[iSection];
            double dbStock = section.getTuyau()->GetStockStationnement() * section.getLength() / section.getTuyau()->GetLength();
            if (dbStock > 0)
            {
                sectionsCandidates[section] = dbStock;
                dbStockSum += dbStock;
            }
        }
        for (std::map<CSection, double, LessSection>::iterator iterSection = sectionsCandidates.begin(); iterSection != sectionsCandidates.end(); ++iterSection)
        {
            iterSection->second /= dbStockSum;
        }

        // Cas du stock nul : on tire une section accessible au hasard en pondÃ©rant par la longueur:
        if (sectionsCandidates.empty())
        {
            FillCandidateSectionsFromLength(accessibleSections, pTypeVehicule, sectionsCandidates);
        }
    }
}

void ZoneDeTerminaison::FillCandidateSectionsMapDestination(CPlaque * pPlaque, TypeVehicule * pTypeVehicule, bool bPerimetreElargi, Connexion * pJunction, SymuViaTripNode * pOrigin, Tuyau * pCurrentTuy, Tuyau * pPrevTuyau, std::map<CSection, double, LessSection> & sectionsCandidates)
{
    // construction de la liste des sections accessibles + Ã©tendues
    const std::vector<Tuyau*> & accessibleLinks = GetAccessibleLinksToEnd(pPlaque, bPerimetreElargi, pTypeVehicule, pJunction, pOrigin, pCurrentTuy, pPrevTuyau);

    // on privilÃ©gie dans un premier temps les tuyaux ne conduisant pas Ã  une sortie ponctuelle, pour Ã©viter de bloquer les vÃ©hicules.
    std::vector<Tuyau*> candidateLinks;
    for (size_t iLink = 0; iLink < accessibleLinks.size(); iLink++)
    {
        Tuyau * pLink = accessibleLinks.at(iLink);
        if (pLink->get_Type_aval() != 'S' && pLink->get_Type_aval() != 'P')
        {
            candidateLinks.push_back(pLink);
        }
    }

    // Si aucun autre candidat que les tuyaux menant Ã  une sortie ponctuelle, tant pis, on les prend
    if (candidateLinks.empty())
    {
        candidateLinks = accessibleLinks;
    }


    std::vector<CSection> accessibleSections;

    if (pPlaque)
    {
        for (size_t iSection = 0; iSection < pPlaque->getSections().size(); iSection++)
        {
            const CSection & section = pPlaque->getSections().at(iSection);
            if (std::find(candidateLinks.begin(), candidateLinks.end(), section.getTuyau()) != candidateLinks.end())
            {
                accessibleSections.push_back(section);
            }
        }
        if (bPerimetreElargi)
        {
            // Il faut dans ce cas ajouter des sections virtuelles pour les liens accessible mais qui n'apparaissent pas dans la plaque
            for (size_t iLink = 0; iLink < candidateLinks.size(); iLink++)
            {
                Tuyau * pLink = candidateLinks.at(iLink);
                if (!pPlaque->HasLink(pLink))
                {
                    accessibleSections.push_back(CSection(pLink, 0, pLink->GetLength()));
                }
            }
        }
    }
    else
    {
        for (size_t iLink = 0; iLink < candidateLinks.size(); iLink++)
        {
            Tuyau * pLink = candidateLinks.at(iLink);
            accessibleSections.push_back(CSection(pLink, 0, pLink->GetLength()));
        }
    }

    FillCandidateSectionsFromLength(accessibleSections, pTypeVehicule, sectionsCandidates);
}


Parking * ZoneDeTerminaison::TirageParkingOrigin(CPlaque * pPlaque, TypeVehicule * pTypeVehicule, bool bPerimetreElargi, SymuViaTripNode * pMultiClassDestination)
{
    Parking * pParkingOrigin = NULL;

    const std::vector<Tuyau*> & accessibleLinks = GetAccessibleLinksToStart(pPlaque, bPerimetreElargi, pTypeVehicule, pMultiClassDestination);

    const std::set<Parking*> & setParkings = GetParkingsFromLinks(accessibleLinks, pTypeVehicule);

    int iStockSum = 0;

    std::map<Parking*, double, LessConnexionPtr<Parking> > parkingCoeffs;
    for (std::set<Parking*>::const_iterator iterPark = setParkings.begin(); iterPark != setParkings.end(); ++iterPark)
    {
        int iStock = (*iterPark)->GetStock();
        if (iStock > 0)
        {
            parkingCoeffs[*iterPark] = iStock;
            iStockSum += (*iterPark)->GetStock();
        }
    }
    for (std::map<Parking*, double, LessConnexionPtr<Parking> >::iterator iterPark = parkingCoeffs.begin(); iterPark != parkingCoeffs.end(); ++iterPark)
    {
        iterPark->second /= iStockSum;
    }

    if (parkingCoeffs.empty())
    {
        // aucun parking avec du stock, on tire un parking au hasard
        for (std::set<Parking*>::const_iterator iterPark = setParkings.begin(); iterPark != setParkings.end(); ++iterPark)
        {
            parkingCoeffs[*iterPark] = 1.0 / setParkings.size();
        }
    }
    // Tirage du parking :
    double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

    double dbSum = 0;
    for (std::map<Parking*, double, LessConnexionPtr<Parking> >::iterator iterPark = parkingCoeffs.begin(); iterPark != parkingCoeffs.end() && !pParkingOrigin; ++iterPark)
    {
        dbSum += iterPark->second;

        if (dbRand < dbSum)
        {
            // on tient notre plaque
            pParkingOrigin = iterPark->first;
        }
    }
    // protection contre les erreurs d'arrondi liÃ©s Ã  la renormalisation
    if (!pParkingOrigin)
    {
        pParkingOrigin = parkingCoeffs.rbegin()->first;
    }

    return pParkingOrigin;
}

Parking * ZoneDeTerminaison::TirageParkingDestination(CPlaque * pPlaque, TypeVehicule * pTypeVehicule, bool bPerimetreElargi, Connexion * pJunction, SymuViaTripNode * pOrigin, Tuyau * pCurrentTuy, Tuyau * pPrevTuyau)
{
    Parking * pParkingDestination = NULL;

    const std::vector<Tuyau*> & accessibleLinks = GetAccessibleLinksToEnd(pPlaque, bPerimetreElargi, pTypeVehicule, pJunction, pOrigin, pCurrentTuy, pPrevTuyau);

    const std::set<Parking*> & setParkings = GetParkingsFromLinks(accessibleLinks, pTypeVehicule);

    int iRoomSum = 0;

    std::map<Parking*, double, LessConnexionPtr<Parking> > parkingCoeffs;
    std::set<Parking*> infiniteStockParkings;
    for (std::set<Parking*>::const_iterator iterPark = setParkings.begin(); iterPark != setParkings.end(); ++iterPark)
    {
        int iStockMax = (*iterPark)->GetStockMax();
        if (iStockMax == -1)
        {
            iStockMax = GetParkingParameters()->GetMaxParkingSpotsWhenNoMaximumParkingCapacity();
            infiniteStockParkings.insert(*iterPark);
        }
        int iRoom = iStockMax - (*iterPark)->GetStock();
        if (iRoom > 0)
        {
            parkingCoeffs[*iterPark] = iRoom;
            iRoomSum += iRoom;
        }
    }
    if (parkingCoeffs.empty() && !infiniteStockParkings.empty())
    {
        // Si pas de place trouvÃ© mais qu'on a des parkings illimitÃ©s, on tire au hasard parmi ceux lÃ 
        parkingCoeffs.clear();
        for (std::set<Parking*>::const_iterator iterIllimitedParking = infiniteStockParkings.begin(); iterIllimitedParking != infiniteStockParkings.end(); ++iterIllimitedParking)
        {
            parkingCoeffs[*iterIllimitedParking] = 1.0 / infiniteStockParkings.size();
        }
    }
    else
    {
        for (std::map<Parking*, double, LessConnexionPtr<Parking> >::iterator iterPark = parkingCoeffs.begin(); iterPark != parkingCoeffs.end(); ++iterPark)
        {
            iterPark->second /= iRoomSum;
        }
    }
    

    if (parkingCoeffs.empty())
    {
        // aucun parking avec de la place, on tire un parking au hasard
        for (std::set<Parking*>::const_iterator iterPark = setParkings.begin(); iterPark != setParkings.end(); ++iterPark)
        {
            parkingCoeffs[*iterPark] = 1.0 / setParkings.size();
        }
    }
    // Tirage du parking :
    double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

    double dbSum = 0;
    for (std::map<Parking*, double, LessConnexionPtr<Parking> >::iterator iterPark = parkingCoeffs.begin(); iterPark != parkingCoeffs.end() && !pParkingDestination; ++iterPark)
    {
        dbSum += iterPark->second;

        if (dbRand < dbSum)
        {
            // on tient notre plaque
            pParkingDestination = iterPark->first;
        }
    }
    // protection contre les erreurs d'arrondi liÃ©s Ã  la renormalisation
    if (!pParkingDestination)
    {
        pParkingDestination = parkingCoeffs.rbegin()->first;
    }

    return pParkingDestination;
}


void ZoneDeTerminaison::TiragesStationnementOrigin(double dbInstant, CPlaque * pPlaque, TypeVehicule * pTypeVehicule, SymuViaTripNode * pMultiClassDestination, bool bForceNonResidential, bool & bIsResidentiel, bool & bIsParkingSurfacique, bool & bPerimetreElargi, bool & bDontCreateVehicle)
{
    // 1 - tirage entre rÃ©sidentiel ou non rÃ©sidentiel
    bIsResidentiel = bForceNonResidential ? false : TirageTypeOrigineResidentielle(pPlaque, dbInstant);

    // 2 - si pas rÃ©sidentiel, tirage entre parking surfacique et parking hors-sol
    if (!bIsResidentiel)
    {
        bool bStockFound;
        bIsParkingSurfacique = TirageTypeParkingSurfaciqueOrigin(dbInstant, pPlaque, bPerimetreElargi, pTypeVehicule, pMultiClassDestination, bStockFound);
        if (!bStockFound)
        {
            // Ã©largissement de la zone de recherche :
            bPerimetreElargi = true;
            bIsParkingSurfacique = TirageTypeParkingSurfaciqueOrigin(dbInstant, pPlaque, bPerimetreElargi, pTypeVehicule, pMultiClassDestination, bStockFound);
            if (!bStockFound)
            {
                ParkingParameters * pParkingParams = pPlaque ? pPlaque->GetParkingParameters() : GetParkingParameters();
                if (!pParkingParams->DoCreateVehiclesWithoutParkingStock())
                {
                    bDontCreateVehicle = true;
                    if (pPlaque)
                    {
                        m_pNetwork->log() << Logger::Warning << "No available stock in plaque " << pPlaque->GetID() << " to craete vehicle : the requested demand level will not be reached !" << std::endl;
                    }
                    else
                    {
                        m_pNetwork->log() << Logger::Warning << "No available stock in zone " << GetOutputID() << " to create vehicle : the demand level will not be reached !" << std::endl;
                    }
                    m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
                else
                {
                    m_pNetwork->log() << Logger::Warning << "Vehicle creation in spite of no available parking stock" << std::endl;
                    m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                }
            }
        }
    }
}

// Tirage du type de stationnement en zone ou plaque Ã  l'arrivÃ©e d'un vÃ©hicule
void ZoneDeTerminaison::TiragesStationnementDestination(double dbInstant, CPlaque * pPlaque, TypeVehicule * pTypeVehicule, Connexion * pJunction, Tuyau * pCurrentTuy, Tuyau * pPrevTuyau, bool bPerimetreElargi, bool & bIsResidentiel, bool & bIsParkingSurfacique)
{
    // 1 - tirage entre rÃ©sidentiel ou non rÃ©sidentiel
    bIsResidentiel = TirageTypeDestinationResidentielle(pPlaque, dbInstant);

    // 2 - si pas rÃ©sidentiel, tirage entre parking surfacique et parking hors-sol
    if (!bIsResidentiel)
    {
        bIsParkingSurfacique = TirageTypeParkingSurfaciqueDestination(dbInstant, pPlaque, bPerimetreElargi, pTypeVehicule, pJunction, pCurrentTuy, pPrevTuyau);
    }
}


bool ZoneDeTerminaison::AddInitialPathFromParking(TripLeg * pTripLeg, Parking * pParkingOrigin, SymuViaTripNode * pMultiClassDestination, TypeVehicule * pTypeVehicule, double dbInstant)
{
    // Cas particulier de l'itinÃ©raire interne Ã  une zone : pMultiClassDestination est NULL et on n'a pas besoin de vÃ©rifier que le parkign est accesible :
    bool bOk;
    if (pMultiClassDestination)
    {
        std::vector<PathResult> newPaths;
        std::map<std::vector<Tuyau*>, double> placeHolder;
        m_pNetwork->GetSymuScript()->ComputePaths(pParkingOrigin, pMultiClassDestination, pTypeVehicule, dbInstant, 1, newPaths, placeHolder, false);

        if (newPaths.size() == 1)
        {
            bOk = true;

            // C'est tout bon, on ajoute le prÃ©itinÃ©raire Ã  celui de pTripLeg
            // Reconstruction de l'itinÃ©raire
            std::vector<Tuyau*> newIti = newPaths.front().links;

            // Ajout en amont de l'itinÃ©raire initial :
            const std::vector<Tuyau*> & oldIti = pTripLeg->GetPath();
            // on retranche le dernier tronÃ§on qui fesait dÃ©jÃ  parti de l'itinÃ©raire en principe (sauf si l'itinÃ©raire initial est vide : cas d'une route vide avec point de jonction = sortie)
            if (!oldIti.empty())
            {
                newIti.pop_back();
            }

            newIti.insert(newIti.end(), oldIti.begin(), oldIti.end());
            pTripLeg->SetPath(newIti);
        }
        else
        {
            bOk = false;
            assert(false); // origine accessible par construction en principe...
        }
    }
    else
    {
        // On prend pour itinÃ©raire le tuyau aval au parking uniquement. l'algo de terminaison en zone doit prendre le relai ensuite.
        pTripLeg->SetPath(std::vector<Tuyau*>(1, pParkingOrigin->GetOutputConnexion()->m_LstTuyAssAv.front()));
        bOk = true;
    }

    return bOk;
}

bool ZoneDeTerminaison::AddFinalPathToParking(Vehicule * pVehicle, Parking * pParkingDestination, Connexion * pJunction, Tuyau * pCurrentTuy, Tuyau * pPrevTuyau, TypeVehicule * pTypeVehicule, double dbInstant)
{
    bool bOk;
    std::vector<PathResult> newPaths;
    std::map<std::vector<Tuyau*>, double> placeHolder;

    if (pCurrentTuy->GetCnxAssAv() == pParkingDestination->GetInputConnexion())
    {
        PathResult res;
        res.links.push_back(pCurrentTuy);
        newPaths.push_back(res);
    }
    else
    {
        m_pNetwork->GetSymuScript()->ComputePaths(pCurrentTuy->GetBriqueParente() ? pCurrentTuy->GetBriqueParente() : pCurrentTuy->GetCnxAssAv(), pParkingDestination, pTypeVehicule, dbInstant, 1, pCurrentTuy->GetBriqueParente() ? pPrevTuyau : pCurrentTuy, newPaths, placeHolder, false);
    }

    
    if (newPaths.size() == 1)
    {
        bOk = true;

        // Reconstruction de l'itinÃ©raire
        std::vector<Tuyau*> newIti;
        if (!pCurrentTuy->GetBriqueParente())
        {
            newIti.push_back(pCurrentTuy);
        }
        newIti.insert(newIti.end(), newPaths.front().links.begin(), newPaths.front().links.end());
        TripLeg * pNewLeg = new TripLeg();
        pNewLeg->SetDestination(pParkingDestination);
        pNewLeg->SetPath(newIti);
        pVehicle->GetTrip()->AddTripLeg(pNewLeg);
        pVehicle->MoveToNextLeg();
    }
    else
    {
        bOk = false;
        assert(false); // destination accessible par construction en principe...
    }

    return bOk;
}


Tuyau * ZoneDeTerminaison::AddInitialPathFromSection(TripLeg * pTripLeg, double dbInstant, CPlaque * pPlaque, TypeVehicule * pTypeVehicule, bool bPerimetreElargi, SymuViaTripNode * pMultiClassDestination, bool bIsResidentiel)
{
    Tuyau * pResult = NULL;

    // Tirage d'une section permettant d'accÃ©der au tuyau cible pMultiClassDestination

    // Construction de la map des sections avec le coefficient de tirage correspondant
    std::map<CSection, double, LessSection> sectionsCandidates;
    FillCandidateSectionsMapOrigin(pPlaque, pTypeVehicule, bPerimetreElargi, pMultiClassDestination, bIsResidentiel, sectionsCandidates);

    if (!sectionsCandidates.empty())
    {
        // Tirage d'une section candidate
        double dbSum = 0;
        double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

        const CSection * pSection = NULL;

        for (std::map<CSection, double, LessSection>::const_iterator iterSection = sectionsCandidates.begin(); iterSection != sectionsCandidates.end() && !pSection; ++iterSection)
        {
            dbSum += iterSection->second;

            if (dbRand < dbSum)
            {
                // on tient notre section
                pSection = &iterSection->first;
            }
        }

        // protection contre les erreurs d'arrondi liÃ©s Ã  le renormalisation
        if (!pSection)
        {
            pSection = &sectionsCandidates.rbegin()->first;
        }

        // Calcul de l'itinÃ©raire vers le tronÃ§on cible pTuy
        std::vector<PathResult> newPaths;
        std::map<std::vector<Tuyau*>, double> placeHolder;

        if (pMultiClassDestination)
        {
			//m_pNetwork->log() << Logger::Warning << "pMultiClassDestination " << pMultiClassDestination->GetID() << std::endl;
            TronconDestination * pLinkDest = dynamic_cast<TronconDestination*>(pMultiClassDestination);

            if (pLinkDest && pSection->getTuyau() == pLinkDest->GetTuyau())
            {
                PathResult res;
                newPaths.push_back(res);
            }
            else
            {
				//if (pTripLeg->GetPath().size() > 0)
					m_pNetwork->GetSymuScript()->ComputePaths(pSection->getTuyau()->GetCnxAssAv(), pMultiClassDestination, pTypeVehicule, dbInstant, 1, pSection->getTuyau(), newPaths, placeHolder, false);
				/*else
				{
					// TODO: passer le point de jonction plutot que le tronÃ§on pour calculer l'itinÃ©raire - afin d'Ãªtre sÃ»re d'avoir le chemin optimal
					// Connexion -> SymuViaTripNode ? Non, nouvelle fonction ComputePaths Ã  crÃ©er ???
					Connexion *pDest = pLinkDest->GetTuyau()->GetCnxAssAv();
					m_pNetwork->GetSymuScript()->ComputePaths(pSection->getTuyau()->GetCnxAssAv(), pDest, pTypeVehicule, dbInstant, 1, pSection->getTuyau(), newPaths, placeHolder, false);
				}*/
			}

            if (newPaths.size() == 1)
            {
                // C'est tout bon, on ajoute le prÃ©itinÃ©raire Ã  celui de pTripLeg
                if (pTripLeg)
                {
                    // Reconstruction de l'itinÃ©raire
                    std::vector<Tuyau*> newIti;
                    newIti.push_back(pSection->getTuyau());
                    newIti.insert(newIti.end(), newPaths.front().links.begin(), newPaths.front().links.end());

                    // Ajout en amont de l'itinÃ©raire initial :
                    const std::vector<Tuyau*> & oldIti = pTripLeg->GetPath();
                    // on retranche le dernier tronÃ§on qui fesait dÃ©jÃ  parti de l'itinÃ©raire en principe (sauf si l'itinÃ©raire initial est vide : cas d'une route vide avec point de jonction = sortie)
                    if (!oldIti.empty())
                    {
                        newIti.pop_back();
                    }

                    newIti.insert(newIti.end(), oldIti.begin(), oldIti.end());
                    pTripLeg->SetPath(newIti);
                }

                pResult = pSection->getTuyau();
            }
            else
            {
                m_pNetwork->log() << Logger::Error << "No path found from the candidate link " << pSection->getTuyau() << " to the destination " << pMultiClassDestination->GetID() << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                assert(false); // en principe on constitue une liste de sections accessibles...
            }
        }
        else
        {
            // Cas particulier de l'itinÃ©raire interne Ã  une zone :
            pResult = pSection->getTuyau();
            pTripLeg->SetPath(std::vector<Tuyau*>(1, pResult));

			//m_pNetwork->log() << Logger::Warning << "Origin and destination zones are the same - Path" << pResult->GetLabel() << std::endl;
        }
    }
    else
    {
        if (pPlaque)
        {
            m_pNetwork->log() << Logger::Warning << "Unable to find a candidate departing section for plaque " << pPlaque->GetID() << std::endl;
        }
        else
        {
            m_pNetwork->log() << Logger::Warning << "Unable to find a candidate departing link for zone " << GetID() << std::endl;
        }
        m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }

    return pResult;
}

bool ZoneDeTerminaison::AddFinalPathToSection(Vehicule * pVehicle, double dbInstant, CPlaque * pPlaque, TypeVehicule * pTypeVehicule, SymuViaTripNode * pOrigin, Tuyau * pCurrentTuy, Tuyau * pPrevTuy, bool bPerimetreElargi)
{
    bool bDone = false;
	//m_pNetwork->log() << std::endl << "AddFinalPathToSection - id " << pVehicle->GetID() << std::endl;
	//if(pCurrentTuy)
		//m_pNetwork->log() << std::endl << "pCurrentTuy " << pCurrentTuy->GetLabel() << std::endl;

    // Tirage d'une section accessible depuis l'origin considÃ©rÃ©e

    // Construction de la map des sections avec le coefficient de tirage correspondant
    std::map<CSection, double, LessSection> sectionsCandidates;
    FillCandidateSectionsMapDestination(pPlaque, pTypeVehicule, bPerimetreElargi, pVehicle ? ((SymuViaFleetParameters*)pVehicle->GetFleetParameters())->GetZoneDestinationInputJunction() : NULL, pOrigin, pCurrentTuy, pPrevTuy, sectionsCandidates);

    if (!sectionsCandidates.empty())
    {
        // Tirage d'une section candidate
        double dbSum = 0;
        double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

        const CSection * pSection = NULL;

        for (std::map<CSection, double, LessSection>::const_iterator iterSection = sectionsCandidates.begin(); iterSection != sectionsCandidates.end() && !pSection; ++iterSection)
        {
            dbSum += iterSection->second;

            if (dbRand < dbSum)
            {
                // on tient notre section
                pSection = &iterSection->first;
            }
        }

        // protection contre les erreurs d'arrondi liÃ©s Ã  le renormalisation
        if (!pSection)
        {
            pSection = &sectionsCandidates.rbegin()->first;
        }

        // Calcul de l'itinÃ©raire vers le tronÃ§on cible pSection->getTuyau()
        std::vector<PathResult> newPaths;

        TronconDestination * pTDest;
        std::map<std::pair<Tuyau*, double>, TronconDestination*>::iterator iter = m_CacheTronconsDestination.find(make_pair(pSection->getTuyau(), pSection->getStartPosition()));
        if (iter != m_CacheTronconsDestination.end())
        {
            pTDest = iter->second;
        }
        else
        {
            pTDest = new TronconDestination(pSection->getTuyau(), pSection->getStartPosition());
            m_CacheTronconsDestination[make_pair(pSection->getTuyau(), pSection->getStartPosition())] = pTDest;
        }

        if (pVehicle)
        {
            if (pSection->getTuyau() == pCurrentTuy)
            {
                PathResult res;
                newPaths.push_back(res);
            }
            else
            {
                std::map<std::vector<Tuyau*>, double> placeHolder;
                m_pNetwork->GetSymuScript()->ComputePaths(pCurrentTuy->GetBriqueParente() ? pCurrentTuy->GetBriqueParente() : pCurrentTuy->GetCnxAssAv(), pTDest, pTypeVehicule, dbInstant, 1, pCurrentTuy->GetBriqueParente() ? pPrevTuy : pCurrentTuy, newPaths, placeHolder, false);
            }
        }
        else
        {
            assert(pOrigin);
            std::map<std::vector<Tuyau*>, double> placeHolder;
            m_pNetwork->GetSymuScript()->ComputePaths(pOrigin, pTDest, pTypeVehicule, dbInstant, 1, newPaths, placeHolder, false);
        }

        if (newPaths.size() == 1)
        {
            bDone = true;
            if (pVehicle)
            {
                // Reconstruction de l'itinÃ©raire
                std::vector<Tuyau*> newIti;
                if (!pCurrentTuy->GetBriqueParente())
                {
                    newIti.push_back(pCurrentTuy);
                }
                newIti.insert(newIti.end(), newPaths.front().links.begin(), newPaths.front().links.end());
                TripLeg * pNewLeg = new TripLeg();
                pNewLeg->SetDestination(pTDest);
                pNewLeg->SetPath(newIti);
                pVehicle->GetTrip()->AddTripLeg(pNewLeg);
                pVehicle->MoveToNextLeg();

				/*m_pNetwork->log() << "Path: " ;
				for (int i = 0; i < newIti.size(); i++)
					m_pNetwork->log() << newIti[i]->GetLabel() << " ";
				m_pNetwork->log() << std::endl;*/
            }
        }
        else
        {
            assert(false); // en principe on constitue une liste de sections accessibles...
        }
    }
    else
    {
        if (pPlaque)
        {
            m_pNetwork->log() << Logger::Warning << "Unable to find a candidate arrival section in plaque " << pPlaque->GetID() << std::endl;
        }
        else
        {
            m_pNetwork->log() << Logger::Warning << "Unable to find a candidate arrival link in zone " << GetID() << std::endl;
        }
        m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }

    return bDone;
}

// Calcul de la distance avant stationnement (mode simplifiÃ©)
void ZoneDeTerminaison::UpdateGoalDistance(double dbInstant, SymuViaFleetParameters * pParams, TypeVehicule * pTypeVeh, Tuyau * pCurrentTuy, Tuyau * pPrevTuyau)
{
    const std::vector<Tuyau*> & accessibleLinks = GetAccessibleLinksToEnd(pParams->GetPlaqueDestination(), pParams->IsPerimetreElargi(dbInstant, this), pTypeVeh, pParams->GetZoneDestinationInputJunction(), NULL, pCurrentTuy, pPrevTuyau);
    pParams->GetAccessibleLinksForParking() = accessibleLinks;
    UpdateGoalDistance(dbInstant, pParams);
}

// Met Ã  jour la distance avant stationnement (mode simplifiÃ©), en se basant sur les tuyaux accessibles dÃ©jÃ  utilisÃ©s lors du calcul initial
void ZoneDeTerminaison::UpdateGoalDistance(double dbInstant, SymuViaFleetParameters * pParams)
{
    ParkingParametersVariation * pParkingParams = pParams->GetPlaqueDestination() ? pParams->GetPlaqueDestination()->GetParkingParameters()->GetVariation(dbInstant) : GetParkingParameters()->GetVariation(dbInstant);

    const double dbLengthBetweenTwoParkBlocks = pParkingParams->GetLengthBetweenTwoParkBlocks();
    const double dbLengthBetweenTwoParkSpots = pParkingParams->GetLengthBetweenTwoParkSpots();
    const double dbNbOfSpotsPerBlock = pParkingParams->GetNbOfSpotsPerBlock();;

    bool bHasSurfacicStock, bHasSurfacicRoom;

    double dbTauxOccupationSurfacique = GetTauxOccupationSurfacique(pParams->GetAccessibleLinksForParking(), bHasSurfacicStock, bHasSurfacicRoom);

    double dbDistance;
    if (!bHasSurfacicRoom || dbTauxOccupationSurfacique >= 1)
    {
        dbDistance = DBL_MAX;
    }
    else
    {
        dbDistance = dbLengthBetweenTwoParkBlocks / (1.0 - pow(dbTauxOccupationSurfacique, dbNbOfSpotsPerBlock)) + dbLengthBetweenTwoParkSpots / (1.0 - dbTauxOccupationSurfacique);
    }

    pParams->UpdateGoalDistance(dbDistance, dbInstant);
}


// RÃ©cupÃ¨re le mode surfacique pour l'arrivÃ©e en zone
bool ZoneDeTerminaison::IsModeSurfaciqueSimplifie(CPlaque * pPlaque, double dbInstant)
{
    if (pPlaque)
    {
        return pPlaque->GetParkingParameters()->GetVariation(dbInstant)->UseSimplifiedSurcaficMode();
    }
    else
    {
        return GetParkingParameters()->GetVariation(dbInstant)->UseSimplifiedSurcaficMode();
    }
}


// RÃ©cupÃ¨re le mode de demande autogÃ©nÃ©rÃ©e par le stationnement
bool ZoneDeTerminaison::IsModeDemandeStationnement(CPlaque * pPlaque, double dbInstant)
{
    if (pPlaque)
    {
        return pPlaque->GetParkingParameters()->GetVariation(dbInstant)->AutoGenerateParkingDemand();
    }
    else
    {
        return GetParkingParameters()->GetVariation(dbInstant)->AutoGenerateParkingDemand();
    }
}

// Gestion de la demande autogÃ©nÃ©rÃ©e pour le stationnement
void ZoneDeTerminaison::GestionDemandeStationnement(CPlaque * pPlaque, double dbInstant)
{
    if (IsModeDemandeStationnement(pPlaque, dbInstant))
    {
        ParkingParametersVariation * pParams = pPlaque ? pPlaque->GetParkingParameters()->GetVariation(dbInstant) : GetParkingParameters()->GetVariation(dbInstant);
        const double dbMeanDuration = pParams->GetParkingMeanTime();
        const double dbSigma = pParams->GetParkingTimeSigma();
        double dbParkingDuration = m_pNetwork->GetRandManager()->myNormalRand(dbMeanDuration, dbSigma);
        this->AddCreationFromStationnement(dbInstant + std::max<double>(0, dbParkingDuration));
    }
}

Tuyau * ZoneDeTerminaison::LinkRandomSelection(double dbInstant, CPlaque * pPlaque, TypeVehicule * pTypeVehicule, bool bPerimetreElargi, SymuViaTripNode * pMultiClassDestination, bool bIsResidentiel)
{
	Tuyau * pResult = NULL;

	// Random selection of a link from the area

	// Construction de la map des sections avec le coefficient de tirage correspondant
	std::map<CSection, double, LessSection> sectionsCandidates;
	FillCandidateSectionsMapOrigin(pPlaque, pTypeVehicule, bPerimetreElargi, pMultiClassDestination, bIsResidentiel, sectionsCandidates);

	if (!sectionsCandidates.empty())
	{
		// Tirage d'une section candidate
		double dbSum = 0;
		double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

		const CSection * pSection = NULL;

		for (std::map<CSection, double, LessSection>::const_iterator iterSection = sectionsCandidates.begin(); iterSection != sectionsCandidates.end() && !pSection; ++iterSection)
		{
			dbSum += iterSection->second;

			if (dbRand < dbSum)
			{
				// on tient notre section
				pSection = &iterSection->first;
			}
		}

		// protection contre les erreurs d'arrondi liÃ©s Ã  le renormalisation
		if (!pSection)
		{
			pSection = &sectionsCandidates.rbegin()->first;
		}
		pResult = pSection->getTuyau();
	}

	return pResult;
}

#ifdef USE_SYMUCORE
void ZoneDeTerminaison::CalculTempsParcours(double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration)
{
    // Gestion du dÃ©part en zone
    m_AreaHelper.CalculTempsParcoursDepart(m_pNetwork, this, dbInstFinPeriode, pMacroType, dbPeriodDuration);
    for (size_t iPlaque = 0; iPlaque < m_LstPlaques.size(); iPlaque++)
    {
        m_LstPlaques[iPlaque]->m_AreaHelper.CalculTempsParcoursDepart(m_pNetwork, this, dbInstFinPeriode, pMacroType, dbPeriodDuration);
    }

    // Gestion de l'arrivÃ©e en zone
    m_AreaHelper.CalculTempsParcoursArrivee(m_pNetwork, this, dbInstFinPeriode, pMacroType, dbPeriodDuration);
    for (size_t iPlaque = 0; iPlaque < m_LstPlaques.size(); iPlaque++)
    {
        m_LstPlaques[iPlaque]->m_AreaHelper.CalculTempsParcoursArrivee(m_pNetwork, this, dbInstFinPeriode, pMacroType, dbPeriodDuration);
    }
}

double ZoneDeTerminaison::GetEmptyMeanSpeed(double dbInstFinPeriode, SymuCore::MacroType * pMacroType)
{
    assert(m_pMFDSensor);

    double dbMeanSpeed = 0;
    double dbCumulatedLength = 0;
    for (size_t j = 0; j < m_pMFDSensor->GetLstSensors().size(); j++)
    {
        Tuyau * pLink = m_pMFDSensor->GetLstSensors()[j]->GetTuyau();

        TypeVehicule * pTypeVeh = pMacroType == NULL ? NULL : m_pNetwork->GetVehiculeTypeFromMacro(pMacroType);
        double dbTpsVitReg = ((TuyauMicro*)pLink)->GetCoutAVide(dbInstFinPeriode, pTypeVeh, false, true);
        if ((!pMacroType || pTypeVeh) && dbTpsVitReg < DBL_MAX && dbTpsVitReg > 0) // on ignore les tronÃ§ons interdits de la zone pour le calcul de la vitesse moyenne, ou les tronÃ§ons de cout nul pour Ã©viter la division apr 0
        {
            dbMeanSpeed += pLink->GetLength() * pLink->GetLength() / dbTpsVitReg;
            dbCumulatedLength += pLink->GetLength();
        }
    }
    if (dbCumulatedLength > 0)
    {
        dbMeanSpeed /= dbCumulatedLength;
    }

    // rmq. si le rÃ©sultat est nul, c'est que la zone n'est pas empruntable
    return dbMeanSpeed;
}
void ZoneDeTerminaison::ComputeConcentration(double dbPeriodDuration)
{
    double dbTotalTT = 0;
    double dbTotalLength = 0;
    for (size_t j = 0; j < m_pMFDSensor->GetLstSensors().size(); j++)
    {
        EdieSensor * pCptEdie = m_pMFDSensor->GetLstSensors()[j];

        // calcul du cumul
        double dbTD = pCptEdie->GetData().GetTotalTravelledDistance();
        double dbTT = pCptEdie->GetData().GetTotalTravelledTime();

        dbTotalTT += dbTT;
        dbTotalLength += pCptEdie->GetSensorLength() * pCptEdie->GetTuyau()->getNb_voies();
    }
    double dbMeanTotalVehicleNb = dbTotalTT / dbPeriodDuration;

    m_dbConcentration = 0;
    if (dbTotalLength > 0)
    {
        m_dbConcentration = dbMeanTotalVehicleNb / dbTotalLength;
    }
}
void ZoneDeTerminaison::ComputeMeanSpeed(double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration)
{
    assert(m_pMFDSensor);

    double & dbMeanSpeed = m_dbMeanSpeed[pMacroType];

    double dbTT = 0;
    double dbTD = 0;
    for (size_t j = 0; j< m_pMFDSensor->GetLstSensors().size(); j++)
    {
        EdieSensor * pCptEdie = m_pMFDSensor->GetLstSensors()[j];

        // calcul du cumul
        const std::map<TypeVehicule*, double> & mapTT = pCptEdie->GetData().dbTotalTravelledTime.back();
        for (std::map<TypeVehicule*, double>::const_iterator iterTT = mapTT.begin(); iterTT != mapTT.end(); ++iterTT)
        {
            if (!pMacroType || pMacroType->hasVehicleType(iterTT->first->GetLabel()))
            {
                dbTT += iterTT->second;
            }
        }

        const std::map<TypeVehicule*, double> & mapTD = pCptEdie->GetData().dbTotalTravelledDistance.back();
        for (std::map<TypeVehicule*, double>::const_iterator iterTD = mapTD.begin(); iterTD != mapTD.end(); ++iterTD)
        {
            if (!pMacroType || pMacroType->hasVehicleType(iterTD->first->GetLabel()))
            {
                dbTD += iterTD->second;
            }
        }
    }

    if (dbTT > 0)
    {
        dbMeanSpeed = dbTD / dbTT;
    }
    else
    {
        // Pas de temps passÃ© : on n'a vu passer personne : on
        // prend la vitesse rÃ©glementaire moyenne sur la zone en pondÃ©rant par la longueur des tronÃ§ons :
        dbMeanSpeed = GetEmptyMeanSpeed(dbInstFinPeriode, pMacroType);
    }
}
#endif // USE_SYMUCORE


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void ZoneDeTerminaison::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void ZoneDeTerminaison::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void ZoneDeTerminaison::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(SymuViaTripNode);

    ar & BOOST_SERIALIZATION_NVP(m_LstParkings);
    ar & BOOST_SERIALIZATION_NVP(m_LstPlaques);
    ar & BOOST_SERIALIZATION_NVP(m_dbMaxDistanceToJunction);
    ar & BOOST_SERIALIZATION_NVP(m_pParkingParameters);
    ar & BOOST_SERIALIZATION_NVP(m_bOwnsParkingParams);
}