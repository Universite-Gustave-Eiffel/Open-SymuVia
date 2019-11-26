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
#include "SymuViaFleet.h"

#include "SymuViaTripNode.h"
#include "Trip.h"
#include "TripLeg.h"
#include "reseau.h"
#include "entree.h"
#include "sortie.h"
#include "Parking.h"
#include "ZoneDeTerminaison.h"
#include "Logger.h"
#include "voie.h"
#include "tuyau.h"
#include "TronconOrigine.h"
#include "../Affectation.h"
#include "SystemUtil.h"
#include "usage/SymuViaFleetParameters.h"
#include "usage/SymuViaVehicleToCreate.h"
#ifdef USE_SYMUCOM
#include "ITS/Stations/DerivedClass/Simulator.h"
#endif // USE_SYMUCOM


SymuViaFleet::SymuViaFleet()
    : AbstractFleet()
{
}

SymuViaFleet::SymuViaFleet(Reseau * pNetwork)
    : AbstractFleet(pNetwork)
{
}

SymuViaFleet::~SymuViaFleet()
{
}

AbstractFleetParameters * SymuViaFleet::CreateFleetParameters()
{
    return new SymuViaFleetParameters();
}

void SymuViaFleet::InitSimuTrafic(std::deque< TimeVariation<TraceDocTrafic> > & docTrafics)
{
    // Nettoyage (utile en cas de SymReset)
    m_LstTripNodes.clear();
    for (size_t i = 0; i < m_LstTrips.size(); i++)
    {
        delete m_LstTrips[i];
    }
    m_LstTrips.clear();

    // DÃ©finition des TripNodes associÃ©s Ã  la flotte
    m_LstTripNodes.insert(m_LstTripNodes.end(), m_pNetwork->Liste_entree.begin(), m_pNetwork->Liste_entree.end());
    m_LstTripNodes.insert(m_LstTripNodes.end(), m_pNetwork->Liste_sortie.begin(), m_pNetwork->Liste_sortie.end());
    m_LstTripNodes.insert(m_LstTripNodes.end(), m_pNetwork->Liste_parkings.begin(), m_pNetwork->Liste_parkings.end());
    m_LstTripNodes.insert(m_LstTripNodes.end(), m_pNetwork->Liste_zones.begin(), m_pNetwork->Liste_zones.end());

    // CrÃ©ation d'un Trip par origine.
    for(size_t iOrigin = 0; iOrigin < m_pNetwork->Liste_origines.size(); iOrigin++)
    {
        SymuViaTripNode * pOrigin = m_pNetwork->Liste_origines[iOrigin];

        // Initialisation de l'origine
        pOrigin->CopyDemandeInitToDemande(m_pNetwork->m_LstTypesVehicule); 
        pOrigin->CopyCoeffDestInitToCoeffDest(m_pNetwork->m_LstTypesVehicule);
        pOrigin->CopyRepVoieInitToRepVoie(m_pNetwork->m_LstTypesVehicule);
        pOrigin->CopyRepMotifDestInitToRepMotifDest(m_pNetwork->m_LstTypesVehicule);

        // CrÃ©ation 
        Trip * pNewTrip = new Trip();
        pNewTrip->SetOrigin(m_pNetwork->Liste_origines[iOrigin]);
        pNewTrip->SetID(pNewTrip->GetOrigin()->GetID());
        m_LstTrips.push_back(pNewTrip);
    }

    // Appel de la mÃ©thode de la classe mÃ¨re
    AbstractFleet::InitSimuTrafic(docTrafics);
}

void SymuViaFleet::SortieTrafic(DocTrafic *pXMLDocTrafic)
{
    if (m_pNetwork->DoSortieTraj())
    {
        // Sortie des infos sur les entrÃ©es
        std::deque<Entree*>::iterator itEntree;
	    for( itEntree = m_pNetwork->Liste_entree.begin(); itEntree != m_pNetwork->Liste_entree.end(); ++itEntree)	
        {
    	    (*itEntree)->SortieTrafic(pXMLDocTrafic);
        }
    }

    // Sortie des infos des stocks
    if(m_pNetwork->IsTraceStocks())
    {
        std::deque<Parking*>::iterator itParking;
        for( itParking = m_pNetwork->Liste_parkings.begin(); itParking != m_pNetwork->Liste_parkings.end(); itParking++)	
        {
		    (*itParking)->SortieTrafic(pXMLDocTrafic);
        }

        std::deque<ZoneDeTerminaison*>::iterator itZone;
        for( itZone = m_pNetwork->Liste_zones.begin(); itZone != m_pNetwork->Liste_zones.end(); itZone++)	
        {
		    (*itZone)->SortieTrafic(pXMLDocTrafic);
        }
    }
}

void SymuViaFleet::FinCalculTrafic(Vehicule * pVeh)
{
    SymuViaFleetParameters * pParams = (SymuViaFleetParameters*)pVeh->GetFleetParameters();

    if(pParams->GetInstantEntreeZone() != -1)
    {
        // incrÃ©mentation de la distance parcourue en terminaison de trajet
        pParams->IncDistanceParcourue(pVeh->GetDstParcourueEx());
    }
}

void SymuViaFleet::ActivateVehicle(double dbInstant, VehicleToCreate * pVehicleToCreate)
{
    SymuViaVehicleToCreate * pVehicle = (SymuViaVehicleToCreate*) pVehicleToCreate;
    boost::shared_ptr<Vehicule> pVeh = pVehicle->GetOrigin()->GenVehicule(pVehicle->GetVehicleID(), pVehicle->GetType(), pVehicle->GetNumVoie(), dbInstant, pVehicle->GetTimeFraction(),
        pVehicle->GetDestination(), pVehicle->GetItinerary().get(), pVehicle->GetJunction(), pVehicle->GetPlaqueOrigin(), pVehicle->GetPlaqueDestination(), false);
    if (pVeh)
    {
        pVeh->SetExternalID(pVehicle->GetExternalID());
        pVeh->SetNextRouteID(pVehicle->GetNextRouteID());
        m_pNetwork->GetSymuViaFleet()->OnVehicleActivated(pVeh, dbInstant);
    }

}


std::vector<boost::shared_ptr<Vehicule> > SymuViaFleet::ActivateVehiclesForTrip(double dbInstant, double dbTimeStep, Trip * pTrip)
{
    std::vector<boost::shared_ptr<Vehicule>> ListVehicule;
    if (!m_pNetwork->IsSymuMasterMode())
    {
        SymuViaTripNode * pOrigin = (SymuViaTripNode*)pTrip->GetOrigin();
        pOrigin->CreationVehicules(dbInstant, ListVehicule);

        if (m_pNetwork->IsCptItineraire())
        {
            size_t count = ListVehicule.size();
            ZoneDeTerminaison * pZone;
            for (size_t i = 0; i < count; i++)
            {
                boost::shared_ptr<Vehicule> pVeh = ListVehicule[i];
                TypeVehicule * pTypVeh = pVeh->GetType();
                pZone = dynamic_cast<ZoneDeTerminaison*>(pOrigin);
                std::string sOrig = pVeh->GetOrigine()->GetOutputID();
                const std::string & sDest = pVeh->GetDestination()->GetInputID();
                m_pNetwork->GetAffectationModule()->IncreaseNBVehEmis(pTypVeh, sOrig, sDest);

                size_t countj = pVeh->GetItineraire()->size();
                std::vector<std::string> listIti;
                for (size_t j = 0; j<countj; j++)
                {
                    listIti.push_back(pVeh->GetItineraire()->at(j)->GetLabel());
                    listIti.push_back(pVeh->GetItineraire()->at(j)->GetCnxAssAv()->GetID());
                }
                if (listIti.size() > 0)
                {
                    listIti.pop_back();
                }
                // dÃ©termination de si l'itinÃ©raire est prÃ©dÃ©fini ou pas pour le fichier de rÃ©sultats d'affectation
                bool bPredefini = false;
                if (m_pNetwork->GetAffectationModule()->IsSaving())
                {
                    std::map<TypeVehicule*, std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> > >::iterator itMapAssignment = ((SymuViaTripNode*)pVeh->GetOrigine())->GetMapAssignment().find(pTypVeh);
                    if (itMapAssignment != ((SymuViaTripNode*)pVeh->GetOrigine())->GetMapAssignment().end())
                    {
                        std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> >::const_iterator itAD = itMapAssignment->second.find(std::pair<Tuyau*, SymuViaTripNode*>(nullptr, (SymuViaTripNode*)pVeh->GetDestination()));
                        if (itAD != itMapAssignment->second.end())
                        {
                            for (size_t nIti = 0; nIti < itAD->second.size(); nIti++)
                            {
                                if (itAD->second[nIti].bPredefini)
                                {
                                    if (itAD->second[nIti].dqTuyaux == *pVeh->GetItineraire())
                                    {
                                        bPredefini = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                m_pNetwork->GetAffectationModule()->AddRealiseItineraire(pTypVeh, sOrig, sDest, listIti, *pVeh->GetItineraire(), bPredefini, pVeh->GetID());
            }
        }
    }

    return ListVehicule;
}

// Gestion de la terminaison d'une Ã©tape par un vÃ©hicule de la flotte.
void SymuViaFleet::OnCurrentLegFinished(boost::shared_ptr<Vehicule> pVehicle, VoieMicro * pDestinationEnterLane, double dbInstDestinationReached, double dbInstant, double dbTimeStep)
{
    SymuViaFleetParameters * pParams = (SymuViaFleetParameters*)pVehicle->GetFleetParameters();

    // cas de recherche de rÃ©sidus, si le temps est Ã©coulÃ©, on arrÃªte l'itinÃ©raire calculÃ© avec le premier algo pour passer au second algo.
    if(pParams->IsTerminaison())
    {
        ZoneDeTerminaison * pZoneDest = dynamic_cast<ZoneDeTerminaison*>(pVehicle->GetDestination());
        
        // remarque : en mode parking, si on arrive dans le parking, c'est de toute faÃ§on trop tard pour rÃ©affecter quoi que ce soit : il faut dÃ©tecter le parking plein en amont.
        if (pZoneDest && !pParams->IsTerminaisonResidentielle() && pParams->IsTerminaisonSurfacique())
        {
            pParams->SetIsRechercheStationnement(true, dbInstant);
            
            if (pParams->DoReaffecteVehicule())
            {
                pZoneDest->ReaffecteVehiculeStationnementDestination(pVehicle.get(), dbInstant, true);
                pParams->SetDoReaffecteVehicule(false);
            }
            else
            {
                pZoneDest->ReaffecteVehiculeTroncon(pVehicle.get(), dbInstant);
            }
        }
        else if (pZoneDest && !pParams->IsTerminaisonSurfacique() && pParams->DoReaffecteVehicule())
        {
            // Cas du rÃ©arbitrage liÃ© au fait qu'un vÃ©hicule arrive sur un parking plein :
            pParams->SetIsRechercheStationnement(true, dbInstant);
            pZoneDest->ReaffecteVehiculeStationnementDestination(pVehicle.get(), dbInstant, true);
            pParams->SetDoReaffecteVehicule(false);
        }
        else
        {
            // Appel de la mÃ©thode de la classe mÃ¨re
            AbstractFleet::OnCurrentLegFinished(pVehicle, pDestinationEnterLane, dbInstDestinationReached, dbInstant, dbTimeStep);
        }
    }
    else
    {
        // DÃ©tection de l'arrivÃ©e en zone et traitements associÃ©s
        ZoneDeTerminaison * pZoneDest = dynamic_cast<ZoneDeTerminaison*>(pVehicle->GetDestination());
        if( pZoneDest )
        {
            // Tirage de la destination finale du vÃ©hicule et passage en mode circulation en zone.
            pZoneDest->ReaffecteVehicule(pVehicle.get(), dbInstant);
        }
		else
		{
			// Appel de la mÃ©thode de la classe mÃ¨re
			AbstractFleet::OnCurrentLegFinished(pVehicle,  pDestinationEnterLane, dbInstDestinationReached, dbInstant, dbTimeStep);
		}
    }
}

// Met Ã  jour le Trip en fonction des tuyaux parcourus
void SymuViaFleet::SetLinkUsed(double dbInstant, Vehicule * pVeh, Tuyau * pLink)
{
    // Appel de la mÃ©thode de la classe mÃ¨re
    AbstractFleet::SetLinkUsed(dbInstant, pVeh, pLink);

    SymuViaFleetParameters * pParams = (SymuViaFleetParameters*)pVeh->GetFleetParameters();

    ZoneDeTerminaison* pZoneDest = dynamic_cast<ZoneDeTerminaison*>(pVeh->GetDestination());

    // dÃ©tection de l'arrivÃ©e en zone
    if(pParams->IsTerminaison() && pZoneDest && pParams->GetInstantEntreeZone() == -1 && pZoneDest->GetInputPosition().IsInZone(pVeh->GetLink(0)))
    {
        double dbDistanceParcourue = pVeh->GetPos(0); // approx si un tronÃ§on a Ã©tÃ© sautÃ©...
        pParams->InitializeZoneTravel(dbInstant - (dbDistanceParcourue / pVeh->GetVit(0)), pZoneDest->GetInputID(), dbDistanceParcourue, pVeh->GetLink(0)->GetCnxAssAm());
    }

    
    // Gestion de la recherche de stationnement surfacique
    if (pParams->IsTerminaisonSurfacique() && pParams->IsRechercheStationnement())
    {
        assert(pZoneDest);
        assert(!pParams->IsTerminaisonResidentielle());

        // Si recherche de stationnement surfacique, on note le tuyau en tant que tuyau essayÃ©
        if (!pLink->GetBriqueParente())
        {
            pParams->GetLstTuyauxDejaParcourus().insert(pLink);
        }

        bool bIsModeSurfaciqueSimplifie = pZoneDest->IsModeSurfaciqueSimplifie(pParams->GetPlaqueDestination(), dbInstant);

        bool bDeleteVehicle = false;
        // Si on a du stock et qu'on est en mode normal,
        // ou si on est en mode simplifiÃ© et que la durÃ©e de recherche est Ã©coulÃ©e, on se gare :
        if ((!bIsModeSurfaciqueSimplifie && (pLink->GetCapaciteStationnement() == -1 || pLink->GetStockStationnement() < pLink->GetCapaciteStationnement()))
            || (bIsModeSurfaciqueSimplifie && pParams->GetDistanceRechercheParcourue() >= pParams->GetGoalDistance()))
        {
            // le vÃ©hicule se gare :
            if (!bIsModeSurfaciqueSimplifie)
            {
                pLink->IncStockStationnement(pVeh->GetLength());
            }
            else
            {
                pZoneDest->IncStockStationnement(pVeh);
            }

            bDeleteVehicle = true;
        }
        // si le temps est Ã©coulÃ©, on rÃ©arbitre
        else if (pParams->IsRearbitrageNeeded(dbInstant, pZoneDest))
        {
            pVeh->SetEndCurrentLeg(true);
            pParams->SetDoReaffecteVehicule(true);
        }
        // si le temps est Ã©coulÃ© pour le rÃ©arbitrage de la distance uniquement en mdo simplifiÃ©, on la met Ã  jour :
        else if (bIsModeSurfaciqueSimplifie && pParams->IsGoalDistanceUpdateNeeded(dbInstant, pZoneDest))
        {
            // MAJ de la distance :
            pZoneDest->UpdateGoalDistance(dbInstant, pParams);
        }
        // gestion de la durÃ©e max de recherche de stationnement surfacique :
        else if (pParams->IsMaxSurfaciqueDureeReached(dbInstant, pZoneDest))
        {
            bDeleteVehicle = true;

            // rmk : pour Ã©viter l'avertissement pour les vÃ©hicules dÃ©jÃ  dÃ©truits...
            if (pVeh->GetLink(0))
            {
                m_pNetwork->log() << Logger::Warning << "Maximum duration of parking search elapsed without success : vehicle " << pVeh->GetID() << " destroyed." << std::endl;
                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
            }
        }

        if (bDeleteVehicle)
        {
            // Destruction du vÃ©hicule
            pVeh->SetTuyau(NULL, dbInstant);
            pVeh->SetVoie(NULL);
            pVeh->SetDejaCalcule(true);
            pVeh->SetInstantSortie(dbInstant);

            // Pour Ãªtre sÃ»r qu'on ne le rÃ©affecte plus :
            pVeh->SetEndCurrentLeg(false);

            // Gestion de la demande autogÃ©rÃ©nÃ©e Ã  cause du stationnement :
            pZoneDest->GestionDemandeStationnement(pParams->GetPlaqueDestination(), dbInstant);
        }
    }

    // gestion du rÃ©arbitrage quand on arrive devant un parking plein :
    if (pParams->IsTerminaison() && !pParams->IsTerminaisonSurfacique())
    {
        Parking * pParkingDest = (Parking*)pVeh->GetTrip()->GetFinalDestination();
        if (pParkingDest->IsFull())
        {
            for (size_t iInputParkLink = 0; iInputParkLink < pParkingDest->GetInputConnexion()->m_LstTuyAssAm.size(); iInputParkLink++)
            {
                if (pParkingDest->GetInputConnexion()->m_LstTuyAssAm.at(iInputParkLink)->GetCnxAssAm() == pLink->GetCnxAssAv())
                {
                    pVeh->SetEndCurrentLeg(true);
                    pParams->SetDoReaffecteVehicule(true);
                    break;
                }
            }
        }
    }
}

void SymuViaFleet::OnVehicleActivated(boost::shared_ptr<Vehicule> pVeh, double dbInstant)
{
    // rmq. dans le cas de cette flotte, les opÃ©rations d'ajout au doc trafic, par exemple,
    // sont gÃ©rÃ©es au cas par cas : on ne fait que l'association entre le vÃ©hicule et la flotte ici
    DoVehicleAssociation(pVeh);
}

std::map<std::string, std::string> SymuViaFleet::GetOutputAttributesAtEnd(Vehicule * pV)
{
    std::map<std::string, std::string> additionalAttributes;

    SymuViaFleetParameters * pParams = (SymuViaFleetParameters*)pV->GetFleetParameters();

    if(pParams->GetInstantEntreeZone() != -1)
    {
        additionalAttributes["zone_id"] = pParams->GetZoneId();
        // On prend le temps de sortie du vÃ©hicule si renseignÃ©, sinon l'instant courant (cas du vÃ©hicule qui reste sur le rÃ©seau aprÃ¨s la fin de la simu)
        double dbDuree = (pV->GetExitInstant() == -1 ? pV->GetReseau()->m_dbInstSimu : pV->GetExitInstant())-pParams->GetInstantEntreeZone();
        additionalAttributes["zone_tps"] = SystemUtil::ToString(2, dbDuree);
        additionalAttributes["zone_dst"] = SystemUtil::ToString(2, pParams->GetDistanceParcourue());
    }
    
    return additionalAttributes;
}

template void SymuViaFleet::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void SymuViaFleet::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void SymuViaFleet::serialize(Archive& ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(AbstractFleet);
}

