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
#include "SymuViaTripNode.h"

#include "RandManager.h"
#include "tuyau.h"
#include "Parking.h"
#include "ZoneDeTerminaison.h"
#include "TronconOrigine.h"
#include "../Affectation.h"
#include "Xerces/XMLUtil.h"
#include "DiagFonda.h"
#include "reseau.h"
#include "vehicule.h"
#include "SystemUtil.h"
#include "voie.h"
#include "TuyauMeso.h"
#include "vehicule.h"
#include "Logger.h"
#include "TraceDocTrafic.h"
#include "RepartitionTypeVehicule.h"
#include "usage/Position.h"
#include "usage/Trip.h"
#include "usage/TripLeg.h"
#include "usage/SymuViaFleet.h"
#include "usage/SymuViaFleetParameters.h"
#include "RepMotif.h"
#include "Motif.h"

#include <xercesc/dom/DOMNode.hpp>

#include <boost/make_shared.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>

using namespace std;

XERCES_CPP_NAMESPACE_USE



SymuViaTripNode::SymuViaTripNode()
: TripNode()
{
    Nb_variations=0;

    m_bDemande = true;
    m_bDistribution = false;

    m_dbLastInstCreation = 0;
    m_dbCritCreationVeh = 0;

    m_pLstRepTypeVeh = new RepartitionTypeVehicule();

    m_pTypeVehiculeACreer = NULL;
    m_pDestinationVehiculeACreer = NULL;
    m_nVoieACreer = 0;
}

SymuViaTripNode::SymuViaTripNode(const std::string & strID, Reseau * pNetwork)
 : TripNode(strID, pNetwork)
{
    Nb_variations=0;

    m_bDemande = true;
    m_bDistribution = false;

    m_dbLastInstCreation = 0;
    m_dbCritCreationVeh = 0;

    m_pLstRepTypeVeh = new RepartitionTypeVehicule();

    m_pTypeVehiculeACreer = NULL;
    m_pDestinationVehiculeACreer = NULL;
    m_nVoieACreer = 0;
}


SymuViaTripNode::~SymuViaTripNode(void)
{
    std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > >::iterator iter;
    for(iter = m_LstCoeffDest.begin(); iter != m_LstCoeffDest.end(); iter++)
    {
        EraseListOfVariation(&iter->second);
    }
    for(iter = m_LstCoeffDestInit.begin(); iter != m_LstCoeffDestInit.end(); iter++)
    {
        EraseListOfVariation(&iter->second);
    }

    // Destruction de la liste du nombre de vÃ©hicules
    m_dqNbVehicules.erase(m_dqNbVehicules.begin(),m_dqNbVehicules.end());    

    DeleteLogMatriceOD();
    std::map<TypeVehicule*, ListOfTimeVariation<tracked_double> *>::iterator itDem;
    
    for (itDem = m_LTVDemande.begin(); itDem !=m_LTVDemande.end(); itDem++)
    { 
        delete itDem->second;
    }
	
    m_LTVDemande.erase(m_LTVDemande.begin(), m_LTVDemande.end());

    for (itDem = m_LTVDemandeInit.begin(); itDem !=m_LTVDemandeInit.end(); itDem++)
    { 
        delete itDem->second;
    }
    m_LTVDemandeInit.erase(m_LTVDemandeInit.begin(), m_LTVDemandeInit.end());
 

   
    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >::iterator itRepVoie;
    for( itRepVoie = m_lstRepVoie.begin(); itRepVoie != m_lstRepVoie.end(); itRepVoie++)
    {
        EraseListOfVariation(itRepVoie->second);
        delete itRepVoie->second;
    }
    m_lstRepVoie.erase(m_lstRepVoie.begin(), m_lstRepVoie.end());
  
    for( itRepVoie = m_lstRepVoieInit.begin(); itRepVoie != m_lstRepVoieInit.end(); itRepVoie++)
    {
        EraseListOfVariation(itRepVoie->second);
        delete itRepVoie->second;
    } 
    m_lstRepVoieInit.erase( m_lstRepVoieInit.begin(), m_lstRepVoieInit.end());

    delete m_pLstRepTypeVeh;

    std::map<TypeVehicule*, std::deque<TimeVariation<CRepMotif> > >::iterator iterMotif;
    for (iterMotif = m_lstRepMotif.begin(); iterMotif != m_lstRepMotif.end(); iterMotif++)
    {
        EraseListOfVariation(&iterMotif->second);
    }
    for (iterMotif = m_lstRepMotifInit.begin(); iterMotif != m_lstRepMotifInit.end(); iterMotif++)
    {
        EraseListOfVariation(&iterMotif->second);
    }
}

//================================================================
std::map<TypeVehicule*, std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> > > & SymuViaTripNode::GetMapAssignment
//================================================================
()
{
    return GetOutputConnexion()->m_mapAssignment;
}

Connexion* SymuViaTripNode::GetInputConnexion()
{
    return m_InputPosition.GetConnection();
}

Connexion* SymuViaTripNode::GetOutputConnexion()
{
    return m_OutputPosition.GetConnection();
}

void SymuViaTripNode::AddCreationFromStationnement(double dbInstantCreation)
{
    m_lstSortiesStationnement.push_back(dbInstantCreation);
    std::sort(m_lstSortiesStationnement.begin(), m_lstSortiesStationnement.end());
}


//================================================================
    void SymuViaTripNode::Initialize
//----------------------------------------------------------------
// Fonction  : Initialisation des varaibles de simulation de l'entrÃ©e
// Version du: 07/05/2009
// Historique: 07/05/2009 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    std::deque< TimeVariation<TraceDocTrafic> > & docTrafics
)
{
    Connexion * pConnexion = GetOutputConnexion();
    if(pConnexion)
    {
        // Initialisation des listes        
        m_dqNbVehicules.clear();
        std::deque<int*>	m_LogMatriceOD(0);

	    if( pConnexion->m_LstTuyAv.size()==0)
		    return;

	    if( !pConnexion->m_LstTuyAv.front())
		    return;

	    m_mapVehEnAttente.clear();
	    m_mapVehPret.clear();

    
        for(size_t nTuyau = 0; nTuyau < pConnexion->m_LstTuyAssAv.size(); nTuyau++)
	    {
	        for(int nVoie = 0; nVoie < pConnexion->m_LstTuyAssAv[nTuyau]->getNb_voies(); nVoie++)
	        {
		        m_mapVehEnAttente[pConnexion->m_LstTuyAssAv[nTuyau]].insert( pair<int, std::deque<boost::shared_ptr<Vehicule> > > (nVoie, std::deque<boost::shared_ptr<Vehicule>>(0) ) );
		        m_mapVehPret[pConnexion->m_LstTuyAssAv[nTuyau]].insert( pair<int, int> (nVoie, -1) );
	        }	
        }

        m_dbCritCreationVeh = 0;
        m_dbLastInstCreation = 0;
    }
}

//================================================================
    void SymuViaTripNode::CreationVehicules
//----------------------------------------------------------------
// Fonction  : Module de crÃ©ation des vÃ©hicules
// Version du: 16/11/2009
// Historique: 16/11/2009 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
	const double dbInst,
	std::vector<boost::shared_ptr<Vehicule>> & listInfoVeh
)
{

    // Gestion des vÃ©hicules qui re-sortent de stationnement
    size_t iNbParkingOutputs = 0;
    for (size_t iParkingOutput = 0; iParkingOutput < m_lstSortiesStationnement.size(); iParkingOutput++)
    {
        double dbOutputTime = m_lstSortiesStationnement.at(iParkingOutput);
        if (dbInst >= dbOutputTime)
        {
            boost::shared_ptr<Vehicule> pVeh = GenVehicule(-1, NULL, -1, dbInst, 0, NULL, NULL, NULL, NULL, NULL, true);
            if (pVeh)
            {
                listInfoVeh.push_back(pVeh);
            }
            iNbParkingOutputs++;
        }
        else
        {
            break;
        }
    }
    if (iNbParkingOutputs > 0)
    {
        m_lstSortiesStationnement.erase(m_lstSortiesStationnement.begin(), m_lstSortiesStationnement.begin() + iNbParkingOutputs);
    }

	double	dbInstCreat;			// Instant de crÃ©ation au cours du pas de temps
	bool	bCreation;

    double dbNextVariationTime;

    bool bContinue = true;
    while(bContinue)
	{
        TypeVehicule* pTV = NULL;
        bCreation = false;
        bool bDoCriterionUpdate = false;

        if(m_bDemande)
        {
            // Demande stochastique
		    if(m_bDistribution)
		    {
                bCreation = IsVehiculeDisponible() && (m_dbCritCreationVeh <= dbInst);

			    if(bCreation)
			    {
				    dbInstCreat = m_dbCritCreationVeh - (dbInst - m_pNetwork->GetTimeStep()) ;
                    if (dbInst - m_pNetwork->GetTimeStep() + dbInstCreat >= 0)
                    {
                        UpdateCritCreationVeh(dbInst - m_pNetwork->GetTimeStep() + dbInstCreat, true);
                    }
			    }
		    }
            // Demande dÃ©terministe
            else
            {
                bCreation = IsVehiculeDisponible() && (m_dbCritCreationVeh >= 1);

			    if( bCreation )
			    {
                    if (GetDemandeValue(dbInst, dbNextVariationTime) != 0.0)
                    {
                        dbInstCreat = m_pNetwork->GetTimeStep() - (m_dbCritCreationVeh - 1) * m_pNetwork->GetTimeStep() /
                            (GetDemandeValue(dbInst, dbNextVariationTime) * m_pNetwork->GetTimeStep());
				        m_dbCritCreationVeh -= 1;
                    }
			    }
            }
        }
        // Liste de vÃ©hicule Ã  crÃ©er
        else
        {
            bCreation = IsVehiculeDisponible() && (m_dbCritCreationVeh <= dbInst);			

			if(bCreation)
			{
                dbInstCreat = m_dbCritCreationVeh - (dbInst - m_pNetwork->GetTimeStep());
                // Cas particulier : la fonction UpdateCritCreationVeh positionne des variables membres
                // pour la crÃ©ation du vÃ©hicule suivant : on reporte le UpdateCritCreationVeh Ã  la fin de cette mÃ©thode
                // pour que ces variables restent valides le temps de la crÃ©ation du vÃ©hicule courant
                bDoCriterionUpdate = true;
			}
        }

		if(bCreation)
		{
            boost::shared_ptr<Vehicule> pVeh = GenVehicule(-1, NULL, -1, dbInst, dbInstCreat, NULL, NULL, NULL, NULL, NULL, false);
            if (pVeh)
            {
                listInfoVeh.push_back(pVeh);
            }
        }
        else
        {
            // On s'arrÃªte Ã  la premiÃ¨re itÃ©ration qui ne provoque pas de crÃ©ation de vÃ©hicule : les suivantes n'en crÃ©eront pas non plus.
            bContinue = false;
        }

        if(bDoCriterionUpdate)
        {
            UpdateCritCreationVeh(dbInst + m_pNetwork->GetTimeStep(), true);
        }
    }
}


//================================================================
boost::shared_ptr<Vehicule> SymuViaTripNode::GenVehicule
//----------------------------------------------------------------
// Fonction  : 
// Version du: 
// Historique: 
//             
//================================================================
(
    int             nID,
    TypeVehicule	*pTV,
    int				nVoie,
    double			dbInst,
    double			dbInstCreat,
    SymuViaTripNode *pDst,
    std::vector<Tuyau*> * pIti,
    Connexion * pJunction,
    CPlaque * pPlaqueOrigin,
    CPlaque * pPlaqueDestination,
    bool bForceNonResidential
)
{
    boost::shared_ptr<Vehicule> pVehicule;

    Connexion * pConnexion = GetOutputConnexion();
    Tuyau * pOutputLink = NULL;
    if (pConnexion)
    {
        // Si un itinÃ©raire est imposÃ©, le premier tuyau est le premier de l'itinÃ©raire.
        if (pIti && !pIti->empty())
        {
            pOutputLink = pIti->front();
        }
        // pConnexion->m_LstTuyAssAv.empty() peut arriver dans le cas d'une origine zone directement en contact avec la sortie destination
        else if (!pConnexion->m_LstTuyAssAv.empty())
        {
            pOutputLink = pConnexion->m_LstTuyAssAv.front();
        }
    }
    else
    {
        pOutputLink = m_OutputPosition.GetLink();
    }

    if (nVoie == -1)
    {
        nVoie = CalculNumVoie(dbInst, pOutputLink);
    }

    // Calcul du type de vÃ©hicule
    if (pTV == NULL)
    {
        pTV = CalculTypeNewVehicule(dbInst, nVoie);
    }

    bool bWarnODMatrixForParkingOutput = false;

    // Pas de crÃ©ation de vÃ©hicule si besoin matrice OD et non dÃ©finie
    // bug nÂ°61 : pas de crÃ©ation si liste de vÃ©hicules Ã  crÃ©er et pas de matrice OD dÃ©finie. On ignore
    // cette condition en cas de liste de vÃ©hicules Ã  crÃ©er.
    if (m_pNetwork->IsUsedODMatrix() && (m_LstCoeffDestInit.find(pTV) == m_LstCoeffDestInit.end() || m_LstCoeffDestInit[pTV].empty())
        && m_LstCoeffDestInit.find(NULL) == m_LstCoeffDestInit.end() && m_LstCoeffDestInit.size() == 1 && m_bDemande  // et de pas de matrix global
        && !pDst) // et si on a une destination dÃ©finie, pas besoin de la matrice OD de totue faÃ§on (cas des vÃ©hicules gÃ©nÃ©rÃ©s Ã  la demande par SymuMaster par exemple)
    {
        bWarnODMatrixForParkingOutput = bForceNonResidential;
    }
    else
    {
        if (pTV)
        {
            pVehicule = boost::make_shared<Vehicule>(m_pNetwork, nID == -1 ? m_pNetwork->IncLastIdVeh() : nID, pTV->GetVx(), pTV, m_pNetwork->GetTimeStep());
            pVehicule->InitializeCarFollowing();

            SymuViaFleetParameters * pFleetParams = (SymuViaFleetParameters*)pVehicule->GetFleetParameters();
            if (pFleetParams == NULL)
            {
                pFleetParams = new SymuViaFleetParameters();
                pVehicule->SetFleetParameters(pFleetParams);
            }
            pFleetParams->SetPlaqueDestination(pPlaqueDestination);
            pFleetParams->SetPlaqueOrigin(pPlaqueOrigin);

            // DÃ©termination du tuyau et voie de crÃ©ation

            if (pDst == NULL)
            {
                // Tirage a priori de la destination
                if (m_pNetwork->IsUsedODMatrix())
                {
                    pDst = GetDestination(dbInst - m_pNetwork->GetTimeStep() + dbInstCreat, nVoie, pTV);
                    // Evolution nÂ°106 : on peut ne pas avoir de destination de dÃ©finie !
                    if (!pDst)
                    {
                        m_pNetwork->log() << Logger::Warning << "No defined destination for origin " << GetOutputID() << " at instant " << dbInst << ". The requested demand level won't be reached." << std::endl;
                        m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                        return NULL;
                    }
                }
            }

            // Tirage du motif de dÃ©placement si une distribution est dÃ©finie
            CMotifCoeff * pMotifCoeff = GetMotif(dbInst - m_pNetwork->GetTimeStep() + dbInstCreat, pTV, pDst);
            std::string motifOrigine, motifDestination;
            if (pMotifCoeff)
            {
                motifOrigine = pMotifCoeff->GetMotifOrigine()->GetID();
                motifDestination = pMotifCoeff->GetMotifDestination()->GetID();
                pFleetParams->SetMotif(pMotifCoeff);
            }

            // En fonction du type, on dÃ©termine le bon tuyau aval de l'origine sur lequel positionner le vÃ©hicule
            double dbCreationPosition = UNDEF_POSITION;
            Tuyau * pTuy = NULL;

            Parking * pParking = NULL;
            bool bSortieStationnement = false;
            ZoneDeTerminaison * pZone = dynamic_cast<ZoneDeTerminaison*>(this);

            // Instanciation du TripLeg correspondant
            TripLeg * pTripLeg = new TripLeg();
            pTripLeg->SetDestination(pDst);

            // Calcul de sa destination si besoin
            if (m_pNetwork->IsUsedODMatrix())
            {
                // Gestion des itinÃ©raires ?
                if (m_pNetwork->GetAffectationModule())
                {
                    if (m_pNetwork->GetAffectationModule()->GetLieuAffectation() == 'E')
                    {
                        std::deque<AssignmentData> dqAssData;

                        bool bEmptyItinerary = false;

                        // RÃ©cupÃ©ration des itinÃ©raires possibles avec leurs coefficients d'affectation
                        if (!pIti)
                        {
                            bool bItineraires = GetItineraires(pTV, pDst, dqAssData);

                            // rmq. : calcul de l'itinÃ©raire automatique dÃ©sactivÃ© en cas de tronÃ§on origine car ne fonctionne pas tel que codÃ© actuellement
                            // Ã  voir si ca nÃ©cessite d'Ãªtre fait.
                            if (!bItineraires)
                            {
                                // Si pas d'itinÃ©raire connu, de deux choses l'une :
                                // - soit l'utilisateur fait une erreur (il crÃ©e un vÃ©hicule avec une destination impossible
                                // Ã  atteindre Ã  partir de cette origine
                                // - soit il s'agit d'un calcul d'itinÃ©raire qui a Ã©tÃ© ignorÃ© lors du prÃ©cÃ©dent calcul d'affectation
                                // (pas de demande Ã  l'origine, par exemple).
                                // Du coup, on gÃ¨re le cas 2 par une relance du calcul d'affectation.
                                // Le cas 1 n'est pas gÃ©rÃ© de faÃ§on optimale d'un point de vue des performances,
                                // mais ce n'est pas grave car n'est pas sensÃ© arriver
                                m_pNetwork->GetAffectationModule()->Run(m_pNetwork, dbInst, 'A', false, pVehicule->GetType(), pDst, this);
                                bItineraires = GetItineraires(pTV, pDst, dqAssData);
                            }
                            if (bItineraires)
                            {
                                // Tirage de l'itinÃ©raire							
                                double dbSum = 0;
                                double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;
                                bool bItiFound = false;

                                // DÃ©termination de l'itinÃ©raire Ã©ligible							    							
                                for (int i = 0; i < (int)dqAssData.size(); i++)
                                {
                                    dbSum += dqAssData[i].dbCoeff;

                                    if (dbRand <= dbSum)
                                    {
                                        pTripLeg->SetPath(dqAssData[i].dqTuyaux);
                                        pFleetParams->SetRouteId(dqAssData[i].strRouteId);
                                        bItiFound = true;

                                        // on a notre tuyau d'origine comme Ã©tant le premier de l'itinÃ©raire
                                        if (!dqAssData[i].dqTuyaux.empty())
                                        {
                                            pTuy = dqAssData[i].dqTuyaux.front();
                                        }
                                        else
                                        {
                                            bEmptyItinerary = true;
                                            pJunction = dqAssData[i].pJunction;
                                        }
                                        break;
                                    }
                                }

                                if (!bItiFound)
                                {
                                    m_pNetwork->log() << Logger::Warning << "The sum of the assignment coefficients for OD " << this->GetOutputID() << " => " << pDst->GetInputID() << " ("
                                        << std::setprecision(std::numeric_limits<double>::digits10) << dbSum << ") is lower than the try (" << dbRand << "). No path could be assigned to the vehicle " << pVehicule->GetID() << std::endl;
                                    m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                }
                            }
                            else
                            {
                                m_pNetwork->log() << Logger::Warning << "No path found to destination " << pDst->GetInputID() << " for vehicle " << pVehicule->GetID() << std::endl;
                                m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                            }
                        }
                        else
                        {
                            // Cas de l'itinÃ©raire imposÃ©
                            bEmptyItinerary = pIti->empty();
                            pTripLeg->SetPath(*pIti);
                            if (!pTuy)
                            {
                                pTuy = pOutputLink;
                            }
                        }

                        // Traitement cas particulier d'une route vide (sans tuyaux)
                        if (bEmptyItinerary)
                        {
                            // Pour gestion des routes vides (l'origine et la destination se touchent. On a soit une zone d'origine soit une zone de destination soit les deux) :
                            // Si l'origine n'est pas une zone, on prend comme tuyau de crÃ©ation le tuyau aval Ã  l'extremitÃ©
                            // Si l'origine est une zone, on prend arbitrairement dans ce cas comme tuyau de crÃ©ation du vÃ©hicule :
                            // - le tuyau amont Ã  la destination s'il n'y en a qu'un
                            // - si plusieurs tuyaux amont Ã  la destination, on prend le premier qu'on trouve qui appartienne Ã  la zone d'origine et qui est un tuyau amont Ã  la destination.
                            // Si un noeud est dÃ©fini dans la route prÃ©dÃ©finie, on en tient compte dans la recherche avec warning si le noeud ne convient pas.

                            if (!pZone)
                            {
                                pTuy = pOutputLink;
                            }
                            else
                            {
                                // Cas particulier de la zone origine et destination pour un mÃªme vÃ©hicule !!
                                if (pDst == pZone)
                                {
                                    // dans ce cas, on tire simplement un tuyau dans la zone pour crÃ©er le vÃ©hicule et on demande Ã  la zone
                                    // de gÃ©rer le dÃ©part en zone dans ce cas particulier :
                                    pTuy = pZone->AddInitialPathToLeg(dbInst, pFleetParams, pPlaqueOrigin, pMotifCoeff, pTuy, pTV, pTripLeg, bForceNonResidential, dbCreationPosition, pParking, bSortieStationnement);
                                    if (!pTuy)
                                    {
                                        return NULL;
                                    }
                                }
                                else
                                {
                                    if (pDst->GetInputConnexion()->m_LstTuyAssAm.size() == 1)
                                    {
                                        pTuy = pDst->GetInputConnexion()->m_LstTuyAssAm.front();
                                    }
                                    else
                                    {
                                        std::set<Tuyau*, LessPtr<Tuyau> > candidateLinks;
                                        for (size_t iTuy = 0; iTuy < pDst->GetInputConnexion()->m_LstTuyAssAm.size(); iTuy++)
                                        {
                                            Tuyau * pCandidateLink = pDst->GetInputConnexion()->m_LstTuyAssAm[iTuy];
                                            if (pZone->GetOutputPosition().IsInZone(pCandidateLink) && !pCandidateLink->IsInterdit(pTV))
                                            {
                                                if (!pJunction)
                                                {
                                                    candidateLinks.insert(pCandidateLink);
                                                }
                                                else if (pCandidateLink->GetCnxAssAv() == pJunction)
                                                {
                                                    candidateLinks.insert(pCandidateLink);
                                                }
                                            }
                                        }

                                        double dbSum = 0;
                                        double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

                                        for (std::set<Tuyau*, LessPtr<Tuyau> >::iterator iterCandidate = candidateLinks.begin(); iterCandidate != candidateLinks.end(); ++iterCandidate)
                                        {
                                            dbSum += 1.0 / (double)candidateLinks.size();

                                            if (dbRand <= dbSum)
                                            {
                                                pTuy = *iterCandidate;
                                                break;
                                            }
                                        }
                                        // Pour protection contre erreurs d'arrondi
                                        if (!pTuy && !candidateLinks.empty())
                                        {
                                            pTuy = *candidateLinks.begin();
                                        }

                                        if (!pTuy && pJunction)
                                        {
                                            m_pNetwork->log() << Logger::Warning << "The junction " << pJunction->GetID() << " doesn't connect origin " << this->GetOutputID() << " with destination " << pDst->GetInputID()
                                                << ". The vehicle " << pVehicule->GetID() << " will not have the desired path." << std::endl;
                                            m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                                        }
                                    }
                                }
                            }
                        } // Fin de la gestion d'un itinÃ©raire vide

                        // Gestion du motif si zone
                        if (pZone && pTuy)
                        {
                            // Calcul de l'itinÃ©raire entre la plaque et le dÃ©but de l'itinÃ©raire "pÃ©riphÃ©rique")
                            pTuy = pZone->AddInitialPathToLeg(dbInst, pFleetParams, pPlaqueOrigin, pMotifCoeff, pTuy, pTV, pTripLeg, bForceNonResidential, dbCreationPosition, pParking, bSortieStationnement);
                            if (!pTuy)
                            {
                                // Avortement de la crÃ©ation de vÃ©hicule (pas de stock de stationnement disponible par exemple, et pas l'option pour crÃ©er mÃªme si pas de stock)
                                return NULL;
                            }
                        }
                    }
                }
            }

            if (!pTuy)
            {
                pTuy = pOutputLink;
            }

            if (pTuy->GetType() == Tuyau::TT_MICRO)
            {
                pVehicule->SetVoie((VoieMicro*)pTuy->GetLstLanes()[nVoie]);
                pVehicule->SetTuyau(pTuy, dbInst);
            }
            else
            {
                pVehicule->SetVoie((VoieMicro*)pTuy->GetLstLanes()[nVoie]);
            }

            if (pTuy->GetType() == Tuyau::TT_MESO && pConnexion)
            {
                SymuViaTripNode *pRealOrigin = pParking == NULL ? this : pParking;

                int nVehWainting = 0;
                if (pRealOrigin && pRealOrigin->m_mapVehEnAttente.find(pTuy) != pRealOrigin->m_mapVehEnAttente.end())
                {
                    std::map<int, std::deque<boost::shared_ptr<Vehicule>> >::iterator itAttBegin = pRealOrigin->m_mapVehEnAttente[pTuy].begin();
                    std::map<int, std::deque<boost::shared_ptr<Vehicule>> >::iterator itAtt;
                    std::map<int, std::deque<boost::shared_ptr<Vehicule>> >::iterator itAttEnd = pRealOrigin->m_mapVehEnAttente[pTuy].end();
                    for (itAtt = itAttBegin; itAtt != itAttEnd; itAtt++)
                    {
                        nVehWainting += (int)itAtt->second.size();
                    }

                }
                if (pConnexion->GetNextEventTime() > dbInst &&
                    nVehWainting == 0 &&
                    ((CTuyauMeso*)pTuy)->GetNextSupplyTime() != DBL_MAX/* &&
                    ((CTuyauMeso*)pTuy)->IsCongested() == false*/)
                {
                    m_pNetwork->ChangeMesoNodeEventTime(pConnexion, dbInst, NULL);
                    pConnexion->SetNextArrival(NULL, std::pair<double, Vehicule*>(dbInst, pVehicule.get()));
                }

                pConnexion->AddArrival(NULL, dbInst, pVehicule.get(), true);

                //   pConnexion->AddCreatedVehicule((CTuyauMeso*)pTuy,pVehicule.get());
            }
            pVehicule->SetGenerationPosition(dbCreationPosition);
            pVehicule->SetInstantCreation(dbInstCreat);
            pVehicule->SetHeureEntree(dbInst - m_pNetwork->GetTimeStep() + dbInstCreat);
            pVehicule->SetVitRegTuyAct((pTuy)->GetVitRegByTypeVeh(pTV, dbInst, dbCreationPosition,
                pVehicule->GetVoie(0) ? pVehicule->GetVoie(0)->GetNum() : 0));

            // AgressivitÃ©
            if (m_pNetwork->IsProcAgressivite())
                pVehicule->SetAgressif(IsAgressif(pTV));

            pVehicule->TirageJorge();

            Trip * pTrip = new Trip();
            pTrip->SetOrigin(this);
            pTrip->AddTripLeg(pTripLeg);
            pVehicule->SetTrip(pTrip);
            pVehicule->MoveToNextLeg();

            // Mise Ã  jour de la liste des vÃ©hicules du doc XML				
            const std::string & ssType = pVehicule->GetType()->GetLabel();
            const std::string & ssGMLType = pVehicule->GetType()->GetGMLLabel();
            std::string ssEntree = GetOutputID();
            const std::string & ssSortie = pVehicule->GetDestination() ? pVehicule->GetDestination()->GetInputID() : "";
            const std::string & ssZoneEntree = pZone ? pZone->GetID() : "";
            ZoneDeTerminaison * pZoneDest = dynamic_cast<ZoneDeTerminaison*>(pVehicule->GetDestination());
            const std::string & ssZoneSortie = pZoneDest ? pZoneDest->GetID() : "";

            std::vector<std::string> initialPath;
            for (size_t iTuy = 0; iTuy < pVehicule->GetItineraire()->size(); iTuy++)
            {
                initialPath.push_back(pVehicule->GetItineraire()->at(iTuy)->GetLabel());
            }

            SymuViaTripNode * pRealOrigin = this;
            if (pParking)
            {
                pRealOrigin = pParking;
                ssEntree = pParking->GetOutputID();
            }
            else if (pZone || !pConnexion)
            {
                // Dans les modes oÃ¹ l'origine est un tuyau, on affecte Ã  l'attribut entrÃ©e le nom du noeud amont Ã  ce tuyau
                ssEntree = pTuy->GetCnxAssAm()->GetID();
            }

            // ETS 140926 
            m_pNetwork->AddVehiculeToDocTrafic(pVehicule->GetID(), "", ssType, ssGMLType,
                pVehicule->GetDiagFonda()->GetKMax(), pVehicule->GetDiagFonda()->GetVitesseLibre(),
                pVehicule->GetDiagFonda()->GetW(), ssEntree, ssSortie, ssZoneEntree, ssZoneSortie, pFleetParams->GetRouteId(),
                dbInst - m_pNetwork->GetTimeStep() + dbInstCreat, "", pVehicule->IsAgressif(),
                nVoie, "", initialPath, pVehicule->GetType()->GetLstPlagesAcceleration(), motifOrigine, motifDestination);

            // ajout si besoin des listes de vÃ©hicules prÃªts et en attente pour le tronÃ§on initial
            if (pConnexion)
            {
                if (pRealOrigin->m_mapVehEnAttente.find(pTuy) == pRealOrigin->m_mapVehEnAttente.end())
                {
                    for (int iLane = 0; iLane < pTuy->getNb_voies(); iLane++)
                    {
                        pRealOrigin->m_mapVehEnAttente[pTuy].insert(pair<int, std::deque<boost::shared_ptr<Vehicule> > >(iLane, std::deque<boost::shared_ptr<Vehicule>>(0)));
                    }
                }
                if (pRealOrigin->m_mapVehPret.find(pTuy) == pRealOrigin->m_mapVehPret.end())
                {
                    for (int iLane = 0; iLane < pTuy->getNb_voies(); iLane++)
                    {
                        pRealOrigin->m_mapVehPret[pTuy].insert(pair<int, int>(iLane, -1));
                    }
                }

                // Ajout dans la liste des vÃ©hicules en attente				
                if (pTuy->GetType() != Tuyau::TT_MESO &&
                    ((pRealOrigin->m_mapVehPret[pTuy].find(nVoie)->second != -1 || pRealOrigin->m_mapVehEnAttente[pTuy].find(nVoie)->second.size() > 0)
                    || !pRealOrigin->CanInsertVehicle(dbInst, m_dbLastInstCreation)))
                {
                    pRealOrigin->m_mapVehEnAttente[pTuy].find(nVoie)->second.push_back(pVehicule);
                    // *Reseau::m_pFicSimulation<< "Veh en attente " << pVehicule->GetID();
                }
                else
                {
                    if (pTuy->GetType() == Tuyau::TT_MESO)// && 
                        //((CTuyauMeso *)pTuy)->IsCongested() )
                    {
                        pRealOrigin->m_mapVehEnAttente[pTuy].find(nVoie)->second.push_back(pVehicule);
                    }
                    else
                    {
                        pRealOrigin->m_mapVehPret[pTuy].find(nVoie)->second = pVehicule->GetID();
                        m_pNetwork->AddVehicule(pVehicule);
                        pRealOrigin->VehiculeExit(pVehicule);
                    }
                    // *Reseau::m_pFicSimulation<< "Veh pret " << pVehicule->GetID();
                }
            }
            else
            {
                // Cas des origines de type tronÃ§on : pas de gestion de l'insertion pour le moment avec GenVehicules) sans doute amÃ©liorable
                // mais ca implique de rajouter des origines dans la liste des origines du rÃ©seau pour gÃ©rer l'insertion comme pour les origines classiques...
                m_pNetwork->AddVehicule(pVehicule);
            }

            if (bSortieStationnement)
            {
                // Cas de la zone en mode surfacique : on gÃ¨re le stock de vÃ©hicules stationnÃ©s sur le tronÃ§on d'origine
                pTuy->DecStockStationnement(pVehicule->GetLength()); // on enlÃ¨ve la longueur correspondant au vÃ©hicule en stationnement sur le tronÃ§on
            }
        }
        else
        {
            bWarnODMatrixForParkingOutput = true;
        }
    }

    if (bWarnODMatrixForParkingOutput)
    {
        m_pNetwork->log() << Logger::Warning << "No OD matrix defined for origin " << GetOutputID() << " : unable to create the vehicle leaving parking spot at this origin." << std::endl;
        m_pNetwork->log() << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
    }

    return pVehicule;
}

//================================================================
   void SymuViaTripNode::UpdateCritCreationVeh
//----------------------------------------------------------------
// Fonction  : Initialisation des prochains instants de crÃ©ation 
//			   d'un vÃ©hicule
// Version du: 12/11/2009
// Historique: 12/11/2009 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
	double	dbInst,		// Instant courant de la simulation
    bool bDueToCreation // vrai si la mise Ã  jour est liÃ©e Ã  la crÃ©ation d'un vÃ©hicule
)
{
	double dbCritCreation;
	std::string ssPath;
	std::string sDestination;

    double dbNextVariationTime;

    Connexion * pConnexion = GetOutputConnexion();
  
    // CommentÃ© pour le cas particulier d'une zone qui a de la demande pour elle mÃªme et qui occupe tout le rÃ©seau : pConnexion->m_LstTuyAssAv.empty() est true dans ce cas.
    // Une autre solution qui fait sens serait de modifier la dÃ©finition de la connexion associÃ©e en lui assignant tous ses tuyaux internes en tant que tuyaux aval,
    // mais j'ai peut que ca casse pas mal de choses, alors qu'il sera plus facile de corriger d'Ã©ventuels problÃ¨mes liÃ©s Ã  ce test (si demande pour une connexion sans 
    // Ã©lÃ©ment aval, c'est qu'on a mal dÃ©fini quelquechose)
    //if( pConnexion->m_LstTuyAssAv.empty())
	//	return;

	//if( pConnexion->m_LstTuyAssAv.front()->IsMacro())
	//	return;
    for (size_t iDownstreamLink = 0; iDownstreamLink < pConnexion->m_LstTuyAssAv.size(); iDownstreamLink++)
    {
        if (pConnexion->m_LstTuyAssAv.at(iDownstreamLink)->IsMacro())
            return;
    }

	// Gestion d'une liste prÃ©dÃ©finie de vÃ©hicules
	if( !m_bDemande && (pConnexion->GetReseau()->GetTimeStep() == dbInst || bDueToCreation) )
	{
        if(m_LstCreationsVehicule.empty())
		{
            // Si on met Ã  jour le critÃ¨re suite Ã  une crÃ©ation, on passe Ã  DBL_MAX, sinon on renverra tout le temps le dernier
            // critÃ¨re de crÃ©ation (instant de crÃ©ation du dernier vÃ©hicule de la liste)
            if(bDueToCreation)
            {
                m_dbCritCreationVeh = DBL_MAX;
            }
			return;
		}

        bool bCreationFound = false;
        m_dbCritCreationVeh = DBL_MAX;	
        for(size_t iVeh = 0; iVeh < m_LstCreationsVehicule.size() && !bCreationFound; iVeh++)
        {
            const CreationVehicule & creationVeh = m_LstCreationsVehicule[iVeh];
            if(creationVeh.dbInstantCreation >= dbInst - pConnexion->GetReseau()->GetTimeStep())
            {
                bCreationFound = true;
                m_pDestinationVehiculeACreer = creationVeh.pDest;	
			    m_dbCritCreationVeh = creationVeh.dbInstantCreation;
                m_pTypeVehiculeACreer = creationVeh.pTypeVehicule;
                m_nVoieACreer = creationVeh.nVoie-1;
                m_LstCreationsVehicule.erase(m_LstCreationsVehicule.begin() + iVeh);
            }
        }
		if(!bCreationFound)
		{
			m_dbCritCreationVeh = DBL_MAX;	
		}		 
	}
	else if(m_bDemande)
	{
		// Niveau de demande dÃ©terministe
		if( !m_bDistribution )
		{
                
            dbCritCreation = GetDemandeValue(dbInst, dbNextVariationTime) * pConnexion->GetReseau()->GetTimeStep();
			m_dbCritCreationVeh += dbCritCreation;
		}
		else	// Niveau de demande stochastique
		{
			// Le prochain instant de crÃ©ation doit Ãªtre recalculÃ© uniquement si le niveau ou la rÃ©partition a Ã©tÃ© modifiÃ© par rapport Ã  l'instant prÃ©cÃ©dent
            if (fabs(GetDemandeValue(dbInst, dbNextVariationTime) - GetDemandeValue(dbInst - pConnexion->GetReseau()->GetTimeStep(), dbNextVariationTime)) > std::numeric_limits<double>::epsilon()
				|| fabs(dbInst - pConnexion->GetReseau()->GetTimeStep()) < std::numeric_limits<double>::epsilon()
				|| bDueToCreation)
			{
                double dbNiveau = GetDemandeValue(dbInst, dbNextVariationTime);
                       
				if( dbNiveau > pConnexion->GetReseau()->GetMaxDebitMax() )	// Le niveau est bornÃ© par le dÃ©bit max
					dbNiveau = pConnexion->GetReseau()->GetMaxDebitMax();

					// Tirage alÃ©atoire entre 0 et 1     
				double          dbRand = 0;
				while(dbRand<=0)
				{						                 
                    dbRand = (double)m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;
					// *(Reseau::m_pFicSimulation) << "rand UpdateCrit " << dbRand << std::endl;
				}

                      
                if(dbNiveau>0 )
				    dbCritCreation = dbInst - log(dbRand) * ( 1 - dbNiveau / pConnexion->GetReseau()->GetMaxDebitMax() ) / dbNiveau + 1 / pConnexion->GetReseau()->GetMaxDebitMax();					
                else
                    dbCritCreation = DBL_MAX; 
                   
				m_dbCritCreationVeh = dbCritCreation;
			}
		}
	}
}

//================================================================
int SymuViaTripNode::GetMatriceODInitSize(bool bIsUsedODMatrix)
//----------------------------------------------------------------
// Fonction  : Taille Ã  laquelle initialiser la taille de la
//              matrice OD de log
// Version du: 10/05/2012
// Historique: 10/05/2012 (O.Tonck - IPSIS)
//             CrÃ©ation
//================================================================
{
    int nSize;

    if(bIsUsedODMatrix)
    {
        // rmq : le nombre de variations temporelles est par construction identique quelquesoit le type, d'oÃ¹ le .begin()
        nSize = (int)m_LstCoeffDest.begin()->second.size();
    }
    else
        nSize = (int)m_LTVDemande.at(0)->GetLstTV()->size();

    return nSize;
}



//================================================================
void SymuViaTripNode::InitLogMatriceOD
//----------------------------------------------------------------
// Fonction  : Initialisation de la matrice OD de log 
// Version du: 13/09/2007
// Historique: 13/09/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    bool bIsUsedODMatrix // indique si la simulation utilise les matrices OD
                            // si oui, la matrice OD de log est calculÃ©e pour chaque
                            // variation de la matrice OD initial 
                            // si non, la matrice OD de log est calculÃ©e pour chaque
                            // variation du dÃ©bit initial 
)
{        
    int nSize = GetMatriceODInitSize(bIsUsedODMatrix);                        
                       
    for(int i = 0; i<nSize; i++)
    {
        int*    pNbVeh = new int[(int)m_LstDestinations.size()];

        for(int j = 0; j<(int)m_LstDestinations.size(); j++)
            pNbVeh[j] = 0;

        m_LogMatriceOD.push_back( pNbVeh );
    }        
}



//================================================================
    void  SymuViaTripNode::MAJLogMatriceOD
//----------------------------------------------------------------
// Fonction  : Met Ã  jour la matrice OD de l'origine
// Version du: 13/09/2007
// Historique: 13/09/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    bool bIsUsedODMatrix,
    double dbHeureEntree, 
    SymuViaTripNode *pDest
)
{
    SimMatOrigDest      *pMat;
    int                 nIt;

    // Recherche de la matrice concernÃ©e en fonction de 
    // l'instant de crÃ©ation du vÃ©hicule
    if(bIsUsedODMatrix)
    {
        pMat = GetVariation(dbHeureEntree, &m_LstCoeffDest.begin()->second, m_pNetwork->GetLag());  

        for(int i=0; i<(int)m_LstCoeffDest.begin()->second.size(); i++)
        {
            if(pMat == m_LstCoeffDest.begin()->second.at(i).m_pData.get())
                nIt = i;
        }

        // MAJ de la structure de stockage si besoin
        for(int i=(int)m_LogMatriceOD.size(); i<(int)m_LstCoeffDest.begin()->second.size(); i++)
        {
            int*    pNbVeh = new int[(int)m_LstDestinations.size()];

            for(int j = 0; j<(int)m_LstDestinations.size(); j++)
                pNbVeh[j] = 0;

            m_LogMatriceOD.push_back( pNbVeh );
        }
    }
    else
    {
        nIt=0;
    }

    // Ajout dans la variable log
    for(size_t j=0; j<m_LstDestinations.size(); j++)
    {
        if( pDest == m_LstDestinations[j] )
        {
            m_LogMatriceOD[nIt][j]++;     
            return;
        }
    }
        
    return;
} 




//================================================================
    void SymuViaTripNode::DeleteLogMatriceOD
//----------------------------------------------------------------
// Fonction  : Suppression de la matrice OD de log 
// Version du: 13/09/2007
// Historique: 13/09/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
)
    {      
        int*    pdb;

        std::deque <int*>::iterator ci, di, fi;
        di = m_LogMatriceOD.begin();
        fi = m_LogMatriceOD.end();

        for(ci=di; ci!=fi; ci++)
        {
            pdb = *ci;
            delete [] pdb;
        }

        m_LogMatriceOD.erase(di, fi);
    }


 
//================================================================
    void SymuViaTripNode::AddDestinations
//----------------------------------------------------------------
// Fonction  : Ajout d'une destination dans la liste
// Version du: 13/09/2007
// Historique: 13/09/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    SymuViaTripNode* pDst
)
{
    // A t-elle dÃ©jÃ  Ã©tÃ© ajoutÃ©e ?
    for(size_t j=0; j<m_LstDestinations.size(); j++)
    {
        if( pDst == m_LstDestinations[j] )
            return;
    }

    m_LstDestinations.push_back(pDst);
}

//================================================================
    void SymuViaTripNode::AddCreationVehicule
//----------------------------------------------------------------
// Fonction  : Ajout d'une crÃ©ation de vÃ©hicule
// Version du: 13/12/2012
// Historique: 13/12/2012 (O.Tonck - Ipsis)
//             CrÃ©ation
//================================================================
(
    double dbInstant,
    TypeVehicule * pTV,
    SymuViaTripNode * pDest,
    int nVoie
)
{
    CreationVehicule creationVeh;
    creationVeh.dbInstantCreation = dbInstant;
    creationVeh.pTypeVehicule = pTV;
    creationVeh.pDest = pDest;
    creationVeh.nVoie = nVoie;
    m_LstCreationsVehicule.push_back(creationVeh);
}

//================================================================
    int SymuViaTripNode::GetLogMatriceOD
//----------------------------------------------------------------
// Fonction  : Ajout d'une destination dans la liste
// Version du: 13/09/2007
// Historique: 13/09/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    int i,
    int j
)
{
    if(i>(int)m_LogMatriceOD.size())
        return 0;

    if(j>(int)m_LstDestinations.size())
        return 0;

    return m_LogMatriceOD[i][j];
}


//================================================================
    bool SymuViaTripNode::GetItineraires
//----------------------------------------------------------------
// Fonction  : Retourne la liste des itinÃ©raires et leurs
// coefficients d'affectation pour le couple OD dÃ©fini et le type
// de vÃ©hicule.
// Version du: 13/09/2007
// Historique: 13/09/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    TypeVehicule *pTV, 
    SymuViaTripNode *pDst,
    std::deque<AssignmentData> &dqAssData
)
{
    bool result = false;
   
	std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> > mapTVAssData;

	// Recherche du type de vÃ©hicule
    if( GetMapAssignment().find(pTV)== GetMapAssignment().end() )
		return result;
	
	mapTVAssData = GetMapAssignment().find(pTV)->second;

	// Recherche de la destination
	if( mapTVAssData.find( std::pair<Tuyau*, SymuViaTripNode*>(nullptr, pDst) )== mapTVAssData.end() )
		return result;


    // Retourne les itinÃ©raires avec leur coefficient d'affectation
    dqAssData = mapTVAssData.find( std::pair<Tuyau*, SymuViaTripNode*>(nullptr, pDst) )->second;
    result = true;

	return result;
}


//================================================================
    SymuViaTripNode* SymuViaTripNode::GetDestination
//----------------------------------------------------------------
// Fonction  : Calcule la destination
// Version du: 05/06/2007
// Historique: 05/06/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    double  dbTime,             // Instant de crÃ©ation
    int		nVoie,  			// Voie considÃ©rÃ©e
    TypeVehicule * pTypeVeh     // Type du vÃ©hicule
)
{
    SimMatOrigDest      *pMat;
    SymuViaTripNode     *pDest;
    deque<VectDest*>    LstVD;
    double              dbRand;
    double              dbSum;

    pDest = NULL;

    if(m_bDemande)
	{
        pMat = GetVariation(dbTime, &m_LstCoeffDest[pTypeVeh], m_pNetwork->GetLag());

		if(!pMat)
			return NULL;

	    LstVD = pMat->MatOrigDest;

	    // Tirage alÃ©atoire entre 0 et 1        
	    unsigned int    number;
	 
        number = m_pNetwork->GetRandManager()->myRand();
	    dbRand = (double)number / (double)MAXIMUM_RANDOM_NUMBER;   

	    // DÃ©termination de la sortie Ã©ligible
	    dbSum = 0;
	    for(int i=0; i < (int)LstVD.size(); i++)
	    {
			if( LstVD[i]->dbCoeff <= 0)
				continue;

		    dbSum += LstVD[i]->dbCoeff;

		    // PrÃ©caution
		    if( i ==  LstVD.size() - 1)
			    dbSum = 1;

		    if( dbRand <= dbSum )
		    {            
			    // Ajout dans la variable log de la matrice OD de l'entrÃ©e
			    if( m_pNetwork->IsDebugOD() )
				    MAJLogMatriceOD(true, dbTime, LstVD[i]->pDest);
	        
			    return LstVD[i]->pDest;
		    }
	    }    
    }
    else // Cas de la liste de vÃ©hicules Ã  crÃ©er
    {
        pDest = m_pDestinationVehiculeACreer;
    }

    return pDest;
}


//================================================================
CMotifCoeff * SymuViaTripNode::GetMotif
//----------------------------------------------------------------
// Fonction  : Calcule le motif de dÃ©placement
// Version du: 24/10/2016
// Historique: 24/10/2016 (O.Tonck - Ipsis)
//             CrÃ©ation
//================================================================

(
    double  dbTime,
    TypeVehicule * pTypeVeh,
    SymuViaTripNode * pDest
)
{
    CMotifCoeff * pResult = NULL;

    if (m_lstRepMotif.empty())
        return pResult;

    std::deque<TimeVariation<CRepMotif> > * pListRepMotif = NULL;

    std::map<TypeVehicule*, std::deque<TimeVariation<CRepMotif> > >::iterator iter = m_lstRepMotif.find(pTypeVeh);
    if (iter != m_lstRepMotif.end())
    {
        pListRepMotif = &iter->second;
    }
    else
    {
        iter = m_lstRepMotif.find(NULL);
        if (iter != m_lstRepMotif.end())
        {
            pListRepMotif = &m_lstRepMotif.at(NULL);
        }
    }

    CRepMotif * pRepMotif = GetVariation(dbTime, pListRepMotif, m_pNetwork->GetLag());

    if (!pRepMotif)
        return pResult;

    CRepMotifDest * pRepMotifDest = pRepMotif->getRepMotifDest(pDest);

    if (!pRepMotifDest)
        return pResult;

    // Tirage alÃ©atoire entre 0 et 1        
    unsigned int    number;

    number = m_pNetwork->GetRandManager()->myRand();
    double dbRand = (double)number / (double)MAXIMUM_RANDOM_NUMBER;

    // DÃ©termination du couple de motifs Ã©ligible
    double dbSum = 0;
    int nbElems = (int)pRepMotifDest->getCoeffs().size();
    for (int i = 0; i < nbElems; i++)
    {
        CMotifCoeff & motifCoeff = pRepMotifDest->getCoeffs()[i];
        if (motifCoeff.getCoeff() <= 0)
            continue;

        dbSum += motifCoeff.getCoeff();

        // PrÃ©caution
        if (i == nbElems - 1)
            dbSum = 1;

        if (dbRand <= dbSum)
        {
            pResult = &motifCoeff;
            break;
        }
    }
    return pResult;
}

//================================================================
    bool SymuViaTripNode::IsAgressif
//----------------------------------------------------------------
// Fonction  : Calcule si le vÃ©hicule en cours de crÃ©ation est agressif
//			   (procÃ©dure stochastique)
// Version du: 05/06/2007
// Historique: 05/06/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    TypeVehicule *pTV
)
{
    double              dbRand;	

	if( m_mapTauxAgressivite.find(pTV) == m_mapTauxAgressivite.end() )
		return false;	

    // Tirage alÃ©atoire entre 0 et 1        
    unsigned int    number;
 
    number = m_pNetwork->GetRandManager()->myRand();
    dbRand = (double)number / (double)MAXIMUM_RANDOM_NUMBER;    
	
	if( dbRand <= m_mapTauxAgressivite[pTV] )
		return true;

	return false;
}

bool SymuViaTripNode::InsertionVehEnAttenteMeso(double dbInst, Vehicule * pV)
{
	bool bInsertion = false;

    Connexion * pConnexion = GetOutputConnexion();

    if( pConnexion->m_LstTuyAv.empty() )
		return false;

	if( !pConnexion->m_LstTuyAv.front() )
		return false;

    std::deque<TraceDocTrafic*> doctrafics = pConnexion->GetReseau()->GetXmlDocTrafic();
    std::deque<TraceDocTrafic* >::iterator itDocTraf;

    // On dÃ©termine la liste des tuyaux avec des vÃ©hicules prÃªts ou en attente (cas des zones dont la liste 
    // des tuyaux avec map des vÃ©hicules prets ou en attente est dynamique si vÃ©hicule crÃ©Ã© Ã  l'intÃ©rieur de la zone)
    std::set<Tuyau*> setTuyauxAval;
    std::map<Tuyau*, std::map<int, std::deque<boost::shared_ptr<Vehicule>>>>::const_iterator iterMapVehAttente;
    for (iterMapVehAttente = m_mapVehEnAttente.begin(); iterMapVehAttente != m_mapVehEnAttente.end(); ++iterMapVehAttente)
    {
        setTuyauxAval.insert(iterMapVehAttente->first);
    }
    std::map<Tuyau*, std::map<int, int > >::const_iterator iterMapVehPret;
    for (iterMapVehPret = m_mapVehPret.begin(); iterMapVehPret != m_mapVehPret.end(); ++iterMapVehPret)
    {
        setTuyauxAval.insert(iterMapVehPret->first);
    }

    for (std::set<Tuyau*>::iterator iterTuyauAval = setTuyauxAval.begin(); iterTuyauAval != setTuyauxAval.end(); ++iterTuyauAval)
	{
        Tuyau * pTuyauAval = *iterTuyauAval;
        if( pTuyauAval->GetType() == Tuyau::TT_MESO )
        {
	        for(int nVoie = 0; nVoie < pTuyauAval->getNb_voies(); nVoie++)
	        {
				bool bInsertionVehPret = false;
		        // Traitement du vÃ©hicule pret au dÃ©but du pas de temps
		        if( m_mapVehPret[pTuyauAval].find(nVoie)->second == pV->GetID() )
		        {
                    if( pTuyauAval->GetType() == Tuyau::TT_MESO)
                    {
                        pV->SetTuyauMeso(pTuyauAval, dbInst);
                        m_mapVehPret[pTuyauAval].find(nVoie)->second = -1;			// Le vÃ©hicule est sur le rÃ©seau, il n'est plus catÃ©gorisÃ© comme 'pret'
                        bInsertion = true;
						bInsertionVehPret = true;

                        for( itDocTraf = doctrafics.begin(); itDocTraf != doctrafics.end(); itDocTraf++)
                        {
                            (*itDocTraf)->UpdateInstEntreeVehicule(pV->GetID(), dbInst);
                        }

                         m_dbLastInstCreation = dbInst;
                    } // 
                /*    else // else meso ou macro
                    {
			        
			            if( pV->GetPos(0) > 0.0001)
			            {
                            pV = pConnexion->GetReseau()->GetVehiculeFromID( m_mapVehPret[GetTuyauxAval()[nTuyau]].find(nVoie)->second );
				            // *Reseau::m_pFicSimulation << " Pret -> Insertion de " << pV->GetID() << std::endl;
				            m_mapVehPret[GetTuyauxAval()[nTuyau]].find(nVoie)->second = -1;			// Le vÃ©hicule est sur le rÃ©seau, il n'est plus catÃ©gorisÃ© comme 'pret'
				            bInsertion = true;
				
				            double dbInstEntree = dbInst;
                            // Correction bug nÂ°45 : instE Ã  -Infini dans certains cas car division par zero ici
                            if(pV->GetVit(0) > 0)
                            {
                                dbInstEntree -= pV->GetPos(0)/pV->GetVit(0);
                            }
				            pConnexion->GetReseau()->GetXmlDocTrafic()->UpdateInstEntreeVehicule(pV->GetID(), dbInstEntree);
                            m_dbLastInstCreation = dbInstEntree;
			            }
			            else
			            {				
				            continue;		// Le vÃ©hicule 'pret' ne s'est pas insÃ©rÃ©, on ne touche pas aux vÃ©hicules en attente
			            }
                    }*/
		        }

				if (!bInsertionVehPret && m_mapVehEnAttente[pTuyauAval].find(nVoie) != m_mapVehEnAttente[pTuyauAval].end())
		        {
					if (m_mapVehEnAttente[pTuyauAval].find(nVoie)->second.size() > 0)
					{
						if (m_mapVehEnAttente[pTuyauAval].find(nVoie)->second.front()->GetID() == pV->GetID())
						{
							if (CanInsertVehicle(dbInst, m_dbLastInstCreation))	// Pas deux insertion au cours du mÃªme pas de temps
							{
								if (pTuyauAval->GetType() == Tuyau::TT_MESO
									&& ((CTuyauMeso*)pTuyauAval)->IsCongested() == false
									)
								{
									pV->SetTuyauMeso(pTuyauAval, dbInst);
									m_mapVehEnAttente[pTuyauAval].find(nVoie)->second.erase(m_mapVehEnAttente[pTuyauAval].find(nVoie)->second.begin());
									m_pNetwork->AddVehicule(pV->shared_from_this());
									VehiculeExit(pV->shared_from_this());
									bInsertion = true;
									for (itDocTraf = doctrafics.begin(); itDocTraf != doctrafics.end(); itDocTraf++)
									{
										(*itDocTraf)->UpdateInstEntreeVehicule(pV->GetID(), dbInst);
									}
									m_dbLastInstCreation = dbInst;
									return bInsertion;
								} // 
								/* else
								 {
								 pV->CalculTraficEx( dbInst );

								 #ifndef NDEBUG
								 *Reseau::m_pFicSimulation << " Nombre de veh en attente  " <<  m_mapVehEnAttente[GetTuyauxAval()[nTuyau]].find( nVoie )->second.size() << " sur " << nVoie << std::endl;
								 #endif

								 if(pV->GetPos(0) >= 0  )	// Le vÃ©hicule a pu s'insÃ©rer
								 {
								 m_mapVehEnAttente[GetTuyauxAval()[nTuyau]].find( nVoie )->second.erase( m_mapVehEnAttente[GetTuyauxAval()[nTuyau]].find( nVoie )->second.begin() );
								 pConnexion->GetReseau()->AddVehicule( pV );
								 VehiculeInserted();


								 double dbInstEntree = dbInst;
								 // Correction bug nÂ°45 : instE Ã  -Infini dans certains cas car division par zero ici
								 if(pV->GetVit(0) > 0)
								 {
								 dbInstEntree -= pV->GetPos(0)/pV->GetVit(0);
								 }
								 pConnexion->GetReseau()->GetXmlDocTrafic()->UpdateInstEntreeVehicule(pV->GetID(), dbInstEntree);
								 m_dbLastInstCreation = dbInstEntree;

								 // *Reseau::m_pFicSimulation << std::endl << "--> insertion de " << pV->GetID() << std::endl;

								 //#ifndef NDEBUG
								 //	*Reseau::m_pFicSimulation << " Insertion de " << pV->GetID() << std::endl;
								 //#endif
								 }
								 else
								 {
								 // Pour que le prochain CalculTraficEx fonctionne
								 pV->SetPos( UNDEF_POSITION );
								 pV->SetVit( pV->GetVitMax() );
								 pV->SetInstantCreation(0);
								 pV->SetDejaCalcule(false);
								 pV->SetVehLeader(boost::shared_ptr<Vehicule>());
								 }
								 }*/
							}
							else
							{
								/*  if( m_mapVehPret[GetTuyauxAval()[nTuyau]].find(nVoie)->second != -1 )
								  {
								  pV->SetPos( UNDEF_POSITION );
								  pV->SetVit( pV->GetVitMax() );
								  pV->SetInstantCreation(0);
								  pV->SetDejaCalcule(false);
								  pV->SetVehLeader(boost::shared_ptr<Vehicule>());

								  //#ifndef NDEBUG
								  //	*Reseau::m_pFicSimulation << " VÃ©hicule en attente " << pV->GetID() << std::endl;
								  //#endif
								  }
								  else
								  {
								  // si la barriÃ¨re n'est pas prÃªte Ã  s'ouvrir, on n'insÃ¨re pas les vÃ©hicules
								  if(CanInsertVehicle(dbInst, m_dbLastInstCreation))
								  {
								  m_mapVehEnAttente[GetTuyauxAval()[nTuyau]].find( nVoie )->second.erase( m_mapVehEnAttente[GetTuyauxAval()[nTuyau]].find( nVoie )->second.begin() );
								  pConnexion->GetReseau()->AddVehicule( pV );
								  VehiculeInserted();
								  m_mapVehPret[GetTuyauxAval()[nTuyau]].find( nVoie )->second = pV->GetID();
								  pV->SetInstantCreation(0);

								  //#ifndef NDEBUG
								  //	*Reseau::m_pFicSimulation << " VÃ©hicule pret " << pV->GetID() << std::endl;
								  //#endif
								  }
								  }
								  */
							}
#ifndef NDEBUG
							// *Reseau::m_pFicSimulation << std::endl;
#endif
						}
					}
		        }
            }
        }
    }
    return bInsertion;
}

//================================================================
	void SymuViaTripNode::InsertionVehEnAttente
//----------------------------------------------------------------
// Fonction  : Module d'insertion des vÃ©hicules en attente
// Version du: 16/11/2009
// Historique: 16/11/2009 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
	double dbInst
)
{
	boost::shared_ptr<Vehicule> pV;
	

    Connexion * pConnexion = GetOutputConnexion();

    if( pConnexion->m_LstTuyAv.empty() )
		return ;

	if( !pConnexion->m_LstTuyAv.front() )
		return ;

    std::deque<TraceDocTrafic*> doctrafics = pConnexion->GetReseau()->GetXmlDocTrafic();
    std::deque<TraceDocTrafic* >::iterator itDocTraf;


    // On dÃ©termine la liste des tuyaux avec des vÃ©hicules prÃªts ou en attente (cas des zones dont la liste 
    // des tuyaux avec map des vÃ©hicules prets ou en attente est dynamique si vÃ©hicule crÃ©Ã© Ã  l'intÃ©rieur de la zone)
    std::set<Tuyau*, LessPtr<Tuyau>> setTuyauxAval;
    std::map<Tuyau*, std::map<int, std::deque<boost::shared_ptr<Vehicule>>>>::const_iterator iterMapVehAttente;
    for (iterMapVehAttente = m_mapVehEnAttente.begin(); iterMapVehAttente != m_mapVehEnAttente.end(); ++iterMapVehAttente)
    {
        setTuyauxAval.insert(iterMapVehAttente->first);
    }
    std::map<Tuyau*, std::map<int, int > >::const_iterator iterMapVehPret;
    for (iterMapVehPret = m_mapVehPret.begin(); iterMapVehPret != m_mapVehPret.end(); ++iterMapVehPret)
    {
        setTuyauxAval.insert(iterMapVehPret->first);
    }

    for (std::set<Tuyau*, LessPtr<Tuyau>>::iterator iterTuyauAval = setTuyauxAval.begin(); iterTuyauAval != setTuyauxAval.end(); ++iterTuyauAval)
    {
        Tuyau * pTuyAval = *iterTuyauAval;

        if(pTuyAval->GetType() == Tuyau::TT_MICRO)
        {

	        for(int nVoie = 0; nVoie < pTuyAval->getNb_voies(); nVoie++)
	        {
		        // Traitement du vÃ©hicule pret au dÃ©but du pas de temps
		        if( m_mapVehPret[pTuyAval].find(nVoie)->second != -1 )
		        {
                        pV = m_pNetwork->GetVehiculeFromID( m_mapVehPret[pTuyAval].find(nVoie)->second );
                        if(!pV )
                        {
                            continue;
                        }
                   
			        if( pV->GetPos(0) > pV->GetGenerationPosition())
			        {
                        pV = m_pNetwork->GetVehiculeFromID( m_mapVehPret[pTuyAval].find(nVoie)->second );
				        // *Reseau::m_pFicSimulation << " Pret -> Insertion de " << pV->GetID() << std::endl;
				        m_mapVehPret[pTuyAval].find(nVoie)->second = -1;			// Le vÃ©hicule est sur le rÃ©seau, il n'est plus catÃ©gorisÃ© comme 'pret'
				
				
				        double dbInstEntree = dbInst;
                        // Correction bug nÂ°45 : instE Ã  -Infini dans certains cas car division par zero ici
                        if(pV->GetVit(0) > 0)
                        {
                            dbInstEntree -= pV->GetDstParcourueEx()/pV->GetVit(0);
                            // Correction d'un bug du au fait que dans le calcul de l'Ã©coulement, on force la vitesse d'un vÃ©hicule
                            // crÃ©Ã© en entrÃ©e congestionnÃ©e Ã  la vitesse de son leader, ce qui dÃ©corelle la vitesse et la distance parcourue pour ce vÃ©hicule
                            // et provoque par la formule ci-dessus un instant d'entrÃ©e trÃ¨s nÃ©gatif si la vitesse du leader est trÃ¨s faible.
                            // On corrige le tir ici en ne permettant pas de passer sous l'instant d'entrÃ©e du vÃ©hicule pour Ã©viter une valeur
                            // aberrante dans l'attribut instE du vÃ©hicule.
                            dbInstEntree = std::max<double>(pV->GetHeureEntree(), dbInstEntree);
                        }
                        for( itDocTraf = doctrafics.begin(); itDocTraf != doctrafics.end(); itDocTraf++)
                        {
				            (*itDocTraf)->UpdateInstEntreeVehicule(pV->GetID(), dbInstEntree);
						    pV->SetInstantEntree(dbInstEntree);
                        }
                        m_dbLastInstCreation = dbInstEntree;
			        }
			        else
			        {				
				        continue;		// Le vÃ©hicule 'pret' ne s'est pas insÃ©rÃ©, on ne touche pas aux vÃ©hicules en attente
			        }
                    
		        }

		        if( m_mapVehEnAttente[pTuyAval].find( nVoie ) !=  m_mapVehEnAttente[pTuyAval].end() )
		        {
			        if( m_mapVehEnAttente[pTuyAval].find( nVoie )->second.size() > 0)
			        {
				        pV = m_mapVehEnAttente[pTuyAval].find( nVoie )->second.front();	// Traitement du premier vÃ©hicule de la liste (premier crÃ©Ã©)

				        if( pV && CanInsertVehicle(dbInst, m_dbLastInstCreation))	// Pas deux insertion au cours du mÃªme pas de temps
				        {
                             
					        pV->CalculTraficEx( dbInst );				

					        //#ifndef NDEBUG
    				        //m_pNetwork->log() << " Nombre de veh en attente  " <<  m_mapVehEnAttente[pTuyAval].find( nVoie )->second.size() << " sur " << nVoie << std::endl;
					        //#endif

					        if(pV->GetPos(0) > pV->GetGenerationPosition()  )	// Le vÃ©hicule a pu s'insÃ©rer
					        {					
						        m_mapVehEnAttente[pTuyAval].find( nVoie )->second.erase( m_mapVehEnAttente[pTuyAval].find( nVoie )->second.begin() );
                                m_pNetwork->AddVehicule(pV);
                                VehiculeExit(pV);
                           
						
						        double dbInstEntree = dbInst;
                                // Correction bug nÂ°45 : instE Ã  -Infini dans certains cas car division par zero ici
                                if(pV->GetVit(0) > 0)
                                {
                                    dbInstEntree -= pV->GetDstParcourueEx()/pV->GetVit(0);
                                    // Correction d'un bug du au fait que dans le calcul de l'Ã©coulement, on force la vitesse d'un vÃ©hicule
                                    // crÃ©Ã© en entrÃ©e congestionnÃ©e Ã  la vitesse de son leader, ce qui dÃ©corelle la vitesse et la distance parcourue pour ce vÃ©hicule
                                    // et provoque par la formule ci-dessus un instant d'entrÃ©e trÃ¨s nÃ©gatif si la vitesse du leader est trÃ¨s faible.
                                    // On corrige le tir ici en ne permettant pas de passer sous l'instant d'entrÃ©e du vÃ©hicule pour Ã©viter une valeur
                                    // aberrante dans l'attribut instE du vÃ©hicule.
                                    dbInstEntree = std::max<double>(pV->GetHeureEntree(), dbInstEntree);
                                }
                                for( itDocTraf = doctrafics.begin(); itDocTraf != doctrafics.end(); itDocTraf++)
                                {
						            (*itDocTraf)->UpdateInstEntreeVehicule(pV->GetID(), dbInstEntree);
									pV->SetInstantEntree(dbInstEntree);
                                }
                                m_dbLastInstCreation = dbInstEntree;
					        }
                            else
                            {
                                // Pour que le prochain CalculTraficEx fonctionne
                                pV->SetPos( UNDEF_POSITION );
						        pV->SetVit( pV->GetVitMax() );
						        pV->SetInstantCreation(0);
						        pV->SetDejaCalcule(false);
                            }
				        }
				        else
				        {					
					        if( m_mapVehPret[pTuyAval].find(nVoie)->second != -1 )
					        {
						        pV->SetPos( UNDEF_POSITION );
						        pV->SetVit( pV->GetVitMax() );
						        pV->SetInstantCreation(0);
						        pV->SetDejaCalcule(false);
					        }
					        else
					        {
                                // si la barriÃ¨re n'est pas prÃªte Ã  s'ouvrir, on n'insÃ¨re pas les vÃ©hicules
                                if(CanInsertVehicle(dbInst, m_dbLastInstCreation))
                                {
						            m_mapVehEnAttente[pTuyAval].find( nVoie )->second.erase( m_mapVehEnAttente[pTuyAval].find( nVoie )->second.begin() );
                                    m_pNetwork->AddVehicule(pV);
                                    VehiculeExit(pV);
						            m_mapVehPret[pTuyAval].find( nVoie )->second = pV->GetID();
						            pV->SetInstantCreation(0);
                                }
					        }

				        }
			        }
		        }
            }
        }
    }
}

//================================================================
	void SymuViaTripNode::FinSimuTrafic()
//----------------------------------------------------------------
// Fonction  : Code de fin de simulation
// Version du: 14/01/2010
// Historique: 14/01/2010 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
{
	std::deque<boost::shared_ptr<Vehicule>>::iterator itV;

    Connexion * pConnexion = GetOutputConnexion();

    if( pConnexion->m_LstTuyAv.empty() )
		return;

    // On dÃ©termine la liste des tuyaux avec des vÃ©hicules prÃªts ou en attente (cas des zones dont la liste 
    // des tuyaux avec map des vÃ©hicules prets ou en attente est dynamique si vÃ©hicule crÃ©Ã© Ã  l'intÃ©rieur de la zone)
    std::set<Tuyau*> setTuyauxAval;
    std::map<Tuyau*, std::map<int, std::deque<boost::shared_ptr<Vehicule>>>>::const_iterator iterMapVehAttente;
    for (iterMapVehAttente = m_mapVehEnAttente.begin(); iterMapVehAttente != m_mapVehEnAttente.end(); ++iterMapVehAttente)
    {
        setTuyauxAval.insert(iterMapVehAttente->first);
    }
    std::map<Tuyau*, std::map<int, int > >::const_iterator iterMapVehPret;
    for (iterMapVehPret = m_mapVehPret.begin(); iterMapVehPret != m_mapVehPret.end(); ++iterMapVehPret)
    {
        setTuyauxAval.insert(iterMapVehPret->first);
    }

    for (std::set<Tuyau*>::iterator iterTuyauAval = setTuyauxAval.begin(); iterTuyauAval != setTuyauxAval.end(); ++iterTuyauAval)
    {
        Tuyau * pTuyAval = *iterTuyauAval;
	    for(int nVoie = 0; nVoie < pTuyAval->getNb_voies(); nVoie++)	
	    {
		    // LibÃ©ration mÃ©moire des vÃ©hicules en attente
		    if( m_mapVehEnAttente[pTuyAval].find( nVoie ) != m_mapVehEnAttente[pTuyAval].end() )
		    {
			    m_mapVehEnAttente[pTuyAval].find( nVoie )->second.clear();
		    }

		    // LibÃ©ration mÃ©moire des vÃ©hicules prets
		    if( m_mapVehPret[pTuyAval].find( nVoie )!=m_mapVehPret[pTuyAval].end()  )
		    {
			    //if( m_mapVehPret.find( nVoie )->second != -1)
			    //	delete( m_mapVehPret.find( nVoie )->second );
			    m_mapVehPret[pTuyAval].clear();
		    }
	    }
    }
}

//================================================================
    TypeVehicule*   SymuViaTripNode::CalculTypeNewVehicule
//----------------------------------------------------------------
// Fonction  : Calcule par un processus alÃ©atoire le type du vÃ©hicule Ã  crÃ©er
// Version du: 08/01/2007
// Historique: 08/01/2007 (C.BÃ©carie - Tinea)
//================================================================
(
    double  dbInstant,
    int     nVoie
)
{
    if(m_pTypeVehiculeACreer)
    {
        return m_pTypeVehiculeACreer;
    }
    else
    {
        return m_pLstRepTypeVeh->CalculTypeNewVehicule(m_pNetwork, dbInstant, nVoie);
    }
}


//================================================================
    int   SymuViaTripNode::CalculNumVoie
//----------------------------------------------------------------
// Fonction  : Calcule par un processus alÃ©atoire le numÃ©ro de
//             voie
// Version du: 08/02/2017
// Historique: 08/02/2017 (O.Tonck - Ipsis)
//================================================================
(
    double  dbInst,
    Tuyau * pOutputLink
)
{
    int nVoie = 0;
    // Calcul de la voie de crÃ©ation (pour les zones, le choix de la voie se fait plus tard)
    if (!m_bDemande)
    {
        nVoie = m_nVoieACreer;
    }
    else
    {
        ZoneDeTerminaison * pZone = dynamic_cast<ZoneDeTerminaison*>(this);
        if (!pZone)
        {
            if (pOutputLink == NULL)
            {
                Connexion * pConnexion = GetOutputConnexion();
                if (pConnexion)
                {
                    pOutputLink = pConnexion->m_LstTuyAssAv.front();
                }
                else
                {
                    pOutputLink = m_OutputPosition.GetLink();
                }
            }

            int nbVoies = pOutputLink->getNb_voies();
            // tirage de la voie
            double dbRand = m_pNetwork->GetRandManager()->myRand() / (double)MAXIMUM_RANDOM_NUMBER;

            double dbSum = 0;
            const std::vector<double> & coeffsLanes = GetRepVoieValue(dbInst);

            for (; nVoie < nbVoies; nVoie++)
            {
                double dbCoefVoie;
                if (!coeffsLanes.empty())
                {
                    dbCoefVoie = coeffsLanes[nVoie];
                }
                else
                {
                    // Cas de rÃ©partition non dÃ©finie : utilisation d'une rÃ©partition uniforme
                    dbCoefVoie = 1.0 / (double)nbVoies;
                }

                if (dbCoefVoie <= 0)
                    continue;

                dbSum += dbCoefVoie;

                if (dbSum >= dbRand)
                {
                    break;
                }
            }
        }
    }

    return nVoie;
}

//================================================================
 std::vector<double> SymuViaTripNode::GetRepVoieValue
//----------------------------------------------------------------
// Fonction  : Retourne la valeur de la repartition par voie pour l'origine
//             considÃ©rÃ©e
// Version du: 02/12/2014
// Historique: 02/12/20141 (E. Trivis - IPSIS)
//              CrÃ©ation
//================================================================
(
    double dbInst
)
{
    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* > ::iterator it;
    std::vector<double> coeffs;
    
    if( dbInst <0) return coeffs;
    if( m_lstRepVoie.find(NULL) != m_lstRepVoie.end() ) // global existe -> retourne la demande globale
    {
        RepartitionEntree * pRepEntree = GetVariation(dbInst,  m_lstRepVoie.find(NULL)->second, m_pNetwork->GetLag());
        if( pRepEntree )
        {
            coeffs.assign(pRepEntree->pCoefficients.begin(), pRepEntree->pCoefficients.end()) ;
        }
      
    }
    else // sinon somme les demandes par types
    {
        int nVeh = 0;
        for (it = m_lstRepVoie.begin(); it!= m_lstRepVoie.end(); it++)
        {
            RepartitionEntree * pRepEntree = GetVariation(dbInst,  it->second, m_pNetwork->GetLag());
            if( coeffs.empty())
            {
                coeffs.assign( pRepEntree->pCoefficients.begin(), pRepEntree->pCoefficients.end());
            }
            else
            {
                for (size_t i = 0; i < pRepEntree->pCoefficients.size(); ++i)
                {
                    coeffs[i] += pRepEntree->pCoefficients[i];
                }
            }
            nVeh++;
        }
		for (size_t i = 0; i < coeffs.size(); ++i)
        {
            coeffs[i] /= nVeh;
        }
       
    }
 
   return coeffs;
 }


 int SymuViaTripNode::SetLstRepVoie(TypeVehicule* pTypeVehicle, const std::vector<double> & coeffs)
 {
     int nNbVoie = (int)coeffs.size();

     // Check sum
     double dbS = 0;
     for (size_t i = 0; i < coeffs.size(); ++i)
     {
         dbS += coeffs[i];
     }
     if (fabs(dbS - 1) > 0.000001)
         return -1;

     // Suppression de l'ancienne dÃ©finition de la rÃ©partition par voiv
     std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* > ::iterator it = m_lstRepVoie.find(pTypeVehicle);
     if (it != m_lstRepVoie.end())
     {
         EraseListOfVariation(it->second);
         delete it->second;
         m_lstRepVoie.erase(it);
     }

     // dÃ©finition de la nouvelle rÃ©partition par voie
     boost::shared_ptr<RepartitionEntree> pE = boost::make_shared<RepartitionEntree>();
     pE->pCoefficients = coeffs;
     AddVariation(m_pNetwork->GetDureeSimu(), pE, GetLstRepVoie(pTypeVehicle));

     // adaptation de la liste de rÃ©partition de vÃ©hicules
     std::deque<TimeVariation<std::vector<std::vector<double> > > > & repTypes = m_pLstRepTypeVeh->GetLstCoefficients();
     // Pour chaque variante temporelle :
     double dbInstant = 0;
     double dbVariationTime;
     for (size_t iRepType = 0; iRepType < repTypes.size(); iRepType++)
     {
         TimeVariation<std::vector<std::vector<double> > > & repTypesVariation = repTypes.at(iRepType);

         std::vector<std::vector<double> > * pRepTypeCoeffs = repTypesVariation.m_pData.get();
         if (pRepTypeCoeffs)
         {
             if (repTypesVariation.m_pPlage)
             {
                 dbVariationTime = (repTypesVariation.m_pPlage->m_Fin + repTypesVariation.m_pPlage->m_Debut) / 2;
             }
             else
             {
                 dbVariationTime = (dbInstant + repTypesVariation.m_dbPeriod) / 2;
             }

             // adaptation des coefficients :
             std::vector<double> sumPerType;
             for (int i = 0; i< nNbVoie; ++i)
             {
                 sumPerType.push_back(0);
             }
             for (size_t iTypeVeh = 0; iTypeVeh < m_pNetwork->m_LstTypesVehicule.size(); iTypeVeh++)
             {
                 TypeVehicule * pTypeVeh = m_pNetwork->m_LstTypesVehicule.at(iTypeVeh);
                 it = m_lstRepVoie.find(pTypeVeh);
                 RepartitionEntree * pE = NULL;
                 if (it != m_lstRepVoie.end())
                 {
                     pE = GetVariation(dbVariationTime, it->second, m_pNetwork->GetLag());
                 }
                 double dbDemand = 0;
                 ListOfTimeVariation<tracked_double> * pDemandLst = GetLstDemande(pTypeVeh);
                 if (pDemandLst)
                 {
                     tracked_double * pDemandValue = pDemandLst->GetVariationEx(dbVariationTime);
                     if (pDemandValue)
                     {
                         dbDemand = *pDemandValue;
                     }
                 }
                 for (int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                 {
                     double dVoieCoeff;
                     // Si on a une rÃ©partition par voie :
                     if (pE && !pE->pCoefficients.empty())
                     {
                         assert(pE->pCoefficients.size() == coeffs.size());

                         dVoieCoeff = pE->pCoefficients.at(iCoeffWay);
                     }
                     else
                     {
                         // Sinon c'est une rÃ©partition homogÃ¨ne
                         dVoieCoeff = 1.0 / nNbVoie;
                     }

                     (*pRepTypeCoeffs)[iTypeVeh][iCoeffWay] = dbDemand *dVoieCoeff;
                     sumPerType[iCoeffWay] += dbDemand *dVoieCoeff;
                 }
             }

             // on norme les coefficients par voie
             for (size_t iTypeVeh = 0; iTypeVeh < m_pNetwork->m_LstTypesVehicule.size(); ++iTypeVeh)
             {
                 for (int iCoeffWay = 0; iCoeffWay < nNbVoie; ++iCoeffWay)
                 {
                     (*pRepTypeCoeffs)[iTypeVeh][iCoeffWay] = (sumPerType[iCoeffWay] != 0) ? (*pRepTypeCoeffs)[iTypeVeh][iCoeffWay] / sumPerType[iCoeffWay] : 0;
                 }
             }
         }

         if (!repTypesVariation.m_pPlage)
         {
             dbInstant += repTypesVariation.m_dbPeriod;
         }
     }

     return 0;
 }

//================================================================
    double SymuViaTripNode::GetDemandeValue
//----------------------------------------------------------------
// Fonction  : Retourne la valeur de la demande pour l'origine
//             considÃ©rÃ©e
// Version du: 22/06/2011
// Historique: 22/06/2011 (O. Tonck - IPSIS)
//              CrÃ©ation
//              07/10/2014 modification ajout pTypeVehicle en paramÃ¨tre pour Ãªtre compatible au xsd v44
//================================================================
(
    double dbInst,
    double & dbNextVariationTime
 
)
{
    dbNextVariationTime = DBL_MAX;
    if( dbInst <0) return -1;
    std::map<TypeVehicule* , ListOfTimeVariation<tracked_double> *>::iterator it;
    double dDemande = 0;
    if( m_LTVDemande.find(NULL) != m_LTVDemande.end() ) // global existe -> retourne la demande globale
    {
        tracked_double * pDemandValue = m_LTVDemande.find(NULL)->second->GetVariationEx(dbInst, dbNextVariationTime);
        if(pDemandValue)
        {
            dDemande+= *pDemandValue;
        }
    }
    else // sinon somme les demandes par types
    {
        for (it = m_LTVDemande.begin(); it!= m_LTVDemande.end(); it++)
        {
            double dbEndVarTime;
            tracked_double * pDemandValue = (*it).second->GetVariationEx(dbInst, dbEndVarTime);
            if(pDemandValue)
            {
                dbNextVariationTime = std::min<double>(dbNextVariationTime, dbEndVarTime);
                dDemande+= *pDemandValue;
            }
        }
    }
    
   return dDemande;
    
    
}



// 
//================================================================
    bool SymuViaTripNode::IsLinkedToDestination(SymuViaTripNode * pDest, TypeVehicule * pTV, double startPeriodTime, double endPeriodTime)
//----------------------------------------------------------------
// Fonction  : teste si un vÃ©hicule est susceptible de rallier 
// cette origine Ã  la destination passÃ©e en paramÃ¨tre
// Version du: 12/09/2012
// Historique: 12/09/2012 (O. Tonck - IPSIS)
//              CrÃ©ation
//================================================================
{
    bool bResult = false;

    // S'il s'agit d'un parking avec zone parent, on regarde les donnÃ©es de la zone parente
    Parking * pParking = dynamic_cast<Parking*>(this);
    if(pParking && pParking->GetZoneParent())
    {
        return pParking->GetZoneParent()->IsLinkedToDestination(pDest, pTV, startPeriodTime, endPeriodTime);
    }

    // 1. Si liste de vÃ©hicule Ã  crÃ©er, on regarde si un du type demandÃ© a pour destination la pDest
    if(!m_bDemande)
    {
        for(size_t iVeh = 0; iVeh < m_LstCreationsVehicule.size() && !bResult; iVeh++)
        {
            if(m_LstCreationsVehicule[iVeh].pDest == pDest && m_LstCreationsVehicule[iVeh].pTypeVehicule == pTV
                && m_LstCreationsVehicule[iVeh].dbInstantCreation >= startPeriodTime && m_LstCreationsVehicule[iVeh].dbInstantCreation <= endPeriodTime)
            {
                bResult = true;
            }
        } 
    }
    //2. Si demande, on regarde si une variation temporelle Ã  une valeur de demande non nulle et que la rÃ©partition
    // des types de vÃ©hicules inclu le vÃ©hicule pTV
    else
    {
        // rÃ©cupÃ©ration de l'index du type de vÃ©hicule testÃ©
        int indexTV = 0;
        for(size_t i = 0; i < m_pNetwork->m_LstTypesVehicule.size(); i++)
        {
            if(m_pNetwork->m_LstTypesVehicule[i] == pTV)
            {
                indexTV = (int)i;
                break;
            }
        }
        bool bHasTypeVeh = m_pLstRepTypeVeh->HasVehicleType(m_pNetwork, indexTV, startPeriodTime, endPeriodTime);

        // Si le type de vÃ©hicule testÃ© est dans la rÃ©partition, on regarde le niveau de demande
        // et le coefficients vers la destination
        if(bHasTypeVeh)
        {
            std::vector<std::pair<TimeVariation<tracked_double>,std::pair<double,double>>> timeVariations = GetVariations(startPeriodTime, endPeriodTime, m_LTVDemande[pTV]->GetLstTV(), m_pNetwork->GetLag());
            for (size_t i = 0; i < timeVariations.size() && !bResult; i++)
            {
                if ((*timeVariations.at(i).first.m_pData) > 0)
                {
                    // on finit par regarder si un coeff vers la destination est non nul
                    std::vector<std::pair<TimeVariation<SimMatOrigDest>, std::pair<double, double>>> timeVariationsOD = GetVariations(startPeriodTime, endPeriodTime, &m_LstCoeffDest[pTV], m_pNetwork->GetLag());
                    for (size_t i = 0; i < timeVariationsOD.size() && !bResult; i++)
                    {
                        const TimeVariation<SimMatOrigDest> & simMatOD = timeVariationsOD.at(i).first;
                        for (size_t j = 0; j < simMatOD.m_pData->MatOrigDest.size() && !bResult; j++)
                        {
                            if (simMatOD.m_pData->MatOrigDest[j]->pDest == pDest
                                && simMatOD.m_pData->MatOrigDest[j]->dbCoeff > 0)
                            {
                                bResult = true;
                            }
                        }
                    }
                }
            }
        }
    }

    return bResult;
}


// Initialisation des donnÃ©es de la pÃ©riode d'affectation Ã  calculer
void SymuViaTripNode::InitAssignmentPeriod(TypeVehicule *pTV)
{	
	std::map< std::pair<Tuyau*, SymuViaTripNode*>, deque<AssignmentData> >::iterator itmapTSAss;
	deque<AssignmentData>::iterator itdqAD;

	if( GetMapAssignment().find(pTV) != GetMapAssignment().end() )
	{		
		for( itmapTSAss = GetMapAssignment().find(pTV)->second.begin(); itmapTSAss != GetMapAssignment().find(pTV)->second.end(); itmapTSAss++)
		{
			for( itdqAD = (*itmapTSAss).second.begin(); itdqAD != (*itmapTSAss).second.end(); itdqAD++)
			{
				(*itdqAD).dbCoeffPrev = (*itdqAD).dbCoeff;
				(*itdqAD).dbCoeff = 0;
				(*itdqAD).dbCout = -1;
			}
		}
	}
}

// Ajout d'un Ã©lement

bool SymuViaTripNode::AddEltAssignment(TypeVehicule *pTV, Tuyau *pTAm, SymuViaTripNode* pDestination, const std::vector<Tuyau*> & Ts, double dbCout,
    double dbCoutPenalise, double dbCommonalityFactor, bool bPredefini, const std::string & strRouteId, Connexion * pJunction)
{
    bool bChanged = false;

	AssignmentData assdt;
	std::pair<Tuyau*, SymuViaTripNode*> pairTS(pTAm, pDestination);
	
	assdt.dqTuyaux = Ts;
	assdt.dbCout =	dbCout;	
    assdt.dbCoutPenalise = dbCoutPenalise;	
	assdt.nNbVeh = 0;
    assdt.bPredefini = bPredefini;
    assdt.strRouteId = strRouteId;
    assdt.dbCommonalityFactor = dbCommonalityFactor;
    assdt.pJunction = pJunction;


	if( GetMapAssignment().find(pTV) == GetMapAssignment().end() )
	{
        std::map< std::pair<Tuyau*, SymuViaTripNode*>, deque<AssignmentData> > mapTSAss;
		GetMapAssignment().insert( pair<TypeVehicule*, std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> > >(pTV, mapTSAss) );		
        bChanged = true;
	}
	
	if( GetMapAssignment().find(pTV)->second.find( pairTS ) != GetMapAssignment().find(pTV)->second.end() )
	{
		// Cet itinÃ©raire (ou ce tronÃ§on amont) a t'il dÃ©jÃ  Ã©tÃ© dÃ©fini ?
		std::deque<AssignmentData>::iterator itAss;
		bool bOK = false;
		for( itAss = GetMapAssignment().find(pTV)->second.find( pairTS )->second.begin(); itAss != GetMapAssignment().find(pTV)->second.find( pairTS )->second.end(); itAss++)
		{
            if ((*itAss).dqTuyaux == Ts && (*itAss).pJunction == pJunction)
			{
                if((*itAss).dbCout != dbCout)
                {
                    bChanged = true;
                }
				(*itAss).dbCout = dbCout;
                (*itAss).dbCoutPenalise = dbCoutPenalise;
				bOK = true;
				break;
			}
		}
		if(!bOK)
        {
			GetMapAssignment().find(pTV)->second.find( pairTS )->second.push_back(assdt);	// Nouvel itinÃ©raire Ã  ajouter
            bChanged = true;
        }
	}
	else
	{
		deque<AssignmentData> dqAS;
		dqAS.push_back(assdt);
		GetMapAssignment().find(pTV)->second.insert( pair<pair<Tuyau*, SymuViaTripNode*>, deque<AssignmentData> >(pairTS, dqAS ) );
        bChanged = true;
	}

    return bChanged;
}

ListOfTimeVariation<tracked_double>*  SymuViaTripNode::GetLstDemande(TypeVehicule * pTypeVeh)
{
    if( m_LTVDemande.find(pTypeVeh) == m_LTVDemande.end())
    {
        m_LTVDemande[pTypeVeh]  = new ListOfTimeVariation<tracked_double>();
    }
    return m_LTVDemande[pTypeVeh] ;
}
std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* > SymuViaTripNode::GetLstDemande()const
{  
    return m_LTVDemande;

}
ListOfTimeVariation<tracked_double>* SymuViaTripNode::GetLstDemandeInit(TypeVehicule *pTypeVeh)
{  
    if( m_LTVDemandeInit.find(pTypeVeh) == m_LTVDemandeInit.end())
    {
        m_LTVDemandeInit[pTypeVeh]  = new ListOfTimeVariation<tracked_double>();
    }
    return m_LTVDemandeInit[pTypeVeh];

}
std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* > SymuViaTripNode::GetLstDemandeInit()const
{  

    return m_LTVDemandeInit;

}
std::deque<TimeVariation<RepartitionEntree>>* SymuViaTripNode::GetLstRepVoie(TypeVehicule * pTypeVehicle)
{
    if( m_lstRepVoie.find(pTypeVehicle) == m_lstRepVoie.end())
    {
        m_lstRepVoie[pTypeVehicle]  = new std::deque<TimeVariation<RepartitionEntree> >();
    }
    return m_lstRepVoie[pTypeVehicle];
}

std::deque<TimeVariation<RepartitionEntree>>* SymuViaTripNode::GetLstRepVoieInit(TypeVehicule * pTypeVehicle)
{
    if( m_lstRepVoieInit.find(pTypeVehicle) == m_lstRepVoieInit.end())
    {
        m_lstRepVoieInit[pTypeVehicle]  = new std::deque<TimeVariation<RepartitionEntree> >();
    }
    return m_lstRepVoieInit[pTypeVehicle];
}
std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >  SymuViaTripNode::GetLstRepVoieInit() const
{
    return m_lstRepVoieInit;
}
std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >  SymuViaTripNode::GetLstRepVoie() const
{
    return m_lstRepVoie;
}


void SymuViaTripNode::CopyDemandeInitToDemande(std::deque<TypeVehicule *> typeVehicles)
{    // si la demande est uniquement global -> on copy Ã  tout les types
    if( m_LTVDemandeInit.size() == 1 && m_LTVDemandeInit.find( NULL) != m_LTVDemandeInit.end() )
    {
        GetLstDemande(NULL)->Copy(m_LTVDemandeInit[NULL]);
		for (size_t i = 0; i < typeVehicles.size(); ++i)
        {
            GetLstDemande(typeVehicles[i])->Copy(m_LTVDemandeInit[NULL]);
        }
    }
    else // on copy l'existant
    {
        std::map<TypeVehicule*, ListOfTimeVariation<tracked_double> *>::iterator it;
        for( it = m_LTVDemandeInit.begin(); it != m_LTVDemandeInit.end(); it++)
        {
            GetLstDemande(it->first)->Copy(it->second);
        }
		for (size_t i = 0; i < typeVehicles.size(); ++i)
        {
            if( m_LTVDemandeInit.find(typeVehicles[i]) == m_LTVDemandeInit.end() && 
                 m_LTVDemandeInit.find(NULL) !=  m_LTVDemandeInit.end() )
            {
                GetLstDemande(typeVehicles[i])->Copy(m_LTVDemandeInit[NULL]);
            }
        }
        
    }
}
void SymuViaTripNode::CopyCoeffDestInitToCoeffDest(std::deque<TypeVehicule *> typeVehicles )
{
    // si la coeff est uniquement global -> on copy Ã  tout les types
    if( m_LstCoeffDestInit.size() == 1 && m_LstCoeffDestInit.find( NULL) != m_LstCoeffDestInit.end() )
    { 
        m_LstCoeffDest[NULL].assign( m_LstCoeffDestInit[NULL].begin(), m_LstCoeffDestInit[NULL].end())  ;
		for (size_t i = 0; i < typeVehicles.size(); ++i)
        {
            m_LstCoeffDest[typeVehicles[i] ].assign( m_LstCoeffDestInit[NULL].begin(), m_LstCoeffDestInit[NULL].end())  ;
        }
    }
    else // on copy l'existant
    {
        if( m_LstCoeffDestInit.size() >0 )
        {
            std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > >::iterator it;
            for( it = m_LstCoeffDestInit.begin(); it!= m_LstCoeffDestInit.end() ; it++)
            {
                GetLstCoeffDest(it->first) = (it->second);
            }
			for (size_t i = 0; i < typeVehicles.size(); ++i)
            {
                if( m_LstCoeffDestInit.find(typeVehicles[i]) == m_LstCoeffDestInit.end() && 
                     m_LstCoeffDestInit.find(NULL) !=  m_LstCoeffDestInit.end() )
                {
                    GetLstCoeffDest(typeVehicles[i]) = m_LstCoeffDestInit[NULL];
                }
            }
          
            if( m_LstCoeffDestInit.find(NULL) ==  m_LstCoeffDestInit.end() )
            {
                GetLstCoeffDest(NULL) =  m_LstCoeffDestInit[typeVehicles[0]];
            }
        }
    }
}
void SymuViaTripNode::CopyRepVoieInitToRepVoie(std::deque<TypeVehicule *> typeVehicles)
{
     // si la demande est uniquement global -> on copy Ã  tout les types
    if( m_lstRepVoieInit.size() == 1 && m_lstRepVoieInit.find( NULL) != m_lstRepVoieInit.end() )
    {
      
        (*GetLstRepVoie(NULL)).assign(m_lstRepVoieInit[NULL]->begin(), m_lstRepVoieInit[NULL]->end());
		for (size_t i = 0; i < typeVehicles.size(); ++i)
        {
            (*GetLstRepVoie(typeVehicles[i])).assign(m_lstRepVoieInit[NULL]->begin(), m_lstRepVoieInit[NULL]->end());
        }
    }
    else // on copy l'existant
    {
        std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >::iterator it;
        for( it = m_lstRepVoieInit.begin(); it != m_lstRepVoieInit.end(); it++)
        {
            (*GetLstRepVoie(it->first)).assign(it->second->begin(), it->second->end());
        }
		for (size_t i = 0; i < typeVehicles.size(); ++i)
        {
            if( m_lstRepVoieInit.find(typeVehicles[i]) == m_lstRepVoieInit.end()&& 
                 m_lstRepVoieInit.find(NULL) !=  m_lstRepVoieInit.end())
            {
                GetLstRepVoie(typeVehicles[i])->assign(m_lstRepVoieInit[NULL]->begin(), m_lstRepVoieInit[NULL]->end());
            }
        }
  
    }
}
void SymuViaTripNode::CopyRepMotifDestInitToRepMotifDest(std::deque<TypeVehicule *> typeVehicles)
{
    // si la coeff est uniquement global -> on copy Ã  tout les types
    if (m_lstRepMotifInit.size() == 1 && m_lstRepMotifInit.find(NULL) != m_lstRepMotifInit.end())
    {
        m_lstRepMotif[NULL].assign(m_lstRepMotifInit[NULL].begin(), m_lstRepMotifInit[NULL].end());
        for (size_t i = 0; i < typeVehicles.size(); ++i)
        {
            m_lstRepMotif[typeVehicles[i]].assign(m_lstRepMotifInit[NULL].begin(), m_lstRepMotifInit[NULL].end());
        }
    }
    else // on copy l'existant
    {
        if (m_lstRepMotifInit.size() >0)
        {
            std::map<TypeVehicule*, std::deque<TimeVariation<CRepMotif> > >::iterator it;
            for (it = m_lstRepMotifInit.begin(); it != m_lstRepMotifInit.end(); it++)
            {
                GetLstRepMotif(it->first) = (it->second);
            }
            for (size_t i = 0; i < typeVehicles.size(); ++i)
            {
                if (m_lstRepMotifInit.find(typeVehicles[i]) == m_lstRepMotifInit.end() &&
                    m_lstRepMotifInit.find(NULL) != m_lstRepMotifInit.end())
                {
                    GetLstRepMotif(typeVehicles[i]) = m_lstRepMotifInit[NULL];
                }
            }

            if (m_lstRepMotifInit.find(NULL) == m_lstRepMotifInit.end())
            {
                GetLstRepMotif(NULL) = m_lstRepMotifInit[typeVehicles[0]];
            }
        }
    }
}
void SymuViaTripNode::SetLstCoeffDestInit(SymuViaTripNode * pDest, TypeVehicule * pTypeVeh, const std::vector<std::pair<double, std::pair<std::vector<Tuyau*>, Connexion*> > > & routes)
{
    // Pour pouvoir appeler la mÃ©thode en cours de simulation ! (SymuMaster)
    std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > > * pLstCoeffDest = m_pNetwork->IsInitSimuTrafic() ? &m_LstCoeffDest : &m_LstCoeffDestInit;

    // Pour tous les types de vÃ©hicules ...
    std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > >::iterator it;
    for (it = pLstCoeffDest->begin(); it != pLstCoeffDest->end(); it++)
    {
        if(pTypeVeh == NULL || pTypeVeh == it->first)
        {
            std::deque<TimeVariation<SimMatOrigDest> > & lstTimeVariations = it->second;

            // Modification de toutes les variantes temporelles de la liste
            for(size_t iVariation = 0; iVariation < lstTimeVariations.size(); iVariation++)
            {
                TimeVariation<SimMatOrigDest> & variation = lstTimeVariations[iVariation];
                boost::shared_ptr<SimMatOrigDest> pMatOrigDest = variation.m_pData;

                // on recherche la destination dans la liste
                VectDest * pVectDest = NULL;
                for(size_t iDest = 0; iDest < pMatOrigDest->MatOrigDest.size(); iDest++)
                {
                    if(pMatOrigDest->MatOrigDest[iDest]->pDest == pDest)
                    {
                        pVectDest = pMatOrigDest->MatOrigDest[iDest];

                        pVectDest->bHasTetaLogit = false;
                        pVectDest->bHasNbPlusCourtsChemins = false;
                        pVectDest->bHasCustomCommonalityParameters = false;
                        pVectDest->dbRelicatCoeff = 0;
                        pVectDest->lstItineraires.clear();

                        for(size_t iIti = 0; iIti < routes.size(); iIti++)
                        {
                            pVectDest->lstItineraires.push_back(std::make_pair(routes[iIti].first, std::make_pair(std::make_pair(routes[iIti].second.first, routes[iIti].second.second), "")));
                        }

                        break;
                    }
                }
            }
        }
    }
}



bool VectDest::operator==(const VectDest& rhs)const
{
    return bHasTetaLogit == rhs.bHasTetaLogit
        && dbTetaLogit == rhs.dbTetaLogit
        && bHasNbPlusCourtsChemins == rhs.bHasNbPlusCourtsChemins
        && nNbPlusCourtsChemins == rhs.nNbPlusCourtsChemins
        && bHasCustomCommonalityParameters == rhs.bHasCustomCommonalityParameters
        && dbCommonalityAlpha == rhs.dbCommonalityAlpha
        && dbCommonalityBeta == rhs.dbCommonalityBeta
        && dbCommonalityGamma == rhs.dbCommonalityGamma
        && lstItineraires == rhs.lstItineraires;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void CreationVehicule::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void CreationVehicule::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void CreationVehicule::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(dbInstantCreation);
    ar & BOOST_SERIALIZATION_NVP(pTypeVehicule);
    ar & BOOST_SERIALIZATION_NVP(pDest);
    ar & BOOST_SERIALIZATION_NVP(nVoie);
}

template void RepartitionEntree::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void RepartitionEntree::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void RepartitionEntree::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(pCoefficients);
}

template void VectDest::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void VectDest::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void VectDest::serialize(Archive & ar, const unsigned int version)
{
	ar & BOOST_SERIALIZATION_NVP(pDest);
    ar & BOOST_SERIALIZATION_NVP(dbCoeff);
    ar & BOOST_SERIALIZATION_NVP(bHasTetaLogit);
    ar & BOOST_SERIALIZATION_NVP(dbTetaLogit);
    ar & BOOST_SERIALIZATION_NVP(bHasNbPlusCourtsChemins);
    ar & BOOST_SERIALIZATION_NVP(nNbPlusCourtsChemins);
    ar & BOOST_SERIALIZATION_NVP(bHasCustomCommonalityParameters);
    ar & BOOST_SERIALIZATION_NVP(dbCommonalityAlpha);
    ar & BOOST_SERIALIZATION_NVP(dbCommonalityBeta);
    ar & BOOST_SERIALIZATION_NVP(dbCommonalityGamma);

    ar & BOOST_SERIALIZATION_NVP(lstItineraires);
    ar & BOOST_SERIALIZATION_NVP(dbRelicatCoeff);

    ar & BOOST_SERIALIZATION_NVP(lstRouteNames);
}

template void SimMatOrigDest::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void SimMatOrigDest::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void SimMatOrigDest::serialize(Archive & ar, const unsigned int version)
{
	ar & BOOST_SERIALIZATION_NVP(MatOrigDest);
}

template void SymuViaTripNode::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void SymuViaTripNode::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void SymuViaTripNode::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(TripNode);

	ar & BOOST_SERIALIZATION_NVP(m_mapVehEnAttente);
    ar & BOOST_SERIALIZATION_NVP(m_mapVehPret);
    ar & BOOST_SERIALIZATION_NVP(m_dbCritCreationVeh);

    ar & BOOST_SERIALIZATION_NVP(m_LstCoeffDest);
    ar & BOOST_SERIALIZATION_NVP(m_LstCoeffDestInit);

    ar & BOOST_SERIALIZATION_NVP(m_pLstRepTypeVeh);

    ar & BOOST_SERIALIZATION_NVP(m_LstDestinations);
    size_t nbDestination = m_LstDestinations.size();
    SerialiseDeque2D<Archive, int>(ar, "m_LogMatriceOD", m_LogMatriceOD, nbDestination);

    ar & BOOST_SERIALIZATION_NVP(m_mapTauxAgressivite);

    ar & BOOST_SERIALIZATION_NVP(m_LTVDemandeInit);
    ar & BOOST_SERIALIZATION_NVP(m_LTVDemande);

    ar & BOOST_SERIALIZATION_NVP(m_lstRepVoie);
    ar & BOOST_SERIALIZATION_NVP(m_lstRepVoieInit);

    ar & BOOST_SERIALIZATION_NVP(m_lstRepMotif);
    ar & BOOST_SERIALIZATION_NVP(m_lstRepMotifInit);

    ar & BOOST_SERIALIZATION_NVP(Nb_variations);
    ar & BOOST_SERIALIZATION_NVP(m_dqNbVehicules);
    ar & BOOST_SERIALIZATION_NVP(m_bDemande);
    ar & BOOST_SERIALIZATION_NVP(m_bDistribution);
    ar & BOOST_SERIALIZATION_NVP(m_dbLastInstCreation);

    ar & BOOST_SERIALIZATION_NVP(m_LstCreationsVehicule);
    ar & BOOST_SERIALIZATION_NVP(m_pTypeVehiculeACreer);
    ar & BOOST_SERIALIZATION_NVP(m_pDestinationVehiculeACreer);
    ar & BOOST_SERIALIZATION_NVP(m_nVoieACreer);

    ar & BOOST_SERIALIZATION_NVP(m_lstSortiesStationnement);
}
