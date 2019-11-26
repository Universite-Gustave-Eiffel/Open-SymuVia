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
#include "SymuCoreDrivingGraphBuilder.h"

#include "SymuCoreGraphBuilder.h"

#include "reseau.h"
#include "tuyau.h"
#include "ControleurDeFeux.h"
#include "entree.h"
#include "sortie.h"
#include "Parking.h"
#include "TronconDrivingPattern.h"
#include "ZoneDrivingPattern.h"
#include "PlaqueDrivingPattern.h"
#include "ConnectionDrivingPenalty.h"
#include "Plaque.h"
#include "VehicleTypePenalty.h"

#include <Graph/Model/MultiLayersGraph.h>
#include <Graph/Model/Link.h>
#include <Graph/Model/Node.h>
#include <Graph/Model/Driving/DrivingPattern.h>
#include <Graph/Model/PatternsSwitch.h>
#include <Demand/MacroType.h>
#include <Utils/Point.h>
#include <Graph/Model/NullPenalty.h>
#include <Graph/Model/Driving/DrivingPenalty.h>
#include <Demand/Population.h>
#include <Demand/Trip.h>
#include <Demand/Path.h>
#include <Demand/Origin.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>

using namespace SymuCore;

SymuCoreDrivingGraphBuilder::SymuCoreDrivingGraphBuilder()
: m_pParent(NULL),
m_pNetwork(NULL),
m_bForSymuMaster(true)
{
}

SymuCoreDrivingGraphBuilder::SymuCoreDrivingGraphBuilder(Reseau * pNetwork, bool bForSymuMaster)
: m_pParent(NULL),
m_pNetwork(pNetwork),
m_bForSymuMaster(bForSymuMaster)
{
}


SymuCoreDrivingGraphBuilder::~SymuCoreDrivingGraphBuilder()
{
}

void SymuCoreDrivingGraphBuilder::SetParent(SymuCoreGraphBuilder * pParent)
{
	m_pParent = pParent;
}

SymuCoreGraphBuilder * SymuCoreDrivingGraphBuilder::GetParent()
{
    return m_pParent;
}

bool SymuCoreDrivingGraphBuilder::CreateDrivingLayer(SymuCore::MultiLayersGraph * pGraph, std::vector<Node*> & lstOrigins, std::vector<Node*> & lstDestinations)
{
	Graph * pDrivingLayer = pGraph->CreateAndAddLayer(SymuCore::ST_Driving);

    // 1 - crÃ©ation des Nodes
    Connexion * pConnexion;
    Node * pNode;
    for (size_t iNode = 0; iNode < m_pNetwork->m_LstUserCnxs.size(); iNode++)
    {
        pConnexion = m_pNetwork->m_LstUserCnxs[iNode];
        const std::string & connectionId = pConnexion->GetID();

        if (!m_pNetwork->GetParkingFromID(connectionId))
        {
            pConnexion->calculBary();
            SymuCore::Point pt(pConnexion->GetAbscisse(), pConnexion->GetOrdonnee());

            NodeType nodeType = NT_RoadJunction;
            SymuViaTripNode * pExternalConnection = dynamic_cast<SymuViaTripNode*>(pConnexion);
            if (pExternalConnection)
            {
                nodeType = NT_RoadExtremity;
            }

            pNode = pDrivingLayer->CreateAndAddNode(connectionId, nodeType, &pt);

            // no need to add extremities in those maps.
            if (nodeType == NT_RoadJunction)
            {
                m_mapInternalConnexionsToNode[pConnexion] = pNode;
                m_mapNodesToInternalConnexions[pNode] = pConnexion;
            }
            else
            {
                m_mapExternalConnexionsToNode[pExternalConnection] = pNode;
                m_mapNodesToExternalConnexions[pNode] = pExternalConnection;
            }

            if (m_bForSymuMaster)
            {
                //On verifie si il s'agit d'origines ou destinations
                if (pDrivingLayer->GetOriginIfExists(connectionId))
                {
                    lstOrigins.push_back(pNode);
                }
                if (pDrivingLayer->GetDestinationIfExists(connectionId))
                {
                    lstDestinations.push_back(pNode);
                }
            }
        }
    }
    // gestion des nodes pour les parkings
    for (size_t iNode = 0; iNode < m_pNetwork->Liste_parkings.size(); iNode++)
    {
        Parking * pParking = m_pNetwork->Liste_parkings.at(iNode);
        const std::string & connectionId = pParking->GetID();
        
        ::Point symPt = pParking->getCoordonnees();
        SymuCore::Point pt(symPt.dbX, symPt.dbY);

        NodeType nodeType = NT_RoadExtremity;

        pNode = pDrivingLayer->CreateAndAddNode(connectionId, nodeType, &pt);

        m_mapExternalConnexionsToNode[pParking] = pNode;
        m_mapNodesToExternalConnexions[pNode] = pParking;

        if (m_bForSymuMaster)
        {
            //On verifie si il s'agit d'origines ou destinations
            if (pDrivingLayer->GetOriginIfExists(connectionId))
            {
                lstOrigins.push_back(pNode);
            }
            if (pDrivingLayer->GetDestinationIfExists(connectionId))
            {
                lstDestinations.push_back(pNode);
            }
        }
    }

    // 2 - crÃ©ation des Links
    Tuyau * pTuyau;
    Link * pLink;
    std::map<std::pair<Node*, Node*>, Link*> mapLinks;
    for (size_t iTuy = 0; iTuy < m_pNetwork->m_LstTuyaux.size(); iTuy++)
    {
        pTuyau = m_pNetwork->m_LstTuyaux[iTuy];

        // On ignore les tuyaux internes
        if (!pTuyau->GetBriqueParente())
        {
            // On crÃ©e un seul Link mÃªme si plusieurs tuyaux relient le mÃªme couple noeud amont + noeud aval
            std::pair<Node*, Node*> linkDef = std::make_pair(GetNodeFromConnexion(pTuyau->GetCnxAssAm()), GetNodeFromConnexion(pTuyau->GetCnxAssAv()));

            std::map<std::pair<Node*, Node*>, Link*>::const_iterator iterLink = mapLinks.find(linkDef);
            if (iterLink == mapLinks.end())
            {
                // pas de doublon pour le moment : on crÃ©e le link
                pLink = pDrivingLayer->CreateAndAddLink(linkDef.first, linkDef.second);
                mapLinks[linkDef] = pLink;
            }
            else
            {
                // cas du doublon
                pLink = iterLink->second;
            }

            m_mapLinkToTuyaux[pLink].insert(pTuyau);
        }
    }

    // 3 - gestion des zones
    for (size_t iZone = 0; iZone < m_pNetwork->Liste_zones.size(); iZone++)
    {
        ZoneDeTerminaison * pZone = m_pNetwork->Liste_zones[iZone];

        Origin * pZoneOrigin;
        Destination * pZoneDestination;
        bool bProcessZone;
        if (m_bForSymuMaster)
        {
            pZoneOrigin = pDrivingLayer->GetOriginIfExists(pZone->GetID());
            pZoneDestination = pDrivingLayer->GetDestinationIfExists(pZone->GetID());
            bProcessZone = pZoneOrigin || pZoneDestination;
        }
        else
        {
            bProcessZone = true;
        }

        if (bProcessZone)
        {
            // CrÃ©ation du Node correspondant Ã  la zone
            ::Point symPt;
            pZone->GetInputPosition().ComputeBarycentre(symPt);
            SymuCore::Point pt(symPt.dbX, symPt.dbY);
            // rmq. : on crÃ©e les zones dans la couche parente car commun aux couches VLs et TECs
            pNode = pDrivingLayer->getParent()->CreateAndAddNode(pZone->GetID(), NT_Area, &pt);

            // liaisons pour accÃ©der Ã  la zone
            if (!m_bForSymuMaster)
            {
                std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > zoneNodes = pZone->GetInputJunctions();
                for (std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> >::const_iterator iterCon = zoneNodes.begin(); iterCon != zoneNodes.end(); ++iterCon)
                {
                    // CrÃ©ation du Link arrivant Ã  la zone
                    pDrivingLayer->CreateAndAddLink(GetNodeFromConnexion(iterCon->first), pNode);
                }
            }
            else if (pZoneDestination)
            {
                std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > zoneNodes = pZone->GetInputJunctions();
                for (std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> >::const_iterator iterCon = zoneNodes.begin(); iterCon != zoneNodes.end(); ++iterCon)
                {
                    pZone->m_AreaHelper.ComputeIncomingPaths(pZone, iterCon->first, iterCon->second, pGraph->GetListMacroTypes());

                    if (pZone->m_AreaHelper.isAcessibleFromConnexion(iterCon->first))
                    {
                        // CrÃ©ation du Link arrivant Ã  la zone
                        pDrivingLayer->CreateAndAddLink(GetNodeFromConnexion(iterCon->first), pNode);
                    }
                }
                lstDestinations.push_back(pNode);
            }

            // liaisons pour sortir de la zone
            if (!m_bForSymuMaster)
            {
                std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > zoneNodes = pZone->GetOutputJunctions();
                for (std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> >::const_iterator iterCon = zoneNodes.begin(); iterCon != zoneNodes.end(); ++iterCon)
                {
                    // CrÃ©ation du Link partant de la zone
                    pDrivingLayer->CreateAndAddLink(pNode, GetNodeFromConnexion(iterCon->first));
                }
            }
            else if (pZoneOrigin)
            {
                std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > zoneNodes = pZone->GetOutputJunctions();
                for (std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> >::const_iterator iterCon = zoneNodes.begin(); iterCon != zoneNodes.end(); ++iterCon)
                {
                    pZone->m_AreaHelper.ComputeOutgoingPaths(pZone, iterCon->first, iterCon->second, pGraph->GetListMacroTypes());

                    if (pZone->m_AreaHelper.isAcessibleToConnexion(iterCon->first))
                    {
                        // CrÃ©ation du Link partant de la zone
                        pDrivingLayer->CreateAndAddLink(pNode, GetNodeFromConnexion(iterCon->first));
                    }
                }
                lstOrigins.push_back(pNode);
            }

            m_mapZonesToNode[pZone] = pNode;
        }

        if (m_bForSymuMaster)
        {
            for (size_t iSubArea = 0; iSubArea < pZone->GetLstPlaques().size(); iSubArea++)
            {
                CPlaque * pSubArea = pZone->GetLstPlaques()[iSubArea];

                Origin * pPlaqueOrigin = pDrivingLayer->GetOriginIfExists(pSubArea->GetID());
                Destination * pPlaqueDestination = pDrivingLayer->GetDestinationIfExists(pSubArea->GetID());

                if (pPlaqueOrigin || pPlaqueDestination)
                {
                    // CrÃ©ation du Node correspondant Ã  la plaque
                    ::Point symPt;
                    pSubArea->ComputeBarycentre(symPt);
                    SymuCore::Point pt(symPt.dbX, symPt.dbY);
                    // rmq. : on crÃ©e les zones dans la couche parente car commun aux couches VLs et TECs
                    pNode = pDrivingLayer->getParent()->CreateAndAddNode(pSubArea->GetID(), NT_SubArea, &pt);

                    // liaisons pour accÃ©der Ã  la plaque
                    if (pPlaqueDestination)
                    {
                        std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > zoneNodes = pZone->GetInputJunctions();
                        for (std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> >::const_iterator iterCon = zoneNodes.begin(); iterCon != zoneNodes.end(); ++iterCon)
                        {
                            pSubArea->m_AreaHelper.ComputeIncomingPaths(NULL, iterCon->first, iterCon->second, pGraph->GetListMacroTypes());

                            if (pSubArea->m_AreaHelper.isAcessibleFromConnexion(iterCon->first))
                            {
                                // CrÃ©ation du Link arrivant Ã  la plaque
                                pDrivingLayer->CreateAndAddLink(GetNodeFromConnexion(iterCon->first), pNode);
                            }
                        }
                        lstDestinations.push_back(pNode);
                    }

                    // liaisons pour sortir de la plaque
                    if (pPlaqueOrigin)
                    {
                        std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> > zoneNodes = pZone->GetOutputJunctions();
                        for (std::map<Connexion*, std::vector<Tuyau*>, LessConnexionPtr<Connexion> >::const_iterator iterCon = zoneNodes.begin(); iterCon != zoneNodes.end(); ++iterCon)
                        {
                            pSubArea->m_AreaHelper.ComputeOutgoingPaths(NULL, iterCon->first, iterCon->second, pGraph->GetListMacroTypes());

                            if (pSubArea->m_AreaHelper.isAcessibleToConnexion(iterCon->first))
                            {
                                // CrÃ©ation du Link partant de la plaque
                                pDrivingLayer->CreateAndAddLink(pNode, GetNodeFromConnexion(iterCon->first));
                            }
                        }
                        lstOrigins.push_back(pNode);
                    }

                    m_mapPlaquesToNodes[pSubArea] = pNode;
                }
            }
        }
    }

    // 4 - crÃ©ation des patterns et penalties
    BuildPatternsAndPenalties(pDrivingLayer);

    return true;
}

void SymuCoreDrivingGraphBuilder::BuildPatternsAndPenalties(SymuCore::Graph * pDrivingLayer)
{
    // 1 - crÃ©ation des patterns
    BuildPatterns(pDrivingLayer);

    // 2 - crÃ©ation des penalties
    BuildPenalties(pDrivingLayer);

    // 3 - gestion des patterns et Penalties liÃ©s aux zones et plaques
    BuildPatternsInZones(pDrivingLayer);
    if (m_bForSymuMaster)
    {
        BuildPatternsInPlaques(pDrivingLayer);
    }
    BuildPenaltiesForZones(pDrivingLayer);
    if (m_bForSymuMaster)
    {
        BuildPenaltiesForPlaques(pDrivingLayer);
    }
}

 
bool SymuCoreDrivingGraphBuilder::CompareLink::operator()(const SymuCore::Link* l1, const SymuCore::Link* l2) const
{
    return l1->getUpstreamNode()->getStrName() < l2->getUpstreamNode()->getStrName()
        || (l1->getUpstreamNode()->getStrName() == l2->getUpstreamNode()->getStrName() && l1->getDownstreamNode()->getStrName() < l2->getDownstreamNode()->getStrName());
}

void SymuCoreDrivingGraphBuilder::BuildPatterns(SymuCore::Graph * pDrivingLayer)
{
    std::map<Link*, std::set<Tuyau*, LessPtr<Tuyau> >, CompareLink>::const_iterator iterLink;
    Link * pLink;
    for (iterLink = m_mapLinkToTuyaux.begin(); iterLink != m_mapLinkToTuyaux.end(); ++iterLink)
    {
        pLink = iterLink->first;

        assert(pLink->getParent() == pDrivingLayer);

        const std::set<Tuyau*, LessPtr<Tuyau> > & tuyaux = iterLink->second;

        // Pour chaque tronÃ§on (un seul en fait sauf si doublons), on a un pattern :
        for (std::set<Tuyau*, LessPtr<Tuyau> >::const_iterator iterTuy = tuyaux.begin(); iterTuy != tuyaux.end(); ++iterTuy)
        {
            TronconDrivingPattern* pDrivingPattern = new TronconDrivingPattern(pLink, m_pNetwork, (TuyauMicro*)*iterTuy);
            pLink->getListPatterns().push_back(pDrivingPattern);
        }
    }
}

void SymuCoreDrivingGraphBuilder::BuildPenalties(SymuCore::Graph * pDrivingLayer)
{
    std::map<Connexion*, SymuCore::Node*>::const_iterator iterNode;
    Link * pUpstreamLink, *pDownstreamLink;
    Tuyau * pUpstreamTuy, *pDownstreamTuy;
    Connexion * pCon;
    Node * pNode;
    for (iterNode = m_mapInternalConnexionsToNode.begin(); iterNode != m_mapInternalConnexionsToNode.end(); ++iterNode)
    {
        pNode = iterNode->second;

        assert(pNode->getParent() == pDrivingLayer);

        pCon = iterNode->first;

		// Pour chaque lien amont
		for (size_t iUpstreamLink = 0; iUpstreamLink < pNode->getUpstreamLinks().size(); iUpstreamLink++)
		{
			pUpstreamLink = pNode->getUpstreamLinks()[iUpstreamLink];

            // pas encore de liens intercouches Ã  ce niveau
            assert(pUpstreamLink->getParent() == pDrivingLayer);

            // Pour chaque pattern amont
            for (size_t iUpstreamPattern = 0; iUpstreamPattern < pUpstreamLink->getListPatterns().size(); iUpstreamPattern++)
            {
                TronconDrivingPattern * pUpstreamPattern = (TronconDrivingPattern*)pUpstreamLink->getListPatterns()[iUpstreamPattern];
                pUpstreamTuy = pUpstreamPattern->GetTuyau();

                // Pour chaque lien aval
                for (size_t iDownstreamLink = 0; iDownstreamLink < pNode->getDownstreamLinks().size(); iDownstreamLink++)
                {
                    pDownstreamLink = pNode->getDownstreamLinks()[iDownstreamLink];

                    // pas encore de liens intercouches Ã  ce niveau
                    assert(pDownstreamLink->getParent() == pDrivingLayer);
                    
                    // Pour chaque pattern aval
                    for (size_t iDownstreamPattern = 0; iDownstreamPattern < pDownstreamLink->getListPatterns().size(); iDownstreamPattern++)
                    {
                        TronconDrivingPattern * pDownstreamPattern = (TronconDrivingPattern*)pDownstreamLink->getListPatterns()[iDownstreamPattern];
                        pDownstreamTuy = pDownstreamPattern->GetTuyau();

                        ConnectionDrivingPenalty * pPenalty = new ConnectionDrivingPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern), m_pNetwork, pCon);
                        pNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = pPenalty;
                    }
                }
            }
		}
    }
}

void SymuCoreDrivingGraphBuilder::BuildPatternsInZones(SymuCore::Graph * pDrivingLayer)
{
    Node * pOtherNode, *pNode;
    Link * pLink;
    std::map<ZoneDeTerminaison*, SymuCore::Node*, LessConnexionPtr<ZoneDeTerminaison> >::const_iterator iterZone;
    for (iterZone = m_mapZonesToNode.begin(); iterZone != m_mapZonesToNode.end(); ++iterZone)
    {
        pNode = iterZone->second;

        // Pour tous les Links aval reliÃ©s Ã  la zone et dont le Node aval est gÃ©rÃ© par SymuVia :
        for (size_t iLink = 0; iLink < pNode->getDownstreamLinks().size(); iLink++)
        {
            pLink = pNode->getDownstreamLinks()[iLink];
            pOtherNode = pLink->getDownstreamNode();

            assert(pOtherNode->getParent() == pDrivingLayer);

            ZoneDrivingPattern* pUpstreamPattern = new ZoneDrivingPattern(pLink, iterZone->first, GetConnexionFromNode(pOtherNode, true), true);
            pLink->getListPatterns().push_back(pUpstreamPattern);
        }

        // Pour tous les Links amont reliÃ©s Ã  la zone et dont le Node amont est gÃ©rÃ© par SymuVia :
        for (size_t iLink = 0; iLink < pNode->getUpstreamLinks().size(); iLink++)
        {
            pLink = pNode->getUpstreamLinks()[iLink];
            pOtherNode = pLink->getUpstreamNode();

            assert(pOtherNode->getParent() == pDrivingLayer);

            ZoneDrivingPattern * pDownstreamPattern = new ZoneDrivingPattern(pLink, iterZone->first, GetConnexionFromNode(pOtherNode, false), false);
            pLink->getListPatterns().push_back(pDownstreamPattern);
        }
    }
}

void SymuCoreDrivingGraphBuilder::BuildPenaltiesForZones(SymuCore::Graph * pDrivingLayer)
{
    Node * pOtherNode, *pNode;
    Link * pLink, *pDownstreamLink, *pUpstreamLink;
    std::map<ZoneDeTerminaison*, SymuCore::Node*, LessConnexionPtr<ZoneDeTerminaison> >::const_iterator iterZone;
    for (iterZone = m_mapZonesToNode.begin(); iterZone != m_mapZonesToNode.end(); ++iterZone)
    {
        pNode = iterZone->second;

        // Pour tous les Links aval reliÃ©s Ã  la zone et dont le Node aval est gÃ©rÃ© par SymuVia et est une connexion interne :
        for (size_t iLink = 0; iLink < pNode->getDownstreamLinks().size(); iLink++)
        {
            pLink = pNode->getDownstreamLinks()[iLink];
            pOtherNode = pLink->getDownstreamNode();

            std::map<SymuCore::Node*, Connexion*>::const_iterator iterInternalNode = m_mapNodesToInternalConnexions.find(pOtherNode);
            if (iterInternalNode != m_mapNodesToInternalConnexions.end())
            {
                // Pas de lien intercouche Ã  ce stade en principe
                assert(pOtherNode->getParent() == pDrivingLayer);

                assert(pLink->getListPatterns().size() == 1);
                ZoneDrivingPattern* pUpstreamPattern = (ZoneDrivingPattern*)pLink->getListPatterns().front();

                // CrÃ©ation des Penalties entre ce pattern et les patterns aval
                for (size_t iDownstreamLink = 0; iDownstreamLink < pOtherNode->getDownstreamLinks().size(); iDownstreamLink++)
                {
                    pDownstreamLink = pOtherNode->getDownstreamLinks()[iDownstreamLink];

                    // Pas de lien intercouche Ã  ce stade en principe
                    assert(pDownstreamLink->getParent() == pDrivingLayer);

                    for (size_t iDownstreamPattern = 0; iDownstreamPattern < pDownstreamLink->getListPatterns().size(); iDownstreamPattern++)
                    {
                        Pattern * pDownstreamPattern = pDownstreamLink->getListPatterns()[iDownstreamPattern];

                        // si le tronÃ§on aval n'est pas accessible depuis le noeud (ce qui peut arriver si le rÃ©seau
                        // est "mal dÃ©fini" avec un tronÃ§on qui part d'un noeud mais sans mouvement autorisÃ©), il ne faut pas mettre de mouvement possible !!
                        TronconDrivingPattern * pTronconPattern = dynamic_cast<TronconDrivingPattern*>(pDownstreamPattern);
                        if (pTronconPattern)
                        {
                            std::set<TypeVehicule*> lstAllowedVehicleTypes;
                            if (iterZone->first->m_AreaHelper.isAcessibleToConnexion(iterInternalNode->second, pTronconPattern->GetTuyau(), m_bForSymuMaster, lstAllowedVehicleTypes))
                            {
                                if (m_bForSymuMaster)
                                {
                                    // CrÃ©ation du switching cost nul entre les deux patterns (cf. Ludo, mieux vaut mettre 0 qu'un coÃ»t faux qui serait par exemple
                                    // la moyenne des pÃ©nalitÃ©s de chaque tronÃ§on amont dans la zone permettant d'en sortir au point de jonction considÃ©rÃ©) :
                                    pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                }
                                else
                                {
                                    // Cas SymuVia seul : le test isAcessibleToConnexion dÃ©pend du type de vÃ©hicule :
                                    pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new VehicleTypePenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern), m_pNetwork, lstAllowedVehicleTypes);
                                }
                            }
                        }
                        else
                        {
                            // Cas de la zone jointive sur le point de jonction :
                            ZoneDrivingPattern * pDownstreamZonePattern = dynamic_cast<ZoneDrivingPattern*>(pDownstreamPattern);
                            if (pDownstreamZonePattern)
                            {
                                // Pas de pÃ©nalitÃ© pour rentrer directement dans la zone d'oÃ¹ l'on vient (pas autorisÃ©)
                                if (pDownstreamZonePattern->getZone() != iterZone->first)
                                {
                                    std::set<TypeVehicule*> lstAllowedVehicleTypes;
                                    if (iterZone->first->m_AreaHelper.isAccessibleToArea(iterInternalNode->second, &pDownstreamZonePattern->getZone()->m_AreaHelper, m_bForSymuMaster, lstAllowedVehicleTypes))
                                    {
                                        // la penalitÃ© peut dÃ©jÃ  exister si on a traitÃ© l'autre zone avant :
                                        std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iterPenalty = pOtherNode->getMapPenalties().find(pUpstreamPattern);
                                        if (iterPenalty == pOtherNode->getMapPenalties().end())
                                        {
                                            if (m_bForSymuMaster)
                                            {
                                                pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                            }
                                            else
                                            {
                                                pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new VehicleTypePenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern), m_pNetwork, lstAllowedVehicleTypes);
                                            }
                                        }
                                        else if (iterPenalty->second.find(pDownstreamPattern) == iterPenalty->second.end())
                                        {
                                            if (m_bForSymuMaster)
                                            {
                                                iterPenalty->second[pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                            }
                                            else
                                            {
                                                iterPenalty->second[pDownstreamPattern] = new VehicleTypePenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern), m_pNetwork, lstAllowedVehicleTypes);
                                            }
                                        }
                                    }
                                }
                            }
                            else
                            {
                                assert(m_bForSymuMaster);

                                // Cas de la plaque jointive
                                PlaqueDrivingPattern * pDownstreamPlaquePattern = dynamic_cast<PlaqueDrivingPattern*>(pDownstreamPattern);
                                if (pDownstreamPlaquePattern)
                                {
                                    std::set<TypeVehicule*> lstAllowedVehicleTypes;
                                    if (iterZone->first->m_AreaHelper.isAccessibleToArea(iterInternalNode->second, &pDownstreamPlaquePattern->getPlaque()->m_AreaHelper, m_bForSymuMaster, lstAllowedVehicleTypes))
                                    {
                                        // la penalitÃ© peut dÃ©jÃ  exister si on a traitÃ© l'autre zone avant :
                                        std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iterPenalty = pOtherNode->getMapPenalties().find(pUpstreamPattern);
                                        if (iterPenalty == pOtherNode->getMapPenalties().end())
                                        {
                                            pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                        }
                                        else if (iterPenalty->second.find(pDownstreamPattern) == iterPenalty->second.end())
                                        {
                                            iterPenalty->second[pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Pour tous les Links amont reliÃ©s Ã  la zone et dont le Node amont est gÃ©rÃ© par SymuVia et est une connexion interne :
        for (size_t iLink = 0; iLink < pNode->getUpstreamLinks().size(); iLink++)
        {
            pLink = pNode->getUpstreamLinks()[iLink];
            pOtherNode = pLink->getUpstreamNode();

            std::map<SymuCore::Node*, Connexion*>::const_iterator iterInternalNode = m_mapNodesToInternalConnexions.find(pOtherNode);
            if (iterInternalNode != m_mapNodesToInternalConnexions.end())
            {
                // Pas de liens intercouches Ã  ce stade en principe
                assert(pOtherNode->getParent() == pDrivingLayer);

                assert(pLink->getListPatterns().size() == 1);
                ZoneDrivingPattern* pDownstreamPattern = (ZoneDrivingPattern*)pLink->getListPatterns().front();

                // CrÃ©ation des Penalties entre ce pattern et les patterns aval
                for (size_t iUpstreamLink = 0; iUpstreamLink < pOtherNode->getUpstreamLinks().size(); iUpstreamLink++)
                {
                    pUpstreamLink = pOtherNode->getUpstreamLinks()[iUpstreamLink];

                    // Pas de lien intercouche Ã  ce stade en principe
                    assert(pUpstreamLink->getParent() == pDrivingLayer);

                    for (size_t iUpstreamPattern = 0; iUpstreamPattern < pUpstreamLink->getListPatterns().size(); iUpstreamPattern++)
                    {
                        Pattern * pUpstreamPattern = pUpstreamLink->getListPatterns()[iUpstreamPattern];

                        // si le tronÃ§on amont ne permet pas de rentrer dans le noeud (ce qui peut arriver si le rÃ©seau
                        // est "mal dÃ©fini" avec un tronÃ§on qui arrive vers un noeud mais sans mouvement autorisÃ©), il ne faut pas mettre de mouvement possible !!
                        TronconDrivingPattern * pTronconPattern = dynamic_cast<TronconDrivingPattern*>(pUpstreamPattern);
                        if (pTronconPattern)
                        {
                            std::set<TypeVehicule*> lstAllowedVehicleTypes;
                            if (iterZone->first->m_AreaHelper.isAcessibleFromConnexion(iterInternalNode->second, pTronconPattern->GetTuyau(), m_bForSymuMaster, lstAllowedVehicleTypes))
                            {
                                if (m_bForSymuMaster)
                                {
                                    // CrÃ©ation du switching cost nul entre les deux patterns :
                                    pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                }
                                else
                                {
                                    pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new VehicleTypePenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern), m_pNetwork, lstAllowedVehicleTypes);
                                }
                            }
                        }
                        else
                        {
                            // Cas de la zone jointive sur le point de jonction :
                            ZoneDrivingPattern * pUpstreamZonePattern = dynamic_cast<ZoneDrivingPattern*>(pUpstreamPattern);
                            if (pUpstreamZonePattern)
                            {
                                // Pas de pÃ©nalitÃ© pour rentrer directement dans la zone d'oÃ¹ l'on vient
                                if (pUpstreamZonePattern->getZone() != iterZone->first)
                                {
                                    std::set<TypeVehicule*> lstAllowedVehicleTypes;
                                    if (iterZone->first->m_AreaHelper.isAccessibleFromArea(iterInternalNode->second, &pUpstreamZonePattern->getZone()->m_AreaHelper, m_bForSymuMaster, lstAllowedVehicleTypes))
                                    {
                                        // la penalitÃ© peut dÃ©jÃ  exister si on a traitÃ© l'autre zone avant :
                                        std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iterPenalty = pOtherNode->getMapPenalties().find(pUpstreamPattern);
                                        if (iterPenalty == pOtherNode->getMapPenalties().end())
                                        {
                                            if (m_bForSymuMaster)
                                            {
                                                pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                            }
                                            else
                                            {
                                                pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new VehicleTypePenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern), m_pNetwork, lstAllowedVehicleTypes);
                                            }
                                        }
                                        else if (iterPenalty->second.find(pDownstreamPattern) == iterPenalty->second.end())
                                        {
                                            if (m_bForSymuMaster)
                                            {
                                                iterPenalty->second[pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                            }
                                            else
                                            {
                                                iterPenalty->second[pDownstreamPattern] = new VehicleTypePenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern), m_pNetwork, lstAllowedVehicleTypes);
                                            }
                                        }
                                    }
                                }
                            }
                            else
                            {
                                assert(m_bForSymuMaster);

                                // Cas de la plaque jointive
                                PlaqueDrivingPattern * pUpstreamPlaquePattern = dynamic_cast<PlaqueDrivingPattern*>(pUpstreamPattern);
                                if (pUpstreamPlaquePattern)
                                {
                                    std::set<TypeVehicule*> lstAllowedVehicleTypes;
                                    if (iterZone->first->m_AreaHelper.isAccessibleFromArea(iterInternalNode->second, &pUpstreamPlaquePattern->getPlaque()->m_AreaHelper, m_bForSymuMaster, lstAllowedVehicleTypes))
                                    {
                                        // la penalitÃ© peut dÃ©jÃ  exister si on a traitÃ© l'autre zone avant :
                                        std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iterPenalty = pOtherNode->getMapPenalties().find(pUpstreamPattern);
                                        if (iterPenalty == pOtherNode->getMapPenalties().end())
                                        {
                                            pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                        }
                                        else if (iterPenalty->second.find(pDownstreamPattern) == iterPenalty->second.end())
                                        {
                                            iterPenalty->second[pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void SymuCoreDrivingGraphBuilder::BuildPatternsInPlaques(SymuCore::Graph * pDrivingLayer)
{
    Node * pOtherNode, *pNode;
    Link * pLink;
    std::map<CPlaque*, SymuCore::Node*, LessConnexionPtr<CPlaque> >::const_iterator iterPlaque;
    for (iterPlaque = m_mapPlaquesToNodes.begin(); iterPlaque != m_mapPlaquesToNodes.end(); ++iterPlaque)
    {
        pNode = iterPlaque->second;

        // Pour tous les Links aval reliÃ©s Ã  la plaque et dont le Node aval est gÃ©rÃ© par SymuVia :
        for (size_t iLink = 0; iLink < pNode->getDownstreamLinks().size(); iLink++)
        {
            pLink = pNode->getDownstreamLinks()[iLink];
            pOtherNode = pLink->getDownstreamNode();

            // Pas de lien intercouche Ã  ce stade en principe
            assert(pOtherNode->getParent() == pDrivingLayer);

            PlaqueDrivingPattern* pUpstreamPattern = new PlaqueDrivingPattern(pLink, iterPlaque->first, GetConnexionFromNode(pOtherNode, true), true);
            pLink->getListPatterns().push_back(pUpstreamPattern);
        }

        // Pour tous les Links amont reliÃ©s Ã  la plaque et dont le Node amont est gÃ©rÃ© par SymuVia :
        for (size_t iLink = 0; iLink < pNode->getUpstreamLinks().size(); iLink++)
        {
            pLink = pNode->getUpstreamLinks()[iLink];
            pOtherNode = pLink->getUpstreamNode();

            // Pas de liens intercouches Ã  ce stade en principe
            assert(pOtherNode->getParent() == pDrivingLayer);

            PlaqueDrivingPattern * pDownstreamPattern = new PlaqueDrivingPattern(pLink, iterPlaque->first, GetConnexionFromNode(pOtherNode, false), false);
            pLink->getListPatterns().push_back(pDownstreamPattern);
        }
    }
}

void SymuCoreDrivingGraphBuilder::BuildPenaltiesForPlaques(SymuCore::Graph * pDrivingLayer)
{
    Node * pOtherNode, *pNode;
    Link * pLink, *pDownstreamLink, *pUpstreamLink;
    std::map<CPlaque*, SymuCore::Node*, LessConnexionPtr<CPlaque>>::const_iterator iterPlaque;

    assert(m_bForSymuMaster);
    std::set<TypeVehicule*> lstAllowedVehicleTypes; // unused cause SymuMaster case.

    for (iterPlaque = m_mapPlaquesToNodes.begin(); iterPlaque != m_mapPlaquesToNodes.end(); ++iterPlaque)
    {
        pNode = iterPlaque->second;

        // Pour tous les Links aval reliÃ©s Ã  la plaque et dont le Node aval est gÃ©rÃ© par SymuVia :
        for (size_t iLink = 0; iLink < pNode->getDownstreamLinks().size(); iLink++)
        {
            pLink = pNode->getDownstreamLinks()[iLink];
            pOtherNode = pLink->getDownstreamNode();

            std::map<SymuCore::Node*, Connexion*>::const_iterator iterInternalNode = m_mapNodesToInternalConnexions.find(pOtherNode);
            if (iterInternalNode != m_mapNodesToInternalConnexions.end())
            {
                // Pas de lien intercouche Ã  ce stade en principe
                assert(pOtherNode->getParent() == pDrivingLayer);

                assert(pLink->getListPatterns().size() == 1);
                PlaqueDrivingPattern* pUpstreamPattern = (PlaqueDrivingPattern*)pLink->getListPatterns().front();

                // CrÃ©ation des Penalties entre ce pattern et les patterns aval
                for (size_t iDownstreamLink = 0; iDownstreamLink < pOtherNode->getDownstreamLinks().size(); iDownstreamLink++)
                {
                    pDownstreamLink = pOtherNode->getDownstreamLinks()[iDownstreamLink];

                    // Pas de lien intercouche Ã  ce stade en principe
                    assert(pDownstreamLink->getParent() == pDrivingLayer);

                    for (size_t iDownstreamPattern = 0; iDownstreamPattern < pDownstreamLink->getListPatterns().size(); iDownstreamPattern++)
                    {
                        Pattern * pDownstreamPattern = pDownstreamLink->getListPatterns()[iDownstreamPattern];

                        // si le tronÃ§on aval n'est pas accessible depuis le noeud (ce qui peut arriver si le rÃ©seau
                        // est "mal dÃ©fini" avec un tronÃ§on qui part d'un noeud mais sans mouvement autorisÃ©), il ne faut pas mettre de mouvement possible !!
                        TronconDrivingPattern * pTronconPattern = dynamic_cast<TronconDrivingPattern*>(pDownstreamPattern);
                        if (pTronconPattern)
                        {
                            if (iterPlaque->first->m_AreaHelper.isAcessibleToConnexion(iterInternalNode->second, pTronconPattern->GetTuyau(), m_bForSymuMaster, lstAllowedVehicleTypes))
                            {
                                // CrÃ©ation du switching cost nul entre les deux patterns :
                                pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                            }
                        }
                        else
                        {
                            // Cas de la zone jointive sur le point de jonction :
                            ZoneDrivingPattern * pDownstreamZonePattern = dynamic_cast<ZoneDrivingPattern*>(pDownstreamPattern);
                            if (pDownstreamZonePattern)
                            {
                                if (iterPlaque->first->m_AreaHelper.isAccessibleToArea(iterInternalNode->second, &pDownstreamZonePattern->getZone()->m_AreaHelper, m_bForSymuMaster, lstAllowedVehicleTypes))
                                {
                                    // la penalitÃ© peut dÃ©jÃ  exister si on a traitÃ© l'autre zone avant :
                                    std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iterPenalty = pOtherNode->getMapPenalties().find(pUpstreamPattern);
                                    if (iterPenalty == pOtherNode->getMapPenalties().end())
                                    {
                                        pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                    }
                                    else if (iterPenalty->second.find(pDownstreamPattern) == iterPenalty->second.end())
                                    {
                                        iterPenalty->second[pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                    }
                                }
                            }
                            else
                            {
                                // Cas de la plaque jointive
                                PlaqueDrivingPattern * pDownstreamPlaquePattern = dynamic_cast<PlaqueDrivingPattern*>(pDownstreamPattern);
                                if (pDownstreamPlaquePattern)
                                {
                                    if (iterPlaque->first->m_AreaHelper.isAccessibleToArea(iterInternalNode->second, &pDownstreamPlaquePattern->getPlaque()->m_AreaHelper, m_bForSymuMaster, lstAllowedVehicleTypes))
                                    {
                                        // la penalitÃ© peut dÃ©jÃ  exister si on a traitÃ© l'autre zone avant :
                                        std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iterPenalty = pOtherNode->getMapPenalties().find(pUpstreamPattern);
                                        if (iterPenalty == pOtherNode->getMapPenalties().end())
                                        {
                                            pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                        }
                                        else if (iterPenalty->second.find(pDownstreamPattern) == iterPenalty->second.end())
                                        {
                                            iterPenalty->second[pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Pour tous les Links amont reliÃ©s Ã  la plaque et dont le Node amont est gÃ©rÃ© par SymuVia :
        for (size_t iLink = 0; iLink < pNode->getUpstreamLinks().size(); iLink++)
        {
            pLink = pNode->getUpstreamLinks()[iLink];
            pOtherNode = pLink->getUpstreamNode();

            std::map<SymuCore::Node*, Connexion*>::const_iterator iterInternalNode = m_mapNodesToInternalConnexions.find(pOtherNode);
            if (iterInternalNode != m_mapNodesToInternalConnexions.end())
            {
                // Pas de liens intercouches Ã  ce stade en principe
                assert(pOtherNode->getParent() == pDrivingLayer);

                assert(pLink->getListPatterns().size() == 1);
                PlaqueDrivingPattern* pDownstreamPattern = (PlaqueDrivingPattern*)pLink->getListPatterns().front();

                // CrÃ©ation des Penalties entre ce pattern et les patterns aval
                for (size_t iUpstreamLink = 0; iUpstreamLink < pOtherNode->getUpstreamLinks().size(); iUpstreamLink++)
                {
                    pUpstreamLink = pOtherNode->getUpstreamLinks()[iUpstreamLink];

                    // Pas de lien intercouche Ã  ce stade en principe
                    assert(pUpstreamLink->getParent() == pDrivingLayer);

                    for (size_t iUpstreamPattern = 0; iUpstreamPattern < pUpstreamLink->getListPatterns().size(); iUpstreamPattern++)
                    {
                        Pattern * pUpstreamPattern = pUpstreamLink->getListPatterns()[iUpstreamPattern];

                        // si le tronÃ§on amont ne permet pas de rentrer dans le noeud (ce qui peut arriver si le rÃ©seau
                        // est "mal dÃ©fini" avec un tronÃ§on qui arrive vers un noeud mais sans mouvement autorisÃ©), il ne faut pas mettre de mouvement possible !!
                        TronconDrivingPattern * pTronconPattern = dynamic_cast<TronconDrivingPattern*>(pUpstreamPattern);
                        if (pTronconPattern)
                        {
                            if (iterPlaque->first->m_AreaHelper.isAcessibleFromConnexion(iterInternalNode->second, pTronconPattern->GetTuyau(), m_bForSymuMaster, lstAllowedVehicleTypes))
                            {
                                // CrÃ©ation du switching cost nul entre les deux patterns :
                                pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                            }
                        }
                        else
                        {
                            // Cas de la zone jointive sur le point de jonction :
                            ZoneDrivingPattern * pUpstreamZonePattern = dynamic_cast<ZoneDrivingPattern*>(pUpstreamPattern);
                            if (pUpstreamZonePattern)
                            {
                                if (iterPlaque->first->m_AreaHelper.isAccessibleFromArea(iterInternalNode->second, &pUpstreamZonePattern->getZone()->m_AreaHelper, m_bForSymuMaster, lstAllowedVehicleTypes))
                                {
                                    // la penalitÃ© peut dÃ©jÃ  exister si on a traitÃ© l'autre zone avant :
                                    std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iterPenalty = pOtherNode->getMapPenalties().find(pUpstreamPattern);
                                    if (iterPenalty == pOtherNode->getMapPenalties().end())
                                    {
                                        pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                    }
                                    else if (iterPenalty->second.find(pDownstreamPattern) == iterPenalty->second.end())
                                    {
                                        iterPenalty->second[pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                    }
                                }
                            }
                            else
                            {
                                // Cas de la plaque jointive
                                PlaqueDrivingPattern * pUpstreamPlaquePattern = dynamic_cast<PlaqueDrivingPattern*>(pUpstreamPattern);
                                if (pUpstreamPlaquePattern)
                                {
                                    if (iterPlaque->first->m_AreaHelper.isAccessibleFromArea(iterInternalNode->second, &pUpstreamPlaquePattern->getPlaque()->m_AreaHelper, m_bForSymuMaster, lstAllowedVehicleTypes))
                                    {
                                        // la penalitÃ© peut dÃ©jÃ  exister si on a traitÃ© l'autre zone avant :
                                        std::map<Pattern*, std::map<Pattern*, AbstractPenalty*, ComparePattern>, ComparePattern>::iterator iterPenalty = pOtherNode->getMapPenalties().find(pUpstreamPattern);
                                        if (iterPenalty == pOtherNode->getMapPenalties().end())
                                        {
                                            pOtherNode->getMapPenalties()[pUpstreamPattern][pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                        }
                                        else if (iterPenalty->second.find(pDownstreamPattern) == iterPenalty->second.end())
                                        {
                                            iterPenalty->second[pDownstreamPattern] = new NullPenalty(pNode, PatternsSwitch(pUpstreamPattern, pDownstreamPattern));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

Node * SymuCoreDrivingGraphBuilder::GetNodeFromConnexion(Connexion * pCon) const
{
    std::map<Connexion*, SymuCore::Node*>::const_iterator iter = m_mapInternalConnexionsToNode.find(pCon);
    if (iter != m_mapInternalConnexionsToNode.end())
    {
        return iter->second;
    }
    else
    {
        SymuViaTripNode * pTripNode = dynamic_cast<SymuViaTripNode*>(pCon);
        if (!pTripNode)
        {
            // C'est qu'il s'agit d'un parking
            pTripNode = m_pNetwork->GetParkingFromID(pCon->GetID());
        }
        return m_mapExternalConnexionsToNode.at(pTripNode);
    }
}

Connexion * SymuCoreDrivingGraphBuilder::GetConnexionFromNode(SymuCore::Node * pNode, bool bNodeInputConnexion) const
{
    std::map<SymuCore::Node*, Connexion*>::const_iterator iter = m_mapNodesToInternalConnexions.find(pNode);
    if (iter != m_mapNodesToInternalConnexions.end())
    {
        return iter->second;
    }
    else
    {
        SymuViaTripNode * pTripNode = m_mapNodesToExternalConnexions.at(pNode);
        Connexion * pConnexion = dynamic_cast<Connexion*>(pTripNode);
        if (pConnexion)
        {
            return pConnexion;
        }
        else
        {
            // C'est qu'il s'agit d'un parking
            Parking * pParking = (Parking*)pTripNode;
            return bNodeInputConnexion ? pParking->GetInputConnexion() : pParking->GetOutputConnexion();
        }
    }
}

const std::map<ZoneDeTerminaison*, SymuCore::Node*, LessConnexionPtr<ZoneDeTerminaison> > & SymuCoreDrivingGraphBuilder::GetMapZones() const
{
    return m_mapZonesToNode;
}

const std::map<CPlaque*, SymuCore::Node*, LessConnexionPtr<CPlaque> > & SymuCoreDrivingGraphBuilder::GetMapPlaques() const
{
    return m_mapPlaquesToNodes;
}

const std::map<SymuCore::Node*, SymuViaTripNode*> & SymuCoreDrivingGraphBuilder::GetMapNodesToExternalConnexions() const
{
    return m_mapNodesToExternalConnexions;
}

SymuCore::Link * SymuCoreDrivingGraphBuilder::GetLinkFromTuyau(Tuyau * pTuyau) const
{
    for (std::map<SymuCore::Link*, std::set<Tuyau*, LessPtr<Tuyau> >, CompareLink>::const_iterator iter = m_mapLinkToTuyaux.begin(); iter != m_mapLinkToTuyaux.end(); ++iter)
    {
        if (iter->second.find(pTuyau) != iter->second.end())
        {
            return iter->first;
        }
    }
    return NULL;
}
