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
#include "SymuCoreGraphBuilder.h"

#include "reseau.h"

#include <Graph/Model/Node.h>
#include <Graph/Model/Link.h>
#include <Graph/Model/Pattern.h>
#include <Graph/Model/AbstractPenalty.h>
#include <Graph/Model/MultiLayersGraph.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>

using namespace SymuCore;

SymuCoreGraphBuilder::SymuCoreGraphBuilder()
: m_pNetwork(NULL),
m_pGraph(NULL),
m_bForSymuMaster(true),
m_DrivingGraphBuilder(NULL, true),
m_PublicTransportGraphBuilder(NULL)
{
	m_DrivingGraphBuilder.SetParent(this);
	m_PublicTransportGraphBuilder.SetParent(this);
}

SymuCoreGraphBuilder::SymuCoreGraphBuilder(Reseau * pNetwork, bool bForSymuMaster)
: m_pNetwork(pNetwork),
m_pGraph(NULL),
m_bForSymuMaster(bForSymuMaster),
m_DrivingGraphBuilder(pNetwork, bForSymuMaster),
m_PublicTransportGraphBuilder(pNetwork)
{
	m_DrivingGraphBuilder.SetParent(this);
	m_PublicTransportGraphBuilder.SetParent(this);
}


SymuCoreGraphBuilder::~SymuCoreGraphBuilder()
{
}

const SymuCoreDrivingGraphBuilder & SymuCoreGraphBuilder::GetDrivingGraphBuilder() const
{
    return m_DrivingGraphBuilder;
}

const SymuCorePublicTransportGraphBuilder & SymuCoreGraphBuilder::GetPublicTransportGraphBuilder() const
{
    return m_PublicTransportGraphBuilder;
}

SymuCore::MultiLayersGraph * SymuCoreGraphBuilder::GetGraph() const
{
    return m_pGraph;
}

bool SymuCoreGraphBuilder::Build(SymuCore::MultiLayersGraph * pGraph, bool bIsPrimaryGraph)
{
    std::vector<SymuCore::Node*> lstOrigins, lstDestination;
    bool bOk = m_DrivingGraphBuilder.CreateDrivingLayer(pGraph, lstOrigins, lstDestination);

    if (bOk && (m_bForSymuMaster || m_pNetwork->IsWithPublicTransportGraph()))
    {
        bOk = m_PublicTransportGraphBuilder.CreatePublicTransportLayer(pGraph, &m_DrivingGraphBuilder);
    }

    // remarque : construction un peu asymétrique entre couche conduite et couche TEC : la couche Conduite
    // gère les origines/destinations et construit les noeuds Zone/plaque alors qu'il s'agit de noeuds utilisés aussi en mode TEC.
    // Pourrait être gênant si on veut désactiver la couche driving, l'autre ne pourrait pas fonctionner sans...
    if (bOk && bIsPrimaryGraph)
    {
        for (size_t iOrigin = 0; iOrigin < lstOrigins.size(); iOrigin++)
        {
            pGraph->CreateLinkToOrigin(lstOrigins[iOrigin]);
            if (m_bForSymuMaster)
            {
                SymuCore::Node * pPublicTransportOrigin = m_PublicTransportGraphBuilder.GetOriginNodeFromNode(lstOrigins[iOrigin]);
                if (pPublicTransportOrigin)
                {
                    pGraph->CreateLinkToOrigin(pPublicTransportOrigin);
                }
            }
        }
        for (size_t iDestination = 0; iDestination < lstDestination.size(); iDestination++)
        {
            pGraph->CreateLinkToDestination(lstDestination[iDestination]);
            if (m_bForSymuMaster)
            {
                SymuCore::Node * pPublicTransportDestination = m_PublicTransportGraphBuilder.GetDestinationNodeFromNode(lstDestination[iDestination]);
                if (pPublicTransportDestination)
                {
                    pGraph->CreateLinkToDestination(pPublicTransportDestination);
                }
            }
        }
    }

    m_pGraph = pGraph;
    
    return bOk;
}






