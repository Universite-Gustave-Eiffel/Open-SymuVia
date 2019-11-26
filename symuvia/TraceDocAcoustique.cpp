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
#include "TraceDocAcoustique.h"

#include "EVEDocAcoustique.h"

TraceDocAcoustique::TraceDocAcoustique(Reseau * pNetwork, XERCES_CPP_NAMESPACE::DOMDocument * pXMLDocument)
: XMLDocAcoustique(pNetwork, pXMLDocument)
{
	m_pDocEveDocAcoustique = new EVEDocAcoustique();
}

TraceDocAcoustique::~TraceDocAcoustique(void) 
{
	delete m_pDocEveDocAcoustique;
	m_pDocEveDocAcoustique = NULL;
}

// Obtention des informations du trafic
eveShared::TrafficState * TraceDocAcoustique::GetTrafficState()
{
	if (m_pDocEveDocAcoustique == NULL)
	{
		return NULL;
	}
	else
	{
		return m_pDocEveDocAcoustique->GetTrafficState();
	}
}

// Ajout d'un instant
TraceDocAcoustique::PINSTANT	TraceDocAcoustique::AddInstant(double dbInstant, int nNbVeh)
{
	TraceDocAcoustique::PINSTANT pInst = XMLDocAcoustique::AddInstant(dbInstant, 0 );
	if (m_pDocEveDocAcoustique != NULL)
	{
		m_pDocEveDocAcoustique->AddInstant(dbInstant,0);
	}
	return pInst;
}

// Ajout des données acoustique d'une source ponctuelle à l'instant considéré
void TraceDocAcoustique::AddSourcePonctuelle(int nID, double dbAbsDeb, double dbOrdDeb, double dbHautDeb, double dbAbsFin, double dbOrdFin, double dbHautFin, double * arEmissions)
{
	XMLDocAcoustique::AddSourcePonctuelle(nID, dbAbsDeb, dbOrdDeb, dbHautDeb, dbAbsFin, dbOrdFin, dbHautFin, arEmissions);
	if (m_pDocEveDocAcoustique != NULL)
	{
		m_pDocEveDocAcoustique->AddSourcePonctuelle(nID, dbAbsDeb, dbOrdDeb, dbHautDeb, dbAbsFin, dbOrdFin, dbHautFin, arEmissions);
	}
}

// Ajout des données acoustiques d'une cellule à l'instant considéré
void TraceDocAcoustique::AddCellSimuEx(int nID, double * arEmissions, const std::string & strTuyau, double dbXam, double dbYam, double dbZam, double dbXav,  double dbYav, double dbZav )
{
	XMLDocAcoustique::AddCellSimuEx(nID, arEmissions, strTuyau, dbXam, dbYam, dbZam, dbXav, dbYav, dbZav);
	if (m_pDocEveDocAcoustique != NULL)
	{
		m_pDocEveDocAcoustique->AddCellSimuEx(nID, arEmissions, strTuyau, dbXam, dbYam, dbZam, dbXav, dbYav, dbZav);
	}
}
