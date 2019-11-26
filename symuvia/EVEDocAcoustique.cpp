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
#include "EVEDocAcoustique.h"

#include "SystemUtil.h"

EVEDocAcoustique::EVEDocAcoustique(void) 
{

	m_pTrafState = NULL;
}

EVEDocAcoustique::~EVEDocAcoustique(void) 
{

	if (m_pTrafState != NULL)
	{
		delete m_pTrafState;
	}
}

// Obtention des informations du trafic
eveShared::TrafficState * EVEDocAcoustique::GetTrafficState() 
{

	return m_pTrafState;
}

// Fin
void EVEDocAcoustique::Terminate() 
{

}

// Ajout d'un instant
EVEDocAcoustique::PINSTANT EVEDocAcoustique::AddInstant(double dbInstant, int nNbVeh) 
{

	if (m_pTrafState != NULL)
	{
		delete m_pTrafState;
	}
	m_pTrafState = new eveShared::TrafficState();
	m_pTrafState->inst = dbInstant;
	return NULL;
}

// Sauvegarde des données acoustiques d'un instant
void EVEDocAcoustique::SaveLastInstant() 
{

}

// Ajout de la description d'une cellule acoustique
void EVEDocAcoustique::AddCellule(int nID, const std::string & strLibelle, const std::string & strTuyau, double dbXam, double dbYam, double dbZam, double dbXav, double dbYav, double dbZav) 
{

}

// Ajout des données acoustique d'une source ponctuelle à l'instant considéré
void EVEDocAcoustique::AddSourcePonctuelle(int nID, double dbAbsDeb, double dbOrdDeb, double dbHautDeb, double dbAbsFin, double dbOrdFin, double dbHautFin, double * arEmissions) 
{

	eveShared::Source * pSrc = new eveShared::Source();

	pSrc->ad = dbAbsDeb;
	pSrc->af = dbAbsFin;
	pSrc->id = nID;
	pSrc->od = dbOrdDeb;
	pSrc->of = dbOrdFin;
	for (int i=0; i<9; i++)
	{
		pSrc->values[i] = arEmissions[i];
	}
	pSrc->zd = dbHautDeb;
	pSrc->zf = dbHautFin;

	m_pTrafState->Sources.push_back(pSrc);
}

// Ajout des données acoustiques d'une cellule à l'instant considéré
void EVEDocAcoustique::AddCellSimu(int nID, double * arEmissions) 
{
//Normalement inutile
}

// Ajout des données acoustiques d'une cellule à l'instant considéré
void EVEDocAcoustique::AddCellSimuEx(int nID, double * arEmissions, const std::string & strTuyau, double dbXam, double dbYam, double dbZam, double dbXav, double dbYav, double dbZav) 
{

	eveShared::Cell * pCell = new eveShared::Cell();

	pCell->id = nID;
	pCell->tron = strTuyau;
	for (int i=0; i<9; i++)
	{
		pCell->values[i] = arEmissions[i];
	}
	pCell->Xam = dbXam;
	pCell->Xav = dbXav;
	pCell->Yam = dbYam;
	pCell->Yav = dbYav;
	pCell->Zam = dbZam;
	pCell->Zav = dbZav;

	m_pTrafState->Cells.push_back(pCell);
}

