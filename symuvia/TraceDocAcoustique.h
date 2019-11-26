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
#ifndef _TRACEDOCACOUSTIQUE_H
#define _TRACEDOCACOUSTIQUE_H

#include "XMLDocAcoustique.h"

class EVEDocAcoustique;
namespace eveShared {
    class TrafficState;
};

class TraceDocAcoustique : public XMLDocAcoustique
{
public:
	TraceDocAcoustique(Reseau * pNetwork, XERCES_CPP_NAMESPACE::DOMDocument * pXMLDocument);

	virtual ~TraceDocAcoustique(void);

private:
	EVEDocAcoustique * m_pDocEveDocAcoustique;

public:
	// Obtention des informations du trafic
	eveShared::TrafficState * GetTrafficState();

    // Ajout d'un instant
	virtual PINSTANT	AddInstant(double dbInstant, int nNbVeh);

    // Ajout des données acoustique d'une source ponctuelle à l'instant considéré
	virtual void AddSourcePonctuelle(int nID, double dbAbsDeb, double dbOrdDeb, double dbHautDeb, double dbAbsFin, double dbOrdFin, double dbHautFin, double * arEmissions);

    // Ajout des données acoustiques d'une cellule à l'instant considéré
	virtual void AddCellSimuEx(int nID, double * arEmissions, const std::string & strTuyau, double dbXam, double dbYam, double dbZam, double dbXav,  double dbYav, double dbZav );

};
#endif
