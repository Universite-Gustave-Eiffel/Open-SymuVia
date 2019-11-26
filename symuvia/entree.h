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
#ifndef entreeH
#define entreeH

#include "ConnectionPonctuel.h"
#include "usage/SymuViaTripNode.h"

class Tuyau;
class DocTrafic;

// Entree est une classe fille de connection. Il s'agit d'un des deux élements
// réseau qu'il est possible de trouver en amont d'un tronçon. Une entrée est
// responsable de l'injection des véhicules sur le réseau

class Entree : public SymuViaTripNode, public ConnectionPonctuel
{
		
public:

        // Constructeurs, destructeurs et assimilés
	    virtual ~Entree(void) ; // Destructeur
	    Entree(char*, Reseau *pR); 
        Entree() ; // Constructeurs par défaut

		void SortieTrafic(DocTrafic *pXMLDocTrafic);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};
#endif

