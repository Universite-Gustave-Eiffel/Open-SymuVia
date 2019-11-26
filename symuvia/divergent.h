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
#ifndef divergentH
#define divergentH

#include "ConnectionPonctuel.h"
#include <string>

class Tuyau;

class Divergent :  public ConnectionPonctuel
{
private:

    // Variables caractéristiques du divergent

    Tuyau*          m_pTuyauAmont;              // Fictif pour un giratoire 

    // Variables de simulation
private:

public:
    // Constructeurs, destructeurs et assimilés
    virtual ~Divergent(); // Destructeur
    Divergent() ; // Constructeur par défaut
	Divergent(std::string strID,  Reseau *pReseau);

    // Fonctions relatives à la simulation des divergents    
    void            Init();
    void            Init(Tuyau *pTAm, Tuyau *pTAvP, Tuyau *pTAvS);

    void            CorrigeAcceleration(double dbInst);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif
