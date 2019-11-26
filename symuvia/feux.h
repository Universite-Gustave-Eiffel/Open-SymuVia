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
#ifndef feuxH
#define feuxH

#include "tools.h"

//---------------------------------------------------------------------------
//***************************************************************************//
//                          structure CycleFeux                              //
//***************************************************************************//
struct CycleFeu
{
    //double      dbDuree;                         // Durée d'application du cycle

    double      dbCycle;                        // Durée du cycle
    double      dbOrange;                       // Durée de l'orange

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

// La classe feux contient toutes les informations correspondant au fonctionnement
// de circulation durant la simulation. Pour info, un feu est défini pour une voie
// d'un tronçon en amont d'un répartiteur

class Repartiteur;

class Feux
{
private:
    //std::deque<CycleFeu*>        m_LstCycles;                   
    Repartiteur*                m_pRepartiteur;                 // Répartiteur parent

    std::deque<TimeVariation<CycleFeu>>           *m_pLstCycle; // Liste des cycles

public:

    // Constructeurs, destructeurs et assimilés
    ~Feux(void) ; // destructeur
	Feux(Repartiteur *pRep) ; // constructeur par défaut      

    Repartiteur*    GetRepartiteur(){return m_pRepartiteur;};

    std::deque<TimeVariation<CycleFeu>>* GetLstCycle(){return m_pLstCycle;};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);

};

#endif
