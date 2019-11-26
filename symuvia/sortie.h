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
#ifndef sortieH
#define sortieH

#include "ConnectionPonctuel.h"
#include "usage/SymuViaTripNode.h"


// Sortie est un classe fixe de connection. Il s'agit d'un élément de réseau qui
// se trouve en aval d'un tronçon et qui est définit par sa capacité à absorber
// le flux quittant le réseau simulé

class Sortie : public SymuViaTripNode, public ConnectionPonctuel
{
private:        

    // Définition des caractéristiques de la sortie	
    ListOfTimeVariation<tracked_double>         *m_pLTVCapacite;

public:

	// Constructeurs, destructeurs et assimilés
	virtual ~Sortie(void); // Destructeur
	Sortie(void); // constructeur par défaut
    Sortie(char*, Reseau *pReseau);

    ListOfTimeVariation<tracked_double>*    GetLstCapacites() { return m_pLTVCapacite; }

    std::map<std::string, double> & GetCapacitiesPerRoutes() { return m_mapCapacityPerRoute; }

    // Surchage de GetNextEnterInstant pour application de la restriction de capacité
    virtual double  GetNextEnterInstant(int nNbVoie, double  dbPrevInstSortie, double  dbInstant, double  dbPasDeTemps, const std::string & nextRouteID);

    // Arrivée du véhicule dans la sortie
    virtual void        VehiculeEnter(boost::shared_ptr<Vehicule> pVehicle, VoieMicro * pDestinationEnterLane, double dbInstDestinationReached, double dbInstant, double dbTimeStep, bool bForcedOutside = false);

protected:
    // Utilisé pour les restrictions de capacité par route imposées par SymuMaster pour l'hybridation avec SimuRes
    std::map<std::string, double> m_mapCapacityPerRoute;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#endif
