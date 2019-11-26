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
#ifndef RepartitionTypeVehiculeH
#define RepartitionTypeVehiculeH

#include "tools.h"

class RepartitionTypeVehicule
{
public:

	RepartitionTypeVehicule();
	~RepartitionTypeVehicule();

    std::deque<TimeVariation<std::vector<std::vector<double> > > > & GetLstCoefficients();

    // Ajout des variations temporelles de répartition des types de véhicules
    void AddVariation(const std::vector<std::vector<double> > & coeffs, PlageTemporelle * pPlage);
    void AddVariation(const std::vector<std::vector<double> > & coeffs, double dbDuree);

    // Supprime toutes les répartitions définies
    void Clear();

    // Supprime les répartitions définies depuis l'instant passé en paramètres
    void ClearFrom(double dbInstant, double dbLag);

    // Effectue le tirage d'un nouveau type de véhicule
    TypeVehicule* CalculTypeNewVehicule(Reseau * pNetwork, double  dbInstant, int nVoie);

    // Indique s'il existe un coefficient non nul pour un type de véhicule donné entre les instants spécifiés
    bool HasVehicleType(Reseau * pNetwork, int indexVehicleType, double dbStartTime, double dbEndTime);

protected:

    // Récupération de la proportion d'un type de véhicule à un instant donné
    double GetRepartitionTypeVehicule(double dbInstant, double dbLag, int nTypeVehicule, int nVoie);

protected:

    // Coefficients par type de véhicule puis par voie
    std::deque<TimeVariation<std::vector<std::vector<double> > > > m_LstCoefficients;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // RepartitionTypeVehicule