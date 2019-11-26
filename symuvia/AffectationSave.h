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
#ifndef affectationSaveH
#define affectationSaveH

#include "TimeUtil.h"

namespace boost {
    namespace serialization {
        class access;
    }
}

#include <deque>
#include <vector>
#include <map>
#include <set>

class Vehicule;
class TypeVehicule;
class Tuyau;
class DOMLSSerializerSymu;
class SymuCoreManager;

//////////////////////////////////////////////////////////////////////////////////////////////////
// Classes aidant Ã  la sauvegarde des affectations
//////////////////////////////////////////////////////////////////////////////////////////////////

// Structure pour la sauvegarde des itinÃ©raires prÃ©dits et rÃ©alisÃ©s
class CItineraire
{
public:
	CItineraire()
	{
		coeffs = 0.0;
		tpsparcours = 0.0;
        distance = 0.0;
        predefini = false;
	}
public:
	std::vector<Tuyau*>		 tuyaux;		    // liste des objets Tuyau constituant l'itinÃ©raire
	std::vector<std::string> elements;			// liste des noms des Tuyaux et des connexions constituant l'itinÃ©raire
    std::set<int>           vehicleIDs;         // Ensemble des identifiants de vÃ©hicules ayant Ã©tÃ© affectÃ©s Ã  cet itinÃ©raire
	double coeffs;								// coeficient d'affectation de l'itinÃ©raire
	double tpsparcours;							// temps de parcours de l'itinÃ©raire de la pÃ©riode courante, prÃ©dit si itÃ©ration, rÃ©alisÃ© si pÃ©riode convergente
    double distance;                            // Longueur de l'itinÃ©raire
    bool predefini;                             // Flag indiquant si l'itinÃ©raire est prÃ©dÃ©fini ou calculÃ©
public:
	void write(DOMLSSerializerSymu * writer, bool bIsRealized);	// Ã©criture d'un itinÃ©raire dans le fichier de sauvegarde des affectations

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

// Structure pour la sauvegarde des itÃ©rations prÃ©dites
class CIteration
{
public:
	CIteration()
	{
		num = 0;
		temps_calcul = 0;
		indice_proximite = 0.0;		
	}
public :
	size_t num;								// numÃ©ro de l'itÃ©ration
	int temps_calcul;						// temps de calcul de l'itÃ©ration
	double indice_proximite;				// indice de proximitÃ© de la convergence
	std::deque<CItineraire> itineraires;	// liste des itinÃ©raires

	STime temps_debut_calcul;					// temps du dÃ©but du calcul
public:
	CItineraire * addItineraire(const std::vector<Tuyau*> & tuyaux, const std::vector<std::string> & elements, double coeffs, double tpsparcours,
                                bool bPredefini, double dbLength); // ajout d'un itinÃ©raire
	void write(DOMLSSerializerSymu * writer);	// Ã©criture d'un itinÃ©raire dans le fichier de sauvegarde des affectations

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

// Structure pour la sauvegarde des couples OD
class CCoupleOD
{
public:
	CCoupleOD()
	{
		nb_veh_emis = 0;
		nb_veh_recus = 0;
	}
	CCoupleOD(const std::string & origineA, const std::string & destinationA)
	{
		origine = origineA;
		destination = destinationA;
		nb_veh_emis = 0;
		nb_veh_recus = 0;
	}
public:
	std::string origine;					// nom de l'origine
	std::string destination;				// nom de la destination
	int nb_veh_emis;						// nombre de vÃ©hicules entrants
	int nb_veh_recus;						// nombre de vÃ©hicules sortants
	std::deque<CIteration> predit;			// liste des itÃ©rations des itinÃ©raires prÃ©dits
	std::deque<CItineraire> realise;		// liste des itinÃ©raires rÃ©alisÃ©s

public:
	CIteration * AddPreditIteration(size_t num);//, double indice_proximite = 0.0);	// Ajout d'in itinÃ©raire prÃ©dit

	CItineraire * AddRealiseItineraire(const std::vector<Tuyau*> & tuyaux, const std::vector<std::string> &, bool bPredefini, int vehID, double dbLength);   // Ajout d'un itinÃ©raire rÃ©alisÃ©

	void write(DOMLSSerializerSymu * writer);	// Ã©criture d'une itÃ©ration dans le fichier de sauvegarde des affectations

private:
	CItineraire * GetRealiseItineraire(const std::vector<std::string> &elements);	// Obtention d'un itinÃ©raire rÃ©alisÃ©

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

// Structure pour la sauvegarde d'une pÃ©riode
class CPeriode
{
public:
	CPeriode()
	{
		type.clear();
		debut = 0;
		fin = 0;
		temps_calcul = 0;
		nb_iteration = 0;
		indicateur_proximite = 1.0;
		seuil_tempsparcours = 1.0;
		temps_debut_calcul = STime();
	}
	~CPeriode()
	{
		free();
	}
public:
	std::string type;						// "periodique" ou "evenementiel" 
	double debut;							// instant du dÃ©but
	double fin;								// instant de fin
	int temps_calcul;						// durÃ©e du calcul de la pÃ©riode
	int nb_iteration;						// nombre d'itÃ©rations pour la convergence
	double indicateur_proximite;			// indicateur de proximitÃ©
	double seuil_tempsparcours;				// seuil des temps de parcours

											// liste de couples origine/destination par type de vÃ©hicule
											// la liste de couples est indexÃ©e sur la paire (origine, destination)
	std::map<TypeVehicule*, std::map<std::pair<std::string, std::string>, CCoupleOD>> periode_type_vehicule;
	STime temps_debut_calcul;				// Instant oÃ¹ les calculs ont commencÃ©.
public:
	void InitPeriode();							// Initialisation de la pÃ©riode
    void FinPeriode(SymuCoreManager * pSymuScript, const std::string & sType, int nb_iterationA, double indicateur_proximiteA, double finA, double seuil_tempsparcoursA, bool bForceGraphReset); // Traitements de fin de pÃ©riode
	CCoupleOD * getCoupleOD(TypeVehicule* pTVeh, const std::string & sOrig, const std::string & sDest); // Obtention d'un couple OD
	std::map<std::pair<std::string, std::string>, CCoupleOD> * addTypeVehicule(TypeVehicule* pTVeh); // Ajout d'un type de vÃ©hicule
	CCoupleOD * addCoupleOD(TypeVehicule* pTVeh, const std::string & sOrig, const std::string & sDest); // Ajout d'un couple O/D
	CCoupleOD * addCoupleOD(std::map<std::pair<std::string, std::string>, CCoupleOD> * pListCoupleOD, const std::string & sOrig, const std::string & sDest); // Ajout d'un couple O/D
	void SetLastPreditIterationIndProx(double dbIndProx);	// Positionnement de l'indice de proximitÃ© et du temps de calcul de la derniÃ¨re itÃ©ration
	void free()									// LibÃ©ration des donnÃ©es de la pÃ©riode
	{
		type.clear();
		debut = 0;
		fin = 0;
		temps_calcul = 0;
		nb_iteration = 0;
		indicateur_proximite = 1.0;
		seuil_tempsparcours = 1.0;
		periode_type_vehicule.clear();
		temps_debut_calcul = STime();
	}
	void	CleanPeriode(bool bClear);						// Nettoyage de la pÃ©riode avant de calculer une nouvelle pÃ©riode

	void write(DOMLSSerializerSymu * writer);	// Ecriture d'une pÃ©riode dans le fichier de sauvegarde des affectations

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif