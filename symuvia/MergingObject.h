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
#ifndef MergingObjectH
#define MergingObjectH

namespace boost {
    namespace serialization {
        class access;
    }
}

class Reseau;
class Tuyau;
class SensorsManager;
class Vehicule;
class PonctualSensor;

class MergingObject
{
public:
	Reseau*	m_pReseau;

private:
	Tuyau*	m_pTuyPrincipal;				// Tronçon principal
	Tuyau*	m_pTuyAval;						// Tronçon aval
	Tuyau*	m_pTuyAmontPrioritaire;			// Tronçon amont prioritaire
	Tuyau*	m_pTuyAmontnonPrioritaire;		// Tronçon amont non prioritaire

	int		m_nVoieReduite;					// Indice de la voie qui se réduit sur le tronçon principal

	double	m_dbAlpha;						// Alpha (équivalent au gama d'un convergent)

	SensorsManager*	    m_pGstCapteurs;     // Objet de gestion des capteurs
	PonctualSensor*     m_pCptAval;			// Capteur aval
	PonctualSensor*     m_pCptAmontP;		// Capteur amont prioritaire
	PonctualSensor*     m_pCptAmontNP;		// Capteur amont non prioritaire

	double	m_dbProbaliteInsertion;			// Probabilité d'insertion du pas de temps courant

public:
	MergingObject();
	MergingObject(Reseau *pReseau, Tuyau* pTP, int nVoieReduite, Tuyau* pTAv, Tuyau *pTAmP, Tuyau* pTAmNP, double dbAlpa);
	~MergingObject();

	void UpdateInfoCapteurs();						// MAJ des info calculées par les capteurs (à appeler à la fin du calcul d'un pas de temps)
	void ProbabilityCalculation(double dbInstant);	// Calcul de la probabilité d'insertion pour le pas de temps courant

	double	GetProbaliteInsertion(Vehicule * pVehicule);
	Tuyau*	GetTuyPrincipal(){return m_pTuyPrincipal;};

private:
    double  CalculDemandeOrigine(Vehicule * pVehicule);
    double  CalculDebitLateral(Vehicule * pVehicule, double dbDemandeOrigine);
    double  CalculConsigne(Vehicule * pVehicule, double dbDebitLateral);

    double CalculSi(Vehicule * pVehicule);
    double CalculLambai(Vehicule * pVehicule);
    void   CalculOffreDemandeDest(Vehicule * pVehicule, double & mu, double & lambda);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif