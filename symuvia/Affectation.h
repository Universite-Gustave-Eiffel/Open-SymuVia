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
#ifndef affectationH
#define affectationH

#include "AffectationSave.h"

class SimulationSnapshot;

class SymuViaTripNode;
class TraceDocTrafic;
class Reseau;
class TuyauMicro;
class Connexion;

// Classe d'implantation du module d'affectation
class Affectation
{
private:

	CPeriode m_Periode;							// Stocke les information d'une pÃ©riode de simulation avec les itÃ©rations

	// ParamÃ©trage du module d'affectation
	bool	m_bLastInstSimu;					// Indique si calcul d'affectataion du dernier instant de la simu

	int		m_nPeriodeAffectation;				// Coefficient multiplicateur du pas de temps dÃ©finissant la pÃ©riode d'affectation

	int		m_nMode;							// Mode d'affectation : 1 -> logit, 2 -> wardrop

	double	m_dbSeuilConvergence;				// Seuil de convergence

	int		m_nNbItMax;							// Nombre d'itÃ©ration max

	bool	m_bConvergent;						// Indique si le processus d'affectation est convergent (par une mÃ©thode itÃ©rative) ou au fil de l'eau
	int		m_nAffectEqui;						// Type d'Ã©quilibre voulu (1: sans, 2 : Ã©quilibre sans mÃ©moire, 3 : Ã©quilibre approchÃ© par la mÃ©thode des moyennes successives)

	char	m_cTypeTpsParcours;					// Type de calcul du temps de parcours : 'P' pour prÃ©dictif 1 (Nico), 'Q' pour prÃ©dictif 2 (Ludo) et 'E' pour estimÃ©

	char	m_cLieuAffectation;					// Type de lieu d'affectation	- 'E' : uniquement aux entrÃ©es (lors de la crÃ©ation du vÃ©hicule, son itinÃ©raire lui est affectÃ© et il le suit jusqu'Ã  la fin)
												//								- 'C' : Ã  toutes les connexions (l'itinÃ©arire du vÃ©hicule est calculÃ© Ã  chaque connexion en fonction des coefficients d'affecation de la connexion)

	double	m_dbVariationTempsDeParcours;		// Limite infÃ©rieure de la variation du temps de parcours pour laquelle un recalcul des coefficients d'affectation au niveau des connexions internes est effectuÃ©
	int		m_nNbNiveauAmont;					// Nombre de niveau du rÃ©seau d'affectation Ã  remonter pour recalcul des coefficients d'affectation suite Ã  une variation importante du temps de parcours

	double	m_dbTetaLogit;						// ParamÃ¨tre teta du modÃ¨le logit

	double	m_dbSeuilTempsParcours;				// Seuil maximal d'Ã©cart des temps de parcours rÃ©alisÃ©s et prÃ©dits pour considÃ©rer un tronÃ§on comme correctement affectÃ©

	bool	m_bDijkstra;						// Mode de calcul des k plus courts chemins

	int		m_nNbPlusCourtCheminCalcule;		// Nombre de plus court chemin Ã  calculer

	int		m_nIteration;

    double  m_dbStartPeriodInstant;

    DOMLSSerializerSymu*		m_XmlWriterAffectation;		// XmlWriter pour les affectations

    bool m_bReroutageVehiculesExistants;
    double m_dbDstMinAvantReroutage;
    double m_dbWardropTolerance;

    // Map de stockage de la longueur des itinÃ©raires (pour ne pas les recalculer Ã  chaque fois (non sÃ©rialisÃ©e car pas nÃ©cessaire)
    std::map<std::vector<Tuyau*>, double> m_mapItineraryLengths;

    // Etats de sauvegarde de la simulation au dÃ©but de la pÃ©riode d'affectation (pour affectation dynamique convergente)
    std::vector<SimulationSnapshot*> m_SimulationSnapshots;

public:

	Affectation();
	Affectation(int nPeriodeAffectation, int nAffectEqui, double dbVariationTempsDeParcours, int nNbNiveauAmont, 
				double dbTetaLogit, char cTypeTpsParcours, int nMode, double dbSeuilConvergence, int nNbItMax, bool bDijkstra,
				int nNbPlusCourtCheminCalcule, double dbSeuilTempsParcours, bool bReroutageVehiculesExistants, double dbDstMinAvantReroutage,
                double dbWardropTolerance);
    ~Affectation();

	bool	Run(Reseau *pReseau, double dbInstant, char cTypeEvent, bool bForceGraphReset, TypeVehicule * pTVA = NULL, SymuViaTripNode * pDestinationA = NULL, SymuViaTripNode * pOrigineA = NULL);	

    void    TakeSimulationSnapshot(Reseau *pReseau);
    void    SimulationCommit(Reseau *pReseau);
    void    SimulationRollback(Reseau *pReseau, size_t snapshotIdx);
    
    void    CalculTempsDeParcours(Reseau * pReseau, double dbInstant, char cTypeEvent, double dbPeriode);
    void    FinCalculTempsDeParcours(Reseau * pReseau, double dbInstant, char cTypeEvent, double dbPeriode);

	bool	IsConvergent() const{return m_bConvergent;};
	char	GetLieuAffectation(){return m_cLieuAffectation;};

	int		GetMode(){return m_nMode;};
	double	GetSeuilConvergence(){return m_dbSeuilConvergence;};
	int		GetNbItMax(){return m_nNbItMax;};
	
	// Gestion de la sauvegarde des pÃ©riodes d'affectation (public)
	void	InitSaveAffectation(bool bSave, Reseau *pReseau);		// Initialisation de la sauvegarde des affectations
	void	CloseSaveAffectation();									// Fermeture du fichier de sauvegarde des affectations
	void	IncreaseNBVehEmis(TypeVehicule*pVeh, const std::string & sOrig, const std::string & sDest); // Augmentation du nombre de type de vehicule Ã©mis
	void	IncreaseNBVehRecus(TypeVehicule*pVeh, const std::string & sOrig, const std::string & sDest); // Augmentation du nombre de type de vehicule reÃ§us
	void	AddRealiseItineraire(TypeVehicule * pTVeh, const std::string & sOrig, const std::string  & sDest, const std::vector<std::string> & listIti, const std::vector<Tuyau*> & tuyaux,
                                 bool bPredefini, int vehID);       // Ajout d'un itinÃ©raire rÃ©alisÃ©
	bool	IsSaving() { return (m_XmlWriterAffectation != NULL); } // Renvoie true si la sauvegarde de l'affectation est programmÃ©e
	// Fin gestion de la sauvegarde des pÃ©riodes d'affectation (public)

    TraceDocTrafic * GetCurrentDocTraficTmp();

private:

	// Gestion des la sauvegarde des pÃ©riodes d'affectation (private)
	void	SaveAffectationSimulation(Reseau *pReseau); // Sauvegarde du noeud "SIMULATION" de la sauvegarde des affectations

	void	GetCnxAmont( TuyauMicro* pT, int &nNiveau, std::set<Connexion*> &dqCnx);
	double	GetProportionCritereOK(Reseau *pReseau, TypeVehicule *pTV);

    double  GetItineraryLength(const std::vector<Tuyau*> & itinerary);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);
};

#endif