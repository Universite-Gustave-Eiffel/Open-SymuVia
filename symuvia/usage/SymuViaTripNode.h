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
#ifndef SymuViaTripNodeH
#define SymuViaTripNodeH

#include "usage/TripNode.h"
#include "tools.h"
#include "MesoNode.h"

#include <boost/shared_ptr.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <deque>
#include <string>
#include <vector>
#include <map>

namespace boost {
    namespace serialization {
        class access;
    }
}

class Vehicule;
class Tuyau;
class Connexion;
class TypeVehicule;
struct AssignmentData;
class SymuViaTripNode;
class RepartitionTypeVehicule;
class CRepMotif;
class CMotifCoeff;
class CPlaque;

struct CreationVehicule
{
public:
    CreationVehicule() {dbInstantCreation = 0.0; pTypeVehicule = NULL; pDest = NULL; nVoie = 1;}
    double dbInstantCreation;
    TypeVehicule * pTypeVehicule;
    SymuViaTripNode * pDest;
    int nVoie;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


//***************************************************************************//
//                          class RepartitionEntree                      //
//***************************************************************************//
class RepartitionEntree
{
public:

    std::vector<double> pCoefficients;      // Liste des coefficients ordonnÃ©s de la rÃ©partion par voie    

	RepartitionEntree(){}
	~RepartitionEntree(){}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


//***************************************************************************//
//                          class VectDest                               //
//***************************************************************************//
// Permet de stocker le vecteur destination (liste des coefficients de rÃ©partition > 0 
// pour chaque destination)
class VectDest
{
public:
    SymuViaTripNode*    pDest;                                                  // Destination
    double          dbCoeff;                                                // Coefficient de rÃ©partition vers la destination
    bool            bHasTetaLogit;                                          // Indique si une valeur particuliÃ¨re du teta logit est prÃ©cisÃ©e
    double          dbTetaLogit;                                            // Valeur particuliÃ¨re du teta logit
    bool            bHasNbPlusCourtsChemins;                                // Indique si une valeur particuliÃ¨re du nombre de plus courts chemins Ã  calculer
    int             nNbPlusCourtsChemins;                                   // Valeur particuliÃ¨re du nombre de plus courts chemins Ã  calculer
    bool            bHasCustomCommonalityParameters;                        // Indique si des paramÃ¨tres particuliers sont dÃ©finis pour le calcul du commonality factor
    double          dbCommonalityAlpha;
    double          dbCommonalityBeta;
    double          dbCommonalityGamma;
    std::vector<std::pair<double, std::pair<std::pair<std::vector<Tuyau*>, Connexion*>, std::string> > > lstItineraires;    // Liste des itinÃ©raires et routes id prÃ©dÃ©finis pour cette destination

    std::vector<std::string>                            lstRouteNames;     // Liste des identifiants de routes correspondants aux itinÃ©raires prÃ©dÃ©finis
    double          dbRelicatCoeff;                                         // relicat de coefficient non affectÃ© Ã  un itinÃ©raire prÃ©dÃ©fini

	VectDest(){pDest=NULL;dbCoeff=0;bHasTetaLogit=false;dbTetaLogit=0;bHasNbPlusCourtsChemins=false;nNbPlusCourtsChemins=0;dbRelicatCoeff=1;bHasCustomCommonalityParameters=false;
               dbCommonalityAlpha=0;dbCommonalityBeta=0;dbCommonalityGamma=0;}
	~VectDest(){};

    bool operator==(const VectDest& rhs)const;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

//***************************************************************************//
//                          class SimMatOrigDest                         //
//***************************************************************************//
// Permet de stocker la matrice des origines destination pour une durÃ©e donnÃ©e 
class SimMatOrigDest
{    
public:
    std::deque<VectDest*>       MatOrigDest;                 // Tableau des listes des vecteurs destination   

	SimMatOrigDest(){};
	~SimMatOrigDest()
	{
		std::deque<VectDest*>::iterator itM;
		for( itM = MatOrigDest.begin(); itM != MatOrigDest.end(); itM++ )
		{
			if( (*itM) )
				delete (*itM);
			(*itM) = NULL;
		}
		MatOrigDest.clear();
	};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

class SymuViaTripNode : public TripNode
{
public:

    // ************************************
    // Constructeurs/Destructeur
    // ************************************
    SymuViaTripNode();
    SymuViaTripNode(const std::string & strID, Reseau * pNetwork);
    virtual ~SymuViaTripNode();

    // ************************************
    // Traitements publics
    // ************************************

    // Initialisation des variables
    virtual void Initialize(std::deque< TimeVariation<TraceDocTrafic> > & docTrafics);

    // CrÃ©ation des vÃ©hicules
    virtual void CreationVehicules(const double dbInst, std::vector<boost::shared_ptr<Vehicule>> & listInfoVeh);

    
    // Connexions associÃ©es, ou "Ã©quivalentes" dans le cas des zones de terminaison
    virtual Connexion* GetInputConnexion();
    virtual Connexion* GetOutputConnexion();

    // Retourne la liste des itinÃ©raires et leurs coefficients d'affectation pour le couple OD dÃ©fini et le type de vÃ©hicule
    bool GetItineraires(TypeVehicule *pTV, SymuViaTripNode *pDst, std::deque<AssignmentData> &dqAssData);	

    virtual std::map<TypeVehicule*, std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> > > & GetMapAssignment();

    bool	AddEltAssignment(TypeVehicule *pTV, Tuyau *pTAm, SymuViaTripNode* pS, const std::vector<Tuyau*> & Ts, double dbCout, double dbCoutPenalise, double dbCommonalityFactor, bool bPredefini, const std::string & strRouteId, Connexion * pJunction);


    // Calcule la destination
    SymuViaTripNode* GetDestination(double  dbTime, int nVoie, TypeVehicule * pTypeVeh);

    std::deque<SymuViaTripNode*>* GetLstDestinations(){return &m_LstDestinations;}
    void AddDestinations(SymuViaTripNode* pDst);
    
    void AddCreationVehicule(double dbInstant, TypeVehicule * pTV, SymuViaTripNode * pDest, int nVoie);
    
    // Gestion du log de la matrice OD de l'origine
    int                 GetLogMatriceODSize(){return (int)m_LogMatriceOD.size();};
    int                 GetLogMatriceOD(int i,int j);

    int                 GetMatriceODInitSize(bool bIsUsedODMatrix);
    void                InitLogMatriceOD(bool bIsUsedODMatrix);
    void                MAJLogMatriceOD(bool bIsUsedODMatrix, double dbInstCreation, SymuViaTripNode *pDest);
    void                DeleteLogMatriceOD();

    // Gestion de la crÃ©ation des vÃ©hicules
	virtual void	    UpdateCritCreationVeh(double dbInst, bool bDueToCreation);   // Mise Ã  jour du critÃ¨re de crÃ©ation des vÃ©hicules

    // CrÃ©ation arbitraire de vÃ©hicules
    boost::shared_ptr<Vehicule> GenVehicule(int nID, TypeVehicule *pTV, int nVoie, double dbInst, double dbInstCreat, SymuViaTripNode *pDst, std::vector<Tuyau*> * pIti, Connexion * pJunction, CPlaque * pPlaqueOrigin, CPlaque * pPlaqueDestination, bool bForceNonResidential); // CrÃ©ation effective d'un vÃ©hicule

    // Gestion de l'agressivitÃ©
    void				AddAgressivite(TypeVehicule *pTV, double dbVal){ m_mapTauxAgressivite.insert( std::pair<TypeVehicule*, double>(pTV, dbVal));};
	bool				IsAgressif(TypeVehicule *pTV);

    void				InsertionVehEnAttente(double dbInst);						// Module d'insertion sur le rÃ©seau des vÃ©hicules en attente
	bool				InsertionVehEnAttenteMeso(double dbInst, Vehicule * pVeh);	// Module d'insertion sur le rÃ©seau des vÃ©hicules en attente pour le mÃ©soscopique
    // retourne vrai si le vÃ©hicule Ã  Ã©tÃ© insÃ©rÃ©

    void				FinSimuTrafic();

    // Indique si on peut crÃ©er des vÃ©hicules Ã  cette origine (par dÃ©faut, on peut toujours)
    virtual bool        IsVehiculeDisponible() {return true;}

    // Peut-on crÃ©er un vÃ©hicule au vu de l'instant courant et du dernier instant d'insertion ?
    virtual bool        CanInsertVehicle(double dbInst, double dbLastInstantInsertion) {return true;}

    // Gestion la valeur de la repartition des voies pour l'origine considÃ©rÃ©e
    std::vector<double> GetRepVoieValue(double dbInst);

    // Renvoie la valeur de la demande pour l'origine considÃ©rÃ©e
    virtual double      GetDemandeValue(double dbInst, double & dbVariationEndTime);

    // teste si un vÃ©hicule est susceptible de rallier cette origine Ã  la destination passÃ©e en paramÃ¨tre
    bool                IsLinkedToDestination(SymuViaTripNode * pDest, TypeVehicule * pTV, double dbStartTime, double dbEndTime);

    void	            InitAssignmentPeriod(TypeVehicule *pTV);


    std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > >&      GetLstCoeffDest(){return m_LstCoeffDest;}
	std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > >&      GetLstCoeffDestInit(){return m_LstCoeffDestInit;}
    std::deque<TimeVariation<SimMatOrigDest>>&      GetLstCoeffDest(TypeVehicule * pTypeVeh){return m_LstCoeffDest[pTypeVeh];}
	std::deque<TimeVariation<SimMatOrigDest>>&      GetLstCoeffDestInit(TypeVehicule * pTypeVeh){return m_LstCoeffDestInit[pTypeVeh];}

    
    // AccÃ¨s aux listes des variantes
    ListOfTimeVariation<tracked_double>*    GetLstDemande(TypeVehicule * pTypeVeh);
	ListOfTimeVariation<tracked_double>*    GetLstDemandeInit(TypeVehicule *pTypeVeh);
    std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* > GetLstDemandeInit() const ;
    std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* > GetLstDemande() const ;

    std::deque<TimeVariation<RepartitionEntree>>*   GetLstRepVoie(TypeVehicule * pTypeVehicle);
	std::deque<TimeVariation<RepartitionEntree>>*   GetLstRepVoieInit(TypeVehicule * pTypeVehicle);
    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >  GetLstRepVoieInit() const;
    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >  GetLstRepVoie() const;

    std::deque<TimeVariation<CRepMotif>>&      GetLstRepMotif(TypeVehicule * pTypeVeh){ return m_lstRepMotif[pTypeVeh]; }
    std::deque<TimeVariation<CRepMotif>>&      GetLstRepMotifInit(TypeVehicule * pTypeVeh){ return m_lstRepMotifInit[pTypeVeh]; }

    void   SetLstCoeffDestInit(SymuViaTripNode * pDest, TypeVehicule * pTypeVeh, const std::vector<std::pair<double, std::pair<std::vector<Tuyau*>, Connexion*> > > & routes);

    void   SetTypeDemande(bool bDemande){m_bDemande = bDemande;}
    bool   IsTypeDemande(){return m_bDemande;}

    void   SetTypeDistribution(bool bDistri){m_bDistribution = bDistri;}
    bool   IsTypeDistribution(){return m_bDistribution;}  

    int     GetNbVariations(){return Nb_variations;}
    int     GetNbVehicule(int nVar){return m_dqNbVehicules[nVar];}

    RepartitionTypeVehicule* GetLstRepTypeVeh(){return m_pLstRepTypeVeh;}

    TypeVehicule*   CalculTypeNewVehicule(double dbInstant, int nVoie);
    int             CalculNumVoie(double dbInstant, Tuyau * pOutputLink = NULL);

    void CopyDemandeInitToDemande(std::deque<TypeVehicule *> typeVehicles); 
    void CopyCoeffDestInitToCoeffDest(std::deque<TypeVehicule *> typeVehicles);
    void CopyRepVoieInitToRepVoie(std::deque<TypeVehicule *> typeVehicles);
    void CopyRepMotifDestInitToRepMotifDest(std::deque<TypeVehicule *> typeVehicles);

    // Calcule le motif d'un dÃ©placement
    CMotifCoeff* GetMotif(double  dbTime, TypeVehicule * pTypeVeh, SymuViaTripNode * pDest);

    std::deque<CreationVehicule> & GetLstCreationsVehicule() {return m_LstCreationsVehicule;}

    void AddCreationFromStationnement(double dbInstantCreation);

    std::vector<double> & GetLstSortiesStationnement() { return m_lstSortiesStationnement; }

    int SetLstRepVoie(TypeVehicule* pTypeVeh, const std::vector<double> & coeffs);

    // ************************************
    // Champs publics
    // ************************************
public:

    // Variables de simulation de l'entrÃ©e
	std::map<Tuyau*, std::map<int, std::deque<boost::shared_ptr<Vehicule>>>>  m_mapVehEnAttente;			// Map de la liste des vÃ©hicules en attente par voie (numÃ©ro de voie, pointeur sur un objet vÃ©hicule)
	std::map<Tuyau*, std::map<int, int > >		 				              m_mapVehPret;				// Map du vÃ©hicule pret Ã  partir par voie (numÃ©ro de voie, identifiant du vÃ©hicule)
    double            			                                              m_dbCritCreationVeh;		// critÃ¨re de crÃ©ation des vÃ©hicule pour l'instant courant
																		// le critÃ¨re peut s'exprimer sous forme d'un instant Ã  atteindre ou d'un nombre (<=1) de vÃ©hicule 

    // ************************************
    // Champs privÃ©s
    // ************************************
protected:

    // Liste des variantes des rÃ©partitions des destinations
    std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > >   m_LstCoeffDest;
	std::map<TypeVehicule*, std::deque<TimeVariation<SimMatOrigDest> > >   m_LstCoeffDestInit;

    RepartitionTypeVehicule*        m_pLstRepTypeVeh;   // Liste des variantes des rÃ©partitions des types de vÃ©hicule Ã  la crÃ©ation (uniquement pour un flux global)

    std::deque<SymuViaTripNode*>    m_LstDestinations;        // Liste des destinations possibles pour cette 
    std::deque<int*>                m_LogMatriceOD;           // Permet de tracer les destinations des vÃ©hicules crÃ©Ã©s

    std::map<TypeVehicule*, double> m_mapTauxAgressivite;	// Map des taux d'agressivitÃ© par classe de vÃ©hicule

     // Liste des variantes des demandes        
    std::map<TypeVehicule*, ListOfTimeVariation<tracked_double> *>      m_LTVDemandeInit;		// Liste initiale des demandes
	std::map<TypeVehicule* , ListOfTimeVariation<tracked_double> *>     m_LTVDemande;			// Liste en cours des demandes

    // Liste des variantes des rÃ©partitions des vÃ©hicules par voie
    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >               m_lstRepVoie;            
	std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionEntree> >* >               m_lstRepVoieInit;

	int                 Nb_variations;          // Nb de variations de la demande durant la simulation
    std::deque<int>     m_dqNbVehicules;        // Liste du nombre de vÃ©hicules crÃ©Ã©s durant chaque variation

    bool                m_bDemande;             // Indique si les vÃ©hicules sont gÃ©nÃ©rÃ©s Ã  l'aide d'une demande ou par une liste de vÃ©hicule (demande par dÃ©faut)
    bool                m_bDistribution;        // Indique si les vÃ©hicules sont gÃ©nÃ©rÃ©s Ã  l'aide d'une distribution exponentielle

    double              m_dbLastInstCreation;   // Dernier instant de crÃ©ation d'un vÃ©hicule Ã  cette origine

    // Liste des crÃ©ation de vÃ©hicules Ã  effectuer (mode de crÃ©ation par liste de vÃ©hicule prÃ©dÃ©finie)
    std::deque<CreationVehicule>    m_LstCreationsVehicule;
    TypeVehicule*                   m_pTypeVehiculeACreer;
    SymuViaTripNode*                m_pDestinationVehiculeACreer;
    int                             m_nVoieACreer;

    // Liste des variantes des distributions de motifs
    std::map<TypeVehicule*, std::deque<TimeVariation<CRepMotif> > >               m_lstRepMotif;
    std::map<TypeVehicule*, std::deque<TimeVariation<CRepMotif> > >               m_lstRepMotifInit;

    // Liste des instants de sortie de stationnement de vÃ©hicule, dans le cas oÃ¹ on est dans le mode de gÃ©nÃ©ration
    // automatique d'une demande liÃ©e au stationnement non rÃ©sidentiel en zone.
    std::vector<double>                                                           m_lstSortiesStationnement;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

BOOST_SERIALIZATION_ASSUME_ABSTRACT(SymuViaTripNode) // la classe est pure virtual

#endif // SymuViaTripNodeH
