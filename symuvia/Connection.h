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

ï»¿#pragma once
#ifndef connexionH
#define connexionH

#include "tools.h"
#include "MesoNode.h"

#include <string>   // pour std::string
#include <deque>
#include <map>
#include <vector>

#ifdef USE_SYMUCORE
namespace SymuCore {
    class MacroType;
}
#endif // USE_SYMUCORE

class Tuyau;
class Voie;
class ControleurDeFeux;
class Reseau;
class TypeVehicule;
class SymuViaTripNode;
class SousTypeVehicule;
class RegulationBrique;


// Structure de stockage des variables d'affectation de la connexion
struct AssignmentData
{
    AssignmentData() { bPredefini = false; strRouteId = ""; pJunction = NULL; dbCoeffPrev = 0; }

	std::vector<Tuyau*>	dqTuyaux;		// TronÃ§on aval dans le cas d'une connexion interne ou liste de tronÃ§ons dÃ©taillant l'itinÃ©raire dans le cas d'une entrÃ©e
	int					nNbVeh;			// Nombre de vÃ©hicule affectÃ© durant la pÃ©riode d'agrÃ©gation courante
	double				dbCout;			// Cout de l'itinÃ©raire
	double              dbCoutPenalise; // cout pÃ©nalisÃ© de l'itinÃ©raire
	double				dbCoeff;		// Coeff d'affectation de la pÃ©riode d'affectation courante
	double				dbCoeffPrev;	// Coeff d'affecation de la pÃ©riode d'affectation prÃ©cÃ©dente
    double              dbCommonalityFactor;    // Commonality factor calculÃ©
    bool                bPredefini;     // Indique s'il s'agit d'un itinÃ©raire prÃ©dÃ©fini ou calculÃ©
    std::string         strRouteId;     // Identifiant de la route si l'itineraire est prÃ©fini depuis une route
    Connexion *         pJunction;      // Noeud correspondant au point de jonction si l'itinÃ©raire est vide (cas origine destination en contact direct)
   

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


//***************************************************************************//
//                          structure RepartitionFlux                        //
//***************************************************************************//
struct RepartitionFlux
{
    int         nNbVoiesAmont;      // nb max de voies amont
    int         nNbVoiesAval;       // nb max de voies aval

    double**    pCoefficients;      // Liste des coefficients ordonnÃ©es de la rÃ©partion pour chaque destination
    int**       nNbVeh;             // Liste des nombres de vÃ©hicules qui ont Ã©tÃ© rÃ©parti pour chaque destination  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

class MouvementsSortie
{
public:
    MouvementsSortie();

    Reseau *m_pReseau;
	double m_dbTaux;
	std::string m_strLibelle;
	double m_dbLigneFeu;
    bool   m_bHasVitMax;
    double m_dbVitMax;
    std::map<RegulationBrique*, std::vector<TypeVehicule*> >  m_LstForbiddenVehicles;

    bool IsTypeExclu(TypeVehicule* pTV, SousTypeVehicule * pSousType);

    void AddTypeExclu(TypeVehicule* typeVehicule);
    void RemoveTypeExclu(TypeVehicule* typeVehicule);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


class Vehicule;
class Connexion : public CMesoNode
{
public:
	Connexion();
    virtual ~Connexion();
	Connexion(std::string strID, Reseau *pReseau);

protected:
	std::string         m_strID;							// Identifiant unique alpha-numÃ©rique
	int					m_nID;								// Identifiant unique de type entier

	ControleurDeFeux    *m_pCtrlDeFeux;						// Controleur de feux    

    Reseau*             m_pReseau;							// Pointeur sur le rÃ©seau

	int					m_nNbMaxVoies;                      // Nombre maximal de voies des tronÃ§ons amont et aval    

	int					m_nZLevel;							// Indique le niveau ("z-level") du tronÃ§on afin de hiÃ©rarchiser les tronÃ§ons se coupant dans le plan. 
															// Cette notion est utile uniquement dans le cadre de l'intÃ©gration de SymuVia au sein de la plateforme EVE.

	// ParamÃ¨tres gÃ©omÃ©triques de la connexion
    double Abscisse;
	double Ordonne; 

    std::map<std::pair<Tuyau*,Tuyau*>, std::map<TypeVehicule*, double> >  m_LstCoutsAVide;    // Liste des couts Ã  vide des mouvements de la connexion pour affectation

public:

	// Couture du rÃ©seau de voirie
	std::deque<Tuyau*> m_LstTuyAm;							// Liste des tuyaux amont
	std::deque<Tuyau*> m_LstTuyAv;							// Liste des tuyaux aval

	// Couture du rÃ©seau d'affectation
	std::deque<Tuyau*> m_LstTuyAssAm;							// Liste des tuyaux amont
	std::deque<Tuyau*> m_LstTuyAssAv;							// Liste des tuyaux aval

	std::map<Voie*, std::map<Tuyau*, std::map<int, boost::shared_ptr<MouvementsSortie> >, LessPtr<Tuyau> >, LessPtr<Voie> >	m_mapMvtAutorises;			// Stockage des mouvements autorisÃ©s	

    // Accesseur pour le membre ci-dessus ajoutÃ© pour accÃ¨s depuis python (problÃ¨me de gestion des maps avec un type pointeur pour clÃ©)
    std::map<int, boost::shared_ptr<MouvementsSortie> > & GetMovement(Voie * pVoie, Tuyau * pTuyau);

    bool HasReservedLaneForMove(Tuyau * pUpstreamLink, Tuyau * pDownstreamLink);

    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionFlux> >* > & GetLstRepartition(){ return m_mapLstRepartition; }

    int     GetNbMaxVoies(){ return m_nNbMaxVoies; } // retourne le nombre max de voies
    void    SetNbMaxVoies(int nNbMax){ m_nNbMaxVoies = nNbMax; }    //(Nombre maximum de voies)


    std::map<TypeVehicule*, std::deque<TimeVariation<RepartitionFlux> >* > m_mapLstRepartition;			// Stockage des rÃ©partitions de flux

	// Stockage des coefficients directionnels



	// Affectation
	typedef std::map<Tuyau*, double, LessPtr<Tuyau>> typeCoeffAffectation;
	
	typedef std::pair<Tuyau*, SymuViaTripNode*> typeCoupleEntreeDestination;
	typedef std::pair<double, std::map< typeCoupleEntreeDestination, typeCoeffAffectation>> typeVarianteTemporelleAffectation;
    typedef std::pair<PlageTemporelle*, std::map< typeCoupleEntreeDestination, typeCoeffAffectation>> typePlageTemporelleAffectation;
	
	std::map<TypeVehicule*, std::deque<typeVarianteTemporelleAffectation> > m_mapCoeffAffectation;						// Stockage des coefficients d'affectation de la pÃ©riode d'affectation courante
	std::map<TypeVehicule*, std::deque<typeVarianteTemporelleAffectation> > m_mapCoeffAffectationPrev;					// Stockage des coefficients d'affectation de la pÃ©riode d'affectation prÃ©cÃ©dente (utile pour la recherche d'Ã©quilibre par la mÃ©thode des moyennes successives)
	std::map<TypeVehicule*, std::deque<typePlageTemporelleAffectation> > m_mapCoeffAffectationPT;					    // Stockage des coefficients d'affectation de la pÃ©riode d'affectation courante par plages temporelles
    std::map<TypeVehicule*, std::map< typeCoupleEntreeDestination, typeCoeffAffectation> > m_mapCoutAffectation;		// Stockage des coÃ»ts d'affectations (affectation calculÃ©e uniquement)

	// Variable de stockage des donnÃ©es d'affectation
	std::map<TypeVehicule*, std::map< std::pair<Tuyau*, SymuViaTripNode*>, std::deque<AssignmentData> > >		m_mapAssignment;

public:
    std::string			GetID() const		{return m_strID;}
    int					GetNumID() const	{ return m_nID; }

    Reseau*				GetReseau(){return m_pReseau;}

    ControleurDeFeux    *GetCtrlDeFeux(){return m_pCtrlDeFeux;}
    virtual void        SetCtrlDeFeux(ControleurDeFeux *pCDF){m_pCtrlDeFeux = pCDF;}

    void  SetAbscisse(double dbVal){ Abscisse = dbVal; }
    void  SetOrdonnee(double dbVal){ Ordonne = dbVal; }

    double GetAbscisse() { return Abscisse; }
    double GetOrdonnee() { return Ordonne; }

	// Fonctions donnant l'Ã©tat des variables        
    int     GetNbElAval() const{return (int)m_LstTuyAv.size() ;}
    int     GetNbElAmont() const {return (int)m_LstTuyAm.size();}

    int     GetNbElAssAval() const {return (int)m_LstTuyAssAv.size() ;}
    int     GetNbElAssAmont() const {return (int)m_LstTuyAssAm.size();}

	int     GetNoEltAmont(Tuyau* pTuyau);           // Renvoie l'indice de l'Ã©lÃ©ment amont passÃ© en paramÃ¨tre
    int     GetNoEltAval(Tuyau* pTuyau);            // Renvoie l'indice de l'Ã©lÃ©ment aval passÃ© en paramÃ¨tre

	bool	IsTuyauAmont(Tuyau *pTAm);
	bool	IsTuyauAval(Tuyau *pTAv);

	bool	IsMouvementAutorise(Voie *pVAm, Tuyau *pTAv, TypeVehicule * pTV, SousTypeVehicule * pSousType);
	bool	IsMouvementAutorise(Tuyau *pTAm, Tuyau *pTAv, TypeVehicule * pTV, SousTypeVehicule * pSousType);
    bool	IsMouvementAutorise(Tuyau *pTAm, Voie *pVAv, TypeVehicule * pTV, SousTypeVehicule * pSousType);
    bool	IsMouvementAutorise(Voie *pVAm, Voie *pVAv, TypeVehicule * pTV, SousTypeVehicule * pSousType);

    int		GetZLevel(){return m_nZLevel;}
    void	SetZLevel(int nZLevel){m_nZLevel = nZLevel;}

	bool	GetCoeffsMouvementAutorise(Voie *pVAm, Tuyau *pTAv, std::map<int, boost::shared_ptr<MouvementsSortie> > &mapCoeffs, TypeVehicule * pTV, SousTypeVehicule * pSousType);
    void	GetCoeffsMouvementAutorise(Voie *pVAm, Tuyau *pTAv, std::map<int, boost::shared_ptr<MouvementsSortie> > &mapCoeffs);

	bool	GetCoeffsAffectation(double dbInst, TypeVehicule* pTypeVeh, Tuyau *pTAm, SymuViaTripNode *pDst, std::map<Tuyau*, double, LessPtr<Tuyau> > &mapCoeffs);

    double  GetRepartition(
                            TypeVehicule * pTypeVeh,
                            int nAmont,             // NumÃ©ro du tronÃ§on amont
                            int nVoieAmont,         // NumÃ©ro de la voie amont
                            int nAval,              // NumÃ©ro du tronÃ§on aval
                            int nVoieAval,          // NumÃ©ro de la voie val
                            double dbTime           // Instant oÃ¹ on veut connaÃ®tre la rÃ©partition
                         );

	void    AddEltAmont(Tuyau *pT);
    void    AddEltAval(Tuyau *pT);        

    void    AddEltAmontAss(Tuyau *pT){m_LstTuyAssAm.push_back(pT);}
    void    AddEltAvalAss(Tuyau *pT){m_LstTuyAssAv.push_back(pT);}

	// Chargement des donnÃ©es Ã  partir d'un noeud XML
	bool	LoadMouvementsAutorises	(XERCES_CPP_NAMESPACE::DOMNode *xmlNodeMvtsAutorises, Logger * pLogger, bool insertion = false, std::string strTAvPrincipal = "", int nbVoiesEch = 0);
	bool	LoadAffectation			(XERCES_CPP_NAMESPACE::DOMNode *xmlNodeAffectation, bool bAllType, Logger *pChargement);

	void	SetCoutAffectation		(TypeVehicule *pTV, Tuyau* pTAm, SymuViaTripNode *pDest, Tuyau* pTAv, double dbCout);
	void	UpdateAffectation		(TypeVehicule *pTV, double dbTetaLogit);
	void	UpdateAffectationMSA	(TypeVehicule *pTV, int nIteration);

	void	ClearMapAssignment();
	void	IncNbVehAff(TypeVehicule *pTV, Tuyau *pTAm, SymuViaTripNode * pDest, const std::vector<Tuyau*> & Ts);

	void	InitSimuTrafic();

	 void calculBary(void); // Calcul le barycentre du rÃ©partiteur
    
    // vÃ©rifie que le carrefour n'est pas en mode meso
    bool IsTotalyMesoNode() const;
    // from CMesoNode
    virtual bool IsAnOrigine() const ;
    virtual bool IsADestination() const;

    //! Retourne vrai si le noeud a au moin un troncon mesoscopique
    virtual bool HasMesoTroncon( ) const ;

    //! Calcule le cout d'un mouvement autorisÃ©
    virtual double ComputeCost(TypeVehicule* pTypeVeh, Tuyau* pTuyauAmont, Tuyau * pTuyauAval) {return 0;}
    virtual double ComputeEmptyCost(TypeVehicule* pTypeVeh, Tuyau* pTuyauAmont, Tuyau * pTuyauAval) { return 0; }

    virtual double GetAdditionalPenaltyForUpstreamLink(Tuyau* pTuyauAmont) { return 0; }

#ifdef USE_SYMUCORE
    virtual double ComputeCost(SymuCore::MacroType* pMacroType, Tuyau* pTuyauAmont, Tuyau * pTuyauAval) { return 0; }
    virtual double GetMarginal(SymuCore::MacroType* pMacroType, Tuyau* pTuyauAmont, Tuyau * pTuyauAval);
#endif // USE_SYMUCORE

private:
	bool	LoadAffectationEx		(XERCES_CPP_NAMESPACE::DOMNode *xmlNodeAffectation, TypeVehicule *pTV, Logger * pChargement);


public:

	std::string GetUnifiedID() { return m_strID; }
	

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif
