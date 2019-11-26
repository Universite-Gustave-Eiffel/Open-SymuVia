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
#ifndef controleurDeFeuxH
#define controleurDeFeuxH
///////////////////////////////////////////////////////////
//  ControleurDeFeux.h
//  Implementation of the Class ControleurDeFeux
//  Created on:      23-oct.-2008 13:38:10
//  Original author: becarie
///////////////////////////////////////////////////////////

#include "tools.h"
#include "StaticElement.h"

#include <boost/shared_ptr.hpp>

class Tuyau;
class Trip;
class Reseau;
class Vehicule;

typedef enum { FEU_ROUGE = 0, FEU_VERT = 1, FEU_ORANGE = 2} eCouleurFeux;

/**
 * Classe de modÃ©lisation d'un contrÃ´leur de feux
 */
struct CoupleEntreeSortie
{
    Tuyau*  pEntree;
    Tuyau*  pSortie;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

// Structure de dÃ©fintion d'une Ligne GuidÃ©e Prioritaire du contrÃ´leur de feux
struct LGP
{
    Tuyau* pTCptAm;             // TronÃ§on du capteur en amont du controleur de feu
	double dbPosCptAm;          // Position du capteur en amont du controleur de feu

    Tuyau* pTCptAv;             // TronÃ§on du capteur en aval du controleur de feu
	double dbPosCptAv;          // Position du capteur en aval du controleur de feu

	Tuyau* pTEntreeZone;        // TronÃ§on de la zone d'entrÃ©e du contrÃ´leur
	double dbPosEntreeZone;     // Position de la zone d'entrÃ©e du contrÃ´leur
	
	bool   bPrioriteTotale;     // Indique si la prioritÃ© est totale (ou partagÃ©e)
    int    nSeqPartagee;        // Si prioritÃ© partagÃ©e, indique la sÃ©quence partagÃ©e
	
    std::deque<Trip*> LstLTG;   // Liste des lignes LTG (lignes de transport guidÃ©s) concernÃ©es

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

/**
 * Classe de modÃ©lisation d'un signal actif d'une sÃ©quence : il est associÃ© soit Ã  un couple d'entrÃ©e / sortie
   soit Ã  une ligne de VGP
 */
class SignalActif
{
public:
	SignalActif();
    SignalActif(Tuyau *pTE, Tuyau *pTS, double dbDureeVert, double dbDureeOrange, double dbRetardAllumage);

    virtual ~SignalActif(){}

private:    
    double      m_dbDureeRetardAllumage;        // durÃ©e du retard Ã  l'allumage
    double      m_dbDureeVert;                  // durÃ©e du vert    
	double		m_dbDureeOrange;				// durÃ©e de l'orange

    Tuyau*      m_pTuyEntree;                   // TronÃ§on d'entrÃ©e concernÃ©
    Tuyau*      m_pTuySortie;                   // TronÃ§on de sortie concernÃ©

public:
    double      GetActivationDelay(){return m_dbDureeRetardAllumage;}
    double      GetGreenDuration(){return m_dbDureeVert;}
    void        SetGreenDuration(double dbDureeVert){m_dbDureeVert = dbDureeVert;}
    double      GetOrangeDuration(){return m_dbDureeOrange;}
    void        SetOrangeDuration(double dbDureeOrange){m_dbDureeOrange = dbDureeOrange;}
    Tuyau*      GetInputLink(){return m_pTuyEntree;}
    Tuyau*      GetOutputLink(){return m_pTuySortie;}

	eCouleurFeux GetCouleurFeux(double dbInstant);
    double      GetRapVertOrange(double dbInstant, double dbPasTemps, bool bVertOuOrange);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

/**
 * Classe de modÃ©lisation d'une sÃ©quence
 */
class CDFSequence
{
public:
    CDFSequence(){m_nNum=0;}
    CDFSequence(double dbDureeTotale, int nNum);
    ~CDFSequence();

private:
    double      m_dbDureeTotale;                // durÃ©e totale de la sÃ©quence


    std::vector<SignalActif*>    m_LstSignActif;     // Liste des signaux actifs de la sÃ©quence
    int                         m_nNum;             // NumÃ©ro d'orde de la sÃ©quence dans le plan de feux associÃ©

public:
    std::vector<SignalActif*>    GetLstActiveSignals(){return m_LstSignActif;};

    double                  GetTotalLength(){return m_dbDureeTotale;};
    void                    SetTotalLength(double dbDuree){m_dbDureeTotale = dbDuree;};

    SignalActif*            GetSignalActif(Tuyau* pTuyEntree, Tuyau* pTuySortie);

    void                    AddActiveSignal(SignalActif *pS){m_LstSignActif.push_back(pS);};
    int                     GetNum(){return m_nNum;}
    void                    SetNum(int nNum){m_nNum = nNum;}

    double                  GetMinDuree(double dbDureeVertMin);

    double                  GetMinDureeVert();

    double                  GetInstMaxFeuxVerts();

    void                    Copy(CDFSequence *pSeqSrc);

    //! Retourne le moment du vert suivant dans la sÃ©quence en cours
    double GetInstantVertSuivant(double dInstant, Tuyau *pTuyauAmont, Tuyau * pTuyauAval );
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

/**
 * Classe de modÃ©lisation d'un plan de feux
 */
class PlanDeFeux
{
public:
    PlanDeFeux();
	PlanDeFeux(char *sID, STime tHrDeb);
    ~PlanDeFeux();

public:
	STime					m_tHrDeb;               // Heure de dÃ©but du plan    

private:    
    std::string             m_strID;            // Identifiant unique du plan
    double                  m_dbDureeCycle;         // DurÃ©e en seconde du cycle de feu du plan        

    std::vector<CDFSequence*>  m_LstSequence;          // Liste ORDONNEE des sÃ©quences du plan

public:
	STime					GetStartTime(){return m_tHrDeb;}

    double                  GetCycleLength(){return m_dbDureeCycle;}
    void                    SetCycleLength(double dbDuree){m_dbDureeCycle = dbDuree;}
    
    CDFSequence*               GetSequence( double dbInstCycle, double &dbTpsPrevSeqs,  double &dbTpsCurSeq );

    std::vector<CDFSequence*>  GetLstSequences(){return m_LstSequence;}

    CDFSequence*               GetNextSequence(CDFSequence *pSeq);

    void                    AddSequence(CDFSequence *pSeq){ m_LstSequence.push_back(pSeq); m_dbDureeCycle+= pSeq->GetTotalLength(); }

    double                  GetDureeCumSeq(int nNum);

    void                    RemoveAllSequences(){m_LstSequence.clear(); m_dbDureeCycle = 0;}

    std::string             GetID(){return m_strID;}

	void                    SetHrDeb(STime tHrDeb){m_tHrDeb = tHrDeb;}

    void                    CopyTo(PlanDeFeux * pTarget);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


class ControleurDeFeux : public StaticElement
{
public:
    ControleurDeFeux(){m_PDFActif=NULL;m_pReseau=NULL;m_pLstPlanDeFeux=NULL;m_pLstPlanDeFeuxInit=NULL;m_pSeqPrePriorite=NULL;m_pSeqRetour1=NULL;m_pSeqRetour2=NULL;};
    ControleurDeFeux(Reseau *pReseau, char *sID, double dbRougeDegagement, double dbVertMin);
    ~ControleurDeFeux();	 

private:
    std::string                              m_strID;                   // Identifiant unique du plan
    ListOfTimeVariationEx<PlanDeFeux>       *m_pLstPlanDeFeux;          // Liste des variantes temporelles des plans de feux
	ListOfTimeVariationEx<PlanDeFeux>       *m_pLstPlanDeFeuxInit;      // Liste des variantes temporelles des plans de feux
    Reseau                                  *m_pReseau;                 // Pointeur sur le rÃ©seau

    PlanDeFeux*                             m_PDFActif;                 // Plan de feux actif (si pilotage)

    double                                  m_dbDureeVertMin;           // durÃ©e minimum du vert (tous les signaux actifs doivent respecter ce temps de vert)
	double                                  m_dbDureeVertMinInit;       // durÃ©e minimum du vert (tous les signaux actifs doivent respecter ce temps de vert)
    double                                  m_dbDureeRougeDegagement;   // durÃ©e du rouge de dÃ©gagement (tous les signaux actifs doivent respecter ce rouge)
	double                                  m_dbDureeRougeDegagementInit;  // durÃ©e du rouge de dÃ©gagement (tous les signaux actifs doivent respecter ce rouge)

    std::vector<CoupleEntreeSortie>         m_LstCoupleEntreeSortie;    // Liste des couples entrÃ©e/sortie traitÃ© par le contrÃ´leur de feux

    char                                    m_cModeFct;                 // Mode de fonctionnement du contrÃ´leur (N : normal, O : prÃ©-prioritÃ©, P : prioritÃ©, Q : post-prioritÃ©)

    std::deque<LGP*>                        m_LstLGP;                   // Liste des lignes guidÃ©es prioritaires du contrÃ´leur

    std::deque<std::pair<boost::shared_ptr<Vehicule>,LGP*> > m_LstVGPs; // Liste des VGP en cours de traitement

    boost::shared_ptr<PlanDeFeux>           m_pPDFEx;                   // Plan de feu actif durant une phase de VGP prioritaire
    double                                  m_dbDebPDFEx;               // DÃ©but de l'application du plan de feux de la phase VGP prioritaire
    double                                  m_dbFinPDFEx;               // Fin de l'application du plan de feux de la phase VGP prioritaire (rteour au plan de feux normal)

    // Gestion du mode pre-priorite
    CDFSequence        *m_pSeqPrePriorite;                 // SÃ©quence Ã  appliquer
    CDFSequence        *m_pSeqRetour1;                     // SÃ©quence de retour 1 aprÃ¨s la sortie du CDF du VGP
    CDFSequence        *m_pSeqRetour2;                     // SÃ©quence de retour 2 aprÃ¨s la sortie du CDF du VGP
    int             m_nTypeGestionPrePriorite;          // Type de gestion ( 1: 'respect temps min', 2: 'sÃ©quence actuelle Ã©courtÃ©e', 3 : 'prolongement sÃ©quence actuelle')

    // Cache des durÃ©es d'attente moyennes estimÃ©es par tronÃ§on amont pour ne pas les recalculer sans arrÃªt (non sÃ©rialisÃ©e car pas besoin)
    std::map<Tuyau*, double> m_MeanWaitTimes;

    // MÃ©thodes privÃ©es
private:
    // DÃ©termine les tuyaux amont et aval du CAF controlÃ© par lequel va arriver le vÃ©hicule guidÃ© prioritaire passÃ© en paramÃ¨tre
    void GetTuyauAmAv(Vehicule * pBus, Tuyau ** pTuyAm, Tuyau ** pTuyAv);

    void UpdatePosition();

public:
    ListOfTimeVariationEx<PlanDeFeux>*  GetLstTrafficLightCycles(){return m_pLstPlanDeFeux;};
	ListOfTimeVariationEx<PlanDeFeux>*  GetLstPlanDeFeuxInit(){return m_pLstPlanDeFeuxInit;};

    void    AddCoupleEntreeSortie(Tuyau* pEntree, Tuyau* pSortie);

    std::vector<CoupleEntreeSortie>* GetLstCoupleEntreeSortie(){return &m_LstCoupleEntreeSortie;};

    std::string GetLabel(){return m_strID;};

	PlanDeFeux* GetTrafficLightCycle( double dbInstant, STimeSpan &tsTpsEcPlan );

    eCouleurFeux GetStatutCouleurFeux  (double dbInstant, Tuyau *pTE, Tuyau *pTS, bool *pbPremierInstCycle = NULL, double *pdbPropVertOrange = NULL, bool *pbPrioritaire = NULL);

    eCouleurFeux GetCouleurFeux (double dbInstant, Tuyau *pTE, Tuyau *pTS, bool *pbPremierInstCycle = NULL, double *pdbPropVertOrange = NULL, bool  *pbPrioritaire = NULL);    

    CDFSequence*   GetSequence     (double dbInstant, double &dbTpsPrevSeqs, double &dbTpsCurSeq, bool &bPrioritaire);

    void    Update(double dbInstant);

    void    AddLGP(LGP *plgp){m_LstLGP.push_back(plgp);}

    char    GetModeFct(){return m_cModeFct;}
    
    std::deque<LGP*>* GetLstLGP(){return &m_LstLGP;}

    void    SetDureeRougeDegagement(double dbDuree){m_dbDureeRougeDegagement = dbDuree;}
    void    SetDureeVertMin(double dbDuree){m_dbDureeVertMin = dbDuree;}

	double    GetDureeRougeDegagementInit(){return m_dbDureeRougeDegagementInit;}
	double    GetDureeVertMinInit(){return m_dbDureeVertMinInit;}

	PlanDeFeux* GetPlanDeFeuxFromID(const std::string & sID);

	int     SendSignalPlan(const std::string & SP);

	std::string GetUnifiedID() { return m_strID; }
    Reseau*    GetReseau() {return m_pReseau;}

    //! Obtient l'instant en second du dÃ©but du prochain feu vert
    double GetInstantVertSuivant(double dRefTime,  double dInstantMax, Tuyau *pTuyauAmont, Tuyau * pTuyauAval );

    //! Obtient l'instant du dÃ©but du vert courant
    double GetInstantDebutVert(double dRefTime, Tuyau *pTuyauAmont, Tuyau * pTuyauAval);

    //! Estime le temps d'attente moyen thÃ©orique pour un tronÃ§on amont donnÃ©, en ne tenant compte que des plans de feux dÃ©finis dans le scÃ©nario initial.
    double GetMeanWaitTime(Tuyau *pTuyauAmont);

    // Accesseurs pour sauvegarde de l'Ã©tat en cas d'affectation dynamique convergente
    void SetPDFActif(PlanDeFeux * pPDF) {m_PDFActif = pPDF;}
    PlanDeFeux * GetPDFActif() {return m_PDFActif;}
    void SetModeFonctionnement(char cModeFcnt) {m_cModeFct = cModeFcnt;}
    std::deque<std::pair<boost::shared_ptr<Vehicule>,LGP*> > & GetLstVGPs() {return m_LstVGPs;}
    boost::shared_ptr<PlanDeFeux> GetPDFEx() {return m_pPDFEx;}
    void SetPDFEx(boost::shared_ptr<PlanDeFeux> pPDF) {m_pPDFEx = pPDF;}
    double GetDebutPDFEx() {return m_dbDebPDFEx;}
    void SetDebutPDFEx(double dbDebut) {m_dbDebPDFEx = dbDebut;}
    double GetFinPDFEx() {return m_dbFinPDFEx;}
    void SetFinPDFEx(double dbFin) {m_dbFinPDFEx = dbFin;}
    CDFSequence * GetSeqPrePriorite() {return m_pSeqPrePriorite;}
    void SetSeqPrePriorite(CDFSequence * pSeq) {m_pSeqPrePriorite = pSeq;}
    CDFSequence * GetSeqRetour1() {return m_pSeqRetour1;}
    void SetSeqRetour1(CDFSequence * pSeq) {m_pSeqRetour1 = pSeq;}
    CDFSequence * GetSeqRetour2() {return m_pSeqRetour2;}
    void SetSeqRetour2(CDFSequence * pSeq) {m_pSeqRetour2 = pSeq;}
    int GetTypeGestionPrePriorite() {return m_nTypeGestionPrePriorite;}
    void SetTypeGestionPrePriorite(int nTypeGestionPrePriorite) {m_nTypeGestionPrePriorite = nTypeGestionPrePriorite;}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
    friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif

