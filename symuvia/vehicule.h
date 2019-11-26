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
#ifndef vehiculeH
#define vehiculeH

#pragma warning(push)
#pragma warning(disable : 4251)

#include "SymubruitExports.h"

#include "DocTrafic.h"
#include "giratoire.h"


#include <boost/thread/mutex.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <set>

class Voie;
class VoieMicro;
class TuyauMicro;
class Loi_emission;
class Segment;
class SensorsManagers;
class CDiagrammeFondamental;
class Connexion;
class MouvementsSortie;
class TripNode;
class Trip;
class Parking;
class Arret;
class Reseau;
class RegulationBrique;
class Tuyau;
class AbstractCarFollowing;
class AbstractFleet;
class AbstractFleetParameters;
class ScheduleElement;
class AbstractSensor;
class SensorsManager;
class TankSensor;
#ifdef USE_SYMUCOM
class ITSStation;
#endif // USE_SYMUCOM




struct PlageAcceleration
{
    double  dbVitSup;       // Vitesse supÃ©rieure de la plage d'accÃ©lÃ©ration
    double  dbAcc;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
friend class boost::serialization::access;
template<class Archive>
void serialize(Archive & ar, const unsigned int version);
};

struct SrcAcoustique            // Structure de description d'une source acoustique d'un type de vÃ©hicule
{
    double      dbPosition;     // Position de la source (par rapport Ã  l'avant du vÃ©hicule)
    std::string strIDDataBase;  // Identifiant base de donnÃ©es de la source (colonne TYPE)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
friend class boost::serialization::access;
template<class Archive>
void serialize(Archive & ar, const unsigned int version);
};

class SYMUBRUIT_EXPORT TypeVehicule
{
public:
    // Constructeur
    TypeVehicule();

    // Destructeur
    ~TypeVehicule(){};

protected:
    std::string m_strLibelle;                   // LibellÃ©
    std::string m_strGMLLibelle;                // LibellÃ© pour la sortie GML

    bool        m_bVxDistr;                     // Indique si la vitesse libre obÃ©it Ã  une ditribution normale (sinon constant)
    double      m_dbVx;                         // Vitesse libre moyenne du type de vÃ©hicule
    double      m_dbVxDisp;                     // Dispersion de la vitesse libre dans le cas d'une ditribution normale
    double      m_dbVxMin;                      // Borne inf de la vitesse libre dans le cas d'une ditribution normale
    double      m_dbVxMax;                      // Borne sup de la vitesse libre dans le cas d'une ditribution normale

    bool        m_bWDistr;                       // Indique si w obÃ©it Ã  une ditribution normale (sinon constant)
    double      m_dbW;                          // w moyen du type de vÃ©hicule
    double      m_dbWDisp;                      // Dispersion de w dans le cas d'une ditribution normale (positif)
    double      m_dbWMin;                       // Borne inf de w dans le cas d'une ditribution normale
    double      m_dbWMax;                       // Borne sup de w dans le cas d'une ditribution normale

    bool        m_bInvKxDistr;                   // Indique si 1/Kx obÃ©it Ã  une ditribution normale (sinon constant)
    double      m_dbKx;                          // Concentration max moyenne du type de vÃ©hicule
    double      m_dbInvKxDisp;                   // Dispersion de 1/Kx dans le cas d'une ditribution normale (positif)
    double      m_dbKxMin;                       // Borne inf de Kx dans le cas d'une ditribution normale
    double      m_dbKxMax;                       // Borne sup de Kx dans le cas d'une ditribution normale


    double      m_dbEspacementArret;            // Espacement entre 2 vÃ©hicules Ã  l'arrÃªt (calculÃ© Ã  partir de l'arriÃ¨re du vÃ©hicule jusqu'Ã  l'avant du vÃ©hicule suivant = espace vide laissÃ© par les 2 vÃ©hicules Ã  l'arrÃªt)
    double      m_dbDeceleration ;              // CapacitÃ© de dÃ©cÃ©lÃ©ration
    double		m_dbDecelerationEcartType;		// Ecart type de la dÃ©cÃ©lÃ©ration

    double		m_dbValAgr;						// Valeur d'agressivitÃ© du type de vÃ©hicule (pour un vÃ©hicule agressif)
                                                // correpsond Ã  la limite infÃ©rieure du delta N du vÃ©hicule avant adaptation de sa vitesse par rapport Ã  son leader
                                                // 1 par dÃ©faut

    double		m_dbTauChgtVoie;				// Coefficient Tau du changement de voie
    double		m_dbPiRabattement;				// Valeur de Pi de rabattement des vÃ©hicules de ce type

    double      m_dbVitLaterale;                // Vitesse latÃ©rale (pour traversÃ©e de voies dans l'axe perpendiculaire au tronÃ§on)

    double      m_dbTauDepassement;             // Coefficient Tau du dÃ©passement sur tronÃ§on opposÃ©

    double      m_dbAgressiveProportion;        // Proportion de vÃ©hicules agressifs
    double      m_dbShyProportion;              // Proportion de vÃ©hicules timides
    double      m_dbAgressiveEtaT;              // Valeur de etaT pour les vÃ©hicules agressifs
    double      m_dbShyEtaT;                    // Valeur de etaT pour les vÃ©hicules timides
    double      m_epsylon0;                     // Valeur absolue de epsylon 0 pour les vÃ©hicules agressifs et timides
    double      m_epsylon1;                     // Valeur absolue de epsylon 1 pour les vÃ©hicules agressifs et timides

    std::deque<PlageAcceleration> m_LstPlagesAcceleration;     // Liste ordonnÃ©es des plages d'accÃ©lÃ©ration

    std::deque<SrcAcoustique>     m_LstSrcAcoustiques;          // Liste des sources acoustiques

    std::map<std::string, double>               m_ScriptedLawsCoeffs;   // Coefficients d'assignation des lois de poursuites scriptÃ©es
    std::map<std::string, boost::python::dict>  m_ScriptedLawsParams;   // ParamÃ¨tres des lois de poursuites scriptÃ©es

public:
    const std::string       GetLabel() const {return m_strLibelle;}
    void                    SetLibelle(std::string strLibelle){ m_strLibelle = strLibelle;}

    const std::string       GetGMLLabel() const {return m_strGMLLibelle;}
    void                    SetGMLLibelle(std::string strLibelle){ m_strGMLLibelle = strLibelle;}

    void                    SetEspacementArret(double dbEsp){m_dbEspacementArret = dbEsp;}

    void                    PushPlageAcc(double dbAcc, double dbVitSup);
    double                  GetAccMax(double dbVit);
    double                  GetEspacementArret(){return m_dbEspacementArret;}

    const std::deque<PlageAcceleration> & GetLstPlagesAcceleration() {return m_LstPlagesAcceleration;}

    double                  GetDeceleration(){return m_dbDeceleration;}
    void                    SetDeceleration(double dbDec){m_dbDeceleration = dbDec;}

    double                  GetDecelerationEcartType(){return m_dbDecelerationEcartType;}
    void                    SetDecelerationEcartType(double dbDecET){m_dbDecelerationEcartType = dbDecET;}

    void                            AddSrcAcoustiques(double dbPos, char *strIDDB);
    std::deque<SrcAcoustique>       GetLstSrcAcoustiques(){return m_LstSrcAcoustiques;}

    double                  GetVx(){return m_dbVx;}
    double                  GetKx(){return m_dbKx;}
    double                  GetW (){return m_dbW;}

    bool                    IsVxDistr(){return m_bVxDistr;}
    void                    SetVxDistr(bool bVxDistr){m_bVxDistr = bVxDistr;}

    double                  GetVxDisp(){return m_dbVxDisp;}
    void                    SetVxDisp(double dbVxDisp){m_dbVxDisp = dbVxDisp;}

    double                  GetVxMin(){return m_dbVxMin;}
    void                    SetVxMin(double dbVxMin){m_dbVxMin = dbVxMin;}

    double                  GetVxMax(){return m_dbVxMax;}
    void                    SetVxMax(double dbVxMax){m_dbVxMax = dbVxMax;}

    bool                    IsWDistr(){return m_bWDistr;}
    void                    SetWDistr(bool bWDistr){m_bWDistr = bWDistr;}

    double                  GetWDisp(){return m_dbWDisp;}
    void                    SetWDisp(double dbWDisp){m_dbWDisp = dbWDisp;}

    double                  GetWMin(){return m_dbWMin;}
    void                    SetWMin(double dbWMin){m_dbWMin = dbWMin;}

    double                  GetWMax(){return m_dbWMax;}
    void                    SetWMax(double dbWMax){m_dbWMax = dbWMax;}

    bool                    IsInvKxDistr(){return m_bInvKxDistr;}
    void                    SetInvKxDistr(bool bInvKxDistr){m_bInvKxDistr = bInvKxDistr;}

    double                  GetInvKxDisp(){return m_dbInvKxDisp;}
    void                    SetInvKxDisp(double dbInvKxDisp){m_dbInvKxDisp = dbInvKxDisp;}

    double                  GetKxMin(){return m_dbKxMin;}
    void                    SetKxMin(double dbKxMin){m_dbKxMin = dbKxMin;}

    double                  GetKxMax(){return m_dbKxMax;}
    void                    SetKxMax(double dbKxMax){m_dbKxMax = dbKxMax;}

    void                    SetVxKxW(double dbVx, double dbKx, double dbW){m_dbVx = dbVx; m_dbKx = dbKx; m_dbW = dbW;}

    double                  GetDebitMax(){return m_dbW * m_dbKx / ( (m_dbW / m_dbVx) - 1);}

    void					SetValAgr(double dbValAgr){m_dbValAgr = dbValAgr;}
    double					GetValAgr(){return m_dbValAgr;}

    double					GetTauChgtVoie(){return m_dbTauChgtVoie;}
    void					SetTauChgtVoie(double dbTau){m_dbTauChgtVoie = dbTau;}

    double					GetTauDepassement(){return m_dbTauDepassement;}
    void					SetTauDepassement(double dbTau){m_dbTauDepassement = dbTau;}

    double					GetPiRabattement(){return m_dbPiRabattement;}
    void					SetPiRabattement(double dbPi){m_dbPiRabattement = dbPi;}

    double					GetVitesseLaterale(){return m_dbVitLaterale;}
    void					SetVitesseLaterale(double dbVitLat){m_dbVitLaterale = dbVitLat;}

    double					CalculDeceleration(Reseau * pNetwork);

    std::string				GetUnifiedID() { return m_strLibelle;}

    std::map<std::string, double> & GetScriptedLawsCoeffs() {return m_ScriptedLawsCoeffs;}
    std::map<std::string, boost::python::dict> & GetScriptedLawsParams() {return m_ScriptedLawsParams;}

    double					GetAgressiveProportion(){return m_dbAgressiveProportion;}
    void					SetAgressiveProportion(double dbValue){m_dbAgressiveProportion = dbValue;}

    double					GetShyProportion(){return m_dbShyProportion;}
    void					SetShyProportion(double dbValue){m_dbShyProportion = dbValue;}

    double					GetAgressiveEtaT(){return m_dbAgressiveEtaT;}
    void					SetAgressiveEtaT(double dbValue){m_dbAgressiveEtaT = dbValue;}

    double					GetShyEtaT(){return m_dbShyEtaT;}
    void					SetShyEtaT(double dbValue){m_dbShyEtaT = dbValue;}

    double					GetEpsylon0(){return m_epsylon0;}
    void					SetEpsylon0(double dbValue){m_epsylon0 = dbValue;}

    double					GetEpsylon1(){return m_epsylon1;}
    void					SetEpsylon1(double dbValue){m_epsylon1 = dbValue;}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);
};

// pseudo type de vÃ©hicule qui permet de dÃ©finir un Ã©tat de repect de la consigne par brique de rÃ©gulation
class SousTypeVehicule
{
public:
    // Constructeur
    SousTypeVehicule(){}

    // Destructeur
    virtual ~SousTypeVehicule(){}

    // Teste si le sous type de vÃ©hicule respecte la brique passÃ©e en paramÃ¨tre (Effectue le tirage si nÃ©cessaire)
    bool checkRespect(RegulationBrique * pBrique);

private:

    // map dÃ©crivant le respect de la consigne des vÃ©hicuels affectÃ©s Ã  ce sous type
    std::map<RegulationBrique*, bool> m_mapRespectConsigne;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);
};

/*===========================================================================================*/
/* Classe de modÃ©lisation d'un vÃ©hicule                                                      */
/*===========================================================================================*/

class SYMUBRUIT_EXPORT Vehicule : public boost::enable_shared_from_this<Vehicule>
{

public:
        // Constructeur
        Vehicule    ();
        Vehicule    (const Vehicule & other);
        Vehicule    (Reseau *pReseau, int nID, double dbVitInit, TypeVehicule *pTV, double dbPasTemps);

        // Destructeur
        virtual     ~Vehicule   ();

protected:
    Reseau		*m_pReseau;

    // Variables intrinsÃ¨ques
    double      m_dbVitMax;             // vitesse max
    int         m_nNbPasTempsHist;      // Nombre de pas de temps historisÃ© pour les variables de simulation
                                        // il dÃ©pend des donnÃ©es du diagramme fondamental associÃ©
    TypeVehicule*   m_pTypeVehicule;    // Pointeur sur le type de vÃ©hicule

    CDiagrammeFondamental *m_pDF;               // Pointeur sur le diagramme fondamental

    // Variables de simulation
    double      *m_pPos;                // pointeur sur le tableau des positions du vÃ©hicule (position relative sur la voie courante)
    VoieMicro   **m_pVoie;              // pointeur sur le tableau de stockage des voies de positionnement
    Tuyau  **m_pTuyau;             // pointeur sur le tableau de stockage des tuyaux de positionnement
    double      *m_pVit;                // pointeur sur le tableau de stockage de la vitesse
    double      *m_pAcc;                // pointeur sur le tableau de stockage de l'accÃ©lÃ©ration

    double		m_dbVitRegTuyAct;		// Valeur de la vitesse rÃ©glementaire du tronÃ§on, voie et position courants

    int         m_nID;                  // Identifiant unique du vÃ©hicule au cours de la simulation

    VoieMicro   *m_pNextVoie;           // Voie suivante de positionnement du vÃ©hicule (par rapport Ã  la voie du dÃ©but du pas de temps courant)
    Tuyau  *m_pNextTuyau;          // Tuyau suivant du vÃ©hicule (par rapport au tuyau du dÃ©but du pas de temps courant), les tuyaux internes ne sont pas pris en compte

    VoieMicro   *m_pPrevVoie;           // PrÃ©cÃ©dente voie parcourue (autre que la voie courante). UtilisÃ©e lors du calcul acoustique

    bool        m_bDejaCalcule;
    bool		m_bDriven;				// Indique si le vÃ©hicule est pilotÃ© pour le pas de temps courant
    bool		m_bForcedDriven;    	// Indique si le vÃ©hicule est pilotÃ© en mode forcÃ© pour le pas de temps courant
    bool        m_bDriveSuccess;        // Indique si le pilotage respecte les lois de l'Ã©coulement ou non
    bool        m_bAttenteInsertion;    // Indique si le vÃ©hicule est en phase d'insertion Ã  la sortie d'un arrÃªt

    double      m_dbInstantCreation;    // Instant de crÃ©ation du vÃ©hicule
    double      m_dbInstantEntree;		// Instant d'entrÃ©e sur le rÃ©seau du vÃ©hicule
    double      m_dbInstantSortie;		// Instant de sortie du vÃ©hicule du rÃ©seau

    bool        m_bAccMaxCalc;          // Indique si la valeur est calculÃ©e ou constante

    double      m_dbResTirFoll;         // Variable permettant de stocker le rÃ©sultat du tirage alÃ©atoire d'un vÃ©hicule sur
                                        // un lien en amont d'un convergent d'un giratoire et qui va sortir (calcul de la gÃ¨ne occasionÃ©e)
    double      m_dbResTirFollInt;      // Variable permettant de stocker le rÃ©sultat du tirage alÃ©atoire d'un vÃ©hicule sur
                                        // un lien en amont d'un convergent d'un giratoire (Ã  2 voies uniquement) sur la voie interne
                                        // permettant de savoir si il gÃ¨ne l'insertion d'un vÃ©hicule sur la voie externe

    bool        m_bOutside;             // Indique si le vÃ©hicule est en retrait de la voie (dans ce cas, il ne gÃ¨ne pas le traffic mais
                                        // il est qaund mÃªme pris en compte pour le calcul acoustique comme si il Ã©tait sur la voie)
                                        // ex: bus Ã  un arrÃªt dÃ©portÃ©

    bool        m_bArretAuFeu;          // Indique si le vÃ©hicule est arrÃªtÃ© car premier vÃ©hicule d'un tronÃ§on avec connexion aval avec feu rouge

    // Variables liÃ©es au changement de voie
    bool        m_bChgtVoie;            // Indique si le vÃ©hicule a changÃ© de voie pour le pas de temps considÃ©rÃ©

    std::deque<bool> m_bVoiesOK;        // Indique pour chaque voie du tronÃ§on si elle est possible

    double*     m_pWEmission;           // tableau Nb_Octaves+Niv_global) contenant les puissances acoustiques des valeurs d'emission acoustique du vÃ©hicule - en Watt

    double      m_dbHeureEntree;         // Heure d'entrÃ©e du vÃ©hicule sur le rÃ©seau ( = heure de crÃ©ation, mÃªme si le vÃ©hicule est en attente au niveau de l'entrÃ©e)

    int         m_nVoieDesiree;         // Permet de forcer le vÃ©hicule Ã  changer de voie si il ne se trouve
                                        // pas sur la bonne voie pour se positionner sur le tronÃ§on suivant

    // RÃ©gime
    bool        m_bRegimeFluide;        // Indique l'Ã©tat du rÃ©gime
    bool        m_bRegimeFluideLeader;  // Variante de m_bRegimeFluide qui n'est Ã  false que si le vÃ©hicule est en mode congestionnÃ© du fait d'un leader (ignore les feux
                                        // ou autres cas ou m_bRegimeFluide peut Ãªtre Ã  false sans leader)

    bool		m_bAgressif;			// Le vÃ©hicule a t'il un comportement agressif ?

    bool        m_bJorgeAgressif;
    bool        m_bJorgeShy;

    bool        m_bDeceleration;
    double      m_dbDeceleration;

    // Cumul d'Ã©mission de polluants atmosphÃ©riques
    double		m_dbCumCO2;
    double		m_dbCumNOx;
    double		m_dbCumPM;

    // Distance cumulÃ©e
    double		m_dbDstParcourue;

    // Gestion des ghosts suite Ã  un changement de voie
    int			m_nGhostRemain;			// Nombre de pas de temps restant du ghost
    boost::weak_ptr<Vehicule>	m_pGhostFollower;		// VÃ©hicule suivant dont le ghost est le leader
    boost::weak_ptr<Vehicule>	m_pGhostLeader;			// VÃ©hicule leader du ghost
    Voie*		m_pGhostVoie;			// Voie sur laquelle se trouve le ghost (voie d'origine du vÃ©hicule qui a changÃ© de voie)

    // Gestion des capteurs de changement de voie
    Voie*       m_pVoieOld;             // Voie d'origine d'un changement de voie rÃ©ussi

    std::vector<Tuyau*> m_TuyauxParcourus; // Liste des tuyaux dÃ©jÃ  empruntÃ©s par le vÃ©hicule

    // Gestion des sorties liÃ©es aux changements de voie
    bool        m_bTypeChgtVoie;        // true si le type du changement de voie est renseignÃ©
    TypeChgtVoie    m_TypeChgtVoie;     // Type de changement de voie
    bool        m_bVoieCible;           // true si le numÃ©ro de voie cible est connu
    int         m_nVoieCible;           // numÃ©ro de voie cible
    bool        m_bPi;                  // true si dbPi a Ã©tÃ© calculÃ©
    double      m_dbPi;                 // Pi calculÃ©
    bool        m_bPhi;                  // true si dbPhi a Ã©tÃ© calculÃ©
    double      m_dbPhi;                 // Phi calculÃ©
    bool        m_bRand;                  // true si dbRand a Ã©tÃ© calculÃ©
    double      m_dbRand;                 // dbRand calculÃ©

    // Gestion du tirage du choix de la voie d'insertion dans les giratoires
    std::vector<MouvementsInsertion> m_MouvementsInsertion;

    // VÃ©hicule que le vÃ©hicule est entrai nde dÃ©passer
    boost::shared_ptr<Vehicule>    m_VehiculeDepasse;

    SousTypeVehicule            m_SousType;                 // Sous-type de vÃ©hicule

    std::map<Voie*, double>     m_NextVoieTirage;   // structure de sauvegarde des tirages pour le choxi des voies suivantes

    AbstractCarFollowing       *m_pCarFollowing; // Loi de poursuite affectÃ©e au vÃ©hicule

    double                      m_dbCreationPosition; // Pour gÃ©rer correctement l'entrÃ©e d'un vÃ©hicule sur le rÃ©seau

    double                      m_dbGenerationPosition; // Position Ã  laquelle un vÃ©hicule est crÃ©Ã© en attente d'insertion sur le rÃ©seau

    // DÃ©finition du parcours du vÃ©hicule
    Trip*                       m_pTrip;

    TripNode*                   m_pLastTripNode;

    // DÃ©finition de la flotte associÃ©e au vÃ©hicule
    AbstractFleet*              m_pFleet;
    AbstractFleetParameters*    m_pFleetParameters;
    ScheduleElement*            m_pScheduleElement; // Element du calendrier qui a dÃ©clenchÃ© l'activation du vÃ©hicule, le cas Ã©chÃ©ant

    double              m_dbTpsArretRestant;    // Temps d'arrÃªt restant au TripNode destination actuel avant d'embrayer sur le TripLeg suivant
    double              m_dbInstInsertion;      // Instant d'insertion du vÃ©hicule dans le trafic
    int                 m_nVoieInsertion;       // NumÃ©ro de la voie d'insertion pour le vÃ©hicule (Ã  la sortie du TropNode en cours)

    bool                m_bEndCurrentLeg;       // Indique qu'on doit passer au leg suivant dans le FinCalculTrafic
    // Variables nÃ©cessaires pour l' appel Ã  OnCurrentLegFinished :
    VoieMicro *         m_pDestinationEnterLane;
    double              m_dbInstDestinationReached;
    double              m_dbDestinationTimeStep;

    std::set<TankSensor*> m_lstReservoirs;      // Pour garder trace des reservoirs dans lequel se trouve le vï¿½hicule afin de pouvoir mettre ï¿½ jour les rï¿½servoir lorsque le vï¿½hicule est dï¿½truit sans en ressortir

    int                 m_ExternalID;
    std::string         m_strNextRouteID;

    boost::mutex        m_Mutex;

#ifdef USE_SYMUCOM
    ITSStation*         m_pConnectedVehicle;
#endif // USE_SYMUCOM

private:
    // stocke le tuyau suivant d'un vehicule pour un instant et un tuyau courant donnÃ©e
    std::map< std::pair< double, Tuyau*>, Tuyau*, LessPairPtrIDRef<std::pair<double, Tuyau*> > > m_nextTuyaux;


public:

    std::deque<Voie*>   m_LstUsedLanes; // Liste des voies micro empruntÃ©es au cours du pas de temps courant (autre que la voie du dÃ©but et de la fin du pas de temps)

    std::deque<VoieMicro*>   m_LstVoiesSvt;  // Liste des voies suivantes

    // MÃ©thodes virtuelles
    virtual double      GetAccMax(double dbVit) ;
    virtual double      GetVitMax()      {return m_dbVitMax;}

    Reseau * GetReseau() {return m_pReseau;}

    double      GetPos  (int nDiffTemps);
    VoieMicro*  GetVoie  (int nDiffTemps);
    Tuyau*      GetLink  (int nDiffTemps);
    double      GetVit  (int nDiffTemps);
    double      GetAcc  (int nDiffTemps);
    int         GetID   (){return m_nID;}

    CDiagrammeFondamental*  GetDiagFonda(){return m_pDF;}
    void                    SetDiagFondaF(CDiagrammeFondamental *pDF){m_pDF = pDF;}

    void        SetTypeVeh(TypeVehicule *pTV){m_pTypeVehicule = pTV;}

    void        SetTrip(Trip * pTrip);

    double      GetLastPos            (){return m_pPos[m_nNbPasTempsHist-1];}
    double      GetBefLastPos         (){return m_pPos[m_nNbPasTempsHist-2];}
    double      GetLastVit          (){return m_pVit[m_nNbPasTempsHist-1];}
    double      GetBefLastVit       (){return m_pVit[m_nNbPasTempsHist-2];}

    int         GetNbPasTempsHist(){return m_nNbPasTempsHist;};

    void        SetPos    (double dbPos)        {m_pPos[0]  = dbPos;}
    void        SetPosAnt    (double dbPos)     {m_pPos[1]  = dbPos;}
    void        SetVoie   (VoieMicro* pVoie)    {m_pVoie[0] = pVoie;}
    void        SetVoieAnt(VoieMicro* pVoie);
    void        SetTuyau   (Tuyau* pT, double dbInst);
    void        SetTuyauAnt(TuyauMicro* pT);
    void        SetVit    (double dbVit)        {m_pVit[0]  = dbVit;}
    void        SetVitAnt (double dbVit)        {m_pVit[1]  = dbVit;}
    void        SetAcc    (double dbAcc)        {if (fabs(dbAcc)<0.000001) dbAcc=0; m_pAcc[0]  = dbAcc;}
    void        SetAccAnt (double dbAcc)        {if (fabs(dbAcc)<0.000001) dbAcc=0; m_pAcc[1]  = dbAcc;}

    void        SetInstantCreation(double dbInst){m_dbInstantCreation = dbInst;}
    double      GetInstantCreation(){return m_dbInstantCreation;}

    void        SetInstantEntree(double dbInst){m_dbInstantEntree = dbInst;}
    double      GetInstantEntree(){return m_dbInstantEntree;}

    void        SetInstantSortie(double dbInst){m_dbInstantSortie = dbInst;}
    double		GetExitInstant(){return m_dbInstantSortie;}

    void        SetNextTuyau(Tuyau* pT);

    const std::vector<Tuyau*> & GetTuyauxParcourus() {return m_TuyauxParcourus;}

    void        DecalVarTrafic      ();
    void        CalculTraficEx      (double dbInstant = 0);
    bool        CalculTraficFeux    (double dbInstant, double dbTimeStep, double dbStartPosOfLane, double dbStartPosOnLane, double dbEndPosOnLane, double dbGoalDistance,
                                        VoieMicro * pLane, size_t iLane, const std::vector<VoieMicro*> & lstLanes, double &dbMinimizedGoalDistance, bool &bArretAuFeu);

    void        FinCalculTrafic     (double dbInst);

    void        SortieTrafic        (DocTrafic	*pXMLDocTrafic, bool bChgtVoieDebug, double dInstant);

    void        InitChgtVoie();

    bool        CalculChangementVoie(int nVoieCible, double dbInstant, SensorsManagers* pGestionsCapteurs, bool bForce = false, bool bRabattement = false);

    bool        CalculDepassement(double dbInstant);

    bool        DepassementAutorise(double dbInstant, double dbPos, boost::shared_ptr<Vehicule> pVehLeader);

    double		CalculVitesseVoieAdjacente(int nVoieCible, double dbInst);

    void        CalculVoieDesiree(double dbInstant);

    bool        IsChgtVoie(){return m_bChgtVoie;}
virtual	void		SetChtgVoie(bool bVal){m_bChgtVoie = bVal;}

    void        SetPi(bool bPi) {m_bPi = bPi;}
    void        SetPi(double dbPi) {m_dbPi = dbPi;}
    void        SetPhi(bool bPhi) {m_bPhi = bPhi;}
    void        SetPhi(double dbPhi) {m_dbPhi = dbPhi;}
    void        SetRand(bool bRand) {m_bRand = bRand;}
    void        SetRand(double dbRand) {m_dbRand = dbRand;}

    TypeVehicule*        GetType()        {return m_pTypeVehicule;}
    SousTypeVehicule&    GetSousType()        {return m_SousType;}

    bool        IsDejaCalcule()  {return m_bDejaCalcule;}
    bool		IsDriven(){return m_bDriven;}
    bool		IsForcedDriven(){return m_bForcedDriven;}
    bool		IsDriveSuccess(){return m_bDriveSuccess;}
    void        ResetDriveState() {m_bDriven = false; m_bForcedDriven = false;}

    bool        IsRegimeFluide() {return m_bRegimeFluide;}
    void        SetRegimeFluide(bool bRegimeFluide) {m_bRegimeFluide = bRegimeFluide;}

    bool        IsPasse(Tuyau *pT, double dbPos);
    bool        IsDejaPasse(Tuyau *pT, double dbPos);

virtual Voie*       CalculNextVoie(Voie*    pVoie, double  dbInstant);
virtual void        CalculVoiesPossibles(double dbInstant);

    Tuyau*       CalculNextTuyau(Tuyau*    pTuyau, double  dbInstant);

    void        SortieEmission(
                                );
    void        CalculEmission(
                                Loi_emission  * Loi,
                                std::string     strType,
                                std::string     typ_rev
                                );

    Segment*    GetCellAcoustique(double dbPos);
    Segment*    GetCellSirane();


    double*     GetPuissanceAcoustique(){return m_pWEmission;}

    double      GetDstParcourueEx();

    Trip*       GetTrip();
    TripNode*   GetOrigine();
    TripNode*   GetDestination();

    // Passage Ã  l'Ã©tape suivante du Trip
    void        MoveToNextLeg();

    // Calcul la distance sÃ©parant le vÃ©hicule de la destination de son Ã©tape courante
    double      GetNextTripNodeDistance(int timeIdx, TripNode ** pTripNode);

    void        SetHeureEntree(double dbHeureEntree){m_dbHeureEntree=dbHeureEntree;}
    double      GetHeureEntree(){return m_dbHeureEntree;}

    std::vector<Tuyau*>*     GetItineraire();

    typedef bool (* itineraryPredicate)(Tuyau * pTuyau, void * pArg);
    int         GetItineraireIndex(itineraryPredicate predicate, void * pArg);
    int         GetItineraireIndex(const std::vector<Tuyau*> & itinerary, itineraryPredicate predicate, void * pArg);
    int         GetItineraireIndexThreadSafe(itineraryPredicate predicate, void * pArg, std::vector<Tuyau*> & itinerary);

    int         GetVoieDesiree(){return m_nVoieDesiree;}
    void        SetVoieDesiree(int nVoieDesiree) {m_nVoieDesiree = nVoieDesiree;}
    bool        IsVoieDesiree(){return m_nVoieDesiree != -1;}
    void        ReinitVoieDesiree() {m_nVoieDesiree = -1;}

    void        CalculXYZ(double &dbX, double &dbY, double &dbZ, bool bFin = true);

    boost::shared_ptr<Vehicule>    ChercheLeader(double dbInstant, bool bIgnoreVehiculesEnDepassement = false);

    AbstractCarFollowing * GetCarFollowing();

    void        SetDejaCalcule(bool bDejaCalcule){m_bDejaCalcule = bDejaCalcule;}

    void        SetCalcul(bool bVal){m_bDejaCalcule = bVal;};

    void        SetResTirFoll(double dbRes){ m_dbResTirFoll = dbRes;};
    double      GetResTirFoll(){return m_dbResTirFoll;};

    void        SetResTirFollInt(double dbRes){ m_dbResTirFollInt = dbRes;};
    double      GetResTirFollInt(){return m_dbResTirFollInt;};

    bool        IsItineraire(Tuyau *pT);

    double      CalculPosition(double dbtr, Voie* pVoie, double dbDistanceLimit);

    bool        IsOutside(){return m_bOutside;}
    void        SetOutside(bool bOutside) {m_bOutside = bOutside;}

    void        SetNextVoie(VoieMicro* pVoie){m_pNextVoie = pVoie;}

    VoieMicro*  GetNextVoie(){return m_pNextVoie;}

    void        CalculEmissionAtmos(double dbPasTemps, double &dbValCO2, double &dbValNOx, double &dbValPM);

    bool        IsVoiePossible(int nVoie){return m_bVoiesOK[nVoie];}
    Tuyau*      GetNextTuyau(){return m_pNextTuyau;}

    double      GetTempsDistance( double dbInst, double dbDst, double dbPasTemps, bool bDebutPasTemps) ;

    // Calcul du temps nÃ©cessaire pour atteindre la position indiquÃ©e en prenant en compte un arrÃªt
    double      CalculTempsAvecArret(double dbInstant, double dbPasTemps, int tIdx, Tuyau * pTuyau, double dbPos, TripNode *pArret);

    bool        IsArretAuFeu(){return m_bArretAuFeu;}

    // Gestion de l'agressivitÃ©
    bool		IsAgressif(){return m_bAgressif;}
    void		SetAgressif(bool bAgressif){m_bAgressif = bAgressif;}
    bool		IsJorgeAgressif(){return m_bJorgeAgressif;}
    void		SetJorgeAgressif(bool bAgressif){m_bJorgeAgressif = bAgressif;}
    bool		IsJorgeShy(){return m_bJorgeShy;}
    void		SetJorgeShy(bool bShy){m_bJorgeShy= bShy;}

    void        TirageJorge();

    void		CopyTo( boost::shared_ptr<Vehicule> pVehDst );

    void		SetID(int nID){m_nID = nID;}

    void		SetVitRegTuyAct( double dbVit) { m_dbVitRegTuyAct= dbVit;}
    double		GetVitRegTuyAct( ){return m_dbVitRegTuyAct;}
    Connexion*	GetConnexionAval(Tuyau * pTuyau);

    double      GetLength();


    std::string GetUnifiedID();

    void SetExternalID(int externalID) { m_ExternalID = externalID; }
    int GetExternalID() { return m_ExternalID; }

    void SetNextRouteID(const std::string & nextRouteID) { m_strNextRouteID = nextRouteID; }
    const std::string & GetNextRouteID() { return m_strNextRouteID; }

    // Polluants atmosphÃ©riques
    double	GetCumCO2(){return m_dbCumCO2;}
    double	GetCumNOx(){return m_dbCumNOx;}
    double	GetCumPM(){return m_dbCumPM;}

    double	GetDstCumulee(){return m_dbDstParcourue;}
    void	SetDstCumulee(double dbDstCumulee){m_dbDstParcourue = dbDstCumulee;}

    // Gestion des ghosts
    int			GetGhostRemain()	{return	m_nGhostRemain;}
    void        SetGhostRemain(int nGhostRemain) {m_nGhostRemain = nGhostRemain;}
    boost::shared_ptr<Vehicule>	GetGhostFollower()	{return m_pGhostFollower.lock();}
    void        SetGhostFollower(boost::shared_ptr<Vehicule> pGhostFollower) {m_pGhostFollower = pGhostFollower;}
    boost::shared_ptr<Vehicule>	GetGhostLeader()	{return m_pGhostLeader.lock();}
    void        SetGhostLeader(boost::shared_ptr<Vehicule> pGhostLeader) {m_pGhostLeader = pGhostLeader;}
    Voie*		GetGhostVoie()		{return m_pGhostVoie;}
    void        SetGhostVoie(Voie * pVoie) {m_pGhostVoie = pVoie;}

    void            SetInstInsertion(double dbInst){m_dbInstInsertion = dbInst;}
    double          GetInstInsertion(){return m_dbInstInsertion;}
    double          GetTpsArretRestant() {return m_dbTpsArretRestant;}
    void            SetTpsArretRestant(double dbTpsArretRestant) {m_dbTpsArretRestant = dbTpsArretRestant;}
    int             GetNumVoieInsertion() {return m_nVoieInsertion;}
    void            SetNumVoieInsertion(int nVoieInsertion) {m_nVoieInsertion = nVoieInsertion;}

    void            SetLastTripNode(TripNode * pLastTripNode) {m_pLastTripNode = pLastTripNode;}
    TripNode*       GetLastTripNode() {return m_pLastTripNode;}

    bool            IsAtTripNode();

    void            SetEndCurrentLeg(bool bValue) { m_bEndCurrentLeg = bValue; }

    // Gestion des portions de vitesse rÃ©glementaire
    void UpdateVitesseReg(double dbInst);

    std::vector<MouvementsInsertion> & GetMouvementsInsertion() {return m_MouvementsInsertion;};

    // Pilotage
    int			Drive(TuyauMicro *pT, int nVoie, double dbPos, bool bForce);
    int			Delete();
    int			AlterRoute(const std::vector<Tuyau*> & newIti);

    // Gestion de la traversÃ©e des sites propres
    bool TraverseeVoiesInterdites(double dbInstant, VoieMicro * pVoieOrigine, VoieMicro * pVoieCible);

    // Gestion du dÃ©passement
    boost::shared_ptr<Vehicule> GetVehiculeDepasse() {return m_VehiculeDepasse;}
    bool                        IsDepassement() {return m_VehiculeDepasse.get() != NULL;}

    AbstractFleet              *GetFleet();
    void                        SetFleet(AbstractFleet * pFleet);

    AbstractFleetParameters    *GetFleetParameters();
    void                        SetFleetParameters(AbstractFleetParameters * pParams);

    ScheduleElement            *GetScheduleElement();
    void                        SetScheduleElement(ScheduleElement * pScheduleElement);

    void                        InitializeCarFollowing();

    double                      GetCreationPosition() {return m_dbCreationPosition;}

    void                        SetGenerationPosition(double dbGenerationPosition) { m_dbGenerationPosition = dbGenerationPosition; }
    double                      GetGenerationPosition() { return m_dbGenerationPosition; }

    // Recalcule la liste des voies intermÃ©diaires empruntÃ©es au cours du pas de temps suite Ã  une modification de la position du vÃ©hicule
    void                        UpdateLstUsedLanes();

    void                        AddTuyauParcouru(Tuyau * pLink, double dbInstant);


    //ETS 130801 -> mesoscopique precessing
    public:
    enum ETimeCode
    {

        TC_INCOMMING,
        TC_OUTGOING,
        TC_ARRIVED_AT_EXIT
    };
    void SetTimeCode( Tuyau * pTuyau, ETimeCode eTimeCodeType,double dInstant);
    double GetTimeCode(Tuyau * pTuyau, ETimeCode eTimeCodeType) const;

    void SetTuyauMeso(  Tuyau * pTuyau, double dInstant);

    Tuyau* GetCurrentTuyauMeso() const;

    virtual double CalculDeceleration(double dbPasTemps, double dbDistanceArret);

    void CalculTraficEx(double dbInstant, std::vector<int> & vehiculeIDs);
    boost::shared_ptr<Vehicule> GetLeader();

    bool IsMeso();
    void SetInsertionMesoEnAttente(Tuyau * pNextTuyauMeso);

    std::set<AbstractSensor*> GetPotentiallyImpactedSensors(SensorsManager * pSensorsManager);

    std::set<TankSensor*> & getReservoirs();

#ifdef USE_SYMUCOM
    ITSStation* GetConnectedVehicle(){return m_pConnectedVehicle;}
    void SetConnectedVehicle(ITSStation* pConnectedVehicle){m_pConnectedVehicle =  pConnectedVehicle;}
#endif // USE_SYMUCOM

private:
    Tuyau * m_pCurrentTuyauMeso;
    std::map< Tuyau*, std::map<ETimeCode, double> > m_timeCode;
    //end ETS add
    Tuyau * m_pNextTuyauMeso;

protected:
    std::map<int, MouvementsSortie> AdaptMouvementAutorises(const std::map<int, boost::shared_ptr<MouvementsSortie> > & mapMvts, double dbInstant, Tuyau* pTS);

    virtual bool CheckNetworkInfluence(double dbInstant, double dbTimeStep, VoieMicro * pLane, double laneLength, size_t iLane, const std::vector<VoieMicro*> & lstLanes,
        double dbTravelledDistance, double dbGoalDistance, double startPos, double endPos, double & dbMinimizedGoalDistanceForLane);

private:
    double GetLigneDeFeu(VoieMicro * pVoiePrev, Connexion * pCnx, Tuyau * pTuyauNext, int NumVoieNext);
    double ComputeContextRange(double dbTimeStep);

    // Comparaison de deux positions au sein de l'itinÃ©raire du vÃ©hicule
    bool IsCloserThan(Tuyau * pT1, double dbPos1, Tuyau * pT2, double dbPos2);
    // Calcul de la distance entre deux points de deux troncons
    bool ComputeDistance(Tuyau * pT1, double dbPos1, Tuyau * pT2, double dbPos2, double & dbDistance);
    void ChangementVoieExt(double dbInstant, VoieMicro * pVoieCible, SensorsManagers * pGestionsCapteurs);
    void ChangementVoie(double dbInstant, VoieMicro * pVoieCible, SensorsManagers * pGestionsCapteurs, boost::shared_ptr<Vehicule> pVehLeader,
        boost::shared_ptr<Vehicule> pVehLeaderOrig, boost::shared_ptr<Vehicule> pVehFollower);
    double GetTirage(Voie* pVoieAmont);
    void GestionTraversees(double dbInstant, std::vector<int> & vehiculeIDs);
    bool CheckInsertionFromLink(double dbInstant, VoieMicro * pPreviousLane);




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);
};

#pragma warning(pop)

#endif
