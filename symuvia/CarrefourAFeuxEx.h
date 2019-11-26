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
#ifndef CarrefourAFeuxExH
#define CarrefourAFeuxExH

#include "BriqueDeConnexion.h"

#include <deque>

class TuyauMicro;
class VoieMicro;
class Reseau;
class Convergent;
struct GrpPtsConflitTraversee;

// Structure de description d'une entrÃ©e du CAF
struct EntreeCAFEx
{
    TuyauMicro*         pTAm;               // TronÃ§on
    int                 nVoie;              // NumÃ©ro de voie

    int                 nPriorite;          // Type de prioritÃ© ( 0 : par dÃ©faut = prioritÃ© Ã  droite, 1 : cÃ©dez-le passage, 2 : stop)

    std::deque<TuyauMicro*> lstTBr;         // Liste ordonnÃ©e des tronÃ§ons internes de la branche

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

// Structure de description d'une sortie du CAF
struct SortieCAFEx
{
    TuyauMicro*         pTAv;               // TronÃ§on
    int                 nVoie;              // NumÃ©ro de voie

    std::deque<TuyauMicro*> lstTBr;         // Liste ordonnÃ©e des tronÃ§ons internes de la branche

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

// Structure de description d'un mouvement du CAF
struct MvtCAFEx
{
    MvtCAFEx() {ent=NULL;sor=NULL;}
    EntreeCAFEx*             ent;
    SortieCAFEx*             sor;

    std::deque<TuyauMicro*> lstTMvt;                                    // Liste ordonnÃ©e des tronÃ§ons internes du mouvement
    std::deque<GrpPtsConflitTraversee*> lstGrpPtsConflitTraversee;      // Liste ordonnÃ©e des groupes de points de conflit traversÃ©e
    std::deque<double> lstPtAttente;                                    // Liste ordonnÃ©es des positions points d'attente du mouvement
	std::deque<int>	   lstNbVehMax;                                     // Liste ordonnÃ©es des nombre de veh. max
    boost::shared_ptr<MouvementsSortie> pMvtSortie;                     // Liste des types de vÃ©hicules exclus du mouvement

    bool IsTypeExclu(TypeVehicule* pTV, SousTypeVehicule * pSousType) const {return pMvtSortie->IsTypeExclu(pTV, pSousType);}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

class CarrefourAFeuxEx : public BriqueDeConnexion
{
public:
    // Constructeurs, destructeurs et assimilÃ©s
    CarrefourAFeuxEx(){}; // Constructeur par dÃ©faut    
    CarrefourAFeuxEx(char *strID, double dbVitMax, double dbTagreg, double dbGamma, double dbMu , 
                    double  dbLong, bool    bTraversees, Reseau *pReseau);
    ~CarrefourAFeuxEx(); // Destructeur

public:

    // Variables caractÃ©ristiques du carrefour Ã  feux
    std::deque<EntreeCAFEx*>        m_LstEntrees;           // Liste des entrÃ©es
    std::deque<SortieCAFEx*>        m_LstSorties;           // Liste des sorties
    std::deque<EntreeCAFEx*>        m_LstEntreesBackup;     // Liste des entrÃ©es sauvegardÃ©es
    std::deque<SortieCAFEx*>        m_LstSortiesBackup;     // Liste des sorties sauvegardÃ©es
    std::deque<MvtCAFEx*>           m_LstMouvements;        // Liste des mouvements
    std::deque<MvtCAFEx*>           m_LstMouvementsBackup;  // Liste des anciens mouvements conservÃ©s en mÃ©moire
    std::deque<Convergent*>         m_LstCvgs;              // Liste des convergents du CAF

private:

    double      m_dbTagreg;                         // PÃ©riode d'agrÃ©gation
    double      m_dbGamma;
    double      m_dbMu;
    
    double      m_dbLongueurCellAcoustique;         // Longueur dÃ©sirÃ©e des cellules acoustiques    

    // Variables de simulation
public:

    EntreeCAFEx*     AddEntree(Tuyau*  pTAm, int nVAm);
    SortieCAFEx*     AddSortie(Tuyau*  pTAv, int nVAv);
    MvtCAFEx*        AddMouvement(TuyauMicro*  pTAm, int nVAm, TuyauMicro*  pTAv, int nVAv, boost::shared_ptr<MouvementsSortie> pMvtSortie);    
    
	bool    Init( XERCES_CPP_NAMESPACE::DOMNode *pXmlNodeCAF, Logger *pChargement);    // Initialisation du carrefour Ã  feux   
    void    FinInit(Logger *pChargement);                                             // etape post initialisation
    bool    ReInit( XERCES_CPP_NAMESPACE::DOMNode *pXmlNodeCAF, Logger *pChargement);  // MAJ du carrefour Ã  feux   
    
    void    UpdateConvergents(double dbInstant);

    std::deque<EntreeCAFEx*> GetLstEntree(){return m_LstEntrees;};

    virtual bool    GetTuyauxInternes( Tuyau *pTAm, Tuyau *pTAv, std::vector<Tuyau*> &dqTuyaux);
    virtual bool    GetTuyauxInternes( Tuyau *pTAm, int nVAm, Tuyau *pTAv, int nVAv, std::vector<Tuyau*> &dqTuyaux);
    virtual bool    GetTuyauxInternesRestants( TuyauMicro *pTInt, Tuyau *pTAv, std::vector<Tuyau*> &dqTuyaux);
    virtual std::set<Tuyau*> GetAllTuyauxInternes( Tuyau *pTAm, Tuyau *pTAv );

    VoieMicro*      GetNextTroncon(TuyauMicro *pTAm, int nVAm, TuyauMicro* pTAv, int nVAv, TuyauMicro *pTI = NULL);
	VoieMicro*      GetNextTroncon(TuyauMicro* pTAv, int nVAv, TuyauMicro *pTI);
    int             GetVoieAmont( Tuyau* pTInterne, Tuyau* pTuyau, TypeVehicule * pTV, SousTypeVehicule * pSousType);
    VoieMicro*      GetNextVoieBackup( Tuyau* pTInterne, Tuyau* pTuyau, TypeVehicule * pTV, SousTypeVehicule * pSousType);
    
    virtual void    CalculTraversee(Vehicule *pVeh, double dbInstant, std::vector<int> & vehiculeIDs);

    void            UpdatePrioriteTraversees(double dbInstant);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif
