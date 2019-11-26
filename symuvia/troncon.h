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
#ifndef tronconH
#define tronconH

#pragma warning(push)
#pragma warning(disable : 4251)

#include "SymubruitExports.h"

#include "tools.h"

#include <boost/serialization/assume_abstract.hpp>

#include <vector>
#include <map>
#include <fstream>

class Reseau;
class TypeVehicule;

struct TerrePlein
{
public:
    TerrePlein() {};
    TerrePlein(double debut, double fin, std::vector<double> durees, std::vector<bool> actifs,
        std::vector<PlageTemporelle*> plages, std::vector<bool> plagesActives)
    {
        dbPosDebut = debut;
        dbPosFin = fin;
        dbDurees = durees;
        bActifs = actifs;
        pPlages = plages;
        bPlagesActives = plagesActives;
    };

    double  dbPosDebut;
    double  dbPosFin;
    std::vector<double>  dbDurees;
    std::vector<bool>    bActifs;
    std::vector<PlageTemporelle*> pPlages;
    std::vector<bool>             bPlagesActives;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};


class SYMUBRUIT_EXPORT Troncon
{
public:
        // Constructeur
        Troncon     ();
		Troncon     (double dbVitReg, double dbVitCat, std::string sNom);

        // Destructeur
        virtual ~Troncon();

		std::string m_strLabel;				// LibellÃ©

protected:
		
        int         m_nID;                  // Identifiant numÃ©rique du tronÃ§on                    

        std::string m_strRevetement;        // RevÃªtement

        Troncon*    m_pParent;              // Parent du tronÃ§on

        // Variables liÃ©es Ã  la simulation
        int         m_nNbCell;              // Nombre de cellule de discrÃ©tisation
        double      m_dbPasEspace;          // Pas d'espace
        double      m_dbPasTemps;           // Pas de temps

        double      m_dbDemande;            // Demande du tronÃ§on
        double      m_dbOffre;              // Offre du tronÃ§on

        double      m_dbDebitEntree;        // DÃ©bit de l'entrÃ©e
        double      m_dbDebitSortie;        // DÃ©bit de la sortie

        double      m_dbVitMaxSortie;       // Vitesse max de sortie du tronÃ§on
        double      m_dbVitMaxEntree;       // Vitesse max d'entrÃ©e du tronÃ§on

        Reseau*     m_pReseau;              // Pointeur sur le rÃ©seau        

        Point       m_ptExtAmont;           // ExtrÃ©mitÃ© amont
        Point       m_ptExtAval;            // ExtrÃ©mitÃ© aval

        double      m_dbDstChgtVoie;        // Distance avant la fin du tronÃ§on pour lequel on calcule le chgt de voie
                                            // d'un vÃ©hicule qui ne se trouve pas sur la bonne voie pour prendre le tronÃ§on suivant

        double      m_dbDstChgtVoieForce;   // Comme m_dbDstChgtVoie, mais pour forcer dans ce cas encore plus le changement de voie avec un phi donnÃ©
        double      m_dbPhiChgtVoieForce;   // Phi imposÃ© dans la zone de changement de voie forcÃ©e dÃ©finie par m_dbDstChgtVoieForce

        // Variables intrinsÃ¨ques
        double      m_dbVitReg;             // Vitesse rÃ©glementaire
        double      m_dbVitCat;             // Vitesse Ã  utiliser pour l'estimation des temps de parcours Ã  vide

        // Variables liÃ©es Ã  la gÃ©ographie
        bool                    m_bCurviligne;                          // Indique si le tronÃ§on est curviligne (la longueur a Ã©tÃ© saisie par l'utilisateur)
        std::deque<Point*>      m_LstPtsInternes;                       // Tableau dynamique des points interne dÃ©finissant la gÃ©omÃ©trie du tronÃ§on

        double  m_dbLongueur;                       // Longueur rÃ©elle du tronÃ§on (en mÃ¨tre)
        double  m_dbLongueurProjection;             // Longueur projetÃ© du tronÃ§on (en mÃ¨tre)

		std::deque<TypeVehicule*>       m_LstTypesVehicule;                     // Liste des types de vÃ©hicule pouvant emprunter le tronÃ§on

        std::string m_strRoadLabel;         // LibellÃ© de la route correspondante
                
public:
        std::map<int, std::vector<TerrePlein>* > m_mapTerrePleins; // map des terre-pleins (liste de variations temporelles par voie)

public:        	  

        // get - set
        int             GetID()         {return m_nID;};
        void            SetID(int nID)  {m_nID = nID;};
		std::string     GetLabel() const       {return m_strLabel;};
		void            SetLabel(std::string sLabel) {m_strLabel = sLabel;};
        int             GetNbCell()     {return m_nNbCell;};
        std::string     GetRevetement() {return m_strRevetement;};

        Troncon*        GetParent()     {return m_pParent;};

        void            SetOffre(double dbOffre){m_dbOffre = dbOffre;};
        double          GetOffre()      {return m_dbOffre;};

        double          GetDemande()    {return m_dbDemande;};
        void            SetDemande(double dbDem){m_dbDemande = dbDem;};

        void            SetDebitEntree            (double dbDebit){m_dbDebitEntree = dbDebit;};
        double          GetDebitEntree            (){return m_dbDebitEntree;}

        void            SetDebitSortie             (double dbDebit){m_dbDebitSortie = dbDebit;};
        double          GetDebitSortie(){return m_dbDebitSortie;}

        void            SetVitMaxSortie         (double dbVit){m_dbVitMaxSortie = dbVit;};
        double          GetVitMaxSortie()       {return m_dbVitMaxSortie;};

        void            SetVitMaxEntree         (double dbVit){m_dbVitMaxEntree = dbVit;};
        double          GetVitMaxEntree()       {return m_dbVitMaxEntree;};

        void            SetVitCat(double dbVit) { m_dbVitCat = dbVit; }
        double          GetVitCat()             { return m_dbVitCat; }

        double          GetDebitMax();
        double          GetVitesseMax();

        double          GetAbsAmont(){return m_ptExtAmont.dbX;};
        double          GetAbsAval (){return m_ptExtAval.dbX; };
        double          GetOrdAmont(){return m_ptExtAmont.dbY;};
        double          GetOrdAval (){return m_ptExtAval.dbY; };
        double          GetHautAmont(){return m_ptExtAmont.dbZ;};
        double          GetHautAval (){return m_ptExtAval.dbZ; };

        double          GetVitReg  (){return m_dbVitReg;  };
        void            SetVitReg  (double dbVitReg){m_dbVitReg = dbVitReg;};

        double          GetDstChgtVoie(double dbDstChgtVoieGlobal){return m_dbDstChgtVoie!=-1?m_dbDstChgtVoie:dbDstChgtVoieGlobal;}
        void            SetDstChgtVoie(double dbDstChgtVoie){m_dbDstChgtVoie = dbDstChgtVoie;}

        double          GetDstChgtVoieForce(){ return m_dbDstChgtVoieForce; }
        void            SetDstChgtVoieForce(double dbDstChgtVoieForce){ m_dbDstChgtVoieForce = dbDstChgtVoieForce; }

        double          GetPhiChgtVoieForce(){ return m_dbPhiChgtVoieForce; }
        void            SetPhiChgtVoieForce(double dbPhiChgtVoieForce){ m_dbPhiChgtVoieForce = dbPhiChgtVoieForce; }

        double          GetLength() const{return m_dbLongueur;};
        double          GetLongueurProjection(){return m_dbLongueurProjection;};

        Reseau*         GetReseau(){return m_pReseau;};

        double          GetPasEspace(){return m_dbPasEspace;};
        double          GetPasTemps(){return m_dbPasTemps;};

        void            SetNbCell(int nCell){m_nNbCell = nCell;};

        void            SetPasEspace(double dbPasEspace){ m_dbPasEspace = dbPasEspace;};

        void            SetPropCom(
                                        Reseau*         pReseau,
                                        Troncon*        pParent,
                                        std::string     strNom,
                                        std::string     strRevetement
                                        );
                                        
        void            SetPropSimu(    int         nNbCell,
                                        double      dbPasEspace,
                                        double      dbPasTemps
                                    );

        void            SetCurviligne(bool bCurv){m_bCurviligne=bCurv;};

        void            SetExtAmont(Point pt){m_ptExtAmont = pt;};
        void            SetExtAmont(double dbX, double dbY, double dbZ){m_ptExtAmont.dbX=dbX; m_ptExtAmont.dbY=dbY; m_ptExtAmont.dbZ = dbZ;};

        void            SetExtAval(Point pt){m_ptExtAval = pt;};
        void            SetExtAval(double dbX, double dbY, double dbZ){m_ptExtAval.dbX=dbX; m_ptExtAval.dbY=dbY; m_ptExtAval.dbZ=dbZ;};

        Point*          GetExtAmont(){return &m_ptExtAmont;};
        Point*          GetExtAval(){return &m_ptExtAval;};

		// Remplit les vecteurs passÃ©s par rÃ©fÃ©rence des coordonnÃ©es de la polyligne du tronÃ§on
		void			BuildLineStringXY(std::vector<double> & x, std::vector<double> & y) const;

        void            AddPtInterne( double dbX, double dbY, double dbZ);    

        std::deque<Point*> &GetLstPtsInternes()              {return m_LstPtsInternes;};
        std::deque<TypeVehicule*>    &GetLstTypesVeh()       {return m_LstTypesVehicule;};

        void                    AddTypeVeh(TypeVehicule *pTV);
        
        void                    DeleteLstPoints();

        void                    CalculeLongueur(double dbL);
        void                    SetLongueur(double dbL){m_dbLongueur = dbL;};

        // MÃ©thodes virtuelles        
        virtual bool    SortieGeoCell(std::ofstream Fic, bool bEmission){return true;};        

        // MÃ©thodes virtuelles pures
		virtual bool    InitSimulation(bool bAcou, bool bSirane, std::string strName)   = 0;
        virtual void    ComputeTraffic(double nInstant)                      = 0;
        virtual void    ComputeTrafficEx(double nInstant)                    = 0;
        virtual void    TrafficOutput()                                  = 0;
        virtual void    EmissionOutput()            = 0;
        virtual void    InitAcousticVariables()                      = 0;

		void			SetReseau(Reseau *pR){m_pReseau = pR;};

        // Ajout de la dÃ©finition d'un terre-plein
	    void            AddTerrePlein(int nVoie, double dbPosDebut, double dbPosFin, std::vector<double> durees, std::vector<bool> actifs,
            std::vector<PlageTemporelle*>   plages, std::vector<bool>   plagesActives);
        // teste la prÃ©sence d'un terre plein
        bool            HasTerrePlein(double dbInst, double dbPos, int nVoie, int nVoie2);

        void            SetRoadLabel(const std::string & strRoadLabel) { m_strRoadLabel = strRoadLabel;}
        std::string     GetRoadLabel() { return m_strRoadLabel;}

public:

		std::string GetUnifiedID() { return m_strLabel; }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Troncon) // la classe est pure virtual

#pragma warning(pop)

#endif
