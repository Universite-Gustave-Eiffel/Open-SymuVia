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
#ifndef voieH
#define voieH


#pragma warning(push)
#pragma warning(disable : 4251)

#include "SymubruitExports.h"

#include "troncon.h"

#include <boost/shared_ptr.hpp>

#include <set>


class Vehicule;
class Segment;
class SegmentMacro;
class Frontiere;
class CDiagrammeFondamental;
class Loi_emission;
class RegulationBrique;
class SousTypeVehicule;
class ControleurDeFeux;

class VoieIndex
{
public:
	VoieIndex() { m_nNumVoie = 0; }
	VoieIndex(std::string sTuyau, int NumVoie) { m_sTuyauID = sTuyau; m_nNumVoie = NumVoie; }
	VoieIndex(const VoieIndex & VI) { m_sTuyauID = VI.m_sTuyauID; m_nNumVoie = VI.m_nNumVoie; }

public:
	std::string m_sTuyauID;
	int m_nNumVoie;

public:
	bool operator==(const VoieIndex & VI)
	{
		return (VI.m_nNumVoie == m_nNumVoie) && (VI.m_sTuyauID == m_sTuyauID);
	}
	bool operator!=(const VoieIndex & VI)
	{
		return (VI.m_nNumVoie != m_nNumVoie) || (VI.m_sTuyauID != m_sTuyauID);
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};


class LigneDeFeu
{
public:
    LigneDeFeu();

    double              m_dbPosition;
    ControleurDeFeux   *m_pControleur;
    Tuyau              *m_pTuyAm;
    Tuyau              *m_pTuyAv;
    int                 nVAm;
    int                 nVAv;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

/*===========================================================================================*/
/* Classe de modÃ©lisation d'une voie                                         */
/*===========================================================================================*/

class SYMUBRUIT_EXPORT Voie : public Troncon
{
public:
    // Constructeur
    Voie    ();
    Voie    (double m_dbLargeur, double dbVitReg);

    // Destructeur
    ~Voie   (){};

protected:
        
    Voie*           m_pVoieGauche;          // Voie Ã  gauche
    Voie*           m_pVoieDroite;          // Voie Ã  droite

    double          m_dbLargeur;            // Largeur de la voie

    std::map<RegulationBrique*, bool>       m_bIsVoieCibleInterdite; // Permet d'interdir le changement de voie vers cette voie
    std::map<RegulationBrique*, bool>       m_bIsRabattementDevantVehiculeGuideInterdit;    // Permet d'interdire le changement de voie vers cette voie si un vÃ©hicule guidÃ© est en amont

    std::vector<LigneDeFeu>                 m_LstLignesFeux;    // Liste des lignes de feux se trouvant sur la voie

public:
        // MÃ©thodes virtuelles pures
        virtual void    InitVarSimu         () = 0;
        virtual void    TrafficOutput       ()                                              = 0;
        virtual void    EndComputeTraffic   (double dbInstant)                               = 0;
        virtual void    ComputeNoise         (Loi_emission* Loi)                               = 0;

        virtual void    ComputeOffer         (double dbInstant =0)                            = 0;
        virtual void    ComputeDemand       (double dbInstSimu, double dbInstant =0)                            = 0;

        virtual bool    Discretisation      (int nNbVoie)                                   = 0;

        virtual void    EmissionOutput(){};                                                     

		int             GetNum();		

        // Input - output
        Voie*           GetVoieDroite() {return m_pVoieDroite;};
        Voie*           GetVoieGauche() {return m_pVoieGauche;};
        void            SetVoieDroite(Voie* pVoie){m_pVoieDroite = pVoie;};
        void            SetVoieGauche(Voie* pVoie){m_pVoieGauche = pVoie;};

        // Gestion des possibilitÃ©s de changement de voie vers cette voie
        void            SetTargetLaneForbidden(bool bIsVoieCibleInterdite);
        bool            IsTargetLaneForbidden(SousTypeVehicule & sousType);
        void            SetPullBackInInFrontOfGuidedVehicleForbidden(bool bIsRabattementDevantVehiculeGuideInterdit);
        bool            IsPullBackInInFrontOfGuidedVehicleForbidden(SousTypeVehicule & sousType);

		VoieIndex GetUnifiedID() { return VoieIndex(m_pParent->m_strLabel, GetNum()); }

        std::vector<LigneDeFeu> & GetLignesFeux() {return m_LstLignesFeux;}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};
BOOST_SERIALIZATION_ASSUME_ABSTRACT(Voie) // la classe est pure virtual

class SYMUBRUIT_EXPORT VoieMacro : public Voie
{
public:
    // Constructeur
    VoieMacro    ();
    VoieMacro    (double dbLargeur, double dbVitReg);

    // Destructeur
    ~VoieMacro   ();

private:
    // Variables liÃ©es Ã  la discrÃ©tisation spatiale
    SegmentMacro**      m_pLstSegment;          // liste de segments
    Frontiere**         m_pLstFrontiere;        // liste de frontiere

public:
            void        SupprimeSegments        ();
    virtual bool        Discretisation          (int nNbVoie);
    virtual void        InitVarSimu             ();

	virtual bool        InitSimulation          (bool bCalculAcoustique, bool bSirane, std::string strTuyau);

    virtual void        ComputeDemand           (double dbInstSimu, double dbInstant =0);
    virtual void        ComputeOffer             (double dbInstant =0);

    virtual void        TrafficOutput           ();
    virtual void        ComputeTraffic            (double dbInstant);
    virtual void        EndComputeTraffic         (double dbInstant){};
    virtual void        ComputeTrafficEx          (double dbInstant);

    virtual void        ComputeNoise            (Loi_emission* Loi);

    SegmentMacro*        GetSegment              (int num);

    virtual void        InitAcousticVariables(){};
    virtual void        EmissionOutput(){};

    CDiagrammeFondamental*  GetDiagFonda();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

class SYMUBRUIT_EXPORT VoieMicro : public Voie
{
public:
    // Constructeur
    VoieMicro    ();
    VoieMicro    (double dbLargeur, double dbVitReg);

    // Destructeur
    ~VoieMicro   ();

private:

	// Variables descriptives
    Segment**           m_pLstCellAcoustique;   // liste des cellules acoustiques
    int                 m_nNbCellAcoustique;    // nombre de cellules acoustiques

    bool                m_bChgtVoieObligatoire; // indique si le chgt de voie est obligatoire ou non
                                                // (obligatoire dans le cas oÃ¹ la voie disparaÃ®t)   
    bool                m_bChgtVoieObligatoireEch;  // premet de caractÃ©riser une voie d'insertion "d'Ã©change" (EVOLUTION nÂ°70)
    std::vector<TypeVehicule*>  m_LstExceptionsTypesVehiculesChgtVoieObligatoire; // types de vÃ©hicules pour lesquels ne s'applique pas le chgt de voie obligatoire

    VoieMicro*          m_pPrevVoie;            // voie prÃ©cÃ©dente de la voie courante (notion utile pour la recherche de follower de la voie cible 
                                                // d'un changement de voie), existe uniquement pour les voies internes des giratoires et les convergents

	// Variables de simulation

	std::map<std::string, double> m_dbNextInstSortie;      // Instant suivant de sortie d'un vÃ©hicule, par potentielle route aval (pour hybridation SymuMaster avec SimuRes)

    double              m_dbNbVehSASEntree;
    double              m_dbNbVehSASSortie;
    double              m_dbInstSASEntreePlein;  // Instant durant le pas de temps (en s) oÃ¹ le SAS d'entrÃ©e est plein
    double              m_dbInstSASSortieVide;   // Instant durant le pas de temps (en s) oÃ¹ le SAS de sortie est vide

    double              m_dbDemandeSortie;

    boost::shared_ptr<Vehicule> m_pVehLeader;   // VÃ©hicule leader de la voie
    boost::shared_ptr<Vehicule> m_pVehLast;     // Dernier vÃ©hicule crÃ©Ã©

	int					m_nNbVeh;				// Nombre de vÃ©hicule sur la voie au dÃ©but du pas de temps
	double				m_dbPosLastVeh;			// Position du dernier vÃ©hicule sur la voie au dÃ©but du pas de temps

    std::set<boost::shared_ptr<Vehicule> , LessVehiculePtr<Vehicule> > m_LstVehiculesAnt;      // ensemble des vÃ©hicules prÃ©sents sur la voie au dÃ©but du pas de temps

public:
    // MÃ©thodes virtuelles pures de Voie
    virtual void    InitVarSimu         (){};
	virtual bool    InitSimulation      (bool bCalculAcoustique, bool bSirane, std::string strTuyau);
    virtual void    TrafficOutput       ()                                              {};
    virtual void    ComputeTraffic        (double dbInstant);
    virtual void    EndComputeTraffic     (double dbInstant);
    virtual void    ComputeTrafficEx      (double dbInstant)                                  {};
    virtual void    ComputeNoise         (Loi_emission* Loi)                               {};

    virtual void    ComputeOffer         (double dbInstant =0);
    virtual void    ComputeDemand       (double dbInstSimu, double dbInstant =0);

    virtual bool    Discretisation      (int nNbVoie);

    // MÃ©thodes d'entrÃ©e sortie
    double          GetNbVehSASEntree(){return m_dbNbVehSASEntree;}
    double          GetNbVehSASSortie(){return m_dbNbVehSASSortie;}

    void            SetNbVehSASEntree(double dbNb){m_dbNbVehSASEntree = dbNb;}
    void            SetNbVehSASSortie(double dbNb){m_dbNbVehSASSortie = dbNb;}

    std::map<std::string, double> & GetNextInstSortie(){return m_dbNextInstSortie;};

    void            SetVehLeader(boost::shared_ptr<Vehicule> pVeh){m_pVehLeader=pVeh;}
    boost::shared_ptr<Vehicule>       GetVehLeader(){return m_pVehLeader;}

    void            SupprimeCellAcoustique        ();

    void            SetChgtVoieObligatoire        (std::vector<TypeVehicule*> pLstExceptions = std::vector<TypeVehicule*>()){m_bChgtVoieObligatoire = true;m_LstExceptionsTypesVehiculesChgtVoieObligatoire = pLstExceptions;};
	void            SetNotChgtVoieObligatoire     (){m_bChgtVoieObligatoire = false;}
    bool            IsChgtVoieObligatoire         (){return m_bChgtVoieObligatoire==true;}
    bool            IsChgtVoieObligatoire         (TypeVehicule* pTV);

    // EVOLUTION nÂ°70 - cas particulier des convergents d'insertion "Ã©changeurs"
    void            SetChgtVoieObligatoireEch   () {m_bChgtVoieObligatoireEch = true;}
    bool            IsChgtVoieObligatoireEch    (){return m_bChgtVoieObligatoireEch;}

    Segment**       GetLstCellAcoustique(){return m_pLstCellAcoustique;};

    virtual void    InitAcousticVariables();
    virtual void    EmissionOutput();

    VoieMicro*      GetPrevVoie(){return m_pPrevVoie;};
    void            SetPrevVoie(VoieMicro* pVoie){m_pPrevVoie = pVoie;};

	void			UpdateNbVeh();	// Met Ã  jour le nombre de vÃ©hicule sur la voie Ã  la fin du pas de temps courant

	int				GetNbVeh(){return m_nNbVeh;}
	double			GetPosLastVeh(){return m_dbPosLastVeh;}

	double			GetLongueurEx(TypeVehicule * pTV);

    std::set<boost::shared_ptr<Vehicule> , LessVehiculePtr<Vehicule> > & GetLstVehiculesAnt() {return m_LstVehiculesAnt;}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#pragma warning(pop)

#endif