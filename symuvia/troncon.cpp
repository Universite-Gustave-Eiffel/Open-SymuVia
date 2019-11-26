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

#include "stdafx.h"
#include "troncon.h"

#include "vehicule.h"
#include "reseau.h"
#include "SystemUtil.h"

#include <boost/serialization/deque.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

using namespace std;

//================================================================
    Troncon::~Troncon   
//----------------------------------------------------------------
// Fonction  :  DestructeurG
// Version du:  11/05/2011
// Historique:  11/05/2011 (O.Tonck - INFO IPSIS)
//               Ajout de la gestion des vitesses reglementaires
//               par portions de tronÃ§on
//              24/06/2008 (C.BÃ©carie - INEO TINEA)
//               CrÃ©ation
//================================================================
(
)
{    
    DeleteLstPoints();
    
    // Nettoyage de la map des terre pleins
    map <int,  std::vector<TerrePlein>*> :: iterator IterTerrePlein;
	if( m_mapTerrePleins.size() > 0)
	{
		for(IterTerrePlein = m_mapTerrePleins.begin(); IterTerrePlein != m_mapTerrePleins.end(); IterTerrePlein++)
		{
			if( (*IterTerrePlein).second )
				delete( (*IterTerrePlein).second );
		}
		m_mapTerrePleins.clear();
	}
};

//================================================================
        Troncon::Troncon()
//----------------------------------------------------------------
// Fonction  :  Constructeur
// Version du:  03/01/2005
// Historique:  03/01/2005 (C.BÃ©carie - Tinea )
//              Ajout de la gestion du changement de voie
//              19/11/2004 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
{
        m_strRevetement = "";

		m_strLabel = "";

        m_pParent               = NULL;

        m_dbPasTemps            = 1;
        m_dbPasEspace           = 1;

        m_nNbCell               = 0;

        m_dbLongueur            = 0;        

        m_nID                   = 0;

        m_dbDstChgtVoie         = -1; // par dÃ©faut on utilisera la valeur globale du rÃ©seau

        m_dbDstChgtVoieForce    = -1;
        m_dbPhiChgtVoieForce    = 1;

        m_bCurviligne           = false;

        m_dbVitReg              = DBL_MAX; 
        m_dbVitCat              = DBL_MAX; 

        m_dbVitMaxSortie        = DBL_MAX;
        m_dbVitMaxEntree        = DBL_MAX;

        m_dbDemande             = 0;
        m_dbOffre               = 0;
}

//================================================================
        Troncon::Troncon
//----------------------------------------------------------------
// Fonction  :  Constructeur
// Version du:  15/11/2007
// Historique:  15/11/2007 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
(
    double			dbVitReg,
    double			dbVitCat,
	std::string		sNom
)
{
	m_strLabel = sNom;

    m_strRevetement = "";

    m_pParent               = NULL;

    m_dbPasTemps            = 1;
    m_dbPasEspace           = 1;

    m_nNbCell               = 0;         

    m_dbLongueur            = 0;        

    m_dbVitReg              = dbVitReg; 
    m_dbVitCat              = dbVitCat;

    m_dbVitMaxSortie        = DBL_MAX;
    m_dbVitMaxEntree        = DBL_MAX;

    m_dbDebitEntree         = 0;
    m_dbDebitSortie         = 0;

    m_dbDemande             = 0;
    m_dbOffre               = 0;

    m_nID                   = 0;

    m_dbDstChgtVoie         = -1; // par dÃ©faut on utilisera la valeur globale du rÃ©seau

    m_dbDstChgtVoieForce    = -1;
    m_dbPhiChgtVoieForce    = 1;

    m_bCurviligne           = false;
}

//================================================================
        void Troncon::SetPropCom
//----------------------------------------------------------------
// Fonction  : Initialise les propriÃ©tÃ©s communes
// Version du: 19/11/2004
// Historique: 19/11/2004 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
        Reseau*         pReseau,
        Troncon*        pParent,
		std::string     sNom,
        std::string     strRevetement
)
{
    m_pReseau = pReseau;
    m_pParent = pParent;

    m_strLabel = sNom;
    m_strRevetement = strRevetement;
}

//================================================================
        void Troncon::SetPropSimu
//----------------------------------------------------------------
// Fonction  :  Initialise les propriÃ©tÃ©s de la simulation
// Version du:  03/01/2005
// Historique:  03/01/2005 (C.BÃ©carie - Tinea )
//              Ajout de la gestion du changement de voie
//              19/11/2004 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
(
        int         nNbCell,
        double      dbPasEspace,
        double      dbPasTemps
)
{
        m_nNbCell       = nNbCell;
        m_dbPasEspace   = dbPasEspace;
        m_dbPasTemps    = dbPasTemps;
}

//================================================================
        double Troncon::GetDebitMax
//----------------------------------------------------------------
// Fonction  :  Retourne le dÃ©bit max du tronÃ§on en considÃ©rant
//              l'ensemble des types de vÃ©hicules pouvant le parcourir
// Version du:  05/01/2007
// Historique:  05/01/2007 (C.BÃ©carie - Tinea )
//              CrÃ©ation
//================================================================
(
)
{
    double  dbMax;
	std::deque <TypeVehicule*>::iterator ItCurTypeVeh;
	std::deque <TypeVehicule*>::iterator ItDebTypeVeh;
    std::deque <TypeVehicule*>::iterator ItFinTypeVeh;

    ItDebTypeVeh = m_LstTypesVehicule.begin();
    ItFinTypeVeh = m_LstTypesVehicule.end();
    dbMax = 0;

    for (ItCurTypeVeh=ItDebTypeVeh;ItCurTypeVeh!=ItFinTypeVeh;ItCurTypeVeh++)
    {
        if (dbMax < (*ItCurTypeVeh)->GetDebitMax())
            dbMax = (*ItCurTypeVeh)->GetDebitMax();
    }

    return dbMax;
}

//================================================================
        double Troncon::GetVitesseMax
//----------------------------------------------------------------
// Fonction  :  Retourne la vitesse max du tronÃ§on en considÃ©rant
//              l'ensemble des types de vÃ©hicules pouvant le parcourir
// Version du:  23/07/2007
// Historique:  05/01/2007 (C.BÃ©carie - Tinea )
//              CrÃ©ation
//              23/07/2007 (C.BÃ©carie - Tinea )
//              Ajout de la notion de vitesse rÃ©glementaire
//================================================================
(
)
{
    double  dbMax;
    std::deque <TypeVehicule*>::iterator ItCurTypeVeh;
    std::deque <TypeVehicule*>::iterator ItDebTypeVeh;
    std::deque <TypeVehicule*>::iterator ItFinTypeVeh;

    ItDebTypeVeh = m_LstTypesVehicule.begin();
    ItFinTypeVeh = m_LstTypesVehicule.end();
    dbMax = 0;

    for (ItCurTypeVeh=ItDebTypeVeh;ItCurTypeVeh!=ItFinTypeVeh;ItCurTypeVeh++)
    {
        if (dbMax < (*ItCurTypeVeh)->GetVx())
            dbMax = (*ItCurTypeVeh)->GetVx();
    }

    dbMax = min( dbMax, m_dbVitReg );

    return dbMax;
}

//================================================================
    void Troncon::AddTypeVeh
//----------------------------------------------------------------
// Fonction  :  Ajoute le type de vÃ©hicule Ã  la liste des types
// Version du:  05/01/2007
// Historique:  05/01/2007 (C.BÃ©carie - Tinea )
//              CrÃ©ation
//================================================================
(
    TypeVehicule *pTV
)
{
    // Fait-il dÃ©jÃ  parti de la liste ?
    std::deque <TypeVehicule*>::iterator cur, debut, fin;

    debut   = m_LstTypesVehicule.begin();
    fin     = m_LstTypesVehicule.end();

    for (cur=debut;cur!=fin;cur++)
    {
        if ( (*cur) == pTV )
            return;
    }

    m_LstTypesVehicule.push_back( pTV );
}

//================================================================
    void Troncon::AddPtInterne
//----------------------------------------------------------------
// Fonction  : Ajoute un point Ã  la liste des points internes
// Version du: 09/11/2007
// Historique: 09/11/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    double dbX, 
    double dbY, 
    double dbZ
)
{
    Point *ppt;
    ppt = new Point;
    
    ppt->dbX = dbX;
    ppt->dbY = dbY;
    ppt->dbZ = dbZ;

    m_LstPtsInternes.push_back( ppt );
}  
    
//================================================================
    void Troncon::DeleteLstPoints
//----------------------------------------------------------------
// Fonction  : Supprime la liste des points
// Version du: 09/11/2007
// Historique: 09/11/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
)
{
    std::deque <Point*>::iterator cur, debut, fin;

    debut   = m_LstPtsInternes.begin();
    fin     = m_LstPtsInternes.end();
    for (cur=debut;cur!=fin;cur++)
    {
        delete *cur;
    }
    m_LstPtsInternes.erase(debut, fin);
   
}

//================================================================
    void Troncon::CalculeLongueur
//----------------------------------------------------------------
// Fonction  : Calcule la longueur d'un tronÃ§on 
// Version du: 15/11/2007
// Historique: 15/11/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    double dbLongueurSaisie       // longueur saisie
)
    {
        int nPt;
        double  dbLongueur;
        
        dbLongueur = 0;
        
        if(m_LstPtsInternes.size()==0)
        {
            dbLongueur += sqrt( (m_ptExtAmont.dbX-m_ptExtAval.dbX)*(m_ptExtAmont.dbX-m_ptExtAval.dbX)
                            +   (m_ptExtAmont.dbY-m_ptExtAval.dbY)*(m_ptExtAmont.dbY-m_ptExtAval.dbY)
                            +   (m_ptExtAmont.dbZ-m_ptExtAval.dbZ)*(m_ptExtAmont.dbZ-m_ptExtAval.dbZ)) ;           
        }
        else
        {
            dbLongueur += sqrt((m_ptExtAmont.dbX-m_LstPtsInternes[0]->dbX)*(m_ptExtAmont.dbX-m_LstPtsInternes[0]->dbX)
                            +  (m_ptExtAmont.dbY-m_LstPtsInternes[0]->dbY)*(m_ptExtAmont.dbY-m_LstPtsInternes[0]->dbY)
                            +  (m_ptExtAmont.dbZ-m_LstPtsInternes[0]->dbZ)*(m_ptExtAmont.dbZ-m_LstPtsInternes[0]->dbZ));        

            for(nPt=0; nPt <(int)m_LstPtsInternes.size()-1; nPt++)
            {                        
                dbLongueur += sqrt((m_LstPtsInternes[nPt+1]->dbX-m_LstPtsInternes[nPt]->dbX)*(m_LstPtsInternes[nPt+1]->dbX-m_LstPtsInternes[nPt]->dbX)
                                +  (m_LstPtsInternes[nPt+1]->dbY-m_LstPtsInternes[nPt]->dbY)*(m_LstPtsInternes[nPt+1]->dbY-m_LstPtsInternes[nPt]->dbY)
                                +  (m_LstPtsInternes[nPt+1]->dbZ-m_LstPtsInternes[nPt]->dbZ)*(m_LstPtsInternes[nPt+1]->dbZ-m_LstPtsInternes[nPt]->dbZ)) ;           
            }

            dbLongueur += sqrt((m_LstPtsInternes[m_LstPtsInternes.size()-1]->dbX-m_ptExtAval.dbX)*(m_LstPtsInternes[m_LstPtsInternes.size()-1]->dbX-m_ptExtAval.dbX)
                            +  (m_LstPtsInternes[m_LstPtsInternes.size()-1]->dbY-m_ptExtAval.dbY)*(m_LstPtsInternes[m_LstPtsInternes.size()-1]->dbY-m_ptExtAval.dbY)
                            +  (m_LstPtsInternes[m_LstPtsInternes.size()-1]->dbZ-m_ptExtAval.dbZ)*(m_LstPtsInternes[m_LstPtsInternes.size()-1]->dbZ-m_ptExtAval.dbZ)) ;           

        }
        m_dbLongueurProjection = dbLongueur;

        if(dbLongueurSaisie == 0)
            m_dbLongueur = dbLongueur;      // Cas linÃ©aire par morceau sinon curviligne
        else
            m_dbLongueur = dbLongueurSaisie;
    }

//================================================================
	void Troncon::BuildLineStringXY
//----------------------------------------------------------------
// Fonction  : Ajoute un terre-plein Ã  la map des variantes
//			   temporelles par voie
// Version du: 27/09/2016
// Historique: 27/09/2016 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
(
	std::vector<double> & x,
	std::vector<double> & y
) const
{
	x.push_back(m_ptExtAmont.dbX);
	y.push_back(m_ptExtAmont.dbY);
	Point * pPoint;
	for (size_t i = 0; i < m_LstPtsInternes.size(); i++)
	{
		pPoint = m_LstPtsInternes[i];
		x.push_back(pPoint->dbX);
		y.push_back(pPoint->dbY);
	}
	x.push_back(m_ptExtAval.dbX);
	y.push_back(m_ptExtAval.dbY);
}

//================================================================
	void Troncon::AddTerrePlein
//----------------------------------------------------------------
// Fonction  : Ajoute un terre-plein Ã  la map des variantes
//			   temporelles par voie
// Version du: 25/07/2011
// Historique: 25/07/2011 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
(
    int 			    nVoie,			            // NumÃ©ro de la voie
    double              dbPosDebut,                 // position du dÃ©but du terre-plein
    double              dbPosFin,                   // position de la fin du terre-plein
	std::vector<double> dbDurees,		            // DurÃ©es des variantes
    std::vector<bool>   bActifs,                    // ActivitÃ© des variantes
    std::vector<PlageTemporelle*>   plages,         // DurÃ©es des variantes
    std::vector<bool>               bPlagesActives  // ActivitÃ© des variantes
)
{    	
    assert(dbDurees.size() == bActifs.size());

    vector<TerrePlein> *pListTP = m_mapTerrePleins[nVoie];
    // (et crÃ©ation si nÃ©cessaire)
	if( pListTP == NULL )
	{
        pListTP = new vector<TerrePlein>;
        m_mapTerrePleins[nVoie] = pListTP;
	}

    // Si un terre-plein identique existe dÃ©ja, on le met Ã  jour (utile pour les updates et sans indidence dans le cas d'un loadreseau normal)
    vector<TerrePlein>::iterator tpItr = pListTP->begin();
    while(tpItr != pListTP->end())
    {
        TerrePlein & tp = *tpItr;
        if(tp.dbPosDebut == dbPosDebut 
            && tp.dbPosFin == dbPosFin)
        {
            tpItr = pListTP->erase(tpItr);
        }
    }

    TerrePlein tp(dbPosDebut, dbPosFin, dbDurees, bActifs, plages, bPlagesActives);

    // Ajout de l'exception Ã  la liste
    pListTP->push_back(tp);
}

//================================================================
    bool Troncon::HasTerrePlein
 //----------------------------------------------------------------
// Fonction  : Teste s'il exist un terre-plein entre les deux
//             voies passÃ©es en paramÃ¨tres Ã  l'instant spÃ©cifiÃ©
// Version du: 25/07/2011
// Historique: 25/07/2011 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
(       
    double dbInst,
    double dbPos,
    int nVoie,
    int nVoie2
)
{
    bool result = false;

    int nVoieMin, nVoieMax;
    nVoieMin = min(nVoie, nVoie2);
    nVoieMax = max(nVoie, nVoie2);

    // si au moins un terre plein entre les deux voies, on renvoie true
    for(int i = nVoieMin; i < nVoieMax && !result; i++)
    {
        if( m_mapTerrePleins.find( i ) != m_mapTerrePleins.end() )
	    {
            std::vector<TerrePlein> * pListTerrePleins = m_mapTerrePleins[i];
            size_t nbTP = pListTerrePleins->size();
            for(size_t TPIdx = 0; TPIdx < nbTP && !result; TPIdx++)
            {
                TerrePlein & tp = pListTerrePleins->at(TPIdx);

                // seulement si la position est couverte par le terre-plein
                if(tp.dbPosDebut <= dbPos && tp.dbPosFin >= dbPos)
                {
                    // rÃ©cupÃ©ration de la dÃ©finition du terre-plein au bon moment

                    // on commence par regarder les plages temporelles
                    size_t nbPlages = tp.pPlages.size();
                    for(size_t j = 0; j < nbPlages && !result; j++)
                    {
                        if(dbInst >= tp.pPlages[j]->m_Debut && dbInst <= tp.pPlages[j]->m_Fin)
                        {
                            // on a la bonne variante pour le terre plein courant
                            if(tp.bPlagesActives[j])
                            {
                                result = true;
                            }
                        }
                    }

                    // on regarde ensuite les sÃ©quences de durÃ©es
                    double dbCumulDuree = - m_pReseau->GetLag();
                    size_t nbVariations = tp.dbDurees.size();
                    for(size_t j = 0; j < nbVariations && !result; j++)
                    {
                        if(dbInst >= dbCumulDuree && dbInst <= dbCumulDuree + tp.dbDurees[j])
                        {
                            // on a la bonne variante pour le terre plein courant
                            if(tp.bActifs[j])
                            {
                                result = true;
                            }
                        }
                        dbCumulDuree += tp.dbDurees[j];
                    }
                }
            }
        }
    }

    return result;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template void TerrePlein::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void TerrePlein::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void TerrePlein::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(dbPosDebut);
    ar & BOOST_SERIALIZATION_NVP(dbPosFin);
    ar & BOOST_SERIALIZATION_NVP(dbDurees);
    ar & BOOST_SERIALIZATION_NVP(bActifs);
    ar & BOOST_SERIALIZATION_NVP(pPlages);
    ar & BOOST_SERIALIZATION_NVP(bPlagesActives);
}

template void Troncon::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void Troncon::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void Troncon::serialize(Archive& ar, const unsigned int version)
{
	ar & BOOST_SERIALIZATION_NVP(m_strLabel);
	ar & BOOST_SERIALIZATION_NVP(m_nID);
	ar & BOOST_SERIALIZATION_NVP(m_strRevetement);
	ar & BOOST_SERIALIZATION_NVP(m_pParent);

    ar & BOOST_SERIALIZATION_NVP(m_strRoadLabel);

	ar & BOOST_SERIALIZATION_NVP(m_nNbCell);
	ar & BOOST_SERIALIZATION_NVP(m_dbPasEspace);
    ar & BOOST_SERIALIZATION_NVP(m_dbPasTemps);

    ar & BOOST_SERIALIZATION_NVP(m_dbDemande);
    ar & BOOST_SERIALIZATION_NVP(m_dbOffre);

    ar & BOOST_SERIALIZATION_NVP(m_dbDebitEntree);
    ar & BOOST_SERIALIZATION_NVP(m_dbDebitSortie);

    ar & BOOST_SERIALIZATION_NVP(m_dbVitMaxSortie);
    ar & BOOST_SERIALIZATION_NVP(m_dbVitMaxEntree);

    ar & BOOST_SERIALIZATION_NVP(m_pReseau);

    ar & BOOST_SERIALIZATION_NVP(m_ptExtAmont);
    ar & BOOST_SERIALIZATION_NVP(m_ptExtAval);

    ar & BOOST_SERIALIZATION_NVP(m_dbDstChgtVoie);

    ar & BOOST_SERIALIZATION_NVP(m_dbDstChgtVoieForce);
    ar & BOOST_SERIALIZATION_NVP(m_dbPhiChgtVoieForce);

    ar & BOOST_SERIALIZATION_NVP(m_dbVitReg);
            
    ar & BOOST_SERIALIZATION_NVP(m_bCurviligne);
    ar & BOOST_SERIALIZATION_NVP(m_LstPtsInternes);

    ar & BOOST_SERIALIZATION_NVP(m_bCurviligne);
    ar & BOOST_SERIALIZATION_NVP(m_LstPtsInternes);

    ar & BOOST_SERIALIZATION_NVP(m_dbLongueur);
    ar & BOOST_SERIALIZATION_NVP(m_dbLongueurProjection);

    ar & BOOST_SERIALIZATION_NVP(m_LstTypesVehicule);

    ar & BOOST_SERIALIZATION_NVP(m_mapTerrePleins);       
}


