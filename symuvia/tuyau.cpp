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
#include "tuyau.h"

#include "BriqueDeConnexion.h"
#include "entree.h"
#include "sortie.h"
#include "repartiteur.h"
#include "DiagFonda.h"
#include "voie.h"
#include "arret.h"
#include "reseau.h"
#include "XMLDocSirane.h"
#include "Segment.h"
#include "vehicule.h"
#include "SystemUtil.h"
#include "sensors/SensorsManager.h"
#include "sensors/PonctualSensor.h"
#include "Logger.h"
#include "ControleurDeFeux.h"

#ifdef USE_SYMUCORE
#include "sensors/EdieSensor.h"
#include <Demand/MacroType.h>
#include <Demand/VehicleType.h>
#include <Utils/TravelTimeUtils.h>
#endif // USE_SYMUCORE

#include <boost/make_shared.hpp>

#include <boost/serialization/deque.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include <sstream>
#define _USE_MATH_DEFINES
#include <math.h>


using namespace std;
//---------------------------------------------------------------------------

//#pragma package(smart_init)

//---------------------------------------------------------------------------
// Constructeur, Destructeur et assimilÃ©s
//---------------------------------------------------------------------------

// Constructeur par dÃ©faut
Tuyau::Tuyau()
{
	Type_amont='E';
	Type_aval='S';
	p_aval=NULL;
	p_amont=NULL;
        
	m_nNbVoies      = 0;
    m_nNbVoiesDis   = 0;

    m_pBriqueAmont = NULL;
    m_pBriqueAval = NULL;

    m_pBriqueParente = NULL;

	m_pCnxAssAm = NULL;
	m_pCnxAssAv = NULL;

    m_nRealNbVoies = 0;

    m_pTuyauOppose = NULL;

    m_dbCapaciteStationnement = 0;
    m_dbStockStationnement = 0;

    m_dbPenalisation = 1.0;

    m_iFunctionalClass = 0;

    m_bHasRealTravelTime = false;

    m_dbCoutPenalise = 0;
}

// Constructeur compplet
Tuyau::Tuyau(Reseau *pReseau, std::string _Nom,char _Type_amont ,char _Type_aval,ConnectionPonctuel *_p_aval,ConnectionPonctuel *_p_amont,
                 char typevoie, std::string nom_rev, std::vector<double> larg_voie, double _Abscisse_amont,double _Ordonnee_amont,
                 double _Abscisse_aval,double _Ordonnee_aval, double _Hauteur_amont,double _Hauteur_aval,
                 int _Nb_cel, int _Nb_voies,
                 double pastemps, double dbVitReg, double dbVitCat, const std::string & strRoadLabel):Troncon(dbVitReg, dbVitCat, _Nom)
{
    m_pReseau = pReseau;    

    

	Type_amont  = _Type_amont ;
	Type_aval   = _Type_aval;
	p_aval      = _p_aval ;
	p_amont     = _p_amont ;

    SetExtAmont(_Abscisse_amont, _Ordonnee_amont, _Hauteur_amont);
    SetExtAval(_Abscisse_aval, _Ordonnee_aval, _Hauteur_aval);
	
	m_nNbCell =_Nb_cel;
    m_nNbVoiesDis = _Nb_voies;

	m_nNbVoies  =_Nb_voies;
    m_dbPasTemps=pastemps;    
	m_strRevetement = nom_rev;
    m_largeursVoie=larg_voie;

    m_pBriqueAmont = NULL;
    m_pBriqueAval = NULL;

    m_pBriqueParente = NULL;

	m_pCnxAssAm = NULL;
	m_pCnxAssAv = NULL;

	m_bExpGrTaboo = false;

    m_nRealNbVoies = 0;

    m_pTuyauOppose = NULL;

    m_dbCapaciteStationnement = 0;
    m_dbStockStationnement = 0;

    m_dbPenalisation = 1.0;

    m_iFunctionalClass = 0;

    m_bHasRealTravelTime = false;

    SetRoadLabel(strRoadLabel);

    m_dbCoutPenalise = 0;
}

// Destructeur
Tuyau::~Tuyau()
{        
    // Vide la liste des rÃ©partitions
    VideRepartitionEntreeVoie();

    p_amont=NULL;
    p_aval=NULL;
}

std::vector<PonctualSensor*> & Tuyau::getListeCapteursConvergents()
{
    return m_LstCapteursConvergents;
}

std::map<SensorsManager*, std::vector<AbstractSensor*> > & Tuyau::getLstRelatedSensorsBySensorsManagers()
{
    return m_LstRelatedSensorsBySensorsManagers;
}

Voie*	Tuyau::GetVoie(int i)
{
	Voie *pVoie = NULL;
	if (i<this->m_nNbVoies)
	{
		pVoie = m_pLstVoie[i];
	}
	return pVoie;
}

bool    Tuyau::SetTuyauOppose(Tuyau * pTuy, Logger * plog)
{
    bool ok = true;
    m_pTuyauOppose = pTuy;
    if(pTuy->GetTuyauOppose() == NULL)
    {
        ok = pTuy->SetTuyauOppose(this, plog);
    }
    else
    {
        if(pTuy->GetTuyauOppose() != this)
        {
			*plog << "Opposite links definition mismatch between links " << pTuy->GetLabel() << " and " << pTuy->GetTuyauOppose()->GetLabel() << std::endl;
            ok = false;
        }
    }

    return ok;
}

// Ajout de la dÃ©finition d'une zone de dÃ©passement interdit
void        Tuyau::AddZoneDepassementInterdit(double dbDebut, double dbFin)
{
    ZoneDepassementInterdit zone(dbDebut, dbFin);
    m_LstZonesDepassementInterdit.push_back(zone);
}

// Ajout de la dÃ©finition d'un zLevel
bool        Tuyau::AddZoneZLevel(double dbDebut, double dbFin, int zLevel)
{
    // Si une zone dÃ©jÃ  existante intersecte, on Ã©mettra un warning
    bool bResult = false;
    for(size_t i = 0; i < m_LstZonesZLevel.size() && !bResult; i++)
    {
        if( dbFin <= m_LstZonesZLevel[i].dbPosDebut || dbDebut >= m_LstZonesZLevel[i].dbPosFin)
        {
            bResult = false; // pas de problÃ¨me
        }
        else
        {
            bResult = true; // chevauchement !
        }
    }

    ZoneZLevel zone(dbDebut, dbFin, zLevel);
    m_LstZonesZLevel.push_back(zone);

    return bResult;
}

// test de la prÃ©sence d'une zone de dÃ©passement interdit Ã  la position demandÃ©e
bool        Tuyau::IsDepassementInterdit(double dbPos)
{
    bool result = false;

    size_t nbZones = m_LstZonesDepassementInterdit.size();
    for(size_t i = 0; i < nbZones; i++)
    {
        if(m_LstZonesDepassementInterdit[i].dbPosDebut <= dbPos
            && m_LstZonesDepassementInterdit[i].dbPosFin > dbPos)
        {
            result = true;
        }
    }

    return result;
}

//---------------------------------------------------------------------------
// Fonctions liÃ©s Ã  la discrÃ©tisation et la simulation des tuyaux
//---------------------------------------------------------------------------

// Segmentation (discrÃ©tisation du tuyau en voie puis en segment)
bool TuyauMacro::Segmentation()
{
//crÃ©e pour un tuyau donnÃ© sa liste de voie et les initialise et
// ecrit ses donnÃ©es gÃ©omÃ©triques dans les fichiers de sortie

    bool    bOk;
    VoieMacro*   pVoie;    
    double  pas_espace;

    bOk = true;

    // MÃ©nage
    DeleteLanes();

    m_nRealNbVoies = m_nNbCell;

    pVoie = new VoieMacro(m_largeursVoie[0], m_dbVitReg);

    if( m_nNbCell > 0)
    {        
        pas_espace=m_dbLongueur/m_nNbCell;
    }

    pVoie->SetPropCom(m_pReseau, this, m_strLabel, m_strRevetement );

    pVoie->SetExtAmont( m_ptExtAmont );
    pVoie->SetExtAval( m_ptExtAval );    

    pVoie->SetPropSimu(m_nNbCell,pas_espace,m_dbPasTemps);

    //pVoie->SetLstTypesVeh( m_LstTypesVehicule, m_LstCoefTypesVeh);

    m_pLstVoie.push_back(pVoie);

    // Segmentation de la voie
    bOk &= pVoie->Discretisation(m_nNbVoies);

    m_nNbVoiesDis = 1;

    return bOk;
}


// Fin du calcul de l'Ã©coulement aux connections (on rÃ©cupÃ¨re les donnÃ©es provenant des rÃ©partiteurs)
void Tuyau::FinCalculEcoulementConnections(double h)
{
    // Fin de calcul des dÃ©bits entrant et sortant des tronÃ§ons en rÃ©cupÃ©rant les infos provenant de la simulation des rÃ©partiteurs
    if (Type_amont =='R')
    {
        for(int i=0;  i<m_nNbVoiesDis;i++)
        {
            double dbDebitEntree =  ((Repartiteur *)p_amont)-> Debit_sortie(m_strLabel,i);

            m_pLstVoie[i]->SetDebitEntree( dbDebitEntree);

        }
    }

    if (Type_aval =='R')
    {
        for(int i=0; i<m_nNbVoiesDis;i++)
        {
            double dbDebitSortie =  ((Repartiteur *)p_aval)-> Debit_entree(m_strLabel, i);

            m_pLstVoie[i]->SetDebitSortie( dbDebitSortie );
        }
    }

}

//================================================================
    void TuyauMacro::SetCoeffAmont
//----------------------------------------------------------------
// Fonction  : Calcul et assignation des coefficients de pondÃ©ration
//             du calcul de N en amont
// Version du: 07/07/2006
// Historique: 07/07/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    double dbPasEspace,
    double dbPasTemps,
    double dbVitMax
)
{
    // Coeff. de pondÃ©ration de N(x-dx, t -dt)
    m_dbCoeffAmont1 = 1 - ( (2*dbPasTemps - (dbPasEspace / dbVitMax))/dbPasTemps);

    // Coeff. de pondÃ©ration de N(x-dx, t)
    m_dbCoeffAmont2 = 1 - m_dbCoeffAmont1;
}

//================================================================
    void TuyauMacro::SetCoeffAval
//----------------------------------------------------------------
// Fonction  : Calcul et assignation des coefficients de pondÃ©ration
//             du calcul de N en aval
// Version du: 07/07/2006
// Historique: 07/07/2006 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    double dbVal
)
{
    // Coeff. de pondÃ©ration de N(x+dx, t - n*dt)
    m_dbCoeffAval1 = 1 - (ceil(dbVal) - dbVal);
    // Coeff. de pondÃ©ration de N(x+dx, t - (n-1)*t)
    m_dbCoeffAval2 = 1 - m_dbCoeffAval1;
}


// Calcul des niveaux de bruit des segments du tronÃ§on
void TuyauMacro::ComputeNoise(Loi_emission* Loi)
{
  int i;

  // calcul du bruit emis par les segments du troncon et ecriture des fichiers de sortie
        for(i=0; i<m_nNbVoiesDis;i++)
        {
                m_pLstVoie[i]->ComputeNoise(Loi);
        }
}

//---------------------------------------------------------------------------
// Fonctions d'affectation des variables des tronÃ§ons
//---------------------------------------------------------------------------

// affecte un pointeur sur la connection aval
void Tuyau::setConnectionAval(ConnectionPonctuel* _p_aval)
{   p_aval=_p_aval;  }

// affecte un pointeur sur la connection amont
void Tuyau::setConnectionAmont(ConnectionPonctuel* _p_amont)
{   p_amont=_p_amont; }


//---------------------------------------------------------------------------
// Fonctions de renvoi des variables des tronÃ§ons
//---------------------------------------------------------------------------

// renvoi pointeur sur connection aval
ConnectionPonctuel* Tuyau::getConnectionAval(void)
{   
   
   return p_aval; 
   

}

// renvoi pointeur sur connection amont
ConnectionPonctuel* Tuyau::getConnectionAmont(void)
{  
    return p_amont; 
  
}

// renvoi le type d'Ã©lements de rÃ©seau en aval
char Tuyau::get_Type_aval(void)
{  return Type_aval;         }

// renvoi le type d'Ã©lements de rÃ©seau en amont
char Tuyau::get_Type_amont(void)
{  return Type_amont;         }

// renvoi le nombre de voies
int Tuyau::getNb_voies(void) const
{  return m_nNbVoies;          }

// renvoi le nombre de voies
void Tuyau::setNb_voies(int iNbVoies)
{  m_nNbVoies = iNbVoies;          }


// renvoi la vitesse maximale autorisÃ©e en sortie
// (pour l'instant, c'est le minimum de la vitesse max en sortie de toutes les voies

double Tuyau::GetVitesseMaxSortie(void)
{
        double  dbVitMax=0;
        double  dbVitVoie=0;
        int     i;

//        dbVitMax = m_pLstVoie[0]->GetVitesseMaxSortie();

        for(i=1; i<m_nNbVoiesDis;i++)
        {
//                dbVitVoie = m_pLstVoie[i]->GetVitesseMaxSortie();

                if( dbVitVoie < dbVitMax )
                        dbVitMax =  dbVitVoie;
        }

        return dbVitMax;
}

double Tuyau::GetVitesseMaxSortie(int nVoie)
{
    if( nVoie >  m_nNbVoiesDis-1)
        return 0;

    return m_pLstVoie[nVoie]->GetVitMaxSortie();
}


// renvoi la largeur, le type de la voie, la longueur du tronÃ§on
double Tuyau::getLargeurVoie(int nNumVoie)
{  return m_largeursVoie[nNumVoie];      }

void Tuyau::ComputeOffer()
{     
    int i;

    m_dbOffre = 0;

    for(i=0; i<m_nNbVoiesDis; i++)
    {
        m_pLstVoie[i]->ComputeOffer();
        m_dbOffre += m_pLstVoie[i]->GetOffre();
    }
}

void Tuyau::ComputeDemand(double dbInstSimu)
{

    int     i;

    m_dbDemande = 0;

    for(i=0; i<m_nNbVoiesDis; i++)
    {
        m_pLstVoie[i]->ComputeDemand(dbInstSimu);
        m_dbDemande += m_pLstVoie[i]->GetDemande();
    }
}


// Ecriture des fichiers trafic
void TuyauMacro::TrafficOutput()
{       // Ecriture du fichier de sortie du trafic pour les bus et les segments

        int i;

         for(i=0; i<m_nNbVoiesDis; i++)
        {
                m_pLstVoie[i]->TrafficOutput();
        }
}

//================================================================
    void TuyauMicro::DeleteLanes
//----------------------------------------------------------------
// Fonction  : Suppression et destruction de la liste des voies
// Version du: 19/11/2004
// Historique: 19/11/2004 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
)
{
        size_t i;

        for( i=0; i< m_pLstVoie.size(); i++)
        {
                delete (VoieMicro*)m_pLstVoie[i];
        }

        m_pLstVoie.clear();
}

//================================================================
        void TuyauMacro::DeleteLanes
//----------------------------------------------------------------
// Fonction  : Suppression et destruction de la liste des voies
// Version du: 19/11/2004
// Historique: 19/11/2004 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
)
{
        size_t i;

        for( i=0; i< m_pLstVoie.size(); i++)
        {
                delete (VoieMacro*)m_pLstVoie[i];
        }
        m_pLstVoie.clear();
}

//================================================================
        double Tuyau::GetDemande
//----------------------------------------------------------------
// Fonction  : Renvoie la demande pour une voie
// Version du: 19/11/2004
// Historique: 19/11/2004 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    int nVoie
)
{
    if( nVoie > m_nNbVoiesDis-1)
        return 0;

    return m_pLstVoie[nVoie]->GetDemande();
}

//================================================================
        double Tuyau::GetOffre
//----------------------------------------------------------------
// Fonction  : Renvoie l'offre pour une voie
// Version du: 19/11/2004
// Historique: 19/11/2004 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    int     nVoie
)
{
    if( nVoie > m_nNbVoiesDis-1)
        return 0;

    return m_pLstVoie[nVoie]->GetOffre();
}

//================================================================
    void Tuyau::CalculDebitEntree
//----------------------------------------------------------------
// Fonction  :  Calcul du dÃ©bit d'entrÃ©e dans le cas d'une connection
//              de type 'EntrÃ©e'
// Version du: 04/01/2007
// Historique: 04/01/2007 (C.BÃ©carie - Tinea)
//              Ajout d'une rÃ©partition par voie des entrÃ©es pouvant varier dans le temps
//             29/09/2006 (C.BÃ©carie - Tinea)
//              Prise en compte de l'offre de la voie lors du calcul
//              du dÃ©bit d'entrÃ©e
//             04/03/2005 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//             07/10/1024 ETS ajoute le type de vÃ©hicule
//================================================================
(
    double h
)
{
    int         i;
    double      dbNiveauDemande;

    if (Type_amont !='E' || (!((Entree *)p_amont)->IsTypeDemande() && !((Entree *)p_amont)->IsTypeDistribution()) )
        return;

    ComputeOffer();

    // RÃ©partion de l'entrÃ©e par voie Ã  l'instant simulÃ©

    std::vector<double>  coeffs = ((Entree *)p_amont)->GetRepVoieValue(h);
  
    if( coeffs.empty()) // si on a pas d'information global -> on sum les variation par type de vehicule
    {

      
        for(size_t i = 0; i <m_pReseau->m_LstTypesVehicule.size(); ++i)
        {
            RepartitionEntree* pREType = GetVariation(h, ((Entree *)p_amont)->GetLstRepVoie(m_pReseau->m_LstTypesVehicule[i]), m_pReseau->GetLag()); // le debit d'entrÃ©e est considÃ©rÃ© "globalement"
            if( pREType)
            {
                std::vector<double> pCoeffType = pREType->pCoefficients;
                for( size_t j = 0; j< pCoeffType.size() ; ++j )
                {
                    if( coeffs.size() <=j)
                    {
                        coeffs.push_back(pCoeffType[j]);
                    }
                    else
                    {
                        coeffs[j] += pCoeffType[j];
                    }
                }
            }
        }
    }
    if( coeffs.empty())
    {
        for(i=0; i<m_nNbVoiesDis; i++)
        {
            coeffs.push_back(1.0/m_nNbVoiesDis);
        }

    }
  /*  else
    {
        pCoeff = pRE->pCoefficients;
    }*/
     dbNiveauDemande= 0.0;
    
    std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* >  lstDemandes =  ((Entree *)p_amont)->GetLstDemande();

    if( lstDemandes.find( NULL ) != lstDemandes.end())
    {
        if( ((Entree *)p_amont)->GetLstDemande(NULL)->GetLstTV()->size() > 0)
        {
           dbNiveauDemande += *((Entree *)p_amont)->GetLstDemande(NULL)->GetVariationEx(h);// on calcul le dÃ©bit d'entrÃ©e globalement
        }
    }
    else
    {
        std::map<TypeVehicule*, ListOfTimeVariation<tracked_double>* > ::iterator it;
        for( it = lstDemandes.begin(); it != lstDemandes.end(); it++)
        {
            dbNiveauDemande += *(it->second->GetVariationEx(h));
        }

    }
    
 

    for(i=0; i<m_nNbVoiesDis; i++)
    {        
		//m_pLstVoie[i]->SetDebitEntree( min(m_pLstVoie[i]->GetOffre(),dbNiveauDemande*pCoeff[i] ));
        m_pLstVoie[i]->SetDebitEntree( dbNiveauDemande*coeffs[i] );
    }
}

//================================================================
    void Tuyau::CalculDebitSortie
//----------------------------------------------------------------
// Fonction  :  Calcul du dÃ©bit de sortie dans le cas d'une connection
//              de type 'Sortie'
// Version du:  04/03/2005
// Historique:  04/03/2005 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
(
    double dbInstant
)
{
    int     i;
    double  dbDemande, dbDebitSortie, dbCap;

    if (Type_aval !='S')
        return;

    dbDemande = 0;

    // Cumul de la demande sur toutes les voies (pour la sortie, pas de diffÃ©renciation
    // des voies)
    for(i=0; i<m_nNbVoiesDis;i++)
        dbDemande = dbDemande +  m_pLstVoie[i]->GetDemande();

    // Calcul du dÃ©bit de sortie par voie
    dbCap = *((Sortie *)p_aval)->GetLstCapacites()->GetVariationEx(dbInstant);

    if(dbCap<dbDemande)
    {
        dbDebitSortie= dbCap;
    }
    else
    {
        dbDebitSortie= dbDemande;
    }

    for(i=0; i<m_nNbVoiesDis;i++)
    {
        // Affectation du dÃ©bit de sortie au prorata de la demande
        if(dbDemande>0)
            m_pLstVoie[i]->SetDebitSortie(dbDebitSortie * m_pLstVoie[i]->GetDemande() / dbDemande) ;
        else
            m_pLstVoie[i]->SetDebitSortie(dbDebitSortie/m_nNbVoiesDis) ;
    }
}


//================================================================
//  TUYAUMACRO - TUYAUMACRO - TUYAUMACRO - TUYAUMACRO - TUYAUMACRO
//================================================================

//================================================================
    TuyauMacro::TuyauMacro
//----------------------------------------------------------------
// Fonction  : Constructeur
// Version du: 04/03/2005
// Historique: 04/03/2005 (C.BÃ©carie - Tinea)
//================================================================
(
):Tuyau()
{
}

//================================================================
    TuyauMacro::TuyauMacro
//----------------------------------------------------------------
// Fonction  : Constructeur complet
// Version du: 04/03/2005
// Historique: 04/03/2005 (C.BÃ©carie - Tinea)
//================================================================
(
 Reseau *pReseau, std::string _Nom,char _Type_amont ,char _Type_aval,ConnectionPonctuel *_p_aval,ConnectionPonctuel *_p_amont,
    char typevoie,  char nom_rev[256], std::vector<double> larg_voie, double _Abscisse_amont,double _Ordonnee_amont,
    double _Abscisse_aval,double _Ordonnee_aval, double _Hauteur_amont, double _Hauteur_aval , int _Nb_cel, int _Nb_voies,
    double pastemps, double dbVitReg, double dbVitCat, const std::string & strRoadLabel
) : Tuyau(pReseau, _Nom, _Type_amont , _Type_aval, _p_aval, _p_amont,
     typevoie, nom_rev, larg_voie, _Abscisse_amont, _Ordonnee_amont,
     _Abscisse_aval, _Ordonnee_aval, _Hauteur_amont, _Hauteur_aval, _Nb_cel, _Nb_voies, pastemps, dbVitReg, dbVitCat, strRoadLabel)
{
    m_nResolution = RESOTUYAU_MACRO;


    m_bCondVitAmont         = true;
    m_bCondVitAval          = true;

    m_nNbVoiesDis = 1;

	m_pDF = new CDiagrammeFondamental();
}

//================================================================
    TuyauMacro::~TuyauMacro
//================================================================
(
)
{
    DeleteLanes();
    delete m_pDF;
}    

//================================================================
    double	TuyauMacro::ComputeCost
//================================================================
(
    double dbInst,
    TypeVehicule* pTypeVeh,
    bool bUsePenalisation
)
{
    double dbCost = GetLength() / m_pDF->GetVitesseLibre();
    if(bUsePenalisation)
    {
        dbCost *= m_dbPenalisation;
    }
    return dbCost;
}

//================================================================
    void Tuyau::CalculEcoulementConnections
//----------------------------------------------------------------
// Fonction  :  Calcul de l'Ã©coulement aux connections du flux
//              (prÃ©cÃ¨de le calcul des rÃ©partiteurs)
// Version du:  07/02/2005
// Historique:  07/02/2005 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//              07/10/1024 ETS ajoute le type de vÃ©hicule
//================================================================
(
    double h
)
{
    // Si en aval d'un rÃ©partiteur, affectation de la vitesse max en entrÃ©e (utile pour le calcul
    // de l'offre modifiÃ©e du premier segment)
    if (Type_amont =='R')
    {
        for(int i=0;  i<m_nNbVoiesDis;i++)
        {
            m_pLstVoie[i]->SetVitMaxEntree(((Repartiteur *)p_amont)->getVitesseMaxEntree(m_strLabel, i));
        }
    }

    // Calcul de l'offre et de la demande des tronÃ§ons pour dÃ©terminer les dÃ©bits
    // entrant et sortant, et prÃ©parer la simulation des rÃ©partiteurs (uniquement dans le
    // cas des tuyaux macroscopiques, dans le cas des tuyaux microscopiques, l'offre
    // est Ã©valuÃ©e uniquement lorque qu'un vÃ©hicule entre ou sort, et la demande idem sauf si
    // on gÃ¨re l'accÃ©lÃ©ration bornÃ©e).
    if( GetResolution() == RESOTUYAU_MACRO)
    {
        ComputeOffer();
    }
    ComputeDemand(h);

    // Calcul du dÃ©bit de l'entrÃ©e si entrÃ©e
    CalculDebitEntree(h);

    // Calcul du dÃ©bit de sortie si sortie
    CalculDebitSortie(h);

}

//================================================================
    void Tuyau::AddVoieInterditeByTypeVeh
//----------------------------------------------------------------
// Fonction  : Ajout d'une interdiction totale de circuler sur une
//             voie pour un type de vÃ©hicule particulier
// Version du: 02/12/2013
// Historique: 02/12/2013 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
(
    std::vector<TypeVehicule*> typesInterdits,
    int voie
)
{
    for(size_t iTV = 0; iTV < typesInterdits.size(); iTV++)
    {
        TypeVehicule * pTV = typesInterdits[iTV];
        if(voie == -1)
        {
            for(int iVoie = 0; iVoie < getNb_voies(); iVoie++)
            {
                m_mapVoiesInterdites[pTV][iVoie] = true;
            }
        }
        else
        {
            m_mapVoiesInterdites[pTV][voie] = true;
        }
    }
}


//================================================================
	void Tuyau::AddVoieReserveeByTypeVeh
//----------------------------------------------------------------
// Fonction  : Ajout d'une interdiction de circuler sur une voie
//             pour un type de vÃ©hicule particulier
// Version du: 26/07/2011
// Historique: 26/07/2011 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
(
    std::vector<TypeVehicule*> typesInterdits,  // Types des vÃ©hicules concernÃ©s
    double          dbLag,                      // instant de dÃ©but de la variante
	double			dbDuree,		            // DurÃ©e de la variante
    PlageTemporelle* pPlage,                    // Plage temporelle Ã©ventuelle
    double          dbDureeSimu,                // DurÃ©e de la simulation
    int             nVoie,                      // NumÃ©ro de la voie Ã  interdire
    bool            bActive                     // activer ou dÃ©sactiver la voie rÃ©servÃ©e
)
{    	
    for(size_t i = 0; i < typesInterdits.size(); i++)
    {
        TypeVehicule* pTV = typesInterdits[i];
        // Si pas de dÃ©finition des vitesses rÃ©glementaire, on la crÃ©e
	    if( bActive && m_mapVoiesReservees.find( pTV ) == m_mapVoiesReservees.end() )
	    {
            for(int j = 0; j < getNb_voies(); j++)
            {
                m_mapVoiesReservees[pTV][j].SetLag(m_pReseau->GetLag());
                boost::shared_ptr<tracked_bool> pNewBool = boost::make_shared<tracked_bool>(bActive);
                if(j != nVoie)
                {
                    *pNewBool = false;
                }
                boost::shared_ptr<tracked_bool> pNewBool2 = boost::make_shared<tracked_bool>(bActive);
                if(!pPlage)
                {
                    m_mapVoiesReservees[pTV][j].AddVariation(dbDuree, pNewBool);
                    m_mapVoiesReservees[pTV][j].AddVariation(dbDureeSimu-dbDuree, pNewBool2);
                }
                else
                {
                    m_mapVoiesReservees[pTV][j].AddVariation(dbDureeSimu, pNewBool2);
                    m_mapVoiesReservees[pTV][j].AddVariation(pPlage, pNewBool);
                }
            }
	    }
        else
        {
            // il y a deja des vit. reg. dÃ©finies pour ce type de vÃ©hicule :
            map<int , ListOfTimeVariation<tracked_bool> > & voiesInterditesMap = m_mapVoiesReservees[pTV];

            // Par construction, il existe une liste par voie :
            ListOfTimeVariation<tracked_bool> & listVitReg = voiesInterditesMap[nVoie];
            boost::shared_ptr<tracked_bool> pNewBool = boost::make_shared<tracked_bool>(bActive);
            if(pPlage)
            {
                listVitReg.AddVariation(pPlage, pNewBool);
            }
            else
            {
                // intÃ©gration de la vitesse nulle entre dbLag et dbLag + dbDuree
                listVitReg.InsertVariation(dbLag, dbLag+dbDuree, pNewBool);
            }
        }
    }
}

// Calcule et renvoie la vitesse rÃ©glementaire maximum (en fonction de la voie) Ã  l'instant et la position donnÃ©e pour un type de vÃ©hicule donnÃ©.
double Tuyau::GetMaxVitRegByTypeVeh(TypeVehicule *pTV, double dbInst, double dbPos)
{
    double dbMaVitReg = 0;
    for(int iVoie = 0; iVoie < getNb_voies(); iVoie++)
    {
        double dbTMp = GetVitRegByTypeVeh(pTV, dbInst, dbPos, iVoie);
        dbMaVitReg = std::max<double>(dbMaVitReg, dbTMp);
    }
	return dbMaVitReg;		
}

// Calcule et renvoie la vitesse rÃ©glementaire Ã  l'instant et la position donnÃ©e pour un type de vÃ©hicule et une voie donnÃ©e,
// en vÃ©rifiant que pour cette voie, la circulation n'est pas interdite
double Tuyau::GetVitRegByTypeVeh(TypeVehicule *pTV, double dbInst, double dbPos, int nVoie)
{
    double result = m_dbVitReg;

    // Si un paramÃ©trage particulier est dÃ©fini, on l'utilise, sinon on utilise la vitesse rÃ©glementaire globale du tronÃ§on
    bool bException = false;
    VitRegDescription * pVitRegDescr = m_LstVitReg.GetVariationEx(dbInst);
    if(pVitRegDescr && pVitRegDescr->vectPortions.find(pTV) != pVitRegDescr->vectPortions.end())
    {
        if(pVitRegDescr->vectPortions[pTV].find(nVoie) != pVitRegDescr->vectPortions[pTV].end())
        {
            vector<VitRegPortion> & portions = pVitRegDescr->vectPortions[pTV][nVoie];
            for(size_t iPortion = 0; iPortion < portions.size() && !bException; iPortion++)
            {
                VitRegPortion & portion = portions[iPortion];
                if(portion.dbPosFin >= dbPos && dbPos >= portion.dbPosDebut)
                {
                    bException = true;
                    result = std::min<double>(result, portion.data); // on borne par la vitesse rÃ©glementaire maximum du tronÃ§on
                }
            }
        }
    }
     
	return result;		
}

//================================================================
    bool Tuyau::IsInterdit(TypeVehicule *pTV, double dbInst)
//----------------------------------------------------------------
// Fonction  : Indique si le troncon est interdit globalement Ã  
//			   la circulation.
// Version du: 12/05/2011
// Historique: 12/05/2011 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
{
    bool bResult = false;

    int nbVoiesInterdites = 0;
    for(int iVoie = 0; iVoie < getNb_voies(); iVoie++)
    {
        if(IsVoieInterdite(pTV, iVoie, dbInst))
        {
            nbVoiesInterdites++;
        }
    }

    bResult = nbVoiesInterdites == getNb_voies();

    return bResult;
}


//================================================================
    bool Tuyau::IsVoieInterdite(TypeVehicule *pTV, int nVoie, double dbInst)
//----------------------------------------------------------------
// Fonction  : Indique si le troncon est interdit Ã  la circulation
//             sur la voie spÃ©cifiÃ©e et pour le type de vÃ©hicule
//             spÃ©cifiÃ©.
// Version du: 28/07/2011
// Historique: 28/07/2011 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
{
    bool bResult = false;

    // on commence par regarder les voies interdites
    if( m_mapVoiesInterdites.find( pTV ) != m_mapVoiesInterdites.end() )
    {
        if(m_mapVoiesInterdites[pTV].find(nVoie) != m_mapVoiesInterdites[pTV].end())
        {
            if(m_mapVoiesInterdites[pTV][nVoie])
            {
                bResult = true;
            }
        }
    }

    // on finit en regardant les voies rÃ©servÃ©es
    if( !bResult && m_mapVoiesReservees.find( pTV ) != m_mapVoiesReservees.end() )
    {
        if(m_mapVoiesReservees[pTV].find(nVoie) != m_mapVoiesReservees[pTV].end())
        {
            tracked_bool * bInterdit = m_mapVoiesReservees[pTV][nVoie].GetVariationEx(dbInst);
            if(*bInterdit)
            {
                bResult = true;
            }
        }
    }

    return bResult;
}

//================================================================
    bool Tuyau::IsInterdit(TypeVehicule *pTV)
//----------------------------------------------------------------
// Fonction  : Indique si le troncon est interdit globalement Ã  
//			   la circulation.
// Version du: 12/05/2011
// Historique: 12/05/2011 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
{
    bool bResult = false;

    int nbVoiesInterdites = 0;
    for(int iVoie = 0; iVoie < getNb_voies(); iVoie++)
    {
        if(IsVoieInterdite(pTV, iVoie))
        {
            nbVoiesInterdites++;
        }
    }

    bResult = nbVoiesInterdites == getNb_voies();

    return bResult;
}


//================================================================
    bool Tuyau::IsVoieInterdite(TypeVehicule *pTV, int nVoie)
//----------------------------------------------------------------
// Fonction  : Indique si le troncon est interdit Ã  la circulation
//             sur la voie spÃ©cifiÃ©e et pour le type de vÃ©hicule
//             spÃ©cifiÃ©.
// Version du: 28/07/2011
// Historique: 28/07/2011 (O.Tonck - IPSIS)
//              CrÃ©ation
//================================================================
{
    bool bResult = false;

    // on commence par regarder les voies interdites
    if( m_mapVoiesInterdites.find( pTV ) != m_mapVoiesInterdites.end() )
    {
        if(m_mapVoiesInterdites[pTV].find(nVoie) != m_mapVoiesInterdites[pTV].end())
        {
            if(m_mapVoiesInterdites[pTV][nVoie])
            {
                bResult = true;
            }
        }
    }

    // on finit en regardant les voies rÃ©servÃ©es
    if( !bResult && m_mapVoiesReservees.find( pTV ) != m_mapVoiesReservees.end() )
    {
        if(m_mapVoiesReservees[pTV].find(nVoie) != m_mapVoiesReservees[pTV].end())
        {
            bResult = true;
            for(size_t i = 0; i < m_mapVoiesReservees[pTV][nVoie].GetLstTV()->size() && bResult; i++)
            {
                tracked_bool * bInterdit = m_mapVoiesReservees[pTV][nVoie].GetLstTV()->at(i).m_pData.get();
                if(!(*bInterdit))
                {
                    bResult = false;
                }
            }
        }
    }

    return bResult;
}


// Retourne le max des vitesses rÃ©glementaires du tronÃ§on Ã  l'instant et la position considÃ©rÃ©s
double Tuyau::GetMaxVitReg(double dbInst, double dbPos, int numVoie)
{
	std::deque<TypeVehicule*>::iterator itTV;
	double dbMaxVitReg = 0;

	for(itTV = m_pReseau->m_LstTypesVehicule.begin(); itTV != m_pReseau->m_LstTypesVehicule.end(); itTV++)
	{
        dbMaxVitReg = std::max<double>(dbMaxVitReg, GetVitRegByTypeVeh(*itTV, dbInst, dbPos, numVoie));
	}
  
	if( dbMaxVitReg>=DBL_MAX )
		return m_pReseau->GetMaxVitMax();
	else
		return std::max<double>(dbMaxVitReg, m_pReseau->GetMaxVitMax());
}


//================================================================
	bool Tuyau::SendSpeedLimit
//----------------------------------------------------------------
// Fonction  : Pilotage de la vitesse rÃ©glementaire
// Version du: 09/11/2009
// Historique: 09/11/2009 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
(
	double dbInst,				// Instant de la commande
	TypeVehicule *pTV,			// Type de vÃ©hicule concernÃ© 	
    int    numVoie,             // numÃ©ro de la voie concernÃ©e
    double dbDebutPortion,      // dÃ©but de la portion
    double dbFinPortion,        // fin de la portion
	double dbSpeedLimit     	// Vitesse limite
)
{
    // dÃ©termination des types de vÃ©hicules concernÃ©s
    set<TypeVehicule*> lstTypesVeh;
    if(pTV == NULL)
    {
        lstTypesVeh.insert(m_pReseau->m_LstTypesVehicule.begin(), m_pReseau->m_LstTypesVehicule.end());
    }
    else
    {
        lstTypesVeh.insert(pTV);
    }

    set<int> lstVoies;
    if(numVoie == -1)
    {
        for(int iVoie = 0; iVoie < getNb_voies(); iVoie++)
        {
            lstVoies.insert(iVoie);
        }
    }
    else
    {
        lstVoies.insert(numVoie);
    }

    // Modification des Ã©lÃ©ments concernÃ©s
    for(size_t iVariation = 0; iVariation < m_LstVitReg.GetLstTV()->size(); iVariation++)
    {
        VitRegDescription * pDescr = m_LstVitReg.GetLstTV()->operator[](iVariation).m_pData.get();

        set<TypeVehicule*>::iterator iterTypeVeh;
        for(iterTypeVeh = lstTypesVeh.begin(); iterTypeVeh != lstTypesVeh.end(); iterTypeVeh++)
        {
            map<int, vector<VitRegPortion> > & mapParVoie = pDescr->vectPortions[*iterTypeVeh];
            
            // Pour chaque voie concernÃ©e ...
            set<int>::iterator iterVoie;
            for(iterVoie = lstVoies.begin(); iterVoie != lstVoies.end(); iterVoie++)
            {
                mapParVoie[*iterVoie].clear();
                VitRegPortion portion;
                portion.data = dbSpeedLimit;
                portion.dbPosDebut = dbDebutPortion;
                portion.dbPosFin = dbFinPortion;
                mapParVoie[*iterVoie].push_back(portion);
            }
        }
    }

    return true;
}

//================================================================
    void TuyauMacro::InitVarSimu
//----------------------------------------------------------------
// Fonction  : Initialisation des variables de simulation
// Version du: 13/07/2007
// Historique: 13/07/2007 (C.BÃ©carie - Tinea)
//================================================================
(
)
{
    for(int i=0; i<getNbVoiesDis(); i++)
    {
        GetLstLanes()[i]->InitVarSimu();
    }            
}


//================================================================
    bool TuyauMacro::InitSimulation
//----------------------------------------------------------------
// Fonction  : Initialisation de la simulation
// Version du: 03/03/2005
// Historique: 03/03/2005 (C.BÃ©carie - Tinea)
//================================================================
(
    bool		bCalculAcoustique,
    bool        bSirane,
	std::string strName
)
{
        int i;

        for(i=0; i<m_nNbVoiesDis; i++)
                m_pLstVoie[i]->InitSimulation(bCalculAcoustique, bSirane, m_strLabel);

        return true;
}

//================================================================
    void TuyauMacro::ComputeTraffic
//----------------------------------------------------------------
// Fonction  :  Calcul de la quantitÃ© N
// Version du:  29/06/2006
// Historique:  29/06/2006 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
    (
        double dbInstant
    )
{
    int i;

    if(m_pLstVoie.size() == 0)
        return;

    for(i=0; i<m_nNbVoiesDis; i++)
    {
        if(m_pLstVoie[i])
            m_pLstVoie[i]->ComputeTraffic(dbInstant);
    }
}

//================================================================
    void TuyauMacro::ComputeTrafficEx
//----------------------------------------------------------------
// Fonction  :  Calcul des autres caractÃ©ristiques du trafic
// Version du:  03/07/2006
// Historique:  03/07/2006 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
    (
        double dbInstant
    )
{
    int i;

    if(m_pLstVoie.size() == 0)
        return;

    for(i=0; i<m_nNbVoiesDis; i++)
    {
        if(m_pLstVoie[i])
            m_pLstVoie[i]->ComputeTrafficEx(dbInstant);
    }
}


//================================================================
//  TUYAUMICRO - TUYAUMICRO - TUYAUMICRO - TUYAUMICRO - TUYAUMICRO
//================================================================

//================================================================
    TuyauMicro::TuyauMicro
//----------------------------------------------------------------
// Fonction  : Constructeur
// Version du: 04/03/2005
// Historique: 04/03/2005 (C.BÃ©carie - Tinea)
//================================================================
(
):Tuyau()
{
    m_nResolution = RESOTUYAU_MICRO;
    m_bAgressif = false;
    m_bChtVoieVersDroiteAutorise = true;

#ifdef USE_SYMUCORE
    m_pEdieSensor = NULL;
#endif // USESYMUCORE
}

//================================================================
    TuyauMicro::TuyauMicro
//----------------------------------------------------------------
// Fonction  : Constructeur complet
// Version du: 04/03/2005
// Historique: 04/03/2005 (C.BÃ©carie - Tinea)
//================================================================
(
 Reseau *pReseau, std::string _Nom,char _Type_amont ,char _Type_aval,ConnectionPonctuel *_p_aval,ConnectionPonctuel *_p_amont,
    char typevoie, std::string nom_rev, std::vector<double> larg_voie, double _Abscisse_amont,double _Ordonnee_amont,
    double _Abscisse_aval,double _Ordonnee_aval, double _Hauteur_amont, double _Hauteur_aval, int _Nb_voies,
    double pastemps, int nNbCellAcoustique, double dbVitReg, double dbVitCat, double dbCellAcousLength, const std::string & strRoadLabel
) : Tuyau(pReseau, _Nom, _Type_amont , _Type_aval, _p_aval, _p_amont,
     typevoie, nom_rev, larg_voie, _Abscisse_amont, _Ordonnee_amont,
     _Abscisse_aval, _Ordonnee_aval, _Hauteur_amont, _Hauteur_aval, 0, _Nb_voies, pastemps, dbVitReg, dbVitCat, strRoadLabel)
{
    m_nResolution           = RESOTUYAU_MICRO;
    
    m_nNbCellAcoustique     = nNbCellAcoustique;
    m_nNbCell = m_nNbCellAcoustique;

	m_dbAcousticCellLength = 0;

    // VÃ©rification                                     
    if(nNbCellAcoustique<1 && dbCellAcousLength<=0)
        m_nNbCellAcoustique = 1;
	m_dbAcousticCellLength = dbCellAcousLength;

    m_nNbVoiesDis = _Nb_voies;

    m_bAgressif = false;
    m_bChtVoieVersDroiteAutorise = true;

#ifdef USE_SYMUCORE
    m_pEdieSensor = NULL;
#endif // USESYMUCORE
}

//================================================================
    TuyauMicro::~TuyauMicro
//----------------------------------------------------------------
// Fonction  : Destructeur
// Version du: 07/03/2005
// Historique: 07/03/2005 (C.BÃ©carie - Tinea)
//================================================================
(
)
{
    // Suppression des traversÃ©es
    std::deque <Traversee*>::iterator par_t;
    std::deque <Traversee*>::iterator debut_t;
    std::deque <Traversee*>::iterator fin_t;

    debut_t= m_LstTraversee.begin();
    fin_t=   m_LstTraversee.end();
    for (par_t=debut_t;par_t!=fin_t;par_t++)
    {
        delete (Traversee*)(*par_t);
    }
    m_LstTraversee.erase(  debut_t,fin_t);    

	DeleteLanes();

    // Suppression des cellules Sirane
    SupprimeCellSirane();

#ifdef USE_SYMUCORE
    if(m_pEdieSensor)
        delete m_pEdieSensor;
#endif // USESYMUCORE
}

//================================================================
    bool TuyauMicro::InitSimulation
//----------------------------------------------------------------
// Fonction  : Initialisation de la simulation
// Version du: 03/03/2005
// Historique: 03/03/2005 (C.BÃ©carie - Tinea)
//================================================================
(
    bool		bCalculAcoustique,
    bool        bSirane,
	std::string strName
)
{
    int i;
    
    for(i=0; i<m_nNbVoies; i++)
    {
        m_pLstVoie[i]->InitSimulation(bCalculAcoustique, bSirane, m_strLabel);
        m_pLstVoie[i]->SetOffre( GetDebitMax());
    }

	if(m_pReseau->IsCptItineraire() )
	{
		std::deque<TypeVehicule*>::iterator itTP;
		m_mapNbVehEntre.clear();
		m_mapNbVehSorti.clear();
		m_mapNbVehSortiPrev.clear();
		m_mapNbVehEntrePrev.clear();

		for(itTP = m_LstTypesVehicule.begin(); itTP != m_LstTypesVehicule.end(); itTP++)
		{
			m_mapNbVehEntre.insert( make_pair(*itTP, 0) );
			m_mapNbVehSorti.insert( make_pair(*itTP, 0) );
			m_mapNbVehSortiPrev.insert( make_pair(*itTP, 0) );
			m_mapNbVehEntrePrev.insert( make_pair(*itTP, 0) );
		}
	}

    // init des cellules sirane
    if(bSirane)
    {
        for(size_t i = 0; i < m_LstCellSirane.size(); i++)
        {
            ePhaseCelluleSirane iPhase = UNDEFINED_PHASE;
            // il faut que le segment soit entourÃ© de deux briques
            if(GetBriqueAmont() && GetBriqueAval())
            {
                bool bStartCell = i == 0;
                bool bStopCell = i == m_LstCellSirane.size()-1;
                // Si on n'a qu'une cellule sur le segment, on lui laisse le -1.
                if(!(bStartCell && bStopCell))
                {
                    if(bStartCell)
                    {
                        iPhase = ACCELERATION_PHASE;
                    }
                    else if(bStopCell)
                    {
                        iPhase = DECELERATION_PHASE;
                    }
                    else
                    {
                        iPhase = STABILIZATION_PHASE;
                    }
                }
            }
            m_LstCellSirane[i]->InitSimulationSirane(iPhase);
        }
    }

    return true;
}


//================================================================
    bool TuyauMicro::Segmentation
//----------------------------------------------------------------
// Fonction  : Segmentation d'un tronÃ§on en voie
// Version du: 07/03/2005
// Historique: 07/03/2005 (C.BÃ©carie - Tinea)
//              Version initiale
//             31/07/2008 (C.BÃ©carie - Tinea)
//              Correction calcul des extrÃ©mitÃ©s des voies d'un
//              tronÃ§on multi-voies
//================================================================
(
)
{
    bool    bOk;
    int     i;
    VoieMicro*   pVoie;
	std::string  strTmp;
    double  dbLargTot;
    double  dbLargCum;
    double  pas_espace;

    bOk = true;

    // MÃ©nage
    DeleteLanes();

    m_nRealNbVoies = m_nNbVoies;

    // Largeur totale
    dbLargTot = 0.0;
    for(size_t i = 0; i < m_largeursVoie.size(); i++)
    {
        dbLargTot += m_largeursVoie[i];
    }
    //dbAlpha  = atan2( GetOrdAval() - GetOrdAmont(), GetAbsAval() - GetAbsAmont() );
    dbLargCum =  (dbLargTot-m_largeursVoie[0])/2.;

    // Calcul du nombre de pas de temps Ã  historiser pour le calcul des variables du trafic
    // (utilisÃ© uniquement dans le cas microscopique car sinon ce calcul est fait au niveau des types
    // de vÃ©hicule et donc Ã  l'aide du type de vÃ©hicule de base)
	m_nNbPasTempsHist =   (int)ceil( m_LstTypesVehicule.front()->GetVx() / - m_LstTypesVehicule.front()->GetW() ) ;

    for(i=0; i<m_nNbVoies; i++)
    {
        pVoie = new VoieMicro(m_largeursVoie[i], m_dbVitReg);

        // Nom        
		strTmp = m_strLabel + "V" + to_string(i+1);

        // Calcul des coordonnÃ©es gÃ©omÃ©triques
        deque<Point> lstPoints = GetLineString(dbLargCum);

        pVoie->SetExtAmont(lstPoints.front().dbX, lstPoints.front().dbY, lstPoints.front().dbZ);
        pVoie->SetExtAval(lstPoints.back().dbX, lstPoints.back().dbY, lstPoints.back().dbZ);
        for(size_t j = 1; j < lstPoints.size()-1; j++)
        {
            pVoie->AddPtInterne(lstPoints[j].dbX, lstPoints[j].dbY, lstPoints[j].dbZ);
        }

        // Calcul de la longueur rÃ©elle et projetÃ©e de la voie 
		if( fabs(m_dbLongueur - m_dbLongueurProjection) > 0.001 )	// Si la longueur projetÃ©e du tronÃ§on est diffÃ©rente de la longueur rÃ©elle (saisie par l'utilisateur)
			pVoie->CalculeLongueur(m_dbLongueur);					// la longueur rÃ©elle de la voie est dÃ©duite de celle du tronÃ§on sinon elle est calculÃ©e Ã  l'aide    
		else														// des extrÃ©mitÃ©s et des points internes
			pVoie->CalculeLongueur(0);  

		pas_espace = 1;//Alain
        if( m_nResolution == RESOTUYAU_MACRO )
            pas_espace=GetLength()/m_nNbCell;

        pVoie->SetPropCom(m_pReseau, this, strTmp, m_strRevetement );
        pVoie->SetPropSimu(m_nNbCellAcoustique,pas_espace,m_dbPasTemps);                                                                   

        // Initialisation des vitesses max d'entrÃ©e et de sortie de la voie
        pVoie->SetVitMaxEntree(GetVitesseMax());
        pVoie->SetVitMaxSortie(GetVitesseMax());                

        m_pLstVoie.push_back(pVoie);

        m_nNbVoiesDis = m_nNbVoies;

        if(i < m_nNbVoies-1)
        {
            dbLargCum = dbLargCum - (m_largeursVoie[i]+m_largeursVoie[i+1])/2.;
        }
    }

    // Initialisation des voies adjacentes
    for(i=0; i<m_nNbVoies; i++)
    {
        // Voie Ã  droite
        if(i==0)
            m_pLstVoie[i]->SetVoieDroite(NULL);
        else
            m_pLstVoie[i]->SetVoieDroite(m_pLstVoie[i-1]);

        // Voie Ã  gauche
        if(i==m_nNbVoies-1)
            m_pLstVoie[i]->SetVoieGauche(NULL);
        else
            m_pLstVoie[i]->SetVoieGauche(m_pLstVoie[i+1]);
    }

    // Segmentation de la voie en cellule acoustique si besoin
    // Finalement, on le fait tout le temps puisque Ã§a se passe au moment du chargement !
    if(  !m_pReseau->IsSortieAcoustiquePonctuel() )
    {
        for(int i=0; i<m_nNbVoies; i++)
            bOk &= m_pLstVoie[i]->Discretisation(m_nNbVoies);
    }
    
    return bOk;
}

// Construit une polyligne correspondant Ã  la gÃ©omÃ©trie du tronÃ§on, Ã  une position transversale donnÃ©e
//================================================================
std::deque<Point> Tuyau::GetLineString
//----------------------------------------------------------------
// Fonction  : Construit une polyligne correspondant Ã  la
// gÃ©omÃ©trie du tronÃ§on, Ã  une position transversale donnÃ©e.
// Version du: 21/04/2015
// Historique: 21/04/2015 (O.Tonck - Ipsis)
//================================================================
(
    double offset
)
{
    std::deque<Point> result;

    double  dbLargTot;

    // Largeur totale
    dbLargTot = 0.0;
    for(size_t i = 0; i < m_largeursVoie.size(); i++)
    {
        dbLargTot += m_largeursVoie[i];
    }

    // Construction de la liste des points qui constituent le tronÃ§on
    deque<Point>    dqPoints;    
    Point           pt;

    pt.dbX = GetAbsAmont();
    pt.dbY = GetOrdAmont();
    pt.dbZ = GetHautAmont();
    dqPoints.push_back(pt);

    for(int j=0; j< (int)m_LstPtsInternes.size(); j++)
        dqPoints.push_back( *m_LstPtsInternes[j] );

    pt.dbX = GetAbsAval();
    pt.dbY = GetOrdAval();
    pt.dbZ = GetHautAval();
    dqPoints.push_back(pt);

    result = BuildLineStringGeometry(dqPoints, offset, dbLargTot, m_strLabel, m_pReseau->GetLogger());

    return result;
}

//================================================================
    void TuyauMicro::ComputeTraffic
//----------------------------------------------------------------
// Fonction  : Calcul du trafic d'un pas de temps
// Version du: 17/07/2006
// Historique: 17/07/2006 (C.BÃ©carie - Tinea)
//================================================================
(
    double dbInstant
)
{
    for(int i=0; i<m_nNbVoies; i++)
        m_pLstVoie[i]->ComputeTraffic(dbInstant);
}

//================================================================
    void TuyauMicro::EndComputeTraffic
//----------------------------------------------------------------
// Fonction  : Fin du calcul du trafic d'un pas de temps
// Version du: 17/07/2006
// Historique: 17/07/2006 (C.BÃ©carie - Tinea)
//================================================================
(
    double dbInstant
)
{
    for(int i=0; i<m_nNbVoies; i++)
        m_pLstVoie[i]->EndComputeTraffic(dbInstant);
}

//================================================================
    void TuyauMicro::TrafficOutput
//----------------------------------------------------------------
// Fonction  :  Ecriture des fichiers trafic
// Version du:  08/03/2005
// Historique:  08/03/2005 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
(
)
{
    int i;
    std::deque <structVehicule*>::iterator par, debut, fin;
    VoieMicro        *pVoie;

    for(i=0; i<m_nNbVoiesDis; i++)
    {
        pVoie = (VoieMicro*)GetLstLanes()[i];

        // Ecriture des donnÃ©es des SAS
		*m_pReseau->FicTraficSAS << "Upstream SAS " << pVoie->GetLabel() << " " << pVoie->GetNbVehSASEntree() <<std::endl;
        *m_pReseau->FicTraficSAS << "Downstream SAS " << pVoie->GetLabel() << " " << pVoie->GetNbVehSASSortie() << std::endl;
    }
}

//================================================================
    void TuyauMicro::InitAcousticVariables
//----------------------------------------------------------------
// Fonction  : Initialisation des variables acoustiques Ã  chaque
//             pas de temps
// Version du: 17/10/2006
// Historique: 17/10/2006 (C.BÃ©carie - Tinea)
//================================================================
(
)
{
    for(int i=0; i<m_nNbVoies; i++)
        m_pLstVoie[i]->InitAcousticVariables();
}

//================================================================
    void TuyauMicro::EmissionOutput
//----------------------------------------------------------------
// Fonction  : Sortie des Ã©missions
// Version du: 17/10/2006
// Historique: 17/10/2006 (C.BÃ©carie - Tinea)
//================================================================
(    
)
{
	if( m_pReseau->m_bAcousCell )
    {
        for(int i=0; i<m_nNbVoies; i++)
            m_pLstVoie[i]->EmissionOutput();
    }            
}


//================================================================
    double  TuyauMicro::ComputeCost
//----------------------------------------------------------------
// Fonction  : Retourne le coÃ»t au sens temps de parcours d'un tuyau
// Remarque  : Un tronÃ§on avec une vitesse rÃ©glementaire Ã  0 pour le
//             type de vÃ©hicule considÃ©rÃ© Ã  un coÃ»t infini afin 
//             d'empÃªcher la circulation des vÃ©hicules sur le tronÃ§on
// Version du: 11/01/2009
// Historique: 11/09/2007 (C.BÃ©carie - Tinea)
//================================================================
(
	double			 dbInst,		// Instant de calcul du coÃ»t
    TypeVehicule    *pTypeVeh,
    bool bUsePenalisation
 )
{
    double result;
    if (IsInterdit( pTypeVeh, dbInst ) )    // CoÃ»t infini
		result = DBL_MAX;
    else
    {
		if( m_mapTempsParcours.find( pTypeVeh ) != m_mapTempsParcours.end() )	// Utilisation du temps de parcours estimÃ©
			result = m_mapTempsParcours.find( pTypeVeh )->second;
		else
            result = GetCoutAVide(dbInst, pTypeVeh, false, true);

        if(bUsePenalisation)
        {
            result *= m_dbPenalisation;
        }
    }
    return result;   
};
//================================================================
double    TuyauMicro::GetTempsParcourRealise
  //----------------------------------------------------------------
// Fonction  : Retourne le temps de parcours pour un vehicule donnÃ©e
//================================================================
(TypeVehicule * pTypeVehicule) const
{
    std::map<TypeVehicule*, double>::const_iterator it =  m_mapTempsParcoursRealise.find(pTypeVehicule);
    if(it != m_mapTempsParcoursRealise.end() )
    {
        return it->second;

    }
    return DBL_MAX;
  

}
//================================================================
    double  TuyauMicro::GetCoutAVide
//----------------------------------------------------------------
// Fonction  : Retourne le coÃ»t Ã  vide d'un tuyau micro
// Remarque  : Un tronÃ§on avec une vitesse rÃ©glementaire Ã  0 pour le
//             type de vÃ©hicule considÃ©rÃ© Ã  un coÃ»t infini afin 
//             d'empÃªcher la circulation des vÃ©hicules sur le tronÃ§on
//================================================================
(
	double			dbInst,
    TypeVehicule    *pTypeVeh,
    bool            bIgnoreVitCat,
    bool            bIncludeDownstreamPenalty
 )
{
    double result;
    if (IsInterdit( pTypeVeh, dbInst ) )    // CoÃ»t infini
		result = DBL_MAX;
    else if (m_dbVitCat != DBL_MAX && !bIgnoreVitCat)
    {
        // Utilisation de la speed catÃ©gorie si dÃ©finie
        result = GetLength() / m_dbVitCat;
    }
    else
    {
        // Calcul du temps de parcours en prenant en compte les exceptions Ã  la vitesse rÃ©glementaire
        result = 0;
        double dbDist;
        double dbCumulDist = 0;

        // 1 - dÃ©coupage du tronÃ§on en portions en fonction de la dÃ©finition des vitesses rÃ©glementaires
        set<double> boundaries;
        boundaries.insert(0.0);
        boundaries.insert(m_dbLongueur);
        VitRegDescription * pDescr = m_LstVitReg.GetVariationEx(dbInst);
        if(pDescr)
        {
            map<TypeVehicule*, map<int, vector<VitRegPortion> > >::iterator iter = pDescr->vectPortions.find(pTypeVeh);
            if(iter != pDescr->vectPortions.end())
            {
                map<int, vector<VitRegPortion> >::iterator voieIter;
                for(voieIter = iter->second.begin(); voieIter != iter->second.end(); voieIter++)
                {
                    for(size_t iPortion = 0; iPortion < voieIter->second.size(); iPortion++)
                    {
                        boundaries.insert(voieIter->second[iPortion].dbPosDebut);
                        if(voieIter->second[iPortion].dbPosFin < m_dbLongueur)
                        {
                            boundaries.insert(voieIter->second[iPortion].dbPosFin);
                        }
                    }
                }
            }
        }


        // 2 - rÃ©cup de la vitesse max sur chaque portion et cumul du coÃ»t
        set<double>::iterator iterPortion, iterStop;
        double dbVx;
        if (pTypeVeh)
        {
            dbVx = pTypeVeh->GetVx();
        }
        else
        {
            dbVx = 0;
            for (size_t iType = 0; iType < m_pReseau->m_LstTypesVehicule.size(); iType++)
            {
                dbVx = max(dbVx, m_pReseau->m_LstTypesVehicule[iType]->GetVx());
            }
        }
        for(iterPortion = boundaries.begin(); iterPortion != boundaries.end(); iterPortion++)
        {
            double dbStart = *iterPortion;
            iterStop = iterPortion;
            iterStop++;
            if(iterStop != boundaries.end())
            {
                double dbStop = *iterStop;
                double dbPos = (dbStop + dbStart)/2.0;
                double dbMaxSpeed = 0;
                for(int iVoie = 0; iVoie < getNb_voies(); iVoie++)
                {
                    // pour chaque voie ...
                    dbMaxSpeed = max(dbMaxSpeed, GetVitRegByTypeVeh(pTypeVeh, dbInst, dbPos, iVoie));
                }
                dbMaxSpeed = min(dbMaxSpeed, dbVx);
                dbDist = dbStop - dbStart;
                result += dbDist/dbMaxSpeed;
            }
            else
            {
                break;
            }
        }
    }

    // Pour les estimations de temps de parcours Ã  vide pour l'affectation, on inclut lorsque c'est possible
    // au temps de parcours Ã  vide des tronÃ§ons la pÃ©nalitÃ© liÃ©e Ã  l'insertion sur la connexion aval (feux, stop, etc...)
    if (bIncludeDownstreamPenalty && GetCnxAssAv())
    {
        result += GetCnxAssAv()->GetAdditionalPenaltyForUpstreamLink(this);

        if (GetCnxAssAv()->GetCtrlDeFeux() && m_pReseau->EstimateTrafficLightsWaitTime())
        {
            result += GetCnxAssAv()->GetCtrlDeFeux()->GetMeanWaitTime(this);
        }
    }

    return result;   
}

//================================================================
    Traversee* TuyauMicro::AddTraversee
//----------------------------------------------------------------
// Fonction  : Ajout une description de traversÃ©e Ã  la liste
// Remarque  : 
// Version du: 13/05/2008
// Historique: 13/05/2008 (C.BÃ©carie - Tinea)
//================================================================
(
    Tuyau *pT,                      // Tuyau prioritaire traversant le tuyau courant
    double dbPosNonPrioritaire,     // Position du point de conflit sur le tuyau courant
    double dbPosPrioritaire,         // Position du point de conflit sur le tuyau prioritaire
    int    nVoieTraversant /* = 0 */
)
{
    Traversee   *pTra;

    pTra = new Traversee;

    pTra->pTuyauTraversant = pT;
    pTra->dbPosition = dbPosNonPrioritaire;
    pTra->dbPositionTraversant = dbPosPrioritaire;
    pTra->dbInstLastTraversee = UNDEF_INSTANT;
    pTra->nVoieTraversant = nVoieTraversant;

    m_LstTraversee.push_back(pTra);

    return pTra;
}


//=================================================================
    double TuyauMicro::GetMaxRapVitSurQx
//----------------------------------------------------------------
// Fonction  : Fonction de calcul du maximum du rapport vitesse
//             libre sur dÃ©bit max avec la vitesse minorÃ©e par
//             les vitesse rÃ©glementaire
// Remarque  : 
// Version du: 19/05/2011
// Historique: 19/05/2011 (O.Tonck - IPSIS)
//             29/09/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//=================================================================
(    
	double dbInst
)
{
    // on renvoi plutot la vitesse reglementaire de base du troncon (qu'on suppose du coup
    // Ãªtre la vitesse max du troncon, variantes et exceptions comprises)
    return min(m_dbVitReg, m_pReseau->GetLstTypesVehicule()[0]->GetVx());

    /*
    double dbMax, dbTmp;
    double dbDebitMax;

    dbMax = 0;
    for(int i=0;i< (int)m_pReseau->GetLstTypesVehicule().size();i++)
    {
        dbTmp = min( GetVitRegByTypeVeh(m_pReseau->GetLstTypesVehicule()[i], dbInst, 0), m_pReseau->GetLstTypesVehicule()[i]->GetVx() );

        dbDebitMax = m_pReseau->GetLstTypesVehicule()[i]->GetW() * m_pReseau->GetLstTypesVehicule()[i]->GetKx() / ( (m_pReseau->GetLstTypesVehicule()[i]->GetW() / m_pReseau->GetLstTypesVehicule()[i]->GetVx() ) - 1);

        dbTmp = dbTmp / dbDebitMax;

        if( dbTmp > dbMax)
            dbMax = dbTmp;
    }
    return dbMax;*/
}

	// Retourne le nombre de vÃ©hicule ayant parcouru le tronÃ§on au cours de la pÃ©riode d'affectation courante
	int	TuyauMicro::GetNbVehPeriodeAffectation(TypeVehicule *pTypeVeh)
	{
		int nVeh = 0;
		std::map<int, pair<TypeVehicule*, pair<double, double>>>::iterator itVeh;		

		for( itVeh = m_mapVeh.begin(); itVeh != m_mapVeh.end(); itVeh++)
		{
			if( (*itVeh).second.first == pTypeVeh && (*itVeh).second.second.second > 0 )	// VÃ©hicule du bon type et sorti
				nVeh++;
		}

		return nVeh;
	}

	// Estimation du temps de parcours du tronÃ§on pour un type de vÃ©hicule spÃ©cifiÃ© pour la pÃ©riode d'affectation et stockage avec sauvegarde
	// du calcul prÃ©cÃ©dent
	void TuyauMicro::CalculTempsDeParcours( double dbInst, TypeVehicule *pTypeVeh, char cTypeEvent, double dbPeriode, char cTypeCalcul )
	{
        assert(!m_pReseau->IsSymuMasterMode()); // ne devrait pas Ãªtre appelÃ©e si utilisation avec SymuMaster

		double dbTps;
        double dbTpsCout;
		double dbInstEntree = 0, dbInstSortie = 0;
		std::map<int, pair<TypeVehicule*, pair<double, double>>>::iterator itVeh;		
		double dbTPpred;
		double dbTPpredL;

        dbTpsCout = ComputeCost( dbInst, pTypeVeh, false);

		// Calcul Ã  vide (uniquement pour la premiÃ¨re pÃ©riode)
		if( dbInst == 0 )
        {
            dbTps = dbTpsCout;

			if( this->m_mapTempsParcours.find( pTypeVeh ) != m_mapTempsParcours.end() )
				m_mapTempsParcours.find( pTypeVeh )->second = dbTps;
			else
				m_mapTempsParcours.insert( make_pair(pTypeVeh, dbTps) );

			//Trace
			//*(Reseau::m_pFicSimulation) << "Temps de parcours de " << this->GetLabel() << " : " << m_mapTempsParcours.find( pTypeVeh )->second << endl;
						
			m_mapTempsParcoursPrecedent.insert( make_pair(pTypeVeh, m_mapTempsParcours.find(pTypeVeh)->second) );
			m_mapTempsParcoursRealise.insert( make_pair(pTypeVeh, m_mapTempsParcours.find(pTypeVeh)->second) );	

            m_bHasRealTravelTime = false;

			return;
		}

		// Sauvegarde des temps de parcours utilisÃ© de la prÃ©cÃ©dente pÃ©riode d'affectation	

        // Cas SymuMaster qui plante ici car n'appelle pas cette mÃ©thode Ã  t = 0 (pas besoin)
        std::map<TypeVehicule*, double>::const_iterator iterMapTpsParcours = m_mapTempsParcours.find(pTypeVeh);
        if (iterMapTpsParcours != m_mapTempsParcours.find(pTypeVeh))
        {
            m_mapTempsParcoursPrecedent.find(pTypeVeh)->second = iterMapTpsParcours->second;
        }

		// -- Temps de parcours rÃ©alisÃ© -- 
		// Estimation Ã  partir de la liste de vÃ©hicule sortant du tronÃ§on gÃ©nÃ©rÃ©e durant la pÃ©riode d'affectation prÃ©cÃ©dente		
		dbTps = 0;
		int nVeh = 0;
		for( itVeh = m_mapVeh.begin(); itVeh != m_mapVeh.end(); itVeh++)
		{
			if( (*itVeh).second.first == pTypeVeh && (*itVeh).second.second.second > 0 )	// VÃ©hicule du bon type et sorti
			{								
				dbInstEntree = (*itVeh).second.second.first;
				dbInstSortie = (*itVeh).second.second.second;

				if( dbInstSortie >= dbInst - dbPeriode )
				{
					dbTps += (*itVeh).second.second.second - (*itVeh).second.second.first;		// temps de parcours du tronÃ§on pour le vÃ©hicule
					nVeh++;
				}
			}
		}

		if(nVeh==0)	// Aucun vÃ©hicule n'est sorti durant la prÃ©cÃ©dente pÃ©riode d'affectation
		{
			// Existe t'il un vÃ©hicule entrÃ© et non encore sorti ?
			for( itVeh = this->m_mapVeh.begin(); itVeh != m_mapVeh.end(); itVeh++)
				if( (*itVeh).second.first == pTypeVeh )
				{
					dbTps += dbInst - (*itVeh).second.second.first;		// temps de parcours du tronÃ§on pour le vÃ©hicule
					nVeh++;
				}

			if(nVeh==0)
			{
				// Utilisation du dernier vÃ©hicule sorti pour l'estimation - Non, modif du 25/07/12
				//if( m_mapDernierVehSortant.find( pTypeVeh ) != m_mapDernierVehSortant.end() )
				//{
					//dbTps += m_mapDernierVehSortant.find( pTypeVeh )->second.second - m_mapDernierVehSortant.find( pTypeVeh )->second.first;
					//nVeh++;
					//dbTps = 0; 
				//}
				dbTps = 0; // Aucun vÃ©hicule sur le lien donc temps de parcours nul
			}
		}

		if( nVeh > 0)
        {
            m_mapTempsParcoursRealise[pTypeVeh] = max(dbTps / nVeh, this->GetCoutAVide(dbInst, pTypeVeh, false, false));
            m_bHasRealTravelTime = true;
        }
		else
        {
            m_mapTempsParcoursRealise[pTypeVeh] = this->GetCoutAVide(dbInst, pTypeVeh, false, true);
            m_bHasRealTravelTime = false;
        }

		if( cTypeCalcul == 'R' )
			m_mapTempsParcours[pTypeVeh] = m_mapTempsParcoursRealise.find( pTypeVeh )->second;	// Le temps de parcours rÃ©alisÃ© est le temps de parcours Ã  prendre en compte pour le calcul d'affecation
			

		// -- Temps de parcours pÃ©dictif formule Nico --
		dbTPpred = 0;
		dbTPpredL = 0;
		if( cTypeCalcul=='P' || cTypeCalcul=='Q')
		{			

			// Calcul du temps de parcours du dernier vÃ©hicule sorti
			double dbTPLastVeh = -1;
			std::map<int, pair<TypeVehicule*, pair<double, double>>>::reverse_iterator ritVeh;		
			
			for( ritVeh = this->m_mapVeh.rbegin(); ritVeh != m_mapVeh.rend(); ritVeh++)
			{
				if( (*ritVeh).second.first == pTypeVeh && (*ritVeh).second.second.second > 0)
				{			
					dbTPLastVeh = (*ritVeh).second.second.second - (*ritVeh).second.second.first;
					break;
				}				
			}

			// Aucun vÃ©hicule sorti durant la pÃ©riode
			if( dbTPLastVeh == -1)
			{
		        dbTPLastVeh = dbTpsCout;
			}

			// Calcul du nombre de vÃ©hicules entrÃ©s Ã  l'instant courant
			int nNbCumEntre = 0;
			if( m_mapNbVehEntre.find( pTypeVeh )!= m_mapNbVehEntre.end() )
				nNbCumEntre = m_mapNbVehEntre.find( pTypeVeh )->second;		

			// Calcul du nombre de vÃ©hicules sortis Ã  l'instant courant
			int nNbCumSorti = 0;
			if( m_mapNbVehSorti.find( pTypeVeh )!= m_mapNbVehSorti.end() )
				nNbCumSorti = m_mapNbVehSorti.find( pTypeVeh )->second;

			// Calcul du nombre de vÃ©hicules sortis Ã  la pÃ©riode prÃ©cÃ©dente
			int nNbCumSortiPrev = 0;
			if( m_mapNbVehSortiPrev.find( pTypeVeh )!= m_mapNbVehSortiPrev.end() )
				nNbCumSortiPrev = m_mapNbVehSortiPrev.find( pTypeVeh )->second;

			// Calcul du nombre de vÃ©hicules entrÃ©s Ã  la pÃ©riode prÃ©cÃ©dente
			int nNbCumEntrePrev = 0;
			if( m_mapNbVehEntrePrev.find( pTypeVeh )!= m_mapNbVehEntrePrev.end() )
				nNbCumEntrePrev = m_mapNbVehEntrePrev.find( pTypeVeh )->second;

			// Calcul du temps de parcours prÃ©dictif par la formule de Nico
			if( abs(nNbCumSorti - nNbCumSortiPrev) != 0 ) 
			{
				dbTPpred = 0.5 * dbTPLastVeh;
				dbTPpred += 0.5 * dbPeriode * ( (double) ( nNbCumEntre - nNbCumSorti ) / (double) ( nNbCumSorti - nNbCumSortiPrev ) );				
			}
			else	// Aucun vÃ©hicule sorti pendant la pÃ©riode considÃ©rÃ©e 
				if( nNbCumEntrePrev ==  nNbCumEntre )	// Aucun vÃ©hicule entrÃ© -> temps de parcours Ã  vide
					dbTPpred = dbTpsCout;
				else
					dbTPpred = DBL_MAX;	// Des vÃ©hicules sont entrÃ©s -> temps de parcours = infini

			dbTPpred = max( dbTPpred, dbTpsCout );
			m_mapTempsParcours.find( pTypeVeh )->second = dbTPpred;

			// Calcul du temps de parcours prÃ©dictif par la formule de Ludo
			dbTPpredL = 0;
 
			if( abs(nNbCumEntre - nNbCumEntrePrev) != 0 ) 
			{
				dbTPpredL = dbPeriode + dbPeriode / (nNbCumEntre - nNbCumEntrePrev) * ( nNbCumSorti - nNbCumEntre + (nNbCumSorti - nNbCumSortiPrev) + ( (nNbCumEntre - nNbCumEntrePrev) - (nNbCumSorti - nNbCumSortiPrev))/dbPeriode );
			}
			else	// Aucun vÃ©hicule sorti pendant la pÃ©riode considÃ©rÃ©e 
				if( nNbCumEntrePrev ==  nNbCumEntre )	// Aucun vÃ©hicule entrÃ© -> temps de parcours Ã  vide
					dbTPpredL = dbTpsCout;
				else
					dbTPpredL = DBL_MAX;	// Des vÃ©hicules sont entrÃ©s -> temps de parcours = infini

			dbTPpredL = max( dbTPpredL, dbTpsCout );
		}

		//Trace
		//if( m_pReseau->IsDebugAffectation() )
		//{
		//	*(Reseau::m_pFicSimulation) << "Temps de parcours rÃ©alisÃ© lors de la prÃ©cÃ©dente pÃ©riode " << pTypeVeh->GetLibelle() << " de " << this->GetLabel() << " : " << m_mapTempsParcoursRealise.find( pTypeVeh )->second << " Nb vehicule sorti considÃ©rÃ©: " << nVeh << " - " << dbTps/nVeh << " - " << dbTpsVitReg << endl;
		//	*(Reseau::m_pFicSimulation) << "Temps de parcours prÃ©dictif Nico "						  << pTypeVeh->GetLibelle() << " de " << this->GetLabel() << " : " << dbTPpred << endl;
		//	*(Reseau::m_pFicSimulation) << "Temps de parcours prÃ©dictif Ludo "						  << pTypeVeh->GetLibelle() << " de " << this->GetLabel() << " : " << dbTPpredL << endl;
		//}

		// MAJ du dernier vÃ©hicule sortant
		if( cTypeCalcul=='P' || cTypeCalcul=='Q')
		{
			if( dbInstSortie > 0)
			{
				if( m_mapDernierVehSortant.find( pTypeVeh ) != m_mapDernierVehSortant.end() )
				{
					m_mapDernierVehSortant.find( pTypeVeh )->second.first = dbInstEntree;
					m_mapDernierVehSortant.find( pTypeVeh )->second.second = dbInstSortie;
				}
				else
				{
					m_mapDernierVehSortant.insert( make_pair(pTypeVeh, make_pair(dbInstEntree, dbInstSortie)));
				}	
			}
		}				
				
	}

#ifdef USE_SYMUCORE
void TuyauMicro::CalculTempsDeParcours(double dbInstFinPeriode, SymuCore::MacroType * pMacroType, double dbPeriodDuration)
{
    // rmq. : on prend comme rÃ©fÃ©rence le premier type de vÃ©hicule dÃ©fini dans le macrotype.
    TypeVehicule * pTypeVeh = m_pReseau->GetVehiculeTypeFromMacro(pMacroType);
    double dbTpsVitReg = this->GetCoutAVide(dbInstFinPeriode, pTypeVeh, false, true);
    bool bIsForbiddenLink = dbTpsVitReg >= DBL_MAX || !pTypeVeh;
    if (bIsForbiddenLink)
    {
        dbTpsVitReg = std::numeric_limits<double>::infinity();
    }


    // Calcul de la concentration et du nombre moyen de vÃ©hicules sur la cellule :
    double dbTravelledTimeForMacroType = 0;
    double dbTravelledDistanceForMacroType = 0;
    double dbTotalTravelledTime = 0;
    double dbTotalTravelledDistance = 0;
    const std::map<TypeVehicule*, double> & mapTT = m_pEdieSensor->GetData().dbTotalTravelledTime.back();
    for (std::map<TypeVehicule*, double>::const_iterator iterTT = mapTT.begin(); iterTT != mapTT.end(); ++iterTT)
    {
        if (pMacroType->hasVehicleType(iterTT->first->GetLabel()))
        {
            dbTravelledTimeForMacroType += iterTT->second;
        }
        dbTotalTravelledTime += iterTT->second;
    }
    const std::map<TypeVehicule*, double> & mapTD = m_pEdieSensor->GetData().dbTotalTravelledDistance.back();
    for (std::map<TypeVehicule*, double>::const_iterator iterTD = mapTD.begin(); iterTD != mapTD.end(); ++iterTD)
    {
        if (pMacroType->hasVehicleType(iterTD->first->GetLabel()))
        {
            dbTravelledDistanceForMacroType += iterTD->second;
        }
        dbTotalTravelledDistance += iterTD->second;
    }
    double dbMeanTotalVehicleNb = dbTotalTravelledTime / dbPeriodDuration;
    double dbMeanTotalConcentration = dbMeanTotalVehicleNb / (GetLength() * getNbVoiesDis());
    double dbMeanVehicleNbForMacroType = dbTravelledTimeForMacroType / dbPeriodDuration;
    double dbMeanConcentrationForMacroType = dbMeanVehicleNbForMacroType / (GetLength() * getNbVoiesDis());

    double & TTForMacroType = m_mapTTByMacroType[pMacroType];
    double & dbMarginalForMacroType = m_mapMarginalByMacroType[pMacroType];

    if (m_pReseau->UseSpatialTTMethod())
    {
        // Mode spatial

        // Calcul du temps de parcours
        TTForMacroType = SymuCore::TravelTimeUtils::ComputeSpatialTravelTime(dbTravelledTimeForMacroType, dbTravelledDistanceForMacroType, GetLength(), dbMeanTotalConcentration, dbTpsVitReg, bIsForbiddenLink, m_pReseau->GetConcentrationRatioForFreeFlowCriterion(), m_pReseau->GetMainKx());
		//m_pReseau->log() << dbInstFinPeriode << " TT " << GetLabel() << " : " << TTForMacroType << endl;

        // Calcul du marginal spatialisÃ© : dans ce mode, on stocke le nombre moyen de vÃ©hicules et le temps de parcours, et on fait le calcul Ã  la fin de la pÃ©riode de prÃ©diction
        m_dbMeanNbVehiclesForTravelTimeUpdatePeriod[pMacroType] = dbMeanVehicleNbForMacroType;
        m_dbMeanNbVehiclesForTravelTimeUpdatePeriod[NULL] = dbMeanTotalVehicleNb;
        m_dbTTForAllMacroTypes = SymuCore::TravelTimeUtils::ComputeSpatialTravelTime(dbTotalTravelledTime, dbTotalTravelledDistance, GetLength(), dbMeanTotalConcentration, dbTpsVitReg, bIsForbiddenLink, m_pReseau->GetConcentrationRatioForFreeFlowCriterion(), m_pReseau->GetMainKx());
    }
    else
    {
        // Mode sortie de vÃ©hicules

        std::map<int, pair<TypeVehicule*, pair<double, double>>>::iterator itVeh;
        std::map<double, std::map<int, std::pair<bool, std::pair<double, double> > > > mapUsefuleVehicles;
        double dbInstSortie;
        for (itVeh = m_mapVeh.begin(); itVeh != m_mapVeh.end(); ++itVeh)
        {
            dbInstSortie = (*itVeh).second.second.second;
            if (dbInstSortie > 0)	// VÃ©hicule du bon macrotype et sorti
            {
                mapUsefuleVehicles[dbInstSortie][itVeh->first] = std::make_pair(pMacroType->hasVehicleType(itVeh->second.first->GetLabel()), std::make_pair(dbInstSortie - (*itVeh).second.second.first, 1.0)); // rmq. on se fiche du ratio 1.0 ici car longueur identique pour tous les vÃ©hicules
            }
        }

        double dbITT;
        std::vector<std::pair<std::pair<double, double>, double> > exitInstantAndTravelTimesWithLengthsForMacroType, exitInstantAndTravelTimesWithLengthsForAllMacroTypes;
        std::vector<int> vehicleIDsToClean;
        SymuCore::TravelTimeUtils::SelectVehicleForTravelTimes(mapUsefuleVehicles, dbInstFinPeriode, dbPeriodDuration, m_pReseau->GetNbPeriodsForTravelTimes(), m_pReseau->GetNbVehiclesForTravelTimes(), m_pReseau->GetMarginalsDeltaN(), m_pReseau->UseMarginalsMeanMethod(), m_pReseau->UseMarginalsMedianMethod(), dbITT, exitInstantAndTravelTimesWithLengthsForMacroType, exitInstantAndTravelTimesWithLengthsForAllMacroTypes, vehicleIDsToClean);

        // Nettoyage des vÃ©hicules devenus trop anciens :
        for (size_t iVeh = 0; iVeh < vehicleIDsToClean.size(); iVeh++)
        {
            m_mapVeh.erase(vehicleIDsToClean[iVeh]);
        }

        TTForMacroType = SymuCore::TravelTimeUtils::ComputeTravelTime(exitInstantAndTravelTimesWithLengthsForMacroType, dbMeanTotalConcentration, dbTpsVitReg, bIsForbiddenLink, m_pReseau->GetNbVehiclesForTravelTimes(), m_pReseau->GetConcentrationRatioForFreeFlowCriterion(), m_pReseau->GetMainKx());

        // calcul du marginal
        // Pour les marginals, on prend tous les derniÃ¨rs vÃ©hicules sortis du tronÃ§on quelquesoit leur macrotype : 
        dbMarginalForMacroType = SymuCore::TravelTimeUtils::ComputeTravelTime(exitInstantAndTravelTimesWithLengthsForAllMacroTypes, dbMeanTotalConcentration, dbTpsVitReg, bIsForbiddenLink, m_pReseau->GetNbVehiclesForTravelTimes(), m_pReseau->GetConcentrationRatioForFreeFlowCriterion(), m_pReseau->GetMainKx());
        // Le dbITT est aussi pour tous les macrotypes : seul le nombre de vÃ©hicules sur le tronÃ§on est pris pour le macro-type considÃ©rÃ©.
        dbMarginalForMacroType += dbITT * dbMeanVehicleNbForMacroType;
    }
}

double TuyauMicro::GetTravelTime(SymuCore::MacroType * pMacroType)
{
    double result;

    TypeVehicule * pTypeVeh = m_pReseau->GetVehiculeTypeFromMacro(pMacroType);
    double dbTpsVitReg = this->GetCoutAVide(m_pReseau->m_dbInstSimu, pTypeVeh, false, true);
    bool bIsForbiddenLink = dbTpsVitReg >= DBL_MAX || !pTypeVeh;

    std::map<SymuCore::MacroType*, double>::const_iterator iter = m_mapTTByMacroType.find(pMacroType);

    if (iter == m_mapTTByMacroType.end())
    {
        if (bIsForbiddenLink)
        {
            result = std::numeric_limits<double>::infinity();
        }
        else
        {
            result = dbTpsVitReg;
        }
    }
    else
    {
        // cas oÃ¹ on a dÃ©jÃ  fait le calcul :
        result = iter->second;
    }

    // Application de la vitesse minimum le cas Ã©chÃ©ant :
    if (m_pReseau->GetMinSpeedForTravelTime() > 0 && !bIsForbiddenLink)
    {
        result = std::min<double>(result, GetLength() / m_pReseau->GetMinSpeedForTravelTime());
    }

    return result;
}

double TuyauMicro::GetMarginal(SymuCore::MacroType * pMacroType)
{
    double result;
    std::map<SymuCore::MacroType*, double>::const_iterator iter = m_mapMarginalByMacroType.find(pMacroType);

    if (iter == m_mapMarginalByMacroType.end())
    {
        // Cas Ã  vide : c'est le temps de parcours
        result = GetTravelTime(pMacroType);
    }
    else
    {
        result = iter->second;
    }

    // Application du marginals minimum
    TypeVehicule * pTypeVeh = m_pReseau->GetVehiculeTypeFromMacro(pMacroType);
    double dbTpsVitReg = this->GetCoutAVide(m_pReseau->m_dbInstSimu, pTypeVeh, false, true);
    bool bIsForbiddenLink = dbTpsVitReg >= DBL_MAX || !pTypeVeh;
    if (!bIsForbiddenLink)
    {
        result = std::min<double>(result,m_pReseau->GetMaxMarginalsValue());
    }
    result = std::max<double>(0, result); // Pas de marginals nÃ©gatifs

    return result;
}

double TuyauMicro::GetMeanNbVehiclesForTravelTimeUpdatePeriod(SymuCore::MacroType * pMacroType)
{
    std::map<SymuCore::MacroType*, double>::const_iterator iter = m_dbMeanNbVehiclesForTravelTimeUpdatePeriod.find(pMacroType);
    if (iter != m_dbMeanNbVehiclesForTravelTimeUpdatePeriod.end())
    {
        return iter->second;
    }
    else
    {
        // pas encore calculÃ© : 0.
        return 0;
    }
}
#endif // USE_SYMUCORE

// MAJ des variables utiles au calcul des temps de parcours pour une pÃ©riode d'affectation
void TuyauMicro::FinCalculTempsDeParcours( double dbInst, TypeVehicule *pTypeVeh, char cTypeEvent, double dbPeriode, char cTypeCalcul )
{
    assert(!m_pReseau->IsSymuMasterMode()); // ne devrait pas Ãªtre appelÃ©e si utilisation avec SymuMaster

	std::map<int, pair<TypeVehicule*, pair<double, double>>>::iterator itVeh;		

	// Suppression des vÃ©hicules traitÃ©s (sorti du tronÃ§on au cours de la pÃ©riode prÃ©cÃ©dente)et MAJ du nombre cumulÃ© de vÃ©hicule de la pÃ©riode prÃ©cÃ©dente
	//if( cTypeCalcul=='P' || cTypeCalcul=='Q')
	{
		for( itVeh = this->m_mapVeh.begin(); itVeh != m_mapVeh.end(); )
		{
			if( (*itVeh).second.first == pTypeVeh && (*itVeh).second.second.second > 0 )
			{
				m_mapVeh.erase( itVeh++ );		
			}
			else
				itVeh++;
		}			
	}

    if(m_mapNbVehSortiPrev.find(pTypeVeh) != m_mapNbVehSortiPrev.end())
    {
	    m_mapNbVehSortiPrev.find(pTypeVeh)->second = m_mapNbVehSorti.find(pTypeVeh)->second;
    }
    if(m_mapNbVehEntrePrev.find(pTypeVeh) != m_mapNbVehEntrePrev.end())
    {
	    m_mapNbVehEntrePrev.find(pTypeVeh)->second = m_mapNbVehEntre.find(pTypeVeh)->second;
    }
}


//================================================================
    void TuyauMicro::DiscretisationSirane
//----------------------------------------------------------------
// Fonction  : Segmentation du tuyau en cellules pour production
// des sortie nÃ©cessaires pour Sirane (pour CityDyne)
// Version du: 05/05/2012
// Historique: 05/05/2012 (O.Tonck - IPSIS)
//             CrÃ©ation
//================================================================
(
)
{
    // mÃ©nage des cellules existantes
    SupprimeCellSirane();

    // Nombre de cellules par tronÃ§on :
    int nNbCell = m_pReseau->GetNbCellSirane();
    // Longueur minimum d'une cellule
    double dbMinLongueurCell = m_pReseau->GetMinLongueurCellSirane();

    // on adapte le nombre de cellules afin de respecter la longueur de cellule minimum
    if(GetLength()/nNbCell < dbMinLongueurCell)
    {
        nNbCell = max(1, (int)floor(GetLength() / dbMinLongueurCell));
    }

    // CrÃ©ation des cellules
    double dbLongueurCell = GetLength()/nNbCell;
    for(int i = 0; i < nNbCell; i++)
    {
        Segment * pCell = new Segment();
        pCell->SetReseau(m_pReseau);
        std::ostringstream oss;
        oss << GetLabel() << "_CEL" << (i+1);
        pCell->SetLabel(oss.str());

        // Calcul des coordonnÃ©es du point amont de la cellule
        Point pointAmont;
        if(i == 0)
        {
            pointAmont = *GetExtAmont();
        }
        else
        {
            CalcCoordFromPos(this, i*dbLongueurCell, pointAmont);
        }
        pCell->SetExtAmont(pointAmont);

        // Calcul des coordonnÃ©es du point aval de la cellule
        Point pointAval;
        if(i == nNbCell-1)
        {
            pointAval = *GetExtAval();
        }
        else
        {
            CalcCoordFromPos(this, (i+1)*dbLongueurCell, pointAval);
        }
        pCell->SetExtAval(pointAval);

        // Calcul des points internes de la cellule
        double dbAbscisseCurviligne = 0;
        for(size_t pointInterneIdx = 0; pointInterneIdx < m_LstPtsInternes.size(); pointInterneIdx++)
        {
            if(pointInterneIdx == 0)
            {
                dbAbscisseCurviligne += sqrt((m_LstPtsInternes[pointInterneIdx]->dbX - GetExtAmont()->dbX)*(m_LstPtsInternes[pointInterneIdx]->dbX - GetExtAmont()->dbX)
                    + (m_LstPtsInternes[pointInterneIdx]->dbY - GetExtAmont()->dbY)*(m_LstPtsInternes[pointInterneIdx]->dbY - GetExtAmont()->dbY)
                    + (m_LstPtsInternes[pointInterneIdx]->dbZ - GetExtAmont()->dbZ)*(m_LstPtsInternes[pointInterneIdx]->dbZ - GetExtAmont()->dbZ));
            }
            else
            {
                dbAbscisseCurviligne += sqrt((m_LstPtsInternes[pointInterneIdx]->dbX - m_LstPtsInternes[pointInterneIdx-1]->dbX)*(m_LstPtsInternes[pointInterneIdx]->dbX - m_LstPtsInternes[pointInterneIdx-1]->dbX)
                    + (m_LstPtsInternes[pointInterneIdx]->dbY - m_LstPtsInternes[pointInterneIdx-1]->dbY)*(m_LstPtsInternes[pointInterneIdx]->dbY - m_LstPtsInternes[pointInterneIdx-1]->dbY)
                    + (m_LstPtsInternes[pointInterneIdx]->dbZ - m_LstPtsInternes[pointInterneIdx-1]->dbZ)*(m_LstPtsInternes[pointInterneIdx]->dbZ - m_LstPtsInternes[pointInterneIdx-1]->dbZ));
            }

            if(dbAbscisseCurviligne > i*dbLongueurCell && dbAbscisseCurviligne < (i+1)*dbLongueurCell)
            {
                pCell->AddPtInterne(m_LstPtsInternes[pointInterneIdx]->dbX, m_LstPtsInternes[pointInterneIdx]->dbY, m_LstPtsInternes[pointInterneIdx]->dbZ);
            }
            
        }
           
        // Calcul de la longueur de la cellule
        pCell->CalculeLongueur(0);

        // mode de sortie avec barycentres
        if(m_pReseau->GetExtensionBarycentresSirane())
        {
            // cas de la brique amont
            if(GetBriqueAmont())
            {
                Point * ptt = new Point;
                ptt->dbX = pCell->GetExtAmont()->dbX;
                ptt->dbY = pCell->GetExtAmont()->dbY;
                ptt->dbZ = pCell->GetExtAmont()->dbZ;
                pCell->GetLstPtsInternes().push_front(ptt);
                ptt = GetBriqueAmont()->CalculBarycentre();
                pCell->SetExtAmont(*ptt);
                delete ptt;
            }
            // cas de la brique aval
            if(GetBriqueAval())
            {
                Point * ptt = new Point;
                ptt->dbX = pCell->GetExtAval()->dbX;
                ptt->dbY = pCell->GetExtAval()->dbY;
                ptt->dbZ = pCell->GetExtAval()->dbZ;
                pCell->GetLstPtsInternes().push_back(ptt);
                ptt = GetBriqueAval()->CalculBarycentre();
                pCell->SetExtAval(*ptt);
                delete ptt;
            }
        }

        m_LstCellSirane.push_back(pCell);
    }


}


//================================================================
    void TuyauMicro::SupprimeCellSirane
//----------------------------------------------------------------
// Fonction  : Nettoie les cellules Sirane
// Version du: 05/05/2012
// Historique: 05/05/2012 (O.Tonck - IPSIS)
//             CrÃ©ation
//================================================================
(
)
{
    for(size_t i = 0; i < m_LstCellSirane.size(); i++)
    {
        delete m_LstCellSirane[i];
    }
    m_LstCellSirane.clear();
}
  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void ZoneDepassementInterdit::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void ZoneDepassementInterdit::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void ZoneDepassementInterdit::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(dbPosDebut);
    ar & BOOST_SERIALIZATION_NVP(dbPosFin);
}

template void ZoneZLevel::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void ZoneZLevel::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void ZoneZLevel::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(dbPosDebut);
    ar & BOOST_SERIALIZATION_NVP(dbPosFin);
    ar & BOOST_SERIALIZATION_NVP(nZLevel);
}

template void VitRegDescription::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void VitRegDescription::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void VitRegDescription::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(vectPortions);
}

template void Tuyau::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void Tuyau::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void Tuyau::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Troncon);

    ar & BOOST_SERIALIZATION_NVP(m_nResolution);
    ar & BOOST_SERIALIZATION_NVP(Type_amont);
    ar & BOOST_SERIALIZATION_NVP(Type_aval);
    ar & BOOST_SERIALIZATION_NVP(p_aval);
    ar & BOOST_SERIALIZATION_NVP(p_amont);

    ar & BOOST_SERIALIZATION_NVP(m_pBriqueAmont);
    ar & BOOST_SERIALIZATION_NVP(m_pBriqueAval);

    ar & BOOST_SERIALIZATION_NVP(m_pCnxAssAm);
    ar & BOOST_SERIALIZATION_NVP(m_pCnxAssAv);

    ar & BOOST_SERIALIZATION_NVP(m_nNbVoies);
    ar & BOOST_SERIALIZATION_NVP(m_nNbVoiesDis);

    ar & BOOST_SERIALIZATION_NVP(m_largeursVoie);
    ar & BOOST_SERIALIZATION_NVP(m_dqRepartitionEntreeVoie);
    ar & BOOST_SERIALIZATION_NVP(m_pLstVoie);
    ar & BOOST_SERIALIZATION_NVP(m_pBriqueParente);
    ar & BOOST_SERIALIZATION_NVP(m_LstCapteursConvergents);
    ar & BOOST_SERIALIZATION_NVP(m_LstRelatedSensorsBySensorsManagers);
    ar & BOOST_SERIALIZATION_NVP(m_nNbPasTempsHist);
    ar & BOOST_SERIALIZATION_NVP(m_bExpGrTaboo);

    ar & BOOST_SERIALIZATION_NVP(m_pTuyauOppose);

    ar & BOOST_SERIALIZATION_NVP(m_dbCapaciteStationnement);
    ar & BOOST_SERIALIZATION_NVP(m_dbStockStationnement);

    ar & BOOST_SERIALIZATION_NVP(m_LstZonesZLevel);

    ar & BOOST_SERIALIZATION_NVP(m_dbPenalisation);
    ar & BOOST_SERIALIZATION_NVP(m_iFunctionalClass);
    ar & BOOST_SERIALIZATION_NVP(m_bHasRealTravelTime);

    ar & BOOST_SERIALIZATION_NVP(m_mapVoiesReservees);
    ar & BOOST_SERIALIZATION_NVP(m_mapVoiesInterdites);
    ar & BOOST_SERIALIZATION_NVP(m_LstVitReg);
}

template void TuyauMacro::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void TuyauMacro::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void TuyauMacro::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Tuyau);

    ar & BOOST_SERIALIZATION_NVP(m_bCondVitAmont);
    ar & BOOST_SERIALIZATION_NVP(m_bCondVitAval);

    ar & BOOST_SERIALIZATION_NVP(m_dbCoeffAmont1);
    ar & BOOST_SERIALIZATION_NVP(m_dbCoeffAmont2);

    ar & BOOST_SERIALIZATION_NVP(m_dbCoeffAval1);
    ar & BOOST_SERIALIZATION_NVP(m_dbCoeffAval2);

    ar & BOOST_SERIALIZATION_NVP(m_pDF);
}

template void structVehicule::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void structVehicule::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void structVehicule::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(dbPosition);
    ar & BOOST_SERIALIZATION_NVP(dbPositionAnt);
    ar & BOOST_SERIALIZATION_NVP(dbVitesse);
    ar & BOOST_SERIALIZATION_NVP(dbVitesseAnt);
    ar & BOOST_SERIALIZATION_NVP(dbVitesseAntAnt);
    ar & BOOST_SERIALIZATION_NVP(nId);
    ar & BOOST_SERIALIZATION_NVP(VehAutonome);
    ar & BOOST_SERIALIZATION_NVP(VehAccelere);
    ar & BOOST_SERIALIZATION_NVP(dbDebitCreation);
}

template void Traversee::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void Traversee::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void Traversee::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(pTuyauTraversant);
    ar & BOOST_SERIALIZATION_NVP(dbPosition);
    ar & BOOST_SERIALIZATION_NVP(dbPositionTraversant);
    ar & BOOST_SERIALIZATION_NVP(dbInstLastTraversee);
    ar & BOOST_SERIALIZATION_NVP(nVoieTraversant);
}

template void TuyauMicro::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void TuyauMicro::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void TuyauMicro::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Tuyau);

    ar & BOOST_SERIALIZATION_NVP(m_nNbCellAcoustique);
    ar & BOOST_SERIALIZATION_NVP(m_bAgressif);

    ar & BOOST_SERIALIZATION_NVP(m_dbCoutPenalise);

    ar & BOOST_SERIALIZATION_NVP(m_LstTraversee);
    ar & BOOST_SERIALIZATION_NVP(m_bChtVoieVersDroiteAutorise);

    ar & BOOST_SERIALIZATION_NVP(m_mapTempsParcours);
    ar & BOOST_SERIALIZATION_NVP(m_mapTempsParcoursPrecedent);
    ar & BOOST_SERIALIZATION_NVP(m_mapTempsParcoursRealise);

    ar & BOOST_SERIALIZATION_NVP(m_mapVeh);

    ar & BOOST_SERIALIZATION_NVP(m_mapDernierVehSortant);

    ar & BOOST_SERIALIZATION_NVP(m_mapNbVehEntre);
    ar & BOOST_SERIALIZATION_NVP(m_mapNbVehSorti);
    ar & BOOST_SERIALIZATION_NVP(m_mapNbVehSortiPrev);
    ar & BOOST_SERIALIZATION_NVP(m_mapNbVehEntrePrev);

    ar & BOOST_SERIALIZATION_NVP(m_LstCellSirane);
}
