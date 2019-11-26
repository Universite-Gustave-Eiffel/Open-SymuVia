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
#ifndef toolsH
#define toolsH

#include "symUtil.h"
#include "TimeUtil.h"

#include <xercesc/util/XercesVersion.hpp>

#include <boost/archive/xml_woarchive.hpp>
#include <boost/archive/xml_wiarchive.hpp>

#include "SerializeUtil.h"

#include <deque>
#include <list>
#include <vector>
#include <sstream>
#include <map>

class Entree;
class Sortie;
class Reseau;
class Troncon;
class Tuyau;
class TuyauMicro;
class TypeVehicule;
class Connexion;
class Voie;
class Logger;

namespace XERCES_CPP_NAMESPACE {
    class DOMNode;
}

template<class T>
class LessPtr 
{
public:
	bool operator()(const T *a, const T *b) const
	{	
	  return (a->GetLabel() < b->GetLabel());	  	
	}
};

template<class T>
class LessOriginPtr 
{
public:
	bool operator()(const T *a, const T *b) const
	{	
	  return (a->GetOutputID() < b->GetOutputID());	  	
	}
};

template<class T>
class LessConnexionPtr 
{
public:
	bool operator()(const T *a, const T *b) const
	{	
	  return (a->GetID() < b->GetID());	  	
	}
};

template<class T>
class LessVehiculePtr 
{
public:
	bool operator()(const boost::shared_ptr<T> a, const boost::shared_ptr<T> b) const
	{	
	  return (a->GetID() < b->GetID());	  	
	}
};

template<class T>
class LessPairPtrIDRef
{
public:
	bool operator()(const T &a, const T &b) const
	{
		if (a.first < b.first)
		{
			return true;
		}
		else if (a.first == b.first)
		{
			if (a.second != NULL && b.second != NULL)
			{
				return a.second->GetID() < b.second->GetID();
			}
			else if (a.second == NULL && b.second != NULL)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
};

// Foncteur servant Ã  libÃ©rer un pointeur - applicable Ã  n'importe quel type
struct Delete 
{ 
   template <class T> void operator ()(T*& p) const 
   { 
      delete p;
      p = NULL;
   } 
}; 

// Structure de dÃ©finition d'une plage temporelle
class PlageTemporelle {
public:

    double m_Debut;   // dÃ©but (en secondes) de la plage temporelle
    double m_Fin;     // fin (en secondes) de la plage temporelle
    std::string m_ID; // Identifiant de la plage temporelle

    static PlageTemporelle * Set(double dStart, double dEnd , std::vector<PlageTemporelle * > & plages, double dDebutSimu )
    {
        // look for existing TimeRange with dStart and dEnd
        std::vector<PlageTemporelle *>::iterator it = plages.begin();
       
        while ( it != plages.end() )
        {
            if( (*it)->m_Debut == dStart- dDebutSimu && (*it)->m_Fin == dEnd -dDebutSimu)
            { 
           
                return (*it);
            }
            it++;
           
        }
        // create the temporal range
        PlageTemporelle* plage = new PlageTemporelle();
        plage->m_Debut = dStart -dDebutSimu;
        plage->m_Fin = dEnd - dDebutSimu;
        std::ostringstream str; // create name from end and start
        str <<STime::FromSecond(dStart).ToString() <<"_"<<STime::FromSecond(dEnd).ToString();
        plage->m_ID = str.str();
        // add to list of temporal Range
        plages.push_back(plage);
        return plage;
    }
    virtual ~PlageTemporelle()
    {
        int i = 0;
    }
     bool operator <(const PlageTemporelle& rhs) const
    {
        return m_ID < rhs.m_ID;
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

// dÃ©finition d'un type double qui peut Ãªtre sÃ©rialisÃ© dans un shared_ptr
BOOST_STRONG_TYPEDEF(double, tracked_double)
template<class Archive>
void serialize(Archive &ar, tracked_double& td, const unsigned int version){
    ar & BOOST_SERIALIZATION_NVP2("tracked_double",static_cast<double&>(td));
} 

// dÃ©finition d'un type bool qui peut Ãªtre sÃ©rialisÃ© dans un shared_ptr
BOOST_STRONG_TYPEDEF(bool, tracked_bool)
template<class Archive>
void serialize(Archive &ar, tracked_bool& tb, const unsigned int version){
    ar & BOOST_SERIALIZATION_NVP2("tracked_bool",static_cast<bool&>(tb));
} 


// Structure template de stockage des variantes temporelles lorsque le caractÃ¨re des variantes est exprimÃ©es Ã  l'aide d'une durÃ©e d'application
// ou d'une plage temporelle (l'un ou l'autre exclusif)
template<class T> struct TimeVariation 
{
public:
    TimeVariation() : m_pPlage(NULL) { m_dbPeriod = 0; };
    double  m_dbPeriod;   // DurÃ©e de la variante
    PlageTemporelle* m_pPlage; // Plage temporelle associÃ©e
    boost::shared_ptr<T>    m_pData;      // DonnÃ©es

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
        ar & BOOST_SERIALIZATION_NVP(m_dbPeriod);
        ar & BOOST_SERIALIZATION_NVP(m_pPlage);
        ar & BOOST_SERIALIZATION_NVP(m_pData);
    }
};

// Structure template de stockage des variantes temporelles lorsque le caractÃ¨re temporel des variantes est exprimÃ©es Ã  l'aide d'une heure de dÃ©but d'aplication
template<class T> struct TimeVariationEx 
{
	STime                   m_dtDebut;        // Heure de dÃ©but de la variante
    boost::shared_ptr<T>    m_pData;          // DonnÃ©es

public:
    TimeVariationEx() {};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
        ar & BOOST_SERIALIZATION_NVP(m_dtDebut);
        ar & BOOST_SERIALIZATION_NVP(m_pData);
    }
};

bool CheckPlagesTemporellesEx(double dbDureeSimu, const std::vector<PlageTemporelle*> & plages);

// Fonctions template de manipulations des listes des variantes temporelles
// REMARQUES : le code des fonctions Template est ici sinon elles ne 
// sont pas reconnues Ã  l'Ã©dition des liens (pas supportÃ© par le compilo)



//================================================================
    template <class T> void AddVariation
//----------------------------------------------------------------
// Fonction  : Ajoute une variante Ã  la liste des variantes
// Remarques :
// Version du: 20/06/2008
// Historique: 20/06/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(   
    double                          dbPeriod,   // PÃ©riode de validitÃ© de la variante
    boost::shared_ptr<T>            pData,      // Variante
    std::deque<TimeVariation<T>>    *pLstTV     // Liste des variantes
)
{
    TimeVariation<T> TV;

    if(!pLstTV)
        return;

    if(dbPeriod<0)
        return;

    TV.m_dbPeriod = dbPeriod;
    TV.m_pData = pData;

    pLstTV->push_back(TV);
};

//================================================================
    template <class T> void AddVariation
//----------------------------------------------------------------
// Fonction  : Ajoute une variante Ã  la liste des variantes
// Remarques :
// Version du: 06/09/2011
// Historique: 06/09/2011 (O.Tonck - Ipsis)
//             CrÃ©ation
//================================================================
(   
    PlageTemporelle                 *pPlage,    // Plage temporelle associÃ©e Ã  la variante
    boost::shared_ptr<T>            pData,      // Variante
    std::deque<TimeVariation<T>>    *pLstTV     // Liste des variantes
)
{
    TimeVariation<T> TV;

    if(!pLstTV)
        return;

    if(pPlage == NULL)
        return;

    TV.m_pPlage = pPlage;
    TV.m_pData = pData;

    pLstTV->push_back(TV);
};


//================================================================
    template <class T> void InsertVariation
//----------------------------------------------------------------
// Fonction  : Insertion d'une variante Ã  la liste des variantes
// Remarques :
// Version du: 27/07/2011
// Historique: 27/07/2011 (O. Tonck - IPSIS)
//             CrÃ©ation
//================================================================
(   
    double                          dbStartTime,    // DÃ©but de validitÃ© de la variante
    double                          dbEndTime,      // Fin de validitÃ© de la variante
    boost::shared_ptr<T>            pData,          // Variante
    std::deque<TimeVariation<T>>    *pLstTV         // Liste des variantes
)
{
    // **************************************************
    // Modification des variantes existantes si besoin
    // **************************************************

    // recherche de la premiÃ¨re variante impactÃ©e
    double dbCumul = 0.0;
    typename std::deque<TimeVariation<T>>::iterator iter = pLstTV->begin();
    while(iter != pLstTV->end())
    {
        double currentDuree = (*iter).m_dbPeriod;

        // sÃ©paration en deux en cas de variante Ã  cheval sur la nouvelle variante (dÃ©but de pÃ©riode)
        if(dbStartTime > dbCumul && dbStartTime < (*iter).m_dbPeriod + dbCumul)
        {
            // modif de la variation existante
            (*iter).m_dbPeriod = dbStartTime - dbCumul;

            // nouvelle variation
            TimeVariation<T> tv;
            tv.m_pData = boost::shared_ptr<T>(new T(*pData));
            tv.m_dbPeriod = (*iter).m_dbPeriod + dbCumul - dbStartTime;
            iter = pLstTV->insert(iter + 1,  tv);
        }
        // cas des variantes intermÃ©diaires
        else if(dbCumul >= dbStartTime && dbCumul+(*iter).m_dbPeriod <= dbEndTime) 
        {
            (*iter).m_pData.reset();
            (*iter).m_pData = boost::shared_ptr<T>(new T(*pData));
        }
        // sÃ©paration en deux en cas de variante Ã  cheval sur la nouvelle variante (fin de pÃ©riode)
        else if(dbEndTime > dbCumul && dbEndTime < (*iter).m_dbPeriod + dbCumul)
        {
            // modif de la variation existante
            T oldValue = *((*iter).m_pData.get());
            (*iter).m_dbPeriod = dbEndTime - dbCumul;
            (*iter).m_pData = boost::shared_ptr<T>(new T(*pData));

            // nouvelle variation
            TimeVariation<T> tv;
            tv.m_pData = boost::shared_ptr<T>(new T(oldValue));
            tv.m_dbPeriod = (*iter).m_dbPeriod + dbCumul - dbEndTime;
            iter = pLstTV->insert(iter + 1,  tv);
        }

        dbCumul += currentDuree;
        iter++;
    }

    // *************************************************************************
    // Ajout de la variante sur la portion temporelle non prÃ©cÃ©demment couverte
    // *************************************************************************
    if(dbEndTime > dbCumul)
    {
        TimeVariation<T> tv;
        tv.m_pData = boost::shared_ptr<T>(new T(*pData));
        tv.m_dbPeriod = dbEndTime - dbCumul;
        pLstTV->push_back(tv);
    }
};

//================================================================
    template <class T> void EraseListOfVariation
//----------------------------------------------------------------
// Fonction  : Suppression de tous les Ã©lÃ©ments de la liste des 
//             variantes temporelles
// Remarques :
// Version du: 20/06/2008
// Historique: 20/06/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
( 
    std::deque<TimeVariation<T>> *pLstTV
)
{
    TimeVariation<T>    TV;

    if(!pLstTV)
        return;
    
    for(int i=0; i<(int)pLstTV->size(); i++)
    {
        TV = pLstTV->at(i); 
        TV.m_pData.reset();
    }

    pLstTV->clear();
}

//================================================================
    template <class T> void EraseVariation
//----------------------------------------------------------------
// Fonction  : Suppression les variations Ã  partir d'un certain
//             instant
// Remarques :
// Version du: 23/06/2008
// Historique: 23/06/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
( 
    std::deque<TimeVariation<T>> *pLstTV,
    double  dbTime,
    double  dbLag
)
{
    int                 nIndex = 0;
    double              dbCumTime = - dbLag;
    double              dbCumTimePre = 0;
    int                 i = 0;
    TimeVariation<T>    TV;
    typename std::deque<TimeVariation<T>>::iterator iTV;
    double              dbPeriod;
    boost::shared_ptr<T>    pData;

    if(!pLstTV)
        return;

    if(dbTime<0)
        return;

    if(pLstTV->size()==0)
        return;

    // on commence par supprimer les variantes correspondantes Ã  des plages temporelles
    iTV = pLstTV->begin();
    while(iTV != pLstTV->end())
    {
        if((*iTV).m_pPlage != NULL)
        {
            iTV = pLstTV->erase(iTV);
        }
        else
        {
            iTV++;
        }
    }

    TV = pLstTV->at(0);   
    dbCumTime += TV.m_dbPeriod;

    while(dbCumTime < dbTime && nIndex < (int)pLstTV->size() )
    {        
        TV = pLstTV->at(++nIndex);
        dbCumTimePre = dbCumTime;
        dbCumTime += TV.m_dbPeriod;
    }    
    
    if(nIndex < (int)pLstTV->size())
    {
        // Modification de la durÃ©e de la derniÃ¨re variante afin que la nouvelle 
        // variante soit pris en compte dÃ¨s le pas de temps courant
        dbPeriod = dbTime - dbCumTimePre - 0.1;
        pData = TV.m_pData;

        // Suppression des variantes suivantes
        for(i=nIndex; i<(int)pLstTV->size(); i++)
        {
            TV = pLstTV->at(i); 

            if(i!=nIndex)
                TV.m_pData.reset();
        }  
        
        i = 0;
        for ( iTV = pLstTV->begin( ); iTV != pLstTV->end( ); iTV++ )
        {
            if(i++ >= nIndex)
            {
                pLstTV->erase(iTV, pLstTV->end( ));

                // Ajout de la variation modifiÃ©e
                AddVariation(dbPeriod, pData, pLstTV);
                return;
            }            
        }        
    }
}

//================================================================
    template <class T> T* GetVariation
//----------------------------------------------------------------
// Fonction  : Retourne la variante correspondnat Ã  l'instant
//             passÃ© en paramÃ¨tre
// Remarques : Si l'instant n'est pas dÃ©fini, retourne la derniÃ¨re
//             variante de la liste
// Version du: 20/06/2008
// Historique: 20/06/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
( 
    double dbTime, 
    std::deque<TimeVariation<T>> *pLstTV,
    double dbLag
)
{    
    double              dbCumTime = - dbLag;

    if(!pLstTV)
        return NULL;

    if(dbTime<0)
        return NULL;

    if(pLstTV->size()==0)
        return NULL;

    // prioritÃ© aux plages temporelles
    for(size_t i = 0; i < pLstTV->size(); i++)
    {
        const TimeVariation<T> & TV = pLstTV->at(i);
        if(TV.m_pPlage != NULL)
        {
            if(TV.m_pPlage->m_Debut <= dbTime && TV.m_pPlage->m_Fin >= dbTime)
            {
                return TV.m_pData.get();
            }
        }
    }
    
    // Si pas de plage temporelle correspondante, on regarde les durÃ©es
    size_t variationIdx = 0;
    while(variationIdx < pLstTV->size() && pLstTV->at(variationIdx).m_pPlage != NULL)
    {
        variationIdx++;
    }

    // si pas de durÃ©e dÃ©finie non plus ...
    if(variationIdx >= pLstTV->size())
    {
        return NULL;
    }

    TimeVariation<T> * pTV = &pLstTV->at(variationIdx);
    dbCumTime += pTV->m_dbPeriod;
    variationIdx++;

    while(dbCumTime < dbTime && variationIdx < pLstTV->size() )
    {        
        // on ne prend pas en compte les plages temporelles
        if(pLstTV->at(variationIdx).m_pPlage == NULL)
        {
            pTV = &pLstTV->at(variationIdx);
            dbCumTime += pTV->m_dbPeriod;
        }
        variationIdx++;
    }

    return pTV->m_pData.get();
}

//================================================================
    template <class T> std::vector<std::pair<TimeVariation<T>, std::pair<double,double> > > GetVariations
//----------------------------------------------------------------
// Fonction  : Retourne les variantes ayant une emrpise commune
//             avec la plage temporelle passÃ©e en paramÃ¨tre
// Version du: 19/01/2017
// Historique: 19/01/2017 (O.Tonck - Ipsis)
//             CrÃ©ation
//================================================================
(
    double dbTimeStart,
    double dbTimeEnd,
    std::deque<TimeVariation<T>> *pLstTV,
    double dbLag
)
{
    double              dbCumTime = -dbLag;

    std::vector<std::pair<TimeVariation<T>, std::pair<double, double> > >   result;

    if (!pLstTV)
        return result;

    if (pLstTV->size() == 0)
        return result;

    // prioritÃ© aux plages temporelles
    for (size_t i = 0; i < pLstTV->size(); i++)
    {
        const TimeVariation<T> & TV = pLstTV->at(i);
        if (TV.m_pPlage != NULL)
        {
            if (TV.m_pPlage->m_Debut <= dbTimeEnd && TV.m_pPlage->m_Fin >= dbTimeStart)
            {
                result.push_back(std::make_pair(TV, std::make_pair(
                    std::max<double>(TV.m_pPlage->m_Debut, dbTimeStart),
                    std::min<double>(TV.m_pPlage->m_Fin, dbTimeEnd)
                    )));
            }
        }
    }

    // Si pas de plage temporelle correspondante, on regarde les durÃ©es
    for (size_t variationIdx = 0; variationIdx < pLstTV->size(); variationIdx++)
    {
        const TimeVariation<T> & TV = pLstTV->at(variationIdx);
        if (TV.m_pPlage == NULL)
        {
            if (dbCumTime <= dbTimeEnd && (dbCumTime + TV.m_dbPeriod) >= dbTimeStart)
            {
                result.push_back(std::make_pair(TV, std::make_pair(
                    std::max<double>(dbCumTime, dbTimeStart),
                    std::min<double>(dbCumTime + TV.m_dbPeriod, dbTimeEnd)
                    )));
            }
            dbCumTime += TV.m_dbPeriod;

            if (dbCumTime > dbTimeEnd)
                break;
        }
    }

    return result;
}

//================================================================
    template <class T> std::deque<T*> GetVariations
//----------------------------------------------------------------
// Fonction  : Retourne les variantes correspondant Ã  l'instant
//             passÃ© en paramÃ¨tre
// Remarques : Si l'instant n'est pas dÃ©fini, retourne la derniÃ¨re
//             variante de la liste
// Version du: 22/10/2014
// Historique: 20/10/2014 (ETS - IPSIS)
//             CrÃ©ation
//================================================================
( 
    double dbTime, 
    std::deque<TimeVariation<T>> *pLstTV,
    double dbLag
)
{    
    double              dbCumTime = - dbLag;

    if(!pLstTV)
        return std::deque<T*>();

    if(dbTime<0)
        return std::deque<T*>();

    if(pLstTV->size()==0)
        return std::deque<T*>();

    std::deque<T*> variations;
    // prioritÃ© aux plages temporelles
    for(size_t i = 0; i < pLstTV->size(); i++)
    {
        const TimeVariation<T> & TV = pLstTV->at(i);
        if(TV.m_pPlage != NULL)
        {
            if(TV.m_pPlage->m_Debut <= dbTime && TV.m_pPlage->m_Fin >= dbTime)
            {
                variations.push_back(TV.m_pData.get() );
            }
        }
    }
    if( variations.size()>0 )
    {
        return variations;
    }
    
    // Si pas de plage temporelle correspondante, on regarde les durÃ©es
    size_t variationIdx = 0;
    while(variationIdx < pLstTV->size() && pLstTV->at(variationIdx).m_pPlage != NULL)
    {
        variationIdx++;
    }

    // si pas de durÃ©e dÃ©finie non plus ...
    if(variationIdx >= pLstTV->size())
    {
        return std::deque<T*>();
    }

    TimeVariation<T> * pTV = &pLstTV->at(variationIdx);
    dbCumTime += pTV->m_dbPeriod;
    variationIdx++;

    while(dbCumTime < dbTime && variationIdx < pLstTV->size() )
    {        
        // on ne prend pas en compte les plages temporelles
        if(pLstTV->at(variationIdx).m_pPlage == NULL)
        {
            pTV = &pLstTV->at(variationIdx);
            dbCumTime += pTV->m_dbPeriod;
        }
        variationIdx++;
    }
    variations.push_back(pTV->m_pData.get());
    return variations;
}
//================================================================
    template <class T> T* GetVariation
//----------------------------------------------------------------
// Fonction  : Retourne la valeur moyenne des variantes entre
//             les deux instants passÃ©s en paramÃ¨tres
// Version du: 24/09/2012
// Historique: 24/09/2012 (O.Tonck - IPSIS)
//             CrÃ©ation
//================================================================
( 
    double dbTimeDebut, 
    double dbTimeFin, 
    std::deque<TimeVariation<T>> *pLstTV,
    double dbLag
)
{    
    double              dbCumTime = -dbLag;
    double              dbCurrentTime = dbTimeDebut;
    TimeVariation<T>   *pTV;

    if(!pLstTV)
        return NULL;

    if(pLstTV->size()==0)
        return NULL;

    T * pValue = new T;
    *pValue = 0;

    // Tant qu'on n'a pas couvert toute la plage temporelle demandÃ©e...
    while(dbCurrentTime < dbTimeFin)
    {
        TimeVariation<T>   *pCurrentTV = NULL;
        dbCumTime = -dbLag;

        // recherche de la plage correspondante Ã  dbCurrentTime :

        // prioritÃ© aux plages temporelles
        for(size_t i = 0; i < pLstTV->size(); i++)
        {
            pTV = &pLstTV->at(i);
            if (pTV->m_pPlage != NULL)
            {
                if (pTV->m_pPlage->m_Debut <= dbCurrentTime && pTV->m_pPlage->m_Fin > dbCurrentTime)
                {
                    pCurrentTV = pTV;
                    dbCumTime = pTV->m_pPlage->m_Fin;
                }
            }
        }

        // Si pas de plage temporelle correspondante, on regarde les durÃ©es
        if(!pCurrentTV)
        {
            size_t variationIdx = 0;
            while(variationIdx < pLstTV->size() && pLstTV->at(variationIdx).m_pPlage != NULL)
            {
                variationIdx++;
            }

            // si pas de durÃ©e dÃ©finie non plus ...
            if(variationIdx >= pLstTV->size())
            {
                return NULL;
            }

            pTV = &pLstTV->at(variationIdx);
            pCurrentTV = pTV;
            dbCumTime += pTV->m_dbPeriod;
            variationIdx++;

            while(dbCumTime <= dbCurrentTime && variationIdx < pLstTV->size() )
            {        
                // on ne prend pas en compte les plages temporelles
                if(pLstTV->at(variationIdx).m_pPlage == NULL)
                {
                    pTV = &pLstTV->at(variationIdx);
                    pCurrentTV = pTV;
                    dbCumTime += pTV->m_dbPeriod;
                }
                variationIdx++;
            }
        }

        // Ici, on a la premiere variation correspondante :
        // on pondÃ¨re par la durÃ©e en commune entre la plage courante et la durÃ©e demandÃ©e
        double dbDuree = std::min<double>(dbCumTime, dbTimeFin)-dbCurrentTime;
        *pValue += (*pCurrentTV->m_pData.get())*dbDuree;
        dbCurrentTime = dbCumTime;
    }

    return pValue;
}

//================================================================
/*    template <class T> T* GetVariationEx
//----------------------------------------------------------------
// Fonction  : Retourne la variante correspondnat Ã  l'instant
//             passÃ© en paramÃ¨tre
// Remarques : Si l'instant n'est pas dÃ©fini, retourne la derniÃ¨re
//             variante de la liste
// Version du: 20/06/2008
// Historique: 20/06/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
( 
    double dbTime, 
    std::deque<TimeVariationEx<T>> *pLstTV
)
{    
    int                 nIndex = 0;
    double              dbCumTime = 0;
    TimeVariationEx<T>    TV;

    if(!pLstTV)
        return NULL;

    if(pLstTV->size()==0)
        return NULL;
    
    TV = pLstTV[0];   
    for(int i=0; i<(int)pLstT->sie(); i++)
    {
        TV = pLstTV[i];   
        if( TV.m_dtDebut > dbTime)
            return pLstTV[i-1];
    }

    return TV.m_pData;
}*/

//================================================================
    template <class T> double GetCumTime
//----------------------------------------------------------------
// Fonction  : Retourne le temps passÃ© dans les variantes prÃ©cÃ©dentes
// Remarques : 
// Version du: 20/06/2008
// Historique: 20/06/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
( 
    double dbTime, 
    std::deque<TimeVariation<T>> *pLstTV
)
{    
    int                 nIndex = 0;
    double              dbCumTime = 0; 

    if(!pLstTV)
        return 0;

    if(dbTime<0)
        return 0;

    if(pLstTV->size()==0)
        return 0;    
    
    TimeVariation<T> * pTV = &pLstTV->at(0);

    while(dbCumTime < dbTime && nIndex < (int)pLstTV->size()-1 )
    {    
        // on ignore les plages temporelles ici
        if (pTV->m_pPlage == NULL)
        {
            dbCumTime += pTV->m_dbPeriod;
        }
        pTV = &pLstTV->at(++nIndex);
    }

    return dbCumTime;
}


// Classe template de stockage des listes des variantes temporelles
template<class T> class ListOfTimeVariation
{
    private:
        std::deque<TimeVariation<T>>            m_LstTV;      // Liste des variantes
        double                                  m_dbLag;        // DÃ©calage (en seconde) entre la premiÃ¨re variante de la liste par rapport au dÃ©but de la simulation

    public:
        ListOfTimeVariation(){m_dbLag = 0;};
        ListOfTimeVariation(double dbLag);
        ~ListOfTimeVariation();

        void RemoveVariations() {EraseListOfVariation(&m_LstTV);};

		void Copy(ListOfTimeVariation *plstTV);

        void AddVariation(double dbPeriod, boost::shared_ptr<T> pData);
        void AddVariation(PlageTemporelle *pPlage, boost::shared_ptr<T> pData);
        T* GetVariationEx(double dbTime);
        T* GetVariationEx(double dbTime, double & dbEndVariationTime);

        void InsertVariation(double dbStartTime, double dbEndTime, boost::shared_ptr<T> pData);

        std::deque<TimeVariation<T>>*           GetLstTV(){return &m_LstTV;}

        double                                  GetLag(){return m_dbLag;}
        void                                    SetLag(double dbLag){m_dbLag = dbLag;}

        bool                                    CheckPlagesTemporelles(double dbDureeSimu);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
        ar & BOOST_SERIALIZATION_NVP(m_LstTV);
        ar & BOOST_SERIALIZATION_NVP(m_dbLag);
    }
};

    template<class T> ListOfTimeVariation<T>::ListOfTimeVariation(double dbLag)
    {
        m_dbLag = dbLag;
        m_LstTV.clear();
    }

    template<class T> ListOfTimeVariation<T>::~ListOfTimeVariation()
    {
    }

	template<class T> void ListOfTimeVariation<T>::Copy(ListOfTimeVariation *plstTV)
	{
		m_dbLag = plstTV->m_dbLag;			
		m_LstTV = plstTV->m_LstTV;		
	}

    template<class T> T* ListOfTimeVariation<T>::GetVariationEx(double dbTime)
    {        
        double dbPlaceHolder;
        return GetVariationEx(dbTime, dbPlaceHolder);
    }

    template<class T> T* ListOfTimeVariation<T>::GetVariationEx(double dbTime, double & endVariationTime)
    {
        double              dbCumTime = 0;
        endVariationTime = DBL_MAX;

        if (dbTime<0)
            return NULL;

        if (m_LstTV.size() == 0)
        {
            return NULL;
        }

        // rÃ©cupÃ©ration en prioritÃ© des valeurs dÃ©finies au niveau des plages temporelles
        for (size_t i = 0; i < m_LstTV.size(); i++)
        {
            const TimeVariation<T> & TVref = m_LstTV[i];
            if (TVref.m_pPlage)
            {
                if (TVref.m_pPlage->m_Debut <= dbTime && TVref.m_pPlage->m_Fin >= dbTime)
                {
                    endVariationTime = TVref.m_pPlage->m_Fin;
                    return TVref.m_pData.get();
                }
            }
        }

        // Ensuite on regarde les sÃ©quences de durÃ©es
        dbTime += m_dbLag;

        // Si pas de plage temporelle correspondante, on regarde les durÃ©es
        size_t variationIdx = 0;
        while (variationIdx < m_LstTV.size() && m_LstTV.at(variationIdx).m_pPlage != NULL)
        {
            variationIdx++;
        }

        // si pas de durÃ©e dÃ©finie non plus ...
        if (variationIdx >= m_LstTV.size())
        {
            return NULL;
        }

        TimeVariation<T> * pTV = &m_LstTV.at(variationIdx);
        dbCumTime = pTV->m_dbPeriod;
        variationIdx++;

        while (dbCumTime < dbTime && variationIdx < m_LstTV.size())
        {
            // on ne prend pas en compte les plages temporelles
            if (m_LstTV.at(variationIdx).m_pPlage == NULL)
            {
                pTV = &m_LstTV.at(variationIdx);
                dbCumTime += pTV->m_dbPeriod;
            }
            variationIdx++;
        }

        // rmq. : dans le cas de la derniÃ¨re variation temporelle, elle termine Ã  la fin de la simulation
        // mÃªme si la durÃ©e indiquÃ©e ne dure pas jusque lÃ .
        endVariationTime = variationIdx == m_LstTV.size() ? DBL_MAX : dbCumTime;
        return pTV->m_pData.get();
    }

    template<class T> void ListOfTimeVariation<T>::AddVariation(double dbPeriod, boost::shared_ptr<T> pData)
    {
        ::AddVariation(dbPeriod, pData, &m_LstTV);
    }

    template<class T> void ListOfTimeVariation<T>::AddVariation(PlageTemporelle *pPlage, boost::shared_ptr<T> pData)
    {
        ::AddVariation(pPlage, pData, &m_LstTV);
    }

    template<class T> void ListOfTimeVariation<T>::InsertVariation(double dbStartTime, double dbEndTime, boost::shared_ptr<T> pData)
    {
        ::InsertVariation(dbStartTime, dbEndTime, pData, &m_LstTV);
    }

    template<class T> bool ListOfTimeVariation<T>::CheckPlagesTemporelles(double dbDureeSimu)
    {
        std::vector<PlageTemporelle*> plages;
        for(size_t i = 0 ; i < m_LstTV.size(); i++)
        {
            if(m_LstTV[i].m_pPlage)
            {
                plages.push_back(m_LstTV[i].m_pPlage);
            }
        }
        if(plages.size() == 0)
        {
            // dans le cas oÃ¹ il n'y a pas de plages temporelles, rien Ã  vÃ©rifier (si on est quand mÃªme en mode 'horaire',
            // on suppose dans ce cas qu'une variation temporelle qui dure toute la simulation a Ã©tÃ© mise en place, conformÃ©ment aux spÃ©cifications)
            return true;
        }
        else
        {
            return CheckPlagesTemporellesEx(dbDureeSimu, plages);
        }
    }


// Classe template de stockage des listes des variantes temporelles
template<class T> class ListOfTimeVariationEx
{
private:
    std::deque<TimeVariationEx<T>>            m_LstTV;      // Liste des variantes

public:
    ListOfTimeVariationEx();        
    ~ListOfTimeVariationEx();

    void AddVariation(STime  dtDebut, T* pData);
    T* GetVariationEx(STime  dtTime);
    T* GetFirstVariationEx();
    T* GetVariation(int n){return m_LstTV[n].m_pData.get();};

    std::deque<TimeVariationEx<T>>*           GetLstTV(){return &m_LstTV;};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(m_LstTV);
    }
};

    template<class T> ListOfTimeVariationEx<T>::ListOfTimeVariationEx()
    {                
        m_LstTV.clear();
    }

    template<class T> ListOfTimeVariationEx<T>::~ListOfTimeVariationEx()
    {
        EraseListOfVariationEx( &m_LstTV );
    }

	template<class T> T* ListOfTimeVariationEx<T>::GetVariationEx(STime dtTmp)
    {        
        int                 nIndex = 0;
        double              dbCumTime = 0;
        TimeVariationEx<T>  *pTV, *pTVEx;  
        size_t              nvars;

		if(dtTmp < STime())
            return NULL;

        nvars = m_LstTV.size();
        if(nvars==0)
            return NULL;

        pTVEx = &(m_LstTV[0]);
        for(size_t i=0; i< nvars; i++)
        {
            pTV = &(m_LstTV[i]);
            if( pTV->m_dtDebut > dtTmp ) 
                return pTVEx->m_pData.get(); 

            pTVEx = pTV;
        }
                       
        return pTV->m_pData.get();       // Non trouvÃ©, on retourne la derniÃ¨re 
    }

	template<class T> void ListOfTimeVariationEx<T>::AddVariation(STime dtDebut, T* pData)
    {
        AddVariationEx(dtDebut, pData, &m_LstTV);
    }

    template<class T> T* ListOfTimeVariationEx<T>::GetFirstVariationEx()
    {                           
        if( m_LstTV.size()>0 )
            return m_LstTV[0].m_pData.get();   
        else
            return NULL;
    }

// Fonctions template de manipulations des listes des variantes temporelles
// REMARQUES : le code des fonctions Template est ici sinon elles ne 
// sont pas reconnues Ã  l'Ã©dition des liens (pas supportÃ© par le compilo)



//================================================================
    template <class T> void AddVariationEx
//----------------------------------------------------------------
// Fonction  : Ajoute une variante Ã  la liste des variantes
// Remarques :
// Version du: 23/10/2008
// Historique: 23/10/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(   
	STime                           dtDebut,    // Heure de dÃ©but de la simulation
    T                               *pData,     // Variante
    std::deque<TimeVariationEx<T>>    *pLstTV   // Liste des variantes
)
{
    if(!pLstTV)
        return;

    TimeVariationEx<T> TV;
    TV.m_dtDebut = dtDebut;
    TV.m_pData = boost::shared_ptr<T>(pData);

    pLstTV->push_back(TV);
};

//================================================================
    template <class T> void EraseListOfVariationEx
//----------------------------------------------------------------
// Fonction  : Suppression de tous les Ã©lÃ©ments de la liste des 
//             variantes temporelles
// Remarques :
// Version du: 23/10/2008
// Historique: 23/10/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
( 
    std::deque<TimeVariationEx<T>> *pLstTV
)
{
    if(!pLstTV)
        return;
    
    TimeVariationEx<T>    TV;
    for(int i=0; i<(int)pLstTV->size(); i++)
    {
        TV = pLstTV->at(i); 
        TV.m_pData.reset();
    }

    pLstTV->clear();
}

////================================================================
//    template <class T> void EraseVariation
////----------------------------------------------------------------
//// Fonction  : Suppression les variations Ã  partir d'un certain
////             instant
//// Remarques :
//// Version du: 23/10/2008
//// Historique: 23/10/2008 (C.BÃ©carie - Tinea)
////             CrÃ©ation
////================================================================
//( 
//    std::deque<TimeVariationEx<T>> *pLstTV,
//    System::DateTime  dtTime
//)
//{
//    int                 nIndex = 0;    
//    int                 i = 0;
//    TimeVariationEx<T>    TV;
//    std::deque<TimeVariationEx<T>>::iterator iTV;    
//    T*                  pData;
//    System::DateTime    dtTmp;
//
//    if(!pLstTV)
//        return;
//
//    if(dbTime<0)
//        return;
//
//    if(pLstTV->size()==0)
//        return;
//
//    TV = pLstTV->at(0);   
//    dtTmp = TV.m_dtDebut;
//
//    while(dtTmp < dtTime && nIndex < (int)pLstTV->size() )
//    {        
//        TV = pLstTV->at(++nIndex);
//        dtTmp = TV.m_dtDebut;
//    }    
//    
//    if(nIndex < (int)pLstTV->size())
//    {
//        // Modification de la durÃ©e de la derniÃ¨re variante afin que la nouvelle 
//        // variante soit pris en compte dÃ¨s le pas de temps courant        
//        pData = TV.m_pData;
//
//        // Suppression des variantes suivantes
//        for(i=nIndex; i<(int)pLstTV->size(); i++)
//        {
//            TV = pLstTV->at(i); 
//
//            if(i!=nIndex)
//                delete TV.m_pData;
//        }  
//        
//        i = 0;
//        for ( iTV = pLstTV->begin( ); iTV != pLstTV->end( ); iTV++ )
//        {
//            if(i++ >= nIndex)
//            {
//                pLstTV->erase(iTV, pLstTV->end( ));
//
//                // Ajout de la variation modifiÃ©e
//                AddVariation(dtTime, pData, pLstTV);
//                return;
//            }            
//        }        
//    }
//}


// ****************************************************************************
// Structure permettant de dÃ©finir un paramÃ¨tre pour une portion
// de tronÃ§on.
// ****************************************************************************
template<class T> class Portion {
public:
    /// Constructeurs
    Portion() {};
    Portion(double debut, double fin, T data) :
      dbPosDebut(debut),
      dbPosFin(fin),
      data(data) {};

    /// Membres
public:
    double dbPosDebut; // position curviligne du dÃ©but de la portion
    double dbPosFin; // position curviligne de la fin de la portion
    T data; // valeur du paramÃ¨tre

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
        ar & BOOST_SERIALIZATION_NVP(dbPosDebut);
        ar & BOOST_SERIALIZATION_NVP(dbPosFin);
        ar & BOOST_SERIALIZATION_NVP(data);
    }
};


template<typename T> std::string to_string( const T & Value )
{
    // utiliser un flux de sortie pour crÃ©er la chaÃ®ne
    std::ostringstream oss;
    // Ã©crire la valeur dans le flux
    oss << Value;
    // renvoyer une string
    return oss.str();
}

double Round (double dbNumber, double dbPrecision);

bool CalculIntersection( Troncon *pT1,Troncon *pT2, Point &ptInt, size_t &iPt1, size_t &iPt2);
bool CalculIntersection( Point pt11, Point pt12, Point pt21, Point pt22, Point &ptInt);
double GetDistance(Point pt1, Point pt2);
bool IsCounterClockWise(Point *pt1, Point *pt2, Point *pt3);
bool IsCounterClockWise(Point *V11, Point *V12, Point *V21, Point *V22);
double AngleOfView( double ViewPt_X, double ViewPt_Y, double Pt1_X, double Pt1_Y, double Pt2_X, double Pt2_Y );
std::deque<Point> BuildLineStringGeometry(const std::deque<Point> & lstPoints, double dbOffset, double dbTotalWidth, const std::string & strElementID, Logger * pLogger);
void CalculAbsCoords(Voie * pLane, double dbPos, bool bOutside, double & dbX, double & dbY, double & dbZ);
double CalculAngleTuyau(Tuyau * pTuyau, double dbPos);

bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, std::string &strVal, Logger* pLogger);
bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, bool &bVal, Logger* pLogger);
bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, double &dbVal, Logger* pLogger);
bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, char &cVal, Logger* pLogger);
bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, char &cVal, Logger* pLogger);
bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, int &nVal, Logger* pLogger);
bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, Point &pt, Logger* pLogger);
bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, SDateTime &dtTime, Logger* pLogger);
bool GetXmlAttributeValue(XERCES_CPP_NAMESPACE::DOMNode *DOMElement, const std::string &strAttr, unsigned int &uiVal, Logger* pLogger);

bool GetXmlDuree(XERCES_CPP_NAMESPACE::DOMNode *pXMLNode, Reseau * pReseau, double &dbVal, std::vector<PlageTemporelle*> & outPlages, Logger  *pLogger);

bool DecelerationProcedure(double dbTauxDec, Reseau* pReseau, const std::string & sTraficFile);
void TrajectoireProcedure(Reseau* pReseau, const std::string & sTraficFile);

void CalcCoordFromPos(Tuyau *pTuyau, double  dbPos, Point &ptCoord);

double CumulativeNormalDistribution(const double x);

Point CalculPositionTronconInsertion(const Point & ptTronconAval1, const Point & ptTronconAval2, std::vector<double> dbLargeurVoie, int nbVoieAmont, int nbVoieInsDroite);

// retourne les indices du vecteur dont les valeurs sont <= Ã  valueToFind
template<class T> 
std::vector<int> FindLessEqual(std::vector<T> vectors, const T& valueToFind )
{
    std::vector<int> returnIndices;
    typename std::vector<T>::iterator it;
    int iLoop =0;
    for( it = vectors.begin(); it!= vectors.end(); it++)
    {
        if((*it )<= valueToFind )
        {
            returnIndices.push_back(iLoop);

        }
        iLoop++;

    }
    return returnIndices;
}

#endif