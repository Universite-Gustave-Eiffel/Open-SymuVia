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
#ifndef ScheduleH
#define ScheduleH

#include "tools.h"

#include <boost/serialization/assume_abstract.hpp>

#include <vector>

class Logger;
class Reseau;
class ScheduleParameters;

class ScheduleElement
{
public:

    virtual ~ScheduleElement() {}

    // Chargement du schedule depuis un noeud XML
    virtual bool Load(XERCES_CPP_NAMESPACE::DOMNode * pXMLNode, Reseau * pNetwork, Logger & loadingLogger, const std::string & strID) = 0;

    // Renvoie vrai si l'élément du calendrier est activé pendant le pas de temps considéré
    virtual bool CheckTime(double dbInstant, double dbEndTimeStep, bool bHasNextConsecutiveElement, bool & bTooOld) = 0;

    virtual double GetStartTime() = 0;
    virtual double GetStopTime() = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

BOOST_SERIALIZATION_ASSUME_ABSTRACT(ScheduleElement)

class ScheduleInstant : public ScheduleElement
{
public:

    // Chargement du schedule depuis un noeud XML
    virtual bool Load(XERCES_CPP_NAMESPACE::DOMNode * pXMLNode, Reseau * pNetwork, Logger & loadingLogger, const std::string & strID);

    // Renvoie vrai si l'élément du calendrier est activé pendant le pas de temps considéré
    virtual bool CheckTime(double dbInstant, double dbEndTimeStep, bool bHasNextConsecutiveElement, bool & bTooOld);

    virtual double GetStartTime();
    virtual double GetStopTime();

private:
    double m_dbInstant;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

class ScheduleFrequency : public ScheduleElement
{
public:

    // Chargement du schedule depuis un noeud XML
    virtual bool Load(XERCES_CPP_NAMESPACE::DOMNode * pXMLNode, Reseau * pNetwork, Logger & loadingLogger, const std::string & strID);

    // Renvoie vrai si l'élément du calendrier est activé pendant le pas de temps considéré
    virtual bool CheckTime(double dbInstant, double dbEndTimeStep, bool bHasNextConsecutiveElement, bool & bTooOld);

    virtual double GetStartTime();
    virtual double GetStopTime();
    
    double GetFrequency();

private:
    double m_dbStartInstant;
    double m_dbStopInstant;
    double m_dbFrequency;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

class Schedule
{
public:
    Schedule();
    virtual ~Schedule();

    // Chargement du schedule depuis un noeud XML
    virtual bool Load(XERCES_CPP_NAMESPACE::DOMNode * pXMLNode, Reseau * pNetwork, Logger & loadingLogger, const std::string & strID,
                      const std::map<std::string, ScheduleParameters*> & scheduleParams, std::map<ScheduleElement*, ScheduleParameters*> & mapScheduleParams);

    const std::vector<ScheduleElement*> & GetLstElements();

    bool CheckTime(double dbInstant, double dbTimeStep, ScheduleElement ** pScheduleElement);

    // Estime une "fréquence instantanée" pour le schedule à l'instant demandé.
    virtual double GetInstantaneousFrequency(double dbInstant);

    int GetLastStartElementIndex();
    void SetLastStartElementIndex(int index);

protected:

    // Liste d'instants
    std::vector<ScheduleElement*> m_LstElements;

    // Variables de travail
    int m_LastStartElementIndex;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sérialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version);
};

#endif // ScheduleH