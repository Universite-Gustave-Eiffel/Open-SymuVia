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
#include "tools.h"

#include "Xerces/XMLUtil.h"
#include "Xerces/XMLReaderSymuvia.h"
#include "ZoneDeTerminaison.h"
#include "TronconDestination.h"
#include "TronconOrigine.h"
#include "tuyau.h"
#include "Connection.h"
#include "reseau.h"
#include "SystemUtil.h"
#include "vehicule.h"
#include "voie.h"
#include "Xerces/DOMLSSerializerSymu.hpp"
#include "Logger.h"

#include <xercesc/dom/DOMAttr.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/dom/DOMPSVITypeInfo.hpp>

#include <sstream>
#include <iomanip>

using namespace std;

XERCES_CPP_NAMESPACE_USE

//================================================================
    double Round
//----------------------------------------------------------------
// Fonction  : Retourne le rÃ©el passÃ© en paramÃ¨tre avec la prÃ©cision
//             demandÃ©
// Remarques :
// Version du: 31/05/2007
// Historique: 231/05/2007 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    double  dbNumber,
    double  dbPrecision
)
{
    double  dbDec;
    double  dbQuot;

    if( dbPrecision <= 0 )
        return dbNumber;

    dbDec = dbNumber - floor(dbNumber);

    dbQuot = dbDec / dbPrecision;

    if (abs( dbQuot - floor(dbQuot) ) < abs( dbQuot - ceil(dbQuot) ))
        return (floor(dbQuot) * dbPrecision) + floor(dbNumber);
    else
        return (ceil(dbQuot) * dbPrecision) + floor(dbNumber);
}


////-----------------------------------------------------------------------------
bool GetXmlAttributeValue(DOMNode *pXMLNode, const std::string &strAttr, std::string &strVal, Logger* pLogger)
{
	DOMAttr * pAttr;

    try
    {
        if(!pXMLNode)
            return false;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS(strAttr.c_str()));
        if(!pAttr)
            return false;

		strVal = US(pAttr->getValue());
    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
        *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		*pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
		*pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}

bool GetXmlAttributeValue(DOMNode *pXMLNode, const std::string &strAttr, bool &bVal, Logger* pLogger)
{
	DOMAttr * pAttr;

    try
    {
        if(!pXMLNode)
            return false;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS(strAttr.c_str())); 
        if(!pAttr)
            return false;    

		std::string sValue = US(pAttr->getValue());

		bVal = ( sValue == "1" || sValue == "true");    
    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
        *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		*pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
	    *pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}

bool GetXmlAttributeValue(DOMNode *pXMLNode, const std::string &strAttr, double &dbVal, Logger* pLogger)
{
	DOMAttr * pAttr;

    try
    {
        if(!pXMLNode)
            return false;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS(strAttr.c_str()));
        if(!pAttr)
            return false;    

		std::string sValue = US(pAttr->getValue());

        // patch pour gestion de la valeur INF (XSD indique INF alors que atof ne connait que +INF)
        if(!sValue.compare("INF"))
        {
            dbVal = numeric_limits<double>::infinity();
        }
        // cas gÃ©nÃ©ral
        else
        {
            dbVal = atof( sValue.c_str() ); 
        }

		
    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
        *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		*pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
		*pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}

bool GetXmlAttributeValue(DOMNode *pXMLNode, const std::string &strAttr, char &cVal, Logger* pLogger)
{
	DOMAttr * pAttr;

    try
    {
        if(!pXMLNode)
            return false;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS(strAttr.c_str()));    
        if(!pAttr)
            return false;    

		cVal = US(pAttr->getValue())[0];

    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
        *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		*pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
		*pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}

bool GetXmlAttributeValue(DOMNode *pXMLNode, const std::string &strAttr, int &nVal, Logger* pLogger)
{
	DOMAttr * pAttr;

    try
    {
        if(!pXMLNode)
            return false;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS(strAttr.c_str()));
        if(!pAttr)
            return false;    

		nVal = atoi(US(pAttr->getValue()));
    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
        *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		*pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
		*pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}

bool GetXmlAttributeValue(DOMNode *pXMLNode, const std::string &strAttr, unsigned int &uiVal, Logger* pLogger)
{
	DOMAttr * pAttr;

    try
    {
        if(!pXMLNode)
            return false;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS(strAttr.c_str()));
        if(!pAttr)
            return false;    

#ifdef WIN32
		int ires = sscanf_s(US(pAttr->getValue()), "%u", &uiVal);
#else
        int ires = sscanf(US(pAttr->getValue()), "%u", &uiVal);
#endif
		if (ires == 0)
		{
			uiVal = 0;

			*pLogger << Logger::Error << "Error parsing XML : " << std::endl;
			*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
            *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		    *pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
            *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
		}

    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
        *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		*pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
		*pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}

bool GetXmlAttributeValue(DOMNode *pXMLNode, const std::string &strAttr, Point &pt, Logger* pLogger)
{
	DOMAttr * pAttr;

    try
    {
        if(!pXMLNode)
            return false;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS(strAttr.c_str()));
        if(!pAttr)
            return false;    

		std::string sValue = US(pAttr->getValue());
		std::deque<std::string> split = SystemUtil::split(sValue, ' ');

		pt.dbX = SystemUtil::ToDouble( split.at(0) );
        pt.dbY = SystemUtil::ToDouble( split.at(1) );

		if( split.size() == 3)
        {
            std::string sS = split.at(2);
            if( sS.length() > 0)
				pt.dbZ = SystemUtil::ToDouble( sS );
            else
                pt.dbZ = 0;
        }
        else
            pt.dbZ = 0;    
    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
        *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		*pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
		*pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}

bool GetXmlAttributeValue(DOMNode *pXMLNode, const std::string &strAttr, SDateTime &dtTime, Logger* pLogger)
{
	DOMAttr * pAttr;
	std::deque<std::string> split;

    dtTime = SDateTime();

    try
    {
        if(!pXMLNode)
            return false;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS(strAttr.c_str()));
        if(!pAttr)
            return false;    

		std::string str = US(pAttr->getValue());
		if(  str.find(':') != std::string::npos)   // C'est une heure
        {
			split = SystemUtil::split(str, ':');
			dtTime = SDateTime(2000,1,1,SystemUtil::ToInt32(split.at(0)),SystemUtil::ToInt32(split.at(1)),SystemUtil::ToInt32(split.at(2)));
            return true;        
        }

		if(  str.find('-') != std::string::npos)   // C'est une heure
        {
            split = SystemUtil::split(std::string(US(pAttr->getValue())), '-');  
            dtTime = SDateTime(SystemUtil::ToInt32(split.at(0)),SystemUtil::ToInt32(split.at(1)),SystemUtil::ToInt32(split.at(2)),0,0,0);
        }
    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode) << std::endl;        
        *pLogger << Logger::Error << " attribute :" << strAttr << std::endl;
		*pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}

bool GetXmlDuree(DOMNode *pXMLNode, Reseau * pReseau, double &dbVal, vector<PlageTemporelle*> & outPlages, Logger  *pLogger)
{
	DOMAttr * pAttr;
    outPlages.clear();
    try
    {
		if(!pXMLNode)
            return true;

		pAttr = ((DOMElement*)pXMLNode)->getAttributeNode(XS("duree"));
        if(!pAttr)
        {
            dbVal = pReseau->GetDureeSimu();
            return true;     
        }

		std::string sValue = US(pAttr->getValue());

        // Si on est en mode plages temporelles
        if(pReseau->m_bTypeProfil)
        {
            DOMPSVITypeInfo* sourcePSVI=(DOMPSVITypeInfo*)pAttr->getFeature(XMLUni::fgXercescInterfacePSVITypeInfo, 0);

		    if (sourcePSVI == NULL)
		    {
			    dbVal = SystemUtil::ToDouble(sValue);
		    }
		    else
		    {
			    const XMLCh * sXmlDef = sourcePSVI->getStringProperty(DOMPSVITypeInfo::PSVI_Schema_Default);
			    if (sXmlDef == NULL)
			    {
				    dbVal = SystemUtil::ToDouble(sValue);
			    }
			    else
			    {
				    std::string sDefault = US(sXmlDef);
				
				    if (sValue == sDefault)		//	C'est la valeur par dÃ©faut
				    {
                        dbVal = pReseau->GetDureeSimu();
				    }
				    else	//Ce n'est pas la valeur par dÃ©faut
				    {
					    dbVal = SystemUtil::ToDouble(sValue);
				    }
			    }
		    }

            if(dbVal == 0.0)
            {
                *pLogger << Logger::Error << "ERROR : Unable to convert the duration to a valid non null number for node " << US(pXMLNode->getNodeName()) << std::endl;
                *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                return false;
            }
        }
        else
        {
            // si "always" (val. par dÃ©faut dans le XSD), on fait comme si on avait une durÃ©e de toute la simulation
            DOMPSVITypeInfo* sourcePSVI=(DOMPSVITypeInfo*)pAttr->getFeature(XMLUni::fgXercescInterfacePSVITypeInfo, 0);

		    if (sourcePSVI != NULL)
		    {
			    const XMLCh * sXmlDef = sourcePSVI->getStringProperty(DOMPSVITypeInfo::PSVI_Schema_Default);
			    if (sXmlDef != NULL)
			    {
				    std::string sDefault = US(sXmlDef);
                    if(!sDefault.compare(sValue))
                    {
                        dbVal = pReseau->GetDureeSimu();
                        return true;
                    }
                }
            }

            vector<PlageTemporelle*> & lstPlages = pReseau->m_LstPlagesTemporelles;
            std::deque<std::string> split = SystemUtil::split(sValue, ' ');
            for(size_t iPlage = 0; iPlage < split.size(); iPlage++)
            {
                bool bFound = false;
                for(size_t i = 0; i < lstPlages.size() && !bFound; i++)
                {
                    if(!lstPlages[i]->m_ID.compare(split[iPlage]))
                    {
                        outPlages.push_back(lstPlages[i]);
                        bFound = true;
                        break;
                    }
                }
                if(!bFound)
                {
                    *pLogger << Logger::Error << "ERROR : The time period of identifier " << split[iPlage] << " defined for the node " << US(pXMLNode->getNodeName()) << " doesn't exist !" << std::endl;
                    *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.
                    return false;
                }
            }
        }
    }
    catch(DOMException ex)
    {
        *pLogger << Logger::Error << "Error parsing XML : " << std::endl;
		*pLogger << Logger::Error << " node      :" << XMLUtil::getOuterXml(pXMLNode).c_str() << std::endl;        
        *pLogger << Logger::Error << " attribute :" << "duree" << std::endl;
		*pLogger << Logger::Error << " value     :" << US(pAttr->getValue()) << std::endl;
		*pLogger << Logger::Error << US(ex.getMessage()) << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        throw(ex);

        return false;
    }

    return true;
}


//================================================================
    bool IsCounterClockWise 
//----------------------------------------------------------------
// Fonction  : Retourne vrai si les trois points passÃ©s en paramÃ¨tre
//             constitus un angle positif (au sens trigonomÃ©trique)
// Remarques :
// Version du: 13/05/2008
// Historique: 13/05/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    Point*   p0,
    Point*   p1,
    Point*   p2
)
{
    double x1, x2, y1, y2;

    x1 = p1->dbX - p0->dbX; 
    y1 = p1->dbY - p0->dbY;
    x2 = p2->dbX - p0->dbX; 
    y2 = p2->dbY - p0->dbY;

    if (x1 * y2 > y1 * x2) 
        return TRUE;
    else
    {
        if (x1 * y2 < y1 * x2) 
            return FALSE;
        else 
        { 
            if (x1 * x2 < 0 || y1 * y2 < 0) 
                return false;
            else 
                if (x1*x1 + y1*y1 >= x2*x2 + y2*y2) 
                    return false;
                else return true;
        }
    }    
}

//================================================================
    bool IsCounterClockWise 
//----------------------------------------------------------------
// Fonction  : Retourne vrai si les vecteurs V1 et V2 forment un
//             angle positif (au sens trigonomÃ©trique)
// Remarques :
// Version du: 13/05/2008
// Historique: 13/05/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    Point*   V11,
    Point*   V12,
    Point*   V21,
    Point*   V22
)
{
   Point V1, V2, V0;

   V1.dbX = V12->dbX - V11->dbX;
   V1.dbY = V12->dbY - V11->dbY;

   V2.dbX = V22->dbX - V21->dbX;
   V2.dbY = V22->dbY - V21->dbY;

   V0.dbX = 0;
   V0.dbY = 0;

   return IsCounterClockWise(&V1, &V0, &V2);
}


//================================================================
    bool CalculIntersection
//----------------------------------------------------------------
// Fonction  : Retourne vrai si les deux tronÃ§ons ont un point 
//             d'intersection ainsi que ses coordonnÃ©es
// Remarques : Si plusieurs point d'intersection, retourne le premier
//             rencontrÃ©
//             On suppose que les deux tronÃ§ons sont plat et Ã  la mÃªme hauteur
// Version du: 08/11/2012
// Historique: 08/11/2012 (O. Tonck - IPSIS)
//             Ajout du retour de l'index des segments correspondants Ã  l'intersection
//             13/05/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    Troncon     *pT1,
    Troncon     *pT2,
    Point       &ptInt,
    size_t      &iPt1,
    size_t      &iPt2
)
{
    bool                bExist;
    unsigned int        i, j;
    std::deque<Point*>  LstPtsT1, LstPtsT2;

    bExist = false;

    // Construction des listes des points constituants les tronÃ§ons T1 et T2
    LstPtsT1.push_back(pT1->GetExtAmont());
    LstPtsT2.push_back(pT2->GetExtAmont());

    for(i=0; i<(int)pT1->GetLstPtsInternes().size(); i++)
        LstPtsT1.push_back(pT1->GetLstPtsInternes()[i]);

    for(i=0; i<(int)pT2->GetLstPtsInternes().size(); i++)
        LstPtsT2.push_back(pT2->GetLstPtsInternes()[i]);

    LstPtsT1.push_back(pT1->GetExtAval());
    LstPtsT2.push_back(pT2->GetExtAval());

    // Calcul de l'existence d'une intersection en dÃ©crivant tous les segments des tronÃ§ons
    for(i=0; i<LstPtsT1.size()-1;i++)
    {
        for(j=0; j<LstPtsT2.size()-1;j++)
        {            			
			double xa = LstPtsT1[i]->dbX;
			double ya = LstPtsT1[i]->dbY;
			double xb = LstPtsT1[i+1]->dbX;
			double yb = LstPtsT1[i+1]->dbY;
			double xc = LstPtsT2[j]->dbX;
			double yc = LstPtsT2[j]->dbY;
			double xd = LstPtsT2[j+1]->dbX;
			double yd = LstPtsT2[j+1]->dbY;

			if( ( (ya-yc)*(xb-xc)-(yb-yc)*(xa-xc) )*( (ya-yd)*(xb-xd)-(yb-yd)*(xa-xd) ) < 0 
				&& ( (yc-yb)*(xd-xb)-(yd-yb)*(xc-xb) )*( (yc-ya)*(xd-xa)-(yd-ya)*(xc-xa) ) < 0 )
			{
				bExist = TRUE;
				break;
			}
        }
        if(bExist)
            break;
    }

    if(bExist)
    {
        // Calcul des coordonnÃ©es exactes    
        double dbDen = (LstPtsT1[i]->dbY - LstPtsT1[i+1]->dbY) * (LstPtsT2[j]->dbX  - LstPtsT2[j+1]->dbX) - (LstPtsT2[j]->dbY - LstPtsT2[j+1]->dbY) * (LstPtsT1[i]->dbX  - LstPtsT1[i+1]->dbX);

        if( fabs( dbDen )  < 0.0001)
            return false;

        ptInt.dbX = ((LstPtsT2[j]->dbX * LstPtsT2[j+1]->dbY - LstPtsT2[j+1]->dbX * LstPtsT2[j]->dbY) * (LstPtsT1[i]->dbX - LstPtsT1[i+1]->dbX) - (LstPtsT1[i]->dbX * LstPtsT1[i+1]->dbY - LstPtsT1[i+1]->dbX * LstPtsT1[i]->dbY) * (LstPtsT2[j]->dbX - LstPtsT2[j+1]->dbX)) / ((LstPtsT1[i]->dbY - LstPtsT1[i+1]->dbY) * (LstPtsT2[j]->dbX  - LstPtsT2[j+1]->dbX) - (LstPtsT2[j]->dbY - LstPtsT2[j+1]->dbY) * (LstPtsT1[i]->dbX  - LstPtsT1[i+1]->dbX ));

        if( fabs(LstPtsT1[i]->dbX - LstPtsT1[i+1]->dbX) > 0.0001 )
            ptInt.dbY = ptInt.dbX * ((LstPtsT1[i]->dbY - LstPtsT1[i+1]->dbY)/(LstPtsT1[i]->dbX - LstPtsT1[i+1]->dbX)) + ((LstPtsT1[i]->dbX * LstPtsT1[i+1]->dbY - LstPtsT1[i+1]->dbX * LstPtsT1[i]->dbY)/(LstPtsT1[i]->dbX - LstPtsT1[i+1]->dbX));
        else        
		{
            double a, b;
			a = (LstPtsT2[j+1]->dbY - LstPtsT2[j]->dbY) / (LstPtsT2[j+1]->dbX - LstPtsT2[j]->dbX);	// correction du 3/03/2011 (CB) avant indice i Ã  la place de j
			b = LstPtsT2[j]->dbY - a * LstPtsT2[j]->dbX;
			ptInt.dbY = a*ptInt.dbX + b;
		}

        ptInt.dbZ = pT1->GetHautAmont();

        iPt1 = i;
        iPt2 = j;
    }

    return bExist;
}

//================================================================
    bool CalculIntersection
//----------------------------------------------------------------
// Fonction  : Retourne vrai si les deux droites constituÃ©es par les points
//             se coupent ainsi que les coordonnÃ©es de leur intersection
// Remarques : 
// Version du: 31/07/2008
// Historique: 31/07/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    Point       pt11, 
    Point       pt12, 
    Point       pt21, 
    Point       pt22,
    Point       &ptInt
)
{
    if( fabs((pt11.dbY - pt12.dbY)*(pt21.dbX - pt22.dbX) - (pt21.dbY - pt22.dbY)*(pt11.dbX - pt12.dbX)) > 0.0001)
    {
        ptInt.dbX = ( (pt21.dbX*pt22.dbY - pt22.dbX*pt21.dbY)*(pt11.dbX - pt12.dbX) - (pt11.dbX*pt12.dbY - pt12.dbX*pt11.dbY)*(pt21.dbX - pt22.dbX) ) / ((pt11.dbY - pt12.dbY)*(pt21.dbX - pt22.dbX) - (pt21.dbY - pt22.dbY)*(pt11.dbX - pt12.dbX));

        if( fabs(pt11.dbX - pt12.dbX) > 0.001 )
            ptInt.dbY = ptInt.dbX * ( (pt11.dbY - pt12.dbY) / (pt11.dbX - pt12.dbX) ) + (( pt11.dbX*pt12.dbY - pt12.dbX*pt11.dbY) / (pt11.dbX - pt12.dbX));
        else
        {
            if( fabs(pt21.dbX - pt22.dbX) > 0.001 )     
                ptInt.dbY = (pt21.dbY - pt22.dbY) / (pt21.dbX - pt22.dbX) * pt11.dbX + ( pt21.dbY - (pt21.dbY - pt22.dbY) / (pt21.dbX - pt22.dbX) * pt21.dbX);
            else
                return false;
        }

    //$Xi = (($X3 * $Y4 - $X4 * $Y3) * ($X1 - $X2) - ($X1 * $Y2 - $X2 * $Y1) * ($X3 - $X4)) / (($Y1 - $Y2) * ($X3 - $X4) - ($Y3 - $Y4) * ($X1 - $X2));
    //$Yi = $Xi * (($Y1 - $Y2)/($X1 - $X2)) + (($X1 * $Y2 - $X2 * $Y1)/($X1 - $X2));

        return true;
    }

    return false;
}

//================================================================
    double GetDistance
//----------------------------------------------------------------
// Fonction  : Retourne la distance sÃ©parant deux points
// Remarques : 
// Version du: 13/05/2008
// Historique: 13/05/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(   Point pt1, 
    Point pt2
)
{
    return sqrt(((pt1.dbX - pt2.dbX) * (pt1.dbX - pt2.dbX))
                             + ((pt1.dbY - pt2.dbY, 2) * (pt1.dbY - pt2.dbY, 2))
                             + ((pt1.dbZ - pt2.dbZ, 2) * (pt1.dbZ - pt2.dbZ, 2)));
}

//================================================================
    bool DecelerationProcedure
//----------------------------------------------------------------
// Fonction  : Procedure de correction des vitesses pour une 
//             meilleure prise en compte de la dÃ©cÃ©lÃ©ration
// Remarques :
// Version du: 11/07/2008
// Historique: 11/07/2008 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    double          dbTauxDec,      // Taux de dÃ©celÃ©ration Ã  appliquer
    Reseau*         pReseau,
	const std::string    & sTraficFile    // Chemin complet du fichier trafic Ã  traiter
)
{
    int                 nReInst;
	std::vector<DOMNode*> tabXmlNode;
    DOMDocument*          XmlDocD;
    XmlReaderSymuvia*     XmlDocR;
    DOMLSSerializerSymu*  XmlDocW;
    DOMElement*           XmlNodeChild;
    DOMElement*           XmlNodePrev;
    DOMElement*           xmlNode;    

    int                 nIdVeh;
    double              dbVal;

    // Calcul du nombre de pas de temps Ã  reprendre
    if(dbTauxDec<=0)
        return false;

	nReInst = (int)floor(pReseau->GetMaxVitMax() / ( pReseau->GetTimeStep() * dbTauxDec));

    deque<deque<int>*> TabTot;
    deque<int> *pTabInst;
    std::deque <int>::iterator it;
    std::deque <int>::iterator it2;
    double dbAcc;

	//
	//				Premier parcours du fichier XML
	//

	XmlDocR = XmlReaderSymuvia::Create(sTraficFile);
    XmlDocR->MoveToContent();
    XmlDocR->ReadStartElement("OUT");
    XmlDocR->Read();

    // Initialsation du document Xml temporaire
    XmlDocD = pReseau->GetXMLUtil()->CreateXMLDocument(XS("DocReader"));

    // Lecture des noeuds avant <INSTANTS>
    while( (!XmlDocR->eof) && (XmlDocR->Name != "INSTANTS"))
    {     
        XmlDocR->Read();
    }

    XmlDocR->ReadToFollowing("INST");

    // RepÃ©rage des noeuds Ã  modifier
    while( !XmlDocR->eof && XmlDocR->Name == "INST" )
    {
        // Lecture du noeud INST courant
		xmlNode = XmlDocR->ReadNode(XmlDocD);

        pTabInst = new deque<int>;

        // RepÃ©rage des couples (instant, ID vÃ©hicule) pour diffusion
        DOMNode * xmlnode = (DOMElement *)pReseau->GetXMLUtil()->SelectSingleNode("TRAJS", xmlNode->getOwnerDocument(), xmlNode);
		XMLSize_t counti = xmlnode->getChildNodes()->getLength();
		for (XMLSize_t i = 0; i<counti; i++)
        {
			XmlNodeChild = (DOMElement*)xmlnode->getChildNodes()->item(i);
			if (XmlNodeChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

			dbAcc = SystemUtil::ToDouble(std::string(US(XmlNodeChild->getAttribute(XS("acc")))));
            if(dbAcc < - dbTauxDec)
            {                
                pTabInst->push_back( SystemUtil::ToInt32(US(XmlNodeChild->getAttribute(XS("id"))) ));
            }
        }

        // Si au cours de l'Ã©tude de l'instant prÃ©cÃ©dent, l'ID avait dÃ©jÃ  Ã©tÃ© repÃ©rÃ©, on le vire
        if( TabTot.size() > 0 && pTabInst->size() >  0)
        {
            deque<int> *pTabPrev = TabTot.back();
            for( it = pTabInst->begin(); it != pTabInst->end(); it++)
            {
                for( it2 = pTabPrev->begin(); it2 != pTabPrev->end(); it2++)
                {
                    if( (*it) == (*it2) )
                    {
                        pTabPrev->erase(it2);
                        break;
                    }
                }
            }
        }

        TabTot.push_back( pTabInst );
    }
    delete XmlDocR;
    
	//
	//				DeuxiÃ¨me parcours du fichier XML
	//

    // Initialisation du reader
	XmlDocR = XmlReaderSymuvia::Create(sTraficFile);

    // Initialisation du writer    
	std::string str = pReseau->GetOutputDir() + "tmp.xml";
	XmlDocW = new DOMLSSerializerSymu();
	XmlDocW->initXmlWriter(str.c_str(), gXMLDecl_ver10, gXMLDecl_encodingASCII); 
	XmlDocW->setIndent(true);


    // Noeud dÃ©claration
	XmlDocW->writeXmlDeclaration();

    // Lecture du fichier Xml Ã  l'aide du reader et Ã©criture dans le fichier temporaire
    XmlDocR->MoveToContent();

	XmlDocW->writeStartElement(XS("OUT"));

    XmlDocR->ReadStartElement("OUT");

	XmlDocR->Read();//Atteindre le "IN"
	XmlDocR->WriteNode(XmlDocW, false);//Recopie du "IN"

    XmlDocW->writeStartElement(XS("SIMULATION")) ;

	std::string sName, sValue;
    while( XmlDocR->MoveToNextAttribute(sName, sValue) )
    {
        XmlDocW->writeAttributeString(XS(sName.c_str()), XS(sValue.c_str()));
    }

    XmlDocR->Read();

    // Lecture et recopie des noeuds avant <INSTANTS>
    while( (!XmlDocR->eof) && (XmlDocR->Name != "INSTANTS"))
    {
		XmlDocR->WriteNode(XmlDocW, false);
    }

    // Copie du noeud INSTANTS
    XmlDocW->writeStartElement(XS("INSTANTS"));
    XmlDocR->ReadToFollowing("INST");

   // Modification des noeuds

     // Initailisation du tableau des noeuds Xml
	tabXmlNode.resize((size_t)(nReInst+1), NULL);

    int nInst = 0;

   // Lecture, modification et Ã©criture des noeuds <INST>
    while( !XmlDocR->eof && XmlDocR->Name == "INST" )
    {
        // DÃ©calage des noeuds INST
        for(int i=nReInst-1; i>=0; i--)
        {
            if(tabXmlNode[i])
                tabXmlNode[i+1] = tabXmlNode[i];
        }

        // Lecture du noeud INST courant
		tabXmlNode[0] = XmlDocR->ReadNode(XmlDocD);

        pTabInst = TabTot[nInst];

        // Boucle sur les trajectoires
        DOMNode * xmlnodes = pReseau->GetXMLUtil()->SelectSingleNode("TRAJS", tabXmlNode[0]->getOwnerDocument(), (DOMElement*)tabXmlNode[0]);
	   XMLSize_t counti = xmlnodes->getChildNodes()->getLength();
	   for (XMLSize_t i=0; i<counti; i++)
       {
		   XmlNodeChild = (DOMElement*)xmlnodes->getChildNodes()->item(i);
		   if (XmlNodeChild->getNodeType() != DOMNode::ELEMENT_NODE) continue;

		   nIdVeh = SystemUtil::ToInt32(std::string(US(XmlNodeChild->getAttribute(XS("id"))))); // Id du vÃ©hicule dont les vitesses doit Ãªtre reprises

           // L'instant et le vÃ©hicule ont t'ils Ã©tÃ© repÃ©rÃ© ?
           for( it = pTabInst->begin(); it != pTabInst->end(); it++)
           {
               if( (*it) == nIdVeh)
               {               
                   XmlNodePrev = (DOMElement*)pReseau->GetXMLUtil()->SelectSingleNode("TRAJS/TRAJ[@id=\"" + SystemUtil::ToString(nIdVeh) + "\"]", tabXmlNode[0]->getOwnerDocument(), (DOMElement*)tabXmlNode[0]);
				   double dbVit =  SystemUtil::ToDouble( std::string(US(XmlNodePrev->getAttribute(XS("vit")))) );
				   XmlNodePrev->getAttributeNode(XS("acc"))->setValue( XS(SystemUtil::ToString(2, -dbTauxDec).c_str()) );

                   // MAJ des vitesses des pas de temps prÃ©cÃ©dent si besoin
                   for(int i=1; i<=nReInst; i++)
                   {
                       XmlNodePrev = (DOMElement*)pReseau->GetXMLUtil()->SelectSingleNode("TRAJS/TRAJ[@id=\"" + SystemUtil::ToString(nIdVeh) + "\"]", tabXmlNode[i]->getOwnerDocument(), (DOMElement*)tabXmlNode[i]);

                        if(XmlNodePrev)
                        {
							if(  SystemUtil::ToDouble( std::string(US(XmlNodePrev->getAttribute(XS("vit"))))) > dbVit + i* dbTauxDec )
                            {
                                dbVal = dbVit + i* dbTauxDec;
								XmlNodePrev->getAttributeNode(XS("vit"))->setValue(XS(SystemUtil::ToString(2, dbVal).c_str()));
								XmlNodePrev->getAttributeNode(XS("acc"))->setValue(XS(SystemUtil::ToString(2, -dbTauxDec).c_str()));
                            }
                        }
                   }                                      
                   for(int i=1; i<nReInst; i++)
                   {
                       DOMElement *XmlNode1 = (DOMElement*)pReseau->GetXMLUtil()->SelectSingleNode("TRAJS/TRAJ[@id=\"" + SystemUtil::ToString(nIdVeh) + "\"]", tabXmlNode[i]->getOwnerDocument(), (DOMElement*)tabXmlNode[i]);
                       DOMElement *XmlNode2 = (DOMElement*)pReseau->GetXMLUtil()->SelectSingleNode("TRAJS/TRAJ[@id=\"" + SystemUtil::ToString(nIdVeh) + "\"]", tabXmlNode[i + 1]->getOwnerDocument(), (DOMElement*)tabXmlNode[i + 1]);

                       if( XmlNode1 && XmlNode2)
                       {
						   double dbVit1 =  SystemUtil::ToDouble( std::string(US(XmlNode1->getAttribute(XS("vit"))) ));
                           double dbVit2 =  SystemUtil::ToDouble( std::string(US(XmlNode2->getAttribute(XS("vit"))) ));
						   XmlNode1->getAttributeNode(XS("acc"))->setValue(XS(SystemUtil::ToString(2, (dbVit1 - dbVit2) / pReseau->GetTimeStep()).c_str()));  
                       }
                   }
               }
           }
       }
       // Ecriture
       if(tabXmlNode[nReInst])
       {
		    XmlDocW->writeNode(tabXmlNode[nReInst]);
       }
       nInst++;
    }

    // Ecriture des derniers instants
    for(int i=nReInst-1; i>=0; i--)
    {
        if(tabXmlNode[i])
        {
			XmlDocW->writeNode(tabXmlNode[i]);
        }
    }

      
    XmlDocW->writeEndElement(); // <INSTANTS/>
    XmlDocR->ResetStartEndFlags();

    // Copie des autres noeuds
    while( !XmlDocR->eof && XmlDocR->Name != "SIMULATION" )
    {
		XmlDocR->WriteNode(XmlDocW, false);
    }

    XmlDocW->writeEndElement(); // </SIMULATION>   

    XmlDocW->writeEndElement(); // </OUT>

	for (size_t i=0; i<tabXmlNode.size(); i++)
	{
		if (tabXmlNode[i])
		{
            tabXmlNode[i]->release();
			tabXmlNode[i] = NULL;
		}
	}

    XmlDocR->Close();
    XmlDocW->close();    

    delete(XmlDocW);
    delete(XmlDocR);    
    delete(XmlDocD);
	tabXmlNode.clear();

    // Suppression du fichier trafic et renommage de la nouvelle version
	remove(sTraficFile.c_str());
	rename(str.c_str(), sTraficFile.c_str());
   
    return true;
}


//================================================================
    void TrajectoireProcedure
//----------------------------------------------------------------
// Fonction  : ProcÃ©dure de post-traitement de gÃ©nÃ©ration des 
//             trajectoires dans un fichier au format XML
// Remarques :
// Version du: 26/03/2009
// Historique: 26/03/2009 (C.BÃ©carie - Tinea)
//             CrÃ©ation
//================================================================
(
    Reseau*         pReseau,    
	const std::string	&sTraficFile
)
{
    DOMDocument*        XmlDocD;
    DOMDocument*        XmlDocDV;
    XmlReaderSymuvia*   XmlDocRV;
    XmlReaderSymuvia*   XmlDocRT;
    DOMLSSerializerSymu*XmlDocW;
    DOMElement*         XmlNodeVeh;
    DOMElement*         xmlNodeVehs;
    DOMElement*         XmlNodeTraj;
    DOMElement*         XmlNodeInst;    
    int                 nID;
	std::string		    sT;
	std::string		    sV;
    double              dbVit;

    // Initialsation du document Xml temporaire
    XmlDocD = pReseau->GetXMLUtil()->CreateXMLDocument(XS("Util"));

    // Initialisation du writer        
	const std::string & str = pReseau->GetPrefixOutputFiles();
        
	XmlDocW = new DOMLSSerializerSymu();
	XmlDocW->initXmlWriter(str.c_str(), gXMLDecl_ver10, gXMLDecl_encodingASCII);
	XmlDocW->setIndent(true);

    // Noeud dÃ©claration
	XmlDocW->writeXmlDeclaration();

    // Initialisation du reader de la liste des vÃ©hicules
	XmlDocRV = XmlReaderSymuvia::Create(sTraficFile);
	XmlDocRV->MoveToContent();
	XmlDocW->writeStartElement(XS(XmlDocRV->Name.c_str()));
	XmlDocRV->Read();//"OUT"

    // Recopie des attributs de la simulation
	std::string sName, sValue;
    while( XmlDocRV->MoveToNextAttribute(sName, sValue) )
    {
        XmlDocW->writeAttributeString(XS(sName.c_str()), XS(sValue.c_str()));
    }
    XmlDocRV->Read();

    XmlDocW->writeStartElement(XS("SIMULATION"));  
    XmlDocW->writeStartElement(XS("TRAJS"));  

    // Recherche de l'Ã©lÃ©ment 'VEHS'
    while( (!XmlDocRV->eof) && (XmlDocRV->Name != "VEHS"))
    {     
        XmlDocRV->Read();
    }
	xmlNodeVehs = XmlDocRV->ReadNode(XmlDocD);

    // Parcours de tous les vÃ©hicules crÃ©Ã©s au cours de la simulation
	XMLSize_t counti = xmlNodeVehs->getChildNodes()->getLength();
	for(XMLSize_t i=0; i<counti; i++)
    {
		XmlNodeVeh = (DOMElement*)xmlNodeVehs->getChildNodes()->item(i);
		if (XmlNodeVeh->getNodeType() != DOMNode::ELEMENT_NODE) continue;

		nID = SystemUtil::ToInt32(US(XmlNodeVeh->getAttribute(XS("id"))));

		std::deque<std::string> iti;
        if( XmlNodeVeh->getAttributeNode(XS("itineraire")) )
			iti = SystemUtil::split(std::string(US(XmlNodeVeh->getAttribute(XS("itineraire")))), ' ');

        double dbDstCum = 0;

        XmlDocDV = pReseau->GetXMLUtil()->CreateXMLDocument(XS("Util"));

        // Initialisation du reader pour le vÃ©hicule traitÃ©
		XmlDocRT = XmlReaderSymuvia::Create(sTraficFile);
        XmlDocRT->MoveToContent();
        XmlDocRT->Read();
        XmlDocRT->Read();

        XmlDocW->writeStartElement(XS("VEH"));    
        XmlDocW->writeAttributeString(XS("id"), XS(SystemUtil::ToString(nID).c_str()));    // ID
		XmlDocW->writeAttributeString(XS("type"), XmlNodeVeh->getAttribute(XS("type")));    // Type de vÃ©hicule
        XmlDocW->writeStartElement(XS("INSTS"));

        Vehicule *pV = new Vehicule();
		TypeVehicule *pTV = pReseau->GetVehicleTypeFromID(std::string(US(XmlNodeVeh->getAttribute(XS("type")))));
        pV->SetTypeVeh( pTV );

        // Lecture des noeuds avant <INSTANTS>
        while( (!XmlDocRT->eof) && (XmlDocRT->Name != "INSTANTS"))
        {     
            XmlDocRT->Read();
        }
        XmlDocRT->ReadToFollowing("INST");
        bool bEnd = false;
        bool bDeb = false;
        

        while(!bEnd && !XmlDocRT->eof)
        {            
			if( XmlDocRT->Name == "INST" )
            {                
				XmlNodeInst = XmlDocRT->ReadNode(XmlDocDV);

                XmlNodeTraj = (DOMElement*)pReseau->GetXMLUtil()->SelectSingleNode("TRAJS/TRAJ[@id=\"" + SystemUtil::ToString(nID) + "\"]", XmlNodeInst->getOwnerDocument(), XmlNodeInst);
                if( XmlNodeTraj )
                {
                    bDeb = true;

                    XmlDocW->writeStartElement(XS("INST"));   

					dbVit =  SystemUtil::ToDouble( std::string(US(XmlNodeTraj->getAttribute(XS("vit"))) ));

                        // Ajout des caractÃ©ristiques de l'instant pour le vÃ©hicule traitÃ©     
                    if(!pReseau->IsTraceRoute() || iti.size()==0)
                    {
                        dbDstCum += dbVit*pReseau->GetTimeStep();
                    }
                    else
                    {
						sT = US(XmlNodeTraj->getAttribute(XS("tron")));
						sV = sT + "V" + std::string(US(XmlNodeTraj->getAttribute(XS("voie"))));
                        dbDstCum = 0;
                        for(XMLSize_t i=0; i<iti.size();i++)
                        {
                            if( sV == iti[i] )
                            {
								dbDstCum += SystemUtil::ToDouble( std::string(US(XmlNodeTraj->getAttribute(XS("dst"))) ));
                                break;
                            }
                            else
                            {                                
								int nV = SystemUtil::ToInt32( iti[i].substr(iti[i].size()-1 ,1) );
								Voie* pV = pReseau->GetLinkFromLabel(iti[i].substr(0, iti[i].size()-2))->GetLstLanes()[ nV - 1];
                                dbDstCum += pV->GetLength();
                            }
                        }
                    }

                    // Calcul des Ã©missions atmosphÃ©riques
                    /*pV->SetVit( dbVit );
                    pV->SetAcc( System::Convert::ToDouble(XmlNodeTraj->Attributes["acc"]->Value) );
                    pV->CalculEmissionAtmos(pReseau->GetPasTemps(), dbValCo2, dbValNox, dbValPM );*/

                    XmlDocW->writeAttributeString(XS("inst"), XmlNodeInst->getAttribute(XS("val")));
					XmlDocW->writeAttributeString(XS("dst"), XS(SystemUtil::ToString(2, dbDstCum).c_str()) );
                    XmlDocW->writeAttributeString(XS("vit"), XmlNodeInst->getAttribute(XS("vit")));
                    XmlDocW->writeAttributeString(XS("acc"), XmlNodeInst->getAttribute(XS("acc")));

                    /*XmlDocW->WriteAttributeString("CO2", dbValCo2.ToString("###0.##;;0") );
                    XmlDocW->WriteAttributeString("NOx", dbValNox.ToString("###0.##;;0") );
                    XmlDocW->WriteAttributeString("PM", dbValPM.ToString("###0.##;;0") );*/

                    XmlDocW->writeEndElement();   // "INST"
                }
                else
                {
                    if(bDeb) bEnd = true;
                }
            }            
            else
                XmlDocRT->Read();
        }

        XmlDocW->writeEndElement();         // "INSTS"
        XmlDocW->writeEndElement();         // "VEH"

        XmlDocRT->Close();
        delete XmlDocRT;
        delete XmlDocDV;

        delete pV;
    }    

    XmlDocW->writeEndElement();                 // "TRAJS"
    XmlDocW->writeEndElement();                 // "SIMULATION"
    XmlDocW->writeEndElement();                 // "OUT"


    XmlDocRV->Close();
    
    XmlDocW->close();    

    delete XmlDocRV;    
    delete(XmlDocW);

    return;
}   

//================================================================
    void CalcCoordFromPos
//----------------------------------------------------------------
// Fonction  :  Calcule les coordonnÃ©es d'un point Ã  partir de
//              sa position sur un tronÃ§on
// Remarque  :  Cette mÃ©thode retourne les coordonnÃ©es du point
//              central de l'ensemble des voies
// Version du:  28/07/2008
// Historique:  28/07/2008 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
(
    Tuyau   *pTuyau,
    double  dbPos,
    Point   &pt    
)
{
    int         i;

    if(dbPos<0)
        return;     // Pas de sortie rÃ©sultat pour les vÃ©hicules accumulÃ©s Ã  l'entrÃ©e
    
    if(!pTuyau)
        return;

    // Calcul de la position du vÃ©hicule projetÃ©
    dbPos = dbPos * pTuyau->GetLongueurProjection() / pTuyau->GetLength();    

    if(pTuyau->GetLstPtsInternes().size()==0)
    {
        pt.dbX = pTuyau->GetAbsAmont()+ (dbPos/ pTuyau->GetLongueurProjection()) * (pTuyau->GetAbsAval()- pTuyau->GetAbsAmont());
        pt.dbY = pTuyau->GetOrdAmont()+ (dbPos/ pTuyau->GetLongueurProjection()) * (pTuyau->GetOrdAval()- pTuyau->GetOrdAmont());
        pt.dbZ = pTuyau->GetHautAmont()+ (dbPos/ pTuyau->GetLongueurProjection()) * (pTuyau->GetHautAval()- pTuyau->GetHautAmont());
    }
    else
    {
        // Recherche du morceau sur lequel est positionnÃ© le vÃ©hicule en fonction de sa position
        double dbLg;
        double dbSumLg = 0;        
        Point *ppt1, *ppt2;
        for(i=-1; i<(int)pTuyau->GetLstPtsInternes().size(); i++)
        {
            if(i==-1)
            {
                ppt1 = pTuyau->GetExtAmont();
                ppt2 = pTuyau->GetLstPtsInternes()[0];
            }
            else if(i==(int)pTuyau->GetLstPtsInternes().size()-1)
            {
                ppt1 = pTuyau->GetLstPtsInternes()[i];
                ppt2 = pTuyau->GetExtAval();
            }
            else
            {                
                ppt1 = pTuyau->GetLstPtsInternes()[i];
                ppt2 = pTuyau->GetLstPtsInternes()[i+1];
            }

            dbLg = sqrt((ppt1->dbX - ppt2->dbX) * (ppt1->dbX - ppt2->dbX) +
                                      (ppt1->dbY - ppt2->dbY) * (ppt1->dbY - ppt2->dbY) +
                                      (ppt1->dbZ - ppt2->dbZ) * (ppt1->dbZ - ppt2->dbZ));

            if(dbPos < dbSumLg+dbLg )
            {
                // Calcul de la postion du vÃ©hicule sur le morceau identifiÃ©
                dbPos = dbPos - dbSumLg;

                // Calcul des coordonnÃ©es dans le repÃ¨re
                pt.dbX = ppt1->dbX + (dbPos / dbLg * (ppt2->dbX - ppt1->dbX));
                pt.dbY = ppt1->dbY + (dbPos / dbLg * (ppt2->dbY - ppt1->dbY));
                pt.dbZ = ppt1->dbZ + (dbPos / dbLg * (ppt2->dbZ - ppt1->dbZ));

                break;
            }
            else
            {
                dbSumLg += dbLg;
            }
        }
        if( i == (int)pTuyau->GetLstPtsInternes().size())
        {
            pt.dbX = pTuyau->GetAbsAval();
            pt.dbY = pTuyau->GetOrdAval();
            pt.dbZ = pTuyau->GetHautAval();
        }
    }
}


//================================================================
    double CumulativeNormalDistribution
//----------------------------------------------------------------
// Fonction  :  Calcule la probabiltÃ© d'une distribution normale
// Remarque  :  La mÃ©thode utilisÃ©e est 'Abromowitz and Stegun approximation' 
// Version du:  27/04/2009
// Historique:  27/04/2009 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
(
    const double x
)
{
  const double b1 =  0.319381530;
  const double b2 = -0.356563782;
  const double b3 =  1.781477937;
  const double b4 = -1.821255978;
  const double b5 =  1.330274429;
  const double p  =  0.2316419;
  const double c  =  0.39894228;

  if(x >= 0.0) {
      double t = 1.0 / ( 1.0 + p * x );
      return (1.0 - c * exp( -x * x / 2.0 ) * t *
      ( t *( t * ( t * ( t * b5 + b4 ) + b3 ) + b2 ) + b1 ));
  }
  else {
      double t = 1.0 / ( 1.0 - p * x );
      return ( c * exp( -x * x / 2.0 ) * t *
      ( t *( t * ( t * ( t * b5 + b4 ) + b3 ) + b2 ) + b1 ));
    }
}

//================================================================
    double AngleOfView
//----------------------------------------------------------------
// Fonction  :  Calcul de l'angle Ã  partir de 3 points (angle de vue
//              Ã  partir d'un point)
// Remarque  :  la fonction n'a plus besoin d'Ãªtre corrigÃ©e en
//              fonction du sens trigonomÃ©trique
// Version du:  17/08/2010
// Historique:  19/06/2009 (C.BÃ©carie - Tinea)
//              CrÃ©ation
//================================================================
( 
    double ViewPt_X,    // Abscisse du point de vue
    double ViewPt_Y,    // OrdonnÃ©e du point de vue
	double Pt1_X,       // Abscisse point 1
    double Pt1_Y,       // OrdonnÃ©e point 1
    double Pt2_X,       // Abscisse point 2
    double Pt2_Y        // OrdonnÃ©e point 2
)
{
    double a1, b1, a2, b2, a, b, t ;

    a1 = Pt1_X - ViewPt_X ;
    a2 = Pt1_Y - ViewPt_Y ;

    b1 = Pt2_X - ViewPt_X ;
    b2 = Pt2_Y - ViewPt_Y ;

    a = (a1*a1) + (a2*a2) ;
    b = (b1*b1) + (b2*b2) ;

    if ( (a == 0.0) || (b == 0.0) )
        return (0.0) ;

	t = atan2(a1*b2 - a2*b1, a1*b1 + a2*b2);

    // correction bug nÂ°44 : problÃ¨mes dans les prioritÃ©s Ã  droite pas toujours respectÃ©es
    // atan2 renvoie un angle dans le sens trigo compris entre -pi et +pi.
    // => on le ramene entre 0 et 2*pi
    if(t < 0)
    {
        t += 2*PI;
    }

    return (t);
}

std::deque<Point> BuildLineStringGeometry(const std::deque<Point> & lstPointsOriginal, double dbOffset, double dbTotalWidth, const std::string & strElementID, Logger * pLogger)
{
    // OTK - rmq. intÃ©rÃªt du paramÃ¨tre dbTotalWidth ? je ne comprend pas. Ca me semble faux, mais on ne doit de toute faÃ§on jamais
    // passer dans le cas ou l'offset dÃ©passe totalWIdth sur deux...

    std::deque<Point> result;

    double dbAlpha;
    Point   pt11, pt12, pt21, pt22, ptInt;

    // on filtre les points identiques
    std::deque<Point> lstPoints;
    for(size_t i = 0; i < lstPointsOriginal.size(); i++)
    {
        if(i == 0)
        {
            lstPoints.push_back(lstPointsOriginal[i]);
        }
        else
        {
            if((lstPointsOriginal[i].dbX != lstPoints.back().dbX)
                || (lstPointsOriginal[i].dbY != lstPoints.back().dbY))
            {
                lstPoints.push_back(lstPointsOriginal[i]);
            }
        }
    }


    if( lstPoints.size() > 2)
    {
        for(int j=1; j<(int)lstPoints.size()-1; j++)
        {                        
            // PremiÃ¨re droite
            dbAlpha  = atan2( lstPoints[j].dbY - lstPoints[j-1].dbY, lstPoints[j].dbX - lstPoints[j-1].dbX );

            if( dbOffset > (dbTotalWidth/2.) )
            {
                    if( dbAlpha == 0)                            
                        pt11.dbX = lstPoints[j-1].dbX;                            
                    else
                        pt11.dbX = dbOffset * cos(dbAlpha - M_PI_2) + lstPoints[j-1].dbX;
                    pt11.dbY = - dbOffset * sin(dbAlpha - M_PI_2) + lstPoints[j-1].dbY;
            }
            else
            {
                    if( dbAlpha == 0)
                        pt11.dbX = lstPoints[j-1].dbX;
                    else                            
                        pt11.dbX = dbOffset * cos(dbAlpha - M_PI_2) + lstPoints[j-1].dbX;
                    pt11.dbY = - dbOffset * sin(dbAlpha + M_PI_2) + lstPoints[j-1].dbY;
            }
            pt12.dbX = pt11.dbX + (lstPoints[j].dbX - lstPoints[j-1].dbX);
            pt12.dbY = pt11.dbY + (lstPoints[j].dbY - lstPoints[j-1].dbY);

            // Seconde droite
            dbAlpha  = atan2( lstPoints[j+1].dbY - lstPoints[j].dbY, lstPoints[j+1].dbX - lstPoints[j].dbX );

            if( dbOffset > (dbTotalWidth/2.) )
            {
                    if( dbAlpha == 0)                            
                        pt21.dbX = lstPoints[j].dbX;                            
                    else
                        pt21.dbX = dbOffset * cos(dbAlpha - M_PI_2) + lstPoints[j].dbX;
                    pt21.dbY = - dbOffset * sin(dbAlpha - M_PI_2) + lstPoints[j].dbY;
            }
            else
            {
                    if( dbAlpha == 0)
                        pt21.dbX = lstPoints[j].dbX;
                    else                            
                        pt21.dbX = dbOffset * cos(dbAlpha - M_PI_2) + lstPoints[j].dbX;
                    pt21.dbY = - dbOffset * sin(dbAlpha + M_PI_2) + lstPoints[j].dbY;
            }
            pt22.dbX = pt21.dbX + (lstPoints[j+1].dbX - lstPoints[j].dbX);
            pt22.dbY = pt21.dbY + (lstPoints[j+1].dbY - lstPoints[j].dbY);

            // Calcul de l'intersection
            if( fabs( pt12.dbX - pt21.dbX )<0.001 && fabs( pt12.dbY - pt21.dbY )<0.001 )
                ptInt = pt12;
            else
                CalculIntersection( pt11, pt12, pt21, pt22, ptInt);

            ptInt.dbZ = lstPoints[j].dbZ;

            result.push_back(ptInt);

            if( j==1)
            {
                pt11.dbZ = lstPoints[0].dbZ;
                result.push_front(pt11);
            }

            if( j+1==(int)lstPoints.size()-1)
            {
                pt22.dbZ = lstPoints[j+1].dbZ;
                result.push_back(pt22);
            }

        }
    }
    else if(lstPoints.size() == 2) // Pas de point interne
    {
        dbAlpha  = atan2( lstPoints.back().dbY - lstPoints.front().dbY, lstPoints.back().dbX - lstPoints.front().dbX );

        if( dbOffset > (dbTotalWidth/2.) )
        {
                if( dbAlpha == 0)                            
                    pt11.dbX = lstPoints.front().dbX;                            
                else
                    pt11.dbX = dbOffset * cos(dbAlpha - M_PI_2) + lstPoints.front().dbX;
                pt11.dbY = - dbOffset * sin(dbAlpha - M_PI_2) + lstPoints.front().dbY;
        }
        else
        {
                if( dbAlpha == 0)
                    pt11.dbX = lstPoints.front().dbX;   
                else                            
                    pt11.dbX = dbOffset * cos(dbAlpha - M_PI_2) + lstPoints.front().dbX;
                pt11.dbY = - dbOffset * sin(dbAlpha + M_PI_2) + lstPoints.front().dbY;
        }
        pt11.dbZ = lstPoints[0].dbZ;

        pt22.dbX = pt11.dbX + (lstPoints.back().dbX - lstPoints.front().dbX);
        pt22.dbY = pt11.dbY + (lstPoints.back().dbY - lstPoints.front().dbY);
        pt22.dbZ = lstPoints[lstPoints.size()-1].dbZ;

        result.push_back(pt11);
        result.push_back(pt22);
    }
    else if (lstPoints.size() == 1) // Un seul point !
    {
        // Peut arriver si CAF ponctuel par exemple (rÃ©seau mal dÃ©fini, donc warning)
        *pLogger << Logger::Warning << "Punctual geometry detected for element " << strElementID << std::endl;
        *pLogger << Logger::Info; // rebascule en mode INFO pour ne pas avoir Ã  reprendre tous les appels aux log en prÃ©cisant que c'est des INFO. Ã  supprimer si on reprend tous les appels au log.

        result.push_back(lstPoints.front());
    }
    else // 0 points : celÃ  peut-il arriver ?
    {
        assert(false);
    }

    return result;
}

void CalculAbsCoords(Voie * pLane, double dbPos, bool bOutside, double & dbX, double & dbY, double & dbZ)
{
    Tuyau * pTuyau = (Tuyau*)pLane->GetParent();
    // dÃ©calage Ã  utiliser dans le cas d'un arrÃªt hors voie
    double largeurTotale = 0;
    if(bOutside) 
    {
        // calcul du dÃ©calage transversal par rapport Ã  la voie du vÃ©hicule
        for(size_t i = 0; i < pTuyau->getLargeursVoies().size() && i <= (size_t)pLane->GetNum(); i++)
        {
            largeurTotale += pTuyau->getLargeursVoies()[i];
        }
    }

    // Calcul de la position projetÃ©e
    if (pLane->GetLength() > 0)
    {
        dbPos = dbPos * pLane->GetLongueurProjection() / pLane->GetLength();
    }
    else
    {
        dbPos = 0;
    }
    

    if(pLane->GetLstPtsInternes().size()==0)
    {
        if (pLane->GetLongueurProjection() > 0)
        {
            dbX = pLane->GetAbsAmont() + (dbPos / pLane->GetLongueurProjection()) * (pLane->GetAbsAval() - pLane->GetAbsAmont());
            dbY = pLane->GetOrdAmont() + (dbPos / pLane->GetLongueurProjection()) * (pLane->GetOrdAval() - pLane->GetOrdAmont());
            dbZ = pLane->GetHautAmont() + (dbPos / pLane->GetLongueurProjection()) * (pLane->GetHautAval() - pLane->GetHautAmont());

            if (bOutside)
            {
                double decaly = (pLane->GetAbsAval() - pLane->GetAbsAmont()) / pLane->GetLongueurProjection();
                double decalx = (pLane->GetOrdAval() - pLane->GetOrdAmont()) / pLane->GetLongueurProjection();

                decalx = decalx *largeurTotale;
                decaly = -decaly *largeurTotale;

                dbX += decalx;
                dbY += decaly;
            }
        }
        else
        {
            dbX = pLane->GetAbsAmont();
            dbY = pLane->GetOrdAmont();
            dbZ = pLane->GetHautAmont();
        }
    }
    else
    {
        // Recherche du morceau sur lequel est positionnÃ© le vÃ©hicule en fonction de sa position
		int i;
        double dbLg;
        double dbSumLg = 0;        
        Point *ppt1, *ppt2;
        for(i=-1; i<(int)pLane->GetLstPtsInternes().size(); i++)
        {
            if(i==-1)
            {
                ppt1 = pLane->GetExtAmont();
                ppt2 = pLane->GetLstPtsInternes()[0];
            }
            else if(i==(int)pLane->GetLstPtsInternes().size()-1)
            {
                ppt1 = pLane->GetLstPtsInternes()[i];
                ppt2 = pLane->GetExtAval();
            }
            else
            {                
                ppt1 = pLane->GetLstPtsInternes()[i];
                ppt2 = pLane->GetLstPtsInternes()[i+1];
            }

            dbLg = sqrt(pow( (ppt1->dbX - ppt2->dbX),2) + 
                pow( (ppt1->dbY - ppt2->dbY),2) +
                pow( (ppt1->dbZ - ppt2->dbZ),2));

            if(dbPos < dbSumLg+dbLg )
            {
                // Calcul de la postion du vÃ©hicule sur le morceau identifiÃ©
                dbPos = dbPos - dbSumLg;

                // Calcul des corrdonnÃ©es dans le repÃ¨re
                if (dbLg > 0)
                {
                    dbX = ppt1->dbX + (dbPos / dbLg * (ppt2->dbX - ppt1->dbX));
                    dbY = ppt1->dbY + (dbPos / dbLg * (ppt2->dbY - ppt1->dbY));
                    dbZ = ppt1->dbZ + (dbPos / dbLg * (ppt2->dbZ - ppt1->dbZ));

                    if (bOutside)
                    {
                        double decaly = (ppt2->dbX - ppt1->dbX) / dbLg;
                        double decalx = (ppt2->dbY - ppt1->dbY) / dbLg;

                        decalx = decalx *largeurTotale;
                        decaly = -decaly *largeurTotale;

                        dbX += decalx;
                        dbY += decaly;
                    }
                }
                else
                {
                    dbX = ppt1->dbX;
                    dbY = ppt1->dbY;
                    dbZ = ppt1->dbZ;
                }

                break;
            }
            else
            {
                dbSumLg += dbLg;
            }
        }
        if( i == (int)pLane->GetLstPtsInternes().size())
        {
            dbX = pLane->GetAbsAval();
            dbY = pLane->GetOrdAval();
            dbZ = pLane->GetHautAval();

            if(bOutside)
            {
                ppt1 = pTuyau->GetLstPtsInternes()[pTuyau->GetLstPtsInternes().size()-1];
                dbLg = sqrt((pLane->GetAbsAval() - ppt1->dbX)*(pLane->GetAbsAval() - ppt1->dbX)
                    - (pLane->GetOrdAval() - ppt1->dbY)*(pLane->GetOrdAval() - ppt1->dbY));
                if (dbLg > 0)
                {
                    double decaly = (pLane->GetAbsAval() - ppt1->dbX) / dbLg;
                    double decalx = (pLane->GetOrdAval() - ppt1->dbY) / dbLg;

                    decalx = decalx *largeurTotale;
                    decaly = -decaly *largeurTotale;

                    dbX += decalx;
                    dbY += decaly;
                }
            }
        }
    }
}

double CalculAngleTuyau(Tuyau * pTuyau, double dbPos)
{
    double result;

    int i;

    if(pTuyau->GetLstPtsInternes().size()==0)
    {
        result = AngleOfView(pTuyau->GetAbsAmont(), pTuyau->GetOrdAmont(), pTuyau->GetAbsAval(), pTuyau->GetOrdAval(), pTuyau->GetAbsAmont(), pTuyau->GetOrdAmont() + 1.0);
    }
    else
    {
        // Recherche du morceau sur lequel est positionnÃ© le vÃ©hicule en fonction de sa position
        double dbLg;
        double dbSumLg = 0;        
        Point *ppt1, *ppt2;
        for(i=-1; i<(int)pTuyau->GetLstPtsInternes().size(); i++)
        {
            if(i==-1)
            {
                ppt1 = pTuyau->GetExtAmont();
                ppt2 = pTuyau->GetLstPtsInternes()[0];
            }
            else if(i==(int)pTuyau->GetLstPtsInternes().size()-1)
            {
                ppt1 = pTuyau->GetLstPtsInternes()[i];
                ppt2 = pTuyau->GetExtAval();
            }
            else
            {                
                ppt1 = pTuyau->GetLstPtsInternes()[i];
                ppt2 = pTuyau->GetLstPtsInternes()[i+1];
            }

            dbLg = sqrt(pow( (ppt1->dbX - ppt2->dbX),2) + 
                pow( (ppt1->dbY - ppt2->dbY),2) +
                pow( (ppt1->dbZ - ppt2->dbZ),2));

            if(dbPos < dbSumLg+dbLg )
            {
                result = AngleOfView(ppt1->dbX, ppt1->dbY, ppt2->dbX, ppt2->dbY, ppt1->dbX, ppt1->dbY + 1.0);
                break;
            }
            else
            {
                dbSumLg += dbLg;
            }
        }
        if( i == (int)pTuyau->GetLstPtsInternes().size())
        {
            ppt1 = pTuyau->GetLstPtsInternes().back();
            ppt2 = pTuyau->GetExtAval();
            result = AngleOfView(ppt1->dbX, ppt1->dbY, ppt2->dbX, ppt2->dbY, ppt1->dbX, ppt1->dbY + 1.0);
        }
    }

    // En lien avec la correction de l'anomalie 64 : il faut repasser l'angle dans la fourchette -pi + pi pour Ãªtre conforme Ã  l'annotation de la direction sortie dans le fichier GML
    if(result > PI)
    {
        result -= 2.0 * PI;
    }

    return result;
}


// calcul de la position aval d'un troncon d'insertion permettant de recaler gÃ©omÃ©triquement les voies d'insertion correctement
Point CalculPositionTronconInsertion(const Point & ptTronconAval1, const Point & ptTronconAval2, std::vector<double> dbLargeurVoies, int nbVoieAmont, int nbVoieInsDroite)
{
    Point result;

    // calculs prÃ©liminaires
    double dbXa = ptTronconAval2.dbX-ptTronconAval1.dbX;
    double dbYa = ptTronconAval2.dbY-ptTronconAval1.dbY;

    // on commence par calculer le vecteur reliant le point amont du troncon aval Ã  son coin amont droit.
    Point vecteurAmontDroit;
    double dbDemieLargeurAval = 0;
    for(size_t i = 0; i < dbLargeurVoies.size(); i++)
    {
        dbDemieLargeurAval += dbLargeurVoies[i];
    }
    dbDemieLargeurAval = dbDemieLargeurAval / 2.0;

    double dbNormeAval = GetDistance(ptTronconAval1, ptTronconAval2);

    if(dbYa == 0)
    {
        vecteurAmontDroit.dbX = 0;
        vecteurAmontDroit.dbY = -(dbDemieLargeurAval*dbNormeAval)/dbXa;
    }
    else
    {
        vecteurAmontDroit.dbX = dbDemieLargeurAval*dbNormeAval/(dbYa+((dbXa*dbXa)/dbYa));
        vecteurAmontDroit.dbY = - (dbXa*vecteurAmontDroit.dbX)/dbYa;
    }

    // on peut maintenant en dÃ©duire facilement le vecteur reliant le point amont du troncon aval au coin aval droit du troncon amont :
    double dbNorm = sqrt(vecteurAmontDroit.dbX*vecteurAmontDroit.dbX + vecteurAmontDroit.dbY*vecteurAmontDroit.dbY);

    assert(dbNorm != 0);

    double dbDemiLargeurAmont = 0.0;
    for(int i = 0; i < nbVoieAmont; i++)
    {
        if(i < nbVoieInsDroite)
        {
            dbDemiLargeurAmont += dbLargeurVoies[0];
        }
        else if(i >= nbVoieInsDroite + (int)dbLargeurVoies.size())
        {
            dbDemiLargeurAmont += dbLargeurVoies[dbLargeurVoies.size()-1];
        }
        else
        {
            dbDemiLargeurAmont += dbLargeurVoies[i-nbVoieInsDroite];
        }
    }
    dbDemiLargeurAmont = dbDemiLargeurAmont / 2.0;

    Point vecteurCoinDroit;
    vecteurCoinDroit.dbX = vecteurAmontDroit.dbX + (nbVoieInsDroite*dbLargeurVoies[0]-dbDemiLargeurAmont)*vecteurAmontDroit.dbX/dbNorm;
    vecteurCoinDroit.dbY = vecteurAmontDroit.dbY + (nbVoieInsDroite*dbLargeurVoies[0]-dbDemiLargeurAmont)*vecteurAmontDroit.dbY/dbNorm;

    result.dbX = ptTronconAval1.dbX + vecteurCoinDroit.dbX;
    result.dbY = ptTronconAval1.dbY + vecteurCoinDroit.dbY;

    return result;
}

// vÃ©rification de la couverture de la durÃ©e de simulation totale par les plages temporelles
bool CheckPlagesTemporellesEx(double dbDureeSimu, const vector<PlageTemporelle*> & plages)
{
    double dbMaxTime = 0.0;
    bool bPlageFound = true;
    while(dbMaxTime < dbDureeSimu && bPlageFound)
    {
        // recherche des plages contenant dbMaxTime
        bPlageFound = false;
        for(size_t iPlage = 0; iPlage < plages.size(); iPlage++)
        {
            if(dbMaxTime >= plages[iPlage]->m_Debut && dbMaxTime <= plages[iPlage]->m_Fin)
            {
                if(plages[iPlage]->m_Fin > dbMaxTime)
                {
                    dbMaxTime = plages[iPlage]->m_Fin;
                    bPlageFound = true;
                }
            }
        }
    }

    return dbMaxTime >= dbDureeSimu;
}

 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template void PlageTemporelle::serialize(boost::archive::xml_woarchive & ar, const unsigned int version);
template void PlageTemporelle::serialize(boost::archive::xml_wiarchive & ar, const unsigned int version);

template<class Archive>
void PlageTemporelle::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(m_Debut);
    ar & BOOST_SERIALIZATION_NVP(m_Fin);
    ar & BOOST_SERIALIZATION_NVP(m_ID);
}
