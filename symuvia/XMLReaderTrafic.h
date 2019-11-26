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
#ifndef XMLReadTraficH
#define XMLReadTraficH

#include <string>
#include <fstream>
#include <map>

class XmlReaderSymuvia;
class Logger;


struct VehTraficData {
    std::string type;
    std::string entree;
    std::string sortie;
    std::string instC;
    std::string instE;
    std::string instS;
    std::string dstParcourue;
};


/*===========================================================================================*/
/* Classe de gestion de la lecture du document XML des données trafic					     */
/*===========================================================================================*/
class XMLReaderTrafic
{
private:
	std::string			m_strFilename;

	XmlReaderSymuvia*	m_XmlReaderTraj;	// Curseur des trajectoires des véhicules au cours de la simu	
	XmlReaderSymuvia*	m_XmlReaderVeh;		// Curseur de la liste des véhicules
    XmlReaderSymuvia*	m_XmlReaderSimuCell;// Curseur des variables de simulation des cellules 
    std::map<std::string, VehTraficData> m_mapVehData; // copie mémoire des données du noeud <VEHS>

    Logger* pFicSimulation;

    // Constructeur
	XMLReaderTrafic()
	{
		m_XmlReaderTraj = NULL;
		m_XmlReaderVeh = NULL;
		m_XmlReaderSimuCell = NULL;
	}

public:
    XMLReaderTrafic(Logger* pFS);

    ~XMLReaderTrafic();

    // Initialisation
	bool Init(const std::string & strFilename, std::string & sTimeExecTraf);
    
    // Routine permettant la lecture des trajectoires
	int ReadNextTrajectoire(double dbInstant, int nID);

    // Retourne la valeur d'un attribut pour la trajectoire courante
	std::string GetValueAttrFromTrajectoire(const std::string & ssAttr);

    // Lecture des données des véhicules
    void ReadVehicules();

    // Retourne les données pour le véhicule demandé
	VehTraficData & GetAttrFromVehicule(const std::string & id);

    // Routine de lecture des cellules au cours de la simulation
    int ReadNextSimuCell(double dbInstant, int nID);

    // Retourne la valeur d'un attribut pour la cellule de simulation courante
	std::string GetValueAttrFromSimuCell(const std::string & ssAttr);

};

#endif