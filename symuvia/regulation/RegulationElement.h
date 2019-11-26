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
#ifndef RegulationElementH
#define RegulationElementH

#pragma warning( disable : 4244 )
#include <boost/python/dict.hpp>
#pragma warning( default : 4244 )

#include <xercesc/util/XercesVersion.hpp>

namespace XERCES_CPP_NAMESPACE {
    class DOMNode;
}

namespace boost {
    namespace serialization {
        class access;
    }
}

class Reseau;

/**
* Classe abstraite commune aux Ã©lÃ©ments de rÃ©gulation (association avec une fonction python)
*/
class RegulationElement
{
public:

    // ************************************
    // Constructeurs/Destructeur
    // ************************************
    RegulationElement(void);
    virtual ~RegulationElement(void);

    // ************************************
    // Traitements publics
    // ************************************
    virtual void initialize(Reseau * pReseau, XERCES_CPP_NAMESPACE::DOMNode * pNode);

    // ************************************
    // Accesseurs
    // ************************************
    boost::python::dict & GetContext() {return m_ContextDict;}
    void SetContext(boost::python::dict & context) {m_ContextDict = context;}

protected:

    // ************************************
    // Traitements protÃ©gÃ©s
    // ************************************
    virtual std::string getModuleName();
    virtual std::string getBuiltinModuleName() = 0;

    // ************************************
    // Membres privÃ©s
    // ************************************
    // Pointeur vers le rÃ©seau
    Reseau* m_pReseau;

    // Fonction python correspondant Ã  l'Ã©lÃ©ment
    std::string m_strFunc;
    // Chemin complet du fichier module contenant la fonction python Ã  exÃ©cuter
    std::string m_strModule;
    // Dictionnaire Python correspondant aux paramÃ¨tres Ã  injecter avant le traitement de la fonction m_strFunc
    boost::python::dict m_FuncParamDict;
    // Contexte associÃ© Ã  l'Ã©lÃ©ment (permet d'avoir un objet persistent manipulable par script entre les diffÃ©rents pas de temps)
    boost::python::dict m_ContextDict;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SÃ©rialisation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
};

#endif // RegulationElementH