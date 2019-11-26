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
#include "DOMDocumentSymuvia.h"
#include <xercesc/dom/impl/DOMElementImpl.hpp>

#include "XercesString.h"

#include <string>

//?xml version
static const XMLCh  gXMLDecl_Tag[] =
{
    chQuestion,	chLatin_x,	chLatin_m,	chLatin_l,  chSpace, chNull
};

//version
static const XMLCh gXMLDecl_VersionInfo[] = 
{
    chLatin_v,   chLatin_e,  chLatin_r,     chLatin_s,  chLatin_i,  chLatin_o,
    chLatin_n,   chNull
};

//encoding
static const XMLCh  gXMLDecl_EncodingDecl[] =
{
    chLatin_e,  chLatin_n,  chLatin_c,  chLatin_o,	chLatin_d,	chLatin_i,
    chLatin_n,  chLatin_g,  chNull
};

//standalone
static const XMLCh  gXMLDecl_SDDecl[] =
{
    chLatin_s, chLatin_t, chLatin_a,   chLatin_n,    chLatin_d,	chLatin_a,
    chLatin_l, chLatin_o, chLatin_n,   chLatin_e,    chNull
};



DOMDocumentSymuvia::DOMDocumentSymuvia(DOMImplementation* domImpl, MemoryManager* const manager)
: DOMDocumentImpl(domImpl, manager)
{
}

DOMDocumentSymuvia::DOMDocumentSymuvia(const XMLCh*     namespaceURI,     //DOM Level 2
                const XMLCh*     qualifiedName,
                DOMDocumentType* doctype,
                DOMImplementation* domImpl,
                MemoryManager* const manager)
				: DOMDocumentImpl(namespaceURI, qualifiedName, doctype, domImpl, manager)
{
}

DOMDocumentSymuvia::~DOMDocumentSymuvia(void)
{
}

DOMElement* DOMDocumentSymuvia::createDeclaration()//A tester pour voir si ça fonctionne vraiment
{
	DOMElement * elem;

	elem = (DOMElementImpl *)createElement(gXMLDecl_Tag);

	elem->setAttribute(gXMLDecl_VersionInfo, this->getXmlVersion());

	elem->setAttribute(gXMLDecl_EncodingDecl, this->getXmlEncoding());

	std::string s;
	if (this->getXmlStandalone())
	{
		s = "yes";
	}
	else
	{
		s = "no";
	}

    elem->setAttribute(gXMLDecl_SDDecl, XercesString(s.c_str()));

	return elem;
}

void DOMDocumentSymuvia::release()
{
	release(this);
}

void DOMDocumentSymuvia::release(DOMNode * node)
{
	if (node->hasChildNodes())
	{
		DOMNodeList * list = node->getChildNodes();
		for (XMLSize_t i = 0; i<list->getLength(); i++)
		{
			release(list->item(i));
		}
	}
	node->release();
}


