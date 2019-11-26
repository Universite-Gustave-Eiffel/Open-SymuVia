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

#ifndef SystemUtilH
#define SystemUtilH

#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#include <deque>

#ifdef MICROSOFT
#pragma warning(disable : 4996)
#endif

#ifdef M_PI
#define PI M_PI
#else
#define PI 3.14159265358979323846
#endif

#ifdef WIN32
#define DIRECTORY_SEPARATOR '\\'
#include <Windows.h>
#else
#define DIRECTORY_SEPARATOR '/'
#endif

class SystemUtil
{
public:
	SystemUtil(void);
	virtual ~SystemUtil(void);

private:
#ifdef WIN32
	static HMODULE m_hInstance;
#endif

public:

#ifdef WIN32
	static void SetInstance(HMODULE hInstance) { m_hInstance = hInstance; };
#endif

	static std::string GetModulePath();

	static std::string GetFileVersion(std::string sPathA = "");

	static std::string GetFilePath(const char * sFile);

    static std::string GetFileName(const std::string & filePath);

	static unsigned long GetSmallFileSize(const std::string & sFilePath);

	static std::string ReadSmallFile(const std::string & sFilePath);

	//Convertit un double en string avec "precision" chiffres aprÃ¨s le point (pas de virgule)
	static std::string ToString(int precision, double d);

    static std::string ToString(const char * str)
	{
		return std::string(str);
	}

	static std::string ToString(const bool bOk)
	{
		if (bOk)
		{
			return std::string("true");
		}
		else
		{
			return std::string("false");
		}
	}

	static std::string ToString(const int d)
	{
		char buf[512];
#ifdef WIN32
		sprintf_s(buf, 512, "%d", d);
#else
        snprintf(buf, 512, "%d", d);
#endif

		std::string sres = buf;
		return sres;
	}

	static std::string ToString(const int64_t d)
	{
		char buf[512];
#ifdef WIN32
		sprintf_s(buf, 512, "%lld", d);
#else
    #ifdef __APPLE__
            snprintf(buf, 512, "%lld", d);
    #else
            snprintf(buf, 512, "%ld", d);
    #endif
#endif

		std::string sres = buf;
		return sres;
	}

	static std::string ToString(const size_t d)
	{
		char buf[512];
#ifdef WIN32
#if _MSC_VER >= 1900
		sprintf_s(buf, 512, "%zu", d);
#else
		sprintf_s(buf, 512, "%u", d);
#endif
#else
        snprintf(buf, 512, "%lu", d);
#endif

		std::string sres = buf;
		return sres;
	}

	static double ToDouble(const std::string &s);

	static int ToInt32(const std::string &s);

#ifdef WIN32
    static std::wstring ToWString(const std::string &s);
    static std::string ToString(const std::wstring &s);
#endif

	static void trim(std::string &sw);

	static std::deque<std::string> split(const std::string & src, const char delim);

	static void split(const std::string & src, const char delim, std::deque<std::string> &splt);

	static bool FileExists(const std::string & sPath);

	static bool FolderExists(const std::string & sPath);

	static std::string ConcatPath(const std::string & path1, const std::string & path2);
};


#endif
      
 