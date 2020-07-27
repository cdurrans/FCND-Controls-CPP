#include "Common.h"
#include "SimpleConfig.h"
#include "Utility/StringUtils.h"

#include <vector>
#include <string>
#define FLOATMAX 100 
using namespace std;

#define MAX_INCLUDE_DEPTH 5

namespace SLR
{

	shared_ptr<SimpleConfig> SimpleConfig::s_config;

	SimpleConfig::SimpleConfig()
	{
		Reset("");
	}

	ParamsHandle SimpleConfig::GetInstance()
	{
		if (!s_config)
		{
			s_config.reset(new SimpleConfig());
		}
		return s_config;
	}

	void SimpleConfig::Reset(string rootParam)
	{
		// todo: go to the right directory
		// load all the files in the directory?
		_params.clear();
		if (rootParam != "")
		{
			ReadFile(rootParam);
		}
	}

	void SimpleConfig::ReplaceFile(string fileToReplace, string fileReplacingOther)
		// https://www.geeksforgeeks.org/c-program-copy-contents-one-file-another-file/
	{
		FILE *fptr1, *fptr2;
		char filename[100], c;

		// Open one file for reading 
		fptr1 = fopen(fileToReplace.c_str(), "r");
		if (fptr1 == NULL)
		{
			printf("Cannot open file %s \n", fileToReplace);
			exit(0);
		}

		// Open another file for writing 
		fptr2 = fopen(fileReplacingOther.c_str(), "w");
		if (fptr2 == NULL)
		{
			printf("Cannot open file %s \n", fileReplacingOther);
			exit(0);
		}

		// Read contents from file 
		c = fgetc(fptr1);
		while (c != EOF)
		{
			fputc(c, fptr2);
			c = fgetc(fptr1);
		}

		//printf("\nContents copied to %s\n", fileToReplace.c_str());

		fclose(fptr1);
		fclose(fptr2);
	}


	void SimpleConfig::ReadFile(const string& filename, int depth)
	{
		if (depth > MAX_INCLUDE_DEPTH)
		{
			SLR_WARNING0("Config includes excede maximum include depth (is something including itself?)");
			return;
		}

		FILE* f = fopen(filename.c_str(), "r");
		if (!f)
		{
			SLR_ERROR1("Can't open file %s", filename.c_str());
			return;
		}

		char buf[512]; buf[511] = 0;
		int lineNum = 0;
		string curNamespace = "";

		// read line by line...
		while (fgets(buf, 510, f))
		{
			lineNum++;
			string s(buf);

			ParseLine(filename, s, lineNum, curNamespace, depth);
		}

		fclose(f);
	}

	void SimpleConfig::AppendFile(string filename, string content)
	{
		FILE* fd = fopen(filename.c_str(), "a");
		if (fd == NULL) {
			printf("Error AppendFile function didn't open file.");
			exit(1);
		}
		fprintf(fd, "%s", content.c_str());
		fclose(fd);
	}

	//TODO consider using this for PID controller. Idea is to update the config file after each simulation using Twiddle Algorithmn.
	void SimpleConfig::TwiddleQuadControlParamsFile()
	{
		string twiddleStage = Get("QuadControlParams.TWIDDLESTAGE", "foobarred");
		if (twiddleStage == "stageOne" || twiddleStage == "stageTwo")
		{
			string paramToTwiddle = Get("QuadControlParams.PARAMTOTWIDDLE", "foobarred");
			float paramValue = Get("QuadControlParams." + ToUpper(paramToTwiddle), 0.0);
			if (paramValue == 0.0) {
				printf("Param value shouldn't be zero \n");
				exit(1);
			}
			float twiddleAdjustmentAmt = Get("QuadControlParams.TWIDDLEADJUSTMENTAMT", 9.9);
			if (twiddleAdjustmentAmt < 0.15) 
			{
				printf("Twiddle Adjustment Amount is really low, is it passing? \n");
				exit(0);
			}
			if (twiddleAdjustmentAmt == 9.9)
			{
				printf("Twiddle Adjustment Amount is default value, so is it not reading? \n");
				exit(1);
			}
			if (twiddleStage == "stageOne")
			{
				paramValue += twiddleAdjustmentAmt;
			}
			if (twiddleStage == "stageTwo")
			{
				paramValue -= 2 * twiddleAdjustmentAmt;
			}
			UpdateValueInConfigFile(paramToTwiddle, paramValue);
		}
		printf("Twiddle Stage is currently %s \n", twiddleStage.c_str());
	}

	void SimpleConfig::updateTwiddleCurrentError(float updatedValue)
	{
		/*printf("before");
		PrintAll();*/

		//printf("TwiddleCurrentError: %s \n", _params["QuadControlParams.TWIDDLECURRENTERROR"].c_str());
		float value = Get("QuadControlParams.TWIDDLECURRENTERROR", 0.0);// _params["QuadControlParams.TWIDDLECURRENTERROR"];
		//printf("Should not be zero TwiddleCurrentValue: %f \n", value);
		value += updatedValue * 100.0;
		string newstring = ConvertFloatToString(value);
		//printf("new string: %s \n", newstring.c_str());
		//printf("new value: %f \n", value);
		_params["QUADCONTROLPARAMS.TWIDDLECURRENTERROR"] = newstring;
		float valueagain = Get("QuadControlParams.TWIDDLECURRENTERROR", 0.0);// _params["QuadControlParams.TWIDDLECURRENTERROR"];
		//printf("Should not be zero updated TwiddleCurrentValue: %f \n", valueagain);
		//printf("TwiddleCurrentError updated: %s \n", _params["QUADCONTROLPARAMS.TWIDDLECURRENTERROR"].c_str());
		
	}

	void SimpleConfig::UpdateValueInConfigFile(string paramToUpdate, float paramValue) 
	{
		//printf("before");
		//PrintAll();
		//printf("UpdateInConfigFloat: %s : %f \n", paramToUpdate.c_str(), paramValue);
		string paramString = ConvertFloatToString(paramValue);
		//printf("UpdateInConfig After Conversion: %s : %s \n", paramToUpdate.c_str(), paramString.c_str());
		UpdateValueInConfigFileHelperFunction(paramToUpdate, paramString);
		_params["QUADCONTROLPARAMS." + ToUpper(paramToUpdate)] = paramString;
		//printf("after:");
		//PrintAll();
	}

	void SimpleConfig::UpdateValueInConfigFile(string paramToUpdate, string paramValue)
	{

		UpdateValueInConfigFileHelperFunction(paramToUpdate, paramValue);
		_params["QUADCONTROLPARAMS." + ToUpper(paramToUpdate)] = paramValue;

	}


void SimpleConfig::UpdateValueInConfigFileHelperFunction(string paramToUpdate, string paramValue) 
{
	printf("UpdateInConfigHelper: %s : %s \n", paramToUpdate.c_str(), paramValue.c_str());
	const string filename = "C:\\Users\\cdurrans\\Documents\\Udacity\\FCND-Term1-Starter-Kit\\FCND-Controls-CPP\\config\\QuadControlParams.txt";

	string fileCopy = filename.c_str();
	string fileDestination = fileCopy + "_temp.txt";
	FILE* f = fopen(filename.c_str(), "r");

	if (!f)
	{
		SLR_ERROR1("Can't open file %s", filename.c_str());
		return;
	}

	char buf[512]; buf[511] = 0;
	int lineNum = 0;
	string curNamespace = "";

	// read line by line...
	while (fgets(buf, 510, f))
	{
		lineNum++;
		string s(buf);
		string saveContents = "";

		std::size_t firstNonWS = s.find_first_not_of("\n\t ");

		if (firstNonWS == std::string::npos)
		{
			continue;
		}
		// is it a comment?
		if (s[firstNonWS] == '#' || firstNonWS == '/' || s[firstNonWS] == '[')
		{
			AppendFile(fileDestination, s);
			continue;
		}

		// is there an equals sign?
		std::size_t equals1 = s.find_first_of("=");
		std::size_t equals2 = s.find_last_of("=");
		if (equals1 != equals2 || equals1 == std::string::npos)
		{
			SLR_WARNING2("Line %d in config file %s is malformed", lineNum, filename.c_str());
			exit(1);
		}

		string leftPart = Trim(s.substr(firstNonWS, equals1 - firstNonWS));
		string rightPart = Trim(s.substr(equals1 + 1));

		if (leftPart == paramToUpdate)
		{
			saveContents = leftPart + " = " + paramValue + '\n';  // right part needing to be new number.
			AppendFile(fileDestination, saveContents);
			continue;
		}

		saveContents = leftPart + " = " + rightPart + '\n';
		AppendFile(fileDestination, saveContents);
	}

	fclose(f);

	ReplaceFile(fileDestination, filename);
	remove(fileDestination.c_str());
	
}


string SimpleConfig::ConvertFloatToString(float number)
{
	char errorString[100];
	gcvt(number, 10, errorString);
	string err = errorString;
	return err;
}

void SimpleConfig::TwiddleEvaluate()
{
	string twiddleStage = Get("QuadControlParams.TWIDDLESTAGE", "foobarred");
	if (twiddleStage == "stageOne" || twiddleStage == "stageTwo") 
	{
		float twiddleBestError = Get("QuadControlParams.TWIDDLEBESTERROR", 99999.9f);
		float twiddleCurrentError = Get("QuadControlParams.TWIDDLECURRENTERROR", 0.0f);
		float twiddleAdjustmentAmt = Get("QuadControlParams.TWIDDLEADJUSTMENTAMT", 1.f);
		//string newAdjustment;
		
		printf("Twiddle Eval Twiddle Current Error: %f \n", twiddleCurrentError);
		printf("Twiddle Eval Twiddle Best Error: %f \n", twiddleBestError);

		if (twiddleCurrentError < twiddleBestError)
		{
			printf("Current error is better. \n");
			UpdateValueInConfigFile("twiddleBestError", twiddleCurrentError);
			UpdateValueInConfigFile("twiddleAdjustmentAmt", twiddleAdjustmentAmt * 1.1);
			UpdateValueInConfigFile("twiddleStage", "stageOne");
			UpdateValueInConfigFile("nextParam", "yes");
			return;
		}

		if (twiddleStage == "stageOne")
		{
			UpdateValueInConfigFile("twiddleStage", "stageTwo");
		}

		if (twiddleStage == "stageTwo")
		{
			string paramToTwiddle = Get("QuadControlParams.PARAMTOTWIDDLE", "");
			if (paramToTwiddle != "")
			{
				float paramTwiddling = Get("QuadControlParams."+ToUpper(paramToTwiddle), 0.0);
				if (paramTwiddling != 0.0)
				{
					printf("Didn't get better\n");
					printf("Param to twiddle: %s, previous Value: %f, new value: %f", paramToTwiddle.c_str(), paramTwiddling, paramTwiddling + twiddleAdjustmentAmt);
					UpdateValueInConfigFile(paramToTwiddle, paramTwiddling + twiddleAdjustmentAmt);
				}
				else
				{
					printf("Get paramTwiddling didn't work \n");
					exit(1);
				}
			}
			UpdateValueInConfigFile("twiddleAdjustmentAmt", twiddleAdjustmentAmt * 0.9);
			UpdateValueInConfigFile("twiddleStage", "stageOne");
			string twiddleStageCheck = Get("QuadControlParams.TWIDDLESTAGE", "foobarred");
			printf("Should be stage one again: %s \n", twiddleStageCheck.c_str());
			UpdateValueInConfigFile("nextParam", "yes");
		}
		return;
	}
	
	if (twiddleStage == "initialize") {
		UpdateValueInConfigFile("twiddleStage", "stageOne");
		printf("Initialized Eval \n");
		return;
	}

	printf("Not twiddling right now.\n");
	return;
	
}

string * SimpleConfig::TwiddleSplitStr(string configParam)
{
	string line = Get("QuadControlParams." + ToUpper(configParam), "");
	string search = " ";
	int spacePos;
	int currPos = 0;
	int k = 0;
	int prevPos = 0;
	static string arr[10];

	do
	{
		spacePos = line.find(search, currPos);

		if (spacePos >= 0)
		{
			currPos = spacePos;
			arr[k] = line.substr(prevPos, currPos - prevPos);
			currPos++;
			prevPos = currPos;
			k++;
		}


	} while (spacePos >= 0);

	arr[k] = line.substr(prevPos, line.length());

	//for (int i = 0; i < k; i++)
	//{
	//	printf("Twiddle Split String value: %s \n", arr[i].c_str());
	//}
	return arr;
}


void SimpleConfig::TwiddleNextParam()
{
	//current implementation uses two pointers to point at the same static array that is created in the TwiddleSplitStr
	//I was having errors because each time I ran the TwiddleSplitStr it would up update the array, but since the two pointers
	//point at the same static array they were behaving the same and the values were for whichever param was used last in the TwiddleSplitStr function.
	string changeParam = Get("QuadControlParams.NEXTPARAM", "foobarred");
	if (changeParam == "yes")
	{
		int numParams = static_cast<int>(Get("QuadControlParams." + ToUpper("numberOfParamOptions"), 0.0));
		int currentPtr = static_cast<int>(Get("QuadControlParams." + ToUpper("paramPtr"), 0.0));

		//Adjustment Amount Update step
		string *twiddleAdjustmentOptions;
		twiddleAdjustmentOptions = TwiddleSplitStr("twiddleOptionsAdjustmentAmt");
		
		float twiddleAdjustmentCurrent = Get("QuadControlParams." + ToUpper("twiddleAdjustmentAmt"), 0.0);
		twiddleAdjustmentOptions[currentPtr] = ConvertFloatToString(twiddleAdjustmentCurrent);

		string updateString = "";
		for (int i = 0; i < numParams; i++)
		{
			updateString += twiddleAdjustmentOptions[i] + " ";
		}
		UpdateValueInConfigFile("twiddleOptionsAdjustmentAmt", updateString);

		if (numParams - 1 != currentPtr)
		{
			currentPtr++;
		}
		else
		{
			currentPtr = 0;
		}
		
		UpdateValueInConfigFile("twiddleAdjustmentAmt", twiddleAdjustmentOptions[currentPtr]);
		

		//Param To Twiddle
		string *twiddleOptions;
		twiddleOptions = TwiddleSplitStr("paramToTwiddleOptions");


		string nextTwiddle = twiddleOptions[currentPtr];
		printf("Next twiddle: %s ", nextTwiddle.c_str());
		// right here this is setting it to 1 instead of what it should be
		UpdateValueInConfigFile("paramToTwiddle", nextTwiddle);

		UpdateValueInConfigFile("nextParam", "no");
		UpdateValueInConfigFile("paramPtr", currentPtr);

		return;
	}

}




void SimpleConfig::ParseLine(const string& filename, const string& line, int lineNum, string& curNamespace, int depth)
{
  // primitive trailing removal
  string s = SLR::LeftOf(line, '#');

  std::size_t firstNonWS = s.find_first_not_of("\n\t ");

  // is it a comment?
  if (firstNonWS == std::string::npos || s[firstNonWS] == '#' || firstNonWS == '/')
  {
    return;
  }

  // include?
  if (SLR::ToUpper(s).find("INCLUDE ") == 0)
  {
    string filenameToInclude = s.substr(7);
    filenameToInclude = Trim(filenameToInclude);
    // need to put the file in the same directory as this one
    auto tmp = filename.find_last_of("/\\");
    string path="";
    if (tmp != string::npos)
    {
      path = filename.substr(0, tmp+1);
    }
    ReadFile(path+filenameToInclude, depth + 1);
    return;
  }

  // is it a namespace?
  std::size_t leftBracket = s.find_first_of("[");
  std::size_t rightBracket = s.find_last_of("]");
  if (leftBracket != std::string::npos && rightBracket != std::string::npos)
  {
    curNamespace = ToUpper(s.substr(leftBracket + 1, rightBracket - leftBracket - 1));
    // is it an inherited namespace?
    if (Contains(curNamespace, ':'))
    {
      string baseNamespace = Trim(RightOf(curNamespace, ':'));
      curNamespace = Trim(LeftOf(curNamespace, ':'));
      CopyNamespaceParams(baseNamespace, curNamespace);
    }
    return;
  }

  // is there an equals sign?
  std::size_t equals1 = s.find_first_of("=");
  std::size_t equals2 = s.find_last_of("=");
  if (equals1 != equals2 || equals1 == std::string::npos)
  {
    SLR_WARNING2("Line %d in config file %s is malformed", lineNum, filename.c_str());
    return;
  }

  // must be a parameter. split off the left part and the right part and remove whitespace
  // TODO: handle "" and '' strings?

  string leftPart = ToUpper(Trim(s.substr(firstNonWS, equals1 - firstNonWS)));
  string rightPart = Trim(s.substr(equals1 + 1));

  if (leftPart == "" || rightPart == "")
  {
    SLR_WARNING2("Line %d in config file %s is malformed", lineNum, filename.c_str());
    return;
  }

  if (curNamespace != "")
  {
    _params[curNamespace + "." + leftPart] = rightPart;
  }
  else
  {
    _params[leftPart] = rightPart;
  }
}

void SimpleConfig::CopyNamespaceParams(const string& fromNamespace, const string& toNamespace)
{
  string searchString = ToUpper(fromNamespace + ".");
  // very lazy implementation
  map<string, string> pCopy = _params;
  for (map<string,string>::iterator i = pCopy.begin(); i != pCopy.end(); i++)
  {
    if (i->first.compare(0, searchString.length(), searchString) == 0)
    {
      string tmp = i->first.substr(searchString.size());
      tmp = toNamespace + "." + tmp;
      _params[tmp] = i->second;
    }

  }

}

void SimpleConfig::PrintAll()
{
  for(auto i=_params.begin(); i!=_params.end(); i++)
  {
    printf("%s=%s\n",i->first.c_str(),i->second.c_str());
  }
}

bool SimpleConfig::Exists(const string& param)
{
  return _params.find(ToUpper(param)) != _params.end();
}

bool SimpleConfig::GetFloat(const string& param, float& ret)
{
  auto i = _params.find(ToUpper(param));
  if(i==_params.end()) return false;
  try
  {
      ret = std::stof(i->second);
      return true;
  }
  catch(...)
  {
      return false;
  }
}


bool SimpleConfig::GetString(const string& param, string& ret)
{
  auto i = _params.find(ToUpper(param));
  if(i==_params.end()) return false;
  ret = i->second;
  return true;
}

bool SimpleConfig::GetV3F(const string& param, V3F& ret)
{
  auto i = _params.find(ToUpper(param));
  if(i==_params.end()) return false;
  string s = i->second;
  std::size_t comma1 = s.find_first_of(",");
  std::size_t comma2 = s.find_last_of(",");
  if(comma1==comma2 || comma1==string::npos || comma2==string::npos) return false;
  string a = s.substr(0,comma1);
  string b = s.substr(comma1+1,comma2-comma1-1);
  string c = s.substr(comma2+1);
  try
  {
      ret = V3F(std::stof(a),std::stof(b),std::stof(c));
      return true;
  }
  catch(...)
  {
      return false;
  }
}

bool SimpleConfig::GetFloatVector(const string& param, vector<float>& ret)
{
  auto i = _params.find(ToUpper(param));
  if (i == _params.end()) return false;
  string s = i->second;
  vector<string> spl = SLR::Split(s, ',');
  ret.clear();
  for (unsigned i = 0; i < s.size(); i++)
  {
    try
    {
      float tmp = std::stof(spl[i]);
      ret.push_back(tmp);
    }
    catch (...)
    {
      return false;
    }
  }
  return true;
}


float SimpleConfig::Get(const string& param, float defaultRet)
{
  this->GetFloat(param,defaultRet);
  return defaultRet;
}

string SimpleConfig::Get(const string& param, string defaultRet)
{
  this->GetString(param,defaultRet);
  return defaultRet;
}

V3F SimpleConfig::Get(const string& param, V3F defaultRet)
{
  this->GetV3F(param,defaultRet);
  return defaultRet;
}



//TODO twiddle implementation idea
//bool SimpleConfig::SetFloat(const string& param, float& ret)
//{
//	this->Get
//}


} // namespace SLR

