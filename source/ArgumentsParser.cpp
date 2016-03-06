#include <cstdlib>
#include "ArgumentsParser.h"

ArgumentsParser::ArgumentsParser(std::string pattern): mPattern(pattern) { };

ArgumentsParser& ArgumentsParser::parse(int argc, char** argv) {

  char ch;
  while(-1 != (ch = getopt(argc, argv, this->mPattern.c_str()))) {
  	std::string value("");
  	if (optarg != NULL)
  		value = optarg;
  	this->mMap[ch] = value;
	}
	
	return *this;
}

ArgumentsMap ArgumentsParser::getMap() const {
	return this->mMap;
}
