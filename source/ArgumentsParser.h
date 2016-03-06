#ifndef __ARGUMENTS_PARSER_H__
#define __ARGUMENTS_PARSER_H__

#include <map>
#include <string>

typedef std::map<char, std::string> ArgumentsMap;
typedef std::map<char, std::string>::iterator ArgumentsIterator;
typedef std::map<char, std::string>::const_iterator ArgumentsConstIterator;

class ArgumentsParser {
public:
	ArgumentsParser(std::string pattern);
	ArgumentsParser& parse(int argc, char** argv);
	ArgumentsMap getMap() const;

private:
	std::string mPattern;
	ArgumentsMap mMap;
};

#endif