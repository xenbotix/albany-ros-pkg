

#ifndef INCLUDED_NavNode_h_
#define INCLUDED_NavNode_h_

class NavNode
{

string line;
public:
void callprint (string instring)
{
line = instring;
std::cout << "GOT HERE " << line << std::endl;
}



};

#endif




