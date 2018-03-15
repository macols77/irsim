#include "general.h"


/* ------------------------------------------------------------------------------------------*/

void nextParam(char sep, istream& I) {
    char c;
    while(I.get(c) && c!= sep); // eat all chars til separtor character is found
}

/* ------------------------------------------------------------------------------------------*/

double getDouble(char sep, istream& I) {
  nextParam(sep, I);
  double d;
  I>>d;  // get the number
  return d;
}

/* ------------------------------------------------------------------------------------------*/

unsigned long getLongInt(char sep, istream& I) {
  nextParam(sep, I);
  unsigned long i;
  I>>i;  // get the number
  return i;
}

/* ------------------------------------------------------------------------------------------*/

int getInt(char sep, istream& I) {
  nextParam(sep, I);
  int i;
  I>>i;  // get the number
  return i;
}

/* ------------------------------------------------------------------------------------------*/

void getStr(char sep, istream& I, char* s) {
  nextParam(sep, I);
  I>>s;
}

/* ------------------------------------------------------------------------------------------*/

char getChar(char sep, istream& I) {
  nextParam(sep, I);
  char c; 
  I>>c;
  return c;
}

/* ------------------------------------------------------------------------------------------*/

bool getYesNo(char sep, istream& I) {
  char c = getChar(sep, I);
  bool result;
  switch(c) {
  case 'y': case 'Y':
    result = true;
    break;
  case 'n': case 'N':
    result = false;
    break;
  default:
    cerr << "ERROR: getYesNo(...) neither y nor n encountered" << endl;
    cerr << "Read: " << c << " instead" << endl;
    exit(-1);
    break;
  }
  return result;
}

/* ------------------------------------------------------------------------------------------*/

int string2Int(char* s) {
  int len = strlen(s);
  int val_start = 0; // where the (unsigned) value starts
  if(*s == '-') {  // negative value
    val_start = 1;
  }
  int pow = 1, result = 0;
  for(int i=len-1; i>=val_start; i--) {
    result += (int)(s[i]-48) * pow;
    pow*=10;
  }
  if(val_start == 1) { // a negative value
    result = -result;
  }
  return result;
}

/* ------------------------------------------------------------------------------------------*/

bool isIntString(char* s)  {
  if(*s == '\0') { 
    return false; // an empty string
  }
  if(*s == '-') {  // first char may be a minus sign
    ++s;
    if(*s == '\0') {  // check its not a solitary '-'
      return false;
    }
  }
  while(*s != '\0') { // do until we hit termination char 
    if(*s < 48 || *s >57) {
      return false;
    }
    ++s;
  }
  return true;
}

/* ------------------------------------------------------------------------------------------*/

bool fileExists(char* F) {
  bool exists = true;
  ifstream In;
  In.open(F);
  if(!In)
    exists = false;
  In.close();
  return exists;
}

/* ------------------------------------------------------------------------------------------*/

void fatal(bool condition, char* location, char* message) {
  
  if(condition) {
    cerr << endl;
    cerr << "Exiting due to Fatal Error" << endl
    	 << "Location: " << location << endl 
      	 << "Message:  " << message << endl;
    exit(-1);
  }
}

/* ------------------------------------------------------------------------------------------*/
/* ------------------------------------------------------------------------------------------*/
