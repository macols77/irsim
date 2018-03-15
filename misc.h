#ifndef MISC_H
#define MISC_H

/******************************************************************************/
/******************************************************************************/
/*

Here are some "utility functions"

/******************************************************************************/
/******************************************************************************/

#include <iostream>
#include <fstream>

using namespace std;

void RecordInFile(float x, float y) {
	ofstream outfile ("/home/juan/Desktop/plotresult.txt", ios::app);
	outfile<<x<<" "<<y<<endl;
	outfile.close();
}




#endif
