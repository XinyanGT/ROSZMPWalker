/**
 * @file generateTraj.cpp
 */

#include "zmpUtilities.h"
#include <stdio.h>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  /*  std::vector<Footprint> res = walkLine(20,
					0.089,
					0.15,
					Footprint(0.0, 0.089, 0.0, true));  
  */
  std::vector<Footprint> footprints = walkCircle( 10.0, 
						  2.0, 
						  0.089,
						  0.15,
						  0.5,
						  Footprint( 0, 0.089, 0, true ) );

  FILE* fpraw;
  fpraw = fopen("footprint_raw_test.txt", "w");
  for( int i = 0; i < footprints.size(); ++i ) {
    printf("Footprint[%d] pos: %f %f %f \n", i, footprints[i].x(), footprints[i].y(), footprints[i].theta() );
    fprintf( fpraw, "%d, %f %f %f \n", i, footprints[i].x(), footprints[i].y(), footprints[i].theta() );
  }
  fclose( fpraw );


 
}
