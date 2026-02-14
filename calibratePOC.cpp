/* 
This program is a POC that will find the difference in rotation and translation between
two images in order to create a 3D map of a shape
*/

#define F 2.5
#define Cx 0
#define Cy 0

int main(){

    // Inverted Translation Matrix
    float K[3][3] = 
        {{1/F, 0, -Cx/F},
         {0, 1/F, -Cy/F},
         {0, 0, 1  }};

    // Vanilla 2D X Coordinates
    int U[2][8] = 
        {{213, 525, 216, 520, 102, 371, 381, 302}, 
        {138, 485, 143, 483, 251, 539, 291, 353}};

    // Vanilla 2D Y Coordinates
     int V[2][8] = 
        {{129, 120, 407, 370, 112, 102, 251, 118}, 
        {106, 116, 369, 391, 74, 81, 240, 94}};

    // Convert to Camera Coordinates

    float Camera[2][8][3];

    for (int i = 0; i < 2; i++){
        for (int j = 0; j < 8; j++){
            Camera[i][j][0] = (U[i][j] - Cx) / F;
            Camera[i][j][1] = (V[i][j] - Cx) / F;
            Camera[i][j][2] = 1;
        }
    }

    return 0;
}