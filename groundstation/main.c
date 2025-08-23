#include "pointdish.h"

void test_pointdish() {
    // { echo "38.53"; echo "121.76"; curl -s 'https://tle.ivanstanojevic.me/api/tle/25544' | jq '.line1, .line2' | tr -d '"'; curl -s "https://ssd-api.jpl.nasa.gov/jd_cal.api?cd=$(date +%F_%T)" | jq -r '.jd'; } | ./pointdish
 
    char lines[4][100]; //pipe to here 

    double az;
    double el_dish;

    FILE *fp = fopen( "PS_data.txt","r");
    if(fp == NULL){
        perror("Error opening file");
        return;
    }

    for (int i=0; i<=4; i++){
        fgets(lines[i], sizeof(lines[i]),fp);  
        printf("%s", lines[i]);    
    }
    
    fclose(fp);

    double lat = strtod(lines[0], NULL); 
    double lon = strtod(lines[1], NULL); 
    char* tle1 = lines[2];
    char* tle2 = lines[3];  
    double UTC = strtod(lines[4], NULL);
    double el_gs = strtod(lines[5], NULL);


    vec3 satellite_position;
    if (satellite_pos(tle1, tle2, UTC, 0.0, &satellite_position) != POS_LOOKUP_SUCCESS) {
        return -4;
    }
    else  {printf("made it\n");}


    point_dish(lat, lon, el_gs, satellite_position, &az, &el_dish);
    printf("Azimuth = %lf, Elevation = %lf\n", az, el_dish); 
}

int main() {
    printf("Starting server\n");
    
    while (true) {
        server();
    }

    return 0;
}

// Code is written with linux in mind. Compile accordingly
// Our Pi isn't going to be running windows.
void server()
{
    
}
