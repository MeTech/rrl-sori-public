#include "softnessRendering.h"
#define M_PI	3.14159265358979323846

double softnessRenderingContactAreaModel (int userID, int materialID, double force) {
    // Finger parameters
    double R = 9, E_f0 = 0.013, beta=0.9, E_f; // [mm, N/mm^2, -]
    // int userID = 1;
    // Material paramete3rs
    double E_material[9] = {0.1, 0.17, 0.45, 0.17, 1.3, 0.056, 0.078, 0.027, 0.019};     // EF30, EF50, DS10, EF50-DS30, DS30-EF50,

    double E_total, R_total, h_total, contactAreaMaterial, contactAreaFlat;

    // Finger calibration parameters
    if(userID == 0){ R=9; E_f = 0.013; beta = 0.9; }
    else if(userID == 1){R=7.75; E_f = 0.021; beta = 1; }

    // Contact area calculation
    E_f = E_f0 * (1 + beta * force);
    E_total = 1/(1/E_material[materialID] + 1/E_f);
    R_total = R*(1+E_material[materialID]/E_f);
    h_total = pow((3*force/(4*sqrt(R_total)*E_material[materialID])),2.0/3); 

    contactAreaMaterial = M_PI * pow((0.75*R*force*(1/E_total)),2.0/3);
    
    // Side area calculation
    contactAreaFlat = M_PI * pow((0.75*R*force*(1/1.32 + 1/E_f)),2.0/3); // finger on a flat dragon skin 30 surface
    double a = sqrt(R_total*h_total); // finger contact radius
    double eps = 0.4;
    if (a > R-eps) {a = R-eps;} // limit the contact radius. Cannot be bigger than finger radius - epsilone

    // Toriod SPA parameters
    double r_TSPA = 11; // [mm] toroid SPA radius
    double b = (r_TSPA-a)/2; // [mm] ellipse radius 
    //double d = sqrt(R*R - a*a); // [mm] center of the finger to the surface

    double desiredContactAreaSides = contactAreaMaterial - contactAreaFlat;
    if(desiredContactAreaSides<0) {desiredContactAreaSides=0;}

    // calcualte desired torus h
    double h_c_torus = desiredContactAreaSides/(2*M_PI*R);
    double M = h_c_torus - sqrt(R*R - a*a);
    double x0 = sqrt(R*R - M*M) - a - b;// [mm] intersection point in x coordinate
    double h_desired = 0; // [mm] desired torus height

    if(pow(x0/b,2)<0.99){
        h_desired = h_c_torus/sqrt(1 - pow(x0/b,2));
    } 
    else{
        h_desired = h_c_torus;
    }
    
    // Calculate the pressure inpput for the torus
    double c1=200, b1=0.13; // fitted for mm's torus height experiment
    double K = c1*(pow((1-b1*pow(force,1.0/3)),2)/pow((1+b1*pow(force,1.0/3)),2));
    double desiredPressure=0; // [MPa]
    double maxPressure=0.024, minPressure=0;
    if(K>0){
        desiredPressure = h_desired/K;
    }
    else{
        desiredPressure = maxPressure;
    }
        // limit the desired pressure
    if(desiredPressure>maxPressure)
        desiredPressure = maxPressure;
    else if(desiredPressure<minPressure)
        desiredPressure = minPressure;

    return desiredPressure*1000; // [kPa]
}

double softnessRenderingStiffnessModel (int userID, int materialID, double displacement){
    // Finger parameters
    double R = 9, E_f0 = 0.013, beta=0.9, E_f; // [mm, N/mm^2, -]
    // Charecterization is done using 9mm spherical indenter
    double R_indenter=9; // [mm]
    double F_indenter; // [N]
    // EF30, EF50, DS10, EF50-DS30, DS30-EF50, salmon, beef, marshmellow, bread
    // Material parameters
    if (materialID == 0){ // EF00-30
        F_indenter = 0.3333*pow(displacement,2) + 0.1045*displacement - 0.0067;
    }
    else if (materialID == 1){ // EF00-50
        F_indenter = 0.4572*pow(displacement,2) + 0.0853*displacement + 0.0063;
    }
    else if (materialID == 2){ // DS10
        F_indenter = 1.0414*pow(displacement,2) + 0.1293*displacement + 0.033;
    }
    else if (materialID == 3){ // EF50-DS30
        F_indenter = 1.384*pow(displacement,2) - 1.0212*displacement + 0.2776;
    }
    else if (materialID == 4){ // DS30-EF50
        F_indenter = 1.384*pow(displacement,2) - 1.0212*displacement + 0.2776;
    }
    else if (materialID == 5){ //  salmon
        F_indenter = 0.0286 * pow(displacement,4) - 0.1527*pow(displacement,3) + 0.3123*pow(displacement,2) - 0.1614*displacement - 0.0125;
    }
    else if (materialID == 6){ // beef
        F_indenter = 0.0338 * pow(displacement,4) - 0.1899*pow(displacement,3) + 0.4415*pow(displacement,2) - 0.2762*displacement + 0.0245;
    }
    else if (materialID == 7){ // marshmellow
        F_indenter = 0.0054*pow(displacement,4) - 0.0366*pow(displacement,3) + 0.1272*pow(displacement,2) - 0.0554*displacement + 0.0075;
    }
    else if (materialID == 8){ // bread
        F_indenter = 0.0054* pow(displacement,4) - 0.0304*pow(displacement,3) + 0.0788*pow(displacement,2) - 0.0196*displacement + 0.0027;
    }

    // Finger calibration parameters
    if(userID == 0){ R=9; E_f = 0.013; beta = 0.9; }
    else if(userID == 1){R=7.75; E_f = 0.021; beta = 1; } 
    
    double F_finger = F_indenter * sqrt(R_indenter/R);
    if(displacement<0)
        F_finger=0;
    return F_finger;
}